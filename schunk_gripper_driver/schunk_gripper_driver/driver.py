#!/usr/bin/env python3
# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

import rclpy

from rclpy.lifecycle import Node, State, TransitionCallbackReturn
import rclpy.logging
from schunk_gripper_library.driver import Driver as GripperDriver
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ListGrippers,
    AddGripper,
    MoveToAbsolutePosition,
    Grip,
    Release,
    ShowConfiguration,
    ShowGripperSpecification,
)
from schunk_gripper_interfaces.msg import (  # type: ignore [attr-defined]
    Gripper as GripperConfig,
    GripperState,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from threading import Lock, Timer as Countdown
from rclpy.service import Service
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from functools import partial
from schunk_gripper_library.utility import Scheduler
from typing import TypedDict
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from pathlib import Path
import tempfile
import json
from collections import OrderedDict


class Gripper(TypedDict):
    host: str
    port: int
    serial_port: str
    device_id: int
    driver: GripperDriver
    gripper_id: str


class Driver(Node):

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)
        self.grippers: list[Gripper] = []

        # Initialization parameters
        self.init_parameters = {
            "host": "",
            "port": 80,
            "serial_port": "/dev/ttyUSB0",
            "device_id": 12,
            "start_empty": False,
        }
        for name, default_value in self.init_parameters.items():
            self.declare_parameter(name, default_value)

        start_empty = self.get_parameter("start_empty").value
        if not start_empty:
            gripper: Gripper = {
                "host": self.get_parameter("host").value,
                "port": self.get_parameter("port").value,
                "serial_port": self.get_parameter("serial_port").value,
                "device_id": self.get_parameter("device_id").value,
                "driver": GripperDriver(),
                "gripper_id": "",
            }
            self.grippers.append(gripper)

        # Clean up initialization parameters
        for name in self.init_parameters.keys():
            self.undeclare_parameter(name)

        # Node parameters
        self.declare_parameter("log_level", "INFO")

        self.scheduler: Scheduler = Scheduler()
        self.gripper_services: list[Service] = []
        self.joint_state_publishers: dict[str, Publisher] = {}
        self.gripper_state_publishers: dict[str, Publisher] = {}
        self.joint_state_lock: Lock = Lock()
        self.gripper_state_lock: Lock = Lock()

        # Setup services
        self.add_gripper_srv = self.create_service(
            AddGripper, "~/add_gripper", self._add_gripper_cb
        )
        self.reset_grippers_srv = self.create_service(
            Trigger, "~/reset_grippers", self._reset_grippers_cb
        )
        self.show_configuration_srv = self.create_service(
            ShowConfiguration, "~/show_configuration", self._show_configuration_cb
        )
        self.save_configuration_srv = self.create_service(
            Trigger, "~/save_configuration", self._save_configuration_cb
        )
        self.load_previous_configuration_srv = self.create_service(
            Trigger,
            "~/load_previous_configuration",
            self._load_previous_configuration_cb,
        )
        self.add_on_set_parameters_callback(self._param_cb)

        # For concurrently running publishers
        self.timers_cb_group: MutuallyExclusiveCallbackGroup = (
            MutuallyExclusiveCallbackGroup()
        )
        self.publishers_cb_group: MutuallyExclusiveCallbackGroup = (
            MutuallyExclusiveCallbackGroup()
        )

        # For running gripper services in parallel
        self.gripper_services_cb_group: ReentrantCallbackGroup = (
            ReentrantCallbackGroup()
        )

        # Timers
        self.joint_states_timer: Timer = self.create_timer(
            timer_period_sec=0.05,
            callback=partial(self._publish_joint_states),
            callback_group=self.timers_cb_group,
        )
        self.gripper_states_timer: Timer = self.create_timer(
            timer_period_sec=0.05,
            callback=partial(self._publish_gripper_states),
            callback_group=self.timers_cb_group,
        )

    def list_grippers(self) -> list[str]:
        devices = []
        for gripper in self.grippers:
            id = gripper["gripper_id"]
            if id:
                devices.append(id)
        return devices

    def show_configuration(self) -> list[GripperConfig]:
        configuration = []
        for gripper in self.grippers:
            cfg = GripperConfig()
            cfg.host = gripper["host"]
            cfg.port = gripper["port"]
            cfg.serial_port = gripper["serial_port"]
            cfg.device_id = gripper["device_id"]
            configuration.append(cfg)
        return configuration

    def save_configuration(self, location: str = "/var/tmp/schunk_gripper") -> bool:
        LOG_NS = "Save configuration:"

        if not self.show_configuration():
            self.get_logger().debug(f"{LOG_NS} Configuration empty")
            return False

        path = Path(location)
        try:
            path.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryFile(dir=path):
                pass
        except Exception:
            self.get_logger().debug(f"{LOG_NS} Can't write to {path}")
            return False

        with open(path.joinpath("configuration.json"), "w") as f:
            configuration = []
            for gripper in self.grippers:
                entry: OrderedDict[str, str | int] = OrderedDict()
                entry["host"] = gripper["host"]
                entry["port"] = gripper["port"]
                entry["serial_port"] = gripper["serial_port"]
                entry["device_id"] = gripper["device_id"]
                configuration.append(entry)
            json.dump(configuration, f, indent=2)
        return True

    def load_previous_configuration(
        self, location: str = "/var/tmp/schunk_gripper"
    ) -> bool:
        LOG_NS = "Load configuration:"

        config_file = Path(location).joinpath("configuration.json")
        if not config_file.exists():
            self.get_logger().debug(f"{LOG_NS} Missing file: {config_file}")
            return False
        try:
            with open(config_file, "r") as f:
                try:
                    content = json.load(f)
                except json.JSONDecodeError:
                    self.get_logger().debug(f"{LOG_NS} Wrong layout")
                    return False
        except Exception as e:
            self.get_logger().debug(f"{LOG_NS} {e}")
            return False

        backup = self.grippers.copy()
        self.reset_grippers()
        for gripper in content:
            try:
                if not self.add_gripper(**gripper):
                    self.grippers = backup
                    self.get_logger().debug(
                        f"{LOG_NS} Failed adding gripper: {gripper}"
                    )
                    return False
            except TypeError:
                self.grippers = backup
                self.get_logger().debug(f"{LOG_NS} Unknown parameter")
                return False
        return True

    def add_gripper(
        self, host: str = "", port: int = 0, serial_port: str = "", device_id: int = 0
    ) -> bool:
        LOG_NS = "Add gripper:"
        if not any([host, port, serial_port, device_id]):
            self.get_logger().debug(f"{LOG_NS} empty gripper")
            return False
        if (host and not port) or (port and not host):
            self.get_logger().debug(
                f"{LOG_NS} missing host or port."
                " You need to specify both or none of them"
            )
            return False
        if (serial_port and not device_id) or (device_id and not serial_port):
            self.get_logger().debug(
                f"{LOG_NS} missing serial_port or device_id."
                " You need to specify both or none of them"
            )
            return False
        for gripper in self.grippers:
            if host:
                if host == gripper["host"] and port == gripper["port"]:
                    self.get_logger().debug(f"{LOG_NS} host and port already used")
                    return False
            else:
                if (
                    serial_port == gripper["serial_port"]
                    and device_id == gripper["device_id"]
                ):
                    self.get_logger().debug(
                        f"{LOG_NS} serial_port and device_id already used"
                    )
                    return False
        self.grippers.append(
            {
                "host": host,
                "port": port,
                "serial_port": serial_port,
                "device_id": device_id,
                "driver": GripperDriver(),
                "gripper_id": "",
            }
        )
        return True

    def reset_grippers(self) -> bool:
        self.grippers.clear()
        return True

    def needs_synchronize(self, gripper: Gripper) -> bool:
        serial_ports = [gripper["serial_port"] for gripper in self.grippers]
        if serial_ports.count(gripper["serial_port"]) > 1:
            return True
        return False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")
        if not self.grippers:
            return TransitionCallbackReturn.FAILURE
        self.scheduler.start()

        # Connect each gripper
        for idx, gripper in enumerate(self.grippers):
            driver = GripperDriver()
            if self.needs_synchronize(gripper):
                update_cycle = None
            else:
                update_cycle = 0.05
            if not driver.connect(
                host=gripper["host"],
                port=gripper["port"],
                serial_port=gripper["serial_port"],
                device_id=gripper["device_id"],
                update_cycle=update_cycle,
            ):
                self.get_logger().warn(f"Gripper connect failed: {gripper}")
                return TransitionCallbackReturn.FAILURE
            else:
                self.grippers[idx]["driver"] = driver

        # Set unique gripper IDs
        devices = []
        for idx, gripper in enumerate(self.grippers):
            id = f"{gripper['driver'].gripper}_1"
            while id in devices:
                count = int(id.split("_")[-1]) + 1
                id = id[:-2] + f"_{count}"
            devices.append(id)
            self.grippers[idx]["gripper_id"] = id

        # Start cyclic updates for each gripper
        for idx, _ in enumerate(self.grippers):
            gripper = self.grippers[idx]
            if self.needs_synchronize(gripper):
                self.scheduler.cyclic_execute(
                    func=partial(gripper["driver"].receive_plc_input), cycle_time=0.05
                )

        # Start info services
        self.list_grippers_srv = self.create_service(
            ListGrippers, "~/list_grippers", self._list_grippers_cb
        )

        # Deactivate setup services
        self.destroy_service(self.add_gripper_srv)
        self.destroy_service(self.reset_grippers_srv)
        self.destroy_service(self.show_configuration_srv)
        self.destroy_service(self.save_configuration_srv)
        self.destroy_service(self.load_previous_configuration_srv)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")

        # Get every gripper ready to go
        for idx, gripper in enumerate(self.grippers):
            if self.needs_synchronize(gripper):
                success = self.grippers[idx]["driver"].acknowledge(
                    scheduler=self.scheduler
                )
            else:
                success = self.grippers[idx]["driver"].acknowledge()
            if not success:
                return TransitionCallbackReturn.FAILURE

        # Gripper-specific services
        for idx, _ in enumerate(self.grippers):
            gripper = self.grippers[idx]
            gripper_id = gripper["gripper_id"]

            self.gripper_services.append(
                self.create_service(
                    Trigger,
                    f"~/{gripper_id}/acknowledge",
                    partial(self._acknowledge_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    Trigger,
                    f"~/{gripper_id}/fast_stop",
                    partial(self._fast_stop_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    MoveToAbsolutePosition,
                    f"~/{gripper_id}/move_to_absolute_position",
                    partial(self._move_to_absolute_position_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    Grip,
                    f"~/{gripper_id}/grip",
                    partial(self._grip_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    Release,
                    f"~/{gripper_id}/release",
                    partial(self._release_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )

            self.gripper_services.append(
                self.create_service(
                    ShowGripperSpecification,
                    f"~/{gripper_id}/show_specification",
                    partial(self._show_gripper_specification_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )

        # Publishers for each gripper
        for idx, _ in enumerate(self.grippers):
            gripper = self.grippers[idx]
            gripper_id = gripper["gripper_id"]

            # Joint states
            self.joint_state_publishers[gripper_id] = self.create_publisher(
                msg_type=JointState,
                topic=f"~/{gripper_id}/joint_states",
                qos_profile=1,
                callback_group=self.publishers_cb_group,
            )

            # Gripper state
            self.gripper_state_publishers[gripper_id] = self.create_publisher(
                msg_type=GripperState,
                topic=f"~/{gripper_id}/gripper_state",
                qos_profile=1,
                callback_group=self.publishers_cb_group,
            )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")

        # Release gripper-specific services
        for idx, _ in enumerate(self.gripper_services):
            self.destroy_service(self.gripper_services[idx])
        self.gripper_services.clear()

        # Remove gripper-specific publishers
        for gripper in self.list_grippers():
            with self.joint_state_lock:
                self.destroy_publisher(self.joint_state_publishers.pop(gripper))
            with self.gripper_state_lock:
                self.destroy_publisher(self.gripper_state_publishers.pop(gripper))

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_cleanup() is called.")
        self.scheduler.stop()
        for gripper in self.grippers:
            gripper["driver"].disconnect()
            gripper["driver"] = GripperDriver()

        # Release info services
        if not self.destroy_service(self.list_grippers_srv):
            return TransitionCallbackReturn.FAILURE

        # Reactivate setup services
        self.add_gripper_srv = self.create_service(
            AddGripper, "~/add_gripper", self._add_gripper_cb
        )
        self.reset_grippers_srv = self.create_service(
            Trigger, "~/reset_grippers", self._reset_grippers_cb
        )
        self.show_configuration_srv = self.create_service(
            ShowConfiguration, "~/show_configuration", self._show_configuration_cb
        )
        self.save_configuration_srv = self.create_service(
            Trigger, "~/save_configuration", self._save_configuration_cb
        )
        self.load_previous_configuration_srv = self.create_service(
            Trigger,
            "~/load_previous_configuration",
            self._load_previous_configuration_cb,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_shutdown() is called.")
        self.joint_states_timer.cancel()
        self.gripper_states_timer.cancel()

        if state is None:
            return TransitionCallbackReturn.SUCCESS

        Countdown(0.5, function=rclpy.try_shutdown).start()
        return TransitionCallbackReturn.SUCCESS

    def _param_cb(self, params):
        valid_log_levels = [
            "DEBUG",
            "INFO",
            "WARN",
            "ERROR",
            "FATAL",
        ]
        for p in params:
            if p.name == "log_level":
                if p.value not in valid_log_levels:
                    self.get_logger().error(
                        f"Invalid log level: {str(p.value)}. "
                        f"Valid options are: {valid_log_levels}"
                    )
                    return SetParametersResult(successful=False)
                level = rclpy.logging.get_logging_severity_from_string(p.value)
                self.get_logger().set_level(level)
                self.get_logger().debug(f"Log level changed to {p.value}")
                return SetParametersResult(successful=True)

        return SetParametersResult(successful=True)

    def _publish_joint_states(self) -> None:

        for gripper in self.grippers:
            msg = JointState()
            gripper_id = gripper["gripper_id"]
            msg.header.frame_id = gripper_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name.append(gripper_id)
            msg.position.append(gripper["driver"].get_actual_position() / 1e6)

            with self.joint_state_lock:
                if gripper_id in self.joint_state_publishers:
                    self.joint_state_publishers[gripper_id].publish(msg)

    def _publish_gripper_states(self) -> None:

        for gripper in self.grippers:
            msg = GripperState()
            gripper_id = gripper["gripper_id"]
            msg.header.frame_id = gripper_id
            msg.header.stamp = self.get_clock().now().to_msg()

            status = gripper["driver"].get_status_diagnostics().split(",")
            msg.error_code = status[0].strip()
            msg.warning_code = status[1].strip()
            msg.additional_code = status[2].strip()

            msg.bit0_ready_for_operation = bool(gripper["driver"].get_status_bit(bit=0))
            msg.bit1_control_authority_fieldbus = bool(
                gripper["driver"].get_status_bit(bit=1)
            )
            msg.bit2_ready_for_shutdown = bool(gripper["driver"].get_status_bit(bit=2))
            msg.bit3_not_feasible = bool(gripper["driver"].get_status_bit(bit=3))
            msg.bit4_command_successfully_processed = bool(
                gripper["driver"].get_status_bit(bit=4)
            )
            msg.bit5_command_received_toggle = bool(
                gripper["driver"].get_status_bit(bit=5)
            )
            msg.bit6_warning = bool(gripper["driver"].get_status_bit(bit=6))
            msg.bit7_error = bool(gripper["driver"].get_status_bit(bit=7))
            msg.bit8_released_for_manual_movement = bool(
                gripper["driver"].get_status_bit(bit=8)
            )
            msg.bit9_software_limit_reached = bool(
                gripper["driver"].get_status_bit(bit=9)
            )
            msg.bit11_no_workpiece_detected = bool(
                gripper["driver"].get_status_bit(bit=11)
            )
            msg.bit12_workpiece_gripped = bool(gripper["driver"].get_status_bit(bit=12))
            msg.bit13_position_reached = bool(gripper["driver"].get_status_bit(bit=13))
            msg.bit14_workpiece_pre_grip_started = bool(
                gripper["driver"].get_status_bit(bit=14)
            )
            msg.bit16_workpiece_lost = bool(gripper["driver"].get_status_bit(bit=16))
            msg.bit17_wrong_workpiece_gripped = bool(
                gripper["driver"].get_status_bit(bit=17)
            )
            msg.bit31_grip_force_and_position_maintenance_activated = bool(
                gripper["driver"].get_status_bit(bit=31)
            )

            with self.gripper_state_lock:
                if gripper_id in self.gripper_state_publishers:
                    self.gripper_state_publishers[gripper_id].publish(msg)

    # Service callbacks
    def _add_gripper_cb(
        self, request: AddGripper.Request, response: AddGripper.Response
    ):
        response.success = self.add_gripper(
            host=request.gripper.host,
            port=request.gripper.port,
            serial_port=request.gripper.serial_port,
            device_id=request.gripper.device_id,
        )
        return response

    def _reset_grippers_cb(self, request: Trigger.Request, response: Trigger.Response):
        response.success = self.reset_grippers()
        return response

    def _show_configuration_cb(
        self, request: ShowConfiguration.Request, response: ShowConfiguration.Response
    ):
        response.configuration = self.show_configuration()
        return response

    def _save_configuration_cb(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        response.success = self.save_configuration()
        return response

    def _load_previous_configuration_cb(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        response.success = self.load_previous_configuration()
        return response

    def _list_grippers_cb(
        self, request: ListGrippers.Request, response: ListGrippers.Response
    ):
        self.get_logger().debug("---> List gripper IDs")
        response.grippers = self.list_grippers()
        return response

    def _show_gripper_specification_cb(
        self,
        request: ShowGripperSpecification.Request,
        response: ShowGripperSpecification.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Show specification")
        spec = gripper["driver"].show_specification()
        if not spec:
            response.success = False
            response.message = gripper["driver"].get_status_diagnostics()
            return response

        response.specification.max_stroke = spec["max_stroke"]
        response.specification.max_speed = spec["max_speed"]
        response.specification.max_force = spec["max_force"]
        response.specification.serial_number = spec["serial_number"]
        response.specification.firmware_version = spec["firmware_version"]
        response.specification.device_id = spec["device_id"]
        response.specification.ip_address = spec["ip_address"]
        response.success = True
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _acknowledge_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Acknowledge")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].acknowledge(scheduler=self.scheduler)
        else:
            response.success = gripper["driver"].acknowledge()
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _fast_stop_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Fast stop")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].fast_stop(scheduler=self.scheduler)
        else:
            response.success = gripper["driver"].fast_stop()
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _move_to_absolute_position_cb(
        self,
        request: MoveToAbsolutePosition.Request,
        response: MoveToAbsolutePosition.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Move to absolute position")
        position = int(request.position * 1e6)
        velocity = int(request.velocity * 1e6)

        options_dict = {kv.feature: kv.value for kv in request.options}
        use_gpe = options_dict.get("use_gpe", False)

        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].move_to_absolute_position(
                position=position,
                velocity=velocity,
                use_gpe=use_gpe,
                scheduler=self.scheduler,
            )
        else:
            response.success = gripper["driver"].move_to_absolute_position(
                position=position, velocity=velocity, use_gpe=use_gpe
            )
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _grip_cb(
        self,
        request: Grip.Request,
        response: Grip.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Grip")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].grip(
                force=request.force,
                use_gpe=request.use_gpe,
                outward=request.outward,
                scheduler=self.scheduler,
            )
        else:
            response.success = gripper["driver"].grip(
                force=request.force,
                use_gpe=request.use_gpe,
                outward=request.outward,
            )
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _release_cb(
        self,
        request: Release.Request,
        response: Release.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Release")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].release(
                use_gpe=request.use_gpe,
                scheduler=self.scheduler,
            )
        else:
            response.success = gripper["driver"].release(
                use_gpe=request.use_gpe,
            )
        response.message = gripper["driver"].get_status_diagnostics()
        return response


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    driver = Driver("driver")
    executor.add_node(driver)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        driver.destroy_node()


if __name__ == "__main__":
    main()
