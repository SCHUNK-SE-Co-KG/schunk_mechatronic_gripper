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
    MoveToAbsolutePositionGPE,
    MoveToRelativePosition,
    MoveToRelativePositionGPE,
    Grip,
    GripWithGPE,
    GripAtPosition,
    GripAtPositionWithGPE,
    GripWithVelocity,
    GripWithVelocityAndGPE,
    GripAtPositionWithVelocity,
    GripAtPositionWithVelocityAndGPE,
    Release,
    ReleaseWithGPE,
    StartJogging,
    StartJoggingGPE,
    ShowConfiguration,
    ShowGripperSpecification,
    ScanGrippers,
    LocateGripper,
    ReadGripperParameter,
    WriteGripperParameter,
    Stop,
    StopWithGPE,
)
from schunk_gripper_interfaces.msg import (  # type: ignore [attr-defined]
    Gripper as GripperConfig,
    GripperState,
    ConnectionState,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from threading import Lock, Thread, Event, Timer as Countdown
from rclpy.service import Service
from rclpy.publisher import Publisher
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from functools import partial
from schunk_gripper_library.utility import Scheduler, EthernetScanner
from typing import TypedDict
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult
from pathlib import Path
import tempfile
import json
from collections import OrderedDict
import time
from typing import Any
import re


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
        self.ethernet_scanner: EthernetScanner = EthernetScanner()

        # Initialization parameters
        self.init_parameters = {
            "host": "",
            "port": 80,
            "serial_port": "",
            "device_id": 12,
            "headless": False,
        }
        for name, default_value in self.init_parameters.items():
            self.declare_parameter(name, default_value)
        gripper: Gripper
        if self.get_parameter("host").value:
            gripper = {
                "host": self.get_parameter("host").value,
                "port": self.get_parameter("port").value,
                "serial_port": "",
                "device_id": 0,
                "driver": GripperDriver(),
                "gripper_id": "",
            }
            self.grippers.append(gripper)
        elif self.get_parameter("serial_port").value:
            gripper = {
                "host": "",
                "port": 0,
                "serial_port": self.get_parameter("serial_port").value,
                "device_id": self.get_parameter("device_id").value,
                "driver": GripperDriver(),
                "gripper_id": "",
            }
            self.grippers.append(gripper)

        self.headless = self.get_parameter("headless").value
        if self.headless:
            self.reset_grippers()
            self.load_previous_configuration()

        # Clean up initialization parameters
        for name in self.init_parameters.keys():
            self.undeclare_parameter(name)

        # Node parameters
        self.declare_parameter("log_level", "INFO")

        self.scheduler: Scheduler = Scheduler()
        self.scheduler.start()
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
        self.scan_grippers_srv = self.create_service(
            ScanGrippers,
            "~/scan",
            self._scan_grippers_cb,
        )
        self.locate_gripper_srv = self.create_service(
            LocateGripper,
            "~/locate_gripper",
            self._locate_gripper_cb,
        )
        self.add_on_set_parameters_callback(self._param_cb)

        # For concurrently running publishers
        self.publishers_cb_group: MutuallyExclusiveCallbackGroup = (
            MutuallyExclusiveCallbackGroup()
        )

        # For running gripper services in parallel
        self.gripper_services_cb_group: ReentrantCallbackGroup = (
            ReentrantCallbackGroup()
        )

        # Joint states
        self.joint_states_stop: Event = Event()
        self.joint_states_period: float = 0.05  # sec
        self.joint_states_thread: Thread = Thread()

        # Gripper states
        self.gripper_states_stop: Event = Event()
        self.gripper_states_period: float = 0.05  # sec
        self.gripper_states_thread: Thread = Thread()

        # Connection state
        self.connection_state_publisher: Publisher = self.create_publisher(
            msg_type=ConnectionState,
            topic="~/connection_state",
            qos_profile=1,
            callback_group=self.publishers_cb_group,
        )
        self.connection_status_stop: Event = Event()
        self.connection_status_period: float = 0.05  # sec
        self.connection_status_thread: Thread = Thread(
            target=self._publish_connection_state
        )
        self.connection_status_thread.start()

        if self.headless:
            self.get_logger().debug(
                f"Headless mode: {self.headless}. Auto configuring.."
            )
            Countdown(0.5, function=self.trigger_configure).start()

    def list_grippers(self) -> list[str]:
        devices = []
        for gripper in self.grippers:
            id = gripper["gripper_id"]
            if id:
                devices.append(id)
        return devices

    def get_unique_id(self, gripper: str) -> str:
        known_ids = [entry["gripper_id"] for entry in self.grippers]

        def increment(name: str) -> str:
            if name.split("_")[-1].isdigit():
                count = int(name.split("_")[-1]) + 1
                name = "_".join(name.split("_")[:-1])
                name += f"_{count}"
            else:
                name = f"{name}_1"
            return name

        unique_id = increment(gripper)
        while unique_id in known_ids:
            unique_id = increment(unique_id)
        return unique_id

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
                entry["gripper_id"] = gripper["gripper_id"]
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
        self,
        gripper_id: str = "",
        host: str = "",
        port: int = 0,
        serial_port: str = "",
        device_id: int = 0,
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

        driver = GripperDriver()
        if not gripper_id:
            scheduler = self.scheduler if serial_port else None
            if not driver.connect(
                host=host,
                port=port,
                serial_port=serial_port,
                device_id=device_id,
                update_cycle=None,
                scheduler=scheduler,
            ):
                return False
            gripper_id = self.get_unique_id(driver.gripper_type)
            driver.disconnect()

        self.grippers.append(
            {
                "host": host,
                "port": port,
                "serial_port": serial_port,
                "device_id": device_id,
                "driver": driver,
                "gripper_id": gripper_id,
            }
        )
        return True

    def reset_grippers(self) -> bool:
        self.grippers.clear()
        return True

    def needs_synchronize(self, gripper: Gripper) -> bool:
        serial_ports = [
            gripper["serial_port"]
            for gripper in self.grippers
            if gripper["serial_port"]
        ]
        if serial_ports.count(gripper["serial_port"]) > 1:
            return True
        return False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_configure() is called.")
        if not self.grippers:
            return TransitionCallbackReturn.FAILURE

        # Try to connect each gripper
        for idx, gripper in enumerate(self.grippers):
            driver = GripperDriver()
            scheduler = self.scheduler if gripper["serial_port"] else None
            driver.connect(
                host=gripper["host"],
                port=gripper["port"],
                serial_port=gripper["serial_port"],
                device_id=gripper["device_id"],
                update_cycle=None,
                scheduler=scheduler,
            )
            self.grippers[idx]["driver"] = driver

        # Update empty gripper IDs
        for idx, gripper in enumerate(self.grippers):
            if not gripper["gripper_id"]:
                self.grippers[idx]["gripper_id"] = self.get_unique_id(
                    gripper=gripper["driver"].gripper_type
                )

        # Start cyclic updates with reconnect mechanism for each gripper
        for idx, _ in enumerate(self.grippers):
            gripper = self.grippers[idx]
            scheduler = self.scheduler if self.needs_synchronize(gripper) else None
            gripper["driver"].start_module_updates(scheduler=scheduler)

        # Start info services
        self.list_grippers_srv = self.create_service(
            ListGrippers, "~/list_grippers", self._list_grippers_cb
        )

        # Deactivate setup services
        self.destroy_service(self.add_gripper_srv)
        self.destroy_service(self.reset_grippers_srv)
        self.destroy_service(self.show_configuration_srv)
        self.destroy_service(self.load_previous_configuration_srv)
        self.destroy_service(self.scan_grippers_srv)
        self.destroy_service(self.locate_gripper_srv)

        if self.headless:
            self.get_logger().debug(
                f"Headless mode: {self.headless}. Auto activating.."
            )
            self.headless = False
            Countdown(0.5, function=self.trigger_activate).start()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_activate() is called.")

        # Get available grippers ready to go
        for idx, gripper in enumerate(self.grippers):
            if self.needs_synchronize(gripper):
                self.grippers[idx]["driver"].acknowledge(scheduler=self.scheduler)
            else:
                self.grippers[idx]["driver"].acknowledge()

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

            # Set module type if gripper is not connected.
            # This is important to determine if the gripper has a brake.
            # If not set, service types will be wrong.
            if not gripper["driver"].connected:
                gripper_properties = gripper_id.split("_")
                # Check if the gripper ID is in the expected format
                if len(gripper_properties) >= 5:
                    gripper["driver"].module_type = "_".join(
                        [
                            gripper_properties[0],
                            gripper_properties[1],
                            gripper_properties[3],
                            gripper_properties[4],
                        ]
                    )

            if gripper["driver"].gpe_available():
                service_types = [MoveToAbsolutePositionGPE, MoveToRelativePositionGPE]
            else:
                service_types = [MoveToAbsolutePosition, MoveToRelativePosition]
            service_names = ["move_to_absolute_position", "move_to_relative_position"]
            is_absolute_flags = [True, False]
            for srv_name, srv_type, is_absolute in zip(
                service_names, service_types, is_absolute_flags
            ):
                self.gripper_services.append(
                    self.create_service(
                        srv_type,
                        f"~/{gripper_id}/{srv_name}",
                        partial(
                            self._move_to_position_cb,
                            gripper=gripper,
                            is_absolute=is_absolute,
                        ),
                        callback_group=self.gripper_services_cb_group,
                    )
                )
            service_type_map = {
                "EGK": {
                    False: {
                        "grip": GripWithVelocity,
                        "grip_at_position": GripAtPositionWithVelocity,
                    },
                    True: {
                        "grip": GripWithVelocityAndGPE,
                        "grip_at_position": GripAtPositionWithVelocityAndGPE,
                    },
                },
                "EGU": {
                    False: {
                        "grip": Grip,
                        "grip_at_position": GripAtPosition,
                    },
                    True: {
                        "grip": GripWithGPE,
                        "grip_at_position": GripAtPositionWithGPE,
                    },
                },
                "EZU": {
                    False: {
                        "grip": Grip,
                        "grip_at_position": GripAtPosition,
                    },
                    True: {
                        "grip": GripWithGPE,
                        "grip_at_position": GripAtPositionWithGPE,
                    },
                },
            }
            variant = gripper["driver"].get_variant()
            use_gpe = gripper["driver"].gpe_available()
            service_map = service_type_map.get(variant, {}).get(use_gpe, {})
            for srv_name, srv_type in service_map.items():
                self.gripper_services.append(
                    self.create_service(
                        srv_type,
                        f"~/{gripper_id}/{srv_name}",
                        partial(self._grip_cb, gripper=gripper),
                        callback_group=self.gripper_services_cb_group,
                    )
                )
            self.gripper_services.append(
                self.create_service(
                    srv_type=(
                        ReleaseWithGPE if gripper["driver"].gpe_available() else Release
                    ),
                    srv_name=f"~/{gripper_id}/release",
                    callback=partial(self._release_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    srv_type=Trigger,
                    srv_name=f"~/{gripper_id}/release_for_manual_movement",
                    callback=partial(
                        self._release_for_manual_movement_cb, gripper=gripper
                    ),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    srv_type=(
                        StartJoggingGPE
                        if gripper["driver"].gpe_available()
                        else StartJogging
                    ),
                    srv_name=f"~/{gripper_id}/start_jogging",
                    callback=partial(self._start_jogging_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    srv_type=Trigger,
                    srv_name=f"~/{gripper_id}/stop_jogging",
                    callback=partial(self._stop_jogging_cb, gripper=gripper),
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

            self.gripper_services.append(
                self.create_service(
                    Trigger,
                    f"~/{gripper_id}/brake_test",
                    partial(self._brake_test_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    ReadGripperParameter,
                    f"~/{gripper_id}/read_parameter",
                    partial(self._read_gripper_parameter_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )
            self.gripper_services.append(
                self.create_service(
                    WriteGripperParameter,
                    f"~/{gripper_id}/write_parameter",
                    partial(self._write_gripper_parameter_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )

            self.gripper_services.append(
                self.create_service(
                    srv_type=(
                        StopWithGPE if gripper["driver"].gpe_available() else Stop
                    ),
                    srv_name=f"~/{gripper_id}/stop",
                    callback=partial(self._stop_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )

            self.gripper_services.append(
                self.create_service(
                    srv_type=Trigger,
                    srv_name=f"~/{gripper_id}/prepare_for_shutdown",
                    callback=partial(self._prepare_for_shutdown_cb, gripper=gripper),
                    callback_group=self.gripper_services_cb_group,
                )
            )

            self.gripper_services.append(
                self.create_service(
                    srv_type=Trigger,
                    srv_name=f"~/{gripper_id}/soft_reset",
                    callback=partial(self._soft_reset_cb, gripper=gripper),
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

        self.joint_states_stop.clear()
        self.joint_states_thread = Thread(target=self._publish_joint_states)
        self.joint_states_thread.start()
        self.gripper_states_stop.clear()
        self.gripper_states_thread = Thread(target=self._publish_gripper_states)
        self.gripper_states_thread.start()

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_deactivate() is called.")

        # Stop publishing
        self.joint_states_stop.set()
        self.joint_states_thread.join()
        self.gripper_states_stop.set()
        self.gripper_states_thread.join()

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
        self.load_previous_configuration_srv = self.create_service(
            Trigger,
            "~/load_previous_configuration",
            self._load_previous_configuration_cb,
        )
        self.scan_grippers_srv = self.create_service(
            ScanGrippers,
            "~/scan",
            self._scan_grippers_cb,
        )
        self.locate_gripper_srv = self.create_service(
            LocateGripper,
            "~/locate_gripper",
            self._locate_gripper_cb,
        )

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().debug("on_shutdown() is called.")
        self.scheduler.stop()
        self.connection_status_stop.set()
        self.connection_status_thread.join()

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
        next_time = time.perf_counter()

        while rclpy.ok() and not self.joint_states_stop.is_set():
            now = time.perf_counter()

            if now >= next_time:
                for gripper in self.grippers:
                    msg = JointState()
                    gripper_id = gripper["gripper_id"]
                    msg.header.frame_id = gripper_id
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name.append(gripper_id)
                    msg.position.append(gripper["driver"].get_actual_position() / 1e6)

                    with self.joint_state_lock:
                        if gripper_id in self.joint_state_publishers:
                            try:
                                self.joint_state_publishers[gripper_id].publish(msg)

                            # Catch InvalidHandle exceptions on SIGINT (ctrl-c)
                            except RuntimeError:
                                break

                next_time += self.joint_states_period
            else:
                time.sleep(max(0, next_time - now))

    def _publish_gripper_states(self) -> None:
        next_time = time.perf_counter()

        while rclpy.ok() and not self.joint_states_stop.is_set():
            now = time.perf_counter()

            if now >= next_time:
                for gripper in self.grippers:
                    msg = GripperState()
                    gripper_id = gripper["gripper_id"]
                    msg.header.frame_id = gripper_id
                    msg.header.stamp = self.get_clock().now().to_msg()

                    status = gripper["driver"].get_status_diagnostics().split(",")
                    msg.error_code = status[0].strip()
                    msg.warning_code = status[1].strip()
                    msg.additional_code = status[2].strip()

                    msg.bit0_ready_for_operation = bool(
                        gripper["driver"].get_status_bit(bit=0)
                    )
                    msg.bit1_control_authority_fieldbus = bool(
                        gripper["driver"].get_status_bit(bit=1)
                    )
                    msg.bit2_ready_for_shutdown = bool(
                        gripper["driver"].get_status_bit(bit=2)
                    )
                    msg.bit3_not_feasible = bool(
                        gripper["driver"].get_status_bit(bit=3)
                    )
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
                    msg.bit12_workpiece_gripped = bool(
                        gripper["driver"].get_status_bit(bit=12)
                    )
                    msg.bit13_position_reached = bool(
                        gripper["driver"].get_status_bit(bit=13)
                    )
                    msg.bit14_workpiece_pre_grip_started = bool(
                        gripper["driver"].get_status_bit(bit=14)
                    )
                    msg.bit16_workpiece_lost = bool(
                        gripper["driver"].get_status_bit(bit=16)
                    )
                    msg.bit17_wrong_workpiece_gripped = bool(
                        gripper["driver"].get_status_bit(bit=17)
                    )
                    msg.bit31_grip_force_and_position_maintenance_activated = bool(
                        gripper["driver"].get_status_bit(bit=31)
                    )

                    with self.gripper_state_lock:
                        if gripper_id in self.gripper_state_publishers:
                            try:
                                self.gripper_state_publishers[gripper_id].publish(msg)

                            # Catch InvalidHandle exceptions on SIGINT (ctrl-c)
                            except RuntimeError:
                                break

                next_time += self.joint_states_period
            else:
                time.sleep(max(0, next_time - now))

    def _publish_connection_state(self) -> None:
        msg = ConnectionState()
        msg.header.frame_id = self.get_name()
        next_time = time.perf_counter()

        while rclpy.ok() and not self.connection_status_stop.is_set():
            now = time.perf_counter()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.grippers.clear()
            msg.connected.clear()

            for gripper in self.grippers:
                id = gripper["gripper_id"]
                driver = gripper["driver"]
                if id:
                    msg.grippers.append(id)
                    msg.connected.append(driver.connected)

            if now >= next_time:
                try:
                    self.connection_state_publisher.publish(msg)

                # Catch "publisher's context is invalid" on node destruction
                except RuntimeError:
                    break
                next_time += self.connection_status_period
            else:
                time.sleep(max(0, next_time - now))

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

    def _scan_grippers_cb(
        self, request: ScanGrippers.Request, response: ScanGrippers.Response
    ):
        with self.ethernet_scanner:
            entries = self.ethernet_scanner.scan()

        for entry in entries:
            host = entry["host"]
            port = entry["port"]
            driver = GripperDriver()
            if driver.connect(host=host, port=port):
                cfg = GripperConfig()
                cfg.host = host
                cfg.port = port
                response.connections.append(cfg)
                response.grippers.append(driver.gripper_type)
                driver.disconnect()

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

    def _locate_gripper_cb(
        self, request: LocateGripper.Request, response: LocateGripper.Response
    ):
        self.get_logger().debug("---> Locate gripper")
        driver = GripperDriver()
        scheduler = self.scheduler if request.gripper.serial_port else None
        driver.connect(
            host=request.gripper.host,
            port=request.gripper.port,
            serial_port=request.gripper.serial_port,
            device_id=request.gripper.device_id,
            update_cycle=None,
            scheduler=scheduler,
        )
        driver.acknowledge(scheduler=scheduler)
        response.success = driver.twitch_jaws(scheduler=scheduler)
        driver.disconnect()
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

    def _stop_cb(
        self,
        request: Any,
        response: Any,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Stop")
        use_gpe = getattr(request, "use_gpe", False)
        scheduler = self.scheduler if self.needs_synchronize(gripper) else None
        response.success = gripper["driver"].stop(
            scheduler=scheduler,
            use_gpe=use_gpe,
        )
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

    def _prepare_for_shutdown_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Prepare for shutdown")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].prepare_for_shutdown(
                scheduler=self.scheduler
            )
        else:
            response.success = gripper["driver"].prepare_for_shutdown()
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _soft_reset_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Soft reset")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].soft_reset(scheduler=self.scheduler)
        else:
            response.success = gripper["driver"].soft_reset()
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _move_to_position_cb(
        self,
        request: Any,
        response: Any,
        gripper: Gripper,
        is_absolute: bool,
    ):
        self.get_logger().debug("---> Move to position")
        position = int(request.position * 1e6)
        velocity = int(request.velocity * 1e6)
        use_gpe = getattr(request, "use_gpe", False)
        scheduler = self.scheduler if self.needs_synchronize(gripper) else None

        response.success = gripper["driver"].move_to_position(
            position=position,
            is_absolute=is_absolute,
            velocity=velocity,
            use_gpe=use_gpe,
            scheduler=scheduler,
        )
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _grip_cb(
        self,
        request: Any,
        response: Any,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Grip")
        use_gpe = getattr(request, "use_gpe", False)
        scheduler = self.scheduler if self.needs_synchronize(gripper) else None
        position = getattr(request, "position", None)
        velocity = getattr(request, "velocity", None)
        if position is not None:
            position = int(position * 1e6)
        if velocity is not None:
            velocity = int(velocity * 1e6)

        grip_result = gripper["driver"].grip(
            position=position,
            velocity=velocity,
            force=request.force,
            use_gpe=use_gpe,
            outward=request.outward,
            scheduler=scheduler,
        )

        response.success = False
        if grip_result == gripper["driver"].GripResult.WORKPIECE_GRIPPED:
            response.workpiece_gripped = True
            response.success = True
        elif grip_result == gripper["driver"].GripResult.WRONG_WORKPIECE_GRIPPED:
            response.wrong_workpiece_gripped = True
            response.success = True
        elif grip_result == gripper["driver"].GripResult.NO_WORKPIECE_DETECTED:
            response.no_workpiece_detected = True
            response.success = True
        elif grip_result == gripper["driver"].GripResult.WORKPIECE_LOST:
            response.workpiece_lost = True
            response.success = True

        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _release_cb(
        self,
        request: Release.Request | ReleaseWithGPE.Request,
        response: Release.Response | ReleaseWithGPE.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Release")
        use_gpe = getattr(request, "use_gpe", False)
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].release(
                use_gpe=use_gpe,
                scheduler=self.scheduler,
            )
        else:
            response.success = gripper["driver"].release(
                use_gpe=use_gpe,
            )
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _release_for_manual_movement_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Release for manual movement")
        if self.needs_synchronize(gripper):
            response.success = gripper["driver"].release_for_manual_movement(
                scheduler=self.scheduler
            )
        else:
            response.success = gripper["driver"].release_for_manual_movement()
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _start_jogging_cb(
        self,
        request: StartJogging.Request | StartJoggingGPE.Request,
        response: StartJogging.Response | StartJoggingGPE.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Start jogging")
        response.success = gripper["driver"].start_jogging(
            velocity=int(request.velocity * 1e6),
            use_gpe=getattr(request, "use_gpe", False),
            scheduler=self.scheduler if self.needs_synchronize(gripper) else None,
        )

        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _stop_jogging_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Stop jogging")
        response.success = gripper["driver"].stop_jogging(
            scheduler=self.scheduler if self.needs_synchronize(gripper) else None,
        )

        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _brake_test_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Brake test")
        response.success = gripper["driver"].brake_test(
            scheduler=self.scheduler if self.needs_synchronize(gripper) else None,
        )
        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _read_gripper_parameter_cb(
        self,
        request: ReadGripperParameter.Request,
        response: ReadGripperParameter.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Read gripper parameter")

        if self.needs_synchronize(gripper):
            data = self.scheduler.execute(
                func=partial(
                    gripper["driver"].read_module_parameter, param=request.parameter
                )
            ).result()
        else:
            data = gripper["driver"].read_module_parameter(request.parameter)

        values, value_type = gripper["driver"].decode_module_parameter(
            data=data, param=request.parameter
        )

        # Find the corresponding message field for this type
        # and assign the values to it if existent.
        msg_field = f"value_{re.split(r'[^a-z0-9]', value_type)[0]}"
        if getattr(response, msg_field, None) is not None:
            setattr(response, msg_field, values)
            response.success = True
        else:
            response.success = False

        response.message = gripper["driver"].get_status_diagnostics()
        return response

    def _write_gripper_parameter_cb(
        self,
        request: WriteGripperParameter.Request,
        response: WriteGripperParameter.Response,
        gripper: Gripper,
    ):
        self.get_logger().debug("---> Write gripper parameter")

        # Find the first non-empty array
        data: list[Any] = []
        for elem in dir(request):
            if elem.startswith("value_"):
                data = getattr(request, elem, [])
                if data:
                    break

        bytes_data = gripper["driver"].encode_module_parameter(
            data=data, param=request.parameter
        )

        if self.needs_synchronize(gripper):
            response.success = self.scheduler.execute(
                func=partial(
                    gripper["driver"].write_module_parameter,
                    param=request.parameter,
                    data=bytes_data,
                )
            ).result()
        else:
            response.success = gripper["driver"].write_module_parameter(
                param=request.parameter, data=bytes_data
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
