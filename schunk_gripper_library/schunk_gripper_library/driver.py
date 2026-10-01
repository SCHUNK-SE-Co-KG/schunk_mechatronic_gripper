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

import struct
from threading import Lock, RLock
from pymodbus.client import ModbusSerialClient
from pymodbus.pdu import ModbusPDU
import re
from threading import Thread, Event
import time
from httpx import Client, ConnectError, ConnectTimeout, HTTPError
from importlib.resources import files
from typing import Union
import json
from .utility import supports_parity, global_scheduler
from functools import partial
from typing import Any, Type, cast
from enum import Enum, auto

# Letting each driver instance have its own non-exclusive modbus client instance does not work,
# because in rare occations the modbus clients seem to interfere with each other when reading parameters.
# Therefore, we create a single global exclusive modbus client, shared by all driver instances.
global_modbus_client_lock = Lock()
# key: serial_port, value: ModbusSerialClient instance (do not use this map directly, use get_global_modbus_client() instead)
_global_modbus_client_map : dict[str, ModbusSerialClient] = {}


def get_global_modbus_client(serial_port: str = "/dev/ttyUSB0", baudrate: int = 115200):
    with global_modbus_client_lock:
        global _global_modbus_client_map
        if _global_modbus_client_map.get(serial_port) is None:
            _global_modbus_client_map[serial_port] = ModbusSerialClient(
                port=serial_port,
                baudrate=baudrate,
                parity="E" if supports_parity(serial_port) else "N",
                stopbits=1,
                timeout=1.0,
                trace_connect=None,
                trace_packet=None,
                trace_pdu=None,
            )
        return _global_modbus_client_map[serial_port]


class Driver(object):
    class GripResult(Enum):
        WORKPIECE_GRIPPED = auto()
        NO_WORKPIECE_DETECTED = auto()
        WRONG_WORKPIECE_GRIPPED = auto()
        WORKPIECE_LOST = auto()
        ERROR = auto()

    def __init__(self) -> None:
        self.plc_input: str = "0x0040"
        self.plc_output: str = "0x0048"
        self.error_byte: int = 12
        self.warning_byte: int = 14
        self.additional_byte: int = 15
        self.gripper_type: str = ""
        self.module_type: str = ""  # e. g. "EGU_50_M_B", see module_types.json
        self.fieldbus: str = ""  # e. g. "EI", see fieldbus_types.json
        self.module_parameters: dict = {  # positions in um, velocities in um/s, forces in %
            "module_type": None,
            "fieldbus_type": None,
            "serial_no_txt": None,
            "sw_version_txt": None,
            "min_pos": None,  # [um]
            "max_pos": None,  # [um]
            "min_vel": None,  # [um/s]
            "max_vel": None,    # [um/s]
            "max_grp_vel": None,  # [um/s]
            "wp_release_delta": None,
            "max_phys_stroke": None,
            "max_grp_force": None,
        }
        # fmt: off
        self.valid_status_bits: list[int] = (
            list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
        )
        self.valid_control_bits: list[int] = (
            list(range(0, 10)) + [11, 12, 13, 14, 16, 30, 31]
        )
        # fmt:on
        self.reserved_status_bits: list[int] = [10, 15] + list(range(18, 31))
        self.reserved_control_bits: list[int] = [10] + list(range(17, 30))

        if __package__ is None:
            raise Exception("This module must be imported as part of a package, not run as a script.")

        valid_module_types = str(
            files(__package__).joinpath("config/module_types.json")
        )
        valid_fieldbus_types = str(
            files(__package__).joinpath("config/fieldbus_types.json")
        )
        readable_params = str(
            files(__package__).joinpath("config/readable_parameters.json")
        )
        writable_params = str(
            files(__package__).joinpath("config/writable_parameters.json")
        )
        with open(valid_module_types, "r") as f:
            self.valid_module_types: dict[str, str] = json.load(f)
        with open(valid_fieldbus_types, "r") as f:
            self.valid_fieldbus_types: dict[str, str] = json.load(f)
        with open(readable_params, "r") as f:
            self.readable_parameters: dict[str, dict[str, Union[int, str]]] = json.load(
                f
            )
        with open(writable_params, "r") as f:
            self.writable_parameters: dict[str, dict[str, Union[int, str]]] = json.load(
                f
            )

        self.plc_input_buffer: bytearray = bytearray(bytes.fromhex("00" * 16))
        self.plc_output_buffer: bytearray = bytearray(bytes.fromhex("00" * 16))
        self.input_buffer_lock: RLock = RLock()
        self.output_buffer_lock: Lock = Lock()
        self.command_lock: Lock = Lock()
        self.motion_lock: Lock = Lock()
        self.motion_cancelled: Event = Event()

        self.mb_device_id: int | None = None
        self.web_client: Client | None = None
        self.host: str = ""
        self.port: int = 80
        self.web_client_lock: Lock = Lock()
        self.connected: bool = False
        self.polling_thread: Thread = Thread()
        self.update_cycle: float = 0.05  # sec
        self.update_cycle_while_streaming: float = 0.5  # sec, this must be shorter than the timeout of #wait_for_status
        self.update_count: int = 0  # since last connect() call
        self.stop_request: Event = Event()
        self.reconnect_interval: float = 1.0  # sec
        self.addr_str: str = ""
        self.stream_target_position_toggle: bool = False
        self.streaming_active: bool = False
        self._normal_update_cycle: float = 0.05  # sec, restored by disable_stream()

    def connect(
        self,
        host: str = "",
        port: int = 80,
        serial_port: str = "/dev/ttyUSB0",
        device_id: int | None = None,
        update_cycle: float | None = 0.05,
        baudrate: int = 115200,
    ) -> bool:
        if (isinstance(update_cycle, float) or isinstance(update_cycle, int)) and update_cycle < 0.001:
            raise ValueError("update_cycle must be at least 0.001 seconds")
        if self.connected:
            return False
        self.update_count = 0

        if host:
            self.addr_str = f"{host}:{port}"
        else:
            self.addr_str = f"{serial_port} (ID {device_id})"

        # TCP/IP
        if host:
            if not isinstance(port, int):
                return False
            if isinstance(port, int) and port < 0:
                return False
            self.host = host
            self.port = port
            with self.web_client_lock:
                self.web_client = Client(timeout=1.0)
                try:
                    self.connected = self.web_client.get(
                        f"http://{host}:{port}/adi/data.json"
                    ).is_success
                except (ConnectError, ConnectTimeout):
                    self.connected = False
                except HTTPError as e:
                    print(f"{type(e)}: {e}")
                    self.connected = False

        # Modbus
        else:
            if not isinstance(serial_port, str):
                return False
            if not isinstance(device_id, int):
                return False
            if isinstance(device_id, int) and device_id < 0:
                return False
            self.mb_device_id = device_id
            self.mb_client = get_global_modbus_client(serial_port=serial_port, baudrate=baudrate)
            with global_modbus_client_lock:
                self.connected = self.mb_client.connect()

        if self.connected:
            updated = global_scheduler.execute(func=partial(self.update_module_parameters)).result()
            if not updated:
                return False
            if update_cycle:
                self.update_cycle = update_cycle
                self.start_module_updates()

        return self.connected

    def disconnect(self) -> bool:
        self.motion_cancelled.set()
        self.disable_stream()
        self.stop_module_updates()

        if len(_global_modbus_client_map) > 0:
            for port in _global_modbus_client_map:
                _global_modbus_client_map[port].close()

        if self.web_client:
            with self.web_client_lock:
                self.web_client = None

        self.connected = False
        self.clear_module_parameters()
        return True

    def start_module_updates(self) -> bool:
        if self.polling_thread.is_alive():
            return True
        self.polling_thread = Thread(target=self._module_update, daemon=True)
        self.polling_thread.start()
        return True

    def stop_module_updates(self) -> bool:
        self.stop_request.set()
        if self.polling_thread.is_alive():
            self.polling_thread.join()
        return True

    def acknowledge(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to acknowledge: Not connected.")

        def do_send() -> dict:
            return {"0": 1, "5": self._send_cmd({"2": True})}

        with self.command_lock:
            expected_status = global_scheduler.execute(func=partial(do_send)).result()
            return self.wait_for_status(bits=expected_status)

    def fast_stop(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to fast stop: Not connected.")

        def do_send() -> dict:
            return {"7": 1, "5": self._send_cmd({"0": False})}  # fast stop triggers on low signal

        with self.command_lock:
            expected_status = global_scheduler.execute(func=partial(do_send)).result()
            stopped = self.wait_for_status(bits=expected_status)

        if stopped:
            self._cancel_active_motion()
        return stopped

    def stop(self, use_gpe: bool = False) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to stop: Not connected.")

        def do_send() -> dict:
            return {"4": 1, "5": self._send_cmd({"1": True, "31": use_gpe and self.gpe_available()})}

        with self.command_lock:
            expected_status = global_scheduler.execute(func=partial(do_send)).result()
            stopped = self.wait_for_status(bits=expected_status)

        if stopped:
            self._cancel_active_motion()
        return stopped

    def prepare_for_shutdown(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to prepare for shutdown: Not connected.")
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                "Failed to prepare for shutdown: motion in progress."
            )

        try:
            def do_send() -> dict:
                self.clear_plc_output()
                if not self.send_plc_output():
                    raise RuntimeError("Failed to clear previous command")
                if not self.receive_plc_input():
                    raise RuntimeError("Failed to read command-toggle status")
                cmd_toggle_before = self.get_status_bit(bit=5)
                self.set_control_bit(bit=3, value=True)
                if not self.send_plc_output():
                    raise RuntimeError("Failed to send command")
                return {"5": cmd_toggle_before ^ 1, "2": 1}

            with self.command_lock:
                expected_status = global_scheduler.execute(func=partial(do_send)).result()
                return self.wait_for_status(bits=expected_status)
        finally:
            self.motion_lock.release()

    def move_to_position(
        self,
        position: int,
        velocity: int,
        is_absolute: bool = True,
        use_gpe: bool = False,
        no_scheduler: bool = False,
        _motion_lock_held: bool = False,
    ) -> bool:
        """Sends a move to position command to the gripper.

        This command blocks until the move is completed or an error occurs.

        Note:
            All integer parameters must be 32-bit signed integers,
            as expected by the gripper.
            If a value exceeds these bounds, an error is returned.

        Args:
            position (int): Target position in micrometers.
            velocity (int): Movement velocity in micrometers per second.
            is_absolute (bool): Whether the position is absolute (True)
                                or relative (False).
            use_gpe (bool): Whether to use GPE functionality.
            no_scheduler (bool): If True, the request is sent directly without any scheduler.

        Returns:
            bool: True if the move was successful, False otherwise.
        """
        if not self.connected:
            raise RuntimeError("Failed to move to position: Not connected.")
        motion_lock_acquired = False
        if not _motion_lock_held and not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                "Failed to move to position: Another motion command is already in progress."
            )
        if not _motion_lock_held:
            motion_lock_acquired = True
            self.motion_cancelled.clear()

        try:
            def do_send() -> dict:
                control_bits = {}
                control_bits["13" if is_absolute else "14"] = True
                control_bits["31"] = use_gpe if self.gpe_available() else False
                return {"3": 0, "5": self._send_cmd(control_bits, vel=velocity, pos=position)}

            with self.command_lock:
                expected_status = do_send() if no_scheduler else global_scheduler.execute(func=partial(do_send)).result()

                # wait for the command to be acknowledged
                if not self.wait_for_status(bits=expected_status):
                    return False

            # estimate how long the move will take
            epsilon_sec = 2  # additional time to account for delays (e. g. releasing brakes)
            estimated_duration_sec = self.estimate_duration(
                position_abs=position, is_absolute=is_absolute, velocity=velocity
            )
            duration_sec = estimated_duration_sec + epsilon_sec

            # wait for the command to complete or an error to occur
            bits: list[dict[str, int]] = []
            bits.append({"4": 1, "13": 1})  # command processed and position reached
            bits.append({"7": 1})  # error state
            matched_pattern = self.wait_for_any_status(
                bits=bits,
                timeout_sec=duration_sec,
                cancel_event=self.motion_cancelled,
            )

            # a move has failed if either an error occured or the wait timed out
            return matched_pattern not in [{}, {"7": 1}]
        finally:
            if motion_lock_acquired:
                self.motion_lock.release()

    def enable_stream(self) -> bool:
        """Initiates the target position stream (streaming mode).

        Note: Call this method before using `move_to_position_streamed` to ensure the stream is properly initialized.
        Returns:
            bool: True if the stream was initiated successfully, False otherwise.
        """
        if not self.connected:
            raise RuntimeError("Failed to initiate target position stream: Not connected.")
        if self.streaming_active:
            return True
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                "Failed to initiate target position stream: Another motion command is already in progress."
            )

        stream_enabled = False
        try:
            # when initializing the stream, we need to init pos, vel and force
            pos = self.get_actual_position()  # current position [um]
            # use max velocity
            vel_addr = "0x0630"
            max_vel_mms, _ = self.decode_module_parameter(self._read_param_now(vel_addr), vel_addr)
            vel = int(max_vel_mms[0] * 1000)  # [mm/s] -> [um/s]

            # max_allow_force (0x06A8) is not available for all variants (e. g. EGK),
            # so we use 90% of max_grp_force as a safe default that works for all grippers.
            force = 90  # [%]

            def do_send() -> dict:
                control_bits = {"15": True}
                return {"3": 0, "5": self._send_cmd(control_bits, pos=pos, vel=vel, force=force)}

            # send the command to initiate the target position stream
            expected_status = global_scheduler.execute(func=partial(do_send)).result()

            # wait for the received status and check if the stream was successfully initiated
            if not self.wait_for_status(bits=expected_status):
                return False

            self._normal_update_cycle = self.update_cycle
            self.update_cycle = self.update_cycle_while_streaming  # sec
            self.stream_target_position_toggle = True
            self.streaming_active = True
            stream_enabled = True
            return True
        finally:
            if not stream_enabled:
                self.update_cycle = self._normal_update_cycle
                self.stream_target_position_toggle = False
                self.streaming_active = False
                self.motion_lock.release()

    def disable_stream(self) -> None:
        """Ends the target position stream and restores the normal update rate."""
        if not self.streaming_active:
            return
        with self.command_lock:
            self.streaming_active = False
            self.stream_target_position_toggle = False
            self.update_cycle = self._normal_update_cycle
            self.motion_lock.release()

    def move_to_streamed_target(self, pos: int) -> bool:
        """Sets and sends a new target position without waiting for completion.

        Intended for streaming mode, where target positions arrive continuously
        and each one should be applied as soon as possible.

        Note: Call `enable_stream` before using this method to ensure the stream is properly initialized.

        Args:
            pos (int): Target position in micrometers.

        Returns:
            bool: True if the position was sent successfully, False otherwise.
        """
        if not self.connected:
            raise RuntimeError("Not connected.")
        with self.command_lock:
            if not self.streaming_active:
                raise RuntimeError("Streaming mode is not enabled.")

            self.stream_target_position_toggle = not self.stream_target_position_toggle

            def do_send() -> bool:
                # The assumption is that the position stream has already been initialized,
                # thus we just send the command without zero frames or acknowledgement.
                control_bits = {
                    "15": True,  # move-interpolate mode
                    "6": self.stream_target_position_toggle,  # repeat-command toggle bit
                }

                self.clear_plc_output()
                for bit_str, value in control_bits.items():
                    self.set_control_bit(bit=int(bit_str), value=value)

                self.set_target_position(pos)
                if not self.send_plc_output(operation="stream target"):
                    raise RuntimeError("Failed to send streamed target position")
                return True

            try:
                # to save bandwidth, we do not read back the gripper's status
                return global_scheduler.execute(func=partial(do_send)).result()
            except Exception:
                self.streaming_active = False
                self.stream_target_position_toggle = False
                self.update_cycle = self._normal_update_cycle
                self.motion_lock.release()
                raise

    def grip(
        self,
        force: int,
        position: int | None = None,
        velocity: int | None = None,
        use_gpe: bool = False,
        outward: bool = False,
    ) -> "Driver.GripResult":
        """Sends a grip command to the gripper.

        This command blocks until the grip is completed or an error occurs.

        Note:
            All integer parameters must be 32-bit signed integers,
            as expected by the gripper.
            If a value exceeds these bounds, an error is returned.

        Args:
            force (int): Gripping force in percentage (can exceed 100 for strong grips).
            position (int | None): Optional position parameter in micrometers.
                                    If None, the gripper will grip fully inwards
                                    or outwards based on the `outward` parameter.
            velocity (int | None): Optional gripping velocity in micrometers per second.
                                  If None, the gripper will use a velocity
                                  based on the specified force.
            use_gpe (bool): Whether to use GPE functionality if available.
            outward (bool): Whether to grip from inside (True) or from outside (False).

        Returns:
            Driver.GripResult: Result of the grip operation.
        """
        if not self.connected:
            raise RuntimeError("Failed to grip: Not connected.")
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError("Failed to grip: Another motion command is already in progress.")
        self.motion_cancelled.clear()

        try:
            def do_send() -> dict:
                control_bits = {}
                control_bits["16" if position is not None else "12"] = True
                control_bits["7"] = outward
                control_bits["31"] = use_gpe if self.gpe_available() else False
                return {"3": 0, "5": self._send_cmd(control_bits, vel=velocity, pos=position, force=force)}

            with self.command_lock:
                expected_status = global_scheduler.execute(func=partial(do_send)).result()

                # wait for the command to be acknowledged
                if not self.wait_for_status(bits=expected_status):
                    return Driver.GripResult.ERROR

            # estimate how long the grip will take
            epsilon_sec = 2  # additional time to account for delays (e. g. releasing brakes)
            estimated_duration_sec = self.estimate_duration(
                position_abs=position, velocity=velocity, force=force, outward=outward
            )
            # retrieve the prehold time in case the gripper is configured for pre-gripping
            prehold_time_sec = 0.0
            prehold_time_data = self._read_param_now("0x0380")
            values, value_type = self.decode_module_parameter(prehold_time_data, "0x0380")
            if value_type == "uint16" and len(values) == 1:
                prehold_time_sec = values[0] / 1000.0  # ms -> s

            duration_sec = estimated_duration_sec + prehold_time_sec + epsilon_sec

            # define the possible status bit patterns to wait for
            patterns = {}
            patterns[Driver.GripResult.WORKPIECE_GRIPPED] = {"4": 1, "12": 1, "31": use_gpe}
            patterns[Driver.GripResult.NO_WORKPIECE_DETECTED] = {"4": 0, "11": 1, "31": use_gpe}
            patterns[Driver.GripResult.WRONG_WORKPIECE_GRIPPED] = {"4": 0, "17": 1, "31": use_gpe}
            patterns[Driver.GripResult.WORKPIECE_LOST] = {"4": 0, "16": 1, "31": use_gpe}  # relevant for pre-gripping
            patterns[Driver.GripResult.ERROR] = {"7": 1}

            # wait for the command to complete or an error to occur
            bits: list[dict[str, int]] = []
            for pattern in patterns.values():
                bits.append(pattern)
            matched_pattern = self.wait_for_any_status(
                bits=bits,
                timeout_sec=duration_sec,
                cancel_event=self.motion_cancelled,
            )

            return next(
                (k for k, v in patterns.items() if v == matched_pattern),
                Driver.GripResult.ERROR,
            )
        finally:
            self.motion_lock.release()

    def release(
        self, use_gpe: bool = False
    ) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to release: Not connected.")
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError("Failed to release: Another motion command is already in progress.")
        self.motion_cancelled.clear()

        try:
            def do_send() -> dict:
                control_bits = {}
                control_bits["11"] = True
                control_bits["31"] = use_gpe if self.gpe_available() else False
                return {"3": 0, "5": self._send_cmd(control_bits)}

            with self.command_lock:
                expected_status = global_scheduler.execute(func=partial(do_send)).result()

                # wait for the command to be acknowledged
                if not self.wait_for_status(bits=expected_status):
                    return False

            # estimate how long the release will take
            epsilon_sec = 2  # additional time to account for delays (e. g. releasing brakes)
            estimated_duration_sec = self.estimate_duration(release=True)
            duration_sec = estimated_duration_sec + epsilon_sec

            # wait for the command to complete or an error to occur
            bits: list[dict[str, int]] = []
            bits.append({"4": 1, "13": 1})  # command processed and position reached
            bits.append({"7": 1})  # error state
            matched_pattern = self.wait_for_any_status(
                bits=bits,
                timeout_sec=duration_sec,
                cancel_event=self.motion_cancelled,
            )

            # a release has failed if either an error occured or the wait timed out
            return matched_pattern not in [{}, {"7": 1}]
        finally:
            self.motion_lock.release()

    def release_for_manual_movement(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to release for manual movement: Not connected.")
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                "Failed to release for manual movement: Another motion command is already in progress."
            )
        self.motion_cancelled.clear()

        try:
            def do_send() -> dict:
                self.clear_plc_output()
                if not self.send_plc_output():
                    raise RuntimeError("Failed to clear previous command")
                if not self.receive_plc_input():
                    raise RuntimeError("Failed to read command-toggle status")
                cmd_toggle_before = self.get_status_bit(bit=5)
                self.set_control_bit(bit=5, value=True)
                if not self.send_plc_output():
                    raise RuntimeError("Failed to send command")
                return {"5": cmd_toggle_before ^ 1, "8": 1}

            with self.command_lock:
                expected_status = global_scheduler.execute(func=partial(do_send)).result()
                return self.wait_for_status(bits=expected_status)
        finally:
            self.motion_lock.release()

    def show_specification(self) -> dict[str, float | str]:
        if not self.connected:
            raise RuntimeError("Failed to show specification: Not connected.")

        connection_info = {
            "ip_address": self.host,
            "device_id": self.mb_device_id or 0,
        }
        spec = {
            "max_stroke": self.module_parameters["max_phys_stroke"] / 1000,
            "max_speed": self.module_parameters["max_vel"] / 1000,
            "max_force": self.module_parameters["max_grp_force"] / 1000,
            "serial_number": self.module_parameters["serial_no_txt"],
            "firmware_version": self.module_parameters["sw_version_txt"],
            **connection_info,
        }

        # firmware version formatting is <major>.<minor>.<patch>.<build> => remove build
        if isinstance(spec["firmware_version"], str):
            tokens = spec["firmware_version"].split(".")
            if len(tokens) == 4:
                spec["firmware_version"] = ".".join(tokens[:3])

        return spec

    def brake_test(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to perform brake test: Not connected.")
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                "Failed to perform brake test: Another motion command is already in progress."
            )
        self.motion_cancelled.clear()

        try:
            def do_send() -> dict:
                control_bits = {}
                control_bits["30"] = True
                return {"4": 1, "5": self._send_cmd(control_bits)}

            with self.command_lock:
                expected_status = global_scheduler.execute(func=partial(do_send)).result()
                if not self.wait_for_status(bits={"5": expected_status["5"]}):
                    return False

            # the timeout value is empirically determined with real hardware
            return self.wait_for_status(
                bits=expected_status,
                timeout_sec=6.0,
                cancel_event=self.motion_cancelled,
            )
        finally:
            self.motion_lock.release()

    def estimate_duration(
        self,
        release: bool = False,
        position_abs: int | None = None,
        is_absolute: bool = True,
        velocity: int | None = None,
        force: int = 0,
        outward: bool = False,
    ) -> float:
        if release:
            return (
                self.module_parameters["wp_release_delta"]
                / self.module_parameters["max_vel"]
            )

        if isinstance(position_abs, int):
            if is_absolute:
                still_to_go = position_abs - self.get_actual_position()
            else:
                still_to_go = position_abs
            if isinstance(velocity, int) and velocity > 0:
                return abs(still_to_go) / velocity
            if isinstance(force, int) and force > 0:
                ratio = min(1.0, (force / 100))
                grip_vel = ratio * self.module_parameters["max_grp_vel"]
                return abs(still_to_go) / grip_vel
            return 0.0

        if isinstance(force, int) and force > 0:
            if outward:
                still_to_go = (
                    self.module_parameters["max_pos"] - self.get_actual_position()
                )
            else:
                still_to_go = (
                    self.module_parameters["min_pos"] - self.get_actual_position()
                )
            if isinstance(velocity, int) and velocity > 0:
                return abs(still_to_go) / velocity
            ratio = min(1.0, force / 100)
            return abs(still_to_go) / (ratio * self.module_parameters["max_grp_vel"])
        return 0.0

    def start_jogging(
        self, velocity: int, use_gpe: bool = False
    ) -> bool:
        """Sends the start jogging command to the gripper.

        Note: this method does not release the motion lock; it must be released by calling stop_jogging.

        Args:
            velocity -- The speed at which to jog in micrometers per second.
                        Positive values jog outwards, negative values jog inwards.
                        This value must fit into a 32-bit signed integer, otherwise
                        False is returned.
            use_gpe -- Whether to use GPE functionality.
            scheduler -- Optional scheduler for command execution.

        Returns:
            bool: True if the command was successful, False otherwise.
        """
        if not self.connected:
            raise RuntimeError("Failed to start jogging: Not connected.")

        def do_send() -> dict:
            cmd = {}
            cmd["8" if velocity < 0 else "9"] = True
            cmd["31"] = use_gpe and self.gpe_available()

            return {"5": self._send_cmd(cmd, vel=abs(velocity)), "6": 0, "7": 0}

        with self.command_lock:
            expected_status = global_scheduler.execute(func=partial(do_send)).result()
            started = self.wait_for_status(bits=expected_status)

        return started

    def stop_jogging(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to stop jogging: Not connected.")

        def do_send() -> dict:
            # Preserve the GPE bit and clear only the active jogging bits.
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=8, value=False)
            self.set_control_bit(bit=9, value=False)
            if not self.send_plc_output():
                raise RuntimeError("Failed to send command")
            return {"5": cmd_toggle_before ^ 1, "6": 0, "7": 0}

        with self.command_lock:
            expected_status = global_scheduler.execute(func=partial(do_send)).result()
            stopped = self.wait_for_status(bits=expected_status)

        return stopped

    def twitch_jaws(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to twitch jaws: Not connected.")
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                "Failed to twitch jaws: Another motion command is already in progress."
            )
        self.motion_cancelled.clear()

        def move(step: int) -> bool:
            return self.move_to_position(
                position=step,
                velocity=self.module_parameters["max_vel"],
                is_absolute=True,
                _motion_lock_held=True,
            )

        try:
            if not global_scheduler.execute(func=partial(self.receive_plc_input)).result():
                return False
            step = 2000  # um
            min_pos = self.module_parameters["min_pos"]
            max_pos = self.module_parameters["max_pos"]
            actual_pos = self.get_actual_position()
            actual_pos = max(min_pos, min(actual_pos, max_pos))  # clamp to valid range
            start_inwards = actual_pos - min_pos > max_pos - actual_pos
            if start_inwards:
                step *= -1
            for _ in range(2):
                if not move(max(min_pos, min(actual_pos + step, max_pos))):
                    return False
                if not move(actual_pos):
                    return False
            return True
        finally:
            self.motion_lock.release()

    def soft_reset(self) -> bool:
        if not self.connected:
            raise RuntimeError("Failed to soft reset: Not connected.")
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                "Failed to soft reset: motion in progress."
            )

        try:
            def do_send() -> dict:
                self.clear_plc_output()
                if not self.send_plc_output():
                    raise RuntimeError("Failed to clear previous command")
                if not self.receive_plc_input():
                    raise RuntimeError("Failed to read command-toggle status")
                cmd_toggle_before = self.get_status_bit(bit=5)
                self.set_control_bit(bit=4, value=True)
                if not self.send_plc_output():
                    raise RuntimeError("Failed to send command")
                return {"5": cmd_toggle_before ^ 1}

            with self.command_lock:
                expected_status = global_scheduler.execute(func=partial(do_send)).result()
                return self.wait_for_status(bits=expected_status)
        finally:
            self.motion_lock.release()

    def receive_plc_input(self) -> bool:
        with self.input_buffer_lock:
            data = self._read_param_now(self.plc_input)
            if data:
                self.plc_input_buffer = data
                return True
            return False

    def send_plc_output(self, operation: str = "heartbeat status") -> bool:
        with self.output_buffer_lock:
            buffer_cpy = bytearray(self.plc_output_buffer)
        try:
            return self._write_param_now(self.plc_output, buffer_cpy)
        except Exception as error:
            print(f"Modbus write failed during {operation}: {error}")
            raise

    def gpe_available(self) -> bool:
        if not self.module_type:
            return False
        keys = self.module_type.split("_")
        if len(keys) < 3:
            return False
        if keys[2] == "M":
            return True
        return False

    def get_variant(self) -> str:
        """Returns the variant of the connected device.

        If no device is connected or the module type is invalid, an empty string is returned.
        Return: str: The variant of the device ("EGU", "EGK", "EZU") or an empty string.
        """
        if not self.module_type:
            return ""
        if self.module_type not in self.valid_module_types.values():
            return ""
        if self.module_type.startswith("EGU"):
            return "EGU"
        elif self.module_type.startswith("EGK"):
            return "EGK"
        elif self.module_type.startswith("EZU"):
            return "EZU"
        return ""

    def get_sub_variant(self) -> int:
        """Returns the sub-variant of the connected device.

        If no device is connected or the module type is invalid or unknown, an exception is raised.
        Return: The subvariant of the module as an integer number, e. g. an EGU50 returns 50.
        """
        if not self.module_type:
            raise RuntimeError("No module connected")
        if self.module_type not in self.valid_module_types.values():
            raise RuntimeError("Invalid module type")
        # extract the number from the module type string
        parts = self.module_type.split("_")
        if not parts:
            raise RuntimeError("Invalid module type")
        digits = "".join(c for part in parts for c in part if c.isdigit())
        if digits:
            return int(digits)
        raise RuntimeError("Unknown sub-variant")

    def update_module_parameters(self) -> bool:
        if not (fieldbus_param := self._read_param_now("0x1130")):
            return False

        self.fieldbus = self.valid_fieldbus_types.get(
            str(struct.unpack("h", fieldbus_param)[0]), ""
        )

        value: int | str
        for param, fields in self.readable_parameters.items():
            if fields["name"] in self.module_parameters:
                field_type = str(fields["type"])
                if not (data := self._read_param_now(param)):
                    return False

                if field_type == "float":
                    if self.fieldbus == "PN":
                        value = int(struct.unpack("f", data[::-1])[0] * 1e3)  # [mm] -> [um]
                    else:
                        value = int(struct.unpack("f", data)[0] * 1e3)  # [mm] -> [um]

                elif field_type == "enum":
                    value = int(struct.unpack("h", data)[0])

                elif field_type.startswith("char"):
                    start = field_type.find("[")
                    end = field_type.find("]")
                    if start != -1 and end != -1:
                        length = int(field_type[start + 1 : end])
                        value = data[:length].decode("ascii").strip("\x00")
                    else:
                        return False
                else:
                    return False
                self.module_parameters[fields["name"]] = value

        if any([entry is None for entry in self.module_parameters.values()]):
            return False

        self.module_type = self.valid_module_types.get(
            str(self.module_parameters["module_type"]), ""
        )
        self.gripper_type = self.compose_gripper_type(
            module_type=self.module_type, fieldbus=self.fieldbus
        )
        if not self.gripper_type:
            return False

        return True

    def clear_module_parameters(self) -> bool:
        for key in self.module_parameters.keys():
            self.module_parameters[key] = None
        self.fieldbus = ""
        self.module_type = ""
        self.gripper_type = ""
        return True

    def read_param(self, param: str, read_raw: bool = False, length: int = 0) -> bytearray:
        """Reads the specified parameter from the module.

        Note: This is the client-side interface for reading module parameters.
        For internal use, see `_read_param_now()` to avoid deadlocks within the scheduler.

        Args:
            param (str): The parameter address in hex format, e.g. "0x0040".
            read_raw (bool): If True, reads the raw parameter value without any decoding.
            length (int): The number of registers to read.

        Returns:
            bytearray: The value of the specified parameter.
                       Use `decode_module_parameter()` to convert the
                       bytearray into the correct type.

        Raises:
            RuntimeError: If the parameter is not readable.
        """
        def do_read() -> bytearray:
            return self._read_param_now(param, length if read_raw else 0)

        return global_scheduler.execute(func=partial(do_read)).result()

    def _read_param_now(self, param: str, length: int = 0) -> bytearray:
        """Reads the specified parameter from the module immediately, bypassing the scheduler.

        Note: This is an internal method and should not be called client-side.
        Use `read_param()` instead for client-side access.

        Args:
            param (str): The parameter address in hex format, e.g. "0x0040".
            length (int): The number of registers to read.

        Returns:
            bytearray: The value of the specified parameter.
                       Use `decode_module_parameter()` to convert the
                       bytearray into the correct type.

        Raises:
            RuntimeError: If the parameter is not readable.
        """
        result = bytearray()
        if param not in self.readable_parameters and length == 0:
            raise RuntimeError(f"Failed to read module parameter '{param}': Parameter is not readable.")

        if not self.web_client:
            # read from modbus
            with global_modbus_client_lock:
                try:
                    if self.mb_device_id is None:
                        raise RuntimeError("Failed to read module parameter: Modbus device ID is not set")
                    pdu = self.mb_client.read_holding_registers(
                        address=int(param, 16) - 1,
                        count=int(self.readable_parameters[param]["registers"]) if length == 0 else length,
                        slave=self.mb_device_id,
                        no_response_expected=False,
                    )
                except (Exception) as e:
                    print(f"{type(e)}: {e} (device id: {self.mb_device_id}, param: {param})")
                    return result

            # Parse each 2-byte register,
            # reverting pymodbus' internal big endian decoding.
            if not pdu.isError():
                for reg in pdu.registers:
                    result.extend(reg.to_bytes(2, byteorder="big"))

        if self.web_client:
            # read from http server
            params = {"inst": param, "count": "1"}
            with self.web_client_lock:
                try:
                    response = self.web_client.get(
                        f"http://{self.host}:{self.port}/adi/data.json", params=params
                    )
                except (Exception) as e:
                    print(f"{type(e)}: {e}")
                    return result
            if response.is_success:
                if response.json() == []:
                    return result
                result = bytearray(bytes.fromhex(response.json()[0]))

        if result:
            current_size = len(result)
            desired_size = int(self.readable_parameters[param]["registers"]) * 2 if length == 0 else length * 2
            if current_size < desired_size:
                result.extend([0] * (desired_size - current_size))  # zero-pad

        return result

    def write_param(self, param: str, data: bytearray, write_raw: bool = False, length: int = 0) -> bool:
        """Writes the given module parameter to the module.

        Note: This is the client-side interface for writing module parameters.
        For internal use, see `_write_param_now()` to avoid deadlocks within the scheduler.

        Args:
            param (str): The parameter address in hex format, e.g. "0x0040".
            data (bytearray): The data to write to the parameter.
            write_raw (bool): If True, writes the raw parameter value without any encoding.
            length (int): The number of registers to write.

        Returns:
            bool: True if the write was successful, False otherwise.

        Raises:
            RuntimeError: If not connected to the module or if the parameter is not writable.
        """
        def do_write() -> bool:
            return self._write_param_now(param, data, length if write_raw else 0)

        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError(
                f"Failed to write module parameter '{param}': motion in progress."
            )

        try:
            with self.command_lock:
                return global_scheduler.execute(func=partial(do_write)).result()
        finally:
            self.motion_lock.release()

    def _write_param_now(self, param: str, data: bytearray, length: int = 0) -> bool:
        """Writes the given module parameter to the module immediately, bypassing the scheduler.

        Note: This is an internal method and should not be called client-side.
        Use `write_param()` instead for client-side access.

        Args:
            param (str): The parameter address in hex format, e.g. "0x0040".
            data (bytearray): The data to write to the parameter.
            length (int): The number of registers to write.
        Returns:
            bool: True if the write was successful, False otherwise.

        Raises:
            RuntimeError: If not connected to the module or if the parameter is not writable.
        """
        if not self.connected:
            raise RuntimeError("Failed to write module parameter '{param}': Not connected.")

        if param not in self.writable_parameters and length == 0:
            raise RuntimeError(f"Failed to write module parameter '{param}': Parameter is not writable.")

        expected_size = self.writable_parameters[param]["registers"] * 2 if length == 0 else length * 2
        if len(data) != expected_size:
            return False

        if not self.web_client:
            # Write to modbus.
            # Turn the bytearray into a list of 2-byte registers.
            # Pymodbus uses big endian internally for their encoding.
            param_size = int(self.writable_parameters[param]["registers"]) * 2 if length == 0 else length * 2
            values = [
                int.from_bytes(data[i : i + 2], byteorder="big")
                for i in range(0, param_size, 2)
            ]
            with global_modbus_client_lock:
                if self.mb_device_id is None:
                    raise RuntimeError("Failed to write module parameter: Modbus device ID is not set")
                pdu = self.mb_client.write_registers(
                    address=int(param, 16) - 1,  # Modbus convention
                    values=values,
                    slave=self.mb_device_id,
                    no_response_expected=False,
                )
            return not pdu.isError()

        if self.web_client:
            # write to http server
            payload = {"inst": param, "value": data.hex().upper()}
            with self.web_client_lock:
                response = self.web_client.post(
                    url=f"http://{self.host}:{self.port}/adi/update.json", data=payload
                )
            return response.is_success

        return False

    def encode_module_parameter(self, data: list[Any], param: str) -> bytearray:
        result = bytearray()
        if not self.connected:
            raise RuntimeError("Failed to encode module parameter: Not connected.")
        if not data or param not in self.writable_parameters:
            raise RuntimeError(f"Failed to encode module parameter: Invalid data or parameter '{param}'.")

        type_str = str(self.writable_parameters[param]["type"])
        expected_size = int(self.writable_parameters[param]["registers"]) * 2
        endianness = ">" if self.fieldbus == "PN" else "<"
        encodings = {
            "bool": {"char": "?", "type": bool},
            "uint8": {"char": "B", "type": int},
            "uint16": {"char": "H", "type": int},
            "uint32": {"char": "I", "type": int},
            "float": {"char": "f", "type": float},
        }
        if type_str not in encodings:
            return result
        char = encodings[type_str]["char"]
        expected_type = cast(Type, encodings[type_str]["type"])

        for entry in data:
            if not isinstance(entry, expected_type):
                return result
            result.extend(struct.pack(f"{endianness}{char}", entry))

        while len(result) < expected_size:
            result.extend(bytes.fromhex("00"))

        return result

    def decode_module_parameter(
        self, data: bytearray, param: str
    ) -> tuple[tuple[Any, ...], str]:
        """
        Converts the given bytearray into the correct type based on the parameter.

        Args:
            data (bytearray): The bytearray data to decode.
            param (str): The parameter identifier to determine
                         the decoding (e.g., "0x3080").

        Returns:
            tuple[tuple[Any, ...], str]: A tuple containing the decoded values and
                                         a status or description string.
        """
        if not self.connected:
            raise RuntimeError("Failed to decode module parameter: Not connected.")
        if not data:
            raise RuntimeError("Failed to decode module parameter: No data provided.")
        if param not in self.readable_parameters:
            raise RuntimeError(f"Failed to decode module parameter: Unknown parameter '{param}'.")

        value_type = str(self.readable_parameters[param]["type"])

        if value_type == "enum":
            values = struct.unpack("h", data)

        elif value_type == "bool":
            values = struct.unpack("?", data[:1])

        elif value_type.startswith("char"):
            count = len(data)
            values = struct.unpack(f"{count}B", data)
            values = ("".join([chr(i) for i in values]).strip(),)

        elif value_type.startswith("uint8"):
            count = len(data)
            values = struct.unpack(f"{count}B", data)

        elif value_type.startswith("float"):
            count = len(data) // 4
            if self.fieldbus == "PN":
                values = struct.unpack(f"{count}f", data[::-1])
                values = values[::-1]
            else:
                values = struct.unpack(f"{count}f", data)

        elif value_type.startswith("uint16"):
            count = len(data) // 2
            if self.fieldbus == "PN":
                values = struct.unpack(f"{count}H", data[::-1])
                values = values[::-1]
            else:
                values = struct.unpack(f"{count}H", data)

        elif value_type.startswith("uint32"):
            count = len(data) // 4
            if self.fieldbus == "PN":
                values = struct.unpack(f"{count}I", data[::-1])
                values = values[::-1]
            else:
                values = struct.unpack(f"{count}I", data)

        else:
            raise RuntimeError(f"Failed to decode module parameter: Unsupported type '{value_type}'.")

        return (values, value_type)

    def wait_for_status(
        self,
        bits: dict[str, int],
        timeout_sec: float = 1.0,
        cancel_event: Event | None = None,
    ) -> bool:
        return bool(
            self.wait_for_any_status(
                bits=[bits],
                timeout_sec=timeout_sec,
                cancel_event=cancel_event,
            )
        )

    def wait_for_any_status(
        self,
        bits: list[dict[str, int]],
        timeout_sec: float,
        cancel_event: Event | None = None,
    ) -> dict[str, int]:
        """Wait for any of the specified status bits to reach the desired value.

        Keyword arguments:
        bits -- a list of patterns containing the bit numbers and their expected values
        timeout_sec -- the maximum time to wait for the status bits to change

        Return: Returns the first matching bit pattern found,
                or an empty dictionary if none matched within the timeout.
        """
        if not timeout_sec > 0.0:
            raise ValueError("Invalid timeout value (must be > 0.0 sec)")
        if not bits:
            raise ValueError("Invalid bits list (must not be empty)")

        deadline_time = time.time() + timeout_sec
        while time.time() < deadline_time:
            if cancel_event and cancel_event.is_set():
                return {}
            with self.input_buffer_lock:
                for bit_pattern in bits:
                    if all([self.get_status_bit(int(bit)) == value for bit, value in bit_pattern.items()]):
                        return bit_pattern
            if cancel_event:
                cancel_event.wait(0.05)
            else:
                time.sleep(0.05)
        return {}

    def error_in(self, duration_sec: float) -> bool:
        if not isinstance(duration_sec, float):
            return False
        if duration_sec < 0.0:
            return False
        duration = time.time() + duration_sec
        while time.time() < duration:
            if self.get_status_bit(bit=7) == 1:
                return True
            time.sleep(self.update_cycle)
        return False

    def contains_non_hex_chars(self, buffer: str) -> bool:
        return bool(re.search(r"[^0-9a-fA-F]", buffer))

    def compose_gripper_type(self, module_type: str, fieldbus: str) -> str:
        if (
            module_type not in self.valid_module_types.values()
            or fieldbus not in self.valid_fieldbus_types.values()
        ):
            return ""
        entries = module_type.split("_")
        gripper_type = "_".join(
            [entries[0], entries[1], fieldbus, entries[2], entries[3]]
        )
        return gripper_type

    def set_plc_input(self, buffer: str) -> bool:
        with self.input_buffer_lock:
            if len(buffer) != 32:
                return False
            if self.contains_non_hex_chars(buffer):
                return False
            self.plc_input_buffer = bytearray(bytes.fromhex(buffer))
            return True

    def get_plc_input(self) -> str:
        with self.input_buffer_lock:
            return self.plc_input_buffer.hex().upper()

    def set_plc_output(self, buffer: str) -> bool:
        with self.output_buffer_lock:
            if len(buffer) != 32:
                return False
            if self.contains_non_hex_chars(buffer):
                return False
            self.plc_output_buffer = bytearray(bytes.fromhex(buffer))
            return True

    def get_plc_output(self) -> str:
        with self.output_buffer_lock:
            return self.plc_output_buffer.hex().upper()

    def clear_plc_output(self) -> None:
        self.set_plc_output("00" * 16)
        self.set_control_bit(
            bit=0, value=True
        )  # deactivate fast stop (inverted behavior)

    def set_control_bit(self, bit: int, value: bool) -> bool:
        with self.output_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_control_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            if value:
                self.plc_output_buffer[byte_index] |= 1 << bit_index
            else:
                self.plc_output_buffer[byte_index] &= ~(1 << bit_index)
            return True

    def get_control_bit(self, bit: int) -> int | bool:
        with self.output_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_control_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            return (
                1 if self.plc_output_buffer[byte_index] & (1 << bit_index) != 0 else 0
            )

    def toggle_control_bit(self, bit: int) -> bool:
        with self.output_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_control_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            self.plc_output_buffer[byte_index] ^= 1 << bit_index
            return True

    def get_status_bit(self, bit: int) -> int:
        with self.input_buffer_lock:
            if bit < 0 or bit > 31:
                raise ValueError("Invalid bit number (must be between 0 and 31)")
            if bit in self.reserved_status_bits:
                raise ValueError("Cannot read reserved status bits")
            byte_index, bit_index = divmod(bit, 8)
            return 1 if self.plc_input_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def get_error_code(self) -> str:
        with self.input_buffer_lock:
            return (
                hex(self.plc_input_buffer[self.error_byte]).upper().replace("0X", "0x")
            )

    def get_warning_code(self) -> str:
        with self.input_buffer_lock:
            return (
                hex(self.plc_input_buffer[self.warning_byte])
                .upper()
                .replace("0X", "0x")
            )

    def get_additional_code(self) -> str:
        with self.input_buffer_lock:
            return (
                hex(self.plc_input_buffer[self.additional_byte])
                .upper()
                .replace("0X", "0x")
            )

    def get_status_diagnostics(self) -> str:
        diagnostics = (
            f"error_code: {self.get_error_code()}"
            + f", warning_code: {self.get_warning_code()}"
            + f", additional_code: {self.get_additional_code()}"
        )
        return diagnostics

    def set_target_position(self, target_pos: int) -> bool:
        if not isinstance(target_pos, int):
            raise ValueError("Target position must be an integer")

        with self.output_buffer_lock:
            # snap to limits if within epsilon to account for rounding errors
            eps = 10  # um
            min_pos_um = self.module_parameters.get("min_pos")  # um
            max_pos_um = self.module_parameters.get("max_pos")  # um
            if min_pos_um is not None and max_pos_um is not None:
                if target_pos < min_pos_um and target_pos + eps >= min_pos_um:
                    target_pos = min_pos_um
                elif target_pos > max_pos_um and target_pos - eps <= max_pos_um:
                    target_pos = max_pos_um

            data = bytes()
            try:
                data = bytes(struct.pack("i", target_pos))
            except struct.error:
                raise ValueError("Failed to pack target position")
            if self.fieldbus == "PN":
                data = data[::-1]
            self.plc_output_buffer[4:8] = data
            return True

    def get_target_position(self) -> int:  # um
        with self.output_buffer_lock:
            data = self.plc_output_buffer[4:8]
            if self.fieldbus == "PN":
                data = data[::-1]
            return struct.unpack("i", data)[0]

    def set_target_speed(self, target_speed: int) -> bool:
        """Sets the target speed for the gripper in um/s."""
        if not isinstance(target_speed, int):
            raise ValueError("Target speed must be an integer")
        if target_speed < 0:
            raise ValueError("Target speed must be non-negative")
        with self.output_buffer_lock:
            # snap to limits if within epsilon to account for rounding errors
            eps = 10  # um/s
            min_speed_um_s = self.module_parameters.get("min_vel")  # um/s
            max_speed_um_s = self.module_parameters.get("max_vel")  # um/s
            max_grp_speed_um_s = self.module_parameters.get("max_grp_vel")  # um/s
            if min_speed_um_s is not None and max_speed_um_s is not None:
                if target_speed < min_speed_um_s and target_speed + eps >= min_speed_um_s:
                    target_speed = min_speed_um_s
                elif target_speed > max_speed_um_s and target_speed - eps <= max_speed_um_s:
                    target_speed = max_speed_um_s
                elif max_grp_speed_um_s is not None:
                    if target_speed > max_grp_speed_um_s and target_speed - eps <= max_grp_speed_um_s:
                        target_speed = max_grp_speed_um_s

            data = bytes()
            try:
                data = bytes(struct.pack("i", target_speed))
            except struct.error:
                raise ValueError("Failed to pack target speed")
            if self.fieldbus == "PN":
                data = data[::-1]
            self.plc_output_buffer[8:12] = data
            return True

    def get_target_speed(self) -> float:
        with self.output_buffer_lock:
            data = self.plc_output_buffer[8:12]
            if self.fieldbus == "PN":
                data = data[::-1]
            return struct.unpack("i", data)[0]  # um/s

    def set_gripping_force(self, gripping_force: int) -> bool:
        with self.output_buffer_lock:
            if not isinstance(gripping_force, int):
                raise ValueError("Gripping force must be an integer")
            data = bytes()
            try:
                data = bytes(struct.pack("i", gripping_force))
            except struct.error:
                raise ValueError("Failed to pack gripping force")
            if self.fieldbus == "PN":
                data = data[::-1]
            self.plc_output_buffer[12:16] = data
            return True

    def get_gripping_force(self) -> int:
        with self.output_buffer_lock:
            data = self.plc_output_buffer[12:16]
            if self.fieldbus == "PN":
                data = data[::-1]
            return struct.unpack("i", data)[0]

    def get_actual_position(self) -> int:  # um
        with self.input_buffer_lock:
            data = self.plc_input_buffer[4:8]
            if self.fieldbus == "PN":
                data = data[::-1]
            return struct.unpack("i", data)[0]

    def _set_status_bit(self, bit: int, value: bool) -> bool:
        with self.input_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_status_bits:
                return False
            byte_index, bit_index = divmod(bit, 8)
            if value:
                self.plc_input_buffer[byte_index] |= 1 << bit_index
            else:
                self.plc_input_buffer[byte_index] &= ~(1 << bit_index)
            return True

    def _cancel_active_motion(self) -> None:
        if not self.motion_lock.locked():
            return
        self.motion_cancelled.set()

    def _module_update(self) -> None:
        self.stop_request.clear()
        fails = 0
        next_time = time.perf_counter()
        while not self.stop_request.is_set():
            runs_fine = global_scheduler.execute(func=partial(self.receive_plc_input)).result()
            if runs_fine:
                if self.connected:
                    self.update_count += 1
                    fails = 0
                else:
                    self.connected = global_scheduler.execute(func=partial(self.update_module_parameters)).result()

                time.sleep(max(0, next_time - time.perf_counter()))
                next_time += self.update_cycle
            else:
                self.connected = False
                fails += 1
                if fails < 3:
                    time.sleep(self.update_cycle)
                else:
                    time.sleep(self.reconnect_interval)

    def _send_cmd(self, control_bits: dict[str, bool], pos: int | None = None, vel: int | None = None, force: int | None = None) -> int:
        """Sends the given control bits to the device.

        Args:
            control_bits -- A dictionary mapping control bits (keys) to their desired values (values).
            pos -- Optional target position in micrometers.
            vel -- Optional target speed in micrometers per second.
            force -- Optional gripping force in percentage (can exceed 100% for strong grips).
        Returns:
            The expected command toggle bit after sending the command
        """
        # the gripper does not clear bits between messages,
        # so we need to explicitly clear them ourselves before sending a new command
        self.clear_plc_output()
        if not self.send_plc_output():
            raise RuntimeError("Failed to clear previous command")

        if not self.receive_plc_input():
            raise RuntimeError("Failed to read command-toggle status")

        cmd_toggle_before = self.get_status_bit(bit=5)

        for bit_str, value in control_bits.items():
            self.set_control_bit(bit=int(bit_str), value=value)

        if pos is not None:
            self.set_target_position(pos)
        if vel is not None:
            self.set_target_speed(vel)
        if force is not None:
            self.set_gripping_force(force)

        if not self.send_plc_output():
            raise RuntimeError("Failed to send command")
        return cmd_toggle_before ^ 1

    def _trace_packet(self, sending: bool, data: bytes) -> bytes:
        txt = "REQUEST stream" if sending else "RESPONSE stream"
        print(f"---> {txt}: {data!r}")
        return data

    def _trace_pdu(self, sending: bool, pdu: ModbusPDU) -> ModbusPDU:
        txt = "REQUEST pdu" if sending else "RESPONSE pdu"
        print(f"---> {txt}: {pdu}")
        return pdu

    def _trace_connect(self, connect: bool) -> None:
        txt = "Connected" if connect else "Disconnected"
        print(f"---> {txt}")
