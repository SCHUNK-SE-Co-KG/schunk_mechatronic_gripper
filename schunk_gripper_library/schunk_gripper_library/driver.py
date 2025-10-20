import struct
from threading import Lock, RLock
from pymodbus.client import ModbusSerialClient
from pymodbus.pdu import ModbusPDU
import re
from threading import Thread, Event
import time
from httpx import Client, ConnectError, ConnectTimeout, ReadTimeout, HTTPError
from importlib.resources import files
from typing import Union
import json
from .utility import Scheduler, supports_parity
from functools import partial
from pymodbus.logging import Log
import serial  # type: ignore [import-untyped]
from pymodbus.exceptions import ModbusIOException
from typing import Any, Type, cast
from enum import Enum, auto


class NonExclusiveSerialClient(ModbusSerialClient):
    def connect(self) -> bool:
        """
        Exact copy of the original connect() method with the sole exception of
        using `exclusive=False` for the serial connection. We need this to have
        several driver instances connect and speak over the same Modbus wire. A
        high-level entity will manage concurrency with a scheduler for
        multi-gripper scenarios.

        """
        if self.socket:  # type: ignore [has-type]
            return True
        try:
            self.socket = serial.serial_for_url(
                self.comm_params.host,
                timeout=self.comm_params.timeout_connect,
                bytesize=self.comm_params.bytesize,
                stopbits=self.comm_params.stopbits,
                baudrate=self.comm_params.baudrate,
                parity=self.comm_params.parity,
                exclusive=False,
            )
            self.socket.inter_byte_timeout = self.inter_byte_timeout
            self.last_frame_end = None
        except Exception as msg:
            Log.error("{}", msg)
            self.close()
        return self.socket is not None


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
        self.module_type: str = ""
        self.fieldbus: str = ""
        self.module_parameters: dict = {
            "module_type": None,
            "fieldbus_type": None,
            "serial_no_txt": None,
            "sw_version_txt": None,
            "min_pos": None,
            "max_pos": None,
            "max_vel": None,
            "max_grp_vel": None,
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
        self.reserved_control_bits: list[int] = [10, 15] + list(range(17, 30))

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

        self.mb_client: NonExclusiveSerialClient | None = None
        self.mb_device_id: int | None = None
        self.web_client: Client | None = None
        self.host: str = ""
        self.port: int = 80
        self.mb_client_lock: Lock = Lock()
        self.web_client_lock: Lock = Lock()
        self.connected: bool = False
        self.polling_thread: Thread = Thread()
        self.update_cycle: float = 0.05  # sec
        self.update_count: int = 0  # since last connect() call
        self.stop_request: Event = Event()
        self.reconnect_interval: float = 1.0

    def connect(
        self,
        host: str = "",
        port: int = 80,
        serial_port: str = "/dev/ttyUSB0",
        device_id: int | None = None,
        update_cycle: float | None = 0.05,
        scheduler: Scheduler | None = None,
    ) -> bool:
        if isinstance(update_cycle, float) and update_cycle < 0.001:
            return False
        if isinstance(update_cycle, int) and update_cycle <= 0:
            return False
        if self.connected:
            return False
        self.update_count = 0

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
            with self.mb_client_lock:
                self.mb_client = NonExclusiveSerialClient(
                    port=serial_port,
                    baudrate=115200,
                    parity="E" if supports_parity(serial_port) else "N",
                    stopbits=1,
                    timeout=0.1,
                    trace_connect=None,
                    trace_packet=None,
                    trace_pdu=None,
                )
                self.connected = self.mb_client.connect()

        if self.connected:
            updated = (
                scheduler.execute(func=partial(self.update_module_parameters)).result()
                if scheduler
                else self.update_module_parameters()
            )
            if not updated:
                return False
            if update_cycle:
                self.update_cycle = update_cycle
                self.start_module_updates(scheduler=scheduler)

        return self.connected

    def disconnect(self) -> bool:
        self.stop_module_updates()

        if self.mb_client and self.mb_client.connected:
            with self.mb_client_lock:
                self.mb_client.close()

        if self.web_client:
            with self.web_client_lock:
                self.web_client = None

        self.connected = False
        self.clear_module_parameters()
        return True

    def start_module_updates(self, scheduler: Scheduler | None = None) -> bool:
        if self.polling_thread.is_alive():
            return True
        self.polling_thread = Thread(
            target=self._module_update,
            args=(scheduler,),
            daemon=True,
        )
        self.polling_thread.start()
        return True

    def stop_module_updates(self) -> bool:
        self.stop_request.set()
        if self.polling_thread.is_alive():
            self.polling_thread.join()
        return True

    def acknowledge(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=2, value=True)
            self.send_plc_output()
            desired_bits = {"0": 1, "5": cmd_toggle_before ^ 1}
            return self.wait_for_status(bits=desired_bits)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def fast_stop(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(
                bit=0, value=False
            )  # activate fast stop (inverted behavior)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "7": 1}
            return self.wait_for_status(bits=desired_bits)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def stop(self, use_gpe: bool = False, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=1, value=True)
            if self.gpe_available():
                self.set_control_bit(bit=31, value=use_gpe)
            else:
                self.set_control_bit(bit=31, value=False)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "4": 1}
            return self.wait_for_status(bits=desired_bits)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def prepare_for_shutdown(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=3, value=True)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "2": 1}
            return self.wait_for_status(bits=desired_bits)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def move_to_position(
        self,
        position: int,
        velocity: int,
        is_absolute: bool = True,
        use_gpe: bool = False,
        scheduler: Scheduler | None = None,
    ) -> bool:
        if not self.connected:
            return False
        if not self.set_target_position(position):
            return False
        if not self.set_target_speed(velocity):
            return False

        if is_absolute:
            trigger_bit = 13
        else:
            trigger_bit = 14

        def start():
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=trigger_bit, value=True)
            if self.gpe_available():
                self.set_control_bit(bit=31, value=use_gpe)
            else:
                self.set_control_bit(bit=31, value=False)
            self.set_target_position(position)
            self.set_target_speed(velocity)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "3": 0}
            return self.wait_for_status(bits=desired_bits, timeout_sec=0.1)

        # send the move command
        if scheduler:
            if not scheduler.execute(func=partial(start)).result():
                return False
        else:
            if not start():
                return False

        # estimate how long the move will take
        epsilon_sec = 0.5  # additional time to account for delays
        estimated_duration_sec = self.estimate_duration(
            position_abs=position, is_absolute=is_absolute, velocity=velocity
        )
        duration_sec = estimated_duration_sec + epsilon_sec

        # wait for the command to complete or an error to occur
        bits: list[dict[str, int]] = []
        bits.append({"4": 1, "13": 1})  # command processed and position reached
        bits.append({"7": 1})  # error state
        matched_pattern = self.wait_for_any_status(bits=bits, timeout_sec=duration_sec)

        # a move has failed if either an error occured or the wait timed out
        return matched_pattern not in [{}, {"7": 1}]

    def grip(
        self,
        force: int,
        position: int | None = None,
        velocity: int | None = None,
        use_gpe: bool = False,
        outward: bool = False,
        scheduler: Scheduler | None = None,
    ) -> "Driver.GripResult":
        if not self.connected:
            return Driver.GripResult.ERROR
        if not self.set_gripping_force(force):
            return Driver.GripResult.ERROR
        if position is not None and not isinstance(position, int):
            return Driver.GripResult.ERROR
        if velocity is not None:
            if not isinstance(velocity, int) or velocity <= 0:
                return Driver.GripResult.ERROR

        if position is not None:
            trigger_bit = 16
        else:
            trigger_bit = 12

        def start() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=trigger_bit, value=True)
            self.set_control_bit(bit=7, value=outward)
            if self.gpe_available():
                self.set_control_bit(bit=31, value=use_gpe)
            else:
                self.set_control_bit(bit=31, value=False)
            self.set_gripping_force(force)
            if position is not None:
                self.set_target_position(position)
            if velocity is not None:
                self.set_target_speed(velocity)
            else:
                self.set_target_speed(0)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "3": 0}
            return self.wait_for_status(bits=desired_bits, timeout_sec=0.1)

        # send the grip command
        if scheduler:
            if not scheduler.execute(func=partial(start)).result():
                return Driver.GripResult.ERROR
        else:
            if not start():
                return Driver.GripResult.ERROR

        # estimate how long the grip will take
        epsilon_sec = 0.5  # additional time to account for delays
        estimated_duration_sec = self.estimate_duration(
            position_abs=position, velocity=velocity, force=force, outward=outward
        )
        # retrieve the prehold time in case the gripper is configured for pre-gripping
        prehold_time_sec = 0.0
        prehold_time_data = self.read_module_parameter("0x0380")
        values, value_type = self.decode_module_parameter(prehold_time_data, "0x0380")
        if value_type == "uint16" and len(values) == 1:
            prehold_time_sec = values[0] / 1000.0  # ms -> s

        duration_sec = estimated_duration_sec + prehold_time_sec + epsilon_sec

        # define the possible status bit patterns to wait for
        patterns = {}
        patterns[Driver.GripResult.WORKPIECE_GRIPPED] = {"4": 1, "12": 1}
        patterns[Driver.GripResult.NO_WORKPIECE_DETECTED] = {"4": 0, "11": 1}
        patterns[Driver.GripResult.WRONG_WORKPIECE_GRIPPED] = {"4": 0, "17": 1}
        patterns[Driver.GripResult.WORKPIECE_LOST] = {
            "4": 0,
            "16": 1,
        }  # relevant for pre-gripping
        patterns[Driver.GripResult.ERROR] = {"7": 1}

        # wait for the command to complete or an error to occur
        bits: list[dict[str, int]] = []
        for pattern in patterns.values():
            bits.append(pattern)
        matched_pattern = self.wait_for_any_status(bits=bits, timeout_sec=duration_sec)

        return next(
            (k for k, v in patterns.items() if v == matched_pattern),
            Driver.GripResult.ERROR,
        )

    def release(
        self, use_gpe: bool = False, scheduler: Scheduler | None = None
    ) -> bool:
        if not self.connected:
            return False

        def start() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=11, value=True)
            if self.gpe_available():
                self.set_control_bit(bit=31, value=use_gpe)
            else:
                self.set_control_bit(bit=31, value=False)
            self.send_plc_output()
            desired_bits = {
                "5": cmd_toggle_before ^ 1,
                "3": 0,
            }
            return self.wait_for_status(bits=desired_bits)

        # send the release command
        if scheduler:
            if not scheduler.execute(func=partial(start)).result():
                return False
        else:
            if not start():
                return False

        # estimate how long the release will take
        epsilon_sec = 0.5  # additional time to account for delays
        estimated_duration_sec = self.estimate_duration(release=True)
        duration_sec = estimated_duration_sec + epsilon_sec

        # wait for the command to complete or an error to occur
        bits: list[dict[str, int]] = []
        bits.append({"4": 1, "13": 1})  # command processed and position reached
        bits.append({"7": 1})  # error state
        matched_pattern = self.wait_for_any_status(bits=bits, timeout_sec=duration_sec)

        # a release has failed if either an error occured or the wait timed out
        return matched_pattern not in [{}, {"7": 1}]

    def release_for_manual_movement(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=5, value=True)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "8": 1}
            return self.wait_for_status(bits=desired_bits)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def show_specification(self) -> dict[str, float | str]:
        if not self.connected:
            return {}

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
        return spec

    def brake_test(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def start() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=30, value=True)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1}
            return self.wait_for_status(bits=desired_bits)

        def check() -> bool:
            desired_bits = {"4": 1}
            return self.wait_for_status(bits=desired_bits, timeout_sec=4.0)

        if scheduler:
            if not scheduler.execute(func=partial(start)).result():
                return False
            return scheduler.execute(func=partial(check)).result()
        else:
            if not start():
                return False
            return check()

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
        self, velocity: int, use_gpe: bool = False, scheduler: Scheduler | None = None
    ) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            still_jogging = False
            if self.get_control_bit(bit=8) == 1 or self.get_control_bit(bit=9) == 1:
                still_jogging = True

            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)

            if not self.set_target_speed(abs(velocity)):
                return False
            if velocity >= 0:
                self.set_control_bit(bit=9, value=True)
            else:
                self.set_control_bit(bit=8, value=True)
            if use_gpe:
                self.set_control_bit(bit=31, value=self.gpe_available())
            self.send_plc_output()

            if still_jogging:
                self.receive_plc_input()
                if self.get_status_bit(bit=6) == 0:
                    return True

            desired_bits = {"5": cmd_toggle_before ^ 1, "6": 0}
            return self.wait_for_status(bits=desired_bits)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def stop_jogging(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            # The firmware behaves differently when stopping jogging:
            # - Status bit toggles if jogging was active before.
            # - GPE bit from when jogging was started must be preserved.
            # We do not send a zero-frame as that would trigger the status bit
            # and clear the GPE bit. Stop jogging is intended to be preceded by
            # start jogging, otherwise this method will always return False
            # because the status bit won’t change.

            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=8, value=False)  # stop negative jogging
            self.set_control_bit(bit=9, value=False)  # stop positive jogging
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "6": 0}
            return self.wait_for_status(bits=desired_bits)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def twitch_jaws(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def move(step: int) -> bool:
            return self.move_to_position(
                position=step,
                velocity=self.module_parameters["max_vel"],
                is_absolute=False,
            )

        def do() -> bool:
            step = 2000  # um
            if not self.receive_plc_input():
                return False
            if self.get_actual_position() > self.module_parameters["max_pos"] - step:
                move(-step)
                move(step)
            move(step)
            move(-step)
            return True

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def soft_reset(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
            self.receive_plc_input()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=4, value=True)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1}
            return self.wait_for_status(bits=desired_bits, timeout_sec=1.0)

        if scheduler:
            return scheduler.execute(func=partial(do)).result()
        else:
            return do()

    def receive_plc_input(self) -> bool:
        with self.input_buffer_lock:
            data = self.read_module_parameter(self.plc_input)
            if data:
                self.plc_input_buffer = data
                return True
            return False

    def send_plc_output(self) -> bool:
        with self.output_buffer_lock:
            return self.write_module_parameter(self.plc_output, self.plc_output_buffer)

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

    def update_module_parameters(self) -> bool:
        if not (fieldbus_param := self.read_module_parameter("0x1130")):
            return False

        self.fieldbus = self.valid_fieldbus_types.get(
            str(struct.unpack("h", fieldbus_param)[0]), ""
        )

        value: int | str
        for param, fields in self.readable_parameters.items():
            if fields["name"] in self.module_parameters:
                field_type = str(fields["type"])
                if not (data := self.read_module_parameter(param)):
                    return False

                if field_type == "float":
                    if self.fieldbus == "PN":
                        value = int(struct.unpack("f", data[::-1])[0] * 1e3)
                    else:
                        value = int(struct.unpack("f", data)[0] * 1e3)

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

    def read_module_parameter(self, param: str) -> bytearray:
        """
        Reads the specified parameter from the module.

        Args:
            param (str): The parameter address in hex format, e.g. "0x0040".

        Returns:
            bytearray: The value of the specified parameter.
                       Use `decode_module_parameter()` to convert the
                       bytearray into the correct type.
        """
        result = bytearray()
        if param not in self.readable_parameters:
            return result

        if self.mb_client and self.mb_client.connected:
            with self.mb_client_lock:
                try:
                    pdu = self.mb_client.read_holding_registers(
                        address=int(param, 16) - 1,
                        count=int(self.readable_parameters[param]["registers"]),
                        slave=self.mb_device_id,
                        no_response_expected=False,
                    )
                except (ModbusIOException, IOError):
                    return result

            # Parse each 2-byte register,
            # reverting pymodbus' internal big endian decoding.
            if not pdu.isError():
                for reg in pdu.registers:
                    result.extend(reg.to_bytes(2, byteorder="big"))

        if self.web_client:
            params = {"inst": param, "count": "1"}
            with self.web_client_lock:
                try:
                    response = self.web_client.get(
                        f"http://{self.host}:{self.port}/adi/data.json", params=params
                    )
                except (ReadTimeout, ConnectError, ConnectTimeout):
                    return result
            if response.is_success:
                if response.json() == []:
                    return result
                result = bytearray(bytes.fromhex(response.json()[0]))

        if result:
            current_size = len(result)
            desired_size = int(self.readable_parameters[param]["registers"]) * 2
            if current_size < desired_size:
                result.extend([0] * (desired_size - current_size))  # zero-pad

        return result

    def write_module_parameter(self, param: str, data: bytearray) -> bool:
        if param not in self.writable_parameters:
            return False
        expected_size = self.writable_parameters[param]["registers"] * 2
        if len(data) != expected_size:
            return False

        if self.mb_client and self.mb_client.connected:
            # Turn the bytearray into a list of 2-byte registers.
            # Pymodbus uses big endian internally for their encoding.
            param_size = int(self.writable_parameters[param]["registers"]) * 2
            values = [
                int.from_bytes(data[i : i + 2], byteorder="big")
                for i in range(0, param_size, 2)
            ]
            with self.mb_client_lock:
                pdu = self.mb_client.write_registers(
                    address=int(param, 16) - 1,  # Modbus convention
                    values=values,
                    slave=self.mb_device_id,
                    no_response_expected=False,
                )
            return not pdu.isError()

        if self.web_client and self.connected:
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
            return result
        if not data or param not in self.writable_parameters:
            return result

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
        error: tuple[tuple[Any, ...], str] = (tuple(), "")
        if not self.connected:
            return error
        if not data:
            return error
        if param not in self.readable_parameters:
            return error

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
            return error

        return (values, value_type)

    def wait_for_status(self, bits: dict[str, int], timeout_sec: float = 1.0) -> bool:
        return bool(self.wait_for_any_status(bits=[bits], timeout_sec=timeout_sec))

    def wait_for_any_status(
        self, bits: list[dict[str, int]], timeout_sec: float
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
            with self.input_buffer_lock:
                for bit_pattern in bits:
                    if all(
                        [
                            self.get_status_bit(int(bit)) == value
                            for bit, value in bit_pattern.items()
                        ]
                    ):
                        return bit_pattern
            time.sleep(self.update_cycle)

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

    def get_status_bit(self, bit: int) -> int | bool:
        with self.input_buffer_lock:
            if bit < 0 or bit > 31:
                return False
            if bit in self.reserved_status_bits:
                return False
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
        with self.output_buffer_lock:
            if not isinstance(target_pos, int):
                return False
            data = bytes(struct.pack("i", target_pos))
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
        with self.output_buffer_lock:
            if not isinstance(target_speed, int):
                return False
            if target_speed < 0:
                return False
            data = bytes(struct.pack("i", target_speed))
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
                return False
            data = bytes(struct.pack("i", gripping_force))
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

    def _module_update(self, scheduler: Scheduler | None = None) -> None:
        self.stop_request.clear()
        fails = 0
        next_time = time.perf_counter()
        while not self.stop_request.is_set():
            runs_fine = (
                scheduler.execute(func=partial(self.receive_plc_input)).result()
                if scheduler
                else self.receive_plc_input()
            )
            if runs_fine:
                if self.connected:
                    self.update_count += 1
                    fails = 0
                else:
                    self.connected = (
                        scheduler.execute(
                            func=partial(self.update_module_parameters)
                        ).result()
                        if scheduler
                        else self.update_module_parameters()
                    )

                time.sleep(max(0, next_time - time.perf_counter()))
                next_time += self.update_cycle
            else:
                self.connected = False
                fails += 1
                if fails < 3:
                    time.sleep(self.update_cycle)
                else:
                    time.sleep(self.reconnect_interval)

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
