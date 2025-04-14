import struct
from threading import Lock
from pymodbus.client import ModbusSerialClient
from pymodbus.pdu import ModbusPDU
import re
from threading import Thread
import time
from httpx import Client, ConnectError, ConnectTimeout
from importlib.resources import files
from typing import Union
import json
from .utility import Scheduler, supports_parity
from functools import partial
from pymodbus.logging import Log
import serial  # type: ignore [import-untyped]


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
    def __init__(self) -> None:
        self.plc_input: str = "0x0040"
        self.plc_output: str = "0x0048"
        self.error_byte: int = 12
        self.warning_byte: int = 14
        self.additional_byte: int = 15
        self.gripper: str = ""
        self.module: str = ""
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
        self.input_buffer_lock: Lock = Lock()
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

    def connect(
        self,
        host: str = "",
        port: int = 80,
        serial_port: str = "/dev/ttyUSB0",
        device_id: int | None = None,
        update_cycle: float | None = 0.05,
    ) -> bool:
        if isinstance(update_cycle, float) and update_cycle < 0.001:
            return False
        if isinstance(update_cycle, int) and update_cycle <= 0:
            return False
        if self.connected:
            return False

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
                    trace_connect=None,
                    trace_packet=None,
                    trace_pdu=None,
                )
                self.connected = self.mb_client.connect()

        if self.connected:
            if not (data := self.read_module_parameter("0x1130")):
                return False
            self.fieldbus = self.valid_fieldbus_types.get(
                str(struct.unpack("h", data)[0]), ""
            )

            if update_cycle:
                self.update_cycle = update_cycle
                self.polling_thread = Thread(
                    target=self._module_update,
                    args=(self.update_cycle,),
                    daemon=True,
                )
                self.polling_thread.start()

            if not self.update_module_parameters():
                return False

            self.module = self.valid_module_types.get(
                str(self.module_parameters["module_type"]), ""
            )
            self.gripper = self.compose_gripper_type(
                module=self.module, fieldbus=self.fieldbus
            )
            if not self.gripper:
                return False

        return self.connected

    def disconnect(self) -> bool:
        self.connected = False
        self.module = ""
        self.fieldbus = ""
        self.gripper = ""
        if self.polling_thread.is_alive():
            self.polling_thread.join()

        if self.mb_client and self.mb_client.connected:
            with self.mb_client_lock:
                self.mb_client.close()

        if self.web_client:
            with self.web_client_lock:
                self.web_client = None

        self.update_module_parameters()
        return True

    def acknowledge(self, scheduler: Scheduler | None = None) -> bool:
        if not self.connected:
            return False

        def do() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
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

    def stop(self) -> bool:
        if not self.connected:
            return False

        self.clear_plc_output()
        self.send_plc_output()

        cmd_toggle_before = self.get_status_bit(bit=5)
        self.set_control_bit(bit=1, value=True)
        self.send_plc_output()

        desired_bits = {"5": cmd_toggle_before ^ 1, "13": 1, "4": 1}
        return self.wait_for_status(bits=desired_bits)

    def move_to_absolute_position(
        self,
        position: int,
        velocity: int,
        use_gpe: bool = False,
        scheduler: Scheduler | None = None,
    ) -> bool:
        if not self.connected:
            return False
        if not self.set_target_position(position):
            return False
        if not self.set_target_speed(velocity):
            return False

        def start():
            self.clear_plc_output()
            self.send_plc_output()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=13, value=True)
            if self.gpe_available():
                self.set_control_bit(bit=31, value=use_gpe)
            else:
                self.set_control_bit(bit=31, value=False)
            self.set_target_position(position)
            self.set_target_speed(velocity)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "3": 0}
            return self.wait_for_status(bits=desired_bits, timeout_sec=0.1)

        def check():
            desired_bits = {"13": 1, "4": 1}
            return self.wait_for_status(bits=desired_bits)

        duration_sec = self.estimate_duration(position_abs=position, velocity=velocity)
        if scheduler:
            if not scheduler.execute(func=partial(start)).result():
                return False
            if self.error_in(duration_sec):
                return False
            return scheduler.execute(func=partial(check)).result()
        else:
            if not start():
                return False
            if self.error_in(duration_sec):
                return False
            return check()

    def move_to_relative_position(
        self, position: int, velocity: int, use_gpe: bool
    ) -> bool:
        if not self.connected:
            return False

        self.clear_plc_output()
        self.send_plc_output()

        cmd_toggle_before = self.get_status_bit(bit=5)
        self.set_control_bit(bit=14, value=True)
        self.set_control_bit(bit=31, value=use_gpe)
        self.set_target_position(position)
        self.set_target_speed(velocity)

        self.send_plc_output()
        desired_bits = {"5": cmd_toggle_before ^ 1, "13": 1, "4": 1}

        return self.wait_for_status(bits=desired_bits)

    def grip(
        self,
        force: int,
        use_gpe: bool = False,
        outward: bool = False,
        scheduler: Scheduler | None = None,
    ) -> bool:
        if not self.connected:
            return False
        if not self.set_gripping_force(force):
            return False

        def start() -> bool:
            self.clear_plc_output()
            self.send_plc_output()

            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=12, value=True)
            self.set_control_bit(bit=7, value=outward)
            if self.gpe_available():
                self.set_control_bit(bit=31, value=use_gpe)
            else:
                self.set_control_bit(bit=31, value=False)
            self.set_gripping_force(force)
            self.set_target_speed(0)
            self.send_plc_output()
            desired_bits = {"5": cmd_toggle_before ^ 1, "3": 0}
            return self.wait_for_status(bits=desired_bits, timeout_sec=0.1)

        def check() -> bool:
            desired_bits = {"4": 1, "12": 1}
            return self.wait_for_status(bits=desired_bits)

        duration_sec = self.estimate_duration(force=force, outward=outward)
        if scheduler:
            if not scheduler.execute(func=partial(start)).result():
                return False
            if self.error_in(duration_sec):
                return False
            return scheduler.execute(func=partial(check)).result()
        else:
            if not start():
                return False
            if self.error_in(duration_sec):
                return False
            return check()

    def release(
        self, use_gpe: bool = False, scheduler: Scheduler | None = None
    ) -> bool:
        if not self.connected:
            return False

        def start() -> bool:
            self.clear_plc_output()
            self.send_plc_output()
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

        def check() -> bool:
            desired_bits = {
                "4": 1,
                "13": 1,
            }
            return self.wait_for_status(bits=desired_bits)

        duration_sec = self.estimate_duration(release=True)
        if scheduler:
            if not scheduler.execute(func=partial(start)).result():
                return False
            if self.error_in(duration_sec):
                return False
            return scheduler.execute(func=partial(check)).result()
        else:
            if not start():
                return False
            if self.error_in(duration_sec):
                return False
            return check()

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

    def estimate_duration(
        self,
        release: bool = False,
        position_abs: int = 0,
        velocity: int = 0,
        force: int = 0,
        outward: bool = False,
    ) -> float:
        if release:
            return (
                self.module_parameters["wp_release_delta"]
                / self.module_parameters["max_vel"]
            )
        if not isinstance(position_abs, int):
            return 0.0
        if isinstance(velocity, int) and velocity > 0:
            still_to_go = position_abs - self.get_actual_position()
            return abs(still_to_go) / velocity
        if isinstance(force, int) and force > 0 and force <= 100:
            if outward:
                still_to_go = (
                    self.module_parameters["max_pos"] - self.get_actual_position()
                )
            else:
                still_to_go = (
                    self.module_parameters["min_pos"] - self.get_actual_position()
                )
            ratio = force / 100
            return abs(still_to_go) / (ratio * self.module_parameters["max_grp_vel"])
        return 0.0

    async def grip_workpiece_at_expected_position(
        self,
        gripping_force: int,
        workpiece_pos: int,
        use_gpe: bool,
        gripping_velocity: int = 0,
        grip_inside: bool = False,
        grip_outside: bool = False,
    ) -> bool:
        if not self.connected:
            return False

        if grip_inside == grip_outside:
            return False

        if gripping_force < 0:
            return False

        self.clear_plc_output()
        self.send_plc_output()

        cmd_toggle_before = self.get_status_bit(bit=5)

        self.set_control_bit(bit=16, value=True)
        self.set_control_bit(bit=7, value=True if grip_inside else False)
        self.set_gripping_force(gripping_force)
        self.set_target_speed(gripping_velocity)
        self.set_target_position(workpiece_pos)

        self.send_plc_output()
        desired_bits = {"5": cmd_toggle_before ^ 1, "4": 1, "12": 1}
        return await self.wait_for_status(bits=desired_bits)

    async def release_for_manual_movement(self) -> bool:
        if not self.connected:
            return False

        self.clear_plc_output()
        self.send_plc_output()
        cmd_toggle_before = None

        if self.get_status_bit(bit=7) == 1:
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=5, value=True)
            self.send_plc_output()
        else:
            self.set_control_bit(bit=0, value=False)
            self.send_plc_output()
            self.clear_plc_output()
            desired_bits = {"7": 1}
            if not await self.wait_for_status(bits=desired_bits):
                return False
            self.set_control_bit(bit=0, value=True)
            self.send_plc_output()
            self.clear_plc_output()
            cmd_toggle_before = self.get_status_bit(bit=5)
            self.set_control_bit(bit=5, value=True)
            self.send_plc_output()

        desired_bits = {"5": cmd_toggle_before ^ 1, "8": 1}
        return await self.wait_for_status(bits=desired_bits, timeout_sec=2)

    async def brake_test(self):
        if not self.connected:
            return False

        self.clear_plc_output()
        self.send_plc_output()
        cmd_toggle_before = self.get_status_bit(bit=5)
        self.set_control_bit(bit=30, value=True)
        self.send_plc_output()

        desired_bits = {"5": cmd_toggle_before ^ 1, "4": 1}

        return await self.wait_for_status(bits=desired_bits)

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
        if not self.module:
            return False
        keys = self.module.split("_")
        if len(keys) < 3:
            return False
        if keys[2] == "M":
            return True
        return False

    def update_module_parameters(self) -> bool:

        if not self.connected:
            for key in self.module_parameters.keys():
                self.module_parameters[key] = None
            return True

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

        return True

    def read_module_parameter(self, param: str) -> bytearray:
        result = bytearray()
        if param not in self.readable_parameters:
            return result

        if self.mb_client and self.mb_client.connected:
            with self.mb_client_lock:
                pdu = self.mb_client.read_holding_registers(
                    address=int(param, 16) - 1,
                    count=int(self.readable_parameters[param]["registers"]),
                    slave=self.mb_device_id,
                    no_response_expected=False,
                )
            # Parse each 2-byte register,
            # reverting pymodbus' internal big endian decoding.
            if not pdu.isError():
                for reg in pdu.registers:
                    result.extend(reg.to_bytes(2, byteorder="big"))

        if self.web_client and self.connected:
            params = {"inst": param, "count": "1"}
            with self.web_client_lock:
                response = self.web_client.get(
                    f"http://{self.host}:{self.port}/adi/data.json", params=params
                )
            if response.is_success:
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

    def wait_for_status(
        self, bits: dict[str, int] = {}, timeout_sec: float = 1.0
    ) -> bool:
        if not timeout_sec > 0.0:
            return False
        if not bits:
            return False
        max_duration = time.time() + timeout_sec
        while not all(
            [self.get_status_bit(int(bit)) == value for bit, value in bits.items()]
        ):
            time.sleep(0.001)
            self.receive_plc_input()
            if time.time() > max_duration:
                return False
        return True

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

    def compose_gripper_type(self, module: str, fieldbus: str) -> str:
        if (
            module not in self.valid_module_types.values()
            or fieldbus not in self.valid_fieldbus_types.values()
        ):
            return ""
        entries = module.split("_")
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
            if target_pos < 0:
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
            if gripping_force <= 0:
                return False
            if gripping_force > 100:
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

    def _module_update(self, update_cycle: float) -> None:
        while self.connected:
            self.receive_plc_input()
            time.sleep(update_cycle)

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
