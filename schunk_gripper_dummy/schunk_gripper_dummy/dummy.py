from threading import Thread, Timer
import time
from importlib.resources import files
import json
import struct
from typing import Tuple
from pathlib import Path


class LinearMotion(object):
    def __init__(
        self,
        initial_pos: int,
        initial_speed: int,
        target_pos: int,
        target_speed: int,
    ):
        self.min_speed = 1  # um / s
        self.initial_pos = initial_pos
        self.initial_speed = max(0, initial_speed)
        self.target_pos = target_pos
        self.target_speed = max(self.min_speed, target_speed)
        self.time_finish = abs(target_pos - initial_pos) / self.target_speed

    def sample(self, t: float) -> Tuple[int, int]:
        if t <= 0.0:
            return (self.initial_pos, self.initial_speed)
        if t >= self.time_finish:
            return (self.target_pos, 0)

        v = self.target_speed
        if self.target_pos < self.initial_pos:
            v = -v
        current_pos = int(v * t) + self.initial_pos
        current_speed = abs(v)
        return (current_pos, current_speed)


class Dummy(object):
    def __init__(self, gripper: str = "EGU_60_EI_M_B"):

        # Gripper variants
        self.available_grippers: dict = {}
        gripper_folder = str(files(__package__).joinpath("config/grippers"))
        for json_file in Path(gripper_folder).glob("*.json"):
            with open(json_file, "r", encoding="utf-8") as f:
                self.available_grippers[json_file.stem] = json.load(f)
        self.data = self.available_grippers.get(gripper, [])
        if not self.data:
            print(
                f"Unknown gripper: {gripper}. "
                f"Available grippers: {list(self.available_grippers.keys())}"
            )
            raise ValueError("Unknown gripper")

        # Fieldbus
        self.valid_fieldbus_types = {
            "1": "PN",
            "2": "EI",
            "3": "EC",
        }
        fieldbus_data = bytes.fromhex(self.data["0x1130"][0])
        self.fieldbus = self.valid_fieldbus_types.get(
            str(struct.unpack("B", fieldbus_data)[0]), ""
        )
        if not self.fieldbus:
            raise ValueError("Unknown fieldbus")

        # Parameters
        self.valid_parameters: dict = {}
        params = str(files(__package__).joinpath("config/gripper_parameter_codes.json"))
        with open(params, "r", encoding="utf-8") as f:
            self.valid_parameters = json.load(f)
        if not self.valid_parameters:
            raise ValueError("Unknown parameter set")

        self.starttime = time.time()
        self.reachable = True
        self.thread = Thread(target=self._run)
        self.running = False
        self.done = False
        self.plc_input = "0x0040"
        self.plc_output = "0x0048"
        self.actual_position = "0x0230"
        self.actual_speed = "0x0238"
        self.system_uptime = "0x1400"
        self.error_byte = 12
        self.diagnostics_byte = 15
        self.valid_status_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 17, 31]
        self.valid_control_bits = list(range(0, 10)) + [11, 12, 13, 14, 16, 30, 31]
        self.reserved_status_bits = [10, 15] + list(range(18, 31))
        self.reserved_control_bits = [10, 15] + list(range(17, 30))
        self.plc_input_buffer = bytearray(bytes.fromhex(self.data[self.plc_input][0]))
        self.plc_output_buffer = bytearray(bytes.fromhex(self.data[self.plc_output][0]))
        self.initial_state = [self.plc_input_buffer.hex().upper()]
        self.clear_plc_output()

        data = bytes.fromhex(self.data["0x0650"][0])
        if self.fieldbus != "PN":
            data = data[::-1]
        self.max_grp_vel = struct.unpack("i", bytearray(data))[0]

        data = bytes.fromhex(self.data["0x0600"][0])
        if self.fieldbus != "PN":
            data = data[::-1]
        min_pos_per_jaw = struct.unpack("i", bytearray(data))[0]

        data = bytes.fromhex(self.data["0x0608"][0])
        if self.fieldbus != "PN":
            data = data[::-1]
        max_pos_per_jaw = struct.unpack("i", bytearray(data))[0]
        self.min_position = 2 * min_pos_per_jaw
        self.max_position = 2 * max_pos_per_jaw

    def start(self) -> None:
        if self.running:
            return
        self.thread.start()
        self.running = True

    def stop(self) -> None:
        self.done = True
        self.thread.join()
        self.running = False

    def _run(self) -> None:
        while not self.done:
            elapsed = time.time() - self.starttime
            self.set_system_uptime(int(elapsed))
            time.sleep(1)
        print("Done")

    def handle_events(self, events: dict) -> bool:
        if "lose_connection_sec" in events:
            self.reachable = False

            def reset() -> None:
                self.reachable = True

            Timer(interval=events["lose_connection_sec"], function=reset).start()
        return True

    def acknowledge(self) -> None:
        self.set_status_bit(bit=0, value=True)
        self.set_status_bit(bit=7, value=False)
        self.set_status_error("00")
        self.set_status_diagnostics("00")

    def move(self, target_pos: int, target_speed: int) -> None:
        motion = LinearMotion(
            initial_pos=self.get_actual_position(),
            initial_speed=self.get_actual_speed(),
            target_pos=target_pos,
            target_speed=target_speed,
        )
        start = time.time()
        actual_pos, actual_speed = motion.sample(0)
        while abs(target_pos - actual_pos) > 1:  # um
            t = time.time() - start
            actual_pos, actual_speed = motion.sample(t)
            self.set_actual_position(actual_pos)
            self.set_actual_speed(actual_speed)
            time.sleep(0.01)

    def update(self, msg: dict) -> dict:
        self.data[msg["inst"]] = [msg["value"]]
        if msg["inst"] == self.plc_output:
            self.plc_output_buffer = bytearray(bytes.fromhex(msg["value"]))

        # Behavior
        self.process_control_bits()
        return {"result": 0}

    def get_data(self, query: dict[str, str]) -> list:
        result: list = []

        if "inst" in query and "count" in query:
            inst = query["inst"]
            count = int(query["count"])
            if count != 1:
                return result
            if inst not in self.data:
                return result
            if inst == self.plc_input:
                return self.get_plc_input()
            if inst == self.plc_output:
                return self.get_plc_output()
            return self.data[inst]
        else:
            return []

    def get_plc_input(self):
        return [self.plc_input_buffer.hex().upper()]

    def get_plc_output(self):
        return [self.plc_output_buffer.hex().upper()]

    def clear_plc_output(self) -> None:
        self.plc_output_buffer = bytearray(bytes.fromhex("00" * 16))
        self.set_control_bit(
            bit=0, value=True
        )  # deactivate fast stop (inverted behavior)

    def set_status_bit(self, bit: int, value: bool) -> bool:
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

    def get_status_bit(self, bit: int) -> int | bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_status_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        return 1 if self.plc_input_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def toggle_status_bit(self, bit: int) -> bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_status_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        self.plc_input_buffer[byte_index] ^= 1 << bit_index
        return True

    def set_status_error(self, error: str) -> bool:
        try:
            self.plc_input_buffer[self.error_byte] = int(error, 16)
            return True
        except ValueError:
            return False

    def get_status_error(self) -> str:
        return hex(self.plc_input_buffer[self.error_byte]).replace("0x", "").upper()

    def set_status_diagnostics(self, diagnostics: str) -> bool:
        try:
            self.plc_input_buffer[self.diagnostics_byte] = int(diagnostics, 16)
            return True
        except ValueError:
            return False

    def get_status_diagnostics(self) -> str:
        return (
            hex(self.plc_input_buffer[self.diagnostics_byte]).replace("0x", "").upper()
        )

    def get_control_bit(self, bit: int) -> int | bool:
        if bit < 0 or bit > 31:
            return False
        if bit in self.reserved_control_bits:
            return False
        byte_index, bit_index = divmod(bit, 8)
        return 1 if self.plc_output_buffer[byte_index] & (1 << bit_index) != 0 else 0

    def set_control_bit(self, bit: int, value: bool) -> bool:
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

    def get_target_position(self) -> int:
        data = self.plc_output_buffer[4:8]
        if self.fieldbus == "PN":
            data = data[::-1]
        return struct.unpack("i", data)[0]

    def get_target_speed(self) -> int:
        data = self.plc_output_buffer[8:12]
        if self.fieldbus == "PN":
            data = data[::-1]
        return struct.unpack("i", data)[0]

    def set_actual_position(self, position: int) -> None:
        data = bytes(struct.pack("i", position))
        if self.fieldbus == "PN":
            data = data[::-1]
        self.plc_input_buffer[4:8] = data

    def set_actual_speed(self, speed: int) -> None:
        self.data[self.actual_speed] = [
            bytes(struct.pack("f", speed / 1e3)).hex().upper()
        ]

    def get_actual_position(self) -> int:
        read_pos = bytes.fromhex(self.plc_input_buffer[4:8].hex().upper())
        if self.fieldbus == "PN":
            read_pos = read_pos[::-1]
        return struct.unpack("i", read_pos)[0]

    def get_actual_speed(self) -> int:
        read_speed = self.data[self.actual_speed][0]
        return int(struct.unpack("f", bytes.fromhex(read_speed))[0] * 1e3)

    def set_system_uptime(self, uptime: int) -> None:
        self.data[self.system_uptime] = [bytes(struct.pack("i", uptime)).hex().upper()]

    def get_system_uptime(self) -> int:
        uptime = self.data[self.system_uptime][0]
        return struct.unpack("i", bytes.fromhex(uptime))[0]

    def process_control_bits(self) -> None:
        """
        See the gripper's firmware documentation for EtherNet/IP [1]:
        https://stb.cloud.schunk.com/media/IM0046706.PDF

        """
        # Reset status bits of previous commands
        for bit in self.valid_status_bits:
            if bit in [5, 7]:
                pass
            else:
                self.set_status_bit(bit=bit, value=False)

        # Clearing all control bits doesn't trigger any action
        if self.get_plc_output()[0] == "01" + "00" * 15:
            return

        # Command received toggle
        self.toggle_status_bit(bit=5)

        # Acknowledge
        if self.get_control_bit(2) == 1:
            self.set_status_bit(bit=0, value=True)
            self.set_status_bit(bit=7, value=False)
            self.set_status_error("00")
            self.set_status_diagnostics("00")
            self.clear_plc_output()
            return

        # Manual release
        if self.get_control_bit(5) == 1:
            if self.get_status_bit(7) == 1:
                self.set_status_bit(bit=8, value=True)
            self.clear_plc_output()
            return

        # Ignore further commands when in error
        if self.get_status_bit(7) == 1:
            self.clear_plc_output()
            return

        # Brake test
        if self.get_control_bit(30) == 1:
            self.set_status_bit(bit=4, value=True)
            self.clear_plc_output()
            return

        # Fast stop
        if self.get_control_bit(0) == 0:  # fail-safe behavior
            self.set_status_bit(bit=7, value=True)
            self.set_status_bit(bit=0, value=False)
            self.set_status_error("D9")
            self.set_status_diagnostics("00")
            self.clear_plc_output()
            return

        # Controlled stop
        if self.get_control_bit(1) == 1:
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=4, value=True)
            self.clear_plc_output()
            return

        # Shutdown
        if self.get_control_bit(3) == 1:
            self.set_status_bit(bit=2, value=True)
            self.clear_plc_output()
            return

        # Release workpiece
        if self.get_control_bit(bit=11) == 1:
            self.set_status_bit(bit=4, value=True)
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=14, value=False)
            self.set_status_bit(bit=12, value=False)
            self.set_status_bit(bit=17, value=False)
            self.clear_plc_output()
            return

        # Soft reset
        if self.get_control_bit(bit=4) == 1:
            self.set_system_uptime(0)
            self.data[self.plc_input] = self.initial_state
            self.clear_plc_output()
            return

        # Move to absolute position
        if self.get_control_bit(bit=13) == 1:
            if self.get_target_speed() <= 0:
                self.set_status_bit(bit=3, value=True)
                self.clear_plc_output()
                return

            self.move(
                target_pos=self.get_target_position(),
                target_speed=self.get_target_speed(),
            )
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=4, value=True)
            self.clear_plc_output()
            return

        # Move to relative position
        if self.get_control_bit(bit=14) == 1:
            target_pos = (
                self.get_actual_position()
                + self.get_target_position()  # interpret relative
            )
            self.move(
                target_pos=target_pos,
                target_speed=self.get_target_speed(),
            )
            self.set_status_bit(bit=13, value=True)
            self.set_status_bit(bit=4, value=True)
            self.clear_plc_output()
            return

        # Grip workpiece
        if self.get_control_bit(bit=12) == 1:

            if self.get_control_bit(bit=7) == 1:  # outward
                self.move(
                    target_pos=self.max_position,
                    target_speed=self.max_grp_vel,
                )
            else:  # normal
                self.move(
                    target_pos=self.min_position,
                    target_speed=self.max_grp_vel,
                )

            self.set_status_bit(bit=12, value=True)
            self.set_status_bit(bit=4, value=True)
            self.clear_plc_output()
            return

        # Grip workpiece at position
        if self.get_control_bit(bit=16) == 1:
            self.set_status_bit(bit=12, value=True)
            self.set_status_bit(bit=4, value=True)
            self.set_status_bit(bit=31, value=True)
            self.clear_plc_output()
            return

        # Start jogging
        def keep_jogging(negative: bool = False) -> None:
            if negative:
                target_pos = self.min_position
                jog_bit = 8
            else:
                target_pos = self.max_position
                jog_bit = 9

            motion = LinearMotion(
                initial_pos=self.get_actual_position(),
                initial_speed=self.get_actual_speed(),
                target_pos=target_pos,
                target_speed=self.get_target_speed(),
            )
            start = time.time()
            actual_pos, actual_speed = motion.sample(0)
            while (
                self.get_control_bit(bit=jog_bit) == 1
                and abs(target_pos - actual_pos) > 1
            ):  # um
                t = time.time() - start
                actual_pos, actual_speed = motion.sample(t)
                self.set_actual_position(actual_pos)
                self.set_actual_speed(actual_speed)
                time.sleep(0.01)
            print("Done!")

        if self.get_control_bit(bit=9) == 1 and self.get_control_bit(bit=8) == 1:
            return
        if self.get_control_bit(bit=9) == 1:
            Timer(interval=0.0, function=keep_jogging).start()
            return
        if self.get_control_bit(bit=8) == 1:
            Timer(
                interval=0.0, function=keep_jogging, kwargs={"negative": True}
            ).start()
            return

        # Stop jogging
        if self.get_control_bit(bit=8) == 0 or self.get_control_bit(bit=9) == 0:
            pass

        self.clear_plc_output()
