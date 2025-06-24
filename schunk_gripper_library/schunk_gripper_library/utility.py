from threading import Thread
from queue import PriorityQueue
from concurrent.futures import Future
from functools import partial
import time
from pathlib import Path
from httpx import Client, ConnectTimeout, ConnectError
import pytest
from pymodbus.client.serial import ModbusSerialClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian
import serial
import struct
from pymodbus.pdu import ModbusPDU
from pymodbus.logging import Log
import os
import math
import pymodbus.logging as logging
import os
import termios

def supports_parity(serial_port: str) -> bool:
    fd = None
    try:
        fd = os.open(serial_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[2] |= termios.PARENB  # enable parity
        attrs[2] &= ~termios.PARODD  # set even parity
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return True
    except Exception:
        return False
    finally:
        if fd is not None:
            os.close(fd)
            
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


class Task(object):
    def __init__(
        self, future: Future | None = None, func: partial | None = None
    ) -> None:
        self.future: Future | None = future
        self.func: partial | None = func
        self.stamp: float = time.time()

    def __bool__(self) -> bool:
        if self.func is None:
            return False
        return True

    def __lt__(self, other: "Task") -> bool:
        return self.stamp < other.stamp

    def __gt__(self, other: "Task") -> bool:
        return self.stamp > other.stamp

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Task):
            return False
        if self.func is None or other.func is None:
            return False
        return (
            self.func.func == other.func.func
            and self.func.args == other.func.args
            and self.func.keywords == other.func.keywords
        )


class Scheduler(object):
    def __init__(self) -> None:
        self.tasks: PriorityQueue = PriorityQueue()
        self.worker_thread: Thread = Thread()
        self.enqueued_tasks: list[Task] = []

    def start(self) -> None:
        if not self.worker_thread.is_alive():
            self.worker_thread = Thread(target=self._process, daemon=True)
            self.worker_thread.start()

    def stop(self) -> None:
        self.tasks.put((0, Task()))  # highest priority
        if self.worker_thread.is_alive():
            self.worker_thread.join()
        self.tasks = PriorityQueue()

    def execute(self, func: partial, priority: int = 2) -> Future:
        future: Future = Future()
        if not self.worker_thread.is_alive():
            future.set_result(False)
            return future
        if priority not in [1, 2]:
            future.set_result(False)
            return future
        task = Task(func=func, future=future)
        if task not in self.enqueued_tasks:
            self.enqueued_tasks.append(task)
            self.tasks.put((priority, task))
        else:
            future.set_result(False)
        return future

    def cyclic_execute(
        self, func: partial, cycle_time: float, priority: int = 2
    ) -> bool:
        if not self.worker_thread.is_alive():
            return False
        if not cycle_time or not cycle_time > 0:
            return False
        if priority not in [1, 2]:
            return False
        task = Task(func=func, future=None)
        Thread(
            target=self._cyclic_add,
            kwargs={
                "task": task,
                "cycle_time": cycle_time,
                "priority": priority,
            },
            daemon=True,
        ).start()
        return True

    def _cyclic_add(self, task: Task, cycle_time: float, priority: int = 2) -> None:
        while self.worker_thread.is_alive():
            if task not in self.enqueued_tasks:
                self.enqueued_tasks.append(task)
                self.tasks.put((priority, task))
            time.sleep(cycle_time)

    def _process(self) -> None:
        while True:
            _, task = self.tasks.get()
            if not task:
                break
            self.enqueued_tasks.remove(task)
            result = task.func()
            if task.future:
                task.future.set_result(result)
            self.tasks.task_done()


class ResponseExpectancyRequest(ModbusPDU):
    function_code = 0x08
    _sub_function = 0x0004

    def __init__(self, expectancy: int, slave: int = 1) -> None:
        super().__init__(dev_id=slave)
        self.expectancy = expectancy

    # --- wire-format helpers -------------------------------------------------
    def encode(self) -> bytes:  # request → bytes
        # Big-endian “HH”: sub-function, data field
        return struct.pack(">HH", self._sub_function, self.expectancy)


class Scanner(object):
    def __init__(self):
        self.client = NonExclusiveSerialClient(
            port="/dev/ttyUSB0",
            baudrate=115200,
            timeout=0.1,
            parity="N",
            stopbits=1,
            bytesize=8,
            retries=0,
        )
        self.client.set_max_no_responses(99999)  # Set a high limit for no responses
        self.client.connect()

    def get_serial_number(self, dev_id: int) -> str | None:
        """Get the serial number of the gripper using the serial_no_num parameter."""

        if not (0 <= dev_id <= 247):
            print(f"Device ID must be between 0 and 247, got: {dev_id}")
            return None

        if not self.client.connected:
            self.client.connect()
            time.sleep(0.1)

        try:
            # Add retry mechanism for reliability
            for _ in range(3):
                result = self.client.read_holding_registers(
                    address=0x1020 - 1, slave=dev_id, count=2
                )

                if not result.isError() and result.dev_id == dev_id:
                    serial_num = (result.registers[0] << 16) | result.registers[1]
                    serial_hex_str = f"{serial_num:08X}"
                    return serial_hex_str

                time.sleep(0.2)  # Wait between attempts

            return None

        except Exception as e:
            print(f"Error reading serial number from device {dev_id}: {e}")
            return None

    def change_gripper_id_by_serial_num(self, serial_number: str, new_id: int) -> bool:
        """
        Change the ID of a specific gripper identified by its serial number.
        Args:
            serial_number: Serial number as hex string (e.g., "12345678")
            new_id: New device ID to assign
        This uses a broadcast message with serial number targeting.
        """
        try:

            # Validate and convert hex string serial number to 4-byte integer
            if len(serial_number) != 8:
                return False

            # Convert hex string directly to integer
            try:
                serial_hex = int(serial_number, 16)
            except ValueError:
                return False

            # Build payload according to the example format
            builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)

            # Add new slave ID (1 byte)
            builder.add_8bit_uint(new_id)
            builder.add_8bit_uint(0x00)  # Padding

            # Add new slave ID again (2 bytes)
            builder.add_16bit_uint(new_id)

            # Add serial number (4 bytes)
            builder.add_32bit_uint(serial_hex)

            # Add padding (4 bytes of 0xFF)
            for _ in range(4):
                builder.add_8bit_uint(0xFF)

            payload = builder.to_registers()
            register_address = 0x11A7

            # Use broadcast (slave ID 0) to send to all grippers
            self.client.write_registers(
                register_address, payload, slave=0, no_response_expected=True
            )

            print(
                f"Successfully sent ID change command for serial "
                f"{serial_number} to ID {new_id}"
            )
            return True

        except Exception as e:
            print(f"Error changing gripper ID by serial: {e}")
            return False

    def set_expectancy(self, expectancy: int = 1, dev_id: int = 0) -> bool:
        """
        Set response expectancy for Modbus devices.
        Args:
            expectancy: Integer value (0-255) that will be converted to hex
            slave: Slave ID (0 for broadcast)
        """

        # Validate input range
        if not (0 <= expectancy <= 255):
            print(f"Expectancy must be between 0 and 255, got: {expectancy}")
            return False

        if not (0 <= dev_id <= 247):
            print(f"Device ID must be between 0 and 247, got: {dev_id}")
            return False

        if not self.client.connected:
            try:
                self.client.connect()
                time.sleep(0.1)
            except Exception:
                return False

        try:
            # TODO: add check for broadcast
            # Convert integer directly to hex (ResponseExpectancyRequest expects int)
            req = ResponseExpectancyRequest(expectancy=expectancy, slave=dev_id)
            self.client.execute(request=req, no_response_expected=True)
            return True
        except Exception as e:
            print(f"Unexpected error sending expectancy command: {e}")
            return False

    def change_gripper_id(self, old_id: int, new_id: int):
        """
        Change the ID of the gripper by writing to a specific register.
        """

        try:

            builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)
            builder.add_8bit_uint(new_id)
            payload = builder.to_registers()
            register_address = 0x11A7

            self.client.retries = 0
            response = self.client.write_registers(
                register_address, payload, slave=old_id, no_response_expected=True
            )
            print("Successfully changed gripper ID to ", new_id)
            print(response)

            return True

        except Exception as e:
            print(f"Unexpected error sending broadcast message: {e}")
            return False

    def assign_ids(
        self,
        gripper_num: int,
        start_id: int = 20,
        universal_id: int = 12,
        lambda_target: float = 0.3,  # tune 0.2-0.5 to trade speed vs. collision risk
    ) -> bool:
        """
        Discover every gripper still listening on `universal_id`
        and give it a unique ID starting at `start_id`.

        Parameters
        ----------
        gripper_num : int
            How many grippers you expect to find.
        start_id : int, default 20
            First individual ID to assign.
        universal_id : int, default 12
            Temporary ID put on all grippers so they answer the same request.
        lambda_target : float, default 0.4
            Desired expected-answer rate (n/k). 0.2–0.5 works well.
        """

        # 1. PUSH ALL GRIPPERS ONTO THE SAME ID
        self.client.retries = 0
        print(f"Broadcasting: set ID → {universal_id} for all devices")
        self.change_gripper_id(old_id=0, new_id=universal_id)  # broadcast
        time.sleep(0.2)

        grippers_found: list[dict] = []
        remaining = gripper_num
        current_k: int | None = None
        attempt = 0

        while remaining > 0:
            # 2. CHOOSE AN EXPECTANCY AND BROADCAST IT
            k = max(1, math.ceil(remaining / lambda_target))

            if remaining == 1:
                k = 1

            if k != current_k:
                print(f"Broadcasting: set expectancy → {k} (n={remaining})")
                self.set_expectancy(expectancy=k, dev_id=0)  # broadcast
                current_k = k
                time.sleep(0.1)

            # 3. ASK “WHO HAS ID 12?” UNTIL WE GET *ONE* NEW SERIAL NUMBER
            if not self.client.connected:
                self.client.connect()
                time.sleep(0.05)

            attempt += 1
            print(f"Request #{attempt} to ID {universal_id} (k={k})")
            serial_number = self.get_serial_number(dev_id=universal_id)
            time.sleep(0.1)

            # no answer
            if not serial_number or not isinstance(serial_number, str):
                # optional: back-off if bus is noisy / collision detected
                continue

            # duplicate (already processed)
            if serial_number in (g["serial"] for g in grippers_found):
                print(f"Duplicate answer from {serial_number} — ignored")
                continue

            new_id = start_id + len(grippers_found)
            print(f"Assigning ID {new_id} to gripper {serial_number}")

            success = self.change_gripper_id_by_serial_num(
                serial_number=serial_number,
                new_id=new_id,
            )
            time.sleep(0.1)

            if not success:
                print("⚠️  ID change failed — retrying")
                continue

            grippers_found.append(
                {"serial": serial_number, "old_id": universal_id, "new_id": new_id}
            )
            remaining -= 1

            time.sleep(0.1)

        self.set_expectancy(expectancy=1, dev_id=0)  # reset expectancy
        return len(grippers_found) == gripper_num


def gripper_available() -> bool:
    client = Client()
    try:
        webserver_up = client.get(
            "http://0.0.0.0:8000/adi/data.json", timeout=1.0
        ).is_success
    except (ConnectTimeout, ConnectError):
        webserver_up = False
    modbus_server_up = Path("/dev/ttyUSB0").exists() or ""
    if webserver_up and modbus_server_up:
        return True
    return False


skip_without_gripper = pytest.mark.skipif(
    not gripper_available(), reason="No gripper/simulator available"
)


def skip_without_bks(func):
    def check_bks_available():
        # Check if BKS_SIMULATION_PATH is set
        if not os.getenv("BKS_SIMULATION_PATH"):
            return False, "BKS_SIMULATION_PATH environment variable not set"

        # Check if the BKS simulation executable exists
        bks_path = os.path.join(
            os.getenv("BKS_SIMULATION_PATH"), "BKS_Simulation_Windows"
        )
        if not os.path.exists(bks_path):
            return False, f"BKS simulation executable not found at {bks_path}"

        # Check if tty_bus socket exists (infrastructure running)
        tty_bus_socket = "/tmp/ttypts2mux"
        if not os.path.exists(tty_bus_socket):
            return False, "BKS test infrastructure not running (missing tty_bus socket)"

        # Check if /dev/ttyUSB0 is available
        if not os.path.exists("/dev/ttyUSB0"):
            return False, "Virtual serial device /dev/ttyUSB0 not available"

        return True, "BKS infrastructure available"

    available, reason = check_bks_available()

    return pytest.mark.skipif(
        not available, reason=f"BKS simulation not available: {reason}"
    )(func)
