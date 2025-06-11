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

    def decode(self, data: bytes) -> None:  # response → object fields
        self._sub_function, self.expectancy = struct.unpack(">HH", data)

    def get_response_pdu_size(self) -> int:
        # Reply carries the same 4-byte body (sub-function + data)
        return 4


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

        self.clear_buffer()

    def clear_buffer(self, delay: float = 0.2):
        """Clear any stale data from the receive and transmit buffers"""
        if self.client.socket and self.client.socket.is_open:
            try:
                self.client.socket.reset_input_buffer()
                # Add a small delay between clearing input and output
                time.sleep(0.05)
                self.client.socket.reset_output_buffer()
                if delay > 0:
                    time.sleep(delay)
            except Exception as e:
                print(f"Warning: Failed to clear buffers: {e}")

    def get_serial_number(self, slave: int) -> str | None:
        """Get the serial number of the gripper using the serial_no_num parameter."""

        # Clear buffer before every operation
        self.clear_buffer()

        if not self.client.connected:
            self.client.connect()
            time.sleep(0.1)

        try:
            # Add retry mechanism for reliability
            for attempt in range(3):
                result = self.client.read_holding_registers(
                    address=0x1020 - 1, slave=slave, count=2
                )

                if not result.isError() and result.dev_id == slave:
                    serial_num = (result.registers[0] << 16) | result.registers[1]
                    serial_hex_str = f"{serial_num:08X}"
                    return serial_hex_str

                time.sleep(0.2)  # Wait between attempts

            return None

        except Exception as e:
            print(f"Error reading serial number from device {slave}: {e}")
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
            self.clear_buffer()
            # Validate and convert hex string serial number to 4-byte integer
            if len(serial_number) != 8:
                raise ValueError(
                    f"Serial number must be 8 hex characters, got: {serial_number}"
                )

            # Convert hex string directly to integer
            try:
                serial_hex = int(serial_number, 16)
            except ValueError:
                raise ValueError(
                    f"Serial number must be valid hex string, got: {serial_number}"
                )

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
                f"Successfully sent ID change command for serial"
                f"{serial_number} to ID {new_id}"
            )
            return True

        except Exception as e:
            print(f"Error changing gripper ID by serial: {e}")
            return False

    def set_expectancy(self, expectancy: int = 1, slave: int = 0) -> bool:
        """
        Set response expectancy for Modbus devices.
        Args:
            expectancy: Integer value (0-255) that will be converted to hex
            slave: Slave ID (0 for broadcast)
        """
        self.clear_buffer()

        # Validate input range
        if not (0 <= expectancy <= 255):
            print(f"Expectancy must be between 0 and 255, got: {expectancy}")
            return False

        if not self.client.connect():
            print("Failed to connect to serial port")
            return False

        try:
            # Convert integer directly to hex (ResponseExpectancyRequest expects int)
            req = ResponseExpectancyRequest(expectancy=expectancy, slave=slave)
            self.client.execute(request=req, no_response_expected=True)
            print(f"Successfully set expectancy to {expectancy} (0x{expectancy:04X})")
            return True
        except Exception as e:
            print(f"Unexpected error sending expectancy command: {e}")
            return False

    def change_gripper_id(self, old_id: int, new_id: int, check_success: bool = False):
        """
        Change the ID of the gripper by writing to a specific register.
        """

        try:
            self.clear_buffer()

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
        self, gripper_num: int, start_id: int = 20, universal_id: int = 10
    ) -> bool:
        """
        Function to find grippers in the system and assign them individual IDs.
        This function will:
        Args:
            gripper_num: Number of grippers expected to be found
        """
        self.clear_buffer()

        # Broadcast to change all IDs to a universal ID first
        self.client.retries = 0
        print(f"Broadcasting ID change from 0 to {universal_id}")
        self.change_gripper_id(old_id=0, new_id=universal_id)
        time.sleep(0.2)

        # self.clear_buffer()

        # Set probability of response for all grippers
        # 5 => every fifth request will be received by the gripper
        self.set_expectancy(expectancy=10, slave=0)  # Broadcast to all
        time.sleep(0.2)

        # self.clear_buffer()
        # Collect serial numbers and assign individual IDs
        grippers_found: list[dict] = []
        attempt = 0
        while True:
            if len(grippers_found) >= gripper_num:
                break

            print(
                f"Attempt {attempt + 1}: Looking for grippers with"
                f"universal ID {universal_id}"
            )
            attempt += 1

            # Ensure connection is active
            if not self.client.connected:
                self.client.connect()
                time.sleep(0.1)

            # Try to get serial number from universal ID
            serial_number = self.get_serial_number(slave=universal_id)

            if serial_number and isinstance(serial_number, str):
                # Check if we already found this gripper
                if serial_number not in [g["serial"] for g in grippers_found]:
                    new_id = start_id + len(grippers_found)
                    print(
                        f"Found new gripper with serial {serial_number},"
                        f"assigning ID {new_id}"
                    )
                    time.sleep(0.1)
                    # Change this specific gripper's ID using its serial number
                    success = self.change_gripper_id_by_serial_num(
                        serial_number=serial_number, new_id=new_id
                    )

                    if success:
                        time.sleep(0.1)  # Small delay to ensure change takes effect
                        grippers_found.append(
                            {
                                "serial": serial_number,
                                "old_id": universal_id,
                                "new_id": new_id,
                            }
                        )
                        print(
                            f"Successfully assigned ID {new_id}"
                            f"to gripper {serial_number}"
                        )

                        if (gripper_num - len(grippers_found)) == 1:
                            self.set_expectancy(
                                expectancy=1, slave=0
                            )  # Set low expectancy for last gripper

            time.sleep(0.1)

        if len(grippers_found) == gripper_num:
            return True
        else:
            return False


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
