from threading import Thread
from queue import PriorityQueue
from concurrent.futures import Future
from functools import partial
import time
from pathlib import Path
from httpx import Client, ConnectTimeout, ConnectError
import pytest
from pymodbus.client.serial import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException, ModbusException, ConnectionException
import re
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


class Scanner(object):
    def __init__(self, driver):
        self.driver = driver

    async def scan(self, start_address=1, end_address=20, gripper_num=None):
        """
        Iterate over a range of Modbus IDs to find devices
        """
        from pymodbus.client.serial import AsyncModbusSerialClient

        client = AsyncModbusSerialClient(
            port="/dev/ttyUSB0",
            baudrate=115200,
            timeout=0.1,
            parity="N",
            stopbits=1,
            bytesize=8,
            retries=0,
        )

        if not await client.connect():
            print("Failed to connect to serial port")
            return []
        client.set_max_no_responses(99999)

        module_type_register = int("0x0500", 16) - 1
        discovered_devices = []

        try:
            for id in range(start_address, end_address + 1):
                if not client.connected:
                    print(f"Reconnecting to Modbus client for ID {id}")
                    await client.connect()
                print(f"connected: {client.connected}")

                try:
                    print(await client.read_device_information(slave=id))
                    response = await client.read_holding_registers(
                        address=module_type_register,
                        count=1,
                        slave=id,
                        no_response_expected=True,
                    )
                    print(f"Scanning ID {id}...")

                    discovered_devices.append(response.dev_id)
                    print(f"Found device at address: {response.dev_id}")

                except Exception as e:
                    print(f"Error reading from ID {id}: {e}")
                    pass

            return discovered_devices
        finally:
            if client.connected:
                client.close()
                time.sleep(3)

    def send_broadcast_message(self, register_address, value):
        """
        Send a broadcast message to all devices on the Modbus network
        and check which devices respond
        !!!only works since the Gripper reponds to every call!!! not very reliable
        """
        client = ModbusClient(
            port="/dev/ttyUSB0",
            baudrate=115200,
            timeout=0.1,
            parity="N",
            stopbits=1,
            bytesize=8,
            retries=10,
        )

        device_ids = []

        if not client.connect():
            print("Failed to connect to serial port")
            return []
        try:
            # Write to all devices using broadcast address (slave=0)
            response = client.write_register(
                address=register_address, value=value, slave=0  # Broadcast address
            )

            # Note: Broadcast messages don't return responses from devices
            # The response just indicates if the message was sent successfully
            if not response.isError():
                print("Broadcast message sent successfully to all devices.")
                return True
            else:
                print(f"Failed to send broadcast message: {response}")
                return False
        except ModbusIOException as e:
            # This catches the specific "Input/Output" error you mentioned
            if "request ask for id=0" in str(e):
                print(f"Device responded with its own ID: {e}")
                # Extract the actual device ID from the error message
                match = re.search(r"but id=(\d+)", str(e))
                if match:
                    actual_id = int(match.group(1))
                    device_ids.append(actual_id)
                    print(f"Device has ID: {actual_id}")
            else:
                print(f"Modbus I/O Error during broadcast: {e}")
            return False
        except ConnectionException as e:
            print(f"Connection error during broadcast: {e}")
            return False

        except ModbusException as e:
            print(f"General Modbus error during broadcast: {e}")
            return False

        except Exception as e:
            print(f"Unexpected error sending broadcast message: {e}")
            return False

        finally:
            print(device_ids)
            client.close()
            print("Closed Modbus client connection")


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
