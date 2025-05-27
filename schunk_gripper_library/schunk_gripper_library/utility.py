from threading import Thread
from queue import PriorityQueue
from concurrent.futures import Future
from functools import partial
import time
from pathlib import Path
from httpx import Client, ConnectTimeout, ConnectError
import pytest
from pymodbus.client.serial import ModbusSerialClient as ModbusClient
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
    def __init__(self):
        pass

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

        discovered_devices = []

        try:
            for id in range(start_address, end_address + 1):
                if not client.connected:
                    await client.connect()

                try:
                    result = await client.read_device_information(slave=id)

                    discovered_devices.append(result.dev_id)

                except Exception:
                    pass

            return discovered_devices
        finally:
            if client.connected:
                client.close()
                time.sleep(3)

    def change_gripper_id(self, new_id: int):
        """
        Change the ID of the gripper by writing to a specific register.
        """
        client = ModbusClient(
            port="/dev/ttyUSB0",
            baudrate=115200,
            timeout=0.1,
            parity="N",
            stopbits=1,
            bytesize=8,
            retries=0,
        )

        if not client.connect():
            print("Failed to connect to serial port")
            return []
        try:
            from pymodbus.payload import BinaryPayloadBuilder

            from pymodbus.constants import Endian

            builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)
            builder.add_8bit_uint(new_id)
            payload = builder.to_registers()
            register_address = 0x11A7

            client.write_registers(
                register_address, payload, slave=0, no_response_expected=True
            )

            print("Successfully changed gripper ID")
            return True

        except Exception as e:
            print(f"Unexpected error sending broadcast message: {e}")
            return False

        finally:
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
