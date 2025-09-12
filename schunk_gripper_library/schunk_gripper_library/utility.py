from threading import Thread
from queue import PriorityQueue
from concurrent.futures import Future
from functools import partial
import time
from pathlib import Path
from httpx import Client, ConnectTimeout, ConnectError
import pytest
import os
import termios
from socket import socket as Socket
import socket
import netifaces


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


class EthernetScanner(object):
    def __init__(self):
        self.is_ready: bool = False
        self.port: int = 3250  # HMS standard
        self._reset_socket()

    def scan(self) -> list[str]:
        result = []
        interfaces = netifaces.interfaces()
        interfaces = list(filter(lambda iface: iface.startswith("eth"), interfaces))

        for iface in interfaces:
            addresses = netifaces.ifaddresses(iface)
            if netifaces.AF_INET in addresses:
                ip_info = addresses[netifaces.AF_INET][0]
                broadcast_ip = ip_info.get("broadcast", "255.255.255.255")
                mac_addr = addresses[netifaces.AF_LINK][0]["addr"].replace(":", "")
                message = (
                    bytes.fromhex("c1ab")
                    + bytes.fromhex("ff" * 6)
                    + bytes.fromhex(mac_addr)
                    + bytes.fromhex("00" * 4)
                )
                try:
                    self.socket.sendto(message, (broadcast_ip, self.port))
                    try:
                        while True:
                            response, addr = self.socket.recvfrom(1024)
                            if response == message:  # it's me
                                continue
                            result.append(addr[0])
                    except socket.timeout:
                        pass
                except Exception as e:
                    print(f"Error on {iface}: {e}")

        return result

    def __enter__(self) -> "EthernetScanner":
        if self.socket.fileno() == -1:  # already closed once
            self._reset_socket()
        self.socket.bind(("", self.port))  # listen on all local interfaces
        self.is_ready = True
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()

    def _reset_socket(self) -> None:
        self.socket: Socket = Socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(1.0)


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
