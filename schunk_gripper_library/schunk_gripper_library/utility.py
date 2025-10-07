from threading import Thread
from queue import PriorityQueue
from concurrent.futures import Future
from functools import partial
import threading
import time
from pathlib import Path
from httpx import Client, ConnectTimeout, ConnectError
import pytest
import os
import termios
import socket
import netifaces
from concurrent.futures import ThreadPoolExecutor, as_completed


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
    """
    A network scanner for detecting grippers on all available Ethernet interfaces.

    This class uses UDP broadcast messages to discover grippers
    on the local network. All interfaces are scanned in parallel.
    This class is supposed to be used as a context manager to ensure sockets are
    properly initialized and closed.

    Usage:
        with EthernetScanner() as scanner:
            grippers = scanner.scan() # list of dicts with 'host'<->'port' mapping
    """

    def __init__(self) -> None:
        self.is_ready: bool = False
        self.discovery_port: int = 3250  # HMS standard
        self.webserver_port: int = 80
        self.socket_timeout: float = 1.0  # in seconds
        self.sockets: dict[str, socket.socket] = {}  # one socket per interface
        self.interfaces: list[str] = []
        self.lock = threading.Lock()

    def __enter__(self) -> "EthernetScanner":
        with self.lock:
            self.sockets = {}
            all_interfaces = netifaces.interfaces()
            AF_INET = netifaces.InterfaceType.AF_INET
            AF_PACKET = netifaces.InterfaceType.AF_PACKET

            for iface in all_interfaces:
                addr = netifaces.ifaddresses(iface)
                if AF_INET not in addr or AF_PACKET not in addr:
                    continue
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.settimeout(self.socket_timeout)
                # Bind to the specific interface's IP address to avoid conflicts
                iface_ip = addr[AF_INET][0]["addr"]
                sock.bind((iface_ip, self.discovery_port))
                self.sockets[iface] = sock
                self.interfaces.append(iface)

            self.is_ready = True

        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        with self.lock:
            for sock in self.sockets.values():
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()
            self.sockets.clear()
            self.is_ready = False

    def scan(self) -> list[dict]:
        """Scans all available ethernet interfaces in parallel for connected grippers.

        Returns:
            list[dict]: A list of dicts with keys 'host' and 'port'
        """
        with self.lock:
            if not self.is_ready:
                raise RuntimeError("EthernetScanner must be used as a context manager.")

            result: list[dict] = []
            max_workers = min(len(self.interfaces), 10)
            with ThreadPoolExecutor(max_workers=max_workers) as executor:
                futures = {
                    executor.submit(self._scan_interface, iface): iface
                    for iface in self.interfaces
                }
                for future in as_completed(futures.keys()):
                    try:
                        result.extend(future.result())
                    except Exception as e:
                        raise RuntimeError(
                            f"Error scanning interface {futures[future]}: {e}"
                        ) from e

            return result

    def _scan_interface(self, iface: str) -> list[dict]:
        """Scans a single interface using its dedicated socket.

        Args:
            iface (str): The name of the interface to scan.
        Returns:
            list[dict]: A list of dicts with keys 'host' and 'port'
        """
        result: list[dict] = []

        if not self.is_ready:
            raise RuntimeError("EthernetScanner must be used as a context manager.")

        if iface not in self.sockets:
            raise ValueError(f"Unsupported interface: {iface}.")

        addresses = netifaces.ifaddresses(iface)
        AF_INET = netifaces.InterfaceType.AF_INET
        AF_PACKET = netifaces.InterfaceType.AF_PACKET
        if AF_INET not in addresses or AF_PACKET not in addresses:
            return result

        broadcast_ip = addresses[AF_INET][0].get("broadcast", "255.255.255.255")
        mac_addr = addresses[AF_PACKET][0]["addr"].replace(":", "")
        message = (
            bytes.fromhex("c1ab")
            + bytes.fromhex("ff" * 6)
            + bytes.fromhex(mac_addr)
            + bytes.fromhex("00" * 4)
        )

        sock = self.sockets[iface]

        sock.sendto(message, (broadcast_ip, self.discovery_port))
        while True:
            try:
                response, addr = sock.recvfrom(1024)
                if response == message:  # ignore our own message
                    continue
                result.append({"host": addr[0], "port": self.webserver_port})
            except socket.timeout:
                break

        return result


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
