import socket
from socket import socket as Socket
from threading import Thread, Event


class HMSChip(object):
    def __init__(self) -> None:
        self.sock: Socket = Socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.5)
        self.port: int = 3250  # HMS standard
        self.trigger: bytes = bytes.fromhex("c1abffffffffffffac91a1644eda00000000")
        self.response: bytes = bytes.fromhex(
            (
                "c1abac91a1644eda00301145328d0100"
                "5b000001000107352e30332e30300215"
                "5346502d352e335f4547552036302d50"
                "4e2d4d2d420301030c01280b01400404"
                "2a00a8c0050400ffffff060400000000"
                "0704000000000804000000000901000a"
                "0c4547552d36302d4e53363636"
            )
        )
        self.thread: Thread = Thread()
        self.stop_event: Event = Event()

    def power_on(self) -> None:
        if self.thread.is_alive():
            return
        self.sock.bind(("", self.port))
        self.thread = Thread(target=self._listen, daemon=True)
        self.thread.start()

    def power_off(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join()

    def _listen(self) -> None:
        while not self.stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(1024)
                if data == self.trigger:
                    self.sock.sendto(self.response, addr)
            except socket.timeout:
                continue
