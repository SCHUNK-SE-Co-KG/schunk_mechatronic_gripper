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
        self.response_payload: bytes = bytes.fromhex(
            (
                "00301145328d0100"
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

    def _valid(self, data: bytes) -> tuple[bool, str]:
        sequence = bytearray(data)
        result = (False, "")
        if len(data) != 18:
            return result
        if sequence[:2] != bytes.fromhex("c1ab"):
            return result
        if sequence[2:8] != bytes.fromhex("ff" * 6):
            return result
        if sequence[-4:] != bytes.fromhex("00" * 4):
            return result
        result = (True, sequence[8:-4].hex())
        return result

    def _listen(self) -> None:
        while not self.stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(1024)
                valid, mac_addr = self._valid(data)
                if valid:
                    response = bytes.fromhex("c1ab")
                    response += bytes.fromhex(mac_addr)
                    response += self.response_payload
                    self.sock.sendto(response, addr)
            except socket.timeout:
                continue
