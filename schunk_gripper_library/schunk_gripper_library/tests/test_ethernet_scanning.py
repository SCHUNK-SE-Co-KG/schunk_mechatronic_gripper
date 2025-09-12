from schunk_gripper_library.utility import EthernetScanner as Scanner


def test_scanner_has_expected_fields():
    with Scanner() as scanner:
        assert scanner.socket is not None
        assert scanner.port is not None


def test_scanner_is_ready_on_enter():
    for _ in range(3):
        with Scanner() as scanner:
            assert scanner.is_ready


def test_scanner_closes_socket_on_exit():
    scanner = Scanner()
    with scanner:
        pass
    assert scanner.socket.fileno() == -1  # means closed

    # Repeated closing
    for _ in range(3):
        with scanner:
            pass


def test_scanner_supports_reusing_the_context_manager():
    scanner = Scanner()
    for _ in range(5):
        with scanner:
            assert scanner


def test_scanner_creates_new_socket_when_reset():
    scanner = Scanner()
    before = scanner.socket
    scanner._reset_socket()
    after = scanner.socket
    assert after != before
