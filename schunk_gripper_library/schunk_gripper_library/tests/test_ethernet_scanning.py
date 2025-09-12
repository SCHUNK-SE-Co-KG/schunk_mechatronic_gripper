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


def test_scanner_offers_a_scan_method(ethernet_gripper):

    # The fixture provides an HMS chip that should
    # respond to the scanning requests.

    with Scanner() as scanner:
        grippers = scanner.scan()
        assert isinstance(grippers, list)
        assert len(grippers) == 1

        for entry in grippers:
            assert isinstance(entry, str)
            assert entry != ""


def test_scan_result_is_empty_when_nothing_found():

    # Start this test without the fixture
    with Scanner() as scanner:
        assert scanner.scan() == []
