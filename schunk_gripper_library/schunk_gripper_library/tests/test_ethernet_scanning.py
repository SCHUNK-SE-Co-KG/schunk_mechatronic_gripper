from schunk_gripper_library.utility import EthernetScanner as Scanner
import time


def test_scanner_is_ready_on_enter():
    for _ in range(3):
        with Scanner() as scanner:
            assert scanner.is_ready


def test_scanner_supports_reusing_the_context_manager():
    scanner = Scanner()
    for _ in range(5):
        with scanner:
            assert scanner


def test_scanner_implements_a_scan_method(ethernet_gripper):
    # The fixture provides an HMS chip that should
    # respond to the scanning requests.
    with Scanner() as scanner:
        grippers = scanner.scan()
        assert isinstance(grippers, list)
        assert len(grippers) >= 1

        for gripper in grippers:
            assert isinstance(gripper, dict)
            assert isinstance(gripper["host"], str)
            assert isinstance(gripper["port"], int)

            assert gripper["host"] != ""
            assert gripper["port"] == 80


def test_scan_raises_exception_without_context_manager():
    scanner = Scanner()
    try:
        scanner.scan()
        assert False, "Expected RuntimeError"
    except RuntimeError:
        pass


def test_scan_completes_within_timeout_limit():
    with Scanner() as scanner:
        start = time.time()
        try:
            scanner.scan()
        except Exception:
            pass
        duration = time.time() - start
        epsilon = 0.25  # some margin for scheduling delays
        assert duration < (scanner.socket_timeout + epsilon)
