from schunk_gripper_library.utility import EthernetScanner as Scanner


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
    # Note: If this test is run on a machine with multiple network interfaces,
    # then the HMS chip will respond multiple times.
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
