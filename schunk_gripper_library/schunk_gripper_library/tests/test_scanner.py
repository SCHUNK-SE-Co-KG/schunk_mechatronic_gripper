from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper, Scanner


@skip_without_gripper
def test_scanner_returns_list_of_IDs():
    driver = Driver()

    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    scanner = Scanner()

    grippers = scanner.scan(start_address=1, end_address=50)

    assert isinstance(grippers, list)
    assert len(grippers) != 0
    for item in grippers:
        assert isinstance(item, int)


@skip_without_gripper
def test_scanner_with_specific_device_id():
    scanner = Scanner()

    device_id = 12

    grippers = scanner.scan(start_address=device_id, end_address=device_id)

    assert device_id in grippers
