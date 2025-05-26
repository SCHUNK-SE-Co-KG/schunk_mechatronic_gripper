from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper, Scanner
import asyncio


@skip_without_gripper
def test_scanner_returns_list_of_IDs(time_execution):
    start_address = 1
    end_address = 20
    driver = Driver()

    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    scanner = Scanner(driver)

    result = asyncio.run(
        scanner.scan(start_address=start_address, end_address=end_address)
    )

    assert isinstance(result, list)
    assert len(result) != 0
    for item in result:
        assert isinstance(item, int)


@skip_without_gripper
def test_scanner_with_specific_device_id():
    import time

    time.sleep(5)
    start_address = 1
    end_address = 20

    driver = Driver()

    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    scanner = Scanner(driver)

    device_id = 12

    grippers = asyncio.run(
        scanner.scan(start_address=start_address, end_address=end_address)
    )

    assert device_id in grippers


@skip_without_gripper
def test_scanner_scans_specified_number_of_grippers():
    driver = Driver()

    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    scanner = Scanner(driver)

    num_grippers = 1

    result = asyncio.run(scanner.scan(gripper_num=num_grippers))

    assert len(result) == num_grippers
