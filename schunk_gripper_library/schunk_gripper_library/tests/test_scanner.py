from schunk_gripper_library.utility import skip_without_gripper, Scanner
import asyncio
import time


# @pytest.mark.skip
@skip_without_gripper
def test_change_gripper_id():
    scanner = Scanner()

    result = scanner.change_gripper_id(old_id=42, new_id=16)

    assert result is True


@skip_without_gripper
def test_scanner_with_specific_device_id():
    start_address = 1
    end_address = 247

    scanner = Scanner()

    device_id = 12

    grippers = asyncio.run(
        scanner.scan(start_address=start_address, end_address=end_address)
    )

    assert device_id in grippers


@skip_without_gripper
def test_scanner_scans_specified_number_of_grippers():
    scanner = Scanner()

    num_grippers = 1

    result = asyncio.run(scanner.scan(gripper_num=num_grippers))

    assert len(result) == num_grippers


@skip_without_gripper
def test_get_serial_number():
    scanner = Scanner()
    for _ in range(10):
        serial_number = scanner.get_serial_number(12)
        if serial_number:
            break

    print(f"Serial number: {serial_number}")
    assert isinstance(serial_number, str)
    assert len(serial_number) > 0
    assert serial_number.isalnum()


@skip_without_gripper
def test_force_listen():
    scanner = Scanner()

    result = scanner.set_expectancy(expectancy=1, slave=0)

    print(result)


@skip_without_gripper
def test_change_serial_number_and_use_it():
    scanner = Scanner()

    scanner.change_gripper_id(old_id=0, new_id=12)
    time.sleep(0.5)
    # Set a known serial number as hex string
    test_serial = "12345678"  # Hex string format
    result = scanner.change_serial_num(dev_id=12, serial_number=test_serial)
    print(f"Change serial number result: {result}")
    assert result is True

    # Wait a bit for the change to take effect
    time.sleep(0.5)

    # Read it back to confirm
    read_serial = scanner.get_serial_number(12)
    print(f"Read back serial: {read_serial}")
    assert read_serial == test_serial
    time.sleep(0.5)
    # Now use it to change the ID
    result = scanner.change_gripper_id_by_serial(serial_number=read_serial, new_id=20)
    print(f"Change using serial number result: {result}")
    assert result is True


@skip_without_gripper
def test_change_using_serial_number():
    scanner = Scanner()

    serial = "12345678"

    result = scanner.change_gripper_id_by_serial(serial_number=serial, new_id=20)
    print(f"Change using serial number result: {result}")
    assert result is True


@skip_without_gripper
def test_change_serial_number():
    scanner = Scanner()

    scanner.set_expectancy(expectancy=3, slave=0)
    time.sleep(0.2)
    serial = "87654321"
    result = scanner.change_serial_num(dev_id=20, serial_number=serial)
    time.sleep(0.2)
    for _ in range(10):
        serial_number = scanner.get_serial_number(20)
        if serial_number:
            break
    print(f"Serial number after change: {serial}")
    print(f"Change serial number result: {result}")
    assert result is True


@skip_without_gripper
def test_setup():
    """
    Setup workflow to have multiple grippers with different IDs and serial numbers:
    1. start one bks simulation
    2. run test_setup with a new id and serial number
    3. start second bks simulation
    4. run test_setup with a different id and serial number
    """
    scanner = Scanner()
    wanted_serial_num = "00000017"  # Example serial number
    new_id = 17

    scanner.change_serial_num(dev_id=12, serial_number=wanted_serial_num)
    time.sleep(0.2)
    scanner.change_gripper_id(old_id=12, new_id=new_id)
    time.sleep(0.2)
    serial_num = scanner.get_serial_number(new_id)
    print(f"Serial number after setup: {serial_num}")
    assert serial_num == wanted_serial_num


@skip_without_gripper
def test_assign_ids():
    scanner = Scanner()

    result = scanner.assign_ids(3)

    assert result is True
