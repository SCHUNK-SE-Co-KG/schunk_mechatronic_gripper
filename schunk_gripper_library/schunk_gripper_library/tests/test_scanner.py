from schunk_gripper_library.utility import skip_without_gripper, Scanner
import asyncio


@skip_without_gripper
def test_scanner_returns_list_of_IDs(time_scan):
    start_address = 1
    end_address = 20

    scanner = Scanner()

    result, execution_time = time_scan(
        scanner, start_address=start_address, end_address=end_address
    )

    print(f"Scanner execution time: {execution_time:.2f} seconds")
    print(f"Scanner result: {result}")
    assert isinstance(result, list)
    assert len(result) != 0
    for item in result:
        assert isinstance(item, int)


# @pytest.mark.skip
@skip_without_gripper
def test_change_gripper_id():
    scanner = Scanner()

    result = scanner.change_gripper_id(new_id=13)

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
