from ..utility import (
    skip_without_bks,
    Scanner,
)
from .etc.scanner_helper import (
    start_bks_simulation,
    stop_bks_simulation,
    stop_all,
    get_last_log,
)
import time


# Bad case tests
@skip_without_bks
def test_scanner_returns_no_serial_number_for_invalid_id():
    scanner = Scanner()
    invalid_id = 9999
    assert scanner.get_serial_number(dev_id=invalid_id) is None


@skip_without_bks
def test_scanner_returns_no_serial_number_for_non_existent_id():
    scanner = Scanner()
    non_existent_id = 40
    assert scanner.get_serial_number(dev_id=non_existent_id) is None


@skip_without_bks
def test_scanner_rejects_invalid_serial_number(cleanup):
    scanner = Scanner()
    start_bks_simulation(sim_id=15, serial_num="00000015")
    time.sleep(0.5)  # Wait for the simulation to stabilize

    # Invalid serial numbers
    invalid_serial_too_many_chars = "0000000000000"
    assert not scanner.change_gripper_id_by_serial_num(
        new_id=15, serial_number=invalid_serial_too_many_chars
    )

    invalid_serial_too_few_chars = "000000"
    assert not scanner.change_gripper_id_by_serial_num(
        new_id=15, serial_number=invalid_serial_too_few_chars
    )

    invalid_serial_invalid_chars = "00000#!$"
    assert not scanner.change_gripper_id_by_serial_num(
        new_id=15, serial_number=invalid_serial_invalid_chars
    )

    stop_bks_simulation(sim_id=15)


@skip_without_bks
def test_scanner_changes_gripper_id(cleanup):
    scanner = Scanner()
    sim_id = 18
    serial_num = "00000018"
    start_bks_simulation(sim_id=sim_id, serial_num=serial_num)

    time.sleep(0.5)

    assert scanner.change_gripper_id(old_id=sim_id, new_id=20)

    assert scanner.get_serial_number(dev_id=20) == serial_num
    stop_bks_simulation(sim_id=18)


@skip_without_bks
def test_scanner_rejects_invalid_expectancy_and_invalid_dev_ids(cleanup):
    scanner = Scanner()
    start_bks_simulation(sim_id=16, serial_num="00000016")
    time.sleep(0.5)

    assert not scanner.set_expectancy(expectancy=400, dev_id=16)

    assert not scanner.set_expectancy(expectancy=-1, dev_id=16)

    assert not scanner.set_expectancy(expectancy=10, dev_id=400)


@skip_without_bks
def test_scanner_connects_automatically(cleanup):
    scanner = Scanner()
    start_bks_simulation(sim_id=17, serial_num="00000017")

    time.sleep(0.5)

    scanner.client.close()
    time.sleep(0.5)  # Wait for the client to close

    scanner.set_expectancy(expectancy=10, dev_id=17)  # This should trigger auto-connect
    time.sleep(0.5)  # Wait for the connection to establish

    assert scanner.client.connected


@skip_without_bks
def test_scanner_offers_reading_serial_number(cleanup):
    scanner = Scanner()
    serial = "00000013"  # Example serial number
    start_bks_simulation(sim_id=13, serial_num=serial)

    time.sleep(0.5)  # Wait for the simulation to stabilize
    serial_number = scanner.get_serial_number(dev_id=13)

    assert serial_number == serial

    stop_bks_simulation(sim_id=13)


@skip_without_bks
def test_scanner_changes_responds_expectancy(cleanup):
    scanner = Scanner()

    start_bks_simulation(sim_id=14, serial_num="00000014")

    scanner.set_expectancy(expectancy=255, dev_id=14)
    time.sleep(0.5)
    last_log = get_last_log(sim_id=14)

    assert "set_response_expectancy" in last_log and "set to 255" in last_log

    scanner.set_expectancy(expectancy=0, dev_id=14)
    time.sleep(0.5)
    last_log = get_last_log(sim_id=14)
    assert "set_response_expectancy" in last_log and "set to 0" in last_log

    stop_bks_simulation(sim_id=14) is True


@skip_without_bks
def test_scanner_changes_id_using_grippers_serial_number(cleanup):
    scanner = Scanner()
    serial = "00000019"  # Example serial number
    start_bks_simulation(sim_id=20, serial_num=serial)
    time.sleep(0.5)

    assert scanner.change_gripper_id_by_serial_num(serial_number=serial, new_id=25)
    time.sleep(0.5)
    assert scanner.get_serial_number(dev_id=25) == serial

    stop_bks_simulation(sim_id=20)


@skip_without_bks
def test_scanner_assigns_individual_ids(cleanup):
    scanner = Scanner()

    max_simulations = 3
    simulations = []
    start_id = 30

    # Start multiple grippers
    for i in range(max_simulations):
        sim_id = 20 + i
        init_serial_num = f"000000{sim_id:02d}"
        start_bks_simulation(sim_id=sim_id, serial_num=init_serial_num, device_index=i)

        time.sleep(0.5)

        simulations.append(sim_id)

    # Assign new IDs to the grippers
    start_time = time.perf_counter()
    scanner.scan(max_simulations, start_id=start_id, expected_response_rate=0.3)
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    time.sleep(0.5)

    # Verify that the serial numbers are unique and match the assigned IDs
    serial_nums = []
    for x in range(max_simulations):
        serial_num = scanner.get_serial_number(dev_id=start_id + x)
        assert serial_num is not None, f"Serial number for ID {start_id + x} is None"
        assert (
            serial_num not in serial_nums
        ), f"Serial number {serial_num} for ID {start_id + x} is not unique"
        serial_nums.append(serial_num)

    stop_all()

    assert execution_time < 15


@skip_without_bks
def test_scan_returns_list_of_devices(cleanup):
    scanner = Scanner()

    # No grippers
    device_ids = scanner.scan(gripper_num=0)
    assert len(device_ids) == 0

    # Multiple grippers
    num_grippers = 2
    for i in range(num_grippers):
        sim_id = 20 + i
        init_serial_num = f"000000{sim_id:02d}"
        start_bks_simulation(sim_id=sim_id, serial_num=init_serial_num, device_index=i)
        time.sleep(0.5)

    device_ids = scanner.scan(gripper_num=num_grippers, expected_response_rate=0.3)
    assert len(device_ids) == num_grippers

    assert all(isinstance(dev_id, int) for dev_id in device_ids)

    stop_all()
