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
    start_bks_simulation(sim_id=15, serial_num="BCDX0015")
    time.sleep(0.5)  # Wait for the simulation to stabilize

    invalid_serial_nums = [
        "123456789",  # Too many characters
        "12345",  # Too few characters
        "AEIOU123",  # Contains AEIOU
        12345678,  # Non-string input
        None,  # None input
    ]

    for serial_num in invalid_serial_nums:
        assert not scanner.change_gripper_id_by_serial_num(
            new_id=20, serial_number=serial_num
        )

    stop_bks_simulation(sim_id=15)


@skip_without_bks
def test_scanner_rejects_invalid_gripper_id(cleanup):
    scanner = Scanner()
    start_bks_simulation(sim_id=19, serial_num="BCDX0019")
    time.sleep(0.5)
    # Invalid gripper IDs
    invalid_gripper_ids = [-1, 0, 1000, "abc"]

    for gripper_id in invalid_gripper_ids:
        assert not scanner.change_gripper_id_by_serial_num(
            new_id=gripper_id, serial_number="BCDX0019"
        )

    stop_bks_simulation(sim_id=19)


@skip_without_bks
def test_scanner_changes_gripper_id(cleanup):
    scanner = Scanner()
    sim_id = 18
    serial_num = "BCDX0018"
    start_bks_simulation(sim_id=sim_id, serial_num=serial_num)

    time.sleep(0.5)

    assert scanner.change_gripper_id(old_id=sim_id, new_id=20)

    # Since alphanumeric serials get converted to hash values and returned as hex,
    # we need to expect the hex representation of the hash
    expected_hash = hash(serial_num.upper()) & 0xFFFFFFFF
    expected_hex = f"{expected_hash:08X}"
    assert scanner.get_serial_number(dev_id=20) == expected_hex
    stop_bks_simulation(sim_id=18)


@skip_without_bks
def test_scanner_rejects_invalid_expectancy_and_invalid_dev_ids(cleanup):
    scanner = Scanner()
    start_bks_simulation(sim_id=16, serial_num="BCDX0016")
    time.sleep(0.5)

    assert not scanner.set_response_expectancy(expectancy=400, dev_id=16)

    assert not scanner.set_response_expectancy(expectancy=-1, dev_id=16)

    assert not scanner.set_response_expectancy(expectancy=10, dev_id=400)


@skip_without_bks
def test_scanner_connects_automatically(cleanup):
    scanner = Scanner()
    start_bks_simulation(sim_id=17, serial_num="BCDX0017")

    time.sleep(0.5)

    scanner.client.close()
    time.sleep(0.5)

    scanner.set_response_expectancy(expectancy=10, dev_id=17)
    time.sleep(0.5)

    assert scanner.client.connected

    scanner.client.close()
    time.sleep(0.5)

    scanner.get_serial_number(dev_id=17)
    time.sleep(0.5)

    assert scanner.client.connected


@skip_without_bks
def test_scanner_offers_reading_serial_number(cleanup):
    scanner = Scanner()
    serial = "BCDX1234"  # Use valid format: 4 letters (no AEIOU) + 4 digits
    start_bks_simulation(sim_id=13, serial_num=serial)

    time.sleep(0.5)  # Wait for the simulation to stabilize
    serial_number = scanner.get_serial_number(dev_id=13)

    # Since alphanumeric serials get converted to hash values and returned as hex,
    # we need to expect the hex representation of the hash
    # Convert the original serial the same way as change_serial_num does
    expected_hash = hash(serial.upper()) & 0xFFFFFFFF
    expected_hex = f"{expected_hash:08X}"

    assert serial_number == expected_hex

    stop_bks_simulation(sim_id=13)


@skip_without_bks
def test_scanner_changes_responds_expectancy(cleanup):
    scanner = Scanner()

    start_bks_simulation(sim_id=14, serial_num="BCDX0014")

    scanner.set_response_expectancy(expectancy=255, dev_id=14)
    time.sleep(0.5)
    last_log = get_last_log(sim_id=14)

    assert "set_response_expectancy" in last_log and "set to 255" in last_log

    scanner.set_response_expectancy(expectancy=0, dev_id=14)
    time.sleep(0.5)
    last_log = get_last_log(sim_id=14)
    assert "set_response_expectancy" in last_log and "set to 0" in last_log

    stop_bks_simulation(sim_id=14)


@skip_without_bks
def test_scanner_changes_id_using_grippers_serial_number(cleanup):
    scanner = Scanner()
    serial = "BCDF0020"  # Test with hex format like documentation example
    start_bks_simulation(sim_id=20, serial_num=serial)
    time.sleep(0.5)
    for x in range(5):
        assert scanner.change_gripper_id_by_serial_num(
            serial_number=serial, new_id=x + 14
        )
        time.sleep(0.5)
        assert scanner.get_serial_number(dev_id=x + 14) == serial
        time.sleep(0.5)

    stop_bks_simulation(sim_id=20)


@skip_without_bks
def test_scanner_assigns_individual_ids(cleanup):
    scanner = Scanner()

    max_simulations = 3
    simulations = []
    sim_start_id = 20
    default_start_id = 12

    # Start multiple grippers
    for i in range(max_simulations):
        sim_id = sim_start_id + i
        # Use valid format: 4 letters (no AEIOU) + 4 digits
        init_serial_num = f"BCDF{sim_id:04d}"
        start_bks_simulation(sim_id=sim_id, serial_num=init_serial_num, device_index=i)

        time.sleep(0.5)

        simulations.append(sim_id)

    # Assign new IDs to the grippers
    start_time = time.perf_counter()
    scanner.scan(gripper_num=max_simulations)
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    time.sleep(0.5)

    # Verify that the serial numbers are unique and match the assigned IDs
    serial_nums = []
    for x in range(max_simulations):
        serial_num = scanner.get_serial_number(dev_id=default_start_id + x)
        assert serial_num is not None
        assert serial_num not in serial_nums
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
        # Use valid format: 4 letters (no AEIOU) + 4 digits
        init_serial_num = f"BFJK{sim_id:04d}"
        start_bks_simulation(sim_id=sim_id, serial_num=init_serial_num, device_index=i)
        time.sleep(0.5)

    device_ids = scanner.scan(gripper_num=num_grippers, expected_response_rate=0.3)
    assert len(device_ids) == num_grippers

    assert all(isinstance(dev_id, int) for dev_id in device_ids)

    stop_all()


@skip_without_bks
def test_scanner_returns_none_for_wrong_dev_id(cleanup):
    scanner = Scanner()
    start_bks_simulation(sim_id=21, serial_num="BCDX0021")
    time.sleep(0.5)

    # Test with a wrong device ID
    wrong_dev_id = 30
    serial_number = scanner.get_serial_number(dev_id=wrong_dev_id)
    assert serial_number is None

    stop_bks_simulation(sim_id=21)


@skip_without_bks
def test_scan_with_scheduler(cleanup):
    from ..utility import Scheduler

    scanner = Scanner()
    scheduler = Scheduler()
    scheduler.start()

    # Start grippers
    for i in range(2):
        sim_id = 30 + i
        # Use valid format: 4 letters (no AEIOU) + 4 digits
        init_serial_num = f"BCGD{sim_id:04d}"
        start_bks_simulation(sim_id=sim_id, serial_num=init_serial_num, device_index=i)
        time.sleep(0.5)

    device_ids = scanner.scan(gripper_num=2, scheduler=scheduler)
    assert len(device_ids) == 2

    scheduler.stop()
    stop_all()


@skip_without_bks
def test_scan_different_response_rates(cleanup):
    """Test scan with different expected_response_rate values."""
    scanner = Scanner()

    # Test with very low rate
    for i in range(2):
        sim_id = 35 + i
        # Use valid format: 4 letters (no AEIOU) + 4 digits
        init_serial_num = f"BFGH{sim_id:04d}"
        start_bks_simulation(sim_id=sim_id, serial_num=init_serial_num, device_index=i)
        time.sleep(0.5)

    device_ids = scanner.scan(gripper_num=2, expected_response_rate=0.1)
    assert len(device_ids) == 2

    stop_all()
