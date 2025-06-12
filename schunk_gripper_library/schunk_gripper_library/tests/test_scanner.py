from ..utility import (
    skip_without_bks,
    Scanner,
)
from .etc.scanner_helper import (
    start_bks_simulation,
    stop_bks_simulation,
    stop_all,
    get_last_log,
    ScannerTestSetup,
)
import time
import pytest

pytestmark = pytest.mark.no_bks


# Bad case tests
@skip_without_bks
def test_scanner_returns_no_serial_number_for_invalid_id(cleanup):
    scanner = Scanner()
    invalid_id = 9999
    assert scanner.get_serial_number(dev_id=invalid_id) is None


@skip_without_bks
def test_scanner_returns_no_serial_number_for_non_existent_id(cleanup):
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
    with pytest.raises(ValueError):
        scanner.change_gripper_id_by_serial_num(
            new_id=15, serial_number=invalid_serial_too_many_chars
        )

    invalid_serial_too_few_chars = "000000"
    with pytest.raises(ValueError):
        scanner.change_gripper_id_by_serial_num(
            new_id=15, serial_number=invalid_serial_too_few_chars
        )

    invalid_serial_invalid_chars = "0000001A"
    with pytest.raises(ValueError):
        scanner.change_gripper_id_by_serial_num(
            new_id=15, serial_number=invalid_serial_invalid_chars
        )

    stop_bks_simulation(sim_id=15)


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
def test_scanner_offers_serial_number(cleanup):
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
    scanner.get_serial_number(dev_id=20) == serial

    assert scanner.change_gripper_id_by_serial_num(serial_number=serial, new_id=25)

    assert scanner.get_serial_number(dev_id=25) == serial

    stop_bks_simulation(sim_id=20)


@skip_without_bks
def test_scanner_assigns_individual_ids(cleanup):
    scanner = ScannerTestSetup()

    max_simulations = 3
    simulations = []
    start_id = 30

    for i in range(max_simulations):
        sim_id = 20 + i
        init_serial_num = f"000000{sim_id:02d}"
        start_bks_simulation(sim_id=sim_id, serial_num=init_serial_num, device_index=i)

        time.sleep(0.5)

        simulations.append(sim_id)

    scanner.assign_ids(3, start_id=start_id)

    serial_nums = []
    for x in range(max_simulations):
        serial_num = scanner.get_serial_number(dev_id=start_id + x)
        assert serial_num is not None, f"Serial number for ID {start_id + x} is None"
        assert (
            serial_num not in serial_nums
        ), f"Serial number {serial_num} for ID {start_id + x} is not unique"
        serial_nums.append(serial_num)

    stop_all()

    # cleanup to not effect other tests
    scanner.change_serial_num(dev_id=0, serial_number="00000000")
    time.sleep(0.5)
    scanner.change_gripper_id_by_serial_num(serial_number="00000000", new_id=12)
    time.sleep(0.5)
