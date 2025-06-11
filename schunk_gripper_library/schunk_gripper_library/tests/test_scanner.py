from schunk_gripper_library.utility import (
    skip_without_gripper,
    skip_without_bks,
    Scanner,
)
from .etc.scanner_helper import start_bks_simulation, stop_bks_simulation, stop_all
import time


@skip_without_gripper
def test_change_gripper_id():
    scanner = Scanner()

    result = scanner.change_gripper_id(old_id=42, new_id=16)

    assert result is True


@skip_without_bks
def test_get_serial_number(cleanup):
    scanner = Scanner()
    serial = "00000013"  # Example serial number
    result = start_bks_simulation(sim_id=13, serial_num=serial)
    assert result is True, "Failed to start BKS simulation with ID 13 and serial number"
    time.sleep(0.5)  # Wait for the simulation to stabilize
    serial_number = scanner.get_serial_number(13)
    assert (
        serial_number == serial
    ), f"Expected serial number {serial}, got {serial_number}"
    stop_bks_simulation(sim_id=13)


@skip_without_bks
def test_change_expectancy(cleanup):
    scanner = Scanner()

    assert start_bks_simulation(sim_id=14, serial_num="00000014")

    assert scanner.set_expectancy(expectancy=255, slave=14)

    for i in range(5):
        assert (
            scanner.get_serial_number(14) is None
        ), "Griper should not respond (~1.9%% change  that it responds)"

    assert (
        stop_bks_simulation(sim_id=14) is True
    ), "Failed to stop BKS simulation with ID 14"


@skip_without_bks
def test_change_id_using_serial_number(cleanup):
    scanner = Scanner()
    serial = "00000019"  # Example serial number
    assert start_bks_simulation(sim_id=20, serial_num=serial)
    time.sleep(0.5)
    assert scanner.get_serial_number(20) == serial

    result = scanner.change_gripper_id_by_serial_num(serial_number=serial, new_id=25)
    assert result is True
    assert (
        scanner.get_serial_number(25) == serial
    ), "Serial number does not match after changing ID"

    stop_bks_simulation(sim_id=20)


@skip_without_bks
def test_create_and_stop_one_simulation_with_id_and_serial_number(cleanup):
    scanner = Scanner()

    sim_id = 21
    serial_num = "00000020"  # Example serial number
    result = start_bks_simulation(sim_id=sim_id, serial_num=serial_num)
    assert result is True, "Failed to start BKS simulation with ID and serial number"
    time.sleep(0.5)
    assert (
        scanner.get_serial_number(sim_id) == serial_num
    ), "Serial number does not match after starting simulation"

    result = stop_bks_simulation(sim_id=sim_id)
    assert result is True, "Failed to stop BKS simulation with ID and serial number"


@skip_without_bks
def test_start_and_stop_multiple_simulations(cleanup):
    scanner = Scanner()

    max_simulations = 3
    simulations = []

    for i in range(max_simulations):
        sim_id = 20 + i
        serial_num = f"000000{sim_id:02d}"
        result = start_bks_simulation(
            sim_id=sim_id, serial_num=serial_num, device_index=i
        )
        assert result is True, (
            f"Failed to start BKS simulation with ID "
            f"{sim_id} and serial number {serial_num}"
        )

        time.sleep(1)  # Wait for simulation to stabilize

        assert scanner.get_serial_number(sim_id) == serial_num, (
            f"Serial number does not match for ID {sim_id}."
            f" Expected: {serial_num}, Got: {scanner.get_serial_number(sim_id)}"
        )

        simulations.append(sim_id)
        time.sleep(1)  # Additional delay before next simulation

    for sim_id in simulations:
        result = stop_bks_simulation(sim_id)
        assert result is True, f"Failed to stop BKS simulation with ID {sim_id}"


@skip_without_bks
def test_assign_ids(cleanup):
    scanner = Scanner()

    max_simulations = 3
    simulations = []
    start_id = 30

    for i in range(max_simulations):
        sim_id = 20 + i
        serial_num = f"000000{sim_id:02d}"
        result = start_bks_simulation(
            sim_id=sim_id, serial_num=serial_num, device_index=i
        )
        assert result is True, (
            f"Failed to start BKS simulation with ID"
            f"{sim_id} and serial number {serial_num}"
        )

        time.sleep(1)

        assert scanner.get_serial_number(sim_id) == serial_num, (
            f"Serial number does not match for ID {sim_id}. "
            f"Expected: {serial_num}, Got: {scanner.get_serial_number(sim_id)}"
        )

        simulations.append(sim_id)
        time.sleep(1)  # Additional delay before next simulation

    scanner.assign_ids(3, start_id=start_id)

    serial_nums = []
    for x in range(max_simulations):
        serial_num = scanner.get_serial_number(start_id + x)
        assert serial_num is not None, f"Serial number for ID {start_id + x} is None"
        assert (
            serial_num not in serial_nums
        ), f"Serial number {serial_num} for ID {start_id + x} is not unique"
        serial_nums.append(serial_num)

    result = stop_all()
