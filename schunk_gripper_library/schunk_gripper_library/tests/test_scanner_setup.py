from ..utility import (
    skip_without_bks,
    Scanner,
)
from .etc.scanner_helper import (
    start_bks_simulation,
    stop_bks_simulation,
    get_last_log,
)
import time
import pytest


@skip_without_bks
def test_start_and_stop_multiple_simulations(cleanup):
    scanner = Scanner()

    max_simulations = 3
    simulations = []

    for i in range(max_simulations):
        sim_id = 20 + i
        serial_num = f"000000{sim_id:02d}"
        assert start_bks_simulation(
            sim_id=sim_id, serial_num=serial_num, device_index=i
        )

        time.sleep(0.5)  # Wait for simulation to stabilize

        assert scanner.get_serial_number(dev_id=sim_id) == serial_num

        simulations.append(sim_id)
        time.sleep(1)  # Additional delay before next simulation

    for sim_id in simulations:
        assert stop_bks_simulation(sim_id)


@skip_without_bks
def test_helper_rejects_invalid_serial_number():
    invalid_serials = ["", "12345678", "000000000000", "00000001a"]

    for serial in invalid_serials:
        assert not start_bks_simulation(sim_id=10, serial_num=serial)


@skip_without_bks
def test_launcher_rejects_duplicate_sim_ids(cleanup):
    sim_id = 10
    serial_num = "00000010"

    start_bks_simulation(sim_id=sim_id, serial_num=serial_num)
    time.sleep(0.5)

    assert not start_bks_simulation(sim_id=sim_id, serial_num="00000011")

    assert stop_bks_simulation(sim_id=sim_id)


@skip_without_bks
def test_launcher_rejects_duplicate_serial_numbers(cleanup):
    sim_id_1 = 10
    sim_id_2 = 11
    serial_num = "00000012"

    assert start_bks_simulation(sim_id=sim_id_1, serial_num=serial_num)
    time.sleep(0.5)

    with pytest.raises(ValueError):
        start_bks_simulation(sim_id=sim_id_2, serial_num=serial_num)

    assert stop_bks_simulation(sim_id=sim_id_1)


@skip_without_bks
def test_launcher_rejects_invalid_device_index(cleanup):
    sim_id = 10
    serial_num = "00000010"

    with pytest.raises(ValueError):
        start_bks_simulation(sim_id=sim_id, serial_num=serial_num, device_index=-1)

    with pytest.raises(ValueError):
        start_bks_simulation(sim_id=sim_id, serial_num=serial_num, device_index=100)

    assert stop_bks_simulation(sim_id=sim_id)


@skip_without_bks
def test_launcher_rejects_nonexistent_sim_id(cleanup):
    non_existent_sim_id = 200

    assert not stop_bks_simulation(sim_id=non_existent_sim_id)


@skip_without_bks
def test_launcher_rejects_nonexistent_log_sim(cleanup):
    non_existent_sim_id = 9999

    assert get_last_log(sim_id=non_existent_sim_id) is None
