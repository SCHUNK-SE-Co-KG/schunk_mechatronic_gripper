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


@skip_without_bks
def test_start_and_stop_multiple_simulations(cleanup):
    scanner = Scanner()

    max_simulations = 3
    simulations = []

    for i in range(max_simulations):
        sim_id = 20 + i
        # Use valid format: 4 letters (no AEIOU) + 4 digits
        serial_num = f"BCGH{sim_id:04d}"
        assert start_bks_simulation(
            sim_id=sim_id, serial_num=serial_num, device_index=i + 1
        )

        time.sleep(0.5)  # Wait for simulation to stabilize

        # Since alphanumeric serials get converted to hash values and returned as hex,
        # we need to expect the hex representation of the hash
        expected_hash = hash(serial_num.upper()) & 0xFFFFFFFF
        expected_hex = f"{expected_hash:08X}"
        assert scanner.get_serial_number(dev_id=sim_id) == expected_hex

        simulations.append(sim_id)
        time.sleep(1)  # Additional delay before next simulation

    for sim_id in simulations:
        assert stop_bks_simulation(sim_id)


@skip_without_bks
def test_helper_rejects_invalid_serial_number():
    invalid_serials = [
        "",
        "123456789",
        "AEIOU123",
        "BCDF12AB",
        "bcdf1234",
        "BCDF",
        "GGGGGGGG",
    ]

    for serial in invalid_serials:
        assert not start_bks_simulation(sim_id=10, serial_num=serial)


@skip_without_bks
def test_launcher_rejects_duplicate_sim_ids(cleanup):
    sim_id = 10
    serial_num = "BCDX1234"

    start_bks_simulation(sim_id=sim_id, serial_num=serial_num)
    time.sleep(0.5)

    assert not start_bks_simulation(sim_id=sim_id, serial_num="BCDF5678")

    assert stop_bks_simulation(sim_id=sim_id)


@skip_without_bks
def test_launcher_rejects_duplicate_serial_numbers(cleanup):
    sim_id_1 = 10
    sim_id_2 = 11
    serial_num = "BCDX9999"

    assert start_bks_simulation(sim_id=sim_id_1, serial_num=serial_num)
    time.sleep(0.5)

    assert not start_bks_simulation(sim_id=sim_id_2, serial_num=serial_num)

    assert stop_bks_simulation(sim_id=sim_id_1)


@skip_without_bks
def test_launcher_rejects_invalid_device_index(cleanup):
    sim_id = 10
    serial_num = "BCDX1010"

    assert not start_bks_simulation(
        sim_id=sim_id, serial_num=serial_num, device_index=-1
    )

    assert not start_bks_simulation(
        sim_id=sim_id, serial_num=serial_num, device_index=100
    )


@skip_without_bks
def test_launcher_rejects_nonexistent_sim_id(cleanup):
    non_existent_sim_id = 200

    assert not stop_bks_simulation(sim_id=non_existent_sim_id)


@skip_without_bks
def test_launcher_rejects_nonexistent_log_sim(cleanup):
    non_existent_sim_id = 9999

    assert get_last_log(sim_id=non_existent_sim_id) is None
