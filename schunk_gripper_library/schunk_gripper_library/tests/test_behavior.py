from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper, Scheduler
import pytest


@skip_without_gripper
def test_acknowledge():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Not connected
        assert not driver.acknowledge()

        # Connected
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.acknowledge()

        # Repetitive
        for _ in range(5):
            assert driver.acknowledge()

        driver.disconnect()


@skip_without_gripper
def test_fast_stop():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):

        # Not connected
        assert not driver.fast_stop()

        # After fresh start
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.fast_stop()

        # From operational
        assert driver.acknowledge()
        assert driver.fast_stop()

        # Repetitive
        for _ in range(5):
            assert driver.fast_stop()

        driver.disconnect()


@skip_without_gripper
def test_all_gripper_commands_run_with_a_scheduler():
    driver = Driver()
    scheduler = Scheduler()
    scheduler.start()

    # Only relevant for Modbus
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)

    # Fast stop
    assert driver.fast_stop(scheduler=scheduler)

    # Acknowledge
    assert driver.acknowledge(scheduler=scheduler)

    # Move to absolute position
    max_pos = driver.module_parameters["max_pos"]
    min_pos = driver.module_parameters["min_pos"]
    half = int(0.5 * (max_pos - min_pos))
    max_vel = driver.module_parameters["max_vel"]
    assert driver.move_to_absolute_position(
        position=half, velocity=max_vel, scheduler=scheduler
    ), f"driver status: {driver.get_status_diagnostics()}"

    # Grip
    assert driver.acknowledge(scheduler=scheduler)
    assert not driver.grip(
        force=75, scheduler=scheduler
    ), f"driver status: {driver.get_status_diagnostics()}"

    # Release
    assert driver.acknowledge(scheduler=scheduler)
    assert not driver.release(
        scheduler=scheduler
    ), f"driver status: {driver.get_status_diagnostics()}"

    driver.disconnect()
    scheduler.stop()


@skip_without_gripper
def test_move_to_absolute_position_fails_with_invalid_arguments():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Invalid arguments
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        driver.acknowledge()
        combinations = [
            {"position": 0.1, "velocity": 1000},
            {"position": -0.1, "velocity": 1000},
            {"position": 1000, "velocity": -1.234},
            {"position": 1000, "velocity": -5005},
            {"position": 1000, "velocity": 177.33},
            {"position": 1000, "velocity": 0},
            {"position": 1000, "velocity": 0.0},
        ]
        for args in combinations:
            assert not driver.move_to_absolute_position(**args)
        driver.disconnect()


def test_move_to_absolute_position_fails_when_not_connected():
    driver = Driver()
    assert not driver.move_to_absolute_position(position=100, velocity=100)


@skip_without_gripper
def test_move_to_absolute_position_succeeds_with_valid_arguments():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Valid arguments
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        driver.acknowledge()
        max_pos = driver.module_parameters["max_pos"]
        min_pos = driver.module_parameters["min_pos"]
        half = int(0.5 * (max_pos - min_pos))
        max_vel = driver.module_parameters["max_vel"]
        combinations = [
            {"position": min_pos, "velocity": max_vel},
            {"position": max_pos, "velocity": max_vel},
            {"position": half, "velocity": max_vel},
        ]
        for args in combinations:
            assert driver.move_to_absolute_position(
                **args
            ), f"host: {host}, module status: {driver.get_status_diagnostics()}"
        driver.disconnect()


@skip_without_gripper
def test_move_to_absolute_position_uses_gpe_only_when_available():
    driver = Driver()
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    driver.acknowledge()

    # Test with _N_B gripper (without GPE).
    # The driver should survive use_gpe = True
    max_pos = driver.module_parameters["max_pos"]
    min_pos = driver.module_parameters["min_pos"]
    half = int(0.5 * (max_pos - min_pos))
    max_vel = driver.module_parameters["max_vel"]
    assert driver.move_to_absolute_position(
        position=half, velocity=max_vel, use_gpe=True
    )
    driver.disconnect()


@pytest.mark.skip
@skip_without_gripper
def test_move_to_relative_position():
    test_position = -50000
    test_velocity = 73000
    test_gpe = True

    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # not connected
        assert not driver.move_to_relative_position(
            position=test_position, velocity=test_velocity, use_gpe=test_gpe
        )

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert driver.acknowledge()
        assert driver.move_to_relative_position(
            position=test_position, velocity=test_velocity, use_gpe=test_gpe
        )

        assert driver.disconnect()


@pytest.mark.skip
@skip_without_gripper
def test_stop():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Not connected
        assert not driver.stop()

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert driver.acknowledge()

        assert driver.stop()
        assert driver.disconnect()


def test_grip_fails_when_not_connected():
    driver = Driver()
    assert not driver.grip(force=100)


@skip_without_gripper
def test_grip_fails_with_invalid_arguments():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Invalid arguments
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        driver.acknowledge()
        invalid_forces = [0.1, -0.1, 170, 75.0]
        for force in invalid_forces:
            assert not driver.grip(force)
        driver.disconnect()


@skip_without_gripper
def test_grip_works_with_valid_arguments():
    driver = Driver()

    # Only web dummy for now.
    # The BKS simulator will always fail.
    driver.connect(host="0.0.0.0", port=8000)
    driver.acknowledge()
    combinations = [
        {"force": 75, "outward": True},
        {"force": 55, "outward": False},
        {"force": 99, "outward": True},
    ]
    for args in combinations:
        assert driver.grip(**args)
    driver.disconnect()


@skip_without_gripper
def test_grip_moves_as_expected_with_the_outward_argument():
    driver = Driver()

    # Check that gripper's jaws moves in the right directions.
    driver.connect(host="0.0.0.0", port=8000)
    max_pos = driver.module_parameters["max_pos"]
    min_pos = driver.module_parameters["min_pos"]
    middle = int(0.5 * (max_pos - min_pos))

    assert driver.acknowledge()
    driver.grip(force=100, outward=True)
    assert driver.get_actual_position() > middle

    assert driver.acknowledge()
    driver.grip(force=100, outward=False)
    assert driver.get_actual_position() < middle

    driver.disconnect()


@skip_without_gripper
def test_grip_fails_when_no_workpiece_detected():
    driver = Driver()
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    driver.acknowledge()
    assert not driver.grip(force=75, outward=True)
    driver.disconnect()


def test_release_fails_when_not_connected():
    driver = Driver()
    assert not driver.release()


@skip_without_gripper
def test_release():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert driver.acknowledge()

        if serial_port:
            # Expected to fail
            assert not driver.release(use_gpe=False)
            assert not driver.release(use_gpe=True)
        else:
            assert driver.release(use_gpe=False)
            assert driver.release(use_gpe=True)

        assert driver.disconnect()
