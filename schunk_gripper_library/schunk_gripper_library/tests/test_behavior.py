from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper, Scheduler
import time


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
    assert driver.move_to_position(
        position=half, velocity=max_vel, scheduler=scheduler
    ), f"driver status: {driver.get_status_diagnostics()}"

    # Grip
    assert driver.acknowledge(scheduler=scheduler)
    grip_result = (
        driver.grip(force=75, scheduler=scheduler),
        f"driver status: {driver.get_status_diagnostics()}",
    )
    assert grip_result == Driver.GripResult.ERROR

    # Release
    assert driver.acknowledge(scheduler=scheduler)
    assert not driver.release(
        scheduler=scheduler
    ), f"driver status: {driver.get_status_diagnostics()}"

    # Jogging
    assert driver.acknowledge(scheduler=scheduler)
    assert driver.start_jogging(
        velocity=driver.module_parameters["max_grp_vel"], scheduler=scheduler
    )
    assert driver.stop_jogging(scheduler=scheduler)

    # Twitching the jaws
    assert driver.acknowledge(scheduler=scheduler)
    assert driver.twitch_jaws(scheduler=scheduler)

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
            assert not driver.move_to_position(**args)
        driver.disconnect()


def test_move_to_absolute_position_fails_when_not_connected():
    driver = Driver()
    assert not driver.move_to_position(position=100, velocity=100)


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
            assert driver.move_to_position(
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
    assert driver.move_to_position(position=half, velocity=max_vel, use_gpe=True)
    driver.disconnect()


@skip_without_gripper
def test_move_to_relative_position():
    test_position = -5000
    test_velocity = 73000
    test_gpe = True

    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # not connected
        assert not driver.move_to_position(
            position=test_position,
            velocity=test_velocity,
            use_gpe=test_gpe,
            is_absolute=False,
        )

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert driver.acknowledge()
        assert driver.move_to_position(
            position=test_position,
            velocity=test_velocity,
            use_gpe=test_gpe,
            is_absolute=False,
        )

        assert driver.disconnect()


@skip_without_gripper
def test_move_to_relative_position_fails_with_invalid_arguments():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        driver.acknowledge()
        invalid_positions = [0.1, -0.5, "1000", None, 1e12]
        invalid_velocities = [-5, 1e9, 5.0]
        for pos in invalid_positions:
            for vel in invalid_velocities:
                assert not driver.move_to_position(
                    position=pos, velocity=vel, is_absolute=False
                )
        driver.disconnect()


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

        assert driver.stop(use_gpe=False)
        assert driver.stop(use_gpe=True)
        assert driver.disconnect()


@skip_without_gripper
def test_prepare_for_shutdown_and_soft_reset():
    driver = Driver()
    # The modbus simulator does not simulate soft reset properly,
    # so we run this test only with the web dummy.
    for host, port, serial_port in zip(["0.0.0.0"], [8000], [None]):
        # Not connected
        assert not driver.prepare_for_shutdown()

        # after connection
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert driver.acknowledge()
        assert driver.prepare_for_shutdown()
        assert driver.soft_reset()

        assert driver.disconnect()


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


@skip_without_gripper
def test_release_for_manual_movement():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert driver.acknowledge()

        # this must fail, because release for manual movement is only allowed by
        # the firmware if the gripper is in an error state
        assert not driver.release_for_manual_movement()

        # bring the gripper into an error state
        driver.fast_stop()

        # now the command shall succeed
        assert driver.release_for_manual_movement()

        assert driver.disconnect()


@skip_without_gripper
def test_brake_test():
    driver = Driver()

    # TCP/IP
    assert driver.connect(host="0.0.0.0", port=8000)
    assert driver.acknowledge()
    assert driver.brake_test()
    driver.disconnect()

    # Modbus
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    assert driver.acknowledge()

    # Check that we give sufficient time for real modules to carry out the brake test.
    # We measured this on the real modules.
    start = time.time()
    driver.brake_test()  # fails in simulation
    duration = time.time() - start
    assert duration > 4.0  # sec

    assert driver.disconnect()


@skip_without_gripper
def test_twitching_jaws_leaves_gripper_where_it_is():
    driver = Driver()

    # When not connected
    assert not driver.twitch_jaws()

    # Only web dummy for now.
    # The BKS firmware has inconsistent accuracy for motion commands
    assert driver.connect(host="0.0.0.0", port=8000, update_cycle=None)
    driver.acknowledge()

    max_pos = driver.module_parameters["max_pos"]
    min_pos = driver.module_parameters["min_pos"]
    half = int(0.5 * (max_pos - min_pos))
    driver.move_to_position(
        position=half, velocity=driver.module_parameters["max_vel"], is_absolute=True
    )

    before = driver.get_actual_position()
    assert driver.twitch_jaws()
    after = driver.get_actual_position()
    assert after == before

    driver.disconnect()


@skip_without_gripper
def test_twitching_jaws_considers_limits():
    driver = Driver()

    # Only on Modbus.
    # Whe need to check with realistic limits
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    driver.acknowledge()

    # When fully open
    driver.move_to_position(
        position=driver.module_parameters["max_pos"],
        velocity=driver.module_parameters["max_vel"],
        is_absolute=True,
    )
    assert driver.twitch_jaws()

    # When fully closed
    driver.move_to_position(
        position=driver.module_parameters["min_pos"],
        velocity=driver.module_parameters["max_vel"],
        is_absolute=True,
    )
    assert driver.twitch_jaws()

    driver.disconnect()
