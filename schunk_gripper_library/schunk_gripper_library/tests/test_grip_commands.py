from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper


def test_grip_fails_when_not_connected():
    driver = Driver()
    assert driver.grip(force=100) == Driver.GripResult.ERROR


@skip_without_gripper
def test_grip_fails_with_invalid_arguments():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        # Invalid arguments
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        driver.acknowledge()
        invalid_forces = [0.1, -0.1, 75.0]
        for force in invalid_forces:
            assert driver.grip(force=force) == Driver.GripResult.ERROR

        driver.disconnect()


@skip_without_gripper
def test_grip_works_with_valid_arguments():
    driver = Driver()

    # Only web dummy for now.
    # The BKS simulator will always fail.
    driver.connect(host="0.0.0.0", port=8000)
    driver.acknowledge()
    combinations = [
        {"force": 175, "outward": True},
        {"force": 155, "outward": False},
        {"force": 199, "outward": True},
    ]
    for args in combinations:
        assert driver.grip(**args) != Driver.GripResult.ERROR
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
    assert driver.grip(force=100, outward=True) != Driver.GripResult.ERROR
    assert driver.get_actual_position() > middle

    assert driver.acknowledge()
    assert driver.grip(force=100, outward=False) != Driver.GripResult.ERROR
    assert driver.get_actual_position() < middle

    driver.disconnect()


@skip_without_gripper
def test_grip_at_position_fails_with_invalid_arguments():
    driver = Driver()

    driver.connect(host="0.0.0.0", port=8000, device_id=12)
    driver.acknowledge()

    invalid_positions = [-1000.0, 1e6, 3.5]
    invalid_forces = [0.0, -10.0]

    for pos in invalid_positions:
        for force in invalid_forces:
            assert driver.grip(force=force, position=pos) == Driver.GripResult.ERROR

    driver.disconnect()


@skip_without_gripper
def test_grip_with_velocity_fails_with_invalid_arguments():
    driver = Driver()
    driver.connect(host="0.0.0.0", port=8000, device_id=12)
    driver.acknowledge()

    invalid_velocities = [-5, 1e9, 5.0]
    forces = [50, -50]

    for vel in invalid_velocities:
        for force in forces:
            assert driver.grip(force=force, velocity=vel) == Driver.GripResult.ERROR

    driver.disconnect()


@skip_without_gripper
def test_grip_at_position_with_velocity_fails_with_invalid_arguments():
    driver = Driver()
    driver.connect(host="0.0.0.0", port=8000, device_id=12)
    driver.acknowledge()

    invalid_positions = [-1000, 1e6, 3.5]
    invalid_velocities = [-5, 1e9, 5.0]
    forces = [50, -50]

    for pos in invalid_positions:
        for vel in invalid_velocities:
            for force in forces:
                assert (
                    driver.grip(force=force, position=pos, velocity=vel)
                    == Driver.GripResult.ERROR
                )

    driver.disconnect()
