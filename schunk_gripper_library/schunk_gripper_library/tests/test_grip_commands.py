from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper


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
