from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver
import time


class Setup(object):
    def __init__(self, driver: Driver):
        self.driver = driver

    def reset(self) -> bool:
        if not self.driver.connected:
            return False
        assert self.driver.acknowledge()
        max_pos = self.driver.module_parameters["max_pos"]
        min_pos = self.driver.module_parameters["min_pos"]
        self.max_vel = self.driver.module_parameters["max_grp_vel"]
        middle = int(0.5 * (max_pos - min_pos))
        return self.driver.move_to_position(position=middle, velocity=self.max_vel)


@skip_without_gripper
def test_driver_offers_start_and_stop_jogging():
    driver = Driver()

    # We need the web dummy for now until the BKS firmware
    # really moves in simulation.

    # When not connected
    assert not driver.start_jogging(velocity=-1000)
    assert not driver.stop_jogging()

    # Setup
    assert driver.connect(host="0.0.0.0", port=8000)
    setup = Setup(driver)

    for use_gpe in [False, True]:

        # Positive jogging
        assert setup.reset()
        before = driver.get_actual_position()
        assert driver.start_jogging(setup.max_vel, use_gpe=use_gpe)
        time.sleep(0.5)
        assert driver.stop_jogging()
        assert driver.get_actual_position() > before

        # Negative jogging
        assert setup.reset()
        before = driver.get_actual_position()
        assert driver.start_jogging(-setup.max_vel, use_gpe=use_gpe)
        time.sleep(0.5)
        assert driver.stop_jogging()
        assert driver.get_actual_position() < before

    # Cleanup
    driver.disconnect()


@skip_without_gripper
def test_driver_allows_repeatedly_calling_jogging_methods():
    driver = Driver()

    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    setup = Setup(driver)
    assert setup.reset()
    velocity = driver.module_parameters["max_grp_vel"]

    # Repeated starting
    for run in range(3):
        assert driver.start_jogging(
            velocity=velocity
        ), f"run: {run}, diagnostics: {driver.get_status_diagnostics()}"
    driver.stop_jogging()

    # Calls to stop_jogging must be preceeded by start_jogging
    assert not driver.stop_jogging()

    driver.disconnect()


@skip_without_gripper
def test_driver_doesnt_start_jogging_with_warnings():

    driver = Driver()

    # Setup
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    setup = Setup(driver)
    assert setup.reset()
    velocity = driver.module_parameters["max_grp_vel"]
    invalid_velocity = velocity + 100

    # Check invalid velocity while stopped
    assert not driver.start_jogging(
        velocity=invalid_velocity
    ), f"diagnostics: {driver.get_status_diagnostics()}"

    # Check invalid velocity while already jogging
    assert driver.start_jogging(velocity=velocity)
    assert not driver.start_jogging(velocity=invalid_velocity)

    # Check after jogging into position limits
    almost_open = driver.module_parameters["max_pos"] - 1000
    full_speed = driver.module_parameters["max_vel"]
    assert driver.move_to_position(position=almost_open, velocity=full_speed)
    assert driver.start_jogging(velocity=velocity)
    time.sleep(2.0)
    assert not driver.start_jogging(
        velocity=velocity
    ), f"diagnostics: {driver.get_status_diagnostics()}"

    driver.disconnect()


@skip_without_gripper
def test_driver_toggles_status_bit_after_stop_jogging():
    driver = Driver()

    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    setup = Setup(driver)
    assert setup.reset()
    velocity = driver.module_parameters["max_grp_vel"]

    assert driver.start_jogging(velocity=velocity)
    time.sleep(0.5)
    before = driver.get_status_bit(bit=5)

    assert driver.stop_jogging()
    after = driver.get_status_bit(bit=5)
    assert before != after, f"before: {before}, after: {after}"

    driver.disconnect()
