from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver
import time


class Setup(object):
    def __init__(self, driver: Driver):
        self.driver = driver

    def reset(self) -> bool:
        if not self.driver.connected:
            return False
        max_pos = self.driver.module_parameters["max_pos"]
        min_pos = self.driver.module_parameters["min_pos"]
        self.max_vel = self.driver.module_parameters["max_vel"]
        self.middle = int(0.5 * (max_pos - min_pos))
        return self.driver.acknowledge() and self.driver.move_to_absolute_position(
            position=self.middle, velocity=self.max_vel
        )


@skip_without_gripper
def test_driver_offers_start_and_stop_jogging():
    driver = Driver()

    # When not connected
    assert not driver.start_jogging(velocity=-1000)
    assert not driver.stop_jogging()

    # Setup
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    setup = Setup(driver)

    # Positive jogging
    assert setup.reset()
    assert driver.start_jogging(+setup.max_vel)
    time.sleep(0.5)
    assert driver.get_actual_position() > setup.middle
    assert driver.stop_jogging()

    # Negative jogging
    assert setup.reset()
    assert driver.start_jogging(-setup.max_vel)
    time.sleep(0.5)
    assert driver.get_actual_position() < setup.middle
    assert driver.stop_jogging()

    # Cleanup
    driver.disconnect()
