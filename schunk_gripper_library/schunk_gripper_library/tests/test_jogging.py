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
        self.max_vel = self.driver.module_parameters["max_grp_vel"]
        middle = int(0.5 * (max_pos - min_pos))
        return self.driver.acknowledge() and self.driver.move_to_absolute_position(
            position=middle, velocity=self.max_vel
        )


@skip_without_gripper
def test_driver_offers_start_and_stop_jogging():
    driver = Driver()

    # When not connected
    assert not driver.start_jogging(velocity=-1000)
    assert not driver.stop_jogging()

    # Setup
    assert driver.connect(host="0.0.0.0", port=8000)
    setup = Setup(driver)

    # Positive jogging
    assert setup.reset()
    before = driver.get_actual_position()
    assert driver.start_jogging(setup.max_vel)
    time.sleep(0.5)
    assert driver.stop_jogging()
    assert driver.get_actual_position() > before

    # Negative jogging
    assert setup.reset()
    before = driver.get_actual_position()
    assert driver.start_jogging(-setup.max_vel)
    time.sleep(0.5)
    assert driver.stop_jogging()
    assert driver.get_actual_position() < before

    # Cleanup
    driver.disconnect()


@skip_without_gripper
def test_driver_allows_repeatedly_calling_jogging_methods():
    driver = Driver()

    assert driver.connect(host="0.0.0.0", port=8000)
    setup = Setup(driver)
    assert setup.reset()

    # Repeated starting
    for run in range(3):
        assert driver.start_jogging(
            setup.max_vel
        ), f"run: {run}, diagnostics: {driver.get_status_diagnostics()}"
    driver.stop_jogging()

    # Repeated stopping
    for _ in range(3):
        assert driver.stop_jogging()

    driver.disconnect()
