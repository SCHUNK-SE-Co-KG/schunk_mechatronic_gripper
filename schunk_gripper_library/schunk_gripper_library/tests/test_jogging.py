from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver
import time


@skip_without_gripper
def test_driver_offers_start_and_stop_jogging():

    driver = Driver()

    # When not connected
    assert not driver.start_jogging(velocity=-1000)
    assert not driver.stop_jogging()

    # Setup
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    max_pos = driver.module_parameters["max_pos"]
    min_pos = driver.module_parameters["min_pos"]
    max_vel = driver.module_parameters["max_vel"]
    middle = int(0.5 * (max_pos - min_pos))

    def reset() -> bool:
        return driver.acknowledge() and driver.move_to_absolute_position(
            position=middle, velocity=max_vel
        )

    # Positive jogging
    assert reset()
    assert driver.start_jogging(+max_vel)
    time.sleep(0.5)
    assert driver.get_actual_position() > middle
    assert driver.stop_jogging()

    # Negative jogging
    assert reset()
    assert driver.start_jogging(-max_vel)
    time.sleep(0.5)
    assert driver.get_actual_position() < middle
    assert driver.stop_jogging()

    # Cleanup
    driver.disconnect()
