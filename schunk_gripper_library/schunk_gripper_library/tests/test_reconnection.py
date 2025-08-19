from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
from .etc.events import lose_connection
import time


def test_driver_has_reconnection_interval():
    driver = Driver()
    assert isinstance(driver.reconnect_interval, float)
    assert driver.reconnect_interval > 0


@skip_without_gripper
def test_driver_automatically_reestablishes_connection():
    driver = Driver()
    assert driver.connect(host="0.0.0.0", port=8000)

    # Cut the connection to the gripper
    # and check that the driver reestablishes it automatically.
    downtime = 1.23
    assert lose_connection(duration_sec=downtime)
    time.sleep(0.1)
    assert not driver.connected

    time.sleep(downtime + driver.reconnect_interval)
    assert driver.connected

    driver.disconnect()
