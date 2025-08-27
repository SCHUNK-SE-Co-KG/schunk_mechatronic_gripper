from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
import time
import pytest


@skip_without_gripper
def test_driver_runs_receiving_background_thread():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        assert not driver.polling_thread.is_alive()
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.polling_thread.is_alive()
        time.sleep(1)  # Let it run a little
        driver.disconnect()
        assert not driver.polling_thread.is_alive()


@skip_without_gripper
def test_driver_updates_with_specified_cycle():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        update_cycle = 0.1
        driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=update_cycle,
        )
        assert pytest.approx(driver.update_cycle) == update_cycle
        driver.disconnect()


@skip_without_gripper
def test_driver_skips_background_thread_without_update_cycle():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        for _ in range(3):
            as_before = driver.update_cycle
            driver.connect(
                host=host,
                port=port,
                serial_port=serial_port,
                device_id=12,
                update_cycle=None,
            )
            assert not driver.polling_thread.is_alive()
            assert pytest.approx(driver.update_cycle) == as_before
            driver.disconnect()
