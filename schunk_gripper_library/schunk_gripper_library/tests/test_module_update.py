from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper, Scheduler
import time
import pytest
import threading


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
def test_driver_stores_specified_update_cycle():
    driver = Driver()

    # Store for successful connects
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        update_cycle = 0.1
        assert driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=update_cycle,
        )
        assert pytest.approx(driver.update_cycle) == update_cycle
        driver.disconnect()

    # Don't store when connect fails
    assert not driver.connect(update_cycle=42.123)
    assert pytest.approx(driver.update_cycle) == update_cycle  # from last connect


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


@skip_without_gripper
def test_driver_counts_module_updates():
    driver = Driver()
    assert driver.update_count == 0

    # Starts counting after successful connect
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=0.1,
        )
        driver.disconnect()

    time.sleep(1.0)
    assert driver.update_count > 0

    # Resets counter after new connect calls
    assert not driver.connect()
    assert driver.update_count == 0


@skip_without_gripper
def test_driver_updates_with_specified_cycle():
    driver = Driver()
    duration = 1.0

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        update_cycles = [0.1, 0.05, 0.01]
        for cycle in update_cycles:
            driver.connect(
                host=host,
                port=port,
                serial_port=serial_port,
                device_id=12,
                update_cycle=cycle,
            )

            time.sleep(duration)
            driver.disconnect()
            expected_count = int(duration / cycle)

            # Slightly more updates is ok here to compensate
            # for rounding errors with low cycles.
            assert (
                driver.update_count >= expected_count
            ), f"with update cycle: {cycle} on host: {host}"


def test_driver_offers_starting_module_updates():
    driver = Driver()
    threads_before = threading.active_count()

    # Normal usage
    assert driver.start_module_updates()
    assert threading.active_count() == threads_before + 1
    driver.stop_module_updates()
    assert threading.active_count() == threads_before

    # Normal lifecycle
    for _ in range(3):
        driver.start_module_updates()
        driver.stop_module_updates()
    assert threading.active_count() == threads_before

    # Repeated starts and stops
    for _ in range(3):
        driver.start_module_updates()
    assert threading.active_count() == threads_before + 1

    for _ in range(3):
        driver.stop_module_updates()
    assert threading.active_count() == threads_before


@skip_without_gripper
def test_module_updates_also_run_with_a_scheduler():
    driver = Driver()
    scheduler = Scheduler()
    scheduler.start()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)

        # With scheduler
        before = driver.update_count
        driver.start_module_updates(scheduler=scheduler)
        time.sleep(0.5)
        driver.stop_module_updates()
        assert driver.update_count > before
