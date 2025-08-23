from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
from .etc.events import lose_connection
import time
import httpx
import pymodbus


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


@skip_without_gripper
def test_driver_handles_httpx_exceptions_on_startup(simulate_httpx_failure):

    # Check httpx.ConnectError
    simulate_httpx_failure["exception"] = httpx.ConnectError("Simulated connect error")
    driver = Driver()
    assert not driver.connect(host="0.0.0.0", port=8000)
    driver.disconnect()

    # Check httpx.ConnectTimeout
    simulate_httpx_failure["exception"] = httpx.ConnectTimeout(
        "Simulated connect timeout"
    )
    driver = Driver()
    assert not driver.connect(host="0.0.0.0", port=8000)
    driver.disconnect()


@skip_without_gripper
def test_driver_handles_httpx_exceptions_during_polling(simulate_httpx_failure):
    driver = Driver()
    assert driver.connect(host="0.0.0.0", port=8000)

    # Simulate different httpx exceptions
    # and check that the driver automatically reconnects

    # Check httpx.ReadTimeout
    simulate_httpx_failure["exception"] = httpx.ReadTimeout("Simulated read timeout")
    time.sleep(0.5)
    assert driver.polling_thread.is_alive()
    assert not driver.connected
    simulate_httpx_failure["exception"] = None
    time.sleep(driver.reconnect_interval)
    assert driver.connected

    # Check httpx.ConnectError
    simulate_httpx_failure["exception"] = httpx.ConnectError("Simulated connect error")
    time.sleep(0.5)
    assert driver.polling_thread.is_alive()
    assert not driver.connected
    simulate_httpx_failure["exception"] = None
    time.sleep(driver.reconnect_interval)
    assert driver.connected

    # Check httpx.ConnectTimeout
    simulate_httpx_failure["exception"] = httpx.ConnectTimeout(
        "Simulated connect timeout"
    )
    time.sleep(0.5)
    assert driver.polling_thread.is_alive()
    assert not driver.connected
    simulate_httpx_failure["exception"] = None
    time.sleep(driver.reconnect_interval)
    assert driver.connected

    driver.disconnect()


@skip_without_gripper
def test_driver_handles_pymodbus_exceptions_on_startup(simulate_pymodbus_failure):

    # Check pymodbus.exceptions.ModbusIOException
    simulate_pymodbus_failure["exception"] = pymodbus.exceptions.ModbusIOException(
        "Simulated IO error"
    )
    driver = Driver()
    assert not driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    driver.disconnect()
