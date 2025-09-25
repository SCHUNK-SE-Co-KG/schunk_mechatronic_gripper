from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
from .etc.events import lose_tcp_connection, lose_modbus_connection
import time
import httpx
import pymodbus


def test_driver_has_reconnection_interval():
    driver = Driver()
    assert isinstance(driver.reconnect_interval, float)
    assert driver.reconnect_interval > 0


@skip_without_gripper
def test_driver_automatically_reestablishes_tcp_connection():
    driver = Driver()
    assert driver.connect(host="0.0.0.0", port=8000)

    # Cut the connection to the gripper
    # and check that the driver reestablishes it automatically.
    downtime = 1.23
    assert lose_tcp_connection(duration_sec=downtime)
    time.sleep(0.1)
    assert not driver.connected

    time.sleep(downtime + driver.reconnect_interval)
    assert driver.connected

    driver.disconnect()


@skip_without_gripper
def test_driver_automatically_reestablishes_modbus_connection():
    driver = Driver()
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)

    downtime = 0.5
    assert lose_modbus_connection(duration_sec=downtime)

    # Modbus has several seconds default timeouts,
    # but we need to react fast to inform high-level callers about
    # the connection status
    time.sleep(0.5)
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


@skip_without_gripper
def test_driver_handles_pymodbus_exceptions_during_polling(simulate_pymodbus_failure):
    driver = Driver()
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=12)

    # Check that the driver automatically reconnects

    # Check pymodbus.exceptions.ModbusIOException
    simulate_pymodbus_failure["exception"] = pymodbus.exceptions.ModbusIOException(
        "Simulated IO error"
    )
    time.sleep(0.5)
    assert driver.polling_thread.is_alive()
    assert not driver.connected
    simulate_pymodbus_failure["exception"] = None
    time.sleep(driver.reconnect_interval)
    assert driver.connected

    # Check OSError when pulling the USB-RS485 cable
    simulate_pymodbus_failure["exception"] = OSError("Someone pulled the cable")
    time.sleep(0.5)
    assert driver.polling_thread.is_alive()
    assert not driver.connected
    simulate_pymodbus_failure["exception"] = None
    time.sleep(driver.reconnect_interval)
    assert driver.connected

    driver.disconnect()


@skip_without_gripper
def test_driver_updates_module_parameters_after_reconnection():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        assert driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=None,
        )
        fieldbus_before = driver.fieldbus
        module_type_before = driver.module_type
        gripper_type_before = driver.gripper_type
        parameters_before = driver.module_parameters

        # Mimic an interruption by clearing all relevant parameters.
        # The driver should read them anew on reconnects.
        driver.clear_module_parameters()
        driver.connected = False

        driver.start_module_updates()
        time.sleep(0.2)
        driver.stop_module_updates()
        assert driver.fieldbus == fieldbus_before
        assert driver.module_type == module_type_before
        assert driver.gripper_type == gripper_type_before
        assert driver.module_parameters == parameters_before

        driver.disconnect()
