import pytest
from .etc.pseudo_terminals import Connection
from unittest.mock import patch
import httpx
import pymodbus
from .etc.modbus_server import ModbusServer
import asyncio
import threading
import time


@pytest.fixture
def time_scan():
    """
    Fixture to measure the execution time of the scanner.scan function.
    """

    def measure_scan_time(scanner, *args, **kwargs):
        start_time = time.time()
        result = asyncio.run(scanner.scan(*args, **kwargs))
        end_time = time.time()
        execution_time = end_time - start_time
        return result, execution_time

    return measure_scan_time


@pytest.fixture(scope="module")
def pseudo_terminals():
    connection = Connection()
    pt1, pt2 = connection.open()
    print(f"Opening two pseudo terminals:\n{pt1}\n{pt2}")

    yield (pt1, pt2)

    print("Closing both pseudo terminals")
    connection.close()


@pytest.fixture
def simulate_httpx_failure():
    controller = {"exception": None}
    pass_through = httpx.Client.get

    def side_effect(self, *args, **kwargs):
        if controller["exception"] is not None:
            raise controller["exception"]
        return pass_through(self, *args, **kwargs)

    patcher = patch("httpx.Client.get", new=side_effect)
    patcher.start()

    yield controller

    patcher.stop()


@pytest.fixture
def simulate_pymodbus_failure():
    controller = {"exception": None}
    pass_through = pymodbus.client.ModbusSerialClient.read_holding_registers

    def side_effect(self, *args, **kwargs):
        if controller["exception"] is not None:
            raise controller["exception"]
        return pass_through(self, *args, **kwargs)

    patcher = patch(
        "pymodbus.client.ModbusSerialClient.read_holding_registers", new=side_effect
    )
    patcher.start()

    yield controller

    patcher.stop()
