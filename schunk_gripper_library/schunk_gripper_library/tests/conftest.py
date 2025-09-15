import pytest
from .etc.pseudo_terminals import Connection
from .etc.hms_chip import HMSChip
from unittest.mock import patch
import httpx
import pymodbus


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


@pytest.fixture(scope="function")
def ethernet_gripper():
    gripper = HMSChip()
    gripper.power_on()
    yield None
    gripper.power_off()
