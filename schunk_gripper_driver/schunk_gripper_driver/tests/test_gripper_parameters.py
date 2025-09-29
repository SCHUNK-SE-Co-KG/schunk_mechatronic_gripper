# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_driver.driver import Driver
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ReadGripperParameter,
)
from rclpy.node import Node
import rclpy
from pathlib import Path
import json
from unittest.mock import patch

# Setup a module-wide grippers configuration
# and let the driver automatically start into the _active_ state
headless = True
location = Path("/var/tmp/schunk_gripper")
location.mkdir(parents=True, exist_ok=True)
config = [
    {"gripper_id": "gripper_1", "host": "0.0.0.0", "port": 8000},
    {"gripper_id": "gripper_2", "serial_port": "/dev/ttyUSB0", "device_id": 12},
]
with open(location.joinpath("configuration.json"), "w") as f:
    json.dump(config, f, indent=2)


def test_driver_implements_callback_for_reading_gripper_parameters(ros2):
    driver = Driver("driver")

    req = ReadGripperParameter.Request()
    res = ReadGripperParameter.Response()

    # Check that we can call the interface
    for gripper in driver.grippers:
        assert driver._read_gripper_parameter_cb(
            request=req, response=res, gripper=gripper
        )

    # Check that it also works with the scheduler
    with patch.object(driver, "needs_synchronize", return_value=True):
        driver.scheduler.start()
        for gripper in driver.grippers:
            assert driver._read_gripper_parameter_cb(
                request=req, response=res, gripper=gripper
            )
        driver.scheduler.stop()


@skip_without_gripper
def test_driver_offers_reading_gripper_parameters(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_reading_gripper_parameters")
    for gripper in driver.list_grippers():

        client = node.create_client(
            ReadGripperParameter, f"/schunk/driver/{gripper}/read_parameter"
        )
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        def read(param: str) -> ReadGripperParameter.Response:
            req = ReadGripperParameter.Request()
            req.parameter = param
            future = client.call_async(request=req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=1)
            return future.result()

        # Wrong usage shouldn't crash the driver
        invalid_params = ["", "0", "0x0", "<actual_pos>"]
        for param in invalid_params:
            result = read(param)
            assert not result.success

        # Some valid parameters
        result = read("0x0500")  # enum
        assert result.success
        assert len(result.value_enum) >= 1

        result = read("0x0630")  # float
        assert result.success
        assert len(result.value_float) >= 1

        result = read("0x1000")  # char[16]
        assert result.success
        assert len(result.value_char) >= 1

        result = read("0x0380")  # uint16
        assert result.success
        assert len(result.value_uint16) >= 1

        result = read("0x1400")  # uint32
        assert result.success
        assert len(result.value_uint32) >= 1

        result = read("0x1330")  # bool
        assert result.success
        assert len(result.value_bool) >= 1

        result = read("0x03B8")  # float[6]
        assert result.success
        assert len(result.value_float) == 6
