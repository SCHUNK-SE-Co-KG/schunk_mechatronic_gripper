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

    for gripper in driver.grippers:
        req = ReadGripperParameter.Request()
        res = ReadGripperParameter.Response()
        assert driver._read_gripper_parameter_cb(
            request=req, response=res, gripper=gripper
        )


@skip_without_gripper
def test_driver_offers_reading_gripper_parameters(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_reading_gripper_parameters")
    for gripper in driver.list_grippers():

        client = node.create_client(
            ReadGripperParameter, f"/schunk/driver/{gripper}/read_parameter"
        )
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        req = ReadGripperParameter.Request()
        req.parameter = "0x0380"  # uint16 return type
        future = client.call_async(request=req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1)

        assert future.result()
        assert future.result().success
        assert future.result().value_uint16[0] == 42
