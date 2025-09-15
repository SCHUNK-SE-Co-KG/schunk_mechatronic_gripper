# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from schunk_gripper_driver.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ScanGrippers,
)
from unittest.mock import patch


def test_driver_has_an_ethernet_scanner(ros2):
    driver = Driver("driver")
    assert driver.ethernet_scanner is not None


@skip_without_gripper
def test_driver_scans_info_from_existing_ethernet_grippers(ros2):
    driver = Driver("driver")

    # Patch our scan method to return a connection to the web dummy
    # and check if we read the module type correctly
    with patch.object(
        driver.ethernet_scanner,
        "scan",
        return_value=[{"host": "0.0.0.0", "port": 8000}],
    ):

        request = ScanGrippers.Request()
        response = ScanGrippers.Response()
        driver._scan_grippers_cb(request=request, response=response)
        assert len(response.grippers) == 1
        assert len(response.connections) == 1

        assert response.grippers[0].startswith("EG")
        assert response.connections[0].host == "0.0.0.0"
        assert response.connections[0].port == 8000
