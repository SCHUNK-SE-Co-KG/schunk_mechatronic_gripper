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
from schunk_gripper_driver.driver import Driver
from pathlib import Path
import json
from unittest.mock import patch, MagicMock


def test_driver_uses_previous_configuration_in_headless_mode(ros2):

    # Store a configuration that the driver should load
    location = Path("/var/tmp/schunk_gripper")
    config = [
        {"host": "1.2.3.4", "port": 1234},
        {"serial_port": "hello", "device_id": 42},
    ]
    with open(location.joinpath("configuration.json"), "w") as f:
        json.dump(config, f)

    # Patch the driver so that `headless` is True
    original = Driver.get_parameter

    def get_parameter(self, name):
        param = MagicMock()
        if name == "headless":
            param.value = True
            return param
        else:
            return original(self, name)

    # Check that the driver has the expected grippers
    with patch.object(Driver, "get_parameter", new=get_parameter):
        driver = Driver("driver")
        for idx, gripper in enumerate(config):
            assert set(gripper).issubset(set(driver.grippers[idx]))

    # Check that invalid configurations leave the driver empty
    invalid_config = [
        {"somebody screwed": "1.2.3.4", "port": 1234, "this up": True},
    ]
    with open(location.joinpath("configuration.json"), "w") as f:
        json.dump(invalid_config, f)

    with patch.object(Driver, "get_parameter", new=get_parameter):
        driver = Driver("driver")
        assert len(driver.grippers) == 0
