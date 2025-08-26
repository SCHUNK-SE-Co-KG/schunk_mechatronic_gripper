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
from schunk_gripper_library.utility import skip_without_gripper
from lifecycle_msgs.msg import State, Transition
from pathlib import Path
import json
from unittest.mock import patch, MagicMock
import time

# Module-wide settings
headless = True


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

    # Store a valid configuration for the next test
    config = [
        {"host": "0.0.0.0", "port": 8000},
        {"serial_port": "/dev/ttyUSB0", "device_id": 12},
    ]
    with open(location.joinpath("configuration.json"), "w") as f:
        json.dump(config, f)


@skip_without_gripper
def test_driver_auto_configures_in_headless_mode(lifecycle_interface):
    driver = lifecycle_interface

    # Give it some time to auto-transition
    time.sleep(1.0)

    assert driver.check_state(State.PRIMARY_STATE_ACTIVE)

    # Clean-up for next test
    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    # Note:
    # The default behavior (headless:=False) is implicitly covered
    # with all other tests outside this module.
