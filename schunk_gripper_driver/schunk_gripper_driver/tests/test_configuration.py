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

from schunk_gripper_driver.driver import Driver, Gripper
from schunk_gripper_library.driver import Driver as GripperDriver
from schunk_gripper_library.utility import skip_without_gripper
import json
from pathlib import Path
from unittest import mock


def test_driver_offers_saving_gripper_configuration(ros2):
    driver = Driver("driver")

    # When empty
    driver.reset_grippers()
    assert not driver.save_configuration()

    # Non-writable locations
    invalid_locations = ["/", "/dev"]
    for location in invalid_locations:
        assert not driver.save_configuration(location=location), f"location: {location}"

    # When filled
    config = {
        "gripper_id": "not-empty",
        "host": "1.2.3.4",
        "port": 77,
        "serial_port": "abc",
        "device_id": 42,
    }
    driver.add_gripper(**config)
    assert driver.save_configuration()


@skip_without_gripper
def test_driver_offers_loading_previous_gripper_configuration(ros2):
    driver = Driver("driver")
    driver.reset_grippers()

    # Nothing previously saved
    assert not driver.load_previous_configuration(location="non-existent")
    assert len(driver.grippers) == 0

    # Configurations that have a non-empty `gripper_id` get
    # loaded without checking the connection
    config = {
        "gripper_id": "abc",
        "host": "1.2.3.4",
        "port": 77,
        "serial_port": "abc",
        "device_id": 42,
    }
    driver.add_gripper(**config)
    assert driver.save_configuration()
    driver.reset_grippers()

    assert driver.load_previous_configuration()
    assert len(driver.grippers) == 1
    for key, value in config.items():
        assert driver.grippers[0][key] == value

    # Configurations with empty `gripper_id` are being checked
    driver.reset_grippers()
    valid_config = {
        "gripper_id": "",
        "host": "0.0.0.0",
        "port": 8000,
        "serial_port": "",
        "device_id": 0,
    }
    driver.add_gripper(**valid_config)
    assert driver.save_configuration()
    driver.reset_grippers()
    assert driver.load_previous_configuration()


def test_driver_rejects_loading_invalid_configuration(ros2):
    driver = Driver("driver")
    driver.reset_grippers()

    invalid_configurations = [
        {"host": "", "port": 0, "serial_port": "", "device_id": 0},  # all empty
        {"host": "", "port": 0, "serial_port": ""},  # missing entry
        {"hots": "", "prot": 0, "sirial_port": "", "divice_id": 0},  # typos
        {
            "host": "",
            "port": "not-ok",
            "serial_port": "",
            "device_id": "not-ok",
        },  # wrong types
        {},  # completely empty
        [
            {"host": "0.0.0.0", "port": 8000, "serial_port": "", "device_id": 0}
        ],  # valid entries, but nested
    ]

    location = Path("/var/tmp/schunk_gripper")

    for config in invalid_configurations:
        with open(location.joinpath("configuration.json"), "w") as f:
            json.dump([config], f)

        assert not driver.load_previous_configuration(location=location)
        assert len(driver.grippers) == 0


def test_driver_rejects_loading_incorrectly_formatted_configuration(ros2):
    driver = Driver("driver")
    driver.reset_grippers()

    location = Path("/var/tmp/schunk_gripper")

    # With nonsense content
    with open(location.joinpath("configuration.json"), "w") as f:
        f.write("This should not crash the driver! #^$%&\0\n")

    assert not driver.load_previous_configuration(location=location)
    assert len(driver.grippers) == 0

    # Simulate permission errors
    with mock.patch("builtins.open", side_effect=PermissionError()):
        assert not driver.load_previous_configuration(location=location)
        assert len(driver.grippers) == 0


def test_driver_overwrites_grippers_when_loading_configuration(ros2):
    driver = Driver("driver")
    driver.get_logger().set_level(10)  # Debug

    # Save a defined configuration
    driver.reset_grippers()
    saved_config = {
        "gripper_id": "abc",
        "host": "0.0.0.0",
        "port": 1,
        "serial_port": "some",
        "device_id": 23,
    }
    driver.add_gripper(**saved_config)
    assert driver.save_configuration()
    driver.reset_grippers()

    # Add some grippers to overwrite
    device_ids = [1, 2, 3, 4, 5]
    for device in device_ids:
        config = {
            "gripper_id": "abc",
            "host": "",
            "port": 0,
            "serial_port": "abc",
            "device_id": device,
        }
        assert driver.add_gripper(**config)

    # We should have the one stored initially
    assert driver.load_previous_configuration()
    assert len(driver.grippers) == 1
    for key, value in saved_config.items():
        assert driver.grippers[0][key] == value


def test_driver_keeps_current_grippers_when_loading_invalid_configuration(ros2):
    driver = Driver("driver")

    # Store two identical grippers to obtain an invalid configuration file
    driver.reset_grippers()
    gripper = Gripper(
        {
            "gripper_id": "",
            "host": "a",
            "port": 0,
            "serial_port": "/",
            "device_id": 42,
            "driver": GripperDriver(),
        }
    )
    for _ in range(2):
        driver.grippers.append(gripper)
    assert driver.save_configuration()

    driver = Driver("driver")
    before = driver.grippers.copy()
    assert not driver.load_previous_configuration()
    assert driver.grippers == before

    # Store invalid parameters
    config = {"invalid_parameter": -1}
    location = Path("/var/tmp/schunk_gripper")
    with open(location.joinpath("configuration.json"), "w") as f:
        json.dump([config], f)

    driver = Driver("driver")
    before = driver.grippers.copy()
    assert not driver.load_previous_configuration()
    assert driver.grippers == before
