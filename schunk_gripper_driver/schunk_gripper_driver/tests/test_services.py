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
from lifecycle_msgs.msg import Transition, State
from std_srvs.srv import Trigger
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ListGrippers,
    AddGripper,
    MoveToAbsolutePosition,
    MoveToAbsolutePositionGPE,
    Grip,
    GripGPE,
    GripAtPosition,
    GripAtPositionGPE,
    SoftGrip,
    SoftGripGPE,
    SoftGripAtPosition,
    SoftGripAtPositionGPE,
    StrongGripGPE,
    Release,
    ShowConfiguration,
    ShowGripperSpecification,
)
from schunk_gripper_interfaces.msg import (  # type: ignore [attr-defined]
    Gripper as GripperConfig,
)
from rclpy.node import Node
import rclpy
import time
from pathlib import Path
import json
import pytest


@skip_without_gripper
def test_driver_advertises_state_depending_services(lifecycle_interface):
    driver = lifecycle_interface
    list_grippers = ["/schunk/driver/list_grippers"]
    config_services = [
        "/schunk/driver/add_gripper",
        "/schunk/driver/reset_grippers",
        "/schunk/driver/show_configuration",
        "/schunk/driver/save_configuration",
        "/schunk/driver/load_previous_configuration",
    ]
    gripper_services = [
        "acknowledge",
        "fast_stop",
        "move_to_absolute_position",
        "grip",
        "release",
        "show_specification",
        "start_jogging",
        "stop_jogging",
    ]
    until_change_takes_effect = 0.1

    for run in range(3):

        # After startup -> unconfigured
        driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)
        assert driver.check(config_services, dtype="service", should_exist=True)
        assert driver.check(list_grippers, dtype="service", should_exist=False)

        # After configure -> inactive
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(until_change_takes_effect)
        assert driver.check(list_grippers, dtype="service", should_exist=True)
        assert driver.check(config_services, dtype="service", should_exist=False)
        assert driver.check(gripper_services, dtype="service", should_exist=False)

        # After activate -> active
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.check(list_grippers, dtype="service", should_exist=True)
        assert driver.check(config_services, dtype="service", should_exist=False)
        assert driver.check(gripper_services, dtype="service", should_exist=True)

        # After deactivate -> inactive
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.check(list_grippers, dtype="service", should_exist=True)
        assert driver.check(config_services, dtype="service", should_exist=False)
        assert driver.check(gripper_services, dtype="service", should_exist=False)

        # After cleanup -> unconfigured
        driver.change_state(Transition.TRANSITION_CLEANUP)
        time.sleep(until_change_takes_effect)
        assert driver.check(config_services, dtype="service", should_exist=True)
        assert driver.check(list_grippers, dtype="service", should_exist=False)


@skip_without_gripper
def test_driver_implements_list_grippers(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)

    node = Node("check_list_grippers")
    client = node.create_client(ListGrippers, "/schunk/driver/list_grippers")
    assert client.wait_for_service(timeout_sec=2)
    future = client.call_async(ListGrippers.Request())
    rclpy.spin_until_future_complete(node, future)
    assert len(future.result().grippers) >= 1
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_adding_and_resetting_grippers(driver):
    node = Node("check_adding_and_resetting_grippers")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Empty request
    request = AddGripper.Request()
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert not future.result().success

    for _ in range(3):

        # Reset gripper list
        request = Trigger.Request()
        future = reset_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success

        # Add Modbus gripper
        request = AddGripper.Request()
        request.gripper.serial_port = "/dev/ttyUSB0"
        request.gripper.device_id = 12
        future = add_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success

        # Add TCP/IP gripper
        request = AddGripper.Request()
        request.gripper.host = "0.0.0.0"
        request.gripper.port = 8000
        future = add_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success


def test_driver_implements_show_configuration(driver):
    node = Node("check_listing_configuration")
    client = node.create_client(ShowConfiguration, "/schunk/driver/show_configuration")
    assert client.wait_for_service(timeout_sec=2)
    future = client.call_async(ShowConfiguration.Request())
    rclpy.spin_until_future_complete(node, future)
    configuration = future.result().configuration
    assert len(configuration) > 0
    assert isinstance(configuration[0], GripperConfig)


@skip_without_gripper
def test_driver_implements_acknowledge(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("check_acknowledge")
    for gripper in driver.list_grippers():
        client = node.create_client(Trigger, f"/schunk/driver/{gripper}/acknowledge")
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1)

        assert future.result().success
        expected_msg = (
            "error_code: 0x0, warning_code: 0x0, additional_code: 0x0"  # everything ok
        )
        assert future.result().message == expected_msg

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_fast_stop(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("check_fast_stop")
    for gripper in driver.list_grippers():
        client = node.create_client(Trigger, f"/schunk/driver/{gripper}/fast_stop")
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1)

        assert future.result().success

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@pytest.mark.skip()
@skip_without_gripper
def test_driver_implements_move_to_absolute_position(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    assert driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("check_move_to_absolute_position")
    for gripper in driver.list_grippers():
        client = node.create_client(
            MoveToAbsolutePosition,
            f"/schunk/driver/{gripper}/move_to_absolute_position",
        )
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        targets = [
            {"position": 0.023, "velocity": 0.02, "use_gpe": False},
            {"position": 0.005, "velocity": 0.02, "use_gpe": True},
        ]
        for target in targets:
            request = MoveToAbsolutePosition.Request()
            request.position = target["position"]
            request.velocity = target["velocity"]
            request.use_gpe = target["use_gpe"]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_grip_and_release(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_grip")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Drop default modbus gripper because that can't grip in simulation.
    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Get the gripper's services
    for gripper in driver.list_grippers():
        try:
            grip_client = node.create_client(GripGPE, f"/schunk/driver/{gripper}/grip")
        except (TypeError, NameError):
            grip_client = node.create_client(Grip, f"/schunk/driver/{gripper}/grip")
        assert grip_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        release_client = node.create_client(
            Release,
            f"/schunk/driver/{gripper}/release",
        )
        assert release_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        targets = [
            {"force": 50, "use_gpe": False, "outward": False},
            {"force": 100, "use_gpe": True, "outward": False},
            {"force": 75, "use_gpe": False, "outward": True},
            {"force": 88, "use_gpe": True, "outward": True},
        ]
        for target in targets:

            # Grip
            request = grip_client.srv_type.Request()
            request.force = target["force"]
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]
            request.outward = target["outward"]
            future = grip_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

            # Release
            future = release_client.call_async(Release.Request())
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_show_specification(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("show_specification")
    for gripper in driver.list_grippers():
        client = node.create_client(
            ShowGripperSpecification,
            f"/schunk/driver/{gripper}/show_specification",
        )
        assert client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        future = client.call_async(ShowGripperSpecification.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1)

        assert future.result().success

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_saving_configuration(driver):
    node = Node("check_save_configuration")
    client = node.create_client(Trigger, "/schunk/driver/save_configuration")
    assert client.wait_for_service(timeout_sec=2)

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success


@skip_without_gripper
def test_driver_implements_loading_previous_configuration(driver):

    node = Node("check_load_previous_configuration")
    client = node.create_client(Trigger, "/schunk/driver/load_previous_configuration")
    assert client.wait_for_service(timeout_sec=2)

    # Store a valid configuration
    config = {"host": "0.0.0.0", "port": 80}
    location = Path("/var/tmp/schunk_gripper")
    with open(location.joinpath("configuration.json"), "w") as f:
        json.dump([config], f)
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success


@skip_without_gripper
def test_driver_implements_start_and_stop_jogging(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("start_jogging")
    # Drop default modbus gripper because jogging is broken there

    # Get the gripper's services
    for gripper in driver.list_grippers():
        StartJogging = driver.get_service_type(
            f"/schunk/driver/{gripper}/start_jogging"
        )
        start_jogging_client = node.create_client(
            StartJogging,
            f"/schunk/driver/{gripper}/start_jogging",
        )
        assert start_jogging_client.wait_for_service(
            timeout_sec=2
        ), f"gripper: {gripper}"
        stop_jogging_client = node.create_client(
            Trigger,
            f"/schunk/driver/{gripper}/stop_jogging",
        )
        assert stop_jogging_client.wait_for_service(
            timeout_sec=2
        ), f"gripper: {gripper}"

        targets = [
            {"velocity": 0.010, "use_gpe": False},
            {"velocity": 0.0100, "use_gpe": True},
            {"velocity": 0.075, "use_gpe": False},
            {"velocity": 0.08, "use_gpe": True},
        ]
        for target in targets:

            # Start
            request = StartJogging.Request()
            request.use_gpe = target["use_gpe"]
            future = start_jogging_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}, for {target}"

            # stop
            future = stop_jogging_client.call_async(Trigger.Request())


def test_driver_implements_grip_at_position_and_release(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_grip_at_position")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Reset grippers (drop default modbus gripper)
    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Get gripper services
    for gripper in driver.list_grippers():
        try:
            grip_client = node.create_client(
                GripAtPositionGPE, f"/schunk/driver/{gripper}/grip_at_position"
            )
        except (TypeError, NameError):
            grip_client = node.create_client(
                GripAtPosition, f"/schunk/driver/{gripper}/grip_at_position"
            )
        assert grip_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        release_client = node.create_client(
            Release, f"/schunk/driver/{gripper}/release"
        )
        assert release_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        targets = [
            {"force": 50, "position": 10000, "use_gpe": False, "outward": False},
            {"force": 100, "position": 20000, "use_gpe": True, "outward": False},
            {"force": 75, "position": 15000, "use_gpe": False, "outward": True},
            {"force": 88, "position": 12000, "use_gpe": True, "outward": True},
        ]

        for target in targets:
            # Grip at position
            request = grip_client.srv_type.Request()
            request.force = target["force"]
            request.at_position = target["position"]
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]
            request.outward = target["outward"]

            future = grip_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

            # Release
            future = release_client.call_async(Release.Request())
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    node.destroy_node()


@skip_without_gripper
def test_driver_implements_soft_grip_and_release(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_soft_grip")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Reset grippers (drop default modbus gripper)
    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Get gripper services
    for gripper in driver.list_grippers():
        if gripper.startswith("EGU") or gripper.startswith("EZU"):
            driver.change_state(Transition.TRANSITION_DEACTIVATE)
            driver.change_state(Transition.TRANSITION_CLEANUP)
            node.destroy_node()
            pytest.skip()
        try:
            grip_client = node.create_client(
                SoftGripGPE, f"/schunk/driver/{gripper}/soft_grip"
            )
        except (TypeError, NameError):
            grip_client = node.create_client(
                SoftGrip, f"/schunk/driver/{gripper}/soft_grip"
            )
        assert grip_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        release_client = node.create_client(
            Release, f"/schunk/driver/{gripper}/release"
        )
        assert release_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        targets = [
            {"force": 50, "velocity": 10000, "use_gpe": False, "outward": False},
            {"force": 100, "velocity": 11000, "use_gpe": True, "outward": False},
            {"force": 75, "velocity": 11000, "use_gpe": False, "outward": True},
            {"force": 88, "velocity": 12000, "use_gpe": True, "outward": True},
        ]

        for target in targets:
            # Soft Grip
            request = grip_client.srv_type.Request()
            request.force = target["force"]
            request.velocity = target["velocity"]
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]
            request.outward = target["outward"]

            future = grip_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

            # Release
            future = release_client.call_async(Release.Request())
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_soft_grip_at_position_and_release(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_soft_grip_at_position")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Reset grippers (drop default modbus gripper)
    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Get gripper services
    for gripper in driver.list_grippers():
        if gripper.startswith("EGU") or gripper.startswith("EZU"):
            driver.change_state(Transition.TRANSITION_DEACTIVATE)
            driver.change_state(Transition.TRANSITION_CLEANUP)
            node.destroy_node()
            pytest.skip()
        try:
            grip_client = node.create_client(
                SoftGripAtPositionGPE, f"/schunk/driver/{gripper}/soft_grip_at_position"
            )
        except (TypeError, NameError):
            grip_client = node.create_client(
                SoftGripAtPosition, f"/schunk/driver/{gripper}/soft_grip_at_position"
            )
        assert grip_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        release_client = node.create_client(
            Release, f"/schunk/driver/{gripper}/release"
        )
        assert release_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        targets = [
            {
                "force": 50,
                "position": 10000,
                "velocity": 10000,
                "use_gpe": False,
                "outward": False,
            },
            {
                "force": 100,
                "position": 20000,
                "velocity": 11000,
                "use_gpe": True,
                "outward": False,
            },
            {
                "force": 75,
                "position": 15000,
                "velocity": 11000,
                "use_gpe": False,
                "outward": True,
            },
            {
                "force": 88,
                "position": 12000,
                "velocity": 12000,
                "use_gpe": True,
                "outward": True,
            },
        ]

        for target in targets:
            # Soft Grip at position
            request = grip_client.srv_type.Request()
            request.force = target["force"]
            request.velocity = target["velocity"]
            request.at_position = target["position"]
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]
            request.outward = target["outward"]

            future = grip_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

            # Release
            future = release_client.call_async(Release.Request())
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_strong_grip_and_release(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_strong_grip")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Reset grippers
    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Only run for grippers that support StrongGrip
    for gripper in driver.list_grippers():
        if gripper.startswith("EGK"):
            driver.change_state(Transition.TRANSITION_DEACTIVATE)
            driver.change_state(Transition.TRANSITION_CLEANUP)
            node.destroy_node()
            pytest.skip()
        grip_client = node.create_client(
            StrongGripGPE, f"/schunk/driver/{gripper}/strong_grip"
        )
        assert grip_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        release_client = node.create_client(
            Release, f"/schunk/driver/{gripper}/release"
        )
        assert release_client.wait_for_service(timeout_sec=2), f"gripper: {gripper}"

        # StrongGrip test targets
        targets = [
            {"force": 110, "use_gpe": True, "outward": False},
            {"force": 130, "use_gpe": True, "outward": False},
            {"force": 150, "use_gpe": True, "outward": True},
            {"force": 200, "use_gpe": True, "outward": True},
        ]

        for target in targets:
            request = grip_client.srv_type.Request()
            request.force = target["force"]
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]
            request.outward = target["outward"]

            future = grip_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

            # Release after StrongGrip
            future = release_client.call_async(Release.Request())
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_move_to_absolute_position_with_or_without_gpe(
    lifecycle_interface,
):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("move_to_absolute_position_service_with_or_without_gpe")

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/move_to_absolute_position"

        client = node.create_client(MoveToAbsolutePositionGPE, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            gpe_supported = True
        else:
            client.destroy()
            client = node.create_client(MoveToAbsolutePosition, service_name)
            assert client.wait_for_service(
                timeout_sec=2
            ), f"{gripper}: service unavailable"
            gpe_supported = False

        if gpe_supported:
            req = MoveToAbsolutePositionGPE.Request()
            print(MoveToAbsolutePositionGPE.Request())
            req.position = 0.01
            req.velocity = 0.01
            assert hasattr(req, "use_gpe"), f"{gripper}: use_gpe expected but not found"
            req.use_gpe = False
        else:
            req = MoveToAbsolutePosition.Request()
            print(MoveToAbsolutePosition.Request())
            req.position = 0.01
            req.velocity = 0.01
            assert not hasattr(
                req, "use_gpe"
            ), f"{gripper}: use_gpe not expected but found"

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.result() is not None, f"{gripper}: no response from service"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    node.destroy_node()


@skip_without_gripper
def test_driver_implements_grip_with_or_without_gpe(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("grip_service_with_or_without_gpe")

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/grip"

        client = node.create_client(GripGPE, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            gpe_supported = True
        else:
            client.destroy()
            client = node.create_client(Grip, service_name)
            assert client.wait_for_service(
                timeout_sec=2
            ), f"{gripper}: service unavailable"
            gpe_supported = False

        if gpe_supported:
            req = GripGPE.Request()
            print(GripGPE.Request())
            req.force = 10
            req.outward = False
            assert hasattr(req, "use_gpe"), f"{gripper}: use_gpe expected but not found"
            req.use_gpe = False
        else:
            req = Grip.Request()
            print(Grip.Request())
            req.force = 10
            req.outward = False
            assert not hasattr(
                req, "use_gpe"
            ), f"{gripper}: use_gpe not expected but found"

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.result() is not None, f"{gripper}: no response from service"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    node.destroy_node()


@skip_without_gripper
def test_driver_implements_grip_at_position_with_or_without_gpe(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("grip_at_position_service_with_or_without_gpe")

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/grip_at_position"

        client = node.create_client(GripAtPositionGPE, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            gpe_supported = True
        else:
            client.destroy()
            client = node.create_client(GripAtPosition, service_name)
            assert client.wait_for_service(
                timeout_sec=2
            ), f"{gripper}: service unavailable"
            gpe_supported = False

        if gpe_supported:
            req = GripAtPositionGPE.Request()
            print(GripAtPositionGPE.Request())
            req.force = 10
            req.outward = False
            req.at_position = 50
        else:
            req = GripAtPosition.Request()
            print(GripAtPosition.Request())
            req.force = 10
            req.outward = False
            req.at_position = 50

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.result() is not None, f"{gripper}: no response from service"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    node.destroy_node()


@skip_without_gripper
def test_driver_implements_soft_grip_with_or_without_gpe(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    node = Node("soft_grip_service_with_or_without_gpe")

    for gripper in driver.list_grippers():
        if gripper.startswith("EGU") or gripper.startswith("EZU"):
            driver.change_state(Transition.TRANSITION_DEACTIVATE)
            driver.change_state(Transition.TRANSITION_CLEANUP)
            node.destroy_node()
            pytest.skip()
        service_name = f"/schunk/driver/{gripper}/soft_grip"

        # Try GPE variant first
        client = node.create_client(SoftGripGPE, service_name)
        if client.wait_for_service(timeout_sec=1.0):
            gpe_supported = True
        else:
            client.destroy()
            client = node.create_client(SoftGrip, service_name)
            assert client.wait_for_service(
                timeout_sec=2
            ), f"{gripper}: service unavailable"
            gpe_supported = False

        # Prepare request
        if gpe_supported:
            req = SoftGripGPE.Request()
            print(SoftGripGPE.Request())
            req.force = 20
            req.velocity = 10000
            req.outward = False
            assert hasattr(req, "use_gpe"), f"{gripper}: use_gpe expected but not found"
            req.use_gpe = False
        else:
            req = SoftGrip.Request()
            print(SoftGrip.Request())
            req.force = 20
            req.velocity = 10000
            req.outward = False
            assert not hasattr(
                req, "use_gpe"
            ), f"{gripper}: use_gpe not expected but found"

        # Call service
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.result() is not None, f"{gripper}: no response from service"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    node.destroy_node()
