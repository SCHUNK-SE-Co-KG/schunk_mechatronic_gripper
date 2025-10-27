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
    ShowConfiguration,
    ShowGripperSpecification,
    LocateGripper,
)
from schunk_gripper_interfaces.msg import (  # type: ignore [attr-defined]
    Gripper as GripperConfig,
)
from rclpy.node import Node
import rclpy
import time
from pathlib import Path
import json


@skip_without_gripper
def test_driver_advertises_state_depending_services(lifecycle_interface):
    driver = lifecycle_interface
    list_grippers = ["/schunk/driver/list_grippers"]
    save_configuration = ["/schunk/driver/save_configuration"]
    config_services = [
        "/schunk/driver/add_gripper",
        "/schunk/driver/reset_grippers",
        "/schunk/driver/show_configuration",
        "/schunk/driver/load_previous_configuration",
        "/schunk/driver/scan",
        "/schunk/driver/locate_gripper",
    ]
    gripper_services = [
        "acknowledge",
        "fast_stop",
        "stop",
        "move_to_absolute_position",
        "grip",
        "release",
        "show_specification",
        "start_jogging",
        "stop_jogging",
        "brake_test",
        "read_parameter",
        "write_parameter",
        "prepare_for_shutdown",
        "release_for_manual_movement",
        "soft_reset",
    ]
    until_change_takes_effect = 0.1

    for run in range(3):

        # After startup -> unconfigured
        driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)
        assert driver.check(config_services, dtype="service", should_exist=True)
        assert driver.check(save_configuration, dtype="service", should_exist=True)
        assert driver.check(list_grippers, dtype="service", should_exist=False)

        # After configure -> inactive
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(until_change_takes_effect)
        assert driver.check(list_grippers, dtype="service", should_exist=True)
        assert driver.check(save_configuration, dtype="service", should_exist=True)
        assert driver.check(config_services, dtype="service", should_exist=False)
        assert driver.check(gripper_services, dtype="service", should_exist=False)

        # After activate -> active
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.check(list_grippers, dtype="service", should_exist=True)
        assert driver.check(save_configuration, dtype="service", should_exist=True)
        assert driver.check(config_services, dtype="service", should_exist=False)
        assert driver.check(gripper_services, dtype="service", should_exist=True)

        # After deactivate -> inactive
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.check(list_grippers, dtype="service", should_exist=True)
        assert driver.check(save_configuration, dtype="service", should_exist=True)
        assert driver.check(config_services, dtype="service", should_exist=False)
        assert driver.check(gripper_services, dtype="service", should_exist=False)

        # After cleanup -> unconfigured
        driver.change_state(Transition.TRANSITION_CLEANUP)
        time.sleep(until_change_takes_effect)
        assert driver.check(config_services, dtype="service", should_exist=True)
        assert driver.check(save_configuration, dtype="service", should_exist=True)
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


@skip_without_gripper
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
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)

        assert future.result().success

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_stop(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_stop_service")
    setup_single_ethernet_dummy(node, "0.0.0.0", 8000)

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/stop"
        ServiceType = driver.get_service_type(service_name)
        assert ServiceType is not None, f"{gripper}: stop service not found"

        stop_client = node.create_client(ServiceType, service_name)
        assert stop_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: stop service unavailable"

        targets = [{"use_gpe": False}, {"use_gpe": True}]
        for target in targets:
            stop_req = ServiceType.Request()
            if hasattr(stop_req, "use_gpe"):
                stop_req.use_gpe = target["use_gpe"]

            future = stop_client.call_async(stop_req)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{gripper}: {future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


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


@skip_without_gripper
def test_driver_implements_prepare_for_shutdown(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_prepare_for_shutdown_service")
    setup_single_ethernet_dummy(node, "0.0.0.0", 8000)

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/prepare_for_shutdown"
        ServiceType = driver.get_service_type(service_name)
        assert (
            ServiceType is not None
        ), f"{gripper}: prepare_for_shutdown service not found"

        pfs_client = node.create_client(ServiceType, service_name)
        assert pfs_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: prepare_for_shutdown service unavailable"

        pfs_req = ServiceType.Request()
        future = pfs_client.call_async(pfs_req)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success, f"{gripper}: {future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_soft_reset(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_soft_reset_service")
    setup_single_ethernet_dummy(node, "0.0.0.0", 8000)

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/soft_reset"
        ServiceType = driver.get_service_type(service_name)
        assert ServiceType is not None, f"{gripper}: soft_reset service not found"

        sr_client = node.create_client(ServiceType, service_name)
        assert sr_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: soft_reset service unavailable"

        sr_req = ServiceType.Request()
        future = sr_client.call_async(sr_req)
        rclpy.spin_until_future_complete(node, future)
        assert future.result().success, f"{gripper}: {future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_move_to_absolute_position(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_move_to_absolute_position")
    setup_single_ethernet_dummy(node, "0.0.0.0", 8000)

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/move_to_absolute_position"
        ServiceType = driver.get_service_type(service_name)
        assert (
            ServiceType is not None
        ), f"{gripper}: move_to_absolute_position service not found"

        move_client = node.create_client(ServiceType, service_name)
        assert move_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: move_to_absolute_position service unavailable"

        targets = [
            {"position": 0.01, "velocity": 0.01, "use_gpe": False},
            {"position": 0.002, "velocity": 0.02, "use_gpe": True},
        ]

        for target in targets:
            move_req = ServiceType.Request()
            move_req.position = target["position"]
            move_req.velocity = target["velocity"]
            if hasattr(move_req, "use_gpe"):
                move_req.use_gpe = target["use_gpe"]

            future = move_client.call_async(move_req)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{gripper}: {future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_move_to_relative_position(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_move_to_relative_position")
    setup_single_ethernet_dummy(node, "0.0.0.0", 8000)

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        service_name = f"/schunk/driver/{gripper}/move_to_relative_position"
        ServiceType = driver.get_service_type(service_name)
        assert (
            ServiceType is not None
        ), f"{gripper}: move_to_relative_position service not found"

        move_client = node.create_client(ServiceType, service_name)
        assert move_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: move_to_relative_position service unavailable"

        targets = [
            {"position": 0.005, "velocity": 0.01, "use_gpe": False},
            {"position": -0.002, "velocity": 0.02, "use_gpe": True},
        ]

        for target in targets:
            move_req = ServiceType.Request()
            move_req.position = target["position"]
            move_req.velocity = target["velocity"]
            if hasattr(move_req, "use_gpe"):
                move_req.use_gpe = target["use_gpe"]

            future = move_client.call_async(move_req)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{gripper}: {future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_grip_and_release(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_grip")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    reset_req = Trigger.Request()
    future = reset_client.call_async(reset_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    add_req = AddGripper.Request()
    add_req.gripper.host = "0.0.0.0"
    add_req.gripper.port = 8000
    future = add_client.call_async(add_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        if not gripper.startswith("EGU") and not gripper.startswith("EZU"):
            continue
        grip_service_name = f"/schunk/driver/{gripper}/grip"
        GripServiceType = driver.get_service_type(grip_service_name)
        assert GripServiceType is not None, f"{gripper}: grip service not found"
        grip_client = node.create_client(GripServiceType, grip_service_name)
        assert grip_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: grip service unavailable"

        release_service_name = f"/schunk/driver/{gripper}/release"
        ReleaseServiceType = driver.get_service_type(release_service_name)
        assert ReleaseServiceType is not None, f"{gripper}: release service not found"
        release_client = node.create_client(ReleaseServiceType, release_service_name)
        assert release_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: release service unavailable"

        targets = [
            {"force": 110, "use_gpe": True, "outward": False},
            {"force": 130, "use_gpe": True, "outward": False},
            {"force": 150, "use_gpe": True, "outward": True},
            {"force": 200, "use_gpe": True, "outward": True},
        ]

        for target in targets:
            # Grip
            grip_req = GripServiceType.Request()
            grip_req.force = target["force"]
            grip_req.outward = target["outward"]
            if hasattr(grip_req, "use_gpe"):
                grip_req.use_gpe = target["use_gpe"]

            future = grip_client.call_async(grip_req)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{gripper}: {future.result().message}"

            # Release
            release_req = ReleaseServiceType.Request()
            future = release_client.call_async(release_req)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{gripper}: {future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_release_for_manual_movement(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_release_for_manual_movement")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Reset grippers (drop default modbus gripper)
    reset_req = Trigger.Request()
    future = reset_client.call_async(reset_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    add_req = AddGripper.Request()
    add_req.gripper.host = "0.0.0.0"
    add_req.gripper.port = 8000
    future = add_client.call_async(add_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        client_release = node.create_client(
            Trigger, f"/schunk/driver/{gripper}/release_for_manual_movement"
        )
        assert client_release.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        future = client_release.call_async(Trigger.Request())
        # calling release_for_manual_movement is not allowed if the gripper is not
        # in an error state, so the following service call should return false
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        assert not future.result().success

        # put the gripper in an error and try again
        client_fast_stop = node.create_client(
            Trigger, f"/schunk/driver/{gripper}/fast_stop"
        )
        assert client_fast_stop.wait_for_service(timeout_sec=2), f"gripper: {gripper}"
        future = client_fast_stop.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        assert future.result().success
        # try release_for_manual_movement again, which should now succeed
        future = client_release.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        assert future.result().success, f"gripper: {gripper}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_grip_at_position(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_grip_at_position")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Reset grippers (drop default modbus gripper)
    reset_req = Trigger.Request()
    future = reset_client.call_async(reset_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    add_req = AddGripper.Request()
    add_req.gripper.host = "0.0.0.0"
    add_req.gripper.port = 8000
    future = add_client.call_async(add_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        if not gripper.startswith("EGU") and not gripper.startswith("EZU"):
            continue
        grip_service_name = f"/schunk/driver/{gripper}/grip_at_position"
        GripServiceType = driver.get_service_type(grip_service_name)
        assert (
            GripServiceType is not None
        ), f"{gripper}: grip_at_position service not found"
        grip_client = node.create_client(GripServiceType, grip_service_name)
        assert grip_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: grip_at_position service unavailable"

        targets = [
            {"force": 120, "position": 0.01, "use_gpe": True, "outward": False},
            {"force": 150, "position": 0.02, "use_gpe": True, "outward": False},
            {"force": 180, "position": 0.015, "use_gpe": True, "outward": True},
            {"force": 200, "position": 0.012, "use_gpe": True, "outward": True},
        ]

        for target in targets:
            # Grip at position
            grip_req = GripServiceType.Request()
            grip_req.force = target["force"]
            grip_req.position = target["position"]
            grip_req.outward = target["outward"]
            if hasattr(grip_req, "use_gpe"):
                grip_req.use_gpe = target["use_gpe"]

            future = grip_client.call_async(grip_req)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{gripper}: {future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_grip_with_velocity(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_grip_with_velocity")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        if not gripper.startswith("EGK"):
            continue
        service_name = f"/schunk/driver/{gripper}/grip"
        ServiceType = driver.get_service_type(service_name)
        assert ServiceType is not None, f"{gripper}: service type not found"

        client = node.create_client(ServiceType, service_name)
        assert client.wait_for_service(timeout_sec=5), f"{gripper}: service unavailable"

        targets = [
            {"force": 50, "velocity": 0.01, "use_gpe": False, "outward": False},
            {"force": 100, "velocity": 0.07, "use_gpe": True, "outward": False},
            {"force": 75, "velocity": 0.011, "use_gpe": False, "outward": True},
            {"force": 88, "velocity": 0.015, "use_gpe": True, "outward": True},
        ]

        for target in targets:
            # Grip With Velocity
            request = ServiceType.Request()
            request.force = target["force"]
            request.velocity = target["velocity"]
            request.outward = target["outward"]
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]

            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_grip_at_position_with_velocity(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_grip_at_position_with_velocity")
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    request = Trigger.Request()
    future = reset_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    request = AddGripper.Request()
    request.gripper.host = "0.0.0.0"
    request.gripper.port = 8000
    future = add_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        if not gripper.startswith("EGK"):
            continue
        service_name = f"/schunk/driver/{gripper}/grip_at_position"
        ServiceType = driver.get_service_type(service_name)
        assert ServiceType is not None, f"{gripper}: service type not found"

        grip_client = node.create_client(ServiceType, service_name)
        assert grip_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: service unavailable"

        targets = [
            {
                "force": 50,
                "position": 0.01,
                "velocity": 0.01,
                "use_gpe": False,
                "outward": False,
            },
            {
                "force": 100,
                "position": 0.02,
                "velocity": 0.07,
                "use_gpe": True,
                "outward": False,
            },
            {
                "force": 75,
                "position": 0.015,
                "velocity": 0.011,
                "use_gpe": False,
                "outward": True,
            },
            {
                "force": 88,
                "position": 0.012,
                "velocity": 0.015,
                "use_gpe": True,
                "outward": True,
            },
        ]

        for target in targets:
            # Grip at position with velocity
            request = ServiceType.Request()
            request.force = target["force"]
            request.velocity = target["velocity"]
            request.position = target["position"]
            request.outward = target["outward"]
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]

            future = grip_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


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
    config = {"gripper_id": "abc", "host": "0.0.0.0", "port": 80}
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
    # we do not setup a single ethernet dummy here, because the dummy server
    # does not support stopping jogging => Use the modbus simulator instead

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    # Get the gripper's services
    for gripper in driver.list_grippers():
        StartJoggingType = driver.get_service_type(
            f"/schunk/driver/{gripper}/start_jogging"
        )
        start_jogging_client = node.create_client(
            StartJoggingType,
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
            request = StartJoggingType.Request()
            if hasattr(request, "use_gpe"):
                request.use_gpe = target["use_gpe"]
            future = start_jogging_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}, for {target}"

            # stop
            future = stop_jogging_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future)
            assert future.result().success, f"{future.result().message}"

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_implements_brake_test(lifecycle_interface):
    driver = lifecycle_interface

    node = Node("check_brake_test")
    setup_single_ethernet_dummy(node, "0.0.0.0", 8000)

    driver.change_state(Transition.TRANSITION_CONFIGURE)
    driver.change_state(Transition.TRANSITION_ACTIVATE)

    for gripper in driver.list_grippers():
        brake_service_name = f"/schunk/driver/{gripper}/brake_test"
        brake_client = node.create_client(Trigger, brake_service_name)
        assert brake_client.wait_for_service(
            timeout_sec=5
        ), f"{gripper}: brake_test service unavailable"

        req = Trigger.Request()
        future = brake_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)

        assert future.result().success

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)
    node.destroy_node()


@skip_without_gripper
def test_driver_implements_locating_gripper(driver):

    node = Node("check_locate_gripper")
    client = node.create_client(LocateGripper, "/schunk/driver/locate_gripper")
    assert client.wait_for_service(timeout_sec=2)

    # TCP/IP
    req = LocateGripper.Request()
    req.gripper.host = "0.0.0.0"
    req.gripper.port = 8000
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Modbus
    req = LocateGripper.Request()
    req.gripper.serial_port = "/dev/ttyUSB0"
    req.gripper.device_id = 12
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success


def setup_single_ethernet_dummy(node: Node, host: str, port: int):
    add_client = node.create_client(AddGripper, "/schunk/driver/add_gripper")
    reset_client = node.create_client(Trigger, "/schunk/driver/reset_grippers")
    assert add_client.wait_for_service(timeout_sec=2)
    assert reset_client.wait_for_service(timeout_sec=2)

    # Reset grippers
    reset_req = Trigger.Request()
    future = reset_client.call_async(reset_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success

    # Add TCP/IP gripper
    add_req = AddGripper.Request()
    add_req.gripper.host = host
    add_req.gripper.port = port
    future = add_client.call_async(add_req)
    rclpy.spin_until_future_complete(node, future)
    assert future.result().success
