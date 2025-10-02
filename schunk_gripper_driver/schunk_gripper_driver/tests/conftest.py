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

import pytest
import rclpy
from launch import LaunchDescription  # type: ignore [attr-defined]
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_pytest
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import SetParameters
from schunk_gripper_interfaces.srv import ListGrippers  # type: ignore [attr-defined]
from launch.launch_description_sources import PythonLaunchDescriptionSource

from rclpy.node import Node
import os
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from typing_extensions import Any
import importlib
from pathlib import Path
import json


def get_directory_size(directory):
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(directory):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            total_size += os.path.getsize(fp)
    return total_size


@pytest.fixture(scope="module")
def log_monitor(request, tmpdir_factory):
    log_dir = str(tmpdir_factory.mktemp("log_dir"))
    os.environ["ROS_LOG_DIR"] = log_dir

    print(f"log_size before: {get_directory_size(log_dir)} Bytes")

    yield

    after = get_directory_size(log_dir)
    print(f"log_size after: {after} Bytes")
    assert after < request.param.get("max_log_size")


@pytest.fixture(scope="module")
def ros2():
    rclpy.init()
    yield
    rclpy.shutdown()


@launch_pytest.fixture(scope="module")
def driver(request, ros2):
    start_empty = getattr(request.module, "start_empty", False)
    headless = getattr(request.module, "headless", False)
    setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("schunk_gripper_driver"),
                    "launch",
                    "driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "start_empty": str(start_empty).lower(),
            "headless": str(headless).lower(),
        }.items(),
    )
    return LaunchDescription([setup, launch_pytest.actions.ReadyToTest()])


class LifecycleInterface(object):
    def __init__(self):
        self.node = Node("lifecycle_interface")
        self.change_state_client = self.node.create_client(
            ChangeState, "/schunk/driver/change_state"
        )
        self.get_state_client = self.node.create_client(
            GetState, "/schunk/driver/get_state"
        )
        self.set_params_client = self.node.create_client(
            SetParameters, "/schunk/driver/set_parameters"
        )
        self.list_grippers_client = self.node.create_client(
            ListGrippers, "/schunk/driver/list_grippers"
        )
        self.change_state_client.wait_for_service(timeout_sec=2)
        self.get_state_client.wait_for_service(timeout_sec=2)
        self.set_params_client.wait_for_service(timeout_sec=2)
        self.list_grippers_client.wait_for_service(timeout_sec=2)

    def change_state(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def check_state(self, state_id) -> bool:
        req = GetState.Request()
        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().current_state.id == state_id

    def check(self, interfaces: list[str], dtype: str, should_exist: bool) -> bool:
        if dtype not in ["service", "topic"]:
            print(f"{dtype} not in available options [service, topic]")
            return False

        interface_names = []
        for name in interfaces:
            if not name.startswith("/schunk/driver"):
                for gripper in self.list_grippers():
                    interface_names.append(f"/schunk/driver/{gripper}/{name}")
            else:
                interface_names.append(name)

        if dtype == "service":
            existing_interfaces = getattr(
                self.node, f"get_{dtype}_names_and_types_by_node"
            )(node_name="driver", node_namespace="/schunk")
        else:
            existing_interfaces = self.node.get_topic_names_and_types()

        advertised_interfaces = [i[0] for i in existing_interfaces]
        advertised = [name in advertised_interfaces for name in interface_names]
        if should_exist:
            for idx, exists in enumerate(advertised):
                if not exists:
                    print(f"{dtype} {interface_names[idx]} not advertised")
            return all(advertised)
        else:
            for idx, exists in enumerate(advertised):
                if exists:
                    print(f"{dtype} {interface_names[idx]} incorrectly advertised")
            return not any(advertised)

    def list_grippers(self) -> list[str]:
        req = ListGrippers.Request()
        future = self.list_grippers_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        grippers = future.result().grippers
        return grippers

    def get_service_type(self, srv_name: str) -> Any | None:
        services = self.node.get_service_names_and_types()
        for name, types in services:
            if name == srv_name:
                pkg, _, srv = types[0].split("/")
                module = importlib.import_module(f"{pkg}.srv")
                return getattr(module, srv)
        return None


@pytest.fixture(scope="module")
def lifecycle_interface(driver):
    return LifecycleInterface()


@pytest.fixture(scope="module")
def log_helper(driver):
    return LogHelper()


def get_log_files():
    log_dir = os.environ.get("ROS_LOG_DIR")
    if not log_dir:
        raise ValueError("ROS_LOG_DIR environment variable is not set.")
    log_files = []
    for dirpath, dirnames, filenames in os.walk(log_dir):
        for filename in filenames:
            if filename.endswith(".log"):
                log_files.append(os.path.join(dirpath, filename))
    return log_files


@pytest.fixture(scope="function")
def log_level_checker(tmpdir_factory):
    log_dir = str(tmpdir_factory.mktemp("log_dir_checker"))
    os.environ["ROS_LOG_DIR"] = log_dir
    print("log_dir: ", log_dir)
    yield

    log_files = get_log_files()
    for log_file in log_files:
        with open(log_file, "r") as f:
            content = f.read()
            checks = {
                "DEBUG": False,
                "INFO": False,
                "on_activate()": False,
                "on_configure()": False,
            }
            for key in checks:
                if key in content:
                    checks[key] = True
            if not all(checks.values()):
                print(
                    f"""Log file {log_file} missing:
                    {[k for k, v in checks.items() if not v]}"""
                )
                assert False


class LogHelper:
    def __init__(self):
        pass

    def change_log_level(self, level):
        client = rclpy.create_node("test_driver_log_helper")
        set_params_client = client.create_client(
            SetParameters, "/schunk/driver/set_parameters"
        )
        assert set_params_client.wait_for_service(timeout_sec=2)

        future = set_params_client.call_async(
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="log_level",
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_STRING,
                            string_value=level,
                        ),
                    )
                ]
            )
        )

        rclpy.spin_until_future_complete(client, future)
        return future.result().results[0].successful


@pytest.fixture(scope="module")
def config_file():
    location = Path("/var/tmp/schunk_gripper")
    location.mkdir(parents=True, exist_ok=True)
    config_file = location.joinpath("configuration.json")
    config = [
        {"gripper_id": "gripper_1", "host": "0.0.0.0", "port": 8000},
        {"gripper_id": "gripper_2", "serial_port": "/dev/ttyUSB0", "device_id": 12},
    ]
    with open(config_file, "w") as f:
        json.dump(config, f, indent=2)

    with open(config_file, "r") as f:
        try:
            assert json.load(f) == config
        except json.JSONDecodeError:
            assert False
