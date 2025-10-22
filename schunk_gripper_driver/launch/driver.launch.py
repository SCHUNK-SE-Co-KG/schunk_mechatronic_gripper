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

from launch import LaunchDescription  # type: ignore [attr-defined]
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

host = DeclareLaunchArgument(
    "host",
    default_value="",
    description="The gripper's TCP/IP host address",
)
port = DeclareLaunchArgument(
    "port",
    default_value="80",
    description="The gripper's TCP/IP port",
)
serial_port = DeclareLaunchArgument(
    "serial_port",
    default_value="",
    description="The gripper's serial port",
)
device_id = DeclareLaunchArgument(
    "device_id",
    default_value="12",
    description="The gripper's Modbus device id",
)

headless = DeclareLaunchArgument(
    "headless",
    default_value="false",
    description=(
        "Whether to `configure` and `activate` the driver "
        "using the last successful configuration"
    ),
)

args = [host, port, serial_port, device_id, headless]


def generate_launch_description():
    return LaunchDescription(
        args
        + [
            Node(
                package="schunk_gripper_driver",
                namespace="schunk",
                executable="driver.py",
                name="driver",
                parameters=[
                    {"host": LaunchConfiguration("host")},
                    {"port": LaunchConfiguration("port")},
                    {"serial_port": LaunchConfiguration("serial_port")},
                    {"device_id": LaunchConfiguration("device_id")},
                    {"headless": LaunchConfiguration("headless")},
                ],
                respawn=True,
                output="both",
                # arguments=['--ros-args', '--log-level', 'DEBUG'],
            )
        ]
    )
