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
from rclpy.node import Node
import rclpy
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    ShowConfiguration,
)


def test_driver_can_start_without_initial_gripper(driver):
    node = Node("test_without_initial_gripper")

    client = node.create_client(ShowConfiguration, "/schunk/driver/show_configuration")

    assert client.wait_for_service(timeout_sec=2.0)

    request = ShowConfiguration.Request()
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    response = future.result()
    assert response.configuration == []

    node.destroy_node()
