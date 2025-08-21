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
from sensor_msgs.msg import JointState
import rclpy
import time
from functools import partial
from schunk_gripper_interfaces.msg import GripperState  # type: ignore [attr-defined]


@skip_without_gripper
def test_driver_advertises_state_depending_topics(lifecycle_interface):
    driver = lifecycle_interface
    gripper_topics = ["joint_states", "gripper_state"]
    until_change_takes_effect = 0.1

    for run in range(1):

        # After startup -> unconfigured
        assert driver.check_state(State.PRIMARY_STATE_UNCONFIGURED)

        # After configure -> inactive
        driver.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(until_change_takes_effect)
        assert driver.check(gripper_topics, dtype="topic", should_exist=False)

        # After activate -> active
        driver.change_state(Transition.TRANSITION_ACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.check(gripper_topics, dtype="topic", should_exist=True)

        # After deactivate -> inactive
        driver.change_state(Transition.TRANSITION_DEACTIVATE)
        time.sleep(until_change_takes_effect)
        assert driver.check(gripper_topics, dtype="topic", should_exist=False)

        # After cleanup -> unconfigured
        driver.change_state(Transition.TRANSITION_CLEANUP)


@skip_without_gripper
def test_driver_publishes_joint_states(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    grippers = driver.list_grippers()

    driver.change_state(Transition.TRANSITION_ACTIVATE)
    messages = []

    def check_fields(msg: JointState, messages: list[JointState]) -> None:
        messages.append(msg)
        assert msg.header.stamp
        assert msg.header.frame_id
        assert len(msg.name) >= 1
        assert len(msg.position) >= 1

    for gripper in grippers:
        _ = driver.node.create_subscription(
            JointState,
            f"/schunk/driver/{gripper}/joint_states",
            partial(check_fields, messages=messages),
            1,
        )

    timeout = time.time() + 1.0
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1


@skip_without_gripper
def test_driver_publishes_gripper_state(lifecycle_interface):
    driver = lifecycle_interface
    driver.change_state(Transition.TRANSITION_CONFIGURE)
    grippers = driver.list_grippers()

    driver.change_state(Transition.TRANSITION_ACTIVATE)
    messages = []

    def check_fields(msg: GripperState, messages: list[GripperState]) -> None:
        messages.append(msg)
        assert msg.header.stamp
        assert msg.header.frame_id

        assert "error_code: 0x" in msg.error_code
        assert "warning_code: 0x" in msg.warning_code
        assert "additional_code: 0x" in msg.additional_code

        assert msg.bit0_ready_for_operation is not None
        assert msg.bit1_control_authority_fieldbus is not None
        assert msg.bit2_ready_for_shutdown is not None
        assert msg.bit3_not_feasible is not None
        assert msg.bit4_command_successfully_processed is not None
        assert msg.bit5_command_received_toggle is not None
        assert msg.bit6_warning is not None
        assert msg.bit7_error is not None
        assert msg.bit8_released_for_manual_movement is not None
        assert msg.bit9_software_limit_reached is not None
        assert msg.bit11_no_workpiece_detected is not None
        assert msg.bit12_workpiece_gripped is not None
        assert msg.bit13_position_reached is not None
        assert msg.bit14_workpiece_pre_grip_started is not None
        assert msg.bit16_workpiece_lost is not None
        assert msg.bit17_wrong_workpiece_gripped is not None
        assert msg.bit31_grip_force_and_position_maintenance_activated is not None

    for gripper in grippers:
        _ = driver.node.create_subscription(
            GripperState,
            f"/schunk/driver/{gripper}/gripper_state",
            partial(check_fields, messages=messages),
            1,
        )

    timeout = time.time() + 1.0
    while time.time() < timeout and len(messages) == 0:
        rclpy.spin_once(driver.node, timeout_sec=0.1)

    driver.change_state(Transition.TRANSITION_DEACTIVATE)
    driver.change_state(Transition.TRANSITION_CLEANUP)

    assert len(messages) >= 1
