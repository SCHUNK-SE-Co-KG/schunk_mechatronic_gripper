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
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    Grip,
    GripWithGPE,
    GripAtPosition,
    GripAtPositionWithGPE,
    GripWithVelocity,
    GripWithVelocityAndGPE,
    GripAtPositionWithVelocity,
    GripAtPositionWithVelocityAndGPE,
    MoveToAbsolutePosition,
    MoveToAbsolutePositionGPE,
    MoveToRelativePosition,
    MoveToRelativePositionGPE,
)


@skip_without_gripper
def test_grip_callback_handles_all_gpe_variants(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    service_types = [
        Grip,
        GripWithGPE,
        GripAtPosition,
        GripAtPositionWithGPE,
        GripWithVelocity,
        GripWithVelocityAndGPE,
        GripAtPositionWithVelocity,
        GripAtPositionWithVelocityAndGPE,
    ]
    for gripper in driver.grippers:
        for service_type in service_types:
            driver._grip_cb(
                request=service_type.Request(),
                response=service_type.Response(),
                gripper=gripper,
            )

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_offers_gpe_specific_services(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)

    module_expectations = {
        "EGU_50_M_B": {
            "/grip": GripWithGPE,
            "/grip_at_position": GripAtPositionWithGPE,
            "/move_to_absolute_position": MoveToAbsolutePositionGPE,
            "/move_to_relative_position": MoveToRelativePositionGPE,
        },
        "EGU_50_N_B": {
            "/grip": Grip,
            "/grip_at_position": GripAtPosition,
            "/move_to_absolute_position": MoveToAbsolutePosition,
            "/move_to_relative_position": MoveToRelativePosition,
        },
        "EGK_25_M_B": {
            "/grip": GripWithVelocityAndGPE,
            "/grip_at_position": GripAtPositionWithVelocityAndGPE,
            "/move_to_absolute_position": MoveToAbsolutePositionGPE,
            "/move_to_relative_position": MoveToRelativePositionGPE,
        },
        "EGK_25_N_B": {
            "/grip": GripWithVelocity,
            "/grip_at_position": GripAtPositionWithVelocity,
            "/move_to_absolute_position": MoveToAbsolutePosition,
            "/move_to_relative_position": MoveToRelativePosition,
        },
    }
    for module, services in module_expectations.items():
        driver.grippers[0]["driver"].module = module
        driver.on_activate(state=None)

        for srv_suffix, expected_type in services.items():
            for service in driver.gripper_services:
                if service.srv_name.endswith(srv_suffix):
                    if expected_type is not None:
                        assert service.srv_type == expected_type
                    else:
                        assert False

        driver.on_deactivate(state=None)

    driver.on_cleanup(state=None)


@skip_without_gripper
def test_move_to_position_callback_handles_all_variants(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    service_type_and_flag = {
        MoveToAbsolutePosition: True,
        MoveToAbsolutePositionGPE: True,
        MoveToRelativePosition: False,
        MoveToRelativePositionGPE: False,
    }

    for gripper in driver.grippers:
        for service_type, is_absolute in service_type_and_flag.items():
            driver._move_to_position_cb(
                request=service_type.Request(),
                response=service_type.Response(),
                gripper=gripper,
                is_absolute=is_absolute,
            )

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)
