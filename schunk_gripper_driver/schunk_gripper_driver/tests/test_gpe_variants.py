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
    GripGPE,
    GripAtPosition,
    GripAtPositionGPE,
    SoftGrip,
    SoftGripGPE,
    SoftGripAtPosition,
    SoftGripAtPositionGPE,
    StrongGripGPE,
    StrongGripAtPositionGPE,
    MoveToAbsolutePosition,
    MoveToAbsolutePositionGPE,
)


@skip_without_gripper
def test_grip_callback_handles_all_gpe_variants(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    service_types = [
        Grip,
        GripGPE,
        GripAtPosition,
        GripAtPositionGPE,
        SoftGrip,
        SoftGripGPE,
        SoftGripAtPosition,
        SoftGripAtPositionGPE,
        StrongGripGPE,
        StrongGripAtPositionGPE,
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
            "/grip": GripGPE,
            "/grip_at_position": GripAtPositionGPE,
            "/soft_grip": None,
            "/soft_grip_at_position": None,
            "/strong_grip": StrongGripGPE,
            "/strong_grip_at_position": StrongGripAtPositionGPE,
            "/move_to_absolute_position": MoveToAbsolutePositionGPE,
        },
        "EGU_50_N_B": {
            "/grip": Grip,
            "/grip_at_position": GripAtPosition,
            "/soft_grip": None,
            "/soft_grip_at_position": None,
            "/strong_grip": None,
            "/strong_grip_at_position": None,
            "/move_to_absolute_position": MoveToAbsolutePosition,
        },
        "EGK_25_M_B": {
            "/grip": GripGPE,
            "/grip_at_position": GripAtPositionGPE,
            "/soft_grip": SoftGripGPE,
            "/soft_grip_at_position": SoftGripAtPositionGPE,
            "/strong_grip": None,
            "/strong_grip_at_position": None,
            "/move_to_absolute_position": MoveToAbsolutePositionGPE,
        },
        "EGK_25_N_B": {
            "/grip": Grip,
            "/grip_at_position": GripAtPosition,
            "/soft_grip": SoftGrip,
            "/soft_grip_at_position": SoftGripAtPosition,
            "/strong_grip": None,
            "/strong_grip_at_position": None,
            "/move_to_absolute_position": MoveToAbsolutePosition,
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

    service_types = [
        MoveToAbsolutePosition,
        MoveToAbsolutePositionGPE,
    ]
    for gripper in driver.grippers:
        for service_type in service_types:
            driver._move_to_position_cb(
                request=service_type.Request(),
                response=service_type.Response(),
                gripper=gripper,
                is_absolute=True,
            )

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)
