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
)


@skip_without_gripper
def test_grip_callback_handles_all_gpe_variants(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)
    driver.on_activate(state=None)

    for gripper in driver.grippers:

        # Grip
        driver._grip_cb(
            request=Grip.Request(), response=Grip.Response(), gripper=gripper
        )

        # GripGPE
        driver._grip_cb(
            request=GripGPE.Request(), response=GripGPE.Response(), gripper=gripper
        )

        # GripAtPositon
        driver._grip_cb(
            request=GripAtPosition.Request(),
            response=GripAtPosition.Response(),
            gripper=gripper,
        )

        # GripAtPositionGPE
        driver._grip_cb(
            request=GripAtPositionGPE.Request(),
            response=GripAtPositionGPE.Response(),
            gripper=gripper,
        )

        # SoftGrip
        driver._grip_cb(
            request=SoftGrip.Request(),
            response=SoftGrip.Response(),
            gripper=gripper,
        )

        # SoftGripGPE
        driver._grip_cb(
            request=SoftGripGPE.Request(),
            response=SoftGripGPE.Response(),
            gripper=gripper,
        )

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_offers_gpe_specific_grip_services(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)

    # Mimic modules with and without GPE and check
    # if the driver creates the expected services

    modules = ["EGU_50_M_B", "EGK_25_N_B"]
    expected_grip_types = [GripGPE, Grip]
    expected_grip_at_position_types = [GripAtPositionGPE, GripAtPosition]

    for module, grip_type, grip_at_position_type in zip(
        modules, expected_grip_types, expected_grip_at_position_types
    ):
        driver.grippers[0]["driver"].module = module
        driver.on_activate(state=None)

        for service in driver.gripper_services:
            if service.srv_name.endswith("/grip"):
                assert service.srv_type == grip_type
            if service.srv_name.endswith("/grip_at_position"):
                assert service.srv_type == grip_at_position_type

        driver.on_deactivate(state=None)

    driver.on_cleanup(state=None)


@skip_without_gripper
def test_driver_offers_gpe_specific_soft_grip_services(ros2):
    driver = Driver("driver")
    driver.on_configure(state=None)

    # Mimic modules with and without GPE and check
    # if the driver creates the expected services

    modules = ["EGK_25_M_B", "EGK_25_N_B"]
    expected_soft_grip_types = [SoftGripGPE, SoftGrip]

    for module, grip_type in zip(modules, expected_soft_grip_types):
        driver.grippers[0]["driver"].module = module
        driver.on_activate(state=None)
        for service in driver.gripper_services:
            if service.srv_name.endswith("/soft_grip"):
                assert service.srv_type == grip_type

        driver.on_deactivate(state=None)

    non_egk_modules = ["EGU_50_M_B", "EGU_50_N_B"]

    for module in non_egk_modules:
        driver.grippers[0]["driver"].module = module
        driver.on_activate(state=None)
        for service in driver.gripper_services:
            if service.srv_name.endswith("/soft_grip"):
                assert False

        driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)
