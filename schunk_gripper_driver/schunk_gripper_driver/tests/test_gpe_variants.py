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
from schunk_gripper_interfaces.srv import (  # type: ignore [attr-defined]
    Grip,
    GripGPE,
)

# Check if we can call all driver callbacks that support non-GPE and GPE variants.


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

    driver.on_deactivate(state=None)
    driver.on_cleanup(state=None)
