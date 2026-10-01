# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

import math
import time

from schunk_gripper_library.tests.utils.functions import skip_if_no_drivers
from schunk_gripper_library.tests.utils.params import FloatParam, read_float_param


STREAM_DURATION_SEC = 60.0
STREAM_PERIOD_SEC = 2.0
STREAM_UPDATE_PERIOD_SEC = 0.0167  # 60 hz
STREAM_OFFSET_MM = 20.0


def test_stream_target_position(drivers):
    """Oscillate each gripper around its middle position for STREAM_DURATION_SEC seconds."""
    skip_if_no_drivers(drivers)

    for driver in drivers:
        min_pos_um = int(read_float_param(driver, FloatParam.min_pos) * 1000)
        max_pos_um = int(read_float_param(driver, FloatParam.max_pos) * 1000)
        middle_pos_um = int((min_pos_um + max_pos_um) / 2)
        offset_um = min(
            int(STREAM_OFFSET_MM * 1000),
            middle_pos_um - min_pos_um,
            max_pos_um - middle_pos_um,
        )

        move_success = driver.move_to_position(
            position=middle_pos_um,
            velocity=int(read_float_param(driver, FloatParam.max_vel) * 1000),
            is_absolute=True,
        )
        assert move_success, (
            f"Move to middle position failed. Driver: {driver.addr_str}, "
            f"Status: {driver.get_status_diagnostics()}"
        )

        stream_enabled = False
        try:
            stream_enabled = driver.enable_stream()
            assert stream_enabled, (
                f"Failed to enable stream. Driver: {driver.addr_str}, "
                f"Status: {driver.get_status_diagnostics()}"
            )
            start_time = time.monotonic()
            next_update = start_time
            while (elapsed := time.monotonic() - start_time) < STREAM_DURATION_SEC:
                target_pos_um = middle_pos_um + int(
                    offset_um * math.sin(2.0 * math.pi * elapsed / STREAM_PERIOD_SEC)
                )
                assert driver.move_to_streamed_target(target_pos_um), (
                    f"Failed to set streamed target. Driver: {driver.addr_str}, "
                    f"Status: {driver.get_status_diagnostics()}"
                )

                next_update += STREAM_UPDATE_PERIOD_SEC
                time.sleep(max(0.0, next_update - time.monotonic()))
        finally:
            try:
                if stream_enabled:
                    driver.stop()
            finally:
                driver.disable_stream()
