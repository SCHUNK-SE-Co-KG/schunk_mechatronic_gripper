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

import time

from schunk_gripper_library.driver import Driver


def make_streaming_bke3_driver() -> tuple[Driver, list[float]]:
    driver = Driver()
    driver.connected = True
    driver.hardware_style = "BKE3"
    driver.streaming_active = True
    driver.update_cycle_while_streaming = 0.2
    polls: list[float] = []

    def fake_receive_plc_input() -> bool:
        polls.append(time.monotonic())
        driver._last_status_read = time.monotonic()
        return True

    driver.receive_plc_input = fake_receive_plc_input  # type: ignore[method-assign]
    driver._last_status_read = time.monotonic()
    return driver, polls


def test_no_heartbeat_poll_while_stream_frames_arrive():
    driver, polls = make_streaming_bke3_driver()
    driver.start_module_updates()
    try:
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            driver._last_status_read = time.monotonic()  # simulates a read-write stream frame
            time.sleep(0.02)
    finally:
        driver.stop_module_updates()
    assert polls == []


def test_heartbeat_poll_when_stream_frames_pause():
    driver, polls = make_streaming_bke3_driver()
    driver.start_module_updates()
    try:
        time.sleep(1.1)
    finally:
        driver.stop_module_updates()
    assert 4 <= len(polls) <= 6
    gaps = [b - a for a, b in zip(polls, polls[1:])]
    assert all(0.15 < gap < 0.3 for gap in gaps)


def test_hardware_style_from_version():
    assert Driver._hardware_style_from_version("6.0.1") == "BKE3"
    assert Driver._hardware_style_from_version(" 7.2 ") == "BKE3"
    assert Driver._hardware_style_from_version("5.4.3") == "BKE2"


def test_read_write_is_only_used_for_bke3():
    driver = Driver()
    driver.hardware_style = "BKE2"
    assert not driver._read_write_plc_now()
