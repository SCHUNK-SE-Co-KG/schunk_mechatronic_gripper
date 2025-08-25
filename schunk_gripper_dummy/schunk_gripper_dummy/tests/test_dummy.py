from schunk_gripper_dummy.dummy import Dummy
import pytest
import struct
import time

# [1]: https://stb.cloud.schunk.com/media/IM0046706.PDF

GRIPPERS = ["EGU_60_PN_M_B", "EGU_60_EI_M_B"]


def test_dummy_starts_a_background_thread():
    dummy = Dummy()
    assert not dummy.running
    dummy.start()
    assert dummy.running
    dummy.stop()
    assert not dummy.running


def test_dummy_survives_repeated_starts_and_stops():
    dummy = Dummy()
    for _ in range(3):
        dummy.start()
    assert dummy.running

    for _ in range(3):
        dummy.stop()
    assert not dummy.running


def test_dummy_loads_available_grippers_on_startup():
    dummy = Dummy()
    assert isinstance(dummy.available_grippers, dict)


def test_dummy_reads_configuration_on_startup():

    # Existing grippers
    for gripper in GRIPPERS:
        dummy = Dummy(gripper=gripper)
        assert gripper in dummy.available_grippers.keys()
        assert dummy.data == dummy.available_grippers[gripper]

    # Non-existent grippers
    invalid_grippers = ["non-existent"]
    for gripper in invalid_grippers:
        with pytest.raises(ValueError, match="Unknown gripper"):
            Dummy(gripper)


def test_dummy_knows_fieldbus_type():
    dummy = Dummy()
    assert dummy.fieldbus
    assert dummy.fieldbus in dummy.valid_fieldbus_types.values()

    pn_dummy = Dummy(gripper="EGU_60_PN_M_B")
    ei_dummy = Dummy(gripper="EGU_60_EI_M_B")
    assert pn_dummy.fieldbus != ei_dummy.fieldbus


def test_dummy_knows_valid_parameters():
    for gripper in GRIPPERS:
        dummy = Dummy(gripper)
        assert dummy.valid_parameters
        for key, value in dummy.valid_parameters.items():
            assert isinstance(key, str)
            assert key.startswith("0x")
            assert isinstance(value, str)


def test_dummy_has_min_max_parameters():
    dummy = Dummy()

    # Position
    assert isinstance(dummy.max_position, int)
    assert isinstance(dummy.min_position, int)
    assert dummy.max_position < 150000  # sane SCHUNK gripper max

    # Speed
    assert isinstance(dummy.max_grp_vel, int)
    assert dummy.max_grp_vel > 0


def test_dummy_starts_in_error_state():
    # See p. 24 in
    # Booting and establishing operational readiness [1]
    dummy = Dummy()
    assert dummy.get_status_bit(0) == 0  # not ready for operation
    assert dummy.get_status_bit(7) == 1  # there's an error
    assert dummy.get_status_error() == "D9"  # ERR_FAST_STOP
    assert dummy.get_status_diagnostics() == "0"


def test_dummy_starts_with_cleared_control_bits():
    dummy = Dummy()
    for bit in dummy.valid_control_bits[1:]:  # without fast_stop
        assert dummy.get_control_bit(bit=bit) == 0


def test_dummy_clears_control_bits_after_processing():
    dummy = Dummy()
    for bit in dummy.valid_control_bits[1:]:  # without fast_stop
        dummy.set_control_bit(bit=bit, value=True)
        dummy.process_control_bits()
        assert dummy.get_control_bit(bit=bit) == 0


def test_dummy_clears_status_bits_before_processing():
    dummy = Dummy()

    # Mimic some operations that will set arbitrary status bits
    for bit in dummy.valid_status_bits:
        dummy.set_status_bit(bit=bit, value=True)

    dummy.process_control_bits()
    for bit in dummy.valid_status_bits:
        # Some status bits shouldn't be reset
        if bit in [5, 7]:
            pass
        else:
            assert dummy.get_status_bit(bit=bit) == 0


def test_dummy_offers_an_acknowledge_method():
    dummy = Dummy()
    dummy.acknowledge()
    assert dummy.get_status_bit(0) == 1  # ready
    assert dummy.get_status_bit(7) == 0  # no error
    assert dummy.get_status_error() == "0"
    assert dummy.get_status_diagnostics() == "0"


def test_dummy_is_ready_after_acknowledge():
    dummy = Dummy()
    dummy.set_control_bit(bit=2, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(0) == 1  # ready
    assert dummy.get_status_bit(7) == 0  # no error
    assert dummy.get_status_error() == "0"
    assert dummy.get_status_diagnostics() == "0"


def test_dummy_toggles_command_received_bit_with_regular_control_bits():
    dummy = Dummy()
    for bit in dummy.valid_control_bits[1:]:  # without inverted fast_stop bit 0
        dummy.set_control_bit(bit=bit, value=True)
        before = dummy.get_status_bit(bit=5)  # command received toggle
        dummy.process_control_bits()
        after = dummy.get_status_bit(bit=5)
        assert after != before


def test_dummy_does_not_toggle_command_received_bit_when_clearing():
    dummy = Dummy()
    before = dummy.get_status_bit(bit=5)
    dummy.clear_plc_output()
    dummy.process_control_bits()
    after = dummy.get_status_bit(bit=5)
    assert after == before


def test_dummy_rejects_commands_when_in_error():
    dummy = Dummy()
    bits = [
        8,  # jog mode -
        9,  # jog mode +
        12,  # grip
        13,  # move to abs
        14,  # move to rel
        16,  # grip at exp
        30,  # brake test
    ]
    for bit in bits:
        dummy.set_control_bit(bit=bit, value=True)
        dummy.process_control_bits()

        # Module stays in error
        assert dummy.get_status_bit(bit=7) == 1
        assert dummy.get_status_bit(bit=0) == 0
        assert dummy.get_status_bit(bit=4) == 0


def test_dummy_moves_to_absolute_position():
    dummy = Dummy()
    dummy.acknowledge()
    target_positions = [12345, 10555, 7000, 1500]  # mu
    target_speeds = [50300, 40000, 10500, 20999]  # mu / s
    for target_pos, target_speed in zip(target_positions, target_speeds):
        control_double_word = "01200000"  # bit 13
        set_position = bytes(struct.pack("i", target_pos)).hex().upper()
        set_speed = bytes(struct.pack("i", target_speed)).hex().upper()
        gripping_force = "00000000"
        command = {
            "inst": dummy.plc_output,
            "value": control_double_word + set_position + set_speed + gripping_force,
        }

        dummy.update(command)
        assert dummy.get_actual_position() == pytest.approx(target_pos, rel=1e-3)
        assert dummy.get_status_bit(bit=13) == 1  # position reached
        assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_rejects_invalid_speeds_for_move_absolute_commands():
    dummy = Dummy()
    dummy.acknowledge()
    target_positions = [12345, 10555, 7000, 1500]
    target_speeds = [0, -100, -0]
    expected_position = dummy.get_actual_position()
    for target_pos, target_speed in zip(target_positions, target_speeds):
        control_double_word = "01200000"  # bit 13
        set_position = bytes(struct.pack("i", target_pos)).hex().upper()
        set_speed = bytes(struct.pack("i", target_speed)).hex().upper()
        gripping_force = "00000000"
        command = {
            "inst": dummy.plc_output,
            "value": control_double_word + set_position + set_speed + gripping_force,
        }

        dummy.update(command)

        # Dummy shouldn't move
        assert dummy.get_actual_position() == pytest.approx(expected_position, rel=1e-3)
        assert dummy.get_status_bit(bit=3) == 1
        assert dummy.get_status_bit(bit=13) == 0  # position reached
        assert dummy.get_status_bit(bit=4) == 0  # command successfully processed


def test_dummy_moves_to_relative_position():
    dummy = Dummy()
    dummy.acknowledge()
    target_pos = -5000  # mu
    target_speed = 12000  # mu / s
    control_double_word = "01400000"  # bit 14
    set_position = bytes(struct.pack("i", target_pos)).hex().upper()
    set_speed = bytes(struct.pack("i", target_speed)).hex().upper()
    gripping_force = "00000000"
    command = {
        "inst": dummy.plc_output,
        "value": control_double_word + set_position + set_speed + gripping_force,
    }
    before = dummy.get_actual_position()
    dummy.update(command)
    after = dummy.get_actual_position()
    assert after < before  # we are decreasing
    assert after == pytest.approx(before + target_pos, rel=1e-3)
    assert dummy.get_status_bit(bit=13) == 1  # position reached
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_updates_internal_state_when_moving():
    dummy = Dummy()
    dummy.acknowledge()

    # Move
    target_pos = 10340
    target_speed = 5000
    assert pytest.approx(dummy.get_actual_position()) != target_pos
    dummy.move(target_pos=target_pos, target_speed=target_speed)
    assert pytest.approx(dummy.get_actual_position()) == target_pos


def test_dummy_performs_break_test():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.set_control_bit(bit=30, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_performs_fast_stop():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.set_control_bit(bit=0, value=False)  # fail-safe behavior
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=7) == 1  # error
    assert dummy.get_status_error() == "D9"  # ERR_FAST_STOP


def test_dummy_performs_controlled_stop():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.set_control_bit(bit=0, value=True)
    dummy.set_control_bit(bit=1, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=13) == 1  # position reached
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_supports_manual_release():
    dummy = Dummy()

    # Reject when not in error state
    dummy.set_status_bit(bit=7, value=False)  # clear error
    dummy.set_status_diagnostics("00")
    dummy.set_control_bit(bit=5, value=True)  # release
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=8) == 0  # not released

    # Accept when in error state
    dummy.set_control_bit(bit=0, value=False)  # trigger fast stop
    dummy.process_control_bits()
    dummy.set_control_bit(bit=5, value=True)  # release
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=8) == 1  # released for manual movement


def test_dummy_supports_prepare_for_shutdown():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.set_control_bit(bit=3, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=2) == 1  # ready for shutdown


def test_dummy_supports_release_workpiece():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.set_control_bit(bit=11, value=True)
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed
    assert dummy.get_status_bit(bit=13) == 1  # position reached
    assert dummy.get_status_bit(bit=14) == 0  # pre-grip started
    assert dummy.get_status_bit(bit=12) == 0  # workpiece gripped
    assert dummy.get_status_bit(bit=17) == 0  # wrong workpiece gripped


def test_dummy_supports_softreset():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.start()  # fake some system uptime
    initial = dummy.get_system_uptime()
    time.sleep(1.5)
    later = dummy.get_system_uptime()
    time.sleep(1.5)
    dummy.set_control_bit(bit=4, value=True)
    dummy.process_control_bits()
    after_reset = dummy.get_system_uptime()
    dummy.stop()

    assert initial < later
    assert after_reset < later  # restart resets the uptime


def test_dummy_supports_grip():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.set_control_bit(bit=12, value=True)  # grip workpiece
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=12) == 1  # workpiece gripped
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed


def test_dummy_moves_when_gripping():
    dummy = Dummy()
    dummy.acknowledge()

    # Move outward
    dummy.set_control_bit(bit=12, value=True)  # grip workpiece
    dummy.set_control_bit(bit=7, value=True)  # move outward
    dummy.process_control_bits()
    assert dummy.get_actual_position() == dummy.max_position

    # Move inward
    dummy.set_control_bit(bit=12, value=True)
    dummy.set_control_bit(bit=7, value=False)
    dummy.process_control_bits()
    assert dummy.get_actual_position() == dummy.min_position


def test_dummy_supports_grip_at_position():
    dummy = Dummy()
    dummy.acknowledge()
    dummy.set_control_bit(bit=16, value=True)  # grip workpiece at expected position
    dummy.process_control_bits()
    assert dummy.get_status_bit(bit=12) == 1  # workpiece gripped
    assert dummy.get_status_bit(bit=4) == 1  # command successfully processed
    assert dummy.get_status_bit(bit=31) == 1  # GPE activated


def test_dummy_handles_events():
    dummy = Dummy()
    assert dummy.reachable

    events = {}
    assert dummy.handle_events(events)

    # Connection loss
    duration = 0.2
    buffer = 0.5
    events = {"lose_connection_sec": duration}
    for _ in range(3):
        assert dummy.handle_events(events)
        assert not dummy.reachable
        time.sleep(duration + buffer)
        assert dummy.reachable
