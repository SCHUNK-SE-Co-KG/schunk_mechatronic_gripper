from schunk_gripper_library.driver import Driver
from schunk_gripper_library.utility import skip_without_gripper
import pytest
import struct
from threading import Timer


@skip_without_gripper
def test_driver_implements_connect_and_disconnect():
    driver = Driver()

    # Modbus
    device_id = 12  # SChUNK default
    assert driver.connect(serial_port="/dev/ttyUSB0", device_id=device_id)
    assert driver.mb_device_id == device_id
    assert driver.mb_client.connected
    assert driver.disconnect()
    assert not driver.mb_client.connected

    # TCP/IP
    host = "0.0.0.0"
    port = 8000
    assert driver.connect(host=host, port=port)
    assert driver.host == host
    assert driver.port == port
    assert driver.web_client is not None
    assert driver.disconnect()
    assert driver.web_client is None


def test_driver_rejects_invalid_connection_arguments():
    driver = Driver()

    # Modbus
    assert not driver.connect(serial_port=42)
    assert not driver.connect(serial_port=42, device_id=12)
    assert not driver.connect(serial_port="/dev/ttyUSB0")  # missing device_id
    assert not driver.connect(
        serial_port="/dev/ttyUSB0", device_id="12"
    )  # wrong device_id type
    assert not driver.connect(serial_port="not ok", device_id=-1)
    assert not driver.connect(serial_port="not ok", device_id=0)
    assert not driver.connect(
        serial_port="non-existent", device_id=12
    )  # non-existent port

    # TCP/IP
    assert not driver.connect(host="0.0.0.0", port=-10)
    assert not driver.connect(host="a.b.c.d")
    assert not driver.connect(host="1.3.3")
    assert not driver.connect(host="some arbitrary string #!?")

    # Wrong update cycles
    invalid_cycles = [-1, -0.001, 0.0, 0, 0.0001]
    for cycle in invalid_cycles:
        assert not driver.connect(update_cycle=cycle)


@skip_without_gripper
def test_driver_supports_repeated_connects_and_disconnects():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        for _ in range(3):
            assert driver.connect(
                host=host, port=port, serial_port=serial_port, device_id=12
            )
            assert driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_switching_between_protocols():
    driver = Driver()
    for _ in range(3):
        assert driver.connect(host="0.0.0.0", port=8000)
        assert driver.disconnect()
        assert driver.connect(port="/dev/ttyUSB0", device_id=12)
        assert driver.disconnect()


@skip_without_gripper
def test_driver_rejects_new_connect_without_disconnect():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        assert driver.connect(
            host=host, port=port, serial_port=serial_port, device_id=12
        )
        assert not driver.connect(host=host, port=port, device_id=12)
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_disconnects():
    driver = Driver()
    assert driver.disconnect()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for _ in range(3):
            assert driver.disconnect()


@skip_without_gripper
def test_driver_implements_sending_plc_output():
    driver = Driver()
    driver.connect(port="/dev/ttyUSB0", device_id=12)
    assert driver.send_plc_output()
    driver.disconnect()
    driver.connect(host="0.0.0.0", port=8000)
    assert driver.send_plc_output()
    driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_sending():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for _ in range(5):
            assert driver.send_plc_output()
        driver.disconnect()


@skip_without_gripper
def test_driver_rejects_sending_when_not_connected():
    driver = Driver()
    assert not driver.send_plc_output()


@skip_without_gripper
def test_driver_implements_receiving_plc_input():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        before = driver.get_plc_input()
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.receive_plc_input()
        after = driver.get_plc_input()
        assert after != before
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_repeated_receiving():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for _ in range(5):
            assert driver.receive_plc_input()
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_waiting_for_desired_status():
    driver = Driver()
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)

        # Timeout for bitsets that don't come
        impossible_bits = {"0": 1, "7": 1}  # operational + error
        assert not driver.wait_for_status(bits=impossible_bits, timeout_sec=0.1)

        # Default timeout works
        impossible_bits = {"0": 1, "7": 1}
        assert not driver.wait_for_status(bits=impossible_bits)

        # Success when bits match
        matching_bits = {"0": 0, "7": 1}  # error on startup
        assert driver.wait_for_status(bits=matching_bits, timeout_sec=0.1)

        # Fails but survives invalid bits
        invalid_bits = {"33": 1, "-1": 0}
        assert not driver.wait_for_status(bits=invalid_bits, timeout_sec=0.1)

        # Fails but survives invalid timeouts
        matching_bits = {"0": 0, "7": 1}
        invalid_timeouts = [0.0, 0, -1.5]
        for timeout in invalid_timeouts:
            assert not driver.wait_for_status(bits=matching_bits, timeout_sec=timeout)

        # Fails with empty bits
        assert not driver.wait_for_status(bits={})

        # Finish
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_sending_and_receiving_after_switching_protocols():
    driver = Driver()
    for _ in range(3):
        for host, port, serial_port in zip(
            ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
        ):
            driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
            assert driver.send_plc_output()
            assert driver.receive_plc_input()
            driver.disconnect()


@skip_without_gripper
def test_driver_supports_reading_module_parameters():
    driver = Driver()

    # Can't read when not connected
    for param in driver.readable_parameters.keys():
        assert not driver.read_module_parameter(param=param)

    # Reject unsupported parameters
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        for param in ["not-ok", "?!#" "-1"]:
            driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
            assert not driver.read_module_parameter(param)
        driver.disconnect()

    # All params have the correct size
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for key, value in driver.readable_parameters.items():
            result = driver.read_module_parameter(key)
            assert len(result) == value["registers"] * 2  # two bytes per register
        driver.disconnect()


@skip_without_gripper
def test_driver_supports_writing_module_parameters():
    driver = Driver()

    # Can't write when not connected
    data = bytearray()
    assert not driver.write_module_parameter("0x0048", data)

    # Reject unsupported parameters
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        for param in ["not-existent", "1234" "0x0"]:
            driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
            assert not driver.write_module_parameter(param, bytearray())
        driver.disconnect()

    # Data arguments must have the correct sizes
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)

        # Correct
        for key, value in driver.writable_parameters.items():
            byte_size = value["registers"] * 2
            data = bytearray(bytes.fromhex("00" * byte_size))
            assert driver.write_module_parameter(key, data)

        # Incorrect
        for key, value in driver.writable_parameters.items():
            data = bytearray()
            assert not driver.write_module_parameter(key, data)
        driver.disconnect()


@skip_without_gripper
def test_connected_driver_has_associated_module_and_fieldbus():
    driver = Driver()

    # empty on startup
    assert not driver.module
    assert not driver.fieldbus

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.module in driver.valid_module_types.values()
        assert driver.fieldbus in driver.valid_fieldbus_types.values()

        # empty after disconnect
        driver.disconnect()
        assert not driver.module
        assert not driver.fieldbus


def test_driver_can_check_for_gpe_support():
    driver = Driver()
    assert not driver.gpe_available()  # when unconnected

    types_with_gpe = [
        "EGU_50_M_B",
        "EGK_25_M_B",
        "EZU_40_M_B",
        "xyz_00_M_B",
    ]
    types_without_gpe = [
        "EGU_80_N_B",
        "EGK_40_N_B",
        "EGU_70_N_SD",
        "EGH",
        "UG4_DIO_80",
        "_",
        "123x",
        "?#\0\n",
        "EGU_40_m_B",
        "0x0048",
        "EGU_M_40_B",
        "_EGU_40_M_B",
        "MMM_not_ok",
    ]

    for type in types_with_gpe:
        driver.module = type
        assert driver.gpe_available()

    for type in types_without_gpe:
        driver.module = type
        assert not driver.gpe_available()


def test_driver_estimates_duration_of_positioning_operations():
    driver = Driver()

    def set_actual_position(position: int) -> None:
        driver.plc_input_buffer[4:8] = bytes(struct.pack("i", position))

    invalid_combinations = [
        # Invalid velocity arguments will lead to an invalid error,
        # and should return at once
        {
            "args": {"position_abs": 30500, "velocity": 0},
            "should_take": 0,
        },
        {
            "args": {"position_abs": 30500, "velocity": -123},
            "should_take": 0,
        },
        # Invalid force arguments
        {
            "args": {"force": 75.0},
            "should_take": 0.0,
        },
        {
            "args": {"force": -70},
            "should_take": 0.0,
        },
        {
            "args": {"force": 270},
            "should_take": 0.0,
        },
    ]
    for entry in invalid_combinations:
        duration_sec = driver.estimate_duration(**entry["args"])
        assert (
            pytest.approx(duration_sec) == entry["should_take"]
        ), f"args: {entry['args']}"

    valid_combinations = [
        # Valid position and velocity arguments
        {
            "start_pos": 0,
            "args": {"position_abs": 10000, "velocity": 5000},
            "should_take": 2.0,
        },
        {
            "start_pos": 0,
            "args": {"position_abs": -10000, "velocity": 5000},
            "should_take": 2.0,
        },
        {
            "start_pos": 5000,
            "args": {"position_abs": 10000, "velocity": 5000},
            "should_take": 1.0,  # when increasing
        },
        {
            "start_pos": 15000,
            "args": {"position_abs": 10000, "velocity": 5000},
            "should_take": 1.0,  # when decreasing
        },
        {
            "start_pos": 1000,
            "args": {"position_abs": -1000, "velocity": 1000},
            "should_take": 2.0,  # crossing zero
        },
        {
            "start_pos": 15366,
            "args": {"position_abs": 15366, "velocity": 12345},
            "should_take": 0.0,  # Already there
        },
    ]

    for entry in valid_combinations:
        if "start_pos" in entry:
            set_actual_position(entry["start_pos"])
        duration_sec = driver.estimate_duration(**entry["args"])
        assert pytest.approx(duration_sec) == entry["should_take"], f"entry: {entry}"


@skip_without_gripper
def test_driver_estimates_duration_of_grip_operations():
    driver = Driver()

    # We need gripper min and max positions.
    # Connect without update cycle to not interfere while
    # manually setting actual positions.
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12, update_cycle=None)
    max_pos = driver.module_parameters["max_pos"]
    min_pos = driver.module_parameters["min_pos"]

    def set_actual_position(position: int) -> None:
        driver.plc_input_buffer[4:8] = bytes(struct.pack("i", position))

    # Fully closed
    set_actual_position(min_pos)
    forces = [50, 75, 100]
    for force in forces:
        assert pytest.approx(driver.estimate_duration(force=force)) == 0.0

    # Fully open
    set_actual_position(max_pos)
    forces = [50, 75, 100]
    for force in forces:
        assert pytest.approx(driver.estimate_duration(force=force, outward=True)) == 0.0

    # Half open/closed
    half = int(0.5 * (max_pos - min_pos))
    set_actual_position(half)
    for outward in [False, True]:
        slow = driver.estimate_duration(force=50, outward=outward)
        middle = driver.estimate_duration(force=75, outward=outward)
        fast = driver.estimate_duration(force=100, outward=outward)
        assert fast < middle < slow

    # Wide range in mm steps
    start = min_pos + 5000
    stop = max_pos - 5000
    for pos in range(start, stop, 1000):
        set_actual_position(pos)
        for outward in [False, True]:
            slow = driver.estimate_duration(force=50, outward=outward)
            middle = driver.estimate_duration(force=75, outward=outward)
            fast = driver.estimate_duration(force=100, outward=outward)
            assert fast < middle < slow

    # Cleanup
    driver.disconnect()


@skip_without_gripper
def test_driver_estimates_duration_of_release():
    driver = Driver()
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12, update_cycle=None)
    expected = (
        driver.module_parameters["wp_release_delta"]
        / driver.module_parameters["max_vel"]
    )
    estimated = driver.estimate_duration(release=True)
    assert pytest.approx(estimated) == expected


@skip_without_gripper
def test_connected_driver_has_module_parameters():
    driver = Driver()
    params = [
        "module_type",
        "max_grp_vel",
        "max_vel",
        "min_pos",
        "max_pos",
        "wp_release_delta",
        "fieldbus_type",
        "max_phys_stroke",
        "max_grp_force",
        "serial_no_txt",
        "sw_version_txt",
    ]
    for param in params:
        assert param in driver.module_parameters
        assert driver.module_parameters[param] is None

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        for param in params:
            assert driver.module_parameters[param] is not None

        driver.disconnect()
        for param in params:
            assert driver.module_parameters[param] is None


@skip_without_gripper
def test_driver_offers_updating_internal_module_parameters():
    driver = Driver()

    # Parameters are updated when connected
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)
        assert driver.update_module_parameters()
        for key, value in driver.module_parameters.items():
            assert value is not None, f"key: {key}"

        # Repetitive
        for _ in range(3):
            assert driver.update_module_parameters()
            for key, value in driver.module_parameters.items():
                assert value is not None, f"key: {key}"

        # Values are reset when disconnected
        driver.disconnect()
        assert driver.update_module_parameters()
        for key, value in driver.module_parameters.items():
            assert value is None, f"key: {key}"


def test_driver_offers_waiting_until_error():
    driver = Driver()
    error_bit = 7

    # Without error
    driver._set_status_bit(bit=error_bit, value=False)
    assert not driver.error_in(duration_sec=0.1)

    # With invalid duration
    invalid_durations = [-1.0, 0, 0.0, None]
    for duration in invalid_durations:
        assert not driver.error_in(duration_sec=duration)

    # With error from the beginning
    driver._set_status_bit(bit=error_bit, value=True)
    assert driver.error_in(duration_sec=1.0)

    # With error in between
    driver._set_status_bit(bit=error_bit, value=False)

    def fail() -> None:
        driver._set_status_bit(bit=error_bit, value=True)

    Timer(interval=0.5, function=fail).start()
    assert driver.error_in(duration_sec=1.0)


@skip_without_gripper
def test_driver_offers_showing_gripper_specification():
    expected_keys = [
        "max_stroke",
        "max_speed",
        "max_force",
        "serial_number",
        "firmware_version",
        "device_id",
        "ip_address",
    ]

    # Not connected
    driver = Driver()
    assert driver.show_specification() == {}

    # Modbus connection
    driver.connect(serial_port="/dev/ttyUSB0", device_id=12)
    spec = driver.show_specification()
    assert spec != {}
    for key in expected_keys:
        assert key in spec

    assert spec["device_id"] == 12
    assert spec["ip_address"] == ""
    driver.disconnect()

    # Ethernet connection
    driver = Driver()
    driver.connect(host="0.0.0.0", port=8000)
    spec = driver.show_specification()
    assert spec != {}
    for key in expected_keys:
        assert key in spec

    assert spec["device_id"] == 0
    assert spec["ip_address"] == "0.0.0.0"
    driver.disconnect()


@skip_without_gripper
def test_connected_driver_has_associated_gripper():
    driver = Driver()
    assert not driver.gripper  # empty on startup

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)

        # We have this convention: {EGU|EGK|EZU}_{xx}_{PN|EI|EC|MB}_{M|N}_{B|SD}
        parts = driver.gripper.split("_")
        print(parts)
        assert parts[0] in ["EGU", "EGK", "EZU"]
        assert int(parts[1])
        assert parts[2] in ["PN", "EI", "EC", "MB"]
        assert parts[3] in ["M", "N"]
        assert parts[4] in ["B", "SD"]

        driver.disconnect()
        assert not driver.gripper  # empty after disconnect


def test_driver_offers_method_for_composing_gripper_type():
    driver = Driver()

    invalid_combinations = [
        {"args": {"module": "EGU_50_", "fieldbus": "EC"}},
        {"args": {"module": "EGU_50_M_B", "fieldbus": "AA"}},
        {"args": {"module": "0x?|^$%", "fieldbus": "PN"}},
    ]
    for entry in invalid_combinations:
        gripper_type = driver.compose_gripper_type(**entry["args"])
        assert gripper_type == ""

    valid_combinations = [
        {
            "args": {"module": "EGU_50_M_B", "fieldbus": "EC"},
            "expected": "EGU_50_EC_M_B",
        },
        {
            "args": {"module": "EGU_80_N_SD", "fieldbus": "EI"},
            "expected": "EGU_80_EI_N_SD",
        },
        {
            "args": {"module": "EZU_35_N_B", "fieldbus": "MB"},
            "expected": "EZU_35_MB_N_B",
        },
        {
            "args": {"module": "EZU_40_N_SD", "fieldbus": "MB"},
            "expected": "EZU_40_MB_N_SD",
        },
        {
            "args": {"module": "EGK_25_M_B", "fieldbus": "MB"},
            "expected": "EGK_25_MB_M_B",
        },
        {
            "args": {"module": "EGK_50_N_B", "fieldbus": "PN"},
            "expected": "EGK_50_PN_N_B",
        },
    ]
    for entry in valid_combinations:
        gripper_type = driver.compose_gripper_type(**entry["args"])
        assert gripper_type == entry["expected"]
