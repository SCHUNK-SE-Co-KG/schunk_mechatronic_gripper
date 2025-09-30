from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver
import struct


def test_driver_knows_readable_and_writable_module_parameters():
    driver = Driver()

    for params in [driver.readable_parameters, driver.writable_parameters]:
        # Check shape
        assert isinstance(params, dict)
        assert all(
            isinstance(key, str) and isinstance(value, dict)
            for key, value in params.items()
        )
        for value in params.values():
            assert "registers" in value and type(value["registers"]) is int
            assert "type" in value and type(value["type"]) is str


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

            # Ethernet-based grippers don't have a baudrate nor slave id
            if host and key in ["0x11A0", "0x11A8"]:
                continue

            # Modbus grippers don't have a mac address
            if not host and key in ["0x1138"]:
                continue

            result = driver.read_module_parameter(key)

            # Two bytes per register
            assert len(result) == value["registers"] * 2, f"host: {host}, param: {key}"
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

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(host=host, port=port, serial_port=serial_port, device_id=12)

        # Data arguments must have the correct sizes for each parameter.
        # To make it easy, we just write back the current values.
        for key, value in driver.writable_parameters.items():
            data = driver.read_module_parameter(param=key)
            if not data:
                byte_size = value["registers"] * 2
                data = bytearray(bytes.fromhex("00" * byte_size))
            assert driver.write_module_parameter(key, data)

        # Reject empty data
        for key, value in driver.writable_parameters.items():
            data = bytearray()
            assert not driver.write_module_parameter(key, data)
        driver.disconnect()


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
        for key, value in driver.module_parameters.items():
            assert value is None, f"key: {key}"


@skip_without_gripper
def test_driver_offers_clearing_internal_module_parameters():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=None,
        )

        assert driver.clear_module_parameters()
        for key, value in driver.module_parameters.items():
            assert value is None, f"key: {key}"

        # Repetitive
        for _ in range(3):
            assert driver.clear_module_parameters()
            for key, value in driver.module_parameters.items():
                assert value is None, f"key: {key}"


@skip_without_gripper
def test_driver_offers_decoding_module_parameters():
    driver = Driver()

    # When not connected
    values, value_type = driver.decode_module_parameter(
        data=bytearray(bytes.fromhex("aabb")), param="0x0500"
    )
    assert values == ()
    assert value_type == ""

    # When connected
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=None,
        )

        # Valid parameters
        for key, value in driver.readable_parameters.items():
            if value["type"].startswith("struct"):
                continue  # not supported yet

            # Ethernet-based grippers don't have a baudrate nor slave id
            if host and key in ["0x11A0", "0x11A8"]:
                continue

            # Modbus grippers don't have a mac address
            if not host and key in ["0x1138"]:
                continue

            # Empty message buffer in simulation
            if key == "0x0130":
                continue

            data = driver.read_module_parameter(param=key)
            values, value_type = driver.decode_module_parameter(data=data, param=key)
            assert len(values) >= 1, f"host: {host}, param: {key}"
            assert value_type == value["type"], f"param: {key}"

        driver.disconnect()


def test_driver_keeps_correct_order_when_decoding_arrays():
    driver = Driver()

    float_array = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    uint16_array = (301, 302, 303, 304, 305, 306)
    uint32_array = (100001, 100002, 100003, 100004, 100005, 100006)

    fake_parameters = {
        "0x1": {"type": "float[6]"},
        "0x2": {"type": "uint16[6]"},
        "0x3": {"type": "uint32[6]"},
    }
    driver.readable_parameters = fake_parameters
    driver.connected = True

    for endianness, fieldbus in zip(["<", ">"], ["", "PN"]):
        driver.fieldbus = fieldbus

        # Float arrays
        data = bytearray()
        for num in float_array:
            data.extend(struct.pack(f"{endianness}f", num))
        values, _ = driver.decode_module_parameter(data, param="0x1")
        assert values == float_array, f"endianness: {endianness}, fieldbus: {fieldbus}"

        # Uint16 arrays
        data = bytearray()
        for num in uint16_array:
            data.extend(struct.pack(f"{endianness}H", num))
        values, _ = driver.decode_module_parameter(data, param="0x2")
        assert values == uint16_array, f"endianness: {endianness}, fieldbus: {fieldbus}"

        # Uint32 arrays
        data = bytearray()
        for num in uint32_array:
            data.extend(struct.pack(f"{endianness}I", num))
        values, _ = driver.decode_module_parameter(data, param="0x3")
        assert values == uint32_array, f"endianness: {endianness}, fieldbus: {fieldbus}"


@skip_without_gripper
def test_driver_survives_decoding_invalid_module_parameters():
    driver = Driver()

    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=None,
        )

        # Empty data
        values, value_type = driver.decode_module_parameter(
            data=bytearray(), param="0x0500"
        )
        assert values == ()
        assert value_type == ""

        # Invalid parameters
        invalid_params = ["", "0500", "-1", "0x0"]
        data = bytearray(bytes.fromhex("aabbccdd"))
        for param in invalid_params:
            values, value_type = driver.decode_module_parameter(data=data, param=param)
            assert values == ()
            assert value_type == ""

        driver.disconnect()


@skip_without_gripper
def test_driver_offers_encoding_module_parameters():
    driver = Driver()

    # When not connected
    assert not driver.encode_module_parameter(data=[], param="")

    # Empty arguments
    driver.connected = True
    assert not driver.encode_module_parameter(data=[], param="ok")
    assert not driver.encode_module_parameter(data=[1, 2, 3], param="")
    driver.connected = False

    # Good cases
    for host, port, serial_port in zip(
        ["0.0.0.0", None], [8000, None], [None, "/dev/ttyUSB0"]
    ):
        assert driver.connect(
            host=host,
            port=port,
            serial_port=serial_port,
            device_id=12,
            update_cycle=None,
        )

        # Check the complete round trip of reading and writing parameters
        for param, _ in driver.writable_parameters.items():
            expected = driver.read_module_parameter(param=param)
            values, value_type = driver.decode_module_parameter(
                data=expected, param=param
            )

            # Not supported yet
            if not value_type:
                continue

            encoded_data = driver.encode_module_parameter(data=values, param=param)
            assert encoded_data == expected, f"host: {host}, param: {param}"

        driver.disconnect()


def test_driver_survives_encoding_invalid_module_parameters():
    driver = Driver()
    driver.connected = True

    valid_params = [
        "0x1330",  # bool
        "0x0528",  # float
        "0x11A8",  # uint8
        "0x0380",  # uint16
        "0x11A0",  # uint32
    ]
    invalid_data = [[1.0, 2.0], ["ok", "really?"], [42], [False, True, True]]

    # Sending invalid data to valid parameters shouldn't
    # crash the driver
    for param in valid_params:
        for data in invalid_data:
            driver.encode_module_parameter(data=data, param=param)
