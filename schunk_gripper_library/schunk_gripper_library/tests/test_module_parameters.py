from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver


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
