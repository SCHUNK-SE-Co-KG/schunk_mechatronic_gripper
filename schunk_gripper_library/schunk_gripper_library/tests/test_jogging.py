from schunk_gripper_library.utility import skip_without_gripper
from schunk_gripper_library.driver import Driver


@skip_without_gripper
def test_jog():
    fast_test_vel = 22000
    slow_test_vel = 50
    driver = Driver()

    # Jog mode communication over TCP is not testable yet.
    for host, port, serial_port in zip(
        [None],
        [None],
        ["/dev/ttyUSB0"],
    ):
        try:
            # not connected
            assert not driver.jog_positive(velocity=fast_test_vel)

            assert not driver.jog_negative(velocity=fast_test_vel)

            # after connection
            assert driver.connect(
                host=host, port=port, serial_port=serial_port, device_id=12
            )

            assert driver.acknowledge()

            assert driver.jog_positive(
                velocity=fast_test_vel
            ), driver.get_status_diagnostics()

            assert driver.reset_jog(), driver.get_status_diagnostics()

            assert driver.jog_negative(
                velocity=slow_test_vel
            ), driver.get_status_diagnostics()

            assert driver.reset_jog(), driver.get_status_diagnostics()

            assert driver.jog_positive(
                velocity=fast_test_vel, timeout=10
            ), driver.get_status_diagnostics()

            assert driver.reset_jog(), driver.get_status_diagnostics()

        finally:
            assert driver.disconnect()
