from threading import Event, Thread

import pytest

from schunk_gripper_library.driver import Driver


def configured_driver() -> Driver:
    driver = Driver()
    driver.connected = True
    driver.module_parameters["max_vel"] = 1000
    return driver


def test_move_locks_command_submission_but_not_motion_wait(monkeypatch):
    driver = configured_driver()

    def send_command(*args, **kwargs):
        assert driver.command_lock.locked()
        assert driver.motion_lock.locked()
        return 1

    def wait_for_acknowledgement(*args, **kwargs):
        assert driver.command_lock.locked()
        assert driver.motion_lock.locked()
        return True

    def wait_for_completion(*args, **kwargs):
        assert not driver.command_lock.locked()
        assert driver.motion_lock.locked()
        return {"4": 1, "13": 1}

    monkeypatch.setattr(driver, "_send_cmd", send_command)
    monkeypatch.setattr(driver, "wait_for_status", wait_for_acknowledgement)
    monkeypatch.setattr(driver, "estimate_duration", lambda **kwargs: 0.0)
    monkeypatch.setattr(driver, "wait_for_any_status", wait_for_completion)

    assert driver.move_to_position(100, 100, no_scheduler=True)
    assert not driver.command_lock.locked()
    assert not driver.motion_lock.locked()


def test_move_rejects_concurrent_motion(monkeypatch):
    driver = configured_driver()
    motion_started = Event()
    finish_motion = Event()
    first_result = []

    monkeypatch.setattr(driver, "_send_cmd", lambda *args, **kwargs: 1)
    monkeypatch.setattr(driver, "wait_for_status", lambda **kwargs: True)
    monkeypatch.setattr(driver, "estimate_duration", lambda **kwargs: 0.0)

    def wait_for_completion(*args, **kwargs):
        motion_started.set()
        finish_motion.wait(timeout=1.0)
        return {"4": 1, "13": 1}

    monkeypatch.setattr(driver, "wait_for_any_status", wait_for_completion)

    first_move = Thread(
        target=lambda: first_result.append(
            driver.move_to_position(100, 100, no_scheduler=True)
        )
    )
    first_move.start()
    assert motion_started.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="Another motion command"):
        driver.move_to_position(100, 100, no_scheduler=True)

    finish_motion.set()
    first_move.join(timeout=1.0)
    assert not first_move.is_alive()
    assert first_result == [True]
    assert not driver.motion_lock.locked()


def test_move_releases_locks_when_command_fails(monkeypatch):
    driver = configured_driver()
    monkeypatch.setattr(
        driver,
        "_send_cmd",
        lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("write failed")),
    )

    with pytest.raises(RuntimeError, match="write failed"):
        driver.move_to_position(100, 100, no_scheduler=True)

    assert not driver.command_lock.locked()
    assert not driver.motion_lock.locked()


@pytest.mark.parametrize(
    ("method_name", "args"),
    [
        ("grip", (50,)),
        ("release", ()),
        ("release_for_manual_movement", ()),
        ("brake_test", ()),
        ("start_jogging", (100,)),
        ("twitch_jaws", ()),
    ],
)
def test_motion_methods_reject_when_another_motion_is_active(method_name, args):
    driver = configured_driver()
    driver.motion_lock.acquire()

    with pytest.raises(RuntimeError, match="[Mm]otion.*progress"):
        getattr(driver, method_name)(*args)

    driver.motion_lock.release()


@pytest.mark.parametrize(
    ("method_name", "args"),
    [
        ("prepare_for_shutdown", ()),
        ("soft_reset", ()),
        ("write_param", ("0x0048", bytearray(16))),
    ],
)
def test_other_writes_reject_during_motion(method_name, args):
    driver = configured_driver()
    driver.motion_lock.acquire()

    with pytest.raises(RuntimeError, match="[Mm]otion.*progress"):
        getattr(driver, method_name)(*args)

    driver.motion_lock.release()


def test_jogging_owns_motion_until_stopped(monkeypatch):
    driver = configured_driver()
    monkeypatch.setattr(driver, "_send_cmd", lambda *args, **kwargs: 1)
    monkeypatch.setattr(driver, "send_plc_output", lambda: True)
    monkeypatch.setattr(driver, "wait_for_status", lambda **kwargs: True)

    assert driver.start_jogging(100)
    assert driver.motion_lock.locked()
    assert driver.start_jogging(100)

    assert driver.stop_jogging()
    assert not driver.motion_lock.locked()


def test_failed_stop_jogging_keeps_motion_ownership(monkeypatch):
    driver = configured_driver()
    monkeypatch.setattr(driver, "_send_cmd", lambda *args, **kwargs: 1)
    monkeypatch.setattr(driver, "send_plc_output", lambda: True)
    monkeypatch.setattr(driver, "wait_for_status", lambda **kwargs: True)
    assert driver.start_jogging(100)

    monkeypatch.setattr(driver, "wait_for_status", lambda **kwargs: False)
    assert not driver.stop_jogging()
    assert driver.motion_lock.locked()

    driver.disconnect()
    assert not driver.motion_lock.locked()


@pytest.mark.parametrize("method_name", ["stop", "fast_stop"])
def test_safety_commands_are_allowed_during_motion(monkeypatch, method_name):
    driver = configured_driver()
    driver.motion_lock.acquire()

    def send_command(*args, **kwargs):
        assert driver.command_lock.locked()
        return 1

    monkeypatch.setattr(driver, "_send_cmd", send_command)
    monkeypatch.setattr(driver, "wait_for_status", lambda **kwargs: True)

    assert getattr(driver, method_name)()
    assert driver.motion_lock.locked()
    driver.motion_lock.release()


@pytest.mark.parametrize("method_name", ["stop", "fast_stop"])
def test_safety_command_cancels_motion_wait(monkeypatch, method_name):
    driver = configured_driver()
    motion_waiting = Event()
    result = []

    monkeypatch.setattr(driver, "_send_cmd", lambda *args, **kwargs: 1)
    monkeypatch.setattr(driver, "wait_for_status", lambda **kwargs: True)
    monkeypatch.setattr(driver, "estimate_duration", lambda **kwargs: 10.0)

    original_wait = driver.wait_for_any_status

    def wait_for_completion(*args, **kwargs):
        motion_waiting.set()
        return original_wait(*args, **kwargs)

    monkeypatch.setattr(driver, "wait_for_any_status", wait_for_completion)
    motion = Thread(
        target=lambda: result.append(
            driver.move_to_position(100, 100, no_scheduler=True)
        )
    )
    motion.start()
    assert motion_waiting.wait(timeout=1.0)

    assert getattr(driver, method_name)()
    motion.join(timeout=1.0)

    assert result == [False]
    assert not driver.motion_lock.locked()
