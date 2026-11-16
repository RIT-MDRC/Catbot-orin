import signal
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from catbot_launcher.zed_watchdog import Health, green_fraction, stop_group


def image(pixel=(0, 255, 0), encoding="rgb8", padding=0):
    return SimpleNamespace(
        width=4,
        height=3,
        step=4 * len(pixel) + padding,
        data=bytes(pixel * 4 + (0,) * padding) * 3,
        encoding=encoding,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
    )


@pytest.mark.parametrize(
    "encoding,pixel,expected",
    [
        ("rgb8", (0, 255, 0), 1),
        ("bgr8", (0, 255, 0), 1),
        ("bgra8", (0, 255, 0, 255), 1),
        ("rgba8", (0, 255, 0, 255), 1),
        ("rgb8", (255, 0, 0), 0),
        ("rgb8", (50, 50, 50), 0),
        ("bgr8", (0, 0, 255), 0),
        ("mono8", (255,), None),
    ],
)
def test_pixels(encoding, pixel, expected):
    assert green_fraction(image(pixel, encoding, padding=8)) == expected


def test_malformed():
    msg = image()
    msg.data = b""
    assert green_fraction(msg) is None


def test_health_persistence_and_reset():
    health = Health(0)
    health.observe(image(), 1)
    assert health.failure(5) is None
    assert "green" in health.failure(6)
    health.observe(image((20, 30, 40)), 7)
    assert health.failure(8) is None


@pytest.mark.parametrize(
    "observed,now,expected",
    [
        (None, 59, None),
        (None, 60, "no first image"),
        (1, 11, "image stream stopped"),
    ],
)
def test_timeouts(observed, now, expected):
    health = Health(0)
    if observed is not None:
        health.observe(image((20, 30, 40)), observed)
    assert health.failure(now) == expected


def test_stuck_timestamp():
    health = Health(0)
    health.observe(image((20, 30, 40)), 1)
    health.observe(image((20, 30, 40)), 11)
    assert health.failure(11) == "image timestamp stopped advancing"
    msg = image((20, 30, 40))
    msg.header.stamp.sec = 2
    health.observe(msg, 12)
    assert health.failure(12) is None


def test_stop_owned_process_group(monkeypatch):
    process = Mock(pid=123)
    calls = []

    def killpg(pid, sig):
        calls.append((pid, sig))
        if sig == 0:
            raise ProcessLookupError

    monkeypatch.setattr("catbot_launcher.zed_watchdog.os.killpg", killpg)
    stop_group(process, timeout=1)
    assert calls == [(123, signal.SIGINT), (123, 0), (123, 0)]
    process.wait.assert_called_once_with(timeout=1)


def test_cleanup_refuses_restart_if_group_survives(monkeypatch):
    process = Mock(pid=123)
    monkeypatch.setattr("catbot_launcher.zed_watchdog.os.killpg", lambda *args: None)
    with pytest.raises(RuntimeError, match="refusing overlapping launch"):
        stop_group(process, timeout=0)
