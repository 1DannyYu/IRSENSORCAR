"""Gate A regression tests: HC-SR04 timing and timeout paths.

``Sonar`` is exercised with a fake GPIO whose echo edge times are expressed in
the injected clock's timeline, and a fake clock that advances on every call
(the real ``time.monotonic`` advances on its own). This makes the three
hardware paths — normal echo, never-heard, stuck-high — deterministic and
hardware-free.
"""

from __future__ import annotations

import pytest

from carbot.sonar import SPEED_OF_SOUND_CM_S, Sonar, distance_from_pulse

TRIG = 17
ECHO = 27


class TickClock:
    """Clock that advances by ``tick`` on every read, plus sleep()."""

    def __init__(self, tick: float = 1e-5) -> None:
        self.now = 0.0
        self.tick = tick

    def __call__(self) -> float:
        self.now += self.tick
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class EchoGpio:
    """Echo goes high at ``high_at`` and low again at ``low_at``."""

    def __init__(
        self, clock: TickClock, high_at: float = float("inf"), low_at: float = float("inf")
    ) -> None:
        self._clock = clock
        self.high_at = high_at
        self.low_at = low_at
        self.outputs: list[tuple[int, bool]] = []

    def output(self, pin: int, value: object) -> None:
        self.outputs.append((pin, bool(value)))

    def input(self, pin: int) -> int:
        t = self._clock()
        if t < self.high_at:
            return 0
        if t < self.low_at:
            return 1
        return 0


def _sonar(gpio: EchoGpio, clock: TickClock, timeout_s: float = 0.5) -> Sonar:
    return Sonar(TRIG, ECHO, gpio, timeout_s=timeout_s, sleep=clock.sleep, clock=clock)


def test_distance_from_pulse():
    assert distance_from_pulse(0.001) == pytest.approx(SPEED_OF_SOUND_CM_S * 0.001 / 2)
    assert distance_from_pulse(0.0) == 0.0


def test_measure_returns_distance_for_normal_echo():
    clock = TickClock()
    # Echo rises ~0.1 ms after the 10 us trigger and falls 1 ms later.
    gpio = EchoGpio(clock, high_at=0.06011, low_at=0.06111)
    sonar = _sonar(gpio, clock)
    dist = sonar.measure()
    assert dist is not None
    # Pulse ~1 ms -> ~17.15 cm; timing quantisation is at most 2 ticks.
    assert dist == pytest.approx(
        distance_from_pulse(0.001), abs=2 * clock.tick * SPEED_OF_SOUND_CM_S / 2 + 1e-6
    )


def test_measure_returns_none_when_echo_never_heard():
    clock = TickClock(tick=1e-4)  # coarse clock keeps the test fast
    gpio = EchoGpio(clock, high_at=float("inf"))
    sonar = _sonar(gpio, clock, timeout_s=0.05)
    assert sonar.measure() is None


def test_measure_returns_none_when_echo_stuck_high():
    clock = TickClock(tick=1e-4)
    gpio = EchoGpio(clock, high_at=0.0, low_at=float("inf"))  # high immediately, never falls
    sonar = _sonar(gpio, clock, timeout_s=0.05)
    assert sonar.measure() is None


def test_measure_short_pulse_close_object():
    clock = TickClock()
    gpio = EchoGpio(clock, high_at=0.06011, low_at=0.06041)  # 0.3 ms -> ~5.1 cm
    sonar = _sonar(gpio, clock)
    dist = sonar.measure()
    assert dist is not None
    assert dist == pytest.approx(distance_from_pulse(0.0003), abs=0.1)


class ScriptedSonar(Sonar):
    """Returns a fixed sequence of readings, so measure_nearest is testable alone."""

    def __init__(self, readings: list[float | None]) -> None:
        self.readings = list(readings)
        self.calls = 0

    def measure(self) -> float | None:
        self.calls += 1
        return self.readings.pop(0) if self.readings else None


def test_measure_nearest_returns_the_closest_reading():
    """A spurious long reading must not average away a real close one."""
    sonar = ScriptedSonar([80.0, 22.0, 75.0])
    assert sonar.measure_nearest() == 22.0
    assert sonar.calls == 3


def test_measure_nearest_ignores_missing_readings():
    sonar = ScriptedSonar([None, 41.0, None])
    assert sonar.measure_nearest() == 41.0


def test_measure_nearest_is_none_when_every_reading_fails():
    """All-None must stay None: the patrol treats it as an obstacle, not as 0 cm."""
    assert ScriptedSonar([None, None, None]).measure_nearest() is None


def test_measure_nearest_honours_the_trial_count():
    sonar = ScriptedSonar([50.0, 30.0, 10.0])
    assert sonar.measure_nearest(trials=2) == 30.0
    assert sonar.calls == 2


def test_measure_nearest_rejects_a_zero_trial_count():
    with pytest.raises(ValueError, match="at least 1"):
        ScriptedSonar([50.0]).measure_nearest(trials=0)
