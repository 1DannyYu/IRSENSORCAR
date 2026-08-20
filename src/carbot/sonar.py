"""HC-SR04 ultrasonic distance measurement with injectable GPIO and clock.

The example scripts 09, 10 and 11 previously duplicated the same TRIG/ECHO
timing loop (each with its own copy of ``measure``). This module is the single
shared implementation. The GPIO object, sleep and clock are injected so the
timeout paths can be unit-tested without hardware; on the Raspberry Pi the
caller passes ``RPi.GPIO`` after its own ``setmode``/``setup`` calls.

Wiring (verified): VCC=Pin 2, GND=Pin 9, TRIG=Pin 11 (BCM 17), ECHO=Pin 13
(BCM 27) via a 2.2k/1k voltage divider. See
docs/hardware/hc-sr04-ultrasonic-sensor.md.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Protocol

SPEED_OF_SOUND_CM_S = 34300.0  # cm/s
DEFAULT_TIMEOUT_S = 0.5
TRIG_SETTLE_S = 0.06  # keep TRIG low before the pulse
TRIG_PULSE_S = 1e-5  # 10 us trigger pulse


class Gpio(Protocol):
    """Minimal subset of RPi.GPIO used by :class:`Sonar`."""

    def output(self, pin: int, value: object) -> None: ...
    def input(self, pin: int) -> int: ...


class Sonar:
    """One HC-SR04 transceiver pair with a measured echo-pulse timeout."""

    def __init__(
        self,
        trig_pin: int,
        echo_pin: int,
        gpio: Gpio,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self._gpio = gpio
        self.timeout_s = timeout_s
        self._sleep = sleep
        self._clock = clock

    def measure(self) -> float | None:
        """One TRIG/ECHO cycle; distance in cm, or None on timeout.

        Returns ``None`` when either edge of the echo pulse never arrives
        within ``timeout_s`` (no target, disconnected echo wire, or divider
        fault) — callers treat that as "no reading", never as distance 0.
        """
        self._gpio.output(self.trig_pin, False)
        self._sleep(TRIG_SETTLE_S)
        self._gpio.output(self.trig_pin, True)
        self._sleep(TRIG_PULSE_S)
        self._gpio.output(self.trig_pin, False)

        t0 = self._clock()
        while self._gpio.input(self.echo_pin) == 0:
            if self._clock() - t0 > self.timeout_s:
                return None
        pulse_start = self._clock()
        while self._gpio.input(self.echo_pin) == 1:
            if self._clock() - pulse_start > self.timeout_s:
                return None
        return distance_from_pulse(self._clock() - pulse_start)

    def measure_nearest(self, trials: int = 3) -> float | None:
        """Nearest of ``trials`` readings, or ``None`` when none return a value.

        ``None`` means "cannot confirm clear" — the blind zone below roughly
        20 cm, or a fault — and callers must treat it as an obstacle, never as
        free space. The first patrol read it as clear and drove into a wall.

        The nearest reading wins rather than the mean because a spurious long
        reading must not average away a real close one.
        """
        if trials < 1:
            raise ValueError("trials must be at least 1")
        values = [d for d in (self.measure() for _ in range(trials)) if d is not None]
        return min(values) if values else None


def distance_from_pulse(pulse_s: float) -> float:
    """Echo pulse width in seconds -> one-way distance in cm."""
    return pulse_s * SPEED_OF_SOUND_CM_S / 2.0
