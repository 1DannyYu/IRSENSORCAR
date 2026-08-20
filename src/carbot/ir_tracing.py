"""Yahboom 4-channel IR tracing sensor with a normalized black/white reading.

The module (inventory photos 041/042) exposes four independent digital outputs,
one per channel. This driver reads them through an injectable GPIO object
(``RPi.GPIO`` on the Raspberry Pi, a fake in unit tests) and normalizes every
channel to the same convention used by the line-following code:

    normalized 1  ->  black line under the channel
    normalized 0  ->  white surface under the channel

Raw comparator polarity is not guaranteed across boards, so each channel is
mapped through an ``invert`` set of channel indices. The default assumes the
Yahboom module reports HIGH over black and LOW over white; if a channel reads
0 while over black, pass that channel's index to ``invert``.

Wiring is intentionally not hardcoded here: the caller passes the four BCM GPIO
numbers in channel order. See docs/hardware/ir-tracing-sensor.md for the
planned wiring and the pin-conflict warning.
"""

from __future__ import annotations

from collections.abc import Collection, Sequence
from typing import Protocol

#: Raw GPIO level assumed to mean "black line" when a channel is not inverted.
BLACK_IS_HIGH = True


class Gpio(Protocol):
    """Minimal subset of RPi.GPIO used by :class:`IRTracingSensor`."""

    def input(self, pin: int) -> int: ...


class IRTracingSensor:
    """N-channel IR line tracer with a uniform 1=black / 0=white reading.

    ``channels`` are BCM GPIO numbers in physical channel order (Out1..OutN).
    ``invert`` holds the *channel indices* whose raw polarity is opposite the
    default (e.g. ``invert={0}`` when channel 1 reports LOW over black).
    """

    def __init__(
        self,
        channels: Sequence[int],
        gpio: Gpio,
        invert: Collection[int] = (),
    ) -> None:
        if len(channels) == 0:
            raise ValueError("at least one channel is required")
        self.channels = tuple(channels)
        self._gpio = gpio
        unknown = set(invert) - set(range(len(self.channels)))
        if unknown:
            raise ValueError(f"invert indices out of range: {sorted(unknown)}")
        self.invert = frozenset(invert)

    def raw(self) -> tuple[int, ...]:
        """Raw GPIO levels (1=HIGH, 0=LOW) in channel order — for calibration."""
        return tuple(self._gpio.input(pin) for pin in self.channels)

    def read(self) -> tuple[int, ...]:
        """Normalized readings in channel order: 1 = black line, 0 = white."""
        normalized: list[int] = []
        for index, level in enumerate(self.raw()):
            value = 1 if level else 0
            if index in self.invert:
                value = 1 - value
            normalized.append(value)
        return tuple(normalized)
