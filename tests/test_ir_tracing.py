"""IRTracingSensor normalization: every channel reports 1 on black, 0 on white.

``IRTracingSensor`` is exercised with a fake GPIO so the raw-level to
normalized-value mapping is deterministic and hardware-free, matching the
pattern used by ``test_sonar.py``.
"""

from __future__ import annotations

import pytest

from carbot.ir_tracing import IRTracingSensor

PINS = (17, 27, 22, 23)  # Out1..Out4 — planned wiring, order matters here only


class FakeGpio:
    """input() returns pre-set raw levels per pin."""

    def __init__(self, levels: dict[int, int]) -> None:
        self._levels = dict(levels)

    def input(self, pin: int) -> int:
        return self._levels[pin]


def _sensor(levels: dict[int, int], **kwargs) -> IRTracingSensor:
    return IRTracingSensor(PINS, FakeGpio(levels), **kwargs)


def test_white_surface_reads_all_zero_with_default_polarity():
    # Raw LOW on every channel -> white under all of them.
    sensor = _sensor({17: 0, 27: 0, 22: 0, 23: 0})
    assert sensor.read() == (0, 0, 0, 0)


def test_black_line_reads_all_one_with_default_polarity():
    # Raw HIGH on every channel -> black line under all of them.
    sensor = _sensor({17: 1, 27: 1, 22: 1, 23: 1})
    assert sensor.read() == (1, 1, 1, 1)


def test_mixed_surface_maps_each_channel_independently():
    sensor = _sensor({17: 1, 27: 0, 22: 1, 23: 0})
    assert sensor.read() == (1, 0, 1, 0)


def test_invert_flips_only_the_named_channels():
    # Board reports LOW over black on channels 1 and 4 (indices 0 and 3).
    sensor = _sensor({17: 0, 27: 0, 22: 1, 23: 0}, invert={0, 3})
    assert sensor.read() == (1, 0, 1, 1)


def test_raw_reports_untransformed_levels():
    sensor = _sensor({17: 1, 27: 0, 22: 1, 23: 0})
    assert sensor.raw() == (1, 0, 1, 0)


def test_read_order_matches_channel_order():
    # Channels need not be ascending GPIO numbers; Out order is what matters.
    sensor = IRTracingSensor((22, 17, 23, 27), FakeGpio({22: 1, 17: 0, 23: 1, 27: 0}))
    assert sensor.read() == (1, 0, 1, 0)


def test_invert_out_of_range_is_rejected():
    with pytest.raises(ValueError, match="out of range"):
        _sensor({17: 1, 27: 1, 22: 1, 23: 1}, invert={4})


def test_zero_channels_is_rejected():
    with pytest.raises(ValueError, match="at least one"):
        IRTracingSensor((), FakeGpio({}))
