"""Tests for ``get_throttled`` decoding.

The regression these lock down: `examples/08` and `examples/14` both had the
live and since-boot halves of the word inverted, so the observed ``0x50000``
(undervoltage and throttling *have occurred*) was reported as "throttling NOW"
and failed the preflight on a healthy 4.9 V supply.
"""

from __future__ import annotations

import pytest

from carbot.power import decode_throttled, parse_throttled_output


def test_zero_word_is_completely_clean():
    status = decode_throttled(0x0)
    assert status.live == ()
    assert status.since_boot == ()
    assert status.throttled_now is False


def test_low_nibble_bits_are_live():
    status = decode_throttled(0x1)
    assert status.live == ("undervoltage",)
    assert status.since_boot == ()
    assert status.throttled_now is True


def test_bit_16_and_18_are_since_boot_history_not_live():
    """The exact word measured on the Pi: sticky history, nothing throttling now."""
    status = decode_throttled(0x50000)
    assert status.live == ()
    assert status.since_boot == ("undervoltage", "throttled")
    assert status.throttled_now is False


def test_all_live_flags_decode_in_bit_order():
    status = decode_throttled(0xF)
    assert status.live == (
        "undervoltage",
        "ARM frequency capped",
        "throttled",
        "soft temperature limit",
    )
    assert status.since_boot == ()


def test_all_since_boot_flags_decode_in_bit_order():
    status = decode_throttled(0xF0000)
    assert status.live == ()
    assert status.since_boot == (
        "undervoltage",
        "ARM frequency capped",
        "throttled",
        "soft temperature limit",
    )
    assert status.throttled_now is False


def test_both_halves_set_reports_live_and_history():
    status = decode_throttled(0x50005)
    assert status.live == ("undervoltage", "throttled")
    assert status.since_boot == ("undervoltage", "throttled")
    assert status.throttled_now is True


def test_negative_word_is_rejected():
    with pytest.raises(ValueError):
        decode_throttled(-1)


def test_describe_names_which_half_each_flag_came_from():
    text = decode_throttled(0x50000).describe()
    assert "0x50000" in text
    assert "no live throttling" in text
    assert "since boot: undervoltage, throttled" in text
    assert "NOW" not in text


def test_describe_marks_live_throttling():
    text = decode_throttled(0x1).describe()
    assert "NOW: undervoltage" in text


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("throttled=0x50000", 0x50000),
        ("throttled=0x0", 0),
        ("  throttled=0xF  ", 0xF),
        ("throttled=0X50000", 0x50000),
    ],
)
def test_parse_throttled_output(text, expected):
    assert parse_throttled_output(text) == expected


@pytest.mark.parametrize("text", ["", "throttled=", "throttled", "throttled=zz"])
def test_parse_throttled_output_rejects_garbage(text):
    with pytest.raises(ValueError):
        parse_throttled_output(text)
