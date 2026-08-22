"""Decoding for ``vcgencmd get_throttled`` power-health bits.

``examples/other/08_battery_check.py`` and ``examples/other/14_all_sensors_preflight_check.py`` each
decoded these bits inline, and both had the two halves inverted: the **low
nibble** carries the live state and **bits 16-19** carry the sticky
since-boot history. A ``0x50000`` reading — undervoltage and throttling *have
occurred* at some point since boot — was therefore reported as "throttling
NOW", failing the preflight and blocking motor tests on a supply that was
measuring a healthy 4.9 V. This module is the single decoder so the two
scripts cannot disagree again, and so the bit meanings are unit-tested
without a Raspberry Pi.

Bit layout, from the Raspberry Pi documentation:

| Mask | Meaning |
|---|---|
| ``0x1`` | undervoltage detected (live) |
| ``0x2`` | ARM frequency capped (live) |
| ``0x4`` | currently throttled (live) |
| ``0x8`` | soft temperature limit active (live) |
| ``0x10000`` | undervoltage has occurred since boot |
| ``0x20000`` | ARM frequency capping has occurred since boot |
| ``0x40000`` | throttling has occurred since boot |
| ``0x80000`` | soft temperature limit has occurred since boot |
"""

from __future__ import annotations

from dataclasses import dataclass

LIVE_MASK = 0xF
SINCE_BOOT_MASK = 0xF0000

LIVE_FLAGS: tuple[tuple[int, str], ...] = (
    (0x1, "undervoltage"),
    (0x2, "ARM frequency capped"),
    (0x4, "throttled"),
    (0x8, "soft temperature limit"),
)

# The since-boot half repeats the live half, shifted 16 bits up.
SINCE_BOOT_SHIFT = 16


@dataclass(frozen=True)
class ThrottleStatus:
    """Decoded ``get_throttled`` word, split into live and since-boot halves."""

    raw: int
    live: tuple[str, ...]
    since_boot: tuple[str, ...]

    @property
    def throttled_now(self) -> bool:
        """True when the SoC is undervolted, capped, or throttled *right now*.

        This is the only half that should gate a motion test. The since-boot
        half stays set until reboot, so treating it as a failure permanently
        blocks motor work after a single power dip.
        """
        return bool(self.live)

    def describe(self) -> str:
        """One-line human summary naming which half each flag came from."""
        parts = [f"0x{self.raw:05X}"]
        if self.live:
            parts.append("NOW: " + ", ".join(self.live))
        else:
            parts.append("no live throttling")
        if self.since_boot:
            parts.append("since boot: " + ", ".join(self.since_boot))
        return " | ".join(parts)


def decode_throttled(raw: int) -> ThrottleStatus:
    """Split a ``get_throttled`` word into its live and since-boot flag names."""
    if raw < 0:
        raise ValueError("get_throttled word must be non-negative")
    live = tuple(name for mask, name in LIVE_FLAGS if raw & mask)
    since_boot = tuple(name for mask, name in LIVE_FLAGS if raw & (mask << SINCE_BOOT_SHIFT))
    return ThrottleStatus(raw=raw, live=live, since_boot=since_boot)


def parse_throttled_output(text: str) -> int:
    """Parse ``vcgencmd get_throttled`` output (``throttled=0x50000``) to an int."""
    _, _, value = text.strip().partition("=")
    value = value.strip()
    if not value:
        raise ValueError(f"unparseable get_throttled output: {text!r}")
    try:
        return int(value, 16)
    except ValueError as exc:
        raise ValueError(f"unparseable get_throttled output: {text!r}") from exc
