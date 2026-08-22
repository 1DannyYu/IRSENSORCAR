"""Verify that emitted byte sequences match the vendor SDK behavior.

The tests intercept calls with a fake SMBus object, so no hardware is required.
"""

from __future__ import annotations

import pytest

from carbot.nezha import NeZha, NeZhaError


class FakeBus:
    """Fake SMBus implementation that records all I2C calls."""

    def __init__(self, encoder_words: dict[int, list[int]] | None = None) -> None:
        self.calls: list[tuple] = []
        self.closed = False
        self._encoder_words = encoder_words or {}

    def write_byte_data(self, addr, reg, value):
        self.calls.append(("write_byte_data", addr, reg, value))

    def write_i2c_block_data(self, addr, reg, data):
        self.calls.append(("write_i2c_block_data", addr, reg, list(data)))

    def read_i2c_block_data(self, addr, reg, length):
        self.calls.append(("read_i2c_block_data", addr, reg, length))
        return self._encoder_words.get(reg, [0, 0])

    def close(self):
        self.closed = True


@pytest.fixture
def bus() -> FakeBus:
    return FakeBus()


@pytest.fixture
def board(bus: FakeBus) -> NeZha:
    b = NeZha(bus, init_motors=False)
    bus.calls.clear()  # Ignore the reset sent during construction.
    return b


def test_construction_resets_and_inits_motors(bus: FakeBus):
    NeZha(bus)
    assert bus.calls == [
        ("write_byte_data", 0x40, 0x00, 0xFF),  # CMD_RESET
        ("write_byte_data", 0x40, 0x00, 0x01),  # CMD_MOTOR_INIT
    ]


def test_motor_forward_matches_vendor_frame(board: NeZha, bus: FakeBus):
    """Vendor `NeZha_Motor1_SetPwm` writes the command register, then 4 data bytes."""
    board.motor(1, 1000)
    assert bus.calls == [
        ("write_byte_data", 0x40, 0x00, 0x05),
        ("write_i2c_block_data", 0x40, 0x05, [0x03, 0xE8, 0x00, 0x00]),
    ]


def test_motor_reverse_swaps_the_pair(board: NeZha, bus: FakeBus):
    board.motor(1, -1000)
    assert bus.calls[-1] == (
        "write_i2c_block_data",
        0x40,
        0x05,
        [0x00, 0x00, 0x03, 0xE8],
    )


def test_motor_zero_sends_both_zero(board: NeZha, bus: FakeBus):
    board.motor(2, 0)
    assert bus.calls[-1] == ("write_i2c_block_data", 0x40, 0x09, [0, 0, 0, 0])


@pytest.mark.parametrize("n,cmd", [(1, 0x05), (2, 0x09), (3, 0x0D), (4, 0x11)])
def test_motor_command_codes(board: NeZha, bus: FakeBus, n: int, cmd: int):
    board.motor(n, 500)
    assert bus.calls[0] == ("write_byte_data", 0x40, 0x00, cmd)


def test_motor_never_sets_both_channels(board: NeZha, bus: FakeBus):
    """The vendor manual marks simultaneous motor_a and motor_b values as invalid."""
    for speed in (-1000, -1, 0, 1, 1000):
        bus.calls.clear()
        board.motor(1, speed)
        _, _, _, data = bus.calls[-1]
        a = data[0] << 8 | data[1]
        b = data[2] << 8 | data[3]
        assert a == 0 or b == 0, f"speed={speed} emitted a={a} b={b}"


@pytest.mark.parametrize("speed", [-1001, 1001, 5000])
def test_motor_rejects_out_of_range(board: NeZha, speed: int):
    with pytest.raises(ValueError):
        board.motor(1, speed)


@pytest.mark.parametrize("n", [0, 5, -1])
def test_channel_must_be_1_to_4(board: NeZha, n: int):
    with pytest.raises(ValueError):
        board.motor(n, 0)


def test_stop_zeroes_all_four(board: NeZha, bus: FakeBus):
    board.stop()
    cmds = [c[3] for c in bus.calls if c[0] == "write_byte_data"]
    assert cmds == [0x05, 0x09, 0x0D, 0x11]


# -------------------------------------------------------------- Encoders
def test_encoder_reads_signed_big_endian():
    bus = FakeBus(encoder_words={0x16: [0xFF, 0x9C]})  # -100
    board = NeZha(bus, init_motors=False)
    assert board.encoder(1) == -100


def test_encoder_positive():
    bus = FakeBus(encoder_words={0x1F: [0x01, 0x2C]})  # 300
    board = NeZha(bus, init_motors=False)
    assert board.encoder(4) == 300


@pytest.mark.parametrize("n,cmd", [(1, 0x15), (2, 0x18), (3, 0x1B), (4, 0x1E)])
def test_encoder_init_codes(board: NeZha, bus: FakeBus, n: int, cmd: int):
    board.init_encoder(n)
    assert bus.calls == [("write_byte_data", 0x40, 0x00, cmd)]


# ---------------------------------------------------------------- Servos
def test_servo_angle_maps_to_pwm_range(board: NeZha, bus: FakeBus):
    for angle, pwm in ((0, 50), (90, 150), (180, 250)):
        bus.calls.clear()
        board.servo(1, angle)
        assert bus.calls[-1] == ("write_i2c_block_data", 0x40, 0x22, [0, pwm])


@pytest.mark.parametrize("pwm", [49, 251])
def test_servo_pwm_bounds(board: NeZha, pwm: int):
    with pytest.raises(ValueError):
        board.servo_pwm(1, pwm)


@pytest.mark.parametrize("angle", [-1, 181])
def test_servo_angle_bounds(board: NeZha, angle: float):
    with pytest.raises(ValueError):
        board.servo(1, angle)


@pytest.mark.parametrize("n,cmd", [(1, 0x22), (2, 0x25), (3, 0x28), (4, 0x2B)])
def test_servo_command_codes(board: NeZha, bus: FakeBus, n: int, cmd: int):
    board.servo(n, 90)
    assert bus.calls[0] == ("write_byte_data", 0x40, 0x00, cmd)


# -------------------------------------------------------------------- LEDs
@pytest.mark.parametrize(
    "name,on,off,toggle",
    [
        ("head", 0x2D, 0x2E, 0x2F),
        ("tail_left", 0x30, 0x31, 0x32),
        ("tail_right", 0x33, 0x34, 0x35),
        ("ambient", 0x36, 0x37, 0x38),
    ],
)
def test_led_codes(board: NeZha, bus: FakeBus, name, on, off, toggle):
    for state, expected in ((True, on), (False, off), (None, toggle)):
        bus.calls.clear()
        board.led(name, state)
        assert bus.calls == [("write_byte_data", 0x40, 0x00, expected)]


def test_led_rejects_unknown_name(board: NeZha):
    with pytest.raises(ValueError):
        board.led("underglow", True)


# ---------------------------------------------------------- Errors and cleanup
def test_oserror_becomes_nezha_error(board: NeZha, bus: FakeBus):
    def boom(*_):
        raise OSError(121, "Remote I/O error")

    bus.write_byte_data = boom
    with pytest.raises(NeZhaError, match="200kHz"):
        board.motor(1, 100)


def test_context_manager_stops_motors_and_keeps_borrowed_bus_open(bus: FakeBus):
    with NeZha(bus, init_motors=False):
        bus.calls.clear()
    cmds = [c[3] for c in bus.calls if c[0] == "write_byte_data"]
    assert cmds == [0x05, 0x09, 0x0D, 0x11]  # stop()
    assert not bus.closed  # Borrowed buses should not be closed here.


def test_close_can_skip_motor_stop(bus: FakeBus):
    board = NeZha(bus, init_motors=False)
    bus.calls.clear()

    board.close(stop_motors=False)

    assert bus.calls == []


# ---------------------------------------------------------------------- Car
class TestCar:
    """Verify that the differential-drive layer applies config mappings and inversion correctly."""

    @staticmethod
    def _motor_calls(bus: FakeBus) -> dict[int, int]:
        """Reconstruct signed motor speeds from `write_i2c_block_data` calls."""
        from carbot.nezha import CMD_MOTOR_SET

        out: dict[int, int] = {}
        for call in bus.calls:
            if call[0] != "write_i2c_block_data":
                continue
            _, _, cmd, data = call
            a = data[0] << 8 | data[1]
            b = data[2] << 8 | data[3]
            out[CMD_MOTOR_SET.index(cmd) + 1] = a if a else -b
        return out

    @classmethod
    def _logical_motor_calls(cls, bus: FakeBus) -> dict[int, int]:
        """Return logical motor speeds after undoing hardware-specific inversion."""
        from carbot import config

        return {
            n: -speed if n in config.INVERTED_MOTORS else speed
            for n, speed in cls._motor_calls(bus).items()
        }

    def test_forward_drives_all_four_the_same_way(self, bus: FakeBus):
        from carbot.car import Car

        car = Car(NeZha(bus, init_motors=False))
        bus.calls.clear()
        car.forward(300)
        assert self._logical_motor_calls(bus) == {1: 300, 2: 300, 3: 300, 4: 300}

    def test_spin_left_opposes_the_two_sides(self, bus: FakeBus):
        from carbot import config
        from carbot.car import Car

        car = Car(NeZha(bus, init_motors=False))
        bus.calls.clear()
        car.spin_left(400)
        speeds = self._logical_motor_calls(bus)
        left = {speeds[config.WHEEL_TO_MOTOR[w]] for w in ("front_left", "rear_left")}
        right = {speeds[config.WHEEL_TO_MOTOR[w]] for w in ("front_right", "rear_right")}
        assert left == {-400} and right == {400}

    def test_inverted_motors_are_flipped(self, bus: FakeBus, monkeypatch):
        from carbot import config
        from carbot.car import Car

        target = config.WHEEL_TO_MOTOR["front_left"]
        monkeypatch.setattr(config, "INVERTED_MOTORS", frozenset({target}))
        car = Car(NeZha(bus, init_motors=False))
        bus.calls.clear()
        car.forward(300)
        speeds = self._motor_calls(bus)
        assert speeds[target] == -300
        assert speeds[config.WHEEL_TO_MOTOR["rear_left"]] == 300

    def test_drive_clamps_to_max_speed(self, bus: FakeBus):
        from carbot.car import Car

        car = Car(NeZha(bus, init_motors=False))
        bus.calls.clear()
        car.drive(5000, -5000)
        assert set(self._logical_motor_calls(bus).values()) == {1000, -1000}

    def test_move_for_stops_even_if_interrupted(self, bus: FakeBus, monkeypatch):
        from carbot.car import Car

        car = Car(NeZha(bus, init_motors=False))
        bus.calls.clear()

        def boom(_seconds):
            raise KeyboardInterrupt

        monkeypatch.setattr("carbot.car.time.sleep", boom)
        with pytest.raises(KeyboardInterrupt):
            car.move_for(1, 300, 300)
        assert set(self._motor_calls(bus).values()) == {0}

    def test_borrowed_board_is_not_closed(self, bus: FakeBus):
        from carbot.car import Car

        board = NeZha(bus, init_motors=False)
        with Car(board):
            pass
        assert not bus.closed


class FlakyBus(FakeBus):
    """Fails the first `fail_times` writes of each kind with the errno the board produced."""

    def __init__(self, fail_times: int) -> None:
        super().__init__()
        self.remaining = fail_times

    def write_byte_data(self, addr, reg, value):
        if self.remaining > 0:
            self.remaining -= 1
            raise OSError(121, "Remote I/O error")
        super().write_byte_data(addr, reg, value)


def test_transient_write_error_is_retried():
    bus = FlakyBus(fail_times=2)
    board = NeZha(bus, init_motors=False)
    assert bus.calls == [("write_byte_data", 0x40, 0x00, 0xFF)]
    assert board.write_retries == 2


def test_write_error_raises_once_retries_are_exhausted():
    bus = FlakyBus(fail_times=99)
    with pytest.raises(NeZhaError, match="after 3 attempts"):
        NeZha(bus, init_motors=False)


def test_close_can_skip_stopping_motors(board: NeZha, bus: FakeBus):
    """`Car.close` uses this after its own best-effort stop, so it must send nothing."""
    board.close(stop_motors=False)
    assert bus.calls == []
