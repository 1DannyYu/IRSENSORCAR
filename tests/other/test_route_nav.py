"""Tests for the route tracker (``carbot.route_nav``).

The tracker wraps a vision-driven nav (``LineNav`` in production). These
tests inject a stub nav so phase bookkeeping is tested in isolation: turn
events advance the phase index, roundabout enter/exit advance it, and the
tracker never invents drive commands.
"""

from carbot.line_follow import LineReading
from carbot.line_nav import NavCommand, NavState
from carbot.route_nav import RouteTracker
from carbot.route_plan import StepKind, task1_route


class StubNav:
    """Minimal nav stand-in with a controllable state."""

    def __init__(self, state: NavState = NavState.FOLLOW) -> None:
        self.state = state
        self.command = NavCommand(
            action="follow",
            left=200,
            right=200,
            reason="stub",
            state=state,
        )

    def step(self, reading, dt):  # stub signature (arguments unused)
        return self.command


def make_tracker(state: NavState = NavState.FOLLOW) -> RouteTracker:
    return RouteTracker(task1_route(), StubNav(state))


def reading(err=0.0) -> LineReading:
    return LineReading(
        visible=True,
        error_px=0.0,
        error_fraction=err,
        centroid_x=0.5,
        line_width_px=10.0,
        dark_fraction=0.2,
        tracked_rows=10,
        roi=(0, 100, 200, 300),
    )


def test_starts_at_stem():
    tr = make_tracker()
    assert tr.step_index == 0
    assert tr.current_step.kind == StepKind.STRAIGHT
    assert tr.current_step.label == "Phase 1 stem"


def test_follow_keeps_phase():
    tr = make_tracker()
    status = tr.step(reading(), 0.05)
    assert status.step_index == 0
    assert status.nav_state == NavState.FOLLOW.value


def test_right_turn_event_advances_phase():
    tr = make_tracker()
    tr.step(reading(), 0.05)
    tr.nav.state = NavState.RIGHT_TURN
    tr.step(reading(), 0.05)
    # straight step consumed + turn step consumed -> Phase 2 east
    assert tr.step_index == 2
    assert "Phase 2" in tr.current_step.label


def test_roundabout_entry_advances_phase():
    tr = make_tracker()
    # a couple of turn events, then enter the roundabout: the tracker skips
    # forward to the roundabout step
    for _ in range(2):
        tr.nav.state = NavState.RIGHT_TURN
        tr.step(reading(), 0.05)
        tr.nav.state = NavState.FOLLOW
        tr.step(reading(), 0.05)
    tr.nav.state = NavState.ROUNDABOUT
    tr.step(reading(), 0.05)
    assert tr.current_step.kind == StepKind.ROUNDABOUT


def test_roundabout_exit_advances_phase():
    tr = make_tracker()
    for _ in range(2):
        tr.nav.state = NavState.RIGHT_TURN
        tr.step(reading(), 0.05)
        tr.nav.state = NavState.FOLLOW
        tr.step(reading(), 0.05)
    tr.nav.state = NavState.ROUNDABOUT
    tr.step(reading(), 0.05)
    assert tr.current_step.kind == StepKind.ROUNDABOUT
    # exit to follow -> past the roundabout step and the exit turn
    tr.nav.state = NavState.FOLLOW
    tr.step(reading(), 0.05)
    assert "Phase 10" in tr.current_step.label


def test_remaining_distance_reported_for_straight():
    tr = make_tracker()
    status = tr.step(reading(), 0.25)
    assert status.step_kind == StepKind.STRAIGHT
    assert status.expected_remaining_m <= 0.10
    assert status.expected_remaining_m > 0.0


def test_tracker_never_drives_itself():
    tr = make_tracker()
    tr.step(reading(), 0.05)
    cmd = tr.last_command
    assert cmd is not None
    assert hasattr(cmd, "left") and hasattr(cmd, "right")
    assert cmd.left == 200 and cmd.right == 200  # exactly the stub's command


def test_route_plan_steps_have_labels():
    for s in task1_route():
        assert s.label
