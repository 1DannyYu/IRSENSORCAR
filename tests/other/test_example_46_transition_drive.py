from pathlib import Path
from runpy import run_path

SCRIPT = Path(__file__).parents[2] / "examples" / "46_map1_ir_transition_drive.py"
NAMESPACE = run_path(str(SCRIPT))
EXAMPLE39_ARGUMENTS = NAMESPACE["example39_arguments"]


def test_default_run_consolidates_the_morning_transition_settings() -> None:
    args = EXAMPLE39_ARGUMENTS()

    assert args[0].endswith("examples/39_map1_ir_line_follow.py")
    assert args[args.index("--test-phase") + 1] == "3"
    assert args[args.index("--duration") + 1] == "20.0"
    assert args[args.index("--speed") + 1] == "150"
    assert args[args.index("--start-acquire-timeout-s") + 1] == "5.0"
    assert args[args.index("--phase3-lead-in-timeout-s") + 1] == "5.0"
    assert args[args.index("--phase3-exit-confirm-s") + 1] == "0.5"
    assert args[args.index("--phase4-proof-s") + 1] == "2.0"
    assert "--log-every" in args


def test_dry_run_and_extra_diagnostics_are_forwarded_last() -> None:
    args = EXAMPLE39_ARGUMENTS(
        duration_s=12.0,
        speed=175,
        dry_run=True,
        extra=("--invert", "0,1,2,3"),
    )

    assert args[args.index("--duration") + 1] == "12.0"
    assert args[args.index("--speed") + 1] == "175"
    assert args[-3:] == ["--dry-run", "--invert", "0,1,2,3"]
