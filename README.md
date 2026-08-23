# Car and Robotic Arm

![Smart car and robotic arm build](assets/assembly/021_RobotCar_With_RoboticArm_Combined.jpg)

A Raspberry Pi 5 smart car and robotic arm build. This README is a map of the repository, not a
description of it — everything it explains lives in the files it links to.

[GitHub Repository](https://github.com/1DannyYu/IRSENSORCAR) · [Contact Danny on GitHub](https://github.com/1DannyYu)

## How to Explore This Repo

There is no single "read the whole thing" path — start from whichever question you have:

| I want to... | Go to |
|---|---|
| See what the robot looks like and what parts it's built from | [assets/](assets/) |
| Get the full technical/operator reference (status, every example script, IR workflow, SSH, safety) | [docs/project-reference.md](docs/project-reference.md) |
| Understand a hardware decision or a verified fact (wiring, protocol, timing) | [docs/hardware/](docs/hardware/) and [docs/adr/](docs/adr/) |
| Read the Python that drives the robot | [src/carbot/](src/carbot/) |
| Run a hardware check myself | [examples/](examples/) — see [docs/project-reference.md#quick-start](docs/project-reference.md#quick-start) |
| Check that the code is tested | [tests/](tests/) |
| Follow the day-to-day engineering trail (what changed, why, what broke) | [docs/progress/](docs/progress/), [docs/handoff-*.md](docs/) |
| See the reasoning behind a major choice | [docs/adr/](docs/adr/) |
| Connect from a Mac / work over SSH | [docs/setup/mac-to-raspberry-pi-access.md](docs/setup/mac-to-raspberry-pi-access.md) |
| Read the safety rules before running anything | [docs/project-reference.md#safety-notes](docs/project-reference.md#safety-notes), [CLAUDE.md](CLAUDE.md) |
| Understand file naming and where new files should go | [CONVENTIONS.md](CONVENTIONS.md) |
| See the working notes for one specific task (e.g. IR line tracking, 3D mapping) | [tasks/](tasks/) |
| Assess this as a school Software Engineering project | see [For Examiners and Teachers](#for-examiners-and-teachers) below |

## Repository Layout

```
Car-and-Robotic-Arm/
├── README.md              You are here
├── docs/                  Written record: hardware facts, setup guides, decisions, progress logs
│   ├── hardware/          Verified specs, protocol notes, wiring
│   ├── setup/             Bring-up and environment setup guides
│   ├── adr/               Architecture decision records (why, not just what)
│   ├── progress/          Dated logs of completed, verified work
│   ├── reflections/       Project reflection and engineering-role write-ups
│   └── project-reference.md   Full technical/operator reference
├── src/carbot/            The importable Python driver and control package
├── examples/              Runnable, numbered scripts that exercise real hardware
├── tests/                 Automated tests (pytest), flat, one file per subsystem
├── tasks/                 Per-task working notes and run books
├── assets/                Build photos and reference diagrams
└── vendor/                Third-party material, read-only when used; empty as of 2026-08-22
```

See [CONVENTIONS.md](CONVENTIONS.md) for the full layout and naming rules.

## For Examiners and Teachers

This repository is also the submission for an 11 Software Engineering assessment.

- [docs/reflections/](docs/reflections/) — engineering-role and project-roadmap reflections
- [docs/progress/](docs/progress/) — dated, verified evidence of work as it happened, in order
- [docs/adr/](docs/adr/) — the significant design decisions and the reasoning behind them
- [tasks/](tasks/) — working notes and run books behind each major task
- Git history — commit messages follow [Conventional Commits](CONVENTIONS.md#commit-messages), scoped by top-level folder
