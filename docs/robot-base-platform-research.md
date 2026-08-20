# Modular Robot Base Platform — Research Background Report

> **Author / Client:** Danny Yu
> **Date:** 2026-08-14
> **Status:** Research report at the ideation stage
> **Related project:** [Car-and-Robotic-Arm](../README.md) (Raspberry Pi 5 interior-mapping car;
> the original robotic arm has been removed)
> **Related glossary:** [docs/project-terminology.md](project-terminology.md)

---

## Executive Summary

Robotic AI has developed rapidly in recent years. The combination of large language models (LLMs) and embodied AI is pushing robots from "pre-programmed actuators" toward "intelligent agents that can perceive, reason, and act autonomously." Against this backdrop, this report explores the following idea:

> **Build a small modular robot base platform that establishes a robot base development ecosystem for personal use and for potential clients.**

This report concludes that the idea is well-timed and rests on solid foundations, with three key arguments:

1. **The base platform is the starting point of an ecosystem** — platformization accumulates more value than a single product, but its success hinges on "standard interfaces" and "developer experience."
2. **The existing project (Car-and-Robotic-Arm) is a natural starting point** — the Raspberry Pi 5 + NeZha I2C driver board + modular Python software already forms a viable prototype of a small, modular, replicable platform.
3. **The biggest risk is not technical, but scope** — we recommend an MVP of "one reference chassis + one standard interface + a set of extensible skills," avoiding premature generalization.

---

## 1. Background: Trends in Robotic AI

### 1.1 From "Programmed Robots" to "AI Robots"

| Era | Characteristics | Examples |
|---|---|---|
| Traditional robots | Pre-programmed, fixed workflows, repetitive motion | Industrial robotic arms |
| Sensor-driven robots | Sensor feedback, closed-loop control, obstacle avoidance | AGVs, robot vacuums |
| AI robots (now) | Visual understanding, natural language commands, autonomous decisions | Embodied AI robots |

Key trend: **LLMs let robots "understand commands"** (e.g., "bring me the cup"), computer vision lets them "understand the world," and **closed-loop control lets them "execute actions correctly"** — these three pillars map directly onto this project's technical backbone (perception / planning / actuation).

### 1.2 The Rise of Open Hardware and Ecosystem Platforms

- **Open hardware**: Arduino, Raspberry Pi, and ESP32 have dramatically lowered the barrier to entry for robotics.
- **Open software frameworks**: ROS / ROS 2 have become the standard for research and industry, but their learning curve remains steep for small and educational projects.
- **Education and maker markets**: Demand for small modular robot platforms (LEGO SPIKE, Micro:bit cars, various Raspberry Pi chassis) is steady, yet most platforms are closed or lack an extensible software ecosystem.

> **Conclusion:** There are plenty of "cheap cars" on the market, but the gap for a "cheap, open, modular small base platform with a good developer experience" remains — and that is exactly the opportunity behind this idea.

---

## 2. Interpreting Your Initial Idea

### 2.1 Deconstructing the Idea

The idea can be decomposed into four layers:

| Layer | Content | Key question |
|---|---|---|
| **Product (base)** | Small robot base (chassis, motors, driver board, sensors) | What can this base do? |
| **Platform (software)** | Modular SDK / drivers / application layer | How quickly can developers get started? |
| **Ecosystem (extensions)** | Sensor kits, skill packages, examples, community | Why would others participate? |
| **Business (value)** | Personal use → potential clients | What would clients pay for? |

### 2.2 Three Core Concepts

1. **"Small"** — focus on desktop-level / educational / lightweight applications, avoiding head-on competition with large robot platforms; low cost, replicable, easy to demonstrate.
2. **"Modular"** — both hardware (chassis, sensors, actuators) and software (drivers, perception, planning, interfaces) are swappable; this is the key to being a "platform" rather than a "single product."
3. **"Ecosystem"** — not just building one robot, but establishing an infrastructure **on which others can develop**; ecosystem value grows with the number of participants (network effects).

### 2.3 One-Sentence Positioning (Proposed)

> **"A Raspberry-Pi-based, modular, extensible small robot base platform that lets individual developers and educators build their own AI robots with minimal effort."**

---

## 3. Connection to the Existing Project

### 3.1 Existing Foundations (Verified)

| Aspect | Current state (Car-and-Robotic-Arm) |
|---|---|
| Hardware | Raspberry Pi 5, Yourfun NeZha driver board (I2C `0x40`), Dasheng multi-form chassis (4× N20 motors), HC-SR04, and IMX500 AI Camera; the 3-DOF arm was removed after a servo failed |
| Software | `src/carbot/`: `nezha.py` (I2C driver), `car.py` (vehicle abstraction), `mapping.py` (2D mapping), `vision.py` (camera calibration and room anchors), and `config.py` |
| Verification | I2C, motor directions, low-speed driving, obstacle avoidance, AI Camera capture/inference, camera calibration, and a fixed-wall room anchor verified; see `docs/progress/` for evidence |
| Website | Bilingual Astro project site, published on GitHub Pages |

### 3.2 Gap Analysis: From "One Car" to "A Platform"

| The existing project has | Platformization still needs |
|---|---|
| Drivers for a single chassis | **Hardware Abstraction Layer (HAL)**: one API supporting different chassis/driver boards |
| Verified hardware records | **Standard interface documentation**: clear extension specs for sensors, actuators, and skills |
| Example scripts (examples/) | **Package system**: installable sensor and skill modules |
| Single-developer workflow | **Developer experience (DX)**: fast onboarding, simulation, error diagnosis |
| Static website showcase | **Dynamic ecosystem portal**: package registry, example sharing, community feedback |

> **Key insight:** The existing project has already verified that "the hardware can run." The next step for platformization is verifying that "others can quickly replicate and extend it."

---

## 4. Conceptual Blueprint: Modular Robot Base Platform

### 4.1 Architecture Overview (Layered)

```
┌─────────────────────────────────────────────┐
│  Applications (demos, courses, client projects)│
├─────────────────────────────────────────────┤
│  Ecosystem (skill packages, sensor kits,      │
│             community examples)               │
├─────────────────────────────────────────────┤
│  Python SDK (Car / Sensor / Skill APIs)       │
├─────────────────────────────────────────────┤
│  Hardware Abstraction Layer (standard HAL)    │
├─────────────────────────────────────────────┤
│  Drivers (NeZha I2C, camera, sensors)         │
├─────────────────────────────────────────────┤
│  Hardware (chassis, motors, servos, sensors)  │
└─────────────────────────────────────────────┘
```

### 4.2 Modular Design Principles

1. **Standard interface first**: define what "a chassis must implement" (e.g., `forward() / turn() / stop()`) before implementation details.
2. **Pluggable sensors**: sensors connect through "registration + data streams" rather than being hardcoded into the main program.
3. **Skills as packages**: capabilities like obstacle avoidance, line following, and target tracking are packaged as installable modules — an evolution of the existing `scripts/` and `examples/`.
4. **Config separated from code**: following the spirit of `config.py`, hardware differences (motor direction, I2C address) are entirely determined by configuration.

### 4.3 Mapping to the Existing Project

| Platform layer | Existing asset | Suggested evolution |
|---|---|---|
| Hardware layer | Dasheng chassis + NeZha | Define the "Reference Chassis" |
| Driver layer | `nezha.py` | Abstract into a standard I2C driver interface |
| Abstraction layer | `car.py` (single Car class) | Split into a `BaseChassis` interface with multiple implementations |
| SDK layer | Scattered across examples | Consolidate into a formal Python SDK |
| Ecosystem layer | None | Define package specs and an example library |
| Application layer | Interior-mapping robot prototype | Become the flagship showcase of the platform |

---

## 5. Value & Market Analysis

### 5.1 Target Users (Initial Personas)

| Type | Needs | Willingness to pay |
|---|---|---|
| Individual developers / makers | An affordable AI robot experimentation platform | Medium (hardware-led) |
| Educators / schools | Teachable robot kits with materials | Medium-high (with curriculum) |
| Potential clients (small companies / studios) | Rapid prototyping, customized bases | High (project/service) |
| Research groups | An extensible experimental platform | Medium-high (customization) |

### 5.2 Business Model Options

| Model | Description | Suitable phase |
|---|---|---|
| **Open-source software + hardware kits** | Open SDK to build the ecosystem; charge for hardware kits | MVP phase (recommended) |
| **Reference design + custom services** | Offer base design and development services to clients | Once clients appear |
| **Skill / package marketplace** | Paid downloads for advanced skill packages | Once the ecosystem matures |
| **Subscription (cloud / courses)** | Teaching courses, remote monitoring services | Long term |

### 5.3 Competition and Differentiation

| Competitor type | Examples | Your differentiation |
|---|---|---|
| Education kits | LEGO SPIKE, Micro:bit | Openness, AI (LLM/vision) integration |
| Maker chassis | Various Raspberry Pi car kits | Modular software ecosystem, documentation quality |
| Research frameworks | ROS 2 | Low learning curve, small footprint |

---

## 6. SWOT Analysis

| | Helpful | Harmful |
|---|---|---|
| **Internal** | **S (Strengths)**: verified hardware and code; modular Python foundation; complete documentation and website | **W (Weaknesses)**: single developer; limited time; constrained hardware resources (one chassis, records of damaged arm parts) |
| **External** | **O (Opportunities)**: the robotic AI boom; growth of the education market; rise of open-source ecosystems | **T (Threats)**: large companies entering (e.g., LEGO, Amazon DeepRacer-class); fast-moving technology; fragmented market |

**Key risks:**

| Risk | Description | Mitigation |
|---|---|---|
| Scope creep | Aiming for a "universal platform" and never finishing | Lock the MVP: one reference chassis + one standard API + 3 example skills |
| Hardware supply | Dependence on specific chassis/driver vendors | The abstraction layer ensures replaceability; document sourcing |
| Motivation | Personal projects can stall | Drive forward with small, demonstrable milestones |

---

## 7. Recommended Roadmap

### Phase 0: Idea Validation (now → 1 month)
- Confirm positioning from this report (target customers, scope)
- Inventory existing assets: what can be reused directly, what needs refactoring

### Phase 1: MVP Platform (1 → 3 months)
- Define and implement the **Hardware Abstraction Layer (HAL)**: a `BaseChassis` standard interface
- Refactor `car.py` / `nezha.py` into a formal SDK (with documentation)
- Package **3 demo skills**: line following, obstacle avoidance, target tracking
- Build a "quick start" guide (get a new user running in 10 minutes)

### Phase 2: Ecosystem Seed (3 → 6 months)
- Package spec (how sensors/skills are packaged and installed)
- Public example library + bilingual teaching documentation
- Recruit 2–3 external testers (educators or makers) for feedback

### Phase 3: Clients and Business (6+ months)
- Decide the business model based on feedback (hardware kits / custom services)
- Establish a repeatable delivery process

---

## 8. Conclusion and Suggested Next Steps

### Conclusion

Your initial idea is directionally sound and has a real foundation: **the robotic AI trend is strong, a gap exists for small modular platforms, and your existing project is a ready-made starting point.** The biggest challenge is not technical — it is **discipline**: first build "one replicable reference chassis + one standard interface," let the ecosystem grow naturally, and avoid chasing a complete platform from day one.

### Suggested Next Steps

1. **Confirm positioning**: answer first, "Who is the first reference chassis for, and what demo does it run?"
2. **Define the HAL interface**: consolidate the methods in the current `car.py` into a standard interface draft (can be written as an ADR under `docs/adr/`).
3. **Inventory the refactoring scope**: assess which parts of `nezha.py` / `car.py` / `config.py` can move directly into the platform and which need adjustment.
4. **Set an MVP milestone**: use "a new user gets the line-following demo running within 10 minutes" as the first success metric.

---

*This report is an internal research document for the ideation phase and will be updated as the project evolves.*
