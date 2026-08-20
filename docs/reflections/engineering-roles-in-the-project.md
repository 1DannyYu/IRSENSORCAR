# Engineering Roles in the Project

## Purpose

This report explains the different responsibilities of a software engineer, a large language
model engineer, and a physical AI engineer in the Car and Robotic Arm project. The roles overlap,
but they solve different classes of problems and carry different safety responsibilities.

## Role Summary

| Role | Primary objective | Main project responsibility |
|---|---|---|
| Software engineer | Make the system maintainable, testable, and reproducible | Software architecture, interfaces, tests, telemetry, replay, and deployment |
| Large language model engineer | Make project evidence and tools usable by an AI assistant | Retrieval, tool calling, agent workflows, evaluation, and approval boundaries |
| Physical AI engineer | Make the robot perceive, decide, and act safely in the real world | Sensors, localisation, mapping, planning, control, calibration, and safety |

The software engineer makes the system reliable as software. The large language model engineer
makes engineering information easier to analyse and use. The physical AI engineer makes the
machine reliable in its physical environment.

## Software Engineer

### Mission

The software engineer builds the engineering foundation used by every sensing, mapping, planning,
and control function. This role does not primarily decide where the robot should travel. It makes
those decisions implementable, observable, testable, and maintainable.

### Responsibilities in this project

- Maintain hardware abstractions for the NeZha board, motors, sonar, camera, and future sensors.
- Separate reusable package code from demonstration and verification programs.
- Define configuration, calibration, and command-line interfaces.
- Build unit, integration, regression, and recorded-scenario tests.
- Guarantee resource cleanup and motor stop behaviour during errors or process termination.
- Record run manifests, sensor telemetry, decisions, actions, and evaluation results.
- Build offline replay so historical failures can be tested without moving the robot.
- Keep Raspberry Pi and workstation environments understandable and reproducible.
- Provide safe deployment, process supervision, storage management, and log rotation.

### Typical deliverables

- A reliable `carbot` package with stable interfaces.
- Automated tests and static checks.
- A run-recording and replay framework.
- Machine-readable configuration and evaluation results.
- Safe process lifecycle and deployment procedures.

### Success criteria

The software engineer succeeds when identical evidence can be replayed, failures can be located,
changes can be compared against known baselines, and an exception cannot leave the motors running.

## Large Language Model Engineer

### Mission

The large language model engineer makes an LLM useful inside the engineering workflow. In this
project, that means building an evidence-grounded engineering assistant, not training a new
foundation model and not allowing a chatbot to drive the robot directly.

### Responsibilities in this project

- Make hardware documentation, architecture decisions, calibration records, progress reports, and
  historical runs searchable by the model.
- Design tools for inspecting runs, comparing results, evaluating reconstructions, and finding
  related failures.
- Require numerical conclusions to come from deterministic tools rather than language-model
  estimation.
- Build workflows that connect operational evidence to failure hypotheses, requirements, and test
  scenarios.
- Evaluate whether the model classifies known failures consistently and cites supporting evidence.
- Detect unsupported claims and require the model to report when evidence is insufficient.
- Define permission boundaries and human approval gates for high-risk operations.

### Suitable agent tools

An engineering agent could safely receive tools such as:

```text
inspect_run(run_id)
compare_runs(first_run, second_run)
evaluate_reconstruction(run_id)
find_related_failures(signature)
propose_test_case(requirement_id)
```

It should not receive unrestricted PWM control, arbitrary motor-port access, permission to bypass
preflight, or authority to begin a motion test.

### Typical deliverables

- Project knowledge retrieval.
- A run-analysis agent.
- Tool schemas and permission boundaries.
- A ground-truth evaluation set for known failure cases.
- Human approval workflows and unsupported-claim checks.

### Success criteria

The large language model engineer succeeds when every important conclusion is linked to project
evidence, repeated cases receive consistent analysis, uncertainty is reported honestly, and the
agent shortens investigation time without gaining unsafe hardware authority.

## Physical AI Engineer

### Mission

The physical AI engineer connects computation to the real world. This role is responsible for how
the robot senses its environment, estimates its position, plans movement, controls its actuators,
and remains safe when sensors or assumptions fail.

### Responsibilities in this project

#### Perception

- Integrate IMX500 object detection, ultrasonic distance, image quality, and fiducial-marker pose.
- Characterise each sensor's field of view, blind zone, noise, latency, update rate, and failure
  modes.
- Add and calibrate future wheel encoders, an IMU, or other sensors when evidence justifies them.

#### Localisation and mapping

- Recover and validate camera trajectories through Structure-from-Motion.
- Anchor reconstruction scale and coordinate frames.
- Estimate the ground plane and convert sparse geometry into a usable floor map.
- Detect walls and produce dimension estimates with measured uncertainty.
- Evaluate whether vision, encoders, and inertial sensing should be fused.

#### Planning and control

- Replace random-bounce routing with deliberate perimeter and interior coverage when required.
- Detect repeated locations, stalled progress, and trapped avoidance loops.
- Define coverage, completion, return-to-start, and battery-aware behaviours.
- Account for wheel slip, floor material, battery condition, payload, and open-loop control error.

#### Safety

- Define the operational design domain in which the robot is allowed to run.
- Place a deterministic safety supervisor between proposed actions and motor commands.
- Bound speed, duration, sensor age, power condition, and obstacle clearance.
- Validate watchdog, heartbeat-loss, emergency-stop, and safe-stop behaviour.
- Design supervised hardware experiments that isolate variables and limit physical risk.

### Typical deliverables

- Sensor models and calibration records.
- Localisation and mapping pipelines.
- Coverage and recovery policies.
- A deterministic safety supervisor.
- Simulation, replay, and Hardware-in-the-Loop scenarios.
- Quantified operational limits and hardware acceptance tests.

### Success criteria

The physical AI engineer succeeds when the robot completes its mapping task within a defined
environment, reports measurable accuracy, stops safely under uncertainty, and clearly identifies
conditions in which it must not operate.

## How the Roles Work Together

A complete room-mapping cycle should follow this pattern:

```text
Physical AI engineer defines perception, control, safety, and acceptance criteria
-> Software engineer implements testable modules, telemetry, replay, and deployment
-> LLM engineer enables evidence-grounded run analysis and experiment proposals
-> Human engineer approves the supervised hardware test
-> Physical AI engineer evaluates real-world behaviour
-> Software engineer preserves the run as structured regression evidence
-> LLM agent compares the result with earlier runs and proposes the next investigation
```

For example, if the robot repeatedly rotates beside a chair, the software engineer preserves the
sensor and action sequence and creates a replay fixture. The LLM engineer enables an agent to find
similar historical failures and identify the relevant evidence. The physical AI engineer decides
whether the cause is a sensor blind zone, a perception error, or a routing-policy failure, then
designs a safe validation experiment.

## Recommended Emphasis for the Current Project Stage

The project currently needs the three disciplines in approximately this order:

| Work area | Suggested emphasis |
|---|---:|
| Software engineering and validation infrastructure | 40% |
| Physical AI, mapping, control, and supervised hardware validation | 50% |
| LLM agent and analysis automation | 10% |

The LLM share can increase after structured telemetry, offline replay, and a labelled failure
corpus exist. Until then, an LLM would mostly interpret prose and make hypotheses from incomplete
evidence. Responsibility for motor safety, mapping accuracy, and operational limits must remain
with deterministic validation and human engineering judgement.

## Conclusion

The three roles form a dependency chain rather than a competition. Software engineering creates a
reliable foundation, physical AI engineering makes that foundation work safely in the real world,
and large language model engineering accelerates the use of accumulated evidence. The project will
benefit most by strengthening reproducibility and physical validation first, then adding an
evidence-grounded engineering agent on top of that foundation.
