# Flowchart — Autonomous IR Line Following Car System

This document provides the **Flowchart** for the autonomous IR line-following car system (excluding camera, robotic arm, and ultrasonic/sonar sensors).

It illustrates the sequential execution flow, IR sensor sampling, geometry classification, line recovery, junction state machine decisions, and motor drive outputs with smooth curved flowlines.

---

## Mermaid Flowchart Diagram

```mermaid
flowchart TD
    %% Smooth Curved Connection Lines
    linkStyle default curve basis;

    %% Custom Color Palette & Styling
    classDef startEnd fill:#1e293b,stroke:#3b82f6,stroke-width:2px,color:#ffffff;
    classDef process fill:#0f172a,stroke:#38bdf8,stroke-width:1.5px,color:#f8fafc;
    classDef decision fill:#1e1e38,stroke:#f59e0b,stroke-width:2px,color:#fef3c7;
    classDef safety fill:#450a0a,stroke:#ef4444,stroke-width:2px,color:#fee2e2;
    classDef io fill:#2e1065,stroke:#a855f7,stroke-width:1.5px,color:#f3e8ff;

    %% --------------------------------------------------
    %% 1. INITIALIZATION & PREFLIGHT
    %% --------------------------------------------------
    Start(["START: Main Control Execution<br/>(39_map1_ir_line_follow.py)"]):::startEnd

    Init["Initialize Hardware & Drivers:<br/>• NeZha I2C Bus Driver (0x40)<br/>• Yahboom 4-Channel IR Sensor (GPIO)<br/>• Navigation & Junction Sequencer"]:::process

    Preflight{"Power Health OK?<br/>(EXT5V >= 4.8V)"}:::decision
    PowerWarn["Log Power Warning & Abort Run"]:::safety
    StopExit(["STOP: Cut Motor Power & Exit"]):::startEnd

    Start --> Init
    Init --> Preflight
    Preflight -- No --> PowerWarn --> StopExit

    %% --------------------------------------------------
    %% 2. MAIN NAVIGATION LOOP
    %% --------------------------------------------------
    Preflight -- Yes --> LoopStart["Begin Main Control Loop"]:::process

    ReadSensors[["Read 4-Channel IR Sensor GPIO Pins<br/>(IRTracingSensor.raw)"]]:::io

    LoopStart --> ReadSensors

    %% --------------------------------------------------
    %% 3. IR SIGNAL PROCESSING & CLASSIFICATION
    %% --------------------------------------------------
    NormIR["Normalize & Invert Channel Readings<br/>Map to Physical Array P1..P4<br/>(1 = Black Line, 0 = White)"]:::process

    ClassifyIR["Classify Geometry (classify)<br/>Lookup IRState in STATE_TABLE<br/>Calculate Offset (cm) & Error Fraction"]:::process

    ReadSensors --> NormIR --> ClassifyIR

    LineVisible{"Line Visible?<br/>any(P1..P4) == 1"}:::decision

    ClassifyIR --> LineVisible

    %% --------------------------------------------------
    %% 4. BRANCH A: LINE NOT VISIBLE (0000)
    %% --------------------------------------------------
    BlindBand{"In Blind Band (0.8 cm)?<br/>(Prev State 0010/0100)"}:::decision
    MaintainCreep["Maintain Prev Steering Vector<br/>Creep Forward Through Blind Band"]:::process

    SearchMode["Enter SEARCH State:<br/>Sweep ±SweepDeg Left/Right<br/>Probe Forward Step-by-Step"]:::process
    SearchTimeout{"Search Timeout?<br/>(Give-up Time Exceeded)"}:::decision

    LineVisible -- No --> BlindBand
    BlindBand -- Yes --> MaintainCreep --> DriveMotors
    BlindBand -- No --> SearchMode --> SearchTimeout
    SearchTimeout -- Yes --> PowerWarn
    SearchTimeout -- No --> ReadSensors

    %% --------------------------------------------------
    %% 5. BRANCH B: LINE VISIBLE (TRACKING & JUNCTIONS)
    %% --------------------------------------------------
    JunctionCheck{"Junction Feature?<br/>Matches SequenceStep Pattern?"}:::decision

    LineVisible -- Yes --> JunctionCheck

    %% Junction Execution Sub-branch
    TargetReached{"Final Lap Target<br/>Junction Reached?"}:::decision
    ExecJunction["Execute Junction Action:<br/>• Pivot Spin Turn (Right/Left)<br/>• Cross Junction Straight"]:::process

    JunctionCheck -- Yes --> TargetReached
    TargetReached -- Yes --> FinishRun["Set State = STOPPED<br/>Issue Motor Stop (0, 0)"]:::process --> StopExit
    TargetReached -- No --> ExecJunction --> DriveMotors

    %% Line Tracking Sub-branch
    CalcSpeed["Compute Differential Speeds:<br/>wheel_speeds(IRState, Speed)"]:::process

    JunctionCheck -- No --> CalcSpeed --> DriveMotors

    %% --------------------------------------------------
    %% 6. MOTOR DRIVE OUTPUT
    %% --------------------------------------------------
    DriveMotors[["Send Motor Speeds to NeZha Driver:<br/>Car.drive(left_speed, right_speed)"]]:::io

    DriveMotors --> LoopStart
```

---

## Process Phase Breakdown

| Phase | Description | Key Source Functions |
|---|---|---|
| **Initialization** | Initializes I2C bus connection at `0x40`, configures GPIO pin mappings for the Yahboom 4-channel IR sensor, and establishes the scripted route plan. | [`NeZha.__init__`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/nezha.py), [`IRTracingSensor`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_tracing.py) |
| **Power Check** | Validates supply voltage ($\ge 4.8\,\text{V}$) before enabling motor movement. | [`check_power_health`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/power.py) |
| **Signal Normalization** | Reads 4 digital GPIO pins, applies inversion mapping, and produces physical channel array $(P1..P4)$ where $1=\text{black line}$ and $0=\text{white}$. | [`IRTracingSensor.read`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_tracing.py#L63) |
| **Geometry Classification** | Classifies the 4-bit reading against `STATE_TABLE` into line position offsets ($\text{cm}$) and identifies blind band conditions. | [`classify`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_geometry.py#L225), [`STATE_TABLE`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_geometry.py) |
| **Junction State Machine** | Sequences multi-step pattern history to recognize track corners/junctions and execute turns or final lap stops. | [`JunctionSequencer`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_route.py#L273), [`IRLineNav`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_line_nav.py) |
| **Line Recovery Search** | Sweeps left/right and creeps forward step-by-step if the track line is lost (`0000`). | [`IRSearchPhase`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_line_nav.py#L126) |
| **Motor Drive Control** | Computes differential wheel speeds and transmits motor control bytes over I2C to NeZha ports `M1`..`M4`. | [`Car.drive`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/car.py#L62), [`wheel_speeds`](file:///Users/dannyyu/Desktop/IRsensorCar/Car-and-Robotic-Arm/src/carbot/ir_geometry.py#L254) |
