# Structure Chart

## Legend & Symbols

| Symbol | Element | Description |
| :---: | :--- | :--- |
| `[ Module Name ]` | **Module** | Functional unit/procedure (simplified, generic description). |
| `───►` | **Call Line** | Direct invocation from a parent module to a sub-module. |
| `○` | **Data Couple** | Parameter or return value passed between modules (Open Circle). |
| `●` | **Control Couple** | Status flag or signal used for decision-making (Filled Circle). |
| `◇` | **Conditional Selection** | Decision point indicating a module is invoked conditionally. |
| `↻` | **Repetition Loop** | Iterative call loop executing repeatedly during runtime. |

---

## Structure Chart Diagram

```mermaid
%%{init: {'flowchart': {'curve': 'linear'}}}%%
flowchart TD
    %% Level 0 - Main Module
    Main["Main Control Module"]

    %% Level 1 Sub-modules
    Init["System Initialization"]
    Sense["Sensory Processing"]
    Nav["Navigation & Guidance Engine"]
    Motor["Motor Drive Controller"]

    %% Level 0 -> Level 1 Invocation & Couples
    Main -->|"● Init Ready Flag"| Init
    Main -->|"↻ Main Loop<br>○ Line Offset<br>● Line Detected Flag"| Sense
    Main -->|"↻ Main Loop<br>○ Steering Command"| Nav
    Main -->|"↻ Main Loop<br>● Motor Output Sent"| Motor

    %% Level 1 -> Level 2: System Initialization
    PwrCheck["Check Power Health"]
    DrvConfig["Configure Hardware Drivers"]
    Init -->|"● Power OK Flag"| PwrCheck
    Init -->|"● Drivers Active Flag"| DrvConfig

    %% Level 1 -> Level 2: Sensory Processing
    ReadIR["Read Sensor Channels"]
    MapIR["Map & Normalize Inputs"]
    CalcPos["Calculate Line Offset"]
    Sense -->|"○ Raw Signals"| ReadIR
    Sense -->|"○ Sensor State"| MapIR
    Sense -->|"○ Line Offset<br>● Line Detected Flag"| CalcPos

    %% Level 1 -> Level 2: Navigation & Guidance
    TrackLine["Compute Tracking Steering"]
    CreepGap["Maintain Creep in Blind Spot"]
    SearchLine["Execute Line Recovery Search"]
    ExecJunct["Execute Junction Maneuver"]

    Nav -->|"◇ Line Visible<br>○ Steering Angle"| TrackLine
    Nav -->|"◇ Blind Spot<br>○ Creep Vector"| CreepGap
    Nav -->|"◇ Line Lost<br>○ Sweep Direction<br>● Search Timeout"| SearchLine
    Nav -->|"◇ Junction Detected<br>○ Turn Direction<br>● Destination Reached"| ExecJunct

    %% Level 1 -> Level 2: Motor Drive Controller
    CalcSpeeds["Compute Wheel Speeds"]
    SendDrive["Transmit Motor Commands"]
    Motor -->|"○ Left/Right Velocities"| CalcSpeeds
    Motor -->|"Transmit Command"| SendDrive

    %% Styling
    classDef main fill:#1e293b,stroke:#3b82f6,stroke-width:2px,color:#ffffff
    classDef l1 fill:#0f172a,stroke:#38bdf8,stroke-width:1.5px,color:#f8fafc
    classDef l2 fill:#1e1e38,stroke:#a855f7,stroke-width:1.5px,color:#f3e8ff

    class Main main
    class Init,Sense,Nav,Motor l1
    class PwrCheck,DrvConfig,ReadIR,MapIR,CalcPos,TrackLine,CreepGap,SearchLine,ExecJunct,CalcSpeeds,SendDrive l2
```
