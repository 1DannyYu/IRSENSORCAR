# Project Terminology

> **Project:** Modular Robot Base Platform
> **Related:** [docs/robot-base-platform-research.md](robot-base-platform-research.md)
> **Note:** This glossary collects English terminology used across the project research and development, grouped into five categories: Domain & Concepts, Hardware, Software & Architecture, Methods & Process, and Business & Ecosystem.

---

## A. Domain & Concepts

| Term | Definition |
|---|---|
| **Robotic AI** | The application of artificial intelligence (perception, decision-making, learning) to robots, enabling them to operate autonomously or semi-autonomously. |
| **Embodied AI** | AI that has a physical body (hardware carrier) and learns by sensing and interacting with the environment; robots are a typical embodiment. |
| **LLM (Large Language Model)** | A neural network model capable of understanding and generating natural language; used in robots to interpret human commands (e.g., "bring me the cup"). |
| **Closed-loop control** | A control method that continuously corrects actions based on sensor feedback; the opposite of open-loop execution. |
| **Open-loop control** | A control method that executes preset commands without consulting feedback; suitable for predictable, precise scenarios. |
| **Autonomous vehicle** | A vehicle capable of navigation and decision-making on its own; this project's smart car is a small-scale example. |
| **Perception** | The process by which a robot understands its environment through cameras and sensors (object detection, distance measurement, etc.). |
| **Planner** | The software module that decides the next action (route, steering, avoidance) based on perception results. |
| **Actuator** | A component that converts electrical signals into mechanical motion (e.g., DC motors, servos). |
| **Obstacle avoidance** | The ability to detect and navigate around obstacles; a core capability of autonomous vehicles. |
| **Vision-guided** | Operation driven primarily by camera imagery as the decision-making input. |
| **Modularity** | Decomposing a system into independently developed, replaceable, and reconfigurable modules; the core principle behind platformization. |
| **Platformization** | The transition from a single product into an infrastructure on which third parties can develop. |
| **Ecosystem** | The network of participants (developers, users, package providers) built around a platform, whose value grows as participation increases. |
| **Network effect** | The phenomenon where a platform's value rises as the number of users/participants grows. |

---

## B. Hardware

| Term | Definition |
|---|---|
| **Raspberry Pi** | A low-cost single-board computer; this project uses a Raspberry Pi 5 as the main controller. |
| **Chassis** | The vehicle body that carries motors, controller boards, and sensors; this project uses the Dasheng multi-form robot chassis. |
| **Driver board** | A board that receives controller signals and drives motors/servos; this project uses the Yourfun NeZha driver board. |
| **NeZha** | Yourfun's bus driver board; controlled over I2C to drive DC motors, servos, LEDs, and encoders. |
| **I2C (Inter-Integrated Circuit)** | A low-speed serial communication protocol between chips; this project controls the NeZha over I2C at address `0x40`. |
| **PWM (Pulse-Width Modulation)** | A technique using the duty cycle of a square wave to control motor speed or servo angle. |
| **DC motor** | A motor driven by direct current; the chassis uses 4× N20 DC motors. |
| **Servo** | An actuator with precise angular control; supported by the NeZha board and previously used by the now-removed robotic arm. |
| **Encoder** | A sensor that reports motor speed/position, enabling closed-loop speed control. |
| **Tracking sensor** | A sensor that detects a route (e.g., a line on the ground) or tracks a target. |
| **Camera** | A sensor providing visual imagery; combined with OpenCV/ML for object detection. |
| **HAL (Hardware Abstraction Layer)** | A software layer that unifies hardware interfaces so one API can drive different chassis/driver boards. |
| **Reference chassis** | The official baseline chassis design that defines compatibility for the platform. |

---

## C. Software & Architecture

| Term | Definition |
|---|---|
| **SDK (Software Development Kit)** | Libraries and tools provided to developers so third parties can build on the platform. |
| **Drivers** | The software layer that communicates directly with hardware and encapsulates low-level operations (e.g., `nezha.py`). |
| **Abstraction layer** | A software layer that hides underlying details and exposes a unified interface. |
| **Standard interface** | The unified API specification defined by the platform (e.g., `BaseChassis` with `forward() / turn() / stop()`). |
| **Skill (package)** | An installable package encapsulating a specific capability (line following, obstacle avoidance, target tracking). |
| **Plugin / Package** | A software module that can be independently installed to extend platform functionality. |
| **Config / Settings** | Configuration that separates hardware differences (motor direction, I2C address) from code (e.g., `config.py`). |
| **OpenCV** | An open-source computer vision library used for image processing and object detection. |
| **ML (Machine Learning)** | Techniques that let models learn from data; used for perception tasks such as visual recognition. |
| **IPython / Kernel** | A persistent interactive Python environment, convenient for experimentation and control. |
| **Simulation environment** | A virtual environment for testing software without physical hardware, reducing cost and risk. |
| **DX (Developer Experience)** | How smoothly developers can work with the platform (documentation, time-to-first-run, debugging tools). |

---

## D. Methods & Process

| Term | Definition |
|---|---|
| **Folio** | An assessment document recording the complete project process from definition to testing (e.g., the school AT2 folio). |
| **Define → Plan → Build → Test** | The four phases of the system development process; folio chapters follow this structure. |
| **Function requirements** | Specifications of what the system must do. |
| **User input requirements** | Specifications of how users operate the system (buttons, remote control, commands, etc.). |
| **Output requirements** | Specifications of the system's output forms (motion, display, logs). |
| **UML (Unified Modeling Language)** | A standard for describing software structure and behavior with diagrams (class diagrams, sequence diagrams, etc.). |
| **Structure Chart** | A hierarchical diagram showing how a system's functions decompose. |
| **Algorithm / Pseudocode** | Describing solution steps in a form close to natural language. |
| **Unit test** | The smallest test targeting a single software module. |
| **MVP (Minimum Viable Product)** | The smallest product version that validates core value with minimal resources. |
| **Milestone** | A verifiable phase goal within the project. |
| **ADR (Architecture Decision Record)** | A document recording significant technical decisions and their rationale (stored under `docs/adr/`). |

---

## E. Business & Ecosystem

| Term | Definition |
|---|---|
| **Value proposition** | The unique value and reasons a product offers to customers. |
| **Target customer / Persona** | A description of the characteristics and needs of potential users (developers, educators, small companies, etc.). |
| **Business model** | The design of how the platform creates and captures value (open source + hardware kits, custom services, marketplace, etc.). |
| **Open source** | A licensing model where source code is public and can be used and modified by others; helps build an ecosystem. |
| **Revenue stream** | The channels through which the platform earns money (hardware sales, service fees, subscriptions, etc.). |
| **Go-to-market** | The strategy and channels for bringing the product to the target market. |
| **SWOT (Strengths, Weaknesses, Opportunities, Threats)** | An analytical framework evaluating internal strengths/weaknesses and external opportunities/threats. |
| **Roadmap** | A blueprint of future phases and timelines for the project. |

---

*This glossary will be extended as the project evolves; add new terms in English under the appropriate category.*
