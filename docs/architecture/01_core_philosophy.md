# Core Philosophy: The "Flat" Simulation

**Related:** [00_index.md](00_index.md) | [02_component_protocol.md](02_component_protocol.md) | [03_signal_backplane.md](03_signal_backplane.md)

---

## 1. Fundamental Decision

The fundamental architectural decision for Icarus is to reject deep object-oriented hierarchies (Scene -> Entity -> Component) in favor of a **Flat, Data-Oriented** topology.

This approach aligns with:
1. **Simulink/GNC Patterns:** Aerospace engineers model systems as block diagrams, not inheritance trees.
2. **Janus Symbolic compatibility:** `casadi::MX` graph generation requires a linear or topologically sorted execution trace. Deep branching logic obscures this graph.
3. **Data Locality:** Keeping state vectors contiguous for ODE solvers/optimizers.

### The Defining Rule

> **"The Gravity Model, the Atmosphere, the Spacecraft, and the Fuel Pump are all structural peers."**

There is no "World" object that *contains* the vehicle. There is a simulation that contains a list of components, some of which calculate gravity, and some of which calculate fuel flow.

---

## 2. Terminology & Concepts

To avoid ambiguity, we define the following taxonomy:

### Core Concepts

| Term | Definition | Context |
| :--- | :--- | :--- |
| **Component** | The fundamental unit of **Execution**. A C++ class that implements `Provision()`, `Stage()`, and `Step()`. It owns State and Logic. | `class JetEngine : public Component` |
| **Model** | A stateless unit of **Physics**. A pure function or helper class (usually from **Vulcan**) that performs a standard calculation. Components *use* Models. | `vulcan::atmosphere::usa76` |
| **System** | The physical concept being simulated. A System is often implemented by one or more Components. | "The Propulsion System" |
| **Entity** | A virtual unit of **Identity**. It does **not** exist as a C++ object. It is a **Namespace** acting as a prefix for signal organization. | "Falcon9" in `Falcon9.Propulsion.Thrust` |

### Data Concepts

| Term | Definition | Context |
| :--- | :--- | :--- |
| **Signal** | A discrete unit of **Data** on the Backplane. Types: `double`, `int32`, `int64`. Booleans use `int32` (0/1). | `nav.altitude`, `gnc.mode`, `aero.ref_area` |
| **Backplane** | The centralized **Registry** where all Signals are published. **All observable/configurable numeric data lives here.** | `std::map<string, double*>` |
| **Signal Lifecycle** | A signal property: **Static** or **Dynamic**. Static signals are immutable after Stage. Dynamic signals are updated every Step. | `mass.dry` (static), `nav.vel` (dynamic) |
| **Parameter** | A **Static Signal** loaded from config. Parameters are set at Provision/Stage and do not change during the run. Parameters ARE static signals—the terms are synonymous. | `max_thrust`, `reference_area`, `time_constant` |
| **State** | The subset of Dynamic Signals that persist across time steps and require integration (differential equations). Components write derivatives; the integrator updates state. | `FuelMass`, `Velocity`, `spool_speed` |
| **Derivative** | The rate of change of a State signal. Written by components during Step, consumed by the integrator. | `state_dot_velocity`, `fuel_flow_rate` |

### Component Lifecycle Phases

| Phase | When Called | Purpose | Key Actions |
| :--- | :--- | :--- | :--- |
| **Provision** | Once at application launch | Heavy lifting—allocate memory, compile graphs, parse assets | Register outputs on Backplane, load parameters from config, allocate state vectors |
| **Stage** | Start of each episode/run | Prepare for $t=0$—reset state, solve trim, wire inputs | Resolve input pointers from Backplane, apply initial conditions, run trim solver if needed |
| **Step** | Every time step (thousands/run) | Advance simulation by $\Delta t$ | Read inputs, compute derivatives, write outputs. **NO allocation, NO string lookups.** |

### Extended Lifecycle Hooks (Optional)

| Hook | When Called | Purpose |
| :--- | :--- | :--- |
| **PreStep** | Before any component's `Step()` | Pre-processing, sensor aggregation, input conditioning |
| **PostStep** | After all components' `Step()` | Post-processing, logging, output aggregation |
| **OnPhaseEnter** | When flight phase changes | Phase-specific initialization (e.g., booster ignition) |
| **OnPhaseExit** | When leaving a flight phase | Phase-specific cleanup (e.g., log total impulse) |
| **OnError** | When simulation encounters an error | Graceful degradation, safe mode activation |
| **Shutdown** | Application termination | Cleanup, flush buffers, release resources |

> [!NOTE]
> **Non-numeric config** (file paths, string identifiers) is NOT a Signal. It's loaded at Provision time into component internals. Only numeric values go on the Backplane.

---

## 3. The "Is-A" vs "Has-A" vs "Uses-A" Relationship

* An **Entity** (Falcon9) is just a label.
* A **Component** (EngineComponent) *belongs to* an Entity (conceptually).
* A **Component** *has* **State** (RPM).
* A **Component** *uses* a **Model** (Thermodynamics) to compute derivatives.
* A **Component** *reads/writes* **Signals** via the **Backplane**.
