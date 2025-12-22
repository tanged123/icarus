# Icarus Data-Oriented Architecture (IDOA)

**Status:** Draft / Proposed
**Supersedes:** Hierarchical "Scene Graph" approaches
**Context:** Icarus 6DOF Simulation Engine

## 1. Core Philosophy: The "Flat" Simulation

The fundamental architectural decision for Icarus is to reject deep object-oriented hierarchies (Scene -> Entity -> Component) in favor of a **Flat, Data-Oriented** topology.

This approach aligns with:
1.  **Simulink/GNC Patterns:** Aerospace engineers model systems as block diagrams, not inheritance trees.
2.  **Janus Symbolic compatibility:** `casadi::MX` graph generation requires a linear or topologically sorted execution trace. Deep branching logic obscures this graph.
3.  **Data Locality:** Keeping state vectors contiguous for ODE solvers/optimizers.

### The Defining Rule
**"The Gravity Model, the Atmosphere, the Spacecraft, and the Fuel Pump are all structural peers."**

There is no "World" object that *contains* the vehicle. There is a simulation that contains a list of components, some of which calculate gravity, and some of which calculate fuel flow.

## 2. Terminology & Concepts

To avoid ambiguity, we define the following taxonomy:

| Term | Definition | Context |
| :--- | :--- | :--- |
| **Component** | The fundamental unit of **Execution**. A C++ class that implements `Stage()` and `Step()`. It owns State and Logic. | `class JetEngine : public Component` |
| **Model** | A stateless unit of **Physics**. A pure function or helper class (usually from **Vulcan**) that performs a standard calculation. Components *use* Models. | `vulcan::atmosphere::usa76` |
| **System** | The physical concept being simulated. A System is often implemented by one or more Components. | "The Propulsion System" |
| **Entity** | A virtual unit of **Identity**. It does **not** exist as a C++ object. It is a **Namespace** acting as a prefix for signal organization. | "Falcon9" in `Falcon9.Propulsion.Thrust` |
| **Signal** | A discrete unit of **Data** on the Backplane. Types: `double`, `int32`, `int64`. Booleans use `int32` (0/1). | `nav.altitude`, `gnc.mode`, `aero.ref_area` |
| **Backplane** | The centralized **Registry** where all Signals are published. **All observable/configurable numeric data lives here.** | `std::map<string, double*>` |
| **Lifecycle** | A signal property: **Static** (set at Stage, immutable) or **Dynamic** (updated every Step). Static signals hold "parameter-like" config values. | `mass.dry` (static), `nav.vel` (dynamic) |
| **State** | The subset of Signals that persist across time steps and require integration (differential equations). | `FuelMass`, `Velocity` |

> [!NOTE]
> **Non-numeric config** (file paths, string identifiers) is NOT a Signal. It's loaded at Provision time into component internals. Only numeric values go on the Backplane.

### The "Is-A" vs "Has-A" vs "Uses-A" Relationship
*   An **Entity** (Falcon9) is just a label.
*   A **Component** (EngineComponent) *belongs to* an Entity (conceptually).
*   A **Component** *has* **State** (RPM).
*   A **Component** *uses* a **Model** (Thermodynamics) to compute derivatives.
*   A **Component** *reads/writes* **Signals** via the **Backplane**.

## 4. The Signal Backplane

In a hierarchical system, data flows via pointers: `parent->child->doThing()`.
In IDOA, data flows via the **Signal Backplane**.

### Concept
The Backplane is a globally addressable registry of all "Signals" in the simulation. It aligns with the findings in `design/signal_system_analysis.md`, implementing a **Hybrid Pointer + Metadata** model.

*   **Runtime:** Fast pointer-based access (using `SignalID` to resolve pointers during `Stage`).
*   **Introspection:** Rich metadata for UI, Telemetry, and Validation.

### Signal Definition
A signal is more than just a value. It defines:

| Property | Description | Examples |
| :--- | :--- | :--- |
| **Type** | The data type. Supported: `double` (default), `int32` (modes), `int64` (counters). | `nav.altitude` (double), `gnc.mode` (int32) |
| **Lifecycle** | **Static** (Constant after Stage) or **Dynamic** (Updated every Step). | `mass.dry` (static), `nav.vel` (dynamic) |
| **Units** | Physical unit for validation and display. | "m/s", "kg", "rad" |

### Implementation Details

#### 1. The Registry (`map<string, SignalMetadata>`)
Holds the "Truth" about what exists. Used during **Provision** and **Stage** for resolution.

```cpp
struct SignalMetadata {
    uint32_t id;          // Unique Hash
    string name;          // "Vehicle.Aero.Cl"
    DataType type;        // DOUBLE, INT32...
    Lifecycle lifecycle;  // STATIC, DYNAMIC
    void* data_ptr;       // Pointer to storage
};
```

#### 2. The Storage (`vector<double>`, `vector<int>`)
Contiguous memory blocks for each type.
*   **Optimization:** Static signals are stored separately.
*   **Telemetry:** Only the "Dynamic Double" vector is streamed over UDP (60Hz). Static signals are sent once via Schema.

#### 3. Component Access ("Bind Once, Read Forever")
Components do **not** look up signals by string every Step. They bind pointers during Stage.

```cpp
// Stage()
this->ptr_density_ = backplane.resolve<double>("Env.Atm.Density");

// Step()
double rho = *this->ptr_density_; // Zero overhead
```

#### 4. Convenience: Grouped Binding (Optional Helper)
For ergonomics, a **convenience layer** can bind multiple signals matching a pattern into a struct. This is **not** a core Backplane feature—it's syntactic sugar.

```cpp
// Define a struct matching the signal namespace
struct NavState {
    double* pos_x;
    double* pos_y;
    double* pos_z;
    double* vel_x;
    // ...
};

// Stage() - bind all Nav.* signals at once
NavState nav = backplane.bind_group<NavState>("Vehicle.Nav");

// Step() - access like normal
double x = *nav.pos_x;
```

Internally, `bind_group` just calls `resolve()` for each member. The Backplane remains flat; this is purely a compile-time convenience.

## 5. Simulation Lifecycle & Component Protocol

The architecture strictly separates memory management (**Provision**) from physics initialization (**Stage**) and temporal evolution (**Step**). This ensures high performance for Monte Carlo loops by eliminating redundant memory allocation.

Every component behaves identically to the `Simulator` host:

### Phase I: Provision (Setup)
**Goal:** Heavy lifting. Allocate memory, compile compute graphs, parse immutable assets (URDFs, Meshes).
**Frequency:** Called once when the application launches.

*   **Action:** Allocate large state vectors and signal buffers.
*   **Action:** Register public outputs in the Backplane.
*   **Action:** Declare parameter structs.
*   **Action:** Load static reference tables (atmosphere, gravity).

### Phase II: Stage (The Merged Reset/Trim)
**Goal:** Prepare the mathematical state for $t=0$. This merges "Reset" (hard values) and "Trim" (solved values) into a single pipeline.
**Frequency:** Called at the start of every new episode/run.

The `Stage(RunConfig)` function acts as a solver pipeline:
1.  **Zeroing:** Wipe state derivatives ($\dot{x}$) and time ($t=0$).
2.  **Hard Constraints (Reset):** Apply fixed values from config (e.g., `Position = [0,0,1000]`).
3.  **Soft Constraints (Trim):** If `RunConfig.mode == EQUILIBRIUM`, we can leverage the **Dual-Headed** nature of Janus:
    *   *Numeric Trim:* Simple Newton-Raphson on the C++ `double` step function for basic cases.
    *   *Symbolic Trim:* Instantiate a temporary `Simulator<casadi::MX>`, trace the forces calculation graph $\sum F(x, u)$, and pass it to a robust nonlinear solver (IPOPT/Kinsol) to find the exact trim state. This is significantly more robust for complex, highly nonlinear vehicles.
4.  **Binding:** Resolve input pointers from the Backplane.

### Phase III: Step (Execute)
**Goal:** Advance the simulation by a discrete time delta ($\Delta t$).
**Frequency:** Called in a loop (thousands of times per episode).

*   **Action:** Read `*input_ptr` from Backplane.
*   **Action:** Calculate dynamics $\dot{x} = f(x, u)$.
*   **Action:** Write `*output_ptr` to Backplane.
*   **Constraint:** NO memory allocation. NO looking up signals by string (do that in Stage).

## 6. Execution Model

Since the hierarchy is flat, execution order is determined by **Dependency Analysis** (Topological Sort) or **Explicit Rate Grouping**.

### The Scheduler
disk
The `Scheduler` holds `std::vector<Component*>`.

```cpp
// The entire simulation loop is essentially:
for (Component* c : scheduled_components) {
    c->step(t, dt);
}
```

### Rate Groups
We support multi-rate execution by having multiple lists:
*   **Group 1kHz:** IMU, Actuators, EOMs.
*   **Group 100Hz:** GNC, Guidance Logic.
*   **Group 10Hz:** Telemetry, Logging, Environmental (Slow).

## 7. "Entities" as Namespaces

If the simulation is flat, how do we handle multiple vehicles? Or a "Rocket" with "Stages"?

**Entities are Virtual.** They are nothing more than a standardized **Namespace Prefix** in the Signal Backplane.

### Example: Multi-Stage Rocket
We don't have a `class Rocket` containing `class Stage`.
We have a config file that generates components with prefixes:

1.  `Falcon.Stage1.Engine.Thrust`
2.  `Falcon.Stage2.Engine.Thrust`
3.  `Falcon.Interstage.SeparationLogic.Active`

The "Separation" logic is just a component that monitors `Stage1` signals and affects `Stage2` signals. It doesn't need to "own" them.

## 8. Janus Integration (The "Template-First" Paradigm)

This architecture is specifically designed to support the **Janus** "Template-First" paradigm, enabling a dual-backend type system.

### The "Scalar" Concept
Components are **not** written for `double` or `casadi::MX`. They are written for a generic `Scalar`.

```cpp
template <typename Scalar>
void Aerodynamics::Step(Scalar& t, Scalar dt) {
    // GOOD: Uses Janus dispatch
    Scalar q = 0.5 * rho * janus::pow(v, 2); 
    
    // BAD: Uses std::Math or raw types
    // double q = 0.5 * rho * std::pow(v, 2); 
}
```

### Numeric Mode (`Simulator<double>`)
*   **Backend:** `Janus::Scalar` resolves to `double`.
*   **Execution:** Fast, compiled C++ assembly. 
*   **Use Case:** Real-time HIL, Monte Carlo, Visualizer connection.

### Symbolic Mode (`Simulator<janus::SymbolicScalar>`)
*   **Backend:** `Janus::Scalar` resolves to `casadi::MX`.
*   **Execution:** Graph recording.
*   **Use Case:** Trajectory Optimization, Sensitivity Analysis.

### Optimization-Only Workflow ("Bridging the Gap")
We can support a workflow that exists purely to bridge Numeric Simulation and Symbolic Optimization:

1.  **Provision** the `Simulator<janus::SymbolicScalar>`.
2.  **Stage** to set the topology.
3.  **Do NOT Step** in a loop.
4.  Instead, export the symbolic function:
    `casadi::Function F_dynamics = sim.GenerateGraph("dynamics");`
5.  This `F_dynamics` is then handed to an external offline trajectory optimizer (using `janus::Opti` or Direct Collocation).

This ensures that the **Optimizer** and the **Simulator** use the *exact same physics code*, guaranteed by the shared `template <typename Scalar>` component implementation. No more maintaining a separate "low fidelity" model for optimization.

## 9. Vulcan Integration (Engineering Library)

**Vulcan** is the domain-specific engineering library that sits between Janus (Math) and Icarus (Architecture).

### Division of Responsibility: "Bricks vs. Houses"

*   **Janus:** The **Math** (Matrices, Autodiff, Solvers).
*   **Vulcan:** The **Physics Utilities** (Stateless). "Here is the equation for Drag." "Here is the US Standard Atmosphere."
*   **Icarus:** The **Systems & State** (Stateful). "I am a Vehicle with a specific Drag Area." "I am an Engine with a current RPM."

| Feature | Vulcan (Utility) | Icarus (Component) |
| :--- | :--- | :--- |
| **Equations of Motion** | Provides `ReferenceFrame`, `Quaternion`, `GravityModel` classes. | **OWNS** the 6DOF EOM Component. Integrates $\dot{X} = f(X)$. |
| **Aerodynamics** | Provides `calc_dynamic_pressure()`, `calc_mach()`. | **OWNS** the `AeroMap` component. Lookups $C_L$ tables, applies dynamics. |
| **Propulsion** | Provides `isentropic_flow_relations()`. | **OWNS** the `JetEngine` component. Models spool-up lag, fuel consumption state. |

### Implementation Strategy
Icarus components **compose** Vulcan utilities.

#### Example: Jet Engine Component
A `JetEngine` is a **System** (Icarus). It has state (Spool Speed $\omega$).
It uses **Functions** from Vulcan (Atmosphere, thermodynamics).

```cpp
template <typename Scalar>
void JetEngine::Step(Scalar t, Scalar dt) {
    // 1. Get Inputs
    Scalar h = *input_alt_;
    Scalar throttle = *input_throttle_;
    
    // 2. Use Vulcan to get environmental context (Functional)
    auto env = vulcan::atmosphere::usa76(h);
    
    // 3. Update Internal State (System Logic)
    // Vulcan doesn't know about "spool up time", Icarus does.
    Scalar target_thrust = throttle * max_thrust_ * (env.density / rho0);
    Scalar thrust_rate = (target_thrust - current_thrust_) / time_constant_;
    
    state_thrust_ += thrust_rate * dt; // Integration happens here
    
    // 4. Publish
    *output_thrust_ = state_thrust_;
}
```

## 10. Memory Layout (Optimization)

To satisfy `solve_ivp` and optimization solvers, the **Continuous State** must be contiguous.

*   **The Global State Vector:** `JanusVector<Scalar> X_global;`
*   **Scatter/Gather:**
    *   Before `Step()`: The Simulator takes `X_global` and "Scatters" chunks of it to `Component->state_`.
    *   After `Step()`: The Simulator "Gathers" `Component->state_dot_` back into `dX_global`.

This allows us to wrap the entire 500-component simulation into a single function compatible with `cvodes` or `IDAS`:
`dX = f(t, X)`

## 11. Summary of Benefits

| Feature | Flat Architecture Benefit |
| :--- | :--- |
| **Refactoring** | Moving a component from "Avionics" to "Payload" is just a string rename. No pointer surgery. |
| **Testing** | Unit test any component in isolation by mocking the Signal Backplane. |
| **Parallelism** | Easy to identify independent sub-graphs in the flat list for thread-pool execution. |
| **Optimization** | Trivially exposes the global `f(x,u)` function needed for trajectory optimization. |

## 12. Force/Moment Aggregation Pattern

In a 6DOF simulation, multiple components generate forces at different locations in different frames. These must be aggregated before the Equations of Motion (EOM) can integrate.

### The Problem
- **Aero** computes in wind frame
- **Propulsion** computes in nozzle-local frame
- **Gravity** is in inertial frame
- **Control Surfaces** produce forces at hinge points

All must become body-frame forces/moments about the CG.

### The Solution: Native Frame + Aggregator Transforms

**Each force-producing component publishes in its NATIVE frame:**

```yaml
# Aero publishes in WIND frame
Aero.force: [L, D, Y]           # Lift, Drag, Sideforce
Aero.moment: [Mx, My, Mz]       # About aero ref point
Aero.frame: WIND                # Declare the frame
Aero.position_body: [1.2, 0, 0] # Application point in body frame

# Propulsion publishes in LOCAL (nozzle) frame
Propulsion.force: [T, 0, 0]     # Thrust along nozzle axis
Propulsion.frame: LOCAL
Propulsion.dcm_to_body: [...]   # Rotation matrix from local to body
Propulsion.position_body: [-3.0, 0, 0]

# Gravity publishes in BODY frame (already transformed by gravity component)
Gravity.force: [Fx, Fy, Fz]
Gravity.frame: BODY
```

**The ForceAggregator centralizes ALL frame transforms:**

```cpp
template <typename Scalar>
class ForceAggregator : public Component {
    void Step(Scalar t, Scalar dt) {
        Vec3<Scalar> F_total = {0, 0, 0};
        Vec3<Scalar> M_total = {0, 0, 0};
        
        for (auto& src : force_sources_) {
            // Transform force to body frame
            Vec3<Scalar> F_body;
            if (src.frame == BODY) {
                F_body = *src.force;
            } else if (src.frame == WIND) {
                F_body = *wind_to_body_dcm_ * (*src.force);
            } else if (src.frame == LOCAL) {
                F_body = (*src.dcm_to_body) * (*src.force);
            }
            
            // Compute moment about CG
            Vec3<Scalar> r = *src.position_body - *cg_position_;
            Vec3<Scalar> M_body = vulcan::dcm_transform(src.frame, *src.moment) 
                                + vulcan::cross(r, F_body);
            
            F_total += F_body;
            M_total += M_body;
        }
        
        *output_total_force_ = F_total;
        *output_total_moment_ = M_total;
    }
};
```

> [!TIP]
> **Benefits of Centralized Transforms:**
> - **DRY:** Transform logic in ONE place (uses `vulcan::coordinates`)
> - **Debuggable:** Inspect each component's native-frame force before transform
> - **Symbolic Clean:** Single transform chain, easy to trace in CasADi

### Execution Order (Scheduler)

```
1. [Gravity]         → publishes force (BODY frame)
2. [Atmosphere]      → publishes density, pressure
3. [Aero]            → reads atmosphere, publishes force/moment (WIND frame)
4. [Propulsion]      → publishes force (LOCAL frame)
5. [ForceAggregator] → transforms ALL to BODY, sums, publishes totals
6. [EOM]             → reads totals, integrates state
```

### Mass Properties as Signals

> [!IMPORTANT]
> **Design Decision:** Mass is NOT a base `Component` property. Mass properties are **Signals**.
>
> Components that contribute mass simply publish:
> - `FuelTank.mass` (dynamic—changes as fuel burns)
> - `FuelTank.cg_local` (position relative to entity origin)
> - `FuelTank.inertia` (optional, if significant)
>
> Components without mass (Controller, Sensor) publish nothing about mass.

The **MassAggregator** subscribes to all mass signals:

```cpp
class MassAggregator : public Component {
    // Discovers mass sources via config or pattern matching (*.mass)
    // Publishes: Total.mass, Total.cg, Total.inertia_tensor
};
```

The **EOM** component subscribes to both aggregators:
- `Total.Force`, `Total.Moment` (from ForceAggregator)
- `Total.Mass`, `Total.Inertia` (from MassAggregator)

> [!NOTE]
> **Design Decision:** Aggregation is explicit via dedicated Components, not hidden inside EOM. This keeps all data flows visible on the Backplane and traceable in symbolic mode.

## 13. Configuration Architecture

All runtime behavior is driven by **Configuration Files** (YAML/JSON). The simulation code is generic; the config makes it specific.

### 13.1 Configuration Layers

| Layer | Scope | Example File | Contents |
| :--- | :--- | :--- | :--- |
| **A. Component Params** | Per-component type | `components/jet_engine.yaml` | Defaults, constants, flags, table paths |
| **A'. Entity Definition** | Per-entity type | `entities/x15_vehicle.yaml` | Component bundles, internal wiring, exposed ports |
| **B. World Setup** | Simulation-wide | `scenarios/x15_mission.yaml` | Entity instances, initial conditions |
| **C. Backplane Wiring** | Data routing | (embedded in B or separate) | Inter-entity port mappings |
| **D. Scheduler** | Execution order | (embedded in B or separate) | Rate groups, ordering policy |
| **E. Trim/Optimization** | Stage behavior | `trim/steady_cruise.yaml` | Free vars, targets, solver settings |
| **F. Services** | Logging, I/O | `services.yaml` | Recording, telemetry, bindings |

---

### 13.2 Layer A: Component Parameters

Each component type can have a **parameter schema**. Instances override defaults.

```yaml
# components/jet_engine.yaml (Component Type Definition)
type: JetEngine
defaults:
  max_thrust: 50000.0      # N
  time_constant: 0.5       # s
  use_afterburner: 0       # 0 - false, 1 - true
tables:
  fuel_flow_map: "data/engine/fuel_flow.csv"
```

---

### 13.3 Layer A': Entity Definition (Hierarchical Abstraction)

While the runtime is **flat**, humans configure in **hierarchies**. An Entity Definition bundles components into a reusable unit.

```yaml
# entities/x15_vehicle.yaml
entity:
  name: X15
  description: "North American X-15 Research Aircraft"
  
  # What components make up this entity?
  components:
    - type: RigidBody6DOF
      name: EOM
    - type: JetEngine
      name: MainEngine
      params:
        max_thrust: 75000.0
    - type: AeroBody
      name: Aero
      params:
        reference_area: 18.6  # m^2
        
  # Internal wiring (within this entity)
  internal_wiring:
    - source: Aero.output_lift
      target: EOM.input_force_z
    - source: MainEngine.output_thrust
      target: EOM.input_force_x
      
  # Exposed ports (for external wiring)
  ports:
    inputs:
      - name: atm_density
        binds_to: Aero.input_density
      - name: gravity
        binds_to: EOM.input_gravity
    outputs:
      - name: position
        binds_to: EOM.output_position
      - name: velocity
        binds_to: EOM.output_velocity
```

**At Load Time:** The entity is "flattened" into the global backplane:
- `X15.EOM.output_position` → `Vehicle.X15.EOM.Position`
- Internal wiring is resolved to direct signal bindings.
- External ports become the entity's public interface.

**Benefits:**
- Develop/test entities in isolation.
- Reuse entity definitions across scenarios.
- Hide internal complexity from scenario authors.

> [!IMPORTANT]
> **Design Decision: Wiring Philosophy**
> - **Internal wiring** lives in the Entity Definition (encapsulated).
> - **External wiring** (inter-entity) lives in the Scenario/World Setup.
> - **No inline transforms.** If you need unit conversion or signal manipulation, add an explicit Converter Component. This keeps the Backplane pure and all logic traceable.

---

### 13.4 Layer B: World Setup (Scenario Definition)

Defines WHAT exists in the simulation.

```yaml
# scenarios/x15_mission.yaml
scenario:
  name: "X-15 High Altitude Test"
  
entities:
  - name: X15
    components:
      - type: RigidBody6DOF
        name: EOM
      - type: JetEngine
        name: MainEngine
      - type: AeroBody
        name: Aero
        
  - name: Environment
    components:
      - type: USA76Atmosphere
        name: Atmosphere
      - type: WGS84Gravity
        name: Gravity

initial_conditions:
  X15.EOM.position: [0, 0, 15000]  # m
  X15.EOM.velocity: [200, 0, 0]   # m/s
```

---

### 13.4 Layer C: Backplane Wiring

Defines HOW data flows. Can be explicit or use conventions.

```yaml
# Explicit wiring (verbose but clear)
wiring:
  - source: Environment.Atmosphere.density
    target: X15.Aero.input_density
  - source: X15.Aero.output_lift
    target: X15.EOM.input_force_z
```

```yaml
# Convention-based (auto-wire by namespace matching)
wiring_policy: AUTO
# Components expecting "Env.Atm.*" auto-bind to Environment.Atmosphere.*
```

**User Choice:** You can expose only vehicle-level wiring (hide world internals).

---

### 13.5 Layer D: Scheduler Configuration

Defines WHEN and in what ORDER components execute.

```yaml
scheduler:
  policy: TOPOLOGICAL  # or MANUAL, RATE_GROUPED
  
  rate_groups:
    - rate_hz: 1000
      components: [X15.EOM, X15.Aero]
    - rate_hz: 100
      components: [X15.GNC, X15.Autopilot]
    - rate_hz: 10
      components: [Telemetry, Logger]
      
  # Manual ordering within a rate group (optional)
  execution_order:
    - Environment.Atmosphere
    - Environment.Gravity
    - X15.Aero
    - X15.EOM
```

---

### 13.6 Layer E: Trim / Optimization Problem Definition

Defines the **Stage** behavior for equilibrium solving.

```yaml
# trim/steady_cruise.yaml
trim:
  mode: EQUILIBRIUM
  solver: IPOPT          # or NEWTON_RAPHSON
  
  # What to hold fixed
  constraints:
    X15.EOM.position.z: 15000.0   # Hold altitude
    X15.EOM.velocity.x: 200.0     # Hold airspeed
    
  # What to solve for
  free_variables:
    - X15.Aero.alpha           # Angle of attack
    - X15.MainEngine.throttle  # Throttle setting
    - X15.Aero.elevator        # Trim elevator
    
  # What must be zero at equilibrium
  targets:
    X15.EOM.accel.x: 0.0
    X15.EOM.accel.z: 0.0
    X15.EOM.angular_accel.y: 0.0  # Pitch moment = 0
```

---

### 13.7 Layer F: Services (Misc Simulation Infrastructure)

```yaml
services:
  # Data Recording
  recording:
    enabled: true
    format: HDF5
    path: "output/x15_run_{timestamp}.h5"
    signals: ["X15.*", "Environment.*"]  # Glob patterns
    rate_hz: 100
    
  # Live Telemetry (to Hermes/Daedalus)
  telemetry:
    enabled: true
    protocol: UDP
    port: 5000
    rate_hz: 60
    signals: ["X15.EOM.*", "X15.Aero.mach"]
    
  # Logging
  logging:
    level: INFO
    file: "logs/sim.log"
    
  # Custom Signals (computed at runtime, not owned by any component)
  custom_signals:
    - name: "Derived.TotalEnergy"
      expression: "0.5 * X15.EOM.mass * X15.EOM.velocity^2 + X15.EOM.mass * 9.81 * X15.EOM.position.z"
      
  # External Bindings
  bindings:
    python:
      enabled: true
      module: "pybind11"
    matlab:
      enabled: false
    zmq:
      enabled: true
      port: 5555
```

---

### 13.8 Configuration Loading Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    Scenario Config                          │
│                (scenarios/x15_mission.yaml)                 │
│                      (Layer B)                              │
└─────────────────────────┬───────────────────────────────────┘
                          │ references
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ Entity Defs     │ │ Trim Config     │ │ Services Config │
│ (Layer A')      │ │ (Layer E)       │ │ (Layer F)       │
│                 │ │                 │ │                 │
│ x15_vehicle.yaml│ │ steady_cruise.  │ │ services.yaml   │
└────────┬────────┘ └─────────────────┘ └─────────────────┘
         │ references
         ▼
┌─────────────────┐
│ Component Defs  │
│ (Layer A)       │
│                 │
│ rocket.yaml     │
│ aero_body.yaml  │
└─────────────────┘
         │
         │ All configs loaded
         ▼
┌─────────────────────────────────────────────────────────────┐
│                     Simulator.Provision()                   │
│  • Instantiate components from Entity Defs                  │
│  • Load tables/assets                                       │
│  • Register all signals on Backplane                        │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                      Simulator.Stage()                      │
│  • Resolve Wiring (Layer C) - internal + external           │
│  • Configure Scheduler (Layer D)                            │
│  • Run Trim/Optimization (Layer E)                          │
│  • Bind signal pointers                                     │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    Simulator.Step() loop                    │
│  • Execute components per Scheduler                         │
│  • Services (Layer F) active: logging, telemetry, recording │
└─────────────────────────────────────────────────────────────┘
```

## 14. External Bindings & Language Interface

Icarus must be usable by diverse consumers:
- **Analysis Engineers:** Python (Jupyter, pandas, matplotlib)
- **GNC Engineers:** MATLAB / Simulink
- **Firmware Engineers:** C / Embedded C
- **Orchestration:** C++ applications, Hermes middleware

### The C API Strategy

> [!IMPORTANT]
> **Design Decision:** The top-level Simulator interface is exposed via a **C API** for maximum compatibility.
>
> C is the universal FFI (Foreign Function Interface). Every language can call C.

### API Surface

```c
// icarus.h - The universal C interface

// Lifecycle
IcarusHandle* icarus_create(const char* config_path);
void icarus_destroy(IcarusHandle* sim);

int icarus_provision(IcarusHandle* sim);
int icarus_stage(IcarusHandle* sim, const char* run_config_path);
int icarus_step(IcarusHandle* sim, double dt);

// State Access
double icarus_get_signal(IcarusHandle* sim, const char* signal_name);
int icarus_set_signal(IcarusHandle* sim, const char* signal_name, double value);

// Bulk Access (for performance)
int icarus_get_state_vector(IcarusHandle* sim, double* out_buffer, int* out_size);
int icarus_set_state_vector(IcarusHandle* sim, const double* buffer, int size);

// Introspection
const char* icarus_get_schema_json(IcarusHandle* sim);
int icarus_get_signal_count(IcarusHandle* sim);

// Error Handling
const char* icarus_get_last_error(IcarusHandle* sim);
```

### Language Bindings

| Language | Binding Method | Notes |
|----------|---------------|-------|
| **Python** | `ctypes` or `cffi` wrapping C API, or PyBind11 for richer interface | Primary for analysis |
| **MATLAB** | `loadlibrary` / MEX calling C API | S-Function wrapper for Simulink |
| **C++** | Direct include of C++ headers, or C API | Hermes uses native C++ |
| **Julia** | `ccall` to C API | Growing in aerospace |
| **Rust** | `bindgen` for C API | Safety-critical applications |

### Simulink Integration Pattern

For MATLAB/Simulink users, wrap the C API in an S-Function:

```matlab
% In Simulink S-Function
function [sys] = mdlOutputs(t, x, u, sim_handle)
    % Set inputs
    icarus_set_signal(sim_handle, 'Control.Throttle', u(1));
    icarus_set_signal(sim_handle, 'Control.Elevator', u(2));
    
    % Step
    icarus_step(sim_handle, dt);
    
    % Get outputs
    sys(1) = icarus_get_signal(sim_handle, 'Nav.Altitude');
    sys(2) = icarus_get_signal(sim_handle, 'Nav.Velocity');
end
```

### Python Integration (PyBind11)

PyBind11 provides a richer, more Pythonic interface than raw ctypes:

```cpp
// icarus_python.cpp - PyBind11 bindings
#include <pybind11/pybind11.h>
#include "icarus/Simulator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(icarus, m) {
    py::class_<Simulator>(m, "Simulator")
        .def(py::init<const std::string&>())
        .def("provision", &Simulator::Provision)
        .def("stage", &Simulator::Stage)
        .def("step", &Simulator::Step)
        .def("get_signal", &Simulator::GetSignal)
        .def("set_signal", &Simulator::SetSignal)
        .def("get_schema", &Simulator::GetSchemaJson)
        .def_property_readonly("time", &Simulator::GetTime);
}
```

```python
# Python usage
import icarus

sim = icarus.Simulator("scenarios/x15_mission.yaml")
sim.provision()
sim.stage("trim/steady_cruise.yaml")

for _ in range(10000):
    sim.step(0.001)
    alt = sim.get_signal("Nav.Altitude")
    print(f"t={sim.time:.3f}s, Altitude: {alt:.1f}m")
```

### Performance Considerations

| Interface | Overhead | Use Case |
|-----------|----------|----------|
| **C API (per-signal)** | ~100ns per call | Interactive, low-frequency |
| **C API (bulk vector)** | ~1μs for full state | Monte Carlo, batch |
| **Native C++** | Zero | Hermes, performance-critical |

> [!NOTE]
> For high-performance Monte Carlo (Hydra), use native C++ instantiation, not the C API. The C API is for external consumers, not internal batch execution.

## 15. Event Handling & Flight Phases

Vehicles have discrete events (stage separation, parachute deploy) and flight phases (boost, coast, descent).

### Phase Concept

Each **Entity** can have a `phase` signal (int32):

```yaml
# Phase definition for a rocket
X15.phase: 0  # int32 signal
# Phases: 0=GROUND, 1=BOOST, 2=COAST, 3=REENTRY, 4=LANDED
```

### Phase-Dependent Components ("Ghosting")

To respect Janus symbolic graph creation, we don't dynamically add/remove components. Instead, components are **always present** but their outputs are gated:

```cpp
template <typename Scalar>
void Booster::Step(Scalar t, Scalar dt) {
    // Component always executes, but output is gated by phase
    Scalar thrust_raw = compute_thrust(...);
    
    // Gate output: only active during BOOST phase
    Scalar is_boost = janus::where(*phase_ == 1, Scalar(1.0), Scalar(0.0));
    *output_thrust_ = thrust_raw * is_boost;
}
```

> [!IMPORTANT]
> **No dynamic component creation/destruction.** All components exist for the full simulation. Use `janus::where()` to gate outputs based on phase. This preserves the symbolic graph structure.

### Phase Transitions

Phase transitions are triggered by **condition signals**:

```yaml
# Phase transition config
transitions:
  - from: BOOST
    to: COAST
    condition: "Propulsion.fuel_mass < 0.01"
  - from: COAST
    to: REENTRY
    condition: "Nav.altitude < 100000 AND Nav.velocity_down > 0"
```

A dedicated `PhaseManager` component evaluates transitions each step.

---

## 16. Error Handling & Logging

### Severity Levels

| Level | Use Case | Behavior |
|-------|----------|----------|
| **ERROR** | Unrecoverable (NaN, solver failure) | Stop simulation, propagate exception |
| **WARNING** | Recoverable issue (constraint violation, clamp) | Log and continue |
| **INFO** | Informational (phase change, event) | Log only |
| **DEBUG** | Development debugging | Log if debug mode enabled |
| **TRACE** | High-frequency (per-step values) | Log if trace mode enabled |

### Logging API

```cpp
// Within a component
ICARUS_ERROR("Solver diverged at t={}", t);      // Throws, stops sim
ICARUS_WARN("Clipping thrust to max: {}", max);  // Logs, continues
ICARUS_INFO("Phase transition: {} -> {}", old, new_phase);
ICARUS_DEBUG("State vector: {}", state);

// Global configuration
icarus::Logger::set_level(icarus::LogLevel::INFO);
icarus::Logger::set_output("logs/sim.log");
```

### Validation Errors

Signal binding failures, configuration errors, and type mismatches throw during **Provision** or **Stage**—never during **Step**.

---

## 17. Determinism Guarantees

> [!IMPORTANT]
> **Numeric mode is bit-identical across runs** with the same inputs.

### How Determinism is Ensured

| Mechanism | Guarantee |
|-----------|-----------|
| **Explicit integrators** | RK4, Euler—no adaptive stepping randomness |
| **No dynamic allocation** | All memory allocated in Provision |
| **Strict signal ordering** | Scheduler order is deterministic |
| **No floating-point non-determinism** | Avoid `std::rand`, use seeded RNG from `vulcan::rng` |

### What Breaks Determinism

- Adaptive integrators (CVODES) with loose tolerances
- Multithreading without careful synchronization
- System time dependencies
- Uninitialized memory

---

## 18. Multi-threading & Parallelism

### Primary Strategy: Single-Threaded HITL Compatibility

> [!NOTE]
> **Parallelism is deprioritized.** The primary use case (HITL) requires deterministic single-threaded execution. For symbolic optimization solves, parallelism is baked into the optimizer.

### Hydra for HPC

For large-scale Monte Carlo (10k+ runs), use **Hydra** (external orchestrator):
- SLURM job submission
- HPC cluster management
- Results aggregation

Icarus remains single-threaded per instance; Hydra handles parallelism.

---

## 19. Recording & Playback

### Recording Format

The `.icarec` format captures all simulation data:

```
icarec file structure:
├── schema.json          # Signal map, types, units
├── static_signals.bin   # Static signals (one-time)
├── dynamic_signals.bin  # Dynamic signals (per-frame)
└── metadata.json        # Scenario, timestamps, version
```

### Telemetry → .icarec Pipeline

Live telemetry (UDP) and recording share the same data path. Recording is enabled via Services config:

```yaml
services:
  recording:
    enabled: true
    format: ICAREC  # or HDF5, CSV
    path: "output/run_{timestamp}.icarec"
```

### Warmstart from Recording

To warmstart from any point in a recording:

```cpp
sim.provision();
sim.warmstart_from("recording.icarec", t_warmstart);
// State vector is restored to t_warmstart
sim.step(dt);  // Continue from that point
```

> [!WARNING]
> **Warmstart requires all input signals to be recorded.** If a component reads an external input that wasn't recorded, warmstart will fail. Consider recording at the component boundary, not just outputs.

---

## 20. Symbolic Mode Constraints (Janus Compatibility)

> **Reference:** See `janus/docs/janus_usage_guide.md` for complete details.

### MANDATORY Rules for Icarus Components

| Rule | Correct ✅ | Wrong ❌ |
|------|-----------|---------|
| **Template on Scalar** | `template <typename Scalar>` | Hardcoded `double` |
| **Use janus:: math** | `janus::sin(x)`, `janus::pow(x,2)` | `std::sin(x)`, `std::pow(x,2)` |
| **Branching** | `janus::where(cond, a, b)` | `if (cond) { a } else { b }` |
| **Loops** | `for (int i=0; i<N; ++i)` (fixed N) | `while (error > tol)` (dynamic) |
| **Types** | `janus::Vec3<Scalar>` | `Eigen::Vector3d` |

### Multi-way Branching

```cpp
// Use janus::select for multiple conditions
Scalar cd = janus::select(
    {mach < 0.3, mach < 0.8, mach < 1.2},
    {Scalar(0.02), Scalar(0.025), Scalar(0.05)},
    Scalar(0.03));  // default
```

### What Breaks Symbolic Mode

- `if/else` on `Scalar` (MX can't evaluate to bool)
- `while` loops with dynamic bounds
- `std::` math functions (bypass CasADi tracing)
- Dynamic memory allocation inside Step()
- Non-templated functions

---

## 21. Testing & Debugging

### Component Unit Testing

Test components in isolation using a **MockBackplane**:

```cpp
TEST(AeroComponent, ComputesLiftCorrectly) {
    MockBackplane bp;
    bp.set("Env.Atm.Density", 1.225);
    bp.set("Nav.Velocity", 100.0);
    bp.set("Nav.Alpha", 0.1);
    
    AeroComponent<double> aero;
    aero.provision(config);
    aero.stage(bp);
    aero.step(0.0, 0.001);
    
    EXPECT_NEAR(bp.get("Aero.Lift"), expected_lift, 1e-6);
}
```

### Symbolic Mode Testing

Every component should be tested in BOTH modes:

```cpp
TEST(AeroComponent, SymbolicModeTraces) {
    MockBackplane<casadi::MX> bp;
    // ... setup symbolic signals ...
    
    AeroComponent<casadi::MX> aero;
    aero.provision(config);
    aero.stage(bp);
    aero.step(janus::sym("t"), janus::sym("dt"));
    
    // Verify output is a valid MX expression
    auto lift = bp.get("Aero.Lift");
    EXPECT_TRUE(lift.is_valid_input());
}
```

### Mini-Sim Pattern (Python/PyTest)

For integration testing:

```python
import icarus
import pytest

def test_x15_reaches_altitude():
    sim = icarus.Simulator("scenarios/x15_test.yaml")
    sim.provision()
    sim.stage("trim/ground_start.yaml")
    
    while sim.time < 60.0:
        sim.step(0.001)
    
    assert sim.get_signal("Nav.Altitude") > 10000.0
```

### Debugging / Introspection

> [!NOTE]
> **Future Work:** Full debugging UI is planned for Daedalus. Current introspection is via:
> - `icarus_get_schema_json()` for signal listing
> - Recording + playback for post-mortem analysis
> - Component-level logging at DEBUG level
