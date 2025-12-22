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

## 3. Component Protocol

Components are the atomic units of execution. They follow a **"Register Outputs, Wire Inputs"** pattern:

- **Outputs** are registered at Provision (intrinsic to the component)
- **Inputs** are wired at Stage (configurable per scenario)
- A **Data Dictionary** is auto-generated for discoverability

### 3.1 Register Outputs at Provision

Components declare what signals they produce. This is intrinsic—a JetEngine *is* a thing that produces thrust.

```cpp
template <typename Scalar>
class JetEngine : public Component<Scalar> {
    // Internal storage for outputs
    Scalar thrust_;
    Scalar fuel_flow_;
    Scalar spool_speed_;

    // Input handles (wired at Stage)
    SignalHandle<Scalar> input_altitude_;
    SignalHandle<Scalar> input_throttle_;

public:
    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Register OUTPUTS with metadata
        // Final name: {entity}.{component}.{signal} e.g., "X15.MainEngine.thrust"

        bp.register_output("thrust", &thrust_, {
            .units = "N",
            .description = "Net thrust force",
            .lifecycle = Lifecycle::Dynamic
        });

        bp.register_output("fuel_flow", &fuel_flow_, {
            .units = "kg/s",
            .description = "Fuel mass flow rate",
            .lifecycle = Lifecycle::Dynamic
        });

        bp.register_output("spool_speed", &spool_speed_, {
            .units = "rad/s",
            .description = "Turbine spool angular velocity",
            .lifecycle = Lifecycle::Dynamic,
            .integrable = true  // This is state—will be integrated
        });

        // Load static parameters
        max_thrust_ = cfg.get<double>("max_thrust", 50000.0);
        time_constant_ = cfg.get<double>("time_constant", 0.5);
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Wire INPUTS from config—these can point anywhere
        input_altitude_ = bp.resolve<Scalar>(cfg.get("input_altitude"));
        input_throttle_ = bp.resolve<Scalar>(cfg.get("input_throttle"));
    }

    void Step(Scalar t, Scalar dt) override {
        Scalar alt = *input_altitude_;
        Scalar throttle = *input_throttle_;

        // ... physics calculations ...

        thrust_ = calculated_thrust;  // Direct write to registered storage
        fuel_flow_ = calculated_fuel_flow;
    }
};
```

### 3.2 Wire Inputs at Stage

Config specifies where inputs come from—this is a wiring decision, not component identity.

```yaml
components:
  - type: JetEngine
    name: MainEngine
    entity: X15
    config:
      # INPUT WIRING: where to read from (fully flexible)
      input_altitude: "Environment.Atmosphere.Altitude"
      input_throttle: "Control.Throttle"

      # PARAMETERS: component configuration
      max_thrust: 75000.0
      time_constant: 0.5

      # Note: outputs are NOT specified here—they're intrinsic to JetEngine
      # Output signals will be: X15.MainEngine.thrust, X15.MainEngine.fuel_flow, etc.
```

### 3.3 Why This Split?

| Aspect | Outputs (Provision) | Inputs (Stage) |
| :--- | :--- | :--- |
| **Defined by** | Component code | Config file |
| **Rationale** | Identity—what the component *is* | Topology—how it's connected |
| **Flexibility** | Fixed per component type | Fully reconfigurable |
| **Example** | JetEngine always produces thrust | Altitude could come from sensor, model, or test harness |

> [!NOTE]
> **Outputs are identity. Inputs are topology.**
> A JetEngine without thrust isn't a JetEngine. But a JetEngine reading altitude from a barometer vs. GPS vs. truth model—that's just wiring.

### 3.4 Auto-Generated Data Dictionary

After Provision, the Simulator dumps all registered signals to a data dictionary file:

```cpp
void Simulator::Provision(const ScenarioConfig& cfg) {
    for (auto* comp : components_) {
        comp->Provision(backplane_, comp->config_);
    }

    // Generate data dictionary for tooling and documentation
    backplane_.GenerateDataDictionary("output/data_dictionary.yaml");
}
```

**Output: `data_dictionary.yaml`**

```yaml
# AUTO-GENERATED by Icarus Simulator
# Scenario: scenarios/x15_mission.yaml
# Generated: 2024-12-22T10:30:00Z
#
# This file documents all signals in the simulation.
# Use for: telemetry setup, recording config, debugging, documentation.

components:
  X15.MainEngine:
    type: JetEngine
    signals:
      thrust:
        type: double
        units: N
        lifecycle: dynamic
        description: "Net thrust force"

      fuel_flow:
        type: double
        units: kg/s
        lifecycle: dynamic
        description: "Fuel mass flow rate"

      spool_speed:
        type: double
        units: rad/s
        lifecycle: dynamic
        integrable: true
        description: "Turbine spool angular velocity"

  X15.Aero:
    type: AeroBody
    signals:
      lift:
        type: double
        units: N
        lifecycle: dynamic
        description: "Aerodynamic lift force"

      drag:
        type: double
        units: N
        lifecycle: dynamic
        description: "Aerodynamic drag force"

      alpha:
        type: double
        units: rad
        lifecycle: dynamic
        description: "Angle of attack"

  Environment.Atmosphere:
    type: StandardAtmosphere
    signals:
      Altitude:
        type: double
        units: m
        lifecycle: dynamic
        description: "Geometric altitude"

      Density:
        type: double
        units: kg/m³
        lifecycle: dynamic
        description: "Atmospheric density"

      Temperature:
        type: double
        units: K
        lifecycle: dynamic
        description: "Static air temperature"

# Summary
total_signals: 42
dynamic_signals: 38
static_signals: 4
integrable_states: 13
```

### 3.5 Data Dictionary Uses

| Use Case | How |
| :--- | :--- |
| **Telemetry Setup** | Select signals to stream by browsing dictionary |
| **Recording Config** | Reference signal names for `signals: ["X15.MainEngine.*"]` |
| **Debugging** | Know exactly what signals exist and their units |
| **Documentation** | Auto-generated, always up-to-date signal reference |
| **Tooling** | Daedalus UI reads dictionary for signal browser |
| **Validation** | Check config references against known signals |

### 3.6 Dependency Discovery

The Scheduler discovers dependencies by tracking Provision (outputs) and Stage (inputs):

```cpp
void Simulator::Stage(const RunConfig& rc) {
    DependencyGraph graph;

    // Outputs already registered during Provision
    for (auto* comp : components_) {
        for (const auto& out : comp->GetRegisteredOutputs()) {
            graph.AddProvider(out.full_name, comp);
        }
    }

    // Track input resolution during Stage
    for (auto* comp : components_) {
        backplane_.BeginTracking();
        comp->Stage(backplane_, comp->config_);

        for (const auto& in : backplane_.GetTrackedInputs()) {
            graph.AddDependency(in.signal_name, comp);
        }
        backplane_.EndTracking();
    }

    // Topological sort for execution order
    execution_order_ = graph.TopologicalSort();  // Throws on cycle
}
```

### 3.7 Validation

| Validation | When | Error Example |
| :--- | :--- | :--- |
| Signal exists | `resolve()` at Stage | `"Env.Atm.Altidude" not found. Did you mean "X15.Atm.Altitude"?` |
| Type matches | `resolve<T>()` at Stage | `"gnc.mode" is int32, but requested double` |
| No duplicate outputs | `register_output()` at Provision | `"thrust" already registered by LeftEngine` |
| No dependency cycles | End of Stage | `Cycle detected: A → B → C → A` |
| Config key exists | `cfg.get()` | `"input_altitude" not specified for X15.MainEngine` |

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

### 4.5 Type-Safe Signal Access

The raw `void* data_ptr` in `SignalMetadata` is an implementation detail. User-facing APIs enforce type safety:

#### Typed Signal Handles

```cpp
// Type-safe handle returned by resolve()
template <typename T>
class SignalHandle {
    T* ptr_;
    SignalMetadata meta_;  // For debugging/introspection
public:
    T& operator*() { return *ptr_; }
    const T& operator*() const { return *ptr_; }
    T* operator->() { return ptr_; }

    // Introspection
    const std::string& name() const { return meta_.name; }
    Units units() const { return meta_.units; }
};

// Backplane resolve with compile-time type checking
template <typename Scalar>
class Backplane {
public:
    template <typename T>
    SignalHandle<T> resolve(const std::string& name) {
        auto& meta = registry_.at(name);

        // Runtime type check (throws if mismatch)
        if (meta.type != TypeTraits<T>::type_id) {
            throw TypeMismatchError(name, meta.type, TypeTraits<T>::type_id);
        }

        return SignalHandle<T>{static_cast<T*>(meta.data_ptr), meta};
    }
};
```

#### Registration with Type Deduction

```cpp
// Component registration - type is deduced
template <typename Scalar>
void JetEngine<Scalar>::Provision(Backplane<Scalar>& bp) {
    // Type is captured at registration
    bp.register_output("Propulsion.Thrust", &thrust_value_, Units::Newtons);
    bp.register_output("Propulsion.FuelFlow", &fuel_flow_, Units::KgPerSec);

    // Static signals
    bp.register_static("Propulsion.MaxThrust", &max_thrust_, Units::Newtons);
}
```

#### Type Validation Matrix

| Registration Type | Resolve Type | Result |
| :--- | :--- | :--- |
| `double` | `double` | ✅ Success |
| `double` | `int32` | ❌ `TypeMismatchError` |
| `int32` | `double` | ❌ `TypeMismatchError` |
| `Vec3<Scalar>` | `Vec3<Scalar>` | ✅ Success |
| `Vec3<double>` | `Vec3<MX>` | ❌ `TypeMismatchError` (Scalar mismatch) |

> [!IMPORTANT]
> **Type checking happens at Stage, not Step.** All `resolve()` calls occur during `Stage()`. If types mismatch, the simulation fails fast with a clear error—before entering the hot loop.

### 4.6 Signal Namespacing & Collision Prevention

Signal names follow a hierarchical namespace convention:

```
<Entity>.<Component>.<Signal>
```

Examples:
- `Falcon9.Stage1.Engine.Thrust`
- `Environment.Atmosphere.Density`
- `X15.Aero.Alpha`

#### Collision Detection

```cpp
void Backplane::register_output(const std::string& name, ...) {
    if (registry_.contains(name)) {
        throw DuplicateSignalError(
            name,
            registry_.at(name).owner_component,
            current_component_
        );
    }
    // ... register
}
```

#### Wildcard Queries (Introspection Only)

```cpp
// For tooling/debugging—NOT for hot-path access
std::vector<SignalMetadata> Backplane::query(const std::string& pattern) {
    // Supports glob patterns: "Falcon9.*.Thrust", "*.Aero.*"
    return match_glob(registry_, pattern);
}
```

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

### 5.1 Extended Lifecycle Hooks

Beyond the core Provision/Stage/Step, components may implement optional hooks for finer control:

```cpp
template <typename Scalar>
class Component {
public:
    // === CORE LIFECYCLE (Required) ===
    virtual void Provision(const ComponentConfig& config) = 0;
    virtual void Stage(Backplane<Scalar>& bp, const RunConfig& rc) = 0;
    virtual void Step(Scalar t, Scalar dt) = 0;

    // === EXTENDED HOOKS (Optional) ===

    // Called when entering a new flight phase
    virtual void OnPhaseEnter(int32_t new_phase) {}

    // Called when exiting a flight phase
    virtual void OnPhaseExit(int32_t old_phase) {}

    // Called before any component steps (for aggregators, pre-processing)
    virtual void PreStep(Scalar t, Scalar dt) {}

    // Called after all components step (for aggregators, post-processing)
    virtual void PostStep(Scalar t, Scalar dt) {}

    // Called when simulation encounters an error (graceful degradation)
    virtual void OnError(const SimulationError& error) {}

    // Called during shutdown (cleanup, flush buffers)
    virtual void Shutdown() {}

    // === INTROSPECTION ===

    // Return declared inputs/outputs for dependency graph
    virtual std::vector<SignalDecl> GetInputs() const { return {}; }
    virtual std::vector<SignalDecl> GetOutputs() const { return {}; }
    virtual size_t GetStateSize() const { return 0; }
};
```

#### Hook Execution Order

```
┌─────────────────────────────────────────────────────────────┐
│                    Simulator.Step(dt)                        │
└─────────────────────────────────────────────────────────────┘
                              │
    ┌─────────────────────────┼─────────────────────────────┐
    ▼                         ▼                             ▼
┌────────────┐         ┌────────────┐                ┌────────────┐
│ PreStep()  │ ──────► │  Step()    │ ─────────────► │ PostStep() │
│ Aggregators│         │ All comps  │                │ Aggregators│
│ Sensors    │         │ in order   │                │ Loggers    │
└────────────┘         └────────────┘                └────────────┘
```

#### Phase Transition Example

```cpp
class Booster : public Component<Scalar> {
    void OnPhaseEnter(int32_t phase) override {
        if (phase == Phase::BOOST) {
            ICARUS_INFO("Booster ignition");
            ignition_time_ = current_time_;
        }
    }

    void OnPhaseExit(int32_t phase) override {
        if (phase == Phase::BOOST) {
            ICARUS_INFO("Booster burnout, total impulse: {}", total_impulse_);
            // Cleanup: reset transient state for potential restart
        }
    }
};
```

#### Error Handling Hook

```cpp
class Autopilot : public Component<Scalar> {
    void OnError(const SimulationError& error) override {
        if (error.severity == Severity::WARNING) {
            // Graceful degradation: switch to backup mode
            *output_mode_ = AutopilotMode::SAFE_HOLD;
            ICARUS_WARN("Autopilot entering safe hold due to: {}", error.message);
        }
        // ERROR severity will propagate and stop simulation
    }
};
```

> [!NOTE]
> **Performance:** Extended hooks have negligible overhead. `OnPhaseEnter`/`OnPhaseExit` are called only on phase transitions. `PreStep`/`PostStep` are only invoked for components that override them (checked once at Stage).

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

### 6.1 Rate Group Synchronization

When components run at different rates, **inter-rate signal access** must be well-defined.

#### The Problem

A 100Hz GNC component reads `Nav.Position` from a 1kHz EOM. Which value does it see?
- The value from 10 steps ago?
- An interpolated value?
- The most recent value?

#### Synchronization Policies

| Policy | Description | Use Case |
| :--- | :--- | :--- |
| **ZOH (Zero-Order Hold)** | Read the last written value | Default. Simple, deterministic. |
| **Interpolated** | Linear interpolation between last two values | Smooth signals (position, velocity) |
| **Extrapolated** | Predict forward using derivatives | Control systems with latency compensation |
| **Synchronized** | All groups tick at LCM rate | Eliminates issue but wastes compute |

#### Configuration

```yaml
scheduler:
  rate_groups:
    - name: fast
      rate_hz: 1000
      components: [EOM, IMU, Actuators]
    - name: medium
      rate_hz: 100
      components: [GNC, Autopilot]
    - name: slow
      rate_hz: 10
      components: [Telemetry, Logger]

  # Inter-rate signal policies
  synchronization:
    default_policy: ZOH  # Zero-Order Hold

    # Override for specific signal patterns
    overrides:
      - pattern: "Nav.position*"
        policy: INTERPOLATED
      - pattern: "Nav.velocity*"
        policy: INTERPOLATED
      - pattern: "Control.*"
        policy: ZOH  # Commands should not be interpolated
```

#### Implementation

```cpp
template <typename Scalar>
class RateGroupScheduler {
    struct RateGroup {
        double rate_hz;
        double accumulated_time = 0.0;
        std::vector<Component<Scalar>*> components;
    };

public:
    void Step(Scalar t, Scalar dt) {
        for (auto& group : rate_groups_) {
            group.accumulated_time += dt;

            double group_dt = 1.0 / group.rate_hz;
            while (group.accumulated_time >= group_dt) {
                // Apply synchronization policy for inter-rate reads
                ApplySyncPolicies(group);

                for (auto* comp : group.components) {
                    comp->Step(t, Scalar(group_dt));
                }

                group.accumulated_time -= group_dt;
            }
        }
    }

private:
    void ApplySyncPolicies(RateGroup& group) {
        for (auto& [signal, policy] : sync_policies_) {
            if (policy == SyncPolicy::INTERPOLATED) {
                // Compute interpolated value from history buffer
                signal.current = Lerp(signal.prev, signal.latest, alpha);
            }
            // ZOH: no action needed—just read latest value
        }
    }
};
```

#### Determinism Guarantee

> [!IMPORTANT]
> **All synchronization policies are deterministic.** Given the same initial state and inputs, the simulation produces identical results regardless of wall-clock timing. Interpolation uses fixed coefficients based on rate ratios, not real timestamps.

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

## 10. Memory Layout & State Ownership

To satisfy `solve_ivp` and optimization solvers, the **Continuous State** must be contiguous.

### 10.1 The Global State Vector

```cpp
// The Simulator owns the authoritative state
template <typename Scalar>
class Simulator {
    JanusVector<Scalar> X_global_;      // Continuous state (integrated)
    JanusVector<Scalar> X_dot_global_;  // State derivatives

    // State layout metadata
    struct StateSlice {
        Component<Scalar>* owner;
        size_t offset;
        size_t size;
    };
    std::vector<StateSlice> state_layout_;
};
```

### 10.2 State Ownership Model

> [!IMPORTANT]
> **The Simulator owns all state.** Components hold *views* (pointers) into the global vector, not copies.

| Concept | Owner | Lifetime |
| :--- | :--- | :--- |
| **Global State Vector** `X_global_` | Simulator | Provision → Destroy |
| **Component State View** `state_` | Component (pointer only) | Stage → Reset |
| **State Derivatives** `X_dot_global_` | Simulator | Provision → Destroy |

### 10.3 Scatter/Gather Protocol

```cpp
// During Stage: Components receive views into global state
void Simulator::Stage(const RunConfig& rc) {
    size_t offset = 0;
    for (auto* comp : components_) {
        size_t state_size = comp->GetStateSize();

        // Component receives a VIEW, not a copy
        comp->BindState(
            X_global_.data() + offset,      // state pointer
            X_dot_global_.data() + offset   // derivative pointer
        );

        state_layout_.push_back({comp, offset, state_size});
        offset += state_size;
    }
}

// Component implementation
template <typename Scalar>
class JetEngine : public Component<Scalar> {
    // These are VIEWS into Simulator's global vector
    Scalar* state_spool_speed_;      // Points into X_global_
    Scalar* state_dot_spool_speed_;  // Points into X_dot_global_

public:
    void BindState(Scalar* state, Scalar* state_dot) override {
        state_spool_speed_ = state;
        state_dot_spool_speed_ = state_dot;
    }

    void Step(Scalar t, Scalar dt) override {
        // Read current state (from global vector via pointer)
        Scalar omega = *state_spool_speed_;

        // Compute derivative
        Scalar omega_dot = (target_omega - omega) / tau;

        // Write derivative (to global vector via pointer)
        *state_dot_spool_speed_ = omega_dot;

        // NOTE: We do NOT update *state_spool_speed_ here.
        // The integrator does that.
    }
};
```

### 10.4 Integration Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    Integrator (RK4/CVODES)                      │
│  Owns: X_global_, X_dot_global_                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
           ┌──────────────────┼──────────────────┐
           ▼                  ▼                  ▼
    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
    │ Component A │    │ Component B │    │ Component C │
    │ state_ ───────────► X_global_[0:3]                │
    │ state_dot_ ─────────► X_dot_[0:3]                 │
    └─────────────┘    └─────────────┘    └─────────────┘

Step 1: Integrator calls Simulator.ComputeDerivatives(t, X)
Step 2: Simulator scatters X into component views (automatic via pointers)
Step 3: Each Component.Step() reads state_, writes state_dot_
Step 4: Simulator gathers X_dot (automatic via pointers)
Step 5: Integrator advances: X_new = X + dt * f(X_dot)
```

### 10.5 Why Components Don't Integrate

| Approach | Pros | Cons |
| :--- | :--- | :--- |
| **Components integrate themselves** | Simple, self-contained | Can't use adaptive solvers, no global error control |
| **Simulator integrates globally** ✓ | Full solver compatibility, global error control | Slightly more complex setup |

> [!NOTE]
> **Exception: Algebraic State.** Some "state" is not integrated (e.g., lookup table outputs, mode flags). These can be updated directly by components since they're not part of the ODE system.

### 10.6 Solver Compatibility

This layout exposes the simulation as a standard ODE function:

```cpp
// This function signature is compatible with CVODES, scipy, MATLAB ode45
JanusVector<Scalar> Simulator::ComputeDerivatives(Scalar t, const JanusVector<Scalar>& X) {
    // X is already scattered to components via pointers
    for (auto* comp : scheduled_components_) {
        comp->Step(t, dt_nominal_);
    }
    // X_dot_global_ is already gathered via pointers
    return X_dot_global_;
}
```

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

#### 13.6.1 Trim Problem Formulation

The trim problem is a **constrained optimization** or **root-finding** problem:

```
Given:    Fixed signals (constraints)
Find:     Free variables
Such that: Target signals = target values (typically 0)
```

Mathematically:
```
minimize  ||f(x) - targets||²
   x
subject to:
   x_lower ≤ x ≤ x_upper   (bounds)
   g(x) = constraints       (equality)
```

#### 13.6.2 Extended Trim Configuration

```yaml
trim:
  mode: EQUILIBRIUM  # or INITIAL_GUESS, OPTIMIZATION

  solver:
    type: IPOPT           # NEWTON_RAPHSON, KINSOL, SNOPT
    max_iterations: 100
    tolerance: 1e-8
    verbose: false

  # Fixed values (equality constraints)
  constraints:
    X15.EOM.position.z: 15000.0
    X15.EOM.velocity.x: 200.0
    X15.EOM.attitude.pitch: 0.0  # Level flight

  # Variables to solve for
  free_variables:
    - name: X15.Aero.alpha
      initial_guess: 0.05       # radians
      bounds: [-0.3, 0.3]       # Optional bounds
    - name: X15.MainEngine.throttle
      initial_guess: 0.5
      bounds: [0.0, 1.0]
    - name: X15.Aero.elevator
      initial_guess: 0.0
      bounds: [-0.5, 0.5]

  # What must equal target (typically zero for equilibrium)
  targets:
    X15.EOM.accel.x: 0.0        # No acceleration
    X15.EOM.accel.z: 0.0
    X15.EOM.angular_accel.y: 0.0

  # Advanced: Weighted residuals (for over-determined systems)
  weights:
    X15.EOM.accel.x: 1.0
    X15.EOM.accel.z: 1.0
    X15.EOM.angular_accel.y: 10.0  # Prioritize pitch trim

  # Fallback behavior
  on_failure:
    action: USE_INITIAL_GUESS  # or ERROR, WARN_AND_CONTINUE
    message: "Trim failed, using initial guess"
```

#### 13.6.3 Runtime Mechanics

The `Stage()` function orchestrates trim solving:

```cpp
template <typename Scalar>
void Simulator<Scalar>::Stage(const RunConfig& rc) {
    // 1. Apply fixed constraints to Backplane
    for (const auto& [signal, value] : rc.trim.constraints) {
        backplane_.set(signal, value);
    }

    // 2. Set initial guesses for free variables
    std::vector<double> x0;
    for (const auto& var : rc.trim.free_variables) {
        x0.push_back(var.initial_guess);
    }

    // 3. Choose solver based on mode
    if (rc.trim.mode == TrimMode::EQUILIBRIUM) {
        if constexpr (std::is_same_v<Scalar, double>) {
            // Numeric trim: Newton-Raphson or KINSOL
            SolveNumericTrim(rc, x0);
        } else {
            // Symbolic trim: Build CasADi problem and solve
            SolveSymbolicTrim(rc, x0);
        }
    }

    // 4. Bind signal pointers for Step phase
    BindAllSignals();
}

void SolveSymbolicTrim(const RunConfig& rc, std::vector<double>& x0) {
    // Create symbolic variables for free variables
    casadi::MX x = casadi::MX::sym("x", rc.trim.free_variables.size());

    // Map symbolic variables to Backplane signals
    int idx = 0;
    for (const auto& var : rc.trim.free_variables) {
        backplane_.set_symbolic(var.name, x(idx++));
    }

    // Trace one Step to build the dynamics graph
    Step(MX::sym("t"), MX::sym("dt"));

    // Extract target signals as symbolic expressions
    std::vector<casadi::MX> residuals;
    for (const auto& [signal, target] : rc.trim.targets) {
        residuals.push_back(backplane_.get_symbolic(signal) - target);
    }

    // Build NLP
    casadi::MXDict nlp = {
        {"x", x},
        {"f", casadi::MX::sumsqr(casadi::MX::vertcat(residuals))}
    };

    // Add bounds
    casadi::DMDict bounds;
    std::vector<double> lbx, ubx;
    for (const auto& var : rc.trim.free_variables) {
        lbx.push_back(var.bounds.first);
        ubx.push_back(var.bounds.second);
    }
    bounds["lbx"] = lbx;
    bounds["ubx"] = ubx;
    bounds["x0"] = x0;

    // Solve
    auto solver = casadi::nlpsol("trim", "ipopt", nlp, rc.trim.solver_options);
    auto result = solver(bounds);

    // Apply solution to Backplane
    auto x_opt = static_cast<std::vector<double>>(result["x"]);
    idx = 0;
    for (const auto& var : rc.trim.free_variables) {
        backplane_.set(var.name, x_opt[idx++]);
    }
}
```

#### 13.6.4 Trim Validation

After solving, validate the trim solution:

```cpp
TrimResult Simulator::ValidateTrim(const RunConfig& rc) {
    TrimResult result;

    // Check residuals
    for (const auto& [signal, target] : rc.trim.targets) {
        double actual = backplane_.get(signal);
        double error = std::abs(actual - target);
        result.residuals[signal] = error;

        if (error > rc.trim.solver.tolerance * 10) {
            result.warnings.push_back(
                fmt::format("Trim residual for '{}' is {:.2e} (target: {})",
                    signal, error, target));
        }
    }

    // Check bounds
    for (const auto& var : rc.trim.free_variables) {
        double value = backplane_.get(var.name);
        if (value <= var.bounds.first + 1e-6 ||
            value >= var.bounds.second - 1e-6) {
            result.warnings.push_back(
                fmt::format("'{}' at bound: {:.4f} (bounds: [{}, {}])",
                    var.name, value, var.bounds.first, var.bounds.second));
        }
    }

    result.success = result.warnings.empty();
    return result;
}
```

#### 13.6.5 Trim Modes Summary

| Mode | Description | Use Case |
| :--- | :--- | :--- |
| `EQUILIBRIUM` | Solve for zero derivatives | Steady-state flight conditions |
| `INITIAL_GUESS` | Apply initial guesses without solving | Quick setup, debugging |
| `OPTIMIZATION` | Minimize a cost function | Optimal trim (min fuel, max range) |

> [!NOTE]
> **Symbolic Trim Advantage:** Using `Simulator<casadi::MX>` for trim provides exact gradients to the solver, enabling faster convergence for complex, highly nonlinear systems. For simple cases, numeric Newton-Raphson is faster.

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

---

### 13.9 Data Dictionary & Tooling Integration

The **Data Dictionary** (auto-generated at Provision, see Section 3.4) is the single source of truth for all signals. Tooling reads the dictionary—no separate schema file needed.

#### Tooling Uses

**1. Config Validation**

```bash
# Validate config against a provisioned simulation's data dictionary
$ icarus provision scenarios/x15_mission.yaml --output-dict data_dictionary.yaml
$ icarus validate scenarios/x15_mission.yaml --dict data_dictionary.yaml

✓ All signal references valid
⚠ Warning: "X15.Aero.alpah" looks like typo for "X15.Aero.alpha"
✗ Error: "X15.Thruster.force" not found in data dictionary
```

**2. IDE Autocomplete**

```yaml
# VS Code extension reads data_dictionary.yaml for autocomplete
components:
  - type: JetEngine
    config:
      input_altitude: "Env|"  # Suggests: Environment.Atmosphere.Altitude
```

**3. Telemetry/Recording Metadata**

```cpp
// Recorder reads data dictionary for units and descriptions
Recording rec("output.icarec");
rec.LoadDataDictionary("output/data_dictionary.yaml");
rec.AddSignal("X15.Nav.position");  // Dictionary provides: units=m, type=Vec3
```

**4. UI Signal Browser (Daedalus)**

Daedalus reads the data dictionary to populate its signal browser with types, units, and descriptions.

#### Why Auto-Generated?

| Manual Schema | Auto-Generated Dictionary |
| :--- | :--- |
| Can drift from reality | Always matches actual signals |
| Extra maintenance burden | Zero maintenance—just run Provision |
| May have typos | Generated from code—no typos |
| Incomplete coverage | Complete by definition |

> [!TIP]
> **Workflow:** Run `icarus provision` once to generate the dictionary. Use it for all tooling. Regenerate whenever components change.

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

### 15.1 Phase Execution Strategies

The basic `janus::where()` gating has tradeoffs. Here are alternative strategies:

#### Strategy 1: Output Gating (Default)

**All components execute; outputs are multiplied by phase mask.**

```cpp
*output_thrust_ = thrust_raw * janus::where(*phase_ == BOOST, 1.0, 0.0);
```

| Pros | Cons |
| :--- | :--- |
| Simple, symbolic-compatible | Wastes compute on inactive components |
| No scheduler complexity | Graph includes all phases (large) |
| Deterministic execution order | |

**Best for:** Small simulations, symbolic mode required for all phases.

#### Strategy 2: Scheduler-Level Skipping (Numeric Only)

**Scheduler checks phase and skips inactive components entirely.**

```cpp
// Scheduler implementation
void Scheduler::Step(double t, double dt) {
    int32_t current_phase = *phase_signal_;

    for (auto* comp : components_) {
        // Check if component is active in current phase
        if (comp->IsActiveInPhase(current_phase)) {
            comp->Step(t, dt);
        } else {
            // Zero outputs without executing
            comp->ZeroOutputs();
        }
    }
}

// Component declares active phases
class Booster : public Component<double> {
    std::set<int32_t> active_phases_ = {Phase::BOOST};

    bool IsActiveInPhase(int32_t phase) const override {
        return active_phases_.contains(phase);
    }
};
```

| Pros | Cons |
| :--- | :--- |
| Saves compute for inactive components | Not compatible with symbolic mode |
| Execution time scales with active set | Slightly more complex scheduler |

**Best for:** Large multi-stage vehicles in numeric mode, HITL.

#### Strategy 3: Phase-Specific Symbolic Graphs

**Generate separate CasADi functions per phase, stitch at boundaries.**

```cpp
// Generate one graph per phase
std::map<Phase, casadi::Function> phase_graphs;

for (Phase p : {BOOST, COAST, REENTRY}) {
    // Create symbolic sim with only active components
    Simulator<MX> sym_sim;
    sym_sim.SetActivePhase(p);
    sym_sim.Provision(config);
    sym_sim.Stage(run_config);

    // Export phase-specific dynamics
    phase_graphs[p] = sym_sim.GenerateGraph("dynamics_" + std::to_string(p));
}

// Trajectory optimizer stitches phases at boundaries
Opti opti;
for (int k = 0; k < N; ++k) {
    Phase p = phase_sequence[k];
    auto x_next = phase_graphs[p](x[k], u[k], dt);
    opti.subject_to(x[k+1] == x_next);
}
```

| Pros | Cons |
| :--- | :--- |
| Minimal graph size per phase | Requires phase sequence known a priori |
| Optimal for trajectory optimization | More complex optimization setup |
| Avoids `janus::where()` overhead | Transitions must be explicit |

**Best for:** Trajectory optimization, known mission profiles.

#### Strategy 4: Hybrid (Recommended)

**Use scheduler skipping for numeric mode, output gating for symbolic mode.**

```cpp
template <typename Scalar>
void Booster::Step(Scalar t, Scalar dt) {
    if constexpr (std::is_same_v<Scalar, double>) {
        // Numeric mode: scheduler handles skipping, no gating needed
        Scalar thrust = compute_thrust(...);
        *output_thrust_ = thrust;
    } else {
        // Symbolic mode: full gating for graph correctness
        Scalar thrust_raw = compute_thrust(...);
        Scalar is_active = janus::where(*phase_ == BOOST, Scalar(1.0), Scalar(0.0));
        *output_thrust_ = thrust_raw * is_active;
    }
}
```

| Pros | Cons |
| :--- | :--- |
| Best performance in numeric mode | Two code paths to maintain |
| Full symbolic compatibility | `if constexpr` adds complexity |
| Optimal for both use cases | |

> [!TIP]
> **Recommendation:** Start with Strategy 1 (output gating) for simplicity. Move to Strategy 4 (hybrid) when profiling shows inactive component overhead is significant (typically >20% of Step time).

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

### 16.1 Runtime Debug & Trace Mode

Debug and Trace modes can be enabled **without recompilation**—critical for diagnosing issues in production builds.

#### Runtime Configuration

```cpp
// Enable tracing for specific components/signals at runtime
sim.EnableTrace("Vehicle.Aero.*");           // All Aero signals
sim.EnableTrace("Vehicle.EOM.position");     // Specific signal
sim.EnableTrace("GNC.*", TraceLevel::DEBUG); // Component with level

// Disable tracing
sim.DisableTrace("Vehicle.Aero.*");

// Query current trace status
auto traced_signals = sim.GetTracedSignals();
```

#### Configuration File

```yaml
services:
  debug:
    enabled: true

    # Enable tracing for signal patterns (glob syntax)
    trace_signals:
      - pattern: "Vehicle.Nav.*"
        level: DEBUG
        rate_hz: 100  # Downsample high-frequency signals
      - pattern: "Vehicle.Aero.alpha"
        level: TRACE
        rate_hz: 1000

    # Enable component-level tracing
    trace_components:
      - name: "Vehicle.GNC"
        level: DEBUG
        log_inputs: true
        log_outputs: true
        log_timing: true  # Measure step duration

    # Breakpoints (pause simulation when condition met)
    breakpoints:
      - condition: "Vehicle.Nav.altitude < 0"
        action: PAUSE
        message: "Vehicle below ground level"
      - condition: "Vehicle.EOM.velocity_norm > 1000"
        action: LOG_WARNING
        message: "Velocity exceeds expected range"

    # Output options
    output:
      console: true
      file: "logs/trace_{timestamp}.log"
      format: JSON  # or TEXT, CSV
```

#### Implementation

```cpp
template <typename Scalar>
class TracingBackplane : public Backplane<Scalar> {
    std::unordered_set<std::string> traced_signals_;
    std::ofstream trace_file_;

public:
    void EnableTrace(const std::string& pattern) {
        for (const auto& [name, meta] : registry_) {
            if (MatchGlob(name, pattern)) {
                traced_signals_.insert(name);
            }
        }
    }

    // Called after each Step
    void LogTracedSignals(Scalar t) {
        if (traced_signals_.empty()) return;

        json frame;
        frame["t"] = t;
        for (const auto& name : traced_signals_) {
            frame[name] = GetSignalValue(name);
        }
        trace_file_ << frame.dump() << "\n";
    }
};
```

#### Component Timing Profiler

```cpp
class ComponentProfiler {
    struct Profile {
        std::string name;
        double total_time_us = 0;
        double max_time_us = 0;
        int64_t call_count = 0;
    };

    std::unordered_map<Component*, Profile> profiles_;

public:
    void BeginStep(Component* comp) {
        start_times_[comp] = HighResolutionClock::now();
    }

    void EndStep(Component* comp) {
        auto elapsed = HighResolutionClock::now() - start_times_[comp];
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

        auto& p = profiles_[comp];
        p.total_time_us += us;
        p.max_time_us = std::max(p.max_time_us, static_cast<double>(us));
        p.call_count++;
    }

    void PrintReport() {
        ICARUS_INFO("=== Component Timing Report ===");
        for (const auto& [comp, p] : profiles_) {
            double avg = p.total_time_us / p.call_count;
            ICARUS_INFO("{}: avg={:.2f}us, max={:.2f}us, calls={}",
                p.name, avg, p.max_time_us, p.call_count);
        }
    }
};
```

#### Interactive Debugging (Future: Daedalus Integration)

```cpp
// For integration with Daedalus visualization tool
class DebugServer {
public:
    void Start(int port = 9999) {
        // ZMQ REP socket for debug commands
        socket_.bind("tcp://*:" + std::to_string(port));
    }

    // Commands from Daedalus:
    // {"cmd": "pause"}
    // {"cmd": "step", "count": 1}
    // {"cmd": "set_breakpoint", "condition": "Nav.alt < 0"}
    // {"cmd": "get_signal", "name": "Nav.position"}
    // {"cmd": "set_signal", "name": "Control.throttle", "value": 0.5}
    // {"cmd": "enable_trace", "pattern": "Aero.*"}
};
```

> [!NOTE]
> **Performance Impact:** Tracing adds overhead only for traced signals. When no signals are traced, cost is a single branch per Step (negligible). Component timing profiler adds ~100ns per component when enabled.

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

### 19.1 Versioned Recording Schema

Every recording includes a versioned schema for forward/backward compatibility:

```json
{
  "schema_version": "2.3.1",
  "icarus_version": "1.5.0",
  "created_at": "2024-12-22T10:30:00Z",

  "signals": [
    {
      "name": "Vehicle.Nav.position",
      "type": "Vec3<double>",
      "units": "m",
      "lifecycle": "dynamic",
      "offset": 0,
      "size": 24
    },
    {
      "name": "Vehicle.Aero.alpha",
      "type": "double",
      "units": "rad",
      "lifecycle": "dynamic",
      "offset": 24,
      "size": 8
    }
  ],

  "components": [
    {
      "name": "Vehicle.EOM",
      "type": "RigidBody6DOF",
      "state_offset": 0,
      "state_size": 13
    }
  ],

  "compatibility": {
    "min_icarus_version": "1.0.0",
    "warmstart_compatible": true,
    "breaking_changes": []
  },

  "checksum": "sha256:a1b2c3d4..."
}
```

### 19.2 Schema Compatibility Rules

| Scenario | Behavior |
| :--- | :--- |
| **Same schema version** | Full compatibility, warmstart works |
| **Newer schema, additive changes** | Compatible—new signals ignored on load |
| **Newer schema, removed signals** | Warning—missing signals set to defaults |
| **Breaking schema change** | Error with clear message |

#### Compatibility Validation

```cpp
ValidationResult Simulator::ValidateRecording(const std::string& path) {
    auto recording_schema = Recording::LoadSchema(path);
    auto current_schema = GetCurrentSchema();

    ValidationResult result;

    // Check version compatibility
    if (recording_schema.version < current_schema.min_compatible_version) {
        result.AddError("Recording schema {} is too old (min: {})",
            recording_schema.version, current_schema.min_compatible_version);
    }

    // Check for missing signals required by current components
    for (const auto& input : GetAllRequiredInputs()) {
        if (!recording_schema.HasSignal(input.name)) {
            if (input.has_default) {
                result.AddWarning("Signal '{}' missing, using default: {}",
                    input.name, input.default_value);
            } else {
                result.AddError("Required signal '{}' not in recording", input.name);
            }
        }
    }

    // Check for type mismatches
    for (const auto& sig : recording_schema.signals) {
        if (current_schema.HasSignal(sig.name)) {
            auto current = current_schema.GetSignal(sig.name);
            if (sig.type != current.type) {
                result.AddError("Type mismatch for '{}': recording={}, current={}",
                    sig.name, sig.type, current.type);
            }
        }
    }

    return result;
}
```

### 19.3 Default Recording Policy

> [!TIP]
> **Recommended: Record everything.** Disk is cheap; debugging time is not.

```yaml
services:
  recording:
    enabled: true
    format: ICAREC
    path: "output/run_{timestamp}.icarec"

    # Record ALL signals by default
    policy: ALL

    # Exclude only high-frequency noise (optional)
    exclude:
      - "Debug.*"
      - "*.raw_sensor_*"

    # Ensure warmstart compatibility
    warmstart_compatible: true  # Validates all inputs are recorded
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

// Validate compatibility BEFORE attempting warmstart
auto validation = sim.ValidateRecording("recording.icarec");
if (!validation.IsValid()) {
    for (const auto& error : validation.errors) {
        ICARUS_ERROR("{}", error);
    }
    throw WarmstartError("Recording incompatible with current simulation");
}

// Warmstart with validated recording
sim.warmstart_from("recording.icarec", t_warmstart);
// State vector is restored to t_warmstart
sim.step(dt);  // Continue from that point
```

> [!WARNING]
> **Warmstart requires all input signals to be recorded.** If a component reads an external input that wasn't recorded, warmstart will fail. Use `warmstart_compatible: true` in recording config to catch this at recording time.

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
