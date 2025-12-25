# Component Protocol

**Related:** [01_core_philosophy.md](01_core_philosophy.md) | [03_signal_backplane.md](03_signal_backplane.md) | [04_lifecycle.md](04_lifecycle.md)

---

Components are the atomic units of execution. Each component has a well-defined **interface** consisting of:

| Category | Description | Types | Registered At | Optimizable |
|:---------|:------------|:------|:--------------|:------------|
| **Outputs** | Dynamic signals produced by the component | `Scalar`, `Vec3<Scalar>` | Provision | N/A |
| **Inputs** | Dynamic signal ports consumed by the component | `Scalar`, `Vec3<Scalar>` | Provision | N/A |
| **Parameters** | Continuous configurable values | `Scalar` | Provision | Yes |
| **Config** | Discrete configuration values | `int`, `bool`, `enum` | Provision | No |

All interface elements are:

- **Discoverable** via the auto-generated Data Dictionary
- **Accessible** at runtime via the Signal Access API (get/set any signal)

---

## 1. Component Interface Model

### 1.1 Outputs

Outputs are dynamic signals **produced** by the component. They represent the component's identity—what it *is*.

```cpp
// Outputs: Component owns storage, updates each Step
Vec3<Scalar> thrust_;
Scalar fuel_flow_;

void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    bp.register_output("thrust", &thrust_, "N", "Thrust force vector");
    bp.register_output("fuel_flow", &fuel_flow_, "kg/s", "Fuel mass flow rate");
}
```

### 1.2 Inputs

Inputs are dynamic signal **ports** that the component needs. The component declares *what* it needs; the simulator wires *where* it comes from.

```cpp
// Inputs: Handles to external signals, wired at Stage
InputHandle<Scalar> throttle_;
InputHandle<Vec3<Scalar>> velocity_;

void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    // Declare input ports (not yet wired to sources)
    bp.register_input("throttle", &throttle_, "", "Throttle command [0,1]");
    bp.register_input("velocity", &velocity_, "m/s", "Vehicle velocity");
}

void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    // Wiring happens here (source comes from wiring config, not component)
    bp.wire_inputs();  // Connects all declared inputs to their sources
}
```

### 1.3 Parameters

Parameters are **continuous configurable values** that affect component behavior. They are **Scalar-typed** for full symbolic compatibility—the same code works with `double` or `casadi::MX`. Parameters can be optimization variables in trim/trajectory solvers.

```cpp
// Parameters: Scalar-typed, can be optimization variables
Scalar max_thrust_;
Scalar isp_sea_level_;
Scalar isp_vacuum_;
Scalar time_constant_;

void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    bp.register_param("max_thrust", &max_thrust_,
                      cfg.get<Scalar>("max_thrust", Scalar{50000.0}),
                      "N", "Maximum thrust");
    bp.register_param("isp_sea_level", &isp_sea_level_,
                      cfg.get<Scalar>("isp_sea_level", Scalar{280.0}),
                      "s", "Specific impulse at sea level");
    bp.register_param("isp_vacuum", &isp_vacuum_,
                      cfg.get<Scalar>("isp_vacuum", Scalar{320.0}),
                      "s", "Specific impulse in vacuum");
    bp.register_param("time_constant", &time_constant_,
                      cfg.get<Scalar>("time_constant", Scalar{0.5}),
                      "s", "Engine spool time constant");
}
```

### 1.4 Config

Config values are **discrete configuration settings** that control component behavior. They use concrete types (`int`, `bool`, `enum`) and are **not part of the symbolic graph**—they cannot be optimization variables.

```cpp
// Config: Discrete values, not optimizable
int num_nozzles_;
bool enable_vectoring_;
GravityModel gravity_model_;

void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    bp.register_config("num_nozzles", &num_nozzles_,
                       cfg.get<int>("num_nozzles", 4),
                       "Number of engine nozzles");
    bp.register_config("enable_vectoring", &enable_vectoring_,
                       cfg.get<bool>("enable_vectoring", true),
                       "Enable thrust vector control");
    bp.register_config("gravity_model", &gravity_model_,
                       cfg.get<GravityModel>("gravity_model", GravityModel::WGS84),
                       "Gravity model selection");
}
```

#### Config vs Parameters

| Aspect | Parameters (`Scalar`) | Config (`int`, `bool`, `enum`) |
|:-------|:----------------------|:-------------------------------|
| **Type** | Continuous | Discrete |
| **Symbolic** | Yes (part of CasADi graph) | No (fixed during trace) |
| **Optimizable** | Yes (trim, trajectory) | No |
| **Example** | Mass, thrust limits, Isp | Nozzle count, flags, model selection |
| **Runtime change** | Safe (value substitution) | Requires re-Provision |

#### Config and Symbolic Mode

Config values are resolved at Provision time, **before** symbolic tracing. This is safe:

```cpp
void Provision(...) {
    // Config resolved here—branch is fixed before trace
    if (cfg.get<bool>("use_j2")) {
        enable_j2_ = true;
    }
}
```

Using config in Step requires care:

```cpp
void Step(Scalar t, Scalar dt) {
    // BAD: CasADi can't trace through bool branch
    if (enable_j2_) {
        accel += j2_perturbation();
    }

    // GOOD: Convert to Scalar flag, use janus::where()
    // (j2_flag_ set at Provision: enable_j2_ ? Scalar{1} : Scalar{0})
    accel += janus::where(j2_flag_ > Scalar{0.5},
                          j2_perturbation(),
                          Vec3<Scalar>::Zero());
}
```

---

## 2. Complete Component Example

```cpp
template <typename Scalar>
class JetEngine : public Component<Scalar> {
    // === OUTPUTS (dynamic signals we produce) ===
    Scalar thrust_;
    Scalar fuel_flow_;
    Scalar spool_speed_;

    // === INPUTS (handles to external signals) ===
    InputHandle<Scalar> altitude_;
    InputHandle<Scalar> throttle_;
    InputHandle<Scalar> mach_;

    // === PARAMETERS (continuous, Scalar-typed, optimizable) ===
    Scalar max_thrust_;
    Scalar time_constant_;
    Scalar min_throttle_;

    // === CONFIG (discrete, not optimizable) ===
    int num_nozzles_;
    bool enable_afterburner_;
    Scalar afterburner_flag_;  // Scalar version for symbolic branching

public:
    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // --- Register Outputs ---
        bp.register_output("thrust", &thrust_, "N", "Net thrust force");
        bp.register_output("fuel_flow", &fuel_flow_, "kg/s", "Fuel mass flow rate");
        bp.register_output("spool_speed", &spool_speed_, {
            .units = "rad/s",
            .description = "Turbine spool angular velocity",
            .integrable = true  // This is state—will be integrated
        });

        // --- Register Input Ports ---
        bp.register_input("altitude", &altitude_, "m", "Geometric altitude");
        bp.register_input("throttle", &throttle_, "", "Throttle command [0,1]");
        bp.register_input("mach", &mach_, "", "Mach number");

        // --- Register Parameters (Scalar, optimizable) ---
        bp.register_param("max_thrust", &max_thrust_,
                          cfg.get<Scalar>("max_thrust", Scalar{50000.0}),
                          "N", "Maximum thrust output");
        bp.register_param("time_constant", &time_constant_,
                          cfg.get<Scalar>("time_constant", Scalar{0.5}),
                          "s", "Engine spool time constant");
        bp.register_param("min_throttle", &min_throttle_,
                          cfg.get<Scalar>("min_throttle", Scalar{0.0}),
                          "", "Minimum throttle setting");

        // --- Register Config (discrete, not optimizable) ---
        bp.register_config("num_nozzles", &num_nozzles_,
                           cfg.get<int>("num_nozzles", 1),
                           "Number of engine nozzles");
        bp.register_config("enable_afterburner", &enable_afterburner_,
                           cfg.get<bool>("enable_afterburner", false),
                           "Enable afterburner capability");

        // Convert bool to Scalar for symbolic-safe branching in Step
        afterburner_flag_ = enable_afterburner_ ? Scalar{1.0} : Scalar{0.0};
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Wire all declared inputs to their sources
        // Sources come from wiring config, not hardcoded here
        bp.wire_inputs();
    }

    void Step(Scalar t, Scalar dt) override {
        // Read inputs via handles
        Scalar alt = altitude_.get();
        Scalar thr = throttle_.get();
        Scalar m = mach_.get();

        // Physics calculations using parameters...
        Scalar base_thrust = max_thrust_ * thr;

        // Use janus::where() for symbolic-safe conditional
        Scalar afterburner_bonus = janus::where(
            afterburner_flag_ > Scalar{0.5},
            base_thrust * Scalar{0.3},  // 30% bonus with afterburner
            Scalar{0.0}
        );

        thrust_ = base_thrust + afterburner_bonus;
        fuel_flow_ = calculated_fuel_flow;
    }
};
```

---

## 3. Wiring Configuration

Input wiring is specified **externally** to the component—either in YAML config or programmatically. Components never hardcode their input sources. Until configuration files are fleshed out, rely on programmatic wiring.

### 3.1 YAML Wiring

```yaml
# scenario.yaml
components:
  - type: JetEngine
    name: MainEngine
    entity: X15
    config:
      # Parameters (Scalar, optimizable)
      max_thrust: 75000.0
      time_constant: 0.5
      # Config (discrete, not optimizable)
      num_nozzles: 4
      enable_afterburner: true

# wiring.yaml (or inline in scenario)
wiring:
  X15.MainEngine:
    altitude: "Environment.Atmosphere.altitude"
    throttle: "X15.GNC.throttle_cmd"
    mach: "Environment.Atmosphere.mach"
```

### 3.2 Programmatic Wiring

```cpp
// Explicit wiring at simulator level
sim.Wire("X15.MainEngine.altitude", "Environment.Atmosphere.altitude");
sim.Wire("X15.MainEngine.throttle", "X15.GNC.throttle_cmd");
sim.Wire("X15.MainEngine.mach", "Environment.Atmosphere.mach");

// Or load from config
sim.LoadWiring("wiring.yaml");

// Validate all inputs are wired before run
sim.ValidateWiring();  // Throws if any input port is unwired
```

---

## 4. Why This Design?

### 4.1 Outputs vs Inputs

| Aspect | Outputs | Inputs |
|:-------|:--------|:-------|
| **Defined by** | Component code | Component code (ports) + Config (wiring) |
| **Rationale** | Identity—what the component *is* | Interface—what it *needs* |
| **Flexibility** | Fixed per component type | Wiring fully reconfigurable |
| **Example** | JetEngine always produces thrust | Altitude could come from sensor, model, or test harness |

> [!NOTE]
> **Outputs are identity. Inputs are interface. Wiring is topology.**
> A JetEngine without thrust isn't a JetEngine. But where altitude comes from—barometer, GPS, truth model—that's just wiring.

### 4.2 Why Explicit Input Registration?

The previous pattern of only declaring inputs implicitly at `resolve()` time had problems:

| Issue | Old Pattern | New Pattern |
|:------|:------------|:------------|
| **Discoverability** | Inputs not in Data Dictionary | All inputs discoverable |
| **Validation** | Runtime crash if signal missing | Pre-run validation possible |
| **Coupling** | Component knows provider names | Component only knows port names |
| **Tooling** | Can't visualize input requirements | Full interface visible |

### 4.3 Why Scalar Parameters?

Parameters are `Scalar`-typed (not `double`) for symbolic compatibility:

```cpp
// In symbolic mode, parameters can be optimization variables
Scalar max_thrust_;  // casadi::MX in symbolic mode

// Trim solver can optimize over parameters
auto optimal_thrust = trim_solver.Optimize("X15.MainEngine.max_thrust");
```

---

## 5. Signal Access API

Any registered signal (output, input source, parameter, or config) can be accessed at runtime:

```cpp
// Get any signal value
Scalar thrust = sim.Get<Scalar>("X15.MainEngine.thrust");
Vec3<Scalar> pos = sim.Get<Vec3<Scalar>>("X15.EOM.position");

// Set parameters (Scalar)
sim.Set("X15.MainEngine.max_thrust", Scalar{60000.0});

// Set config values (discrete types)
sim.Set("X15.MainEngine.num_nozzles", 6);
sim.Set("X15.MainEngine.enable_afterburner", false);

// Query signal metadata
SignalInfo info = sim.GetSignalInfo("X15.MainEngine.thrust");
// info.units = "N"
// info.type = SignalType::Output
// info.description = "Net thrust force"
```

> [!WARNING]
> **Config changes at runtime require re-Provision.** Changing discrete config values (int, bool, enum) after Provision may leave the component in an inconsistent state. Parameters (Scalar) can be changed safely at any time.

### Use Cases

| Use Case | API |
|:---------|:----|
| **Debugging** | `sim.Get("signal")` to inspect values |
| **Parameter Tuning** | `sim.Set("param", Scalar{value})` during runtime |
| **Config Override** | `sim.Set("config", value)` before Provision |
| **Test Harnesses** | Override inputs/outputs for unit testing |
| **Recording** | Record any signal by name |
| **Telemetry** | Stream selected signals |

---

## 6. Auto-Generated Data Dictionary

After Provision, the Simulator generates a complete interface catalog:

```cpp
void Simulator::Provision(const ScenarioConfig& cfg) {
    for (auto* comp : components_) {
        comp->Provision(backplane_, comp->config_);
    }
    backplane_.GenerateDataDictionary("output/data_dictionary.yaml");
}
```

**Output: `data_dictionary.yaml`**

```yaml
# AUTO-GENERATED by Icarus Simulator
# Scenario: scenarios/x15_mission.yaml
# Generated: 2024-12-22T10:30:00Z

components:
  X15.MainEngine:
    type: JetEngine

    outputs:
      thrust:
        type: Scalar
        units: N
        description: "Net thrust force"
      fuel_flow:
        type: Scalar
        units: kg/s
        description: "Fuel mass flow rate"
      spool_speed:
        type: Scalar
        units: rad/s
        description: "Turbine spool angular velocity"
        integrable: true

    inputs:
      altitude:
        type: Scalar
        units: m
        description: "Geometric altitude"
        wired_to: "Environment.Atmosphere.altitude"
      throttle:
        type: Scalar
        units: ""
        description: "Throttle command [0,1]"
        wired_to: "X15.GNC.throttle_cmd"
      mach:
        type: Scalar
        units: ""
        description: "Mach number"
        wired_to: "Environment.Atmosphere.mach"

    parameters:  # Scalar-typed, optimizable
      max_thrust:
        type: Scalar
        units: N
        value: 75000.0
        description: "Maximum thrust output"
      time_constant:
        type: Scalar
        units: s
        value: 0.5
        description: "Engine spool time constant"
      min_throttle:
        type: Scalar
        units: ""
        value: 0.0
        description: "Minimum throttle setting"

    config:  # Discrete, not optimizable
      num_nozzles:
        type: int
        value: 4
        description: "Number of engine nozzles"
      enable_afterburner:
        type: bool
        value: true
        description: "Enable afterburner capability"

  Environment.Atmosphere:
    type: StandardAtmosphere
    outputs:
      altitude:
        type: Scalar
        units: m
        description: "Geometric altitude"
      density:
        type: Scalar
        units: kg/m³
        description: "Atmospheric density"
      mach:
        type: Scalar
        units: ""
        description: "Mach number"

# Summary
summary:
  total_components: 12
  total_outputs: 42
  total_inputs: 38
  total_parameters: 24
  total_config: 18
  integrable_states: 13
  unwired_inputs: 0  # Validation: should be 0
```

---

## 7. Data Dictionary Uses

| Use Case | How |
|:---------|:----|
| **Telemetry Setup** | Browse signals, select for streaming |
| **Recording Config** | Reference by name: `["X15.MainEngine.*"]` |
| **Wiring Validation** | Check `unwired_inputs: 0` |
| **Documentation** | Auto-generated, always current |
| **Tooling** | UI signal browser, wiring editor |
| **Test Generation** | Know required inputs for component |

---

## 8. Dependency Discovery & Scheduling

The Scheduler builds a dependency graph from registered inputs/outputs:

```cpp
void Simulator::Stage(const RunConfig& rc) {
    DependencyGraph graph;

    // Build provider map from outputs
    for (auto* comp : components_) {
        for (const auto& out : comp->GetOutputs()) {
            graph.AddProvider(out.full_name, comp);
        }
    }

    // Build dependency edges from inputs
    for (auto* comp : components_) {
        for (const auto& in : comp->GetInputs()) {
            graph.AddDependency(in.wired_to, comp);
        }
    }

    // Topological sort for execution order
    execution_order_ = graph.TopologicalSort();  // Throws on cycle
}
```

---

## 9. Validation

| Validation | When | Error Example |
|:-----------|:-----|:--------------|
| Signal exists | `wire_inputs()` at Stage | `"Env.Atm.altidude" not found. Did you mean "Environment.Atmosphere.altitude"?` |
| Type matches | Wiring validation | `"gnc.mode" is int, but input expects Scalar` |
| No duplicate outputs | `register_output()` at Provision | `"thrust" already registered by LeftEngine` |
| All inputs wired | `ValidateWiring()` | `X15.MainEngine.throttle: input not wired` |
| No dependency cycles | End of Stage | `Cycle detected: A → B → C → A` |

---

## 10. Aggregation Registration

Some quantities require **global aggregation**—summing contributions from many components. Components **opt-in** by registering as sources during Provision.

> [!NOTE]
> **See [12_quantity_aggregation.md](12_quantity_aggregation.md) for complete implementation details.**

### Force Sources

```cpp
template <typename Scalar>
void RocketEngine<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // Standard interface registration
    bp.register_output("thrust", &thrust_mag_, "N", "Thrust magnitude");
    bp.register_param("max_thrust", &max_thrust_, cfg.get(...), "N", "Max thrust");

    // Force source registration (opt-in for aggregation)
    bp.register_force_source({
        .name = full_name_ + ".thrust",
        .force = &thrust_force_,
        .moment = &thrust_moment_,
        .frame = Frame::LOCAL,
        .application_point = &nozzle_pos_,
        .dcm_to_body = &nozzle_dcm_,
        .entity_active = entity_active_ptr_
    });
}
```

### Mass Sources

```cpp
template <typename Scalar>
void FuelTank<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    bp.register_output("fuel_mass", &fuel_mass_, "kg", "Current fuel mass");

    bp.register_mass_source({
        .name = full_name_ + ".fuel",
        .mass = &fuel_mass_,
        .cg_body = &fuel_cg_,
        .inertia_cg = nullptr,
        .lifecycle = Lifecycle::DYNAMIC,
        .entity_active = entity_active_ptr_
    });
}
```

### Why Registration (Not Base Class)?

| Approach | Problem |
|:---------|:--------|
| **Bake into Component** | Forces every component to define force/mass even when N/A |
| **Virtual getForce()** | Breaks symbolic mode (CasADi can't trace through vtables) |
| **Registration** | Opt-in, explicit, traceable, symbolic-compatible |

Components without forces or mass (e.g., `Autopilot`, `IMU`, `Telemetry`) simply don't register—no overhead, no boilerplate.
