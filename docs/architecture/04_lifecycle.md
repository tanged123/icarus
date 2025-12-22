# Simulation Lifecycle & Component Protocol

**Related:** [01_core_philosophy.md](01_core_philosophy.md) | [02_component_protocol.md](02_component_protocol.md) | [05_execution_model.md](05_execution_model.md)

---

The architecture strictly separates memory management (**Provision**) from physics initialization (**Stage**) and temporal evolution (**Step**). This ensures high performance for Monte Carlo loops by eliminating redundant memory allocation.

Every component behaves identically to the `Simulator` host:

---

## 1. Lifecycle Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              APPLICATION START                               │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  PROVISION (once)                                                            │
│  • Allocate memory, state vectors                                           │
│  • Register outputs on Backplane                                            │
│  • Load parameters from config                                              │
│  • Parse static assets (tables, meshes)                                     │
│  • Generate Data Dictionary                                                 │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                    ┌─────────────────┴─────────────────┐
                    │         FOR EACH RUN/EPISODE      │
                    ▼                                   │
┌─────────────────────────────────────────────────────────────────────────────┐
│  STAGE (per run)                                                             │
│  • Zero state derivatives, set t=0                                          │
│  • Apply initial conditions from RunConfig                                  │
│  • Wire inputs (resolve signal pointers)                                    │
│  • Run trim/equilibrium solver if needed                                    │
│  • Build execution order (topological sort)                                 │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                    ┌─────────────────┴─────────────────┐
                    │         SIMULATION LOOP           │
                    ▼                                   │
┌─────────────────────────────────────────────────────────────────────────────┐
│  STEP (per Δt)                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                   │
│  │   PreStep()  │ ─► │    Step()    │ ─► │  PostStep()  │                   │
│  │  (optional)  │    │ (all comps)  │    │  (optional)  │                   │
│  └──────────────┘    └──────────────┘    └──────────────┘                   │
│                              │                                               │
│                              ▼                                               │
│                    ┌──────────────────┐                                      │
│                    │    Integrator    │  X_new = X + dt * f(X_dot)          │
│                    └──────────────────┘                                      │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      │ (end of run)
                                      ▼
                    ┌─────────────────────────────────────┐
                    │  Return to STAGE for next run       │
                    │  (Monte Carlo, parameter sweep)     │
                    └─────────────────────────────────────┘
                                      │
                                      │ (application exit)
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  SHUTDOWN                                                                    │
│  • Flush recording buffers                                                  │
│  • Close file handles                                                       │
│  • Release resources                                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Phase I: Provision (Setup)

**Goal:** Heavy lifting. Allocate memory, compile compute graphs, parse immutable assets (URDFs, Meshes).
**Frequency:** Called once when the application launches.

* **Action:** Allocate large state vectors and signal buffers.
* **Action:** Register public outputs in the Backplane.
* **Action:** Declare parameter structs.
* **Action:** Load static reference tables (atmosphere, gravity).

---

## 3. Phase II: Stage (The Merged Reset/Trim)

**Goal:** Prepare the mathematical state for $t=0$. This merges "Reset" (hard values) and "Trim" (solved values) into a single pipeline.
**Frequency:** Called at the start of every new episode/run.

The `Stage(RunConfig)` function acts as a solver pipeline:
1. **Zeroing:** Wipe state derivatives ($\dot{x}$) and time ($t=0$).
2. **Hard Constraints (Reset):** Apply fixed values from config (e.g., `Position = [0,0,1000]`).
3. **Soft Constraints (Trim):** If `RunConfig.mode == EQUILIBRIUM`, we can leverage the **Dual-Headed** nature of Janus:
    * *Numeric Trim:* Simple Newton-Raphson on the C++ `double` step function for basic cases.
    * *Symbolic Trim:* Instantiate a temporary `Simulator<casadi::MX>`, trace the forces calculation graph $\sum F(x, u)$, and pass it to a robust nonlinear solver (IPOPT/Kinsol) to find the exact trim state. This is significantly more robust for complex, highly nonlinear vehicles.
4. **Binding:** Resolve input pointers from the Backplane.

---

## 4. Phase III: Step (Execute)

**Goal:** Advance the simulation by a discrete time delta ($\Delta t$).
**Frequency:** Called in a loop (thousands of times per episode).

* **Action:** Read `*input_ptr` from Backplane.
* **Action:** Calculate dynamics $\dot{x} = f(x, u)$.
* **Action:** Write `*output_ptr` to Backplane.
* **Constraint:** NO memory allocation. NO looking up signals by string (do that in Stage).

---

## 5. Extended Lifecycle Hooks

Beyond the core Provision/Stage/Step, components may implement optional hooks for finer control:

```cpp
template <typename Scalar>
class Component {
public:
    // === CORE LIFECYCLE (Required) ===
    virtual void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) = 0;
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

---

## 6. Hook Execution Order

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

---

## 7. Phase Transition Example

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

---

## 8. Error Handling Hook

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
