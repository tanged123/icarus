# Event Handling & Flight Phases

**Related:** [04_lifecycle.md](04_lifecycle.md) | [05_execution_model.md](05_execution_model.md) | [07_janus_integration.md](07_janus_integration.md)

---

Vehicles have discrete events (stage separation, parachute deploy) and flight phases (boost, coast, descent).

---

## 1. Phase Concept

Each **Entity** can have a `phase` signal (int32):

```yaml
# Phase definition for a rocket
X15.phase: 0  # int32 signal
# Phases: 0=GROUND, 1=BOOST, 2=COAST, 3=REENTRY, 4=LANDED
```

---

## 2. Phase-Dependent Components ("Ghosting")

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

---

## 3. Phase Execution Strategies

The basic `janus::where()` gating has tradeoffs. Here are alternative strategies:

### Strategy 1: Output Gating (Default)

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

### Strategy 2: Scheduler-Level Skipping (Numeric Only)

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

### Strategy 3: Phase-Specific Symbolic Graphs

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

### Strategy 4: Hybrid (Recommended)

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

---

## 4. Phase Transitions

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

## 5. Phase Lifecycle Hooks

Components can implement phase-specific behavior:

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
        }
    }
};
```

See [04_lifecycle.md](04_lifecycle.md) for full hook documentation.
