# Execution Model

**Related:** [04_lifecycle.md](04_lifecycle.md) | [02_component_protocol.md](02_component_protocol.md) | [19_determinism_parallelism.md](19_determinism_parallelism.md)

---

Since the hierarchy is flat, execution order is determined by **Dependency Analysis** (Topological Sort) or **Explicit Rate Grouping**.

---

## 1. The Scheduler

The `Scheduler` holds `std::vector<Component*>`.

```cpp
// The entire simulation loop is essentially:
for (Component* c : scheduled_components) {
    c->step(t, dt);
}
```

---

## 2. Rate Groups

We support multi-rate execution by having multiple lists:
* **Group 1kHz:** IMU, Actuators, EOMs.
* **Group 100Hz:** GNC, Guidance Logic.
* **Group 10Hz:** Telemetry, Logging, Environmental (Slow).

---

## 3. Rate Group Synchronization

When components run at different rates, **inter-rate signal access** must be well-defined.

### 3.1 The Problem

A 100Hz GNC component reads `Nav.Position` from a 1kHz EOM. Which value does it see?
- The value from 10 steps ago?
- An interpolated value?
- The most recent value?

### 3.2 Synchronization Policies

| Policy | Description | Use Case |
| :--- | :--- | :--- |
| **ZOH (Zero-Order Hold)** | Read the last written value | Default. Simple, deterministic. |
| **Interpolated** | Linear interpolation between last two values | Smooth signals (position, velocity) |
| **Extrapolated** | Predict forward using derivatives | Control systems with latency compensation |
| **Synchronized** | All groups tick at LCM rate | Eliminates issue but wastes compute |

### 3.3 Configuration

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

---

## 4. Implementation

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

---

## 5. Determinism Guarantee

> [!IMPORTANT]
> **All synchronization policies are deterministic.** Given the same initial state and inputs, the simulation produces identical results regardless of wall-clock timing. Interpolation uses fixed coefficients based on rate ratios, not real timestamps.

---

## 6. Symbolic Mode & Multi-Rate Interaction

Symbolic graph generation (for trajectory optimization, AD, etc.) has specific implications for multi-rate scheduling.

### 6.1 Default: Single-Rate Snapshot

> [!IMPORTANT]
> **Symbolic graph generation assumes single-rate execution by default.** The generated CasADi function represents one "snapshot" of the dynamics at the fastest rate.

When you export a symbolic graph via `Simulator<MX>::GenerateGraph()`, all components execute once per call. Rate-group logic is **not** embedded in the graph.

### 6.2 Embedding Rate Transitions (Advanced)

For optimizers that need to respect multi-rate behavior, rate-transition blocks can be explicitly included in the symbolic graph:

```cpp
// Rate transition appears as explicit interpolation in graph
Scalar nav_position_interp = janus::lerp(
    nav_position_prev,
    nav_position_latest,
    alpha  // Fixed coefficient based on rate ratio
);
```

This makes the ZOH or interpolation visible to the optimizer, but increases graph complexity.

### 6.3 Recommendation

| Use Case | Strategy |
| :--- | :--- |
| Trajectory optimization | Single-rate graph (simplest) |
| HITL / real-time | Multi-rate numeric scheduler |
| High-fidelity symbolic | Embed rate-transition blocks |

See [21_symbolic_constraints.md](21_symbolic_constraints.md) for general symbolic mode rules.
