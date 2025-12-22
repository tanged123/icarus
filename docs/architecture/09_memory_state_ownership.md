# Memory Layout & State Ownership

**Related:** [03_signal_backplane.md](03_signal_backplane.md) | [04_lifecycle.md](04_lifecycle.md) | [07_janus_integration.md](07_janus_integration.md)

---

To satisfy `solve_ivp` and optimization solvers, the **Continuous State** must be contiguous.

---

## 1. The Global State Vector

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

---

## 2. State Ownership Model

> [!IMPORTANT]
> **The Simulator owns all state.** Components hold *views* (pointers) into the global vector, not copies.

| Concept | Owner | Lifetime |
| :--- | :--- | :--- |
| **Global State Vector** `X_global_` | Simulator | Provision → Destroy |
| **Component State View** `state_` | Component (pointer only) | Stage → Reset |
| **State Derivatives** `X_dot_global_` | Simulator | Provision → Destroy |

---

## 3. Scatter/Gather Protocol

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

---

## 4. Integration Flow

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

---

## 5. Why Components Don't Integrate

| Approach | Pros | Cons |
| :--- | :--- | :--- |
| **Components integrate themselves** | Simple, self-contained | Can't use adaptive solvers, no global error control |
| **Simulator integrates globally** ✓ | Full solver compatibility, global error control | Slightly more complex setup |

> [!NOTE]
> **Exception: Algebraic State.** Some "state" is not integrated (e.g., lookup table outputs, mode flags). These can be updated directly by components since they're not part of the ODE system.

---

## 6. Solver Compatibility

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

---

## 7. Memory Layout Example

For a simulation with 3 components:

```
X_global_ layout:
┌─────────────────────────────────────────────────────────────────┐
│ Component A (3 states) │ Component B (6 states) │ Component C (4 states) │
│  [0]  [1]  [2]         │  [3]  [4]  [5]  [6]  [7]  [8]  │  [9] [10] [11] [12]  │
└─────────────────────────────────────────────────────────────────┘
         ▲                          ▲                          ▲
         │                          │                          │
    A.state_ = &X[0]           B.state_ = &X[3]           C.state_ = &X[9]
```

This contiguous layout is optimal for:
- Cache efficiency during integration
- SIMD vectorization
- Direct handoff to external solvers (scipy, CVODES)
- Checkpoint/restore (single memcpy)
