# Component Authoring Guide

**Related:** [02_component_protocol.md](../architecture/02_component_protocol.md) | [04_lifecycle.md](../architecture/04_lifecycle.md) | [05_execution_model.md](../architecture/05_execution_model.md)

---

This guide documents patterns and gotchas for implementing Icarus components.

## Quick Checklist

- [ ] Template on `Scalar` for dual-mode (numeric/symbolic) support
- [ ] Use `vulcan::` for physics, `janus::` for math dispatch
- [ ] Initialize output signals in `BindState()` (not just state arrays)
- [ ] Update output signals at **start** of `Step()`, before reading inputs
- [ ] Consider component execution order until topological sorting is implemented

---

## Lifecycle Responsibilities

| Phase | Component Must... |
|:------|:------------------|
| `Provision()` | Register outputs with Backplane |
| `Stage()` | Resolve input handles from Backplane |
| `BindState()` | Store state pointers AND initialize output signals |
| `Step()` | Update outputs first, then read inputs, then compute derivatives |

---

## Critical Pattern: Signal Initialization

Output signals must be initialized in `BindState()` so other components can read correct values during the first derivative evaluation.

```cpp
void BindState(Scalar* state, Scalar* state_dot, std::size_t size) override {
    // Store pointers
    state_pos_ = state;
    state_dot_pos_ = state_dot;
    
    // Apply ICs to state
    state_pos_[0] = ic_position_(0);
    state_pos_[1] = ic_position_(1);
    state_pos_[2] = ic_position_(2);
    
    // CRITICAL: Also initialize output signals!
    position_ = ic_position_;
}
```

> [!WARNING]
> If output signals are not initialized, other components will read zeros (or worse, NaN) during the first RK4 evaluation.

---

## Critical Pattern: Output Update Order in Step()

During multi-stage integrators (RK4, RK45), `Step()` is called multiple times with trial states. Output signals must be updated **before** reading inputs so upstream components see current values.

```cpp
void Step(Scalar t, Scalar dt) override {
    // 1. Read current state from state vector
    Vec3<Scalar> pos{state_pos_[0], state_pos_[1], state_pos_[2]};
    
    // 2. Update outputs FIRST
    position_ = pos;
    
    // 3. Now read inputs (they depend on our updated outputs)
    Vec3<Scalar> force = force_handle_.get();
    
    // 4. Compute derivatives
    state_dot_vel_[0] = force(0) / mass_;
    // ...
}
```

---

## Component Execution Order

> [!IMPORTANT]
> Until topological sorting is implemented, **component add order matters**.

Components execute in the order added to the Simulator. For correct data flow:

```cpp
// Gravity computes force from position
sim.AddComponent(std::move(gravity));  // Runs first

// Dynamics reads force, outputs position
sim.AddComponent(std::move(dynamics)); // Runs second
```

**Future:** When `scheduler: policy: TOPOLOGICAL` is implemented, execution order will be determined automatically from signal dependencies.

---

## Stateful vs Stateless Components

| Type | `StateSize()` | Has `BindState()` | Example |
|:-----|:--------------|:------------------|:--------|
| Stateful | > 0 | Yes | Dynamics, Integrator |
| Stateless | 0 | No | Gravity, Atmosphere, Sensor |

Stateless components compute outputs purely from inputs—they don't own integrated state.

---

## See Also

- [PointMass3DOF.hpp](../../components/dynamics/PointMass3DOF.hpp) - Reference stateful component
- [PointMassGravity.hpp](../../components/environment/PointMassGravity.hpp) - Reference stateless component
