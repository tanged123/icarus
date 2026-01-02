# Unified Signal Model: States as Signals

**Status:** Proposed  
**Phase:** 6.1 Refactor  
**Related:** [17_events_phases.md](../../architecture/17_events_phases.md) | [09_memory_state_ownership.md](../../architecture/09_memory_state_ownership.md)

---

## Overview

This document proposes unifying **states** with the **signal backplane**. Currently, states are managed separately by `StateManager` while outputs are managed by `Backplane`. This creates duplication and boilerplate in components.

### Current Problems

1. **Dual registration**: Components register outputs with Backplane AND declare `StateSize()` for StateManager
2. **Manual sync**: Components must manually copy state values to output signals (`PublishOutputs()`)
3. **Raw offset math**: Components manage `state_[kPosOffset + 0]` etc.
4. **Phase gating complexity**: Would require separate gating for outputs vs. derivative zeroing

### Proposed Solution

Register states directly with the Backplane as signal pairs (value + derivative). The `StateManager` queries the Backplane for integrable signals instead of scanning `Component::StateSize()`.

---

## Design

### New Registration API

```cpp
// In Backplane
template <typename T>
void register_state(const std::string& name, T* value, T* derivative,
                    const std::string& unit = "", const std::string& desc = "");

template <typename S>
void register_state_vec3(const std::string& name, Vec3<S>* value, Vec3<S>* derivative,
                         const std::string& unit = "", const std::string& desc = "");

template <typename S>
void register_state_quat(const std::string& name, janus::Quaternion<S>* value,
                         janus::Quaternion<S>* derivative,
                         const std::string& unit = "", const std::string& desc = "");
```

### SignalDescriptor Extension

```cpp
struct SignalDescriptor {
    // ... existing fields ...
    
    // State metadata (new)
    bool is_integrable = false;             // True for state values
    std::string derivative_signal;          // Name of paired derivative signal
    std::string integrated_signal;          // For derivatives: name of value signal
};
```

### StateManager Refactor

`StateManager` becomes a thin integration coordinator:

```cpp
template <typename Scalar>
class StateManager {
public:
    // Query backplane for all integrable signals, build integration vector
    void DiscoverStates(const Backplane<Scalar>& bp);
    
    // Get/set integration vectors (built from discovered state pointers)
    JanusVector<Scalar> GetState() const;
    JanusVector<Scalar> GetDerivatives() const;
    void SetState(const JanusVector<Scalar>& X);
    void ZeroDerivatives();
    
private:
    struct StateBinding {
        std::string name;
        Scalar* value_ptr;
        Scalar* derivative_ptr;
    };
    std::vector<StateBinding> bindings_;
};
```

### Phase Gating Architecture

With unified signals, gating applies at two levels:

1. **Component-level gating**: Scheduler skips `Step()` for inactive components
2. **Route-level gating**: Signal routes can be gated (deliver zero when inactive)

For states, derivative signals are just signals - gating a derivative route zeros the derivative, freezing the state.

```yaml
components:
  Booster:
    active_phases: [BOOST]  # Component-level

wiring:
  - from: Booster.fuel_mass_dot
    to: __integrator__      # Special sink
    active_phases: [BOOST]  # Route-level gating
```

---

## Component Simplification

### Before (RigidBody6DOF)

```cpp
class RigidBody6DOF : public Component<Scalar> {
    static constexpr std::size_t kStateSize = 13;
    static constexpr std::size_t kPosOffset = 0;
    // ... 4 more offset constants ...
    
    Scalar* state_ = nullptr;
    Scalar* state_dot_ = nullptr;
    
    // State duplicated as output signals
    Vec3<Scalar> position_;
    Vec3<Scalar> velocity_body_;
    // ... etc ...
    
    std::size_t StateSize() const override { return kStateSize; }
    void BindState(Scalar* s, Scalar* sd, std::size_t sz) override { ... }
    void PublishOutputs() { /* manual copy state_ -> outputs */ }
};
```

### After

```cpp
class RigidBody6DOF : public Component<Scalar> {
    // State IS the output - no duplication
    Vec3<Scalar> position_, position_dot_;
    Vec3<Scalar> velocity_body_, velocity_dot_;
    Vec4<Scalar> attitude_, attitude_dot_;  // Quaternion as Vec4
    Vec3<Scalar> omega_body_, omega_dot_;
    
    void Provision(Backplane<Scalar>& bp) override {
        bp.register_state_vec3("position", &position_, &position_dot_, "m");
        bp.register_state_vec3("velocity_body", &velocity_body_, &velocity_dot_, "m/s");
        bp.register_state_quat("attitude", &attitude_, &attitude_dot_);
        bp.register_state_vec3("omega_body", &omega_body_, &omega_dot_, "rad/s");
        // ... inputs as before ...
    }
    
    // No StateSize(), no BindState(), no PublishOutputs()!
};
```

**Estimated reduction**: ~150 lines per stateful component.

---

## Proposed Changes

### Signal Infrastructure

#### [MODIFY] [Signal.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Signal.hpp)

- Add `is_integrable`, `derivative_signal`, `integrated_signal` fields to `SignalDescriptor`

---

#### [MODIFY] [Registry.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Registry.hpp)

- Add `register_state<T>()`, `register_state_vec3<S>()`, `register_state_quat<S>()` methods
- Add `get_integrable_signals()` query method
- Track state/derivative pairs internally

---

#### [MODIFY] [Backplane.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Backplane.hpp)

- Forward state registration methods to Registry
- Ensure context (entity.component) applied to state names

---

### State Management

#### [MODIFY] [StateManager.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/StateManager.hpp)

- Remove `AllocateState(components)` that scans `StateSize()`
- Add `DiscoverStates(backplane)` that queries for integrable signals
- Build internal binding vector from discovered states
- Keep `GetState()`, `SetState()`, `ZeroDerivatives()` - same interface

---

#### [MODIFY] [Component.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Component.hpp)

- Remove `StateSize()` virtual method (or make optional/deprecated)
- Remove `BindState()` virtual method (or make optional/deprecated)
- Keep lifecycle methods unchanged

---

#### [MODIFY] [Simulator.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp)

- Call `state_manager_.DiscoverStates(backplane_)` after Provision
- Remove old `AllocateAndBindState()` logic
- Integration loop unchanged (uses same `GetState()`/`SetState()` interface)

---

### Component Updates

#### [MODIFY] [RigidBody6DOF.hpp](file:///home/tanged/sources/icarus/components/dynamics/RigidBody6DOF.hpp)

- Replace `state_[]` / `state_dot_[]` arrays with Vec3/Quat member variables
- Use `register_state_vec3()` / `register_state_quat()` in Provision
- Remove `StateSize()`, `BindState()`, offset constants
- Remove `PublishOutputs()` / `ApplyInitialConditions()` sync code
- Initial conditions set directly on member variables in `Stage()`

---

#### [MODIFY] [PointMass3DOF.hpp](file:///home/tanged/sources/icarus/components/dynamics/PointMass3DOF.hpp)

- Same pattern as RigidBody6DOF
- Simpler: only position (3) and velocity (3) states

---

## Verification Plan

### Automated Tests

#### Existing Tests (must pass)

```bash
./scripts/test.sh
```

All existing tests in:

- `tests/signal/test_signal.cpp` - Signal registration and wiring
- `tests/sim/test_state_management.cpp` - State management
- `tests/components/test_point_mass_3dof.cpp` - PointMass component
- `tests/components/test_rigid_body_6dof.cpp` - RigidBody component

#### New Tests

1. **`tests/signal/test_state_registration.cpp`** - New file
   - `StateRegistration_ScalarState` - Register scalar state/derivative pair
   - `StateRegistration_Vec3State` - Register Vec3 state with 6 signals created
   - `StateRegistration_QuatState` - Register quaternion state with 8 signals created
   - `StateRegistration_QueryIntegrable` - Query returns only integrable signals
   - `StateRegistration_DerivativePairing` - Verify derivative/value linking

2. **`tests/sim/test_unified_state.cpp`** - New file
   - `UnifiedState_DiscoverFromBackplane` - StateManager discovers registered states
   - `UnifiedState_GetSetState` - Round-trip through integration vector
   - `UnifiedState_ZeroDerivatives` - All derivatives zeroed
   - `UnifiedState_IntegrationLoop` - Full RK4 step with unified states

Run new tests:

```bash
./scripts/test.sh
ctest --test-dir build -R "StateRegistration|UnifiedState" -VV
```

### Integration Verification

1. Run existing demos to verify no regression:

   ```bash
   ./scripts/run_examples.sh
   ```

2. Verify tumbling body example produces same results:

   ```bash
   ./build/examples/multi_body_demo
   ```

### Manual Verification

> [!NOTE]
> User should verify that refactored components produce identical numerical results to pre-refactor versions. Run a known simulation and compare trajectories.

---

## Migration Path

1. **Phase A**: Add new registration API (backward compatible)
   - Components can use either old or new API
   - StateManager supports both discovery modes

2. **Phase B**: Migrate components to new API
   - Update RigidBody6DOF, PointMass3DOF
   - Verify tests pass

3. **Phase C**: Deprecate old API
   - Mark `StateSize()`, `BindState()` as deprecated
   - Log warnings if used

4. **Phase D**: Remove old API (future)
   - Clean break after transition period

---

## Open Questions

1. **Quaternion normalization**: Should the integrator auto-normalize quaternions? Currently done in component.

2. **State ordering**: Does integration order matter for coupled states? Current ordering is component registration order.

3. **Symbolic graph implications**: Need to verify state discovery works correctly for `Simulator<MX>`.
