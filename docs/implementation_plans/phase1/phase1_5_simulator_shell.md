# Phase 1.5: Simulator Shell Implementation Plan

**Status:** Complete
**Target:** Top-level Simulator class coordinating components and lifecycle

---

## Overview

Phase 1.5 establishes the `Simulator<Scalar>` class as the top-level coordinator for component lifecycle management. The Simulator owns the component list and signal backplane, orchestrating the Provision/Stage/Step sequence as specified in [04_lifecycle.md](../../architecture/04_lifecycle.md) and [05_execution_model.md](../../architecture/05_execution_model.md).

**Key Responsibilities:**

1. **Component Ownership** — Manage component list and execution order
2. **Backplane Ownership** — Own SignalRegistry and Backplane facade
3. **Lifecycle Orchestration** — Call Provision/Stage/Step in correct sequence
4. **Dependency Tracking** — Record inputs/outputs for future scheduler
5. **Time Management** — Track simulation time across steps

---

## Architecture Alignment

Per [04_lifecycle.md](../../architecture/04_lifecycle.md), the Simulator must enforce:

```
┌─────────────────────────────────────────────────────────────┐
│                     SIMULATOR LIFECYCLE                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   AddComponent()  ──►  Provision()  ──►  Stage()  ──►  Step()
│        │                   │               │            │
│        │                   │               │            │
│        ▼                   ▼               ▼            ▼
│   ┌─────────┐       ┌───────────┐   ┌─────────┐   ┌────────┐
│   │Component│  ──►  │Provision()│──►│ Stage() │──►│ Step() │
│   │  List   │       │  (once)   │   │(per run)│   │(per dt)│
│   └─────────┘       └───────────┘   └─────────┘   └────────┘
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Lifecycle Invariants:**

| Phase | Called | Purpose | Allocation | String Lookups |
|:------|:-------|:--------|:-----------|:---------------|
| Provision | Once at startup | Register signals, load config | ✅ Allowed | ✅ Allowed |
| Stage | Per run/episode | Wire inputs, apply ICs | ❌ Forbidden | ✅ Allowed |
| Step | Every dt (hot path) | Compute dynamics | ❌ Forbidden | ❌ Forbidden |

---

## Implementation Summary

### Core Class: `Simulator<Scalar>`

**File:** [`include/icarus/sim/Simulator.hpp`](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp)

```cpp
template <typename Scalar>
class Simulator {
public:
    Simulator();
    ~Simulator() = default;

    // Non-copyable, movable
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    Simulator(Simulator&&) = default;
    Simulator& operator=(Simulator&&) = default;

    // Component management
    void AddComponent(std::unique_ptr<Component<Scalar>> component);

    // Lifecycle
    void Provision();  // Call once at startup
    void Stage();      // Call per run/episode
    void Step(Scalar dt);  // Call per timestep

    // State queries
    Scalar Time() const;
    Phase GetPhase() const;
    std::size_t NumComponents() const;

    // Backplane/Registry access
    SignalRegistry<Scalar>& GetRegistry();
    const SignalRegistry<Scalar>& GetRegistry() const;
    Backplane<Scalar>& GetBackplane();
    const Backplane<Scalar>& GetBackplane() const;

    // Signal access (debugging/initialization)
    Scalar GetSignal(const std::string& name) const;
    void SetSignal(const std::string& name, const Scalar& value);

    // Dependency tracking (for scheduler)
    const std::vector<std::string>& GetComponentOutputs(Component<Scalar>* comp) const;
    const std::vector<std::string>& GetComponentInputs(Component<Scalar>* comp) const;

private:
    std::vector<std::unique_ptr<Component<Scalar>>> components_;
    SignalRegistry<Scalar> registry_;
    Backplane<Scalar> backplane_;
    Scalar time_{};
    Phase phase_ = Phase::Uninitialized;

    // Dependency tracking
    std::unordered_map<Component<Scalar>*, ComponentConfig> configs_;
    std::unordered_map<Component<Scalar>*, std::vector<std::string>> outputs_;
    std::unordered_map<Component<Scalar>*, std::vector<std::string>> inputs_;
};
```

---

## Detailed Method Specifications

### 1. `AddComponent()`

**Purpose:** Add a component to the simulation before Provision.

**Contract:**
- Must be called before `Provision()`
- Components are stored in insertion order (execution order)
- Takes ownership via `std::unique_ptr`

**Implementation:**
```cpp
void AddComponent(std::unique_ptr<Component<Scalar>> component) {
    components_.push_back(std::move(component));
}
```

---

### 2. `Provision()`

**Purpose:** Initialize all components, register signals, allocate memory.

**Contract:**
- Called exactly once
- Throws `LifecycleError` if called again
- Sets context for each component before calling
- Tracks registered outputs per component

**Sequence:**
```
for each component in components_:
    1. backplane_.set_context(entity, name)
    2. backplane_.clear_tracking()
    3. Create ComponentConfig
    4. comp->Provision(backplane_, config)
    5. comp->MarkProvisioned()
    6. outputs_[comp] = backplane_.registered_outputs()
    7. backplane_.clear_context()
phase_ = Phase::Provisioned
```

**Implementation:**
```cpp
void Provision() {
    if (phase_ != Phase::Uninitialized) {
        throw LifecycleError("Provision() can only be called once");
    }
    for (auto& comp : components_) {
        backplane_.set_context(comp->Entity(), comp->Name());
        backplane_.clear_tracking();

        ComponentConfig config;
        config.name = comp->Name();
        config.entity = comp->Entity();
        configs_[comp.get()] = config;

        comp->Provision(backplane_, config);
        comp->MarkProvisioned();

        outputs_[comp.get()] = backplane_.registered_outputs();
        backplane_.clear_context();
    }
    phase_ = Phase::Provisioned;
}
```

---

### 3. `Stage()`

**Purpose:** Wire inputs, apply initial conditions, prepare for simulation.

**Contract:**
- Requires prior `Provision()`
- Can be called multiple times (per run/episode)
- Resets time to zero
- Tracks resolved inputs per component

**Sequence:**
```
for each component in components_:
    1. backplane_.set_context(entity, name)
    2. backplane_.clear_tracking()
    3. comp->Stage(backplane_, configs_[comp])
    4. comp->MarkStaged()
    5. inputs_[comp] = backplane_.resolved_inputs()
    6. backplane_.clear_context()
phase_ = Phase::Staged
time_ = 0
```

**Implementation:**
```cpp
void Stage() {
    if (phase_ != Phase::Provisioned) {
        throw LifecycleError("Stage() requires prior Provision()");
    }
    for (auto& comp : components_) {
        backplane_.set_context(comp->Entity(), comp->Name());
        backplane_.clear_tracking();

        comp->Stage(backplane_, configs_[comp.get()]);
        comp->MarkStaged();

        inputs_[comp.get()] = backplane_.resolved_inputs();
        backplane_.clear_context();
    }
    phase_ = Phase::Staged;
    time_ = Scalar{0};
}
```

---

### 4. `Step()`

**Purpose:** Execute one timestep of the simulation.

**Contract:**
- Requires prior `Stage()`
- Calls PreStep → Step → PostStep on all components
- Advances time by dt
- Sets phase to Running

**Sequence:**
```
phase_ = Phase::Running
for each component: comp->PreStep(time_, dt)
for each component: comp->Step(time_, dt)
for each component: comp->PostStep(time_, dt)
time_ = time_ + dt
```

**Implementation:**
```cpp
void Step(Scalar dt) {
    phase_ = Phase::Running;
    for (auto& comp : components_) {
        comp->PreStep(time_, dt);
    }
    for (auto& comp : components_) {
        comp->Step(time_, dt);
    }
    for (auto& comp : components_) {
        comp->PostStep(time_, dt);
    }
    time_ = time_ + dt;
}
```

---

### 5. Signal Access Methods

**Purpose:** Read/write signals by name for debugging and initialization.

**Note:** These are slow-path methods, NOT for hot-path simulation code.

```cpp
Scalar GetSignal(const std::string& name) const {
    return registry_.GetByName(name);
}

void SetSignal(const std::string& name, const Scalar& value) {
    registry_.SetByName(name, value);
}
```

---

### 6. Dependency Tracking Methods

**Purpose:** Enable future scheduler to build execution order.

```cpp
const std::vector<std::string>& GetComponentOutputs(Component<Scalar>* comp) const {
    static const std::vector<std::string> empty;
    auto it = outputs_.find(comp);
    return (it != outputs_.end()) ? it->second : empty;
}

const std::vector<std::string>& GetComponentInputs(Component<Scalar>* comp) const {
    static const std::vector<std::string> empty;
    auto it = inputs_.find(comp);
    return (it != inputs_.end()) ? it->second : empty;
}
```

---

## Phase Transition Diagram

```
                 ┌──────────────┐
                 │ Uninitialized│
                 └──────┬───────┘
                        │
                   AddComponent()
                        │
                        ▼
                 ┌──────────────┐
                 │ Uninitialized│ (with components)
                 └──────┬───────┘
                        │
                   Provision()
                        │
                        ▼
                 ┌──────────────┐
                 │  Provisioned │
                 └──────┬───────┘
                        │
                     Stage()
                        │
                        ▼
                 ┌──────────────┐
                 │    Staged    │◄────────────┐
                 └──────┬───────┘             │
                        │                     │
                    Step(dt)              Stage()
                        │                 (re-run)
                        ▼                     │
                 ┌──────────────┐             │
                 │   Running    │─────────────┘
                 └──────────────┘
                        │
                    Step(dt)
                        │
                        ▼
                 ┌──────────────┐
                 │   Running    │ (repeat)
                 └──────────────┘
```

---

## Integration with Backplane

The Simulator uses the `Backplane<Scalar>` facade to:

1. **Set Component Context** — Before each lifecycle call:
   ```cpp
   backplane_.set_context(comp->Entity(), comp->Name());
   ```

2. **Track Dependencies** — After each lifecycle call:
   ```cpp
   outputs_[comp] = backplane_.registered_outputs();  // After Provision
   inputs_[comp] = backplane_.resolved_inputs();      // After Stage
   ```

3. **Clear Between Components**:
   ```cpp
   backplane_.clear_tracking();  // Before
   backplane_.clear_context();   // After
   ```

This enables automatic full-name generation (`Entity.Component.Signal`) and dependency tracking for future scheduler implementation.

---

## Error Handling

The Simulator enforces lifecycle invariants with exceptions:

| Error | Condition | Message |
|:------|:----------|:--------|
| `LifecycleError` | Provision() called twice | "Provision() can only be called once" |
| `LifecycleError` | Stage() before Provision() | "Stage() requires prior Provision()" |

**Future enhancements (Phase 2+):**
- Step() without Stage() → `LifecycleError`
- Component exceptions → `SimulationError` wrapping

---

## File Inventory

| File | Purpose | Status |
|:-----|:--------|:-------|
| [`include/icarus/sim/Simulator.hpp`](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp) | Simulator class | ✅ Complete |
| [`src/simulator/Simulator.cpp`](file:///home/tanged/sources/icarus/src/simulator/Simulator.cpp) | Template instantiation | ✅ Complete |
| [`include/icarus/signal/Backplane.hpp`](file:///home/tanged/sources/icarus/include/icarus/signal/Backplane.hpp) | Backplane facade | ✅ Complete (Phase 1.4) |
| [`include/icarus/core/Component.hpp`](file:///home/tanged/sources/icarus/include/icarus/core/Component.hpp) | Component base | ✅ Complete (Phase 1.4) |
| [`include/icarus/core/Types.hpp`](file:///home/tanged/sources/icarus/include/icarus/core/Types.hpp) | Phase enum, configs | ✅ Complete (Phase 1.2) |

---

## Test Coverage

### Existing Tests (All Passing)

From [`tests/core/test_component.cpp`](file:///home/tanged/sources/icarus/tests/core/test_component.cpp):

| Test Category | Test Name | Purpose |
|:--------------|:----------|:--------|
| **Simulator** | `Simulator_AddComponent` | Verify component addition |
| **Simulator** | `Simulator_Provision` | Verify Provision sequence |
| **Simulator** | `Simulator_Stage` | Verify Stage sequence |
| **Simulator** | `Simulator_Step` | Verify Step loop and time advancement |
| **Simulator** | `Simulator_MultipleComponents` | Multiple component orchestration |
| **Simulator** | `Simulator_EntityNamespacing` | Entity.Component.Signal naming |
| **Symbolic** | `SimulatorSymbolic_FullLifecycle` | Full lifecycle with `casadi::MX` |

### Test Verification Commands

```bash
# Run all tests
./scripts/test.sh

# Run component/simulator tests specifically
cd build && ctest -R component --output-on-failure

# Run symbolic tests
cd build && ctest -R Symbolic --output-on-failure
```

**Current Status:** 62/62 tests passing (100%)

---

## Implementation Checklist

### Simulator Class

- [x] Create `Simulator<Scalar>` template class
- [x] Own `SignalRegistry<Scalar>` member
- [x] Own `Backplane<Scalar>` member wrapping registry
- [x] Own `std::vector<std::unique_ptr<Component<Scalar>>>` for components
- [x] Track `Phase` state (Uninitialized → Provisioned → Staged → Running)
- [x] Track simulation `time_`

### Component Management

- [x] Implement `AddComponent()` taking ownership
- [x] Store components in insertion order

### Lifecycle Methods

- [x] Implement `Provision()` with lifecycle enforcement
- [x] Implement `Stage()` with lifecycle enforcement
- [x] Implement `Step()` with PreStep/Step/PostStep sequence
- [x] Set backplane context before each component call
- [x] Clear backplane context after each component call
- [x] Call `MarkProvisioned()`/`MarkStaged()` on components

### Dependency Tracking

- [x] Store `ComponentConfig` per component
- [x] Track `outputs_` per component from Provision
- [x] Track `inputs_` per component from Stage
- [x] Implement `GetComponentOutputs()`
- [x] Implement `GetComponentInputs()`

### Accessors

- [x] Implement `Time()` getter
- [x] Implement `GetPhase()` getter
- [x] Implement `NumComponents()` getter
- [x] Implement `GetRegistry()` (const and non-const)
- [x] Implement `GetBackplane()` (const and non-const)

### Signal Access

- [x] Implement `GetSignal()` by name
- [x] Implement `SetSignal()` by name

### Error Handling

- [x] Throw `LifecycleError` for invalid transitions
- [x] Include meaningful error messages

### Template Instantiation

- [x] Explicit instantiation for `double`
- [x] Explicit instantiation for `casadi::MX`

---

## Design Decisions

### 1. Single Backplane Instance

**Decision:** Simulator owns one `Backplane<Scalar>` wrapping one `SignalRegistry<Scalar>`.

**Rationale:** All components share the same signal namespace, enabling cross-component communication via well-defined signals.

### 2. Insertion Order Execution

**Decision:** Components execute in the order they were added via `AddComponent()`.

**Rationale:** Simple and predictable. Phase 5 will add topological sorting via Scheduler.

### 3. Context Management Per Component

**Decision:** Set/clear backplane context around each lifecycle call.

**Rationale:** Enables automatic full-name generation without component author burden.

### 4. Dependency Tracking via Backplane

**Decision:** Record registered_outputs/resolved_inputs from Backplane after each lifecycle call.

**Rationale:** Enables future scheduler to build dependency graph without requiring component changes.

### 5. Time as Template Type

**Decision:** `time_` is `Scalar` type, not `double`.

**Rationale:** Supports symbolic mode where time may be a symbolic variable.

---

## Future Enhancements (Not in Phase 1.5)

| Enhancement | Target Phase | Description |
|:------------|:-------------|:------------|
| Scheduler | Phase 5 | Topological sort based on dependency graph |
| Rate Groups | Phase 5 | Multi-rate component execution |
| Shutdown | Phase 6 | Graceful shutdown sequence |
| Phase Manager | Phase 6 | Flight phase transitions |
| Warmstart | Phase 6 | Resume from recording |
| State Management | Phase 2 | Global state vectors, integrator |

---

## Exit Criteria Validation

Per the main implementation plan:

| Criterion | Status | Evidence |
|:----------|:-------|:---------|
| Can instantiate `Simulator<double>`, add dummy component | ✅ | `Simulator_AddComponent` test |
| Call Provision/Stage/Step without crashes | ✅ | `Simulator_Provision/Stage/Step` tests |
| Read/write signals via Backplane | ✅ | `Backplane_*` tests |
| All Phase 1 tests pass | ✅ | 62/62 tests passing |

---

## References

| Topic | Document |
|:------|:---------|
| Lifecycle phases | [04_lifecycle.md](../../architecture/04_lifecycle.md) |
| Execution model | [05_execution_model.md](../../architecture/05_execution_model.md) |
| Component protocol | [02_component_protocol.md](../../architecture/02_component_protocol.md) |
| Signal backplane | [03_signal_backplane.md](../../architecture/03_signal_backplane.md) |
| Core philosophy | [01_core_philosophy.md](../../architecture/01_core_philosophy.md) |

---

## Appendix: Usage Example

```cpp
#include <icarus/icarus.hpp>
#include <icarus/testing/DummyComponent.hpp>

int main() {
    using namespace icarus;

    // Create simulator
    Simulator<double> sim;

    // Add components
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Counter", "Test"));
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Timer", "Test"));

    // Initialize
    sim.Provision();  // Register signals
    sim.Stage();      // Wire inputs, reset to t=0

    // Run simulation
    for (int i = 0; i < 100; ++i) {
        sim.Step(0.01);  // 10ms timestep
    }

    // Query results
    std::cout << "Final time: " << sim.Time() << std::endl;
    std::cout << "Counter: " << sim.GetSignal("Test.Counter.counter") << std::endl;

    return 0;
}
```

**Output:**
```
Final time: 1.0
Counter: 100
```
