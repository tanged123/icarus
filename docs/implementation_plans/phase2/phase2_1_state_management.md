# Phase 2.1: State Management Implementation Plan

**Status:** Proposed
**Target:** Global state vectors with pointer-based scatter/gather for integrator compatibility

---

## Overview

Phase 2.1 establishes the global state vector infrastructure that enables integrators (RK4, CVODES, scipy) to advance the simulation. The architecture uses a **pointer-based scatter/gather pattern** where:

1. **Simulator owns** `X_global_` and `X_dot_global_` contiguous vectors
2. **Components hold pointers** into these vectors (views, not copies)
3. **Stage binds** component pointers to specific offsets
4. **Integrator advances** `X_global_` using `X_dot_global_` computed by components

This decoupling allows external solvers to manage integration while preserving component locality.

---

## Architecture Alignment

Per [09_memory_state_ownership.md](../../architecture/09_memory_state_ownership.md):

```
┌─────────────────────────────────────────────────────────────┐
│                    STATE VECTOR OWNERSHIP                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   Simulator owns:                                           │
│   ┌───────────────────────────────────────────────────┐    │
│   │ X_global_     [═══════════════════════════════]   │    │
│   │ X_dot_global_ [═══════════════════════════════]   │    │
│   └───────────────────────────────────────────────────┘    │
│                     ▲       ▲       ▲                       │
│   Component A ──────┘       │       │                       │
│   Component B ──────────────┘       │                       │
│   Component C ──────────────────────┘                       │
│                                                             │
│   Components hold POINTERS (views) into global vectors      │
│   Components ONLY WRITE to X_dot_global_ during Step()      │
│   Integrator OWNS state advancement (X_global_ updates)     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### State Ownership Model

| Concept | Owner | Lifetime | Constraints |
|:--------|:------|:---------|:------------|
| `X_global_` | Simulator | Provision → Destroy | Authoritative; never modified by components |
| `X_dot_global_` | Simulator | Provision → Destroy | Populated by components during Step() |
| Component `state_` pointers | Component | Stage → Reset | Views into `X_global_` only |
| Component `state_dot_` pointers | Component | Stage → Reset | Views into `X_dot_global_` only |

### Integration Flow

```
┌─────────────────────────────────────────┐
│  Integrator (RK4/CVODES)               │
│  X_new = X + dt * f(X_dot)             │
└─────────────────────────────────────────┘
                    │
        ┌───────────┼───────────┐
        ▼           ▼           ▼
   Component A  Component B  Component C
   state_ ──────► X_global_[0:3]
   state_dot_ ───► X_dot_[0:3]

Step 1: Integrator calls Simulator.ComputeDerivatives(t, X)
Step 2: Components already have pointers to X, X_dot (automatic)
Step 3: Each Component.Step() reads *state_, writes *state_dot_
Step 4: Integrator reads X_dot_global_, computes X_new
```

---

## Current State Analysis

### What Exists (Phase 1 Complete)

| File | Contents | Status |
|:-----|:---------|:-------|
| [`Component.hpp`](file:///home/tanged/sources/icarus/include/icarus/core/Component.hpp) | `StateSize()` virtual method | ✅ Complete |
| [`Types.hpp`](file:///home/tanged/sources/icarus/include/icarus/core/Types.hpp) | `JanusVector<Scalar>` typedef | ✅ Available |
| [`Simulator.hpp`](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp) | Lifecycle orchestration | ✅ Complete |
| [`Backplane.hpp`](file:///home/tanged/sources/icarus/include/icarus/signal/Backplane.hpp) | Signal registration/resolution | ✅ Complete |

### What's Missing (Phase 2.1 Scope)

| Feature | Description | Status |
|:--------|:------------|:-------|
| Global state vectors | `X_global_`, `X_dot_global_` in Simulator | ❌ Missing |
| `BindState()` method | Component virtual for pointer binding | ❌ Missing |
| State binding logic | Simulator::Stage() calls BindState() | ❌ Missing |
| `ComputeDerivatives()` | Integrator-callable derivative evaluation | ❌ Missing |
| State layout metadata | Offset/size tracking per component | ❌ Missing |
| State accessors | GetState(), SetState() for integrator | ❌ Missing |

---

## Proposed Implementation

### 1. State Infrastructure in Simulator

#### [MODIFY] `include/icarus/sim/Simulator.hpp`

Add global state vectors and metadata:

```cpp
#pragma once

#include <icarus/core/Component.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <memory>
#include <vector>

namespace icarus {

/**
 * @brief Metadata for a component's state slice
 */
template <typename Scalar>
struct StateSlice {
    Component<Scalar>* owner;   ///< Component that owns this slice
    std::size_t offset;         ///< Starting index in X_global_
    std::size_t size;           ///< Number of state variables
};

template <typename Scalar>
class Simulator {
public:
    // ... existing interface ...

    // =========================================================================
    // State Management (Phase 2.1)
    // =========================================================================

    /**
     * @brief Get total state vector size
     *
     * Sum of StateSize() across all components.
     */
    [[nodiscard]] std::size_t GetTotalStateSize() const {
        std::size_t total = 0;
        for (const auto& comp : components_) {
            total += comp->StateSize();
        }
        return total;
    }

    /**
     * @brief Get current state vector
     *
     * Returns a copy of X_global_ (for integrator use).
     */
    [[nodiscard]] JanusVector<Scalar> GetState() const {
        return X_global_;
    }

    /**
     * @brief Set state vector
     *
     * Copies values into X_global_. Called by integrator after advancing.
     */
    void SetState(const JanusVector<Scalar>& X) {
        if (X.size() != X_global_.size()) {
            throw StateError("State size mismatch: expected " +
                           std::to_string(X_global_.size()) +
                           ", got " + std::to_string(X.size()));
        }
        X_global_ = X;
    }

    /**
     * @brief Compute derivatives for current state
     *
     * Called by integrator. Components read from X_global_ (via pointers)
     * and write to X_dot_global_ (via pointers).
     *
     * @param t Current time
     * @return Derivative vector X_dot_global_
     */
    const JanusVector<Scalar>& ComputeDerivatives(Scalar t) {
        // Zero derivatives (components accumulate into them)
        for (std::size_t i = 0; i < X_dot_global_.size(); ++i) {
            X_dot_global_[i] = Scalar{0};
        }

        // All components compute derivatives
        // (pointers already bound, scatter/gather automatic)
        Scalar dt = dt_nominal_;
        for (auto& comp : components_) {
            comp->PreStep(t, dt);
        }
        for (auto& comp : components_) {
            comp->Step(t, dt);
        }
        for (auto& comp : components_) {
            comp->PostStep(t, dt);
        }

        return X_dot_global_;
    }

    /**
     * @brief Get derivative vector (after ComputeDerivatives)
     */
    [[nodiscard]] const JanusVector<Scalar>& GetDerivatives() const {
        return X_dot_global_;
    }

    /**
     * @brief Get state layout metadata
     *
     * Useful for debugging and introspection.
     */
    [[nodiscard]] const std::vector<StateSlice<Scalar>>& GetStateLayout() const {
        return state_layout_;
    }

    /**
     * @brief Set nominal timestep for derivative computation
     */
    void SetNominalDt(Scalar dt) { dt_nominal_ = dt; }

    /**
     * @brief Get nominal timestep
     */
    [[nodiscard]] Scalar GetNominalDt() const { return dt_nominal_; }

private:
    // ... existing members ...

    // State management (Phase 2.1)
    JanusVector<Scalar> X_global_;               ///< Global state vector
    JanusVector<Scalar> X_dot_global_;           ///< Global derivative vector
    std::vector<StateSlice<Scalar>> state_layout_; ///< Per-component metadata
    Scalar dt_nominal_{0.01};                    ///< Nominal timestep
};

} // namespace icarus
```

---

### 2. State Binding in Component

#### [MODIFY] `include/icarus/core/Component.hpp`

Add `BindState()` virtual method:

```cpp
template <typename Scalar>
class Component {
public:
    // ... existing interface ...

    // =========================================================================
    // State Management (Phase 2.1)
    // =========================================================================

    /**
     * @brief Bind component to slices of global state vectors
     *
     * Called by Simulator during Stage(). Component stores pointers
     * into X_global_ and X_dot_global_ for efficient access during Step().
     *
     * Default implementation does nothing (component has no state).
     * Override if StateSize() > 0.
     *
     * @param state_ptr Pointer to X_global_[offset] for this component
     * @param state_dot_ptr Pointer to X_dot_global_[offset] for this component
     * @param state_size Number of state variables (matches StateSize())
     */
    virtual void BindState(Scalar* state_ptr,
                          Scalar* state_dot_ptr,
                          std::size_t state_size) {
        // Default: no state binding (component has no integrated state)
        (void)state_ptr;
        (void)state_dot_ptr;
        (void)state_size;
    }

    /**
     * @brief Check if component has integrated state
     */
    [[nodiscard]] bool HasState() const { return StateSize() > 0; }
};
```

---

### 3. State Binding Logic in Simulator::Stage()

#### [MODIFY] `include/icarus/sim/Simulator.hpp`

Update `Stage()` to allocate vectors and bind state:

```cpp
void Stage() {
    if (phase_ != Phase::Provisioned) {
        throw LifecycleError("Stage() requires prior Provision()");
    }

    // =========================================================================
    // Phase 2.1: Allocate global state vectors
    // =========================================================================
    std::size_t total_state_size = GetTotalStateSize();
    X_global_.resize(total_state_size);
    X_dot_global_.resize(total_state_size);
    state_layout_.clear();

    // Initialize to zero
    for (std::size_t i = 0; i < total_state_size; ++i) {
        X_global_[i] = Scalar{0};
        X_dot_global_[i] = Scalar{0};
    }

    // Bind each component to its slice
    std::size_t offset = 0;
    for (auto& comp : components_) {
        std::size_t state_size = comp->StateSize();

        if (state_size > 0) {
            comp->BindState(
                X_global_.data() + offset,
                X_dot_global_.data() + offset,
                state_size
            );
            state_layout_.push_back({comp.get(), offset, state_size});
            offset += state_size;
        }
    }

    // =========================================================================
    // Existing Stage logic (signal wiring)
    // =========================================================================
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

### 4. Error Types for State Management

#### [MODIFY] `include/icarus/core/Error.hpp`

Add state-related exceptions:

```cpp
namespace icarus {

// ... existing exceptions ...

/**
 * @brief Base class for state-related errors
 */
class StateError : public SimulationError {
public:
    using SimulationError::SimulationError;
};

/**
 * @brief Thrown when state size doesn't match expected
 */
class StateSizeMismatchError : public StateError {
public:
    StateSizeMismatchError(std::size_t expected, std::size_t actual)
        : StateError("State size mismatch: expected " +
                    std::to_string(expected) + ", got " +
                    std::to_string(actual)),
          expected_(expected), actual_(actual) {}

    [[nodiscard]] std::size_t expected() const { return expected_; }
    [[nodiscard]] std::size_t actual() const { return actual_; }

private:
    std::size_t expected_;
    std::size_t actual_;
};

/**
 * @brief Thrown when state binding fails
 */
class StateBindingError : public StateError {
public:
    using StateError::StateError;
};

} // namespace icarus
```

---

## Memory Layout Example

With 3 components (PointMass: 6, Aerodynamics: 4, Engine: 2):

```
X_global_ layout (total size: 12)
┌─────────────────────────────────────────────────────────────┐
│ PointMass3DOF (6)           │ Aerodynamics (4) │ Engine (2) │
│ [x][y][z][vx][vy][vz]       │ [α][β][p][q]     │ [ω₁][ω₂]  │
│  0  1  2  3   4   5         │  6  7  8  9      │ 10  11     │
└─────────────────────────────────────────────────────────────┘
         ▲                              ▲                ▲
         │                              │                │
   state_pos_ ──► X[0:3]        state_alpha_       state_spool_
   state_vel_ ──► X[3:6]        state_beta_

state_layout_ metadata:
  [0] = {owner: PointMass,   offset: 0,  size: 6}
  [1] = {owner: Aerodynamics, offset: 6,  size: 4}
  [2] = {owner: Engine,      offset: 10, size: 2}

Checkpoint: memcpy(backup, X_global_.data(), 12 * sizeof(Scalar))
```

---

## Verification Tests

### Test File: `tests/sim/test_state_management.cpp`

```cpp
#include <gtest/gtest.h>
#include <icarus/icarus.hpp>
#include <icarus/testing/DummyComponent.hpp>

using namespace icarus;

// ---------------------------------------------------------------------------
// Test Component with State
// ---------------------------------------------------------------------------

template <typename Scalar>
class StatefulComponent : public Component<Scalar> {
public:
    explicit StatefulComponent(std::string name, std::size_t state_size)
        : name_(std::move(name)), state_size_(state_size) {}

    std::string Name() const override { return name_; }
    std::size_t StateSize() const override { return state_size_; }

    void Provision(Backplane<Scalar>& bp, const ComponentConfig&) override {
        // Register outputs for observability
        bp.register_output("state_bound", &state_bound_);
    }

    void Stage(Backplane<Scalar>&, const ComponentConfig&) override {
        // Nothing beyond state binding
    }

    void BindState(Scalar* state, Scalar* state_dot, std::size_t size) override {
        if (size != state_size_) {
            throw StateSizeMismatchError(state_size_, size);
        }
        state_ptr_ = state;
        state_dot_ptr_ = state_dot;
        state_bound_ = Scalar{1};
    }

    void Step(Scalar t, Scalar dt) override {
        // Simple dynamics: x_dot = -x (decay)
        for (std::size_t i = 0; i < state_size_; ++i) {
            state_dot_ptr_[i] = -state_ptr_[i];
        }
    }

    Scalar* GetStatePtr() { return state_ptr_; }
    Scalar* GetStateDotPtr() { return state_dot_ptr_; }

private:
    std::string name_;
    std::size_t state_size_;
    Scalar* state_ptr_ = nullptr;
    Scalar* state_dot_ptr_ = nullptr;
    Scalar state_bound_{0};
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(StateManagement, TotalStateSizeCalculation) {
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 3));
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("B", 5));
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("C", 2));

    EXPECT_EQ(sim.GetTotalStateSize(), 10);
}

TEST(StateManagement, StateVectorAllocation) {
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 6));
    sim.Provision();
    sim.Stage();

    auto state = sim.GetState();
    EXPECT_EQ(state.size(), 6);
}

TEST(StateManagement, StateBindingCorrectOffsets) {
    Simulator<double> sim;

    auto compA = std::make_unique<StatefulComponent<double>>("A", 3);
    auto compB = std::make_unique<StatefulComponent<double>>("B", 4);
    auto* ptrA = compA.get();
    auto* ptrB = compB.get();

    sim.AddComponent(std::move(compA));
    sim.AddComponent(std::move(compB));
    sim.Provision();
    sim.Stage();

    // Verify layout
    auto layout = sim.GetStateLayout();
    EXPECT_EQ(layout.size(), 2);
    EXPECT_EQ(layout[0].owner, ptrA);
    EXPECT_EQ(layout[0].offset, 0);
    EXPECT_EQ(layout[0].size, 3);
    EXPECT_EQ(layout[1].owner, ptrB);
    EXPECT_EQ(layout[1].offset, 3);
    EXPECT_EQ(layout[1].size, 4);
}

TEST(StateManagement, StatePointersSeparate) {
    Simulator<double> sim;

    auto compA = std::make_unique<StatefulComponent<double>>("A", 3);
    auto compB = std::make_unique<StatefulComponent<double>>("B", 4);
    auto* ptrA = compA.get();
    auto* ptrB = compB.get();

    sim.AddComponent(std::move(compA));
    sim.AddComponent(std::move(compB));
    sim.Provision();
    sim.Stage();

    // Pointers should be offset by A's state size
    auto* stateA = static_cast<StatefulComponent<double>*>(ptrA)->GetStatePtr();
    auto* stateB = static_cast<StatefulComponent<double>*>(ptrB)->GetStatePtr();

    EXPECT_EQ(stateB - stateA, 3);  // B starts 3 elements after A
}

TEST(StateManagement, SetAndGetState) {
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 4));
    sim.Provision();
    sim.Stage();

    // Set state
    JanusVector<double> new_state = {1.0, 2.0, 3.0, 4.0};
    sim.SetState(new_state);

    // Get state
    auto retrieved = sim.GetState();
    EXPECT_EQ(retrieved.size(), 4);
    EXPECT_DOUBLE_EQ(retrieved[0], 1.0);
    EXPECT_DOUBLE_EQ(retrieved[1], 2.0);
    EXPECT_DOUBLE_EQ(retrieved[2], 3.0);
    EXPECT_DOUBLE_EQ(retrieved[3], 4.0);
}

TEST(StateManagement, ComputeDerivatives) {
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 3));
    sim.Provision();
    sim.Stage();

    // Set initial state
    JanusVector<double> X0 = {1.0, 2.0, 3.0};
    sim.SetState(X0);

    // Compute derivatives (x_dot = -x for StatefulComponent)
    const auto& X_dot = sim.ComputeDerivatives(0.0);

    EXPECT_EQ(X_dot.size(), 3);
    EXPECT_DOUBLE_EQ(X_dot[0], -1.0);
    EXPECT_DOUBLE_EQ(X_dot[1], -2.0);
    EXPECT_DOUBLE_EQ(X_dot[2], -3.0);
}

TEST(StateManagement, ZeroStateComponent) {
    Simulator<double> sim;

    // DummyComponent has StateSize() = 0
    sim.AddComponent(std::make_unique<DummyComponent<double>>("NoState"));
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("WithState", 3));

    sim.Provision();
    sim.Stage();

    // Only WithState should appear in layout
    auto layout = sim.GetStateLayout();
    EXPECT_EQ(layout.size(), 1);
    EXPECT_EQ(layout[0].size, 3);

    // Total state size is 3
    EXPECT_EQ(sim.GetTotalStateSize(), 3);
}

TEST(StateManagement, StateSizeMismatchThrows) {
    Simulator<double> sim;

    // This would be caught if BindState is called with wrong size
    // (Implementation should never do this, but test the error path)
    StatefulComponent<double> comp("Test", 5);

    double state[3];
    double state_dot[3];

    EXPECT_THROW(
        comp.BindState(state, state_dot, 3),  // Wrong size (expects 5)
        StateSizeMismatchError
    );
}

// ---------------------------------------------------------------------------
// Symbolic Mode Tests
// ---------------------------------------------------------------------------

TEST(StateManagementSymbolic, AllocationCompiles) {
    using MX = casadi::MX;

    Simulator<MX> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<MX>>("A", 4));
    sim.Provision();
    sim.Stage();

    auto state = sim.GetState();
    EXPECT_EQ(state.size(), 4);
}

TEST(StateManagementSymbolic, DerivativeComputation) {
    using MX = casadi::MX;

    Simulator<MX> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<MX>>("A", 2));
    sim.Provision();
    sim.Stage();

    // Set symbolic state
    MX x0 = MX::sym("x0");
    MX x1 = MX::sym("x1");
    JanusVector<MX> X = {x0, x1};
    sim.SetState(X);

    // Compute derivatives (symbolic)
    const auto& X_dot = sim.ComputeDerivatives(MX{0});

    EXPECT_EQ(X_dot.size(), 2);
    // Verify symbolic expressions are created
}
```

---

## Implementation Checklist

### Task 2.1a: State Vectors in Simulator

- [x] Add `JanusVector<Scalar> X_global_` member
- [x] Add `JanusVector<Scalar> X_dot_global_` member
- [x] Add `std::vector<StateSlice<Scalar>> state_layout_` member
- [x] Add `Scalar dt_nominal_` member with default
- [x] Implement `GetTotalStateSize()` method
- [x] Implement `GetState()` method
- [x] Implement `SetState()` method with size validation
- [x] Implement `GetDerivatives()` method
- [x] Implement `GetStateLayout()` method
- [x] Implement `SetNominalDt()` and `GetNominalDt()` methods

### Task 2.1b: BindState() in Component

- [x] Add `BindState(Scalar*, Scalar*, size_t)` virtual method
- [x] Default implementation is no-op
- [x] Add `HasState()` convenience method
- [x] Document contract in Doxygen comments

### Task 2.1c: State Binding in Stage()

- [x] Calculate total state size before allocation
- [x] Allocate `X_global_` and `X_dot_global_` vectors
- [x] Initialize vectors to zero
- [x] Iterate components, call `BindState()` with correct offsets
- [x] Build `state_layout_` metadata
- [x] Handle components with `StateSize() == 0` (skip binding)

### Task 2.1d: ComputeDerivatives()

- [x] Zero `X_dot_global_` at start
- [x] Call PreStep/Step/PostStep on all components
- [x] Return reference to `X_dot_global_`
- [x] Use `dt_nominal_` for hook parameters

### Task 2.1e: Error Types

- [x] Add `StateError` base exception
- [x] Add `StateSizeMismatchError` with expected/actual
- [x] Add `StateBindingError` for binding failures

### Task 2.1f: Tests

- [x] Create `tests/sim/test_state_management.cpp`
- [x] Add `TotalStateSizeCalculation` test
- [x] Add `StateVectorAllocation` test
- [x] Add `StateBindingCorrectOffsets` test
- [x] Add `StatePointersSeparate` test
- [x] Add `SetAndGetState` test
- [x] Add `ComputeDerivatives` test
- [x] Add `ZeroStateComponent` test
- [x] Add `StateSizeMismatchThrows` test
- [x] Add symbolic mode compilation tests
- [x] Update `tests/CMakeLists.txt`

### Task 2.1g: Integration

- [x] Update `include/icarus/icarus.hpp` if needed
- [x] Verify `./scripts/build.sh` succeeds
- [x] Verify `./scripts/test.sh` all pass
- [x] Update main implementation plan checkboxes

---

## Design Decisions

### 1. Pointer-Based Scatter/Gather

**Decision:** Components receive raw pointers into global vectors, not copies.

**Rationale:**

- Zero-copy access during hot-path Step()
- Integrator sees contiguous vector (optimal for CVODES, scipy)
- Single memcpy checkpoints entire simulation state

**Trade-off:** Pointers become invalid if Stage() called again (re-bind required).

### 2. State Allocation in Stage(), Not Provision()

**Decision:** Allocate state vectors in Stage(), not Provision().

**Rationale:**

- Total state size isn't known until all components added
- Stage() can be called multiple times (re-run scenarios)
- Matches architecture: Stage binds pointers, Provision allocates component memory

### 3. Derivative Vector Zeroing

**Decision:** Zero `X_dot_global_` at start of ComputeDerivatives().

**Rationale:**

- Components may accumulate into derivatives (e.g., multiple force sources)
- Clean slate each evaluation prevents stale data
- Matches typical integrator expectations

### 4. StateSize() Returns Zero for No-State Components

**Decision:** Components without integrated state return `StateSize() == 0`.

**Rationale:**

- No special marker needed (zero is natural default)
- BindState() simply not called for these components
- Keeps interface simple

### 5. dt_nominal_ for Derivative Computation

**Decision:** Store nominal timestep in Simulator for ComputeDerivatives().

**Rationale:**

- Some components may need dt hint for filtering
- Integrator chooses actual dt, but components need representative value
- Can be changed via SetNominalDt() if needed

---

## Janus Compatibility Checklist

All code in Phase 2.1 must be Janus-compatible:

- [x] All functions templated on `Scalar` (not `double`)
- [x] Use `JanusVector<Scalar>` (not `std::vector<double>`)
- [x] No `std::` math functions (use `janus::` namespace)
- [x] No `if/else` branching on `Scalar` values (use `janus::where()`)
- [x] Verify `Simulator<casadi::MX>` compiles and runs

---

## Exit Criteria

- [x] Global `X_global_`, `X_dot_global_` vectors in Simulator
- [x] `Component::BindState()` method added and documented
- [x] `Simulator::Stage()` allocates vectors and binds state
- [x] `Simulator::ComputeDerivatives()` callable by integrators
- [x] State layout metadata accessible via `GetStateLayout()`
- [x] All Phase 2.1 tests pass for both `double` and `casadi::MX`
- [x] Existing Phase 1 tests continue to pass

---

## Dependencies

| Dependency | Purpose | Status |
|:-----------|:--------|:-------|
| Phase 1.4/1.5 | Component base, Simulator shell | ✅ Complete |
| Janus `JanusVector` | Symbolic-compatible vector type | ✅ Available |
| GoogleTest | Testing framework | ✅ Available |

---

## Next Steps (Phase 2.2)

After Phase 2.1 completes, Phase 2.2 (Integrator Interface) will:

1. Define abstract `Integrator<Scalar>` interface
2. Implement `RK4Integrator` using Janus `rk4_step()`
3. Implement `RK45Integrator` for adaptive stepping
4. Modify `Simulator::Step()` to use integrator

The state infrastructure from 2.1 enables integrators to:

```cpp
// Integrator uses Simulator's state interface
auto X = sim.GetState();
auto X_dot = sim.ComputeDerivatives(t);
auto X_new = integrator.Step(t, X, dt, [&](Scalar t, auto& X) {
    sim.SetState(X);
    return sim.ComputeDerivatives(t);
});
sim.SetState(X_new);
```

---

## References

| Topic | Document |
|:------|:---------|
| State ownership & vectors | [09_memory_state_ownership.md](../../architecture/09_memory_state_ownership.md) |
| Lifecycle & Stage binding | [04_lifecycle.md](../../architecture/04_lifecycle.md) |
| Janus template paradigm | [07_janus_integration.md](../../architecture/07_janus_integration.md) |
| Symbolic constraints | [21_symbolic_constraints.md](../../architecture/21_symbolic_constraints.md) |
| Component protocol | [02_component_protocol.md](../../architecture/02_component_protocol.md) |
