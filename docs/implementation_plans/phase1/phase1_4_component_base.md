# Phase 1.4: Component Base Implementation Plan

**Status:** Proposed
**Target:** Complete Component base class with full lifecycle support

---

## Overview

Phase 1.4 establishes the complete Component base class as specified in [02_component_protocol.md](../../architecture/02_component_protocol.md) and [04_lifecycle.md](../../architecture/04_lifecycle.md). The current implementation has a basic skeleton; this phase adds:

1. **Full lifecycle protocol** — Provision/Stage/Step with proper Backplane integration
2. **Introspection support** — Input/output declaration for dependency discovery
3. **Component identity** — Full naming with entity.component.signal pattern
4. **Optional hooks** — PreStep/PostStep/OnPhaseEnter/OnPhaseExit for extended lifecycle
5. **Backplane wrapper** — Type-safe facade over SignalRegistry for component authors
6. **Dummy component** — Minimal implementation for testing the lifecycle

---

## Current State Analysis

### Existing Infrastructure

| File | Contents | Status |
|:-----|:---------|:-------|
| [Component.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Component.hpp) | Basic abstract class with lifecycle stubs | ⚠️ Needs enhancement |
| [Simulator.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp) | Top-level coordinator using Component | ⚠️ Needs alignment |
| [Registry.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Registry.hpp) | Signal registration/resolution | ✅ Complete (Phase 1.3) |
| [Types.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Types.hpp) | Phase enum, ComponentConfig | ✅ Complete |
| [Concepts.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Concepts.hpp) | ComponentType concept | ⚠️ Needs Backplane forward decl |

### Gap Analysis

| Feature | Architecture Requirement | Current Status |
|:--------|:-------------------------|:---------------|
| Backplane facade | Component receives `Backplane<Scalar>&` | ❌ Uses `SignalRegistry<Scalar>&` directly |
| Full name generation | `entity.component.signal` pattern | ❌ Component doesn't build full names |
| Introspection | `GetInputs()`, `GetOutputs()`, `GetStateSize()` | ⚠️ Stub exists, returns empty |
| Type identification | `TypeName()` for data dictionary | ❌ Missing |
| Dependency tracking | Track resolved inputs during Stage | ❌ Missing |
| Component config access | Store config for later use | ❌ Only passed to Provision |
| RunConfig in Stage | Stage receives `RunConfig` for ICs | ⚠️ Inconsistent with architecture |

---

## Architecture Alignment

Per [02_component_protocol.md](../../architecture/02_component_protocol.md):

```cpp
template <typename Scalar>
class JetEngine : public Component<Scalar> {
public:
    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        bp.register_output("thrust", &thrust_, {.units = "N"});
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        input_altitude_ = bp.resolve<Scalar>(cfg.get("input_altitude"));
    }

    void Step(Scalar t, Scalar dt) override {
        Scalar alt = *input_altitude_;
        // ... physics ...
    }
};
```

**Key observations:**

1. `Backplane` is a facade (not `SignalRegistry` directly)
2. `Stage` receives both `Backplane&` and `ComponentConfig&` (for input wiring config)
3. Component stores `input_handles` as members for hot-path access
4. Full signal names built from entity prefix

---

## Proposed Changes

### Component 1: Backplane Facade

#### [NEW] `include/icarus/signal/Backplane.hpp`

The Backplane wraps SignalRegistry and adds component-context-aware registration:

```cpp
#pragma once

#include <icarus/signal/Registry.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/VecHandle.hpp>
#include <string>

namespace icarus {

/**
 * @brief Component-facing facade for signal registration and resolution
 *
 * Backplane adds:
 * - Automatic full name generation (entity.component.signal)
 * - Context tracking for dependency discovery
 * - Cleaner API for component authors
 */
template <typename Scalar>
class Backplane {
public:
    explicit Backplane(SignalRegistry<Scalar>& registry) : registry_(registry) {}

    // =========================================================================
    // Context Management
    // =========================================================================

    /**
     * @brief Set current component context
     *
     * Called by Simulator before invoking component lifecycle methods.
     */
    void set_context(const std::string& entity, const std::string& component) {
        entity_ = entity;
        component_ = component;
        registry_.set_current_component(full_prefix());
    }

    void clear_context() {
        entity_.clear();
        component_.clear();
        registry_.clear_current_component();
    }

    [[nodiscard]] std::string full_prefix() const {
        if (entity_.empty()) return component_;
        return entity_ + "." + component_;
    }

    // =========================================================================
    // Output Registration (Provision phase)
    // =========================================================================

    /**
     * @brief Register a scalar output signal
     *
     * @param local_name Signal name (without entity/component prefix)
     * @param data_ptr Pointer to component-owned storage
     * @param unit Physical unit
     * @param description Human-readable description
     */
    template <typename T>
    void register_output(const std::string& local_name, T* data_ptr,
                        const std::string& unit = "",
                        const std::string& description = "") {
        std::string full_name = make_full_name(local_name);
        registry_.register_output<T>(full_name, data_ptr, unit, description);
        registered_outputs_.push_back(full_name);
    }

    /**
     * @brief Register a static (immutable) signal
     */
    template <typename T>
    void register_static(const std::string& local_name, const T* data_ptr,
                        const std::string& unit = "",
                        const std::string& description = "") {
        std::string full_name = make_full_name(local_name);
        registry_.register_static<T>(full_name, data_ptr, unit, description);
        registered_outputs_.push_back(full_name);
    }

    /**
     * @brief Register a Vec3 signal (expands to .x/.y/.z)
     */
    template <typename S>
    void register_vec3(const std::string& local_name, Vec3<S>* data_ptr,
                      const std::string& unit = "",
                      const std::string& description = "") {
        std::string full_name = make_full_name(local_name);
        registry_.register_vec3<S>(full_name, data_ptr, unit, description);
        registered_outputs_.push_back(full_name + ".x");
        registered_outputs_.push_back(full_name + ".y");
        registered_outputs_.push_back(full_name + ".z");
    }

    /**
     * @brief Register a quaternion signal (expands to .w/.x/.y/.z)
     */
    template <typename S>
    void register_quat(const std::string& local_name, Vec4<S>* data_ptr,
                      const std::string& unit = "",
                      const std::string& description = "") {
        std::string full_name = make_full_name(local_name);
        registry_.register_quat<S>(full_name, data_ptr, unit, description);
        registered_outputs_.push_back(full_name + ".w");
        registered_outputs_.push_back(full_name + ".x");
        registered_outputs_.push_back(full_name + ".y");
        registered_outputs_.push_back(full_name + ".z");
    }

    // =========================================================================
    // Input Resolution (Stage phase)
    // =========================================================================

    /**
     * @brief Resolve a signal by full path
     *
     * Tracks the resolution for dependency discovery.
     *
     * @param full_name Full signal path (e.g., "Environment.Atm.density")
     * @return SignalHandle<T> for zero-overhead access
     */
    template <typename T>
    [[nodiscard]] SignalHandle<T> resolve(const std::string& full_name) {
        resolved_inputs_.push_back(full_name);
        return registry_.resolve<T>(full_name);
    }

    /**
     * @brief Resolve a Vec3 signal
     */
    template <typename S>
    [[nodiscard]] Vec3Handle<S> resolve_vec3(const std::string& full_name) {
        resolved_inputs_.push_back(full_name + ".x");
        resolved_inputs_.push_back(full_name + ".y");
        resolved_inputs_.push_back(full_name + ".z");
        return registry_.resolve_vec3<S>(full_name);
    }

    /**
     * @brief Resolve a quaternion signal
     */
    template <typename S>
    [[nodiscard]] QuatHandle<S> resolve_quat(const std::string& full_name) {
        resolved_inputs_.push_back(full_name + ".w");
        resolved_inputs_.push_back(full_name + ".x");
        resolved_inputs_.push_back(full_name + ".y");
        resolved_inputs_.push_back(full_name + ".z");
        return registry_.resolve_quat<S>(full_name);
    }

    /**
     * @brief Check if a signal exists
     */
    [[nodiscard]] bool has_signal(const std::string& full_name) const {
        return registry_.HasSignal(full_name);
    }

    // =========================================================================
    // Dependency Tracking
    // =========================================================================

    /**
     * @brief Get outputs registered by current component
     */
    [[nodiscard]] const std::vector<std::string>& registered_outputs() const {
        return registered_outputs_;
    }

    /**
     * @brief Get inputs resolved by current component
     */
    [[nodiscard]] const std::vector<std::string>& resolved_inputs() const {
        return resolved_inputs_;
    }

    /**
     * @brief Clear tracking for next component
     */
    void clear_tracking() {
        registered_outputs_.clear();
        resolved_inputs_.clear();
    }

    // =========================================================================
    // Direct Registry Access (for Simulator)
    // =========================================================================

    [[nodiscard]] SignalRegistry<Scalar>& registry() { return registry_; }
    [[nodiscard]] const SignalRegistry<Scalar>& registry() const { return registry_; }

private:
    [[nodiscard]] std::string make_full_name(const std::string& local_name) const {
        std::string prefix = full_prefix();
        if (prefix.empty()) return local_name;
        return prefix + "." + local_name;
    }

    SignalRegistry<Scalar>& registry_;
    std::string entity_;
    std::string component_;
    std::vector<std::string> registered_outputs_;
    std::vector<std::string> resolved_inputs_;
};

} // namespace icarus
```

---

### Component 2: Enhanced Component Base Class

#### [MODIFY] `include/icarus/core/Component.hpp`

Update Component to use Backplane and add introspection:

```cpp
#pragma once

#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>
#include <string>
#include <vector>

namespace icarus {

// Forward declarations
template <typename Scalar> class Backplane;

/**
 * @brief Signal declaration for introspection
 */
struct SignalDecl {
    std::string name;        ///< Local signal name
    std::string unit;        ///< Physical unit
    std::string description; ///< Human-readable description
    bool is_input = false;   ///< True if input, false if output
};

/**
 * @brief Base class for all simulation components
 *
 * Components are the fundamental unit of execution in Icarus. They own
 * state and implement the Provision/Stage/Step lifecycle.
 *
 * **Lifecycle:**
 * 1. Provision (once) - Register outputs, allocate memory, load params
 * 2. Stage (per run) - Wire inputs, apply ICs
 * 3. Step (per dt) - Compute dynamics (hot path!)
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar>
class Component {
public:
    virtual ~Component() = default;

    // =========================================================================
    // Core Lifecycle (Required)
    // =========================================================================

    /**
     * @brief Provision phase - called once at application launch
     *
     * Heavy lifting: allocate memory, register signals, parse config.
     *
     * @param bp Backplane for registering outputs
     * @param config Component configuration
     */
    virtual void Provision(Backplane<Scalar>& bp, const ComponentConfig& config) = 0;

    /**
     * @brief Stage phase - called at start of each run/episode
     *
     * Wire inputs, apply initial conditions, prepare for t=0.
     *
     * @param bp Backplane for resolving inputs
     * @param config Component configuration (for input wiring paths)
     */
    virtual void Stage(Backplane<Scalar>& bp, const ComponentConfig& config) = 0;

    /**
     * @brief Step phase - called every time step (hot path!)
     *
     * Read inputs, compute derivatives, write outputs.
     * NO allocation, NO string lookups.
     *
     * @param t Current simulation time
     * @param dt Time step size
     */
    virtual void Step(Scalar t, Scalar dt) = 0;

    // =========================================================================
    // Extended Lifecycle Hooks (Optional)
    // =========================================================================

    /**
     * @brief Called before any component Steps (for pre-processing)
     */
    virtual void PreStep(Scalar /*t*/, Scalar /*dt*/) {}

    /**
     * @brief Called after all component Steps (for post-processing)
     */
    virtual void PostStep(Scalar /*t*/, Scalar /*dt*/) {}

    /**
     * @brief Called when entering a new flight phase
     */
    virtual void OnPhaseEnter(Phase /*phase*/) {}

    /**
     * @brief Called when exiting a flight phase
     */
    virtual void OnPhaseExit(Phase /*phase*/) {}

    /**
     * @brief Called when simulation encounters an error
     */
    virtual void OnError(const SimulationError& /*error*/) {}

    /**
     * @brief Called during shutdown (cleanup, flush buffers)
     */
    virtual void Shutdown() {}

    // =========================================================================
    // Identity & Introspection
    // =========================================================================

    /**
     * @brief Component instance name (e.g., "MainEngine")
     */
    [[nodiscard]] virtual std::string Name() const = 0;

    /**
     * @brief Entity namespace (e.g., "X15")
     *
     * Return empty string for singleton components (e.g., Environment).
     */
    [[nodiscard]] virtual std::string Entity() const { return ""; }

    /**
     * @brief Component type name for data dictionary (e.g., "JetEngine")
     */
    [[nodiscard]] virtual std::string TypeName() const { return Name(); }

    /**
     * @brief Full qualified name: entity.component (or just component)
     */
    [[nodiscard]] std::string FullName() const {
        std::string entity = Entity();
        if (entity.empty()) return Name();
        return entity + "." + Name();
    }

    /**
     * @brief Declared inputs (for documentation/dependency graph)
     *
     * Override to declare expected inputs for tooling.
     */
    [[nodiscard]] virtual std::vector<SignalDecl> DeclareInputs() const { return {}; }

    /**
     * @brief Declared outputs (for documentation/dependency graph)
     *
     * Override to declare outputs for tooling.
     */
    [[nodiscard]] virtual std::vector<SignalDecl> DeclareOutputs() const { return {}; }

    /**
     * @brief Number of state variables owned by this component
     */
    [[nodiscard]] virtual std::size_t StateSize() const { return 0; }

    // =========================================================================
    // Lifecycle State
    // =========================================================================

    /**
     * @brief Check if Provision has been called
     */
    [[nodiscard]] bool IsProvisioned() const { return provisioned_; }

    /**
     * @brief Check if Stage has been called
     */
    [[nodiscard]] bool IsStaged() const { return staged_; }

protected:
    // Called by Simulator to track lifecycle state
    void MarkProvisioned() { provisioned_ = true; }
    void MarkStaged() { staged_ = true; }
    void ResetStaged() { staged_ = false; }

    friend class Simulator; // Allow Simulator to call Mark* methods

private:
    bool provisioned_ = false;
    bool staged_ = false;
};

} // namespace icarus
```

---

### Component 3: Updated Simulator

#### [MODIFY] `include/icarus/sim/Simulator.hpp`

Update to use Backplane and track component lifecycle properly:

Key changes:

1. Create Backplane wrapping registry_
2. Set context before each component lifecycle call
3. Track registered/resolved signals for dependency graph
4. Pass ComponentConfig to Stage (not just Provision)

```cpp
// In Provision():
for (auto& comp : components_) {
    backplane_.set_context(comp->Entity(), comp->Name());
    backplane_.clear_tracking();

    ComponentConfig config;
    config.name = comp->Name();
    config.entity = comp->Entity();
    configs_[comp.get()] = config;  // Store for Stage

    comp->Provision(backplane_, config);
    comp->MarkProvisioned();

    // Record outputs for dependency graph
    outputs_[comp.get()] = backplane_.registered_outputs();

    backplane_.clear_context();
}

// In Stage():
for (auto& comp : components_) {
    backplane_.set_context(comp->Entity(), comp->Name());
    backplane_.clear_tracking();

    comp->Stage(backplane_, configs_[comp.get()]);
    comp->MarkStaged();

    // Record inputs for dependency graph
    inputs_[comp.get()] = backplane_.resolved_inputs();

    backplane_.clear_context();
}
```

---

### Component 4: Dummy Component for Testing

#### [NEW] `include/icarus/components/DummyComponent.hpp`

A minimal component for testing the lifecycle:

```cpp
#pragma once

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>

namespace icarus {

/**
 * @brief Minimal component for testing lifecycle
 *
 * Outputs a single signal that increments each step.
 */
template <typename Scalar>
class DummyComponent : public Component<Scalar> {
public:
    explicit DummyComponent(std::string name = "Dummy", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& /*config*/) override {
        bp.register_output("counter", &counter_, "", "Step counter");
        bp.register_output("time", &last_time_, "s", "Last step time");
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& /*config*/) override {
        counter_ = Scalar{0};
        last_time_ = Scalar{0};
    }

    void Step(Scalar t, Scalar /*dt*/) override {
        counter_ = counter_ + Scalar{1};
        last_time_ = t;
    }

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "DummyComponent"; }

    [[nodiscard]] std::vector<SignalDecl> DeclareOutputs() const override {
        return {
            {"counter", "", "Step counter", false},
            {"time", "s", "Last step time", false}
        };
    }

private:
    std::string name_;
    std::string entity_;
    Scalar counter_{};
    Scalar last_time_{};
};

} // namespace icarus
```

---

### Component 5: Updated Concepts

#### [MODIFY] `include/icarus/core/Concepts.hpp`

Fix forward declaration and update concept to use Backplane:

```cpp
// Forward declarations
template <typename Scalar> class Backplane;  // Changed from SignalRegistry

template <typename T, typename Scalar>
concept ComponentType = requires(T& c, Backplane<Scalar>& bp,
                                 const ComponentConfig& cfg,
                                 Scalar t, Scalar dt) {
    { c.Name() } -> std::convertible_to<std::string>;
    { c.Provision(bp, cfg) } -> std::same_as<void>;
    { c.Stage(bp, cfg) } -> std::same_as<void>;  // Now takes config
    { c.Step(t, dt) } -> std::same_as<void>;
};
```

---

## File Summary

| Action | File |
|:-------|:-----|
| NEW | `include/icarus/signal/Backplane.hpp` |
| MODIFY | `include/icarus/core/Component.hpp` |
| MODIFY | `include/icarus/sim/Simulator.hpp` |
| MODIFY | `include/icarus/core/Concepts.hpp` |
| NEW | `include/icarus/components/DummyComponent.hpp` |
| NEW | `tests/core/test_component.cpp` |
| MODIFY | `include/icarus/icarus.hpp` (add new headers) |
| MODIFY | `tests/CMakeLists.txt` (add new test) |

---

## Verification Plan

### Automated Tests

All tests use GoogleTest. Run with:

```bash
./scripts/test.sh

# Or run component tests specifically
cd build && ctest -R component --output-on-failure
```

#### Test Cases for `test_component.cpp`

| Test Name | Description |
|:----------|:------------|
| `Backplane_Context` | Verify set_context/clear_context works |
| `Backplane_FullNameGeneration` | Verify `entity.component.signal` pattern |
| `Backplane_RegisterOutput` | Register output, verify in registry |
| `Backplane_RegisterVec3` | Register Vec3, verify .x/.y/.z expansion |
| `Backplane_Resolve` | Resolve signal, verify handle works |
| `Backplane_DependencyTracking` | Verify registered_outputs/resolved_inputs tracking |
| `Component_Lifecycle` | Provision/Stage/Step sequence with DummyComponent |
| `Component_FullName` | Verify FullName() with and without entity |
| `Component_Introspection` | Verify DeclareInputs/DeclareOutputs |
| `Component_LifecycleState` | Verify IsProvisioned/IsStaged flags |
| `Simulator_ProvisionStage` | Full lifecycle with multiple components |
| `Simulator_SignalAccess` | Access signals after Provision |
| `Simulator_StepIncrementsTime` | Verify time advances |

#### Symbolic Backend Tests

| Test Name | Description |
|:----------|:------------|
| `ComponentSymbolic_Lifecycle` | DummyComponent with `casadi::MX` |
| `SimulatorSymbolic_FullLifecycle` | Full lifecycle with symbolic Scalar |

---

## Implementation Checklist

### Backplane (Backplane.hpp)

- [x] Create `Backplane<Scalar>` wrapper class
- [x] Implement `set_context(entity, component)`
- [x] Implement `clear_context()`
- [x] Implement `full_prefix()` for name generation
- [x] Implement `register_output<T>()` with full name
- [x] Implement `register_static<T>()` with full name
- [x] Implement `register_vec3<S>()` with expansion
- [x] Implement `register_quat<S>()` with expansion
- [x] Implement `resolve<T>()` with tracking
- [x] Implement `resolve_vec3<S>()` with tracking
- [x] Implement `resolve_quat<S>()` with tracking
- [x] Implement `has_signal()`
- [x] Implement `registered_outputs()` getter
- [x] Implement `resolved_inputs()` getter
- [x] Implement `clear_tracking()`

### Component (Component.hpp)

- [x] Update `Provision()` signature to use `Backplane<Scalar>&`
- [x] Update `Stage()` signature to take `Backplane<Scalar>&` and `ComponentConfig&`
- [x] Add `SignalDecl` struct
- [x] Add `TypeName()` virtual method
- [x] Add `FullName()` helper method
- [x] Add `DeclareInputs()` virtual method
- [x] Add `DeclareOutputs()` virtual method
- [x] Add `StateSize()` virtual method
- [x] Add `IsProvisioned()` / `IsStaged()` state tracking
- [x] Add `MarkProvisioned()` / `MarkStaged()` protected methods

### Simulator (Simulator.hpp)

- [x] Add `Backplane<Scalar>` member wrapping registry
- [x] Store `ComponentConfig` per component for Stage access
- [x] Update `Provision()` to use Backplane and track outputs
- [x] Update `Stage()` to pass ComponentConfig and track inputs
- [x] Call `comp->MarkProvisioned()` / `comp->MarkStaged()`
- [x] Add `GetBackplane()` accessor

### Concepts (Concepts.hpp)

- [x] Update forward declaration to use `Backplane`
- [x] Update `ComponentType` concept for new signatures

### DummyComponent (DummyComponent.hpp)

- [x] Create `DummyComponent<Scalar>` class
- [x] Implement `Provision()` with counter/time outputs
- [x] Implement `Stage()` with initialization
- [x] Implement `Step()` incrementing counter
- [x] Implement `Name()`, `Entity()`, `TypeName()`
- [x] Implement `DeclareOutputs()`

### Tests (test_component.cpp)

- [x] Create `tests/core/test_component.cpp`
- [x] Add Backplane context tests
- [x] Add Backplane registration tests
- [x] Add Backplane resolution tests
- [x] Add Component lifecycle tests
- [x] Add Simulator integration tests
- [x] Add symbolic backend tests
- [x] Update `tests/CMakeLists.txt`

### Integration

- [x] Update `include/icarus/icarus.hpp` to include new headers
- [x] Verify existing tests still pass
- [x] Verify `./scripts/build.sh` succeeds
- [x] Verify `./scripts/test.sh` all pass

---

## Design Decisions

### 1. Backplane vs Direct Registry Access

**Decision:** Backplane facade with registry access for advanced use

- Component authors use `Backplane` for clean API
- Simulator uses `backplane_.registry()` for low-level access
- Preserves flexibility while simplifying common case

**Rationale:** Matches architecture doc; reduces boilerplate in components.

### 2. ComponentConfig in Stage

**Decision:** Stage receives ComponentConfig (not just Backplane)

- Architecture shows input wiring config in Stage: `cfg.get("input_altitude")`
- Component needs config to know which signals to resolve

**Rationale:** Enables flexible input wiring per scenario.

### 3. Introspection Methods

**Decision:** Virtual `DeclareInputs()`/`DeclareOutputs()` returning `SignalDecl` vectors

- Optional to implement (defaults to empty)
- Used by tooling, not required for simulation
- Separate from runtime registration (can declare without registering)

**Rationale:** Enables data dictionary generation and documentation without runtime overhead.

### 4. Lifecycle State Tracking

**Decision:** Track provisioned/staged state in base class

- `IsProvisioned()`, `IsStaged()` for debugging
- Protected `Mark*` methods called by Simulator

**Rationale:** Helps catch lifecycle violations (e.g., Stage before Provision).

### 5. Entity Prefix Handling

**Decision:** Entity is optional; components can exist without entity namespace

- `FullName()` returns just `Name()` if entity is empty
- Enables singleton components like `Environment`, `Time`

**Rationale:** Not all components belong to an entity (e.g., environment models).

---

## Exit Criteria

- [x] Can instantiate `Simulator<double>`, add DummyComponent
- [x] Provision/Stage/Step lifecycle works without crashes
- [x] Signals registered with full `entity.component.signal` names
- [x] Backplane tracks registered outputs and resolved inputs
- [x] All Phase 1.4 tests pass for both `double` and `casadi::MX`
- [x] Existing tests from Phase 1.1-1.3 continue to pass

---

## Dependencies

| Dependency | Purpose | Status |
|:-----------|:--------|:-------|
| Phase 1.3 Signal Backplane | SignalRegistry, Handle, VecHandle | ✅ Complete |
| Janus types | Vec3, Vec4, JanusScalar | ✅ Available |
| GoogleTest | Test framework | ✅ Available |

---

## Migration Notes

### Breaking Changes

1. `Component::Provision()` signature changes from `SignalRegistry&` to `Backplane&`
2. `Component::Stage()` signature adds `ComponentConfig&` parameter
3. Existing code using direct `SignalRegistry&` in components must update

### Migration Path

1. Update component Provision/Stage signatures
2. Replace `registry.register_output()` with `bp.register_output()`
3. Replace `registry.resolve()` with `bp.resolve()`
4. Use local signal names (prefix added automatically)

---

## References

| Topic | Document |
|:------|:---------|
| Component protocol | [02_component_protocol.md](../../architecture/02_component_protocol.md) |
| Lifecycle phases | [04_lifecycle.md](../../architecture/04_lifecycle.md) |
| Signal backplane | [03_signal_backplane.md](../../architecture/03_signal_backplane.md) |
| Core philosophy | [01_core_philosophy.md](../../architecture/01_core_philosophy.md) |
