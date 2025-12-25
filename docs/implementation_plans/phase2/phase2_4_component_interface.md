# Phase 2.4: Component Interface System

**Status:** Proposed
**Prerequisites:** Phase 1 (Foundation), Phase 2.1 (State Management)
**Architecture Reference:** [02_component_protocol.md](../../architecture/02_component_protocol.md)

---

## Overview

This phase implements the complete Component Interface Model with explicit registration of:
- **Outputs** — Dynamic signals produced by the component
- **Inputs** — Dynamic signal ports consumed by the component (wired externally)
- **Parameters** — Configurable Scalar values accessible at runtime

All interface elements are Scalar-templated for symbolic compatibility, discoverable via the Data Dictionary, and accessible via a unified Signal Access API.

---

## Goals

1. Components declare their full interface at Provision (outputs, inputs, parameters)
2. Input wiring is external to components (config or programmatic)
3. Any signal can be get/set at runtime via unified API
4. Data Dictionary captures complete interface for tooling
5. Pre-run validation ensures all inputs are wired

---

## Tasks

### 2.4.1 Input Handle Infrastructure

**File:** `include/icarus/signal/InputHandle.hpp`

```cpp
#pragma once

#include <icarus/core/Types.hpp>
#include <string>

namespace icarus {

/**
 * @brief Handle to an input signal port
 *
 * Registered at Provision, wired to a source at Stage.
 * Provides type-safe access to the wired signal value.
 */
template <typename T>
class InputHandle {
public:
    InputHandle() = default;

    /// Get the current value from the wired source
    [[nodiscard]] T get() const {
        if (!source_) {
            throw UnwiredInputError(name_);
        }
        return *source_;
    }

    /// Check if this input has been wired
    [[nodiscard]] bool is_wired() const { return source_ != nullptr; }

    /// Get the port name
    [[nodiscard]] const std::string& name() const { return name_; }

    /// Get the wired source signal name (empty if unwired)
    [[nodiscard]] const std::string& wired_to() const { return wired_to_; }

private:
    friend class Backplane;

    std::string name_;           // Port name (e.g., "throttle")
    std::string full_name_;      // Full name (e.g., "X15.Engine.throttle")
    std::string wired_to_;       // Source signal name
    const T* source_ = nullptr;  // Pointer to source value

    // Metadata
    std::string units_;
    std::string description_;
};

// Convenience aliases
template <typename Scalar>
using ScalarInput = InputHandle<Scalar>;

template <typename Scalar>
using Vec3Input = InputHandle<Vec3<Scalar>>;

} // namespace icarus
```

**Exit Criteria:**
- [ ] `InputHandle<T>` template compiles for Scalar and Vec3 types
- [ ] `get()` throws `UnwiredInputError` if not wired
- [ ] Metadata (units, description) stored correctly

---

### 2.4.2 Signal Registry Enhancements

**File:** `include/icarus/signal/Registry.hpp` (extend existing)

Add storage for inputs and parameters alongside outputs:

```cpp
template <typename Scalar>
class SignalRegistry {
public:
    // === Output Registration (existing, enhanced) ===
    template <typename T>
    void register_output(const std::string& name, T* storage,
                         const std::string& units, const std::string& desc);

    // === Input Port Registration (NEW) ===
    template <typename T>
    void register_input(const std::string& name, InputHandle<T>* handle,
                        const std::string& units, const std::string& desc);

    // === Parameter Registration (NEW) ===
    template <typename T>
    void register_param(const std::string& name, T* storage, T initial_value,
                        const std::string& units, const std::string& desc);

    // === Wiring (NEW) ===
    void wire_input(const std::string& input_name, const std::string& source_name);
    void wire_inputs_from_config(const WiringConfig& cfg);
    void wire_all_inputs();  // Uses stored wiring config

    // === Access API (NEW) ===
    template <typename T>
    T get(const std::string& signal_name) const;

    template <typename T>
    void set(const std::string& signal_name, const T& value);

    // === Introspection ===
    [[nodiscard]] std::vector<SignalInfo> get_outputs() const;
    [[nodiscard]] std::vector<SignalInfo> get_inputs() const;
    [[nodiscard]] std::vector<SignalInfo> get_parameters() const;
    [[nodiscard]] SignalInfo get_signal_info(const std::string& name) const;

    // === Validation ===
    [[nodiscard]] std::vector<std::string> get_unwired_inputs() const;
    void validate_wiring() const;  // Throws if any input unwired

private:
    // Output storage: name -> {pointer, metadata}
    std::unordered_map<std::string, OutputEntry> outputs_;

    // Input ports: name -> {handle*, metadata, wired_to}
    std::unordered_map<std::string, InputEntry> inputs_;

    // Parameters: name -> {pointer, metadata, initial_value}
    std::unordered_map<std::string, ParamEntry> parameters_;

    // Pending wiring configuration
    WiringConfig wiring_config_;
};
```

**Exit Criteria:**
- [ ] `register_input()` stores handle and metadata
- [ ] `register_param()` stores pointer, initial value, and metadata
- [ ] `wire_input()` connects input handle to output source
- [ ] `get<T>()` works for outputs, parameters
- [ ] `set<T>()` works for parameters (and outputs in test mode)
- [ ] `validate_wiring()` throws with helpful message for unwired inputs

---

### 2.4.3 Backplane API Updates

**File:** `include/icarus/signal/Backplane.hpp`

Extend Backplane to expose the new registration methods:

```cpp
template <typename Scalar>
class Backplane {
public:
    // === Output Registration ===
    template <typename T>
    void register_output(const std::string& name, T* storage,
                         const std::string& units, const std::string& desc);

    // Overload with SignalMetadata struct
    template <typename T>
    void register_output(const std::string& name, T* storage,
                         const SignalMetadata& meta);

    // === Input Port Registration ===
    template <typename T>
    void register_input(const std::string& name, InputHandle<T>* handle,
                        const std::string& units, const std::string& desc);

    // === Parameter Registration ===
    template <typename T>
    void register_param(const std::string& name, T* storage, T initial_value,
                        const std::string& units, const std::string& desc);

    // === Wiring ===
    void set_wiring_config(const WiringConfig& cfg);
    void wire_inputs();  // Called by component in Stage()

    // === Signal Access ===
    template <typename T>
    T get(const std::string& name) const;

    template <typename T>
    void set(const std::string& name, const T& value);

    SignalInfo get_signal_info(const std::string& name) const;

    // === Data Dictionary ===
    void generate_data_dictionary(const std::string& path) const;
    DataDictionary get_data_dictionary() const;

    // === Validation ===
    void validate_wiring() const;
    std::vector<std::string> get_unwired_inputs() const;

private:
    SignalRegistry<Scalar> registry_;
    std::string current_component_prefix_;  // Set during component registration
};
```

**Exit Criteria:**
- [ ] All registration methods delegate to registry
- [ ] Component prefix automatically prepended to signal names
- [ ] `wire_inputs()` uses stored wiring config for current component
- [ ] `get<T>()`/`set<T>()` provide type-safe access

---

### 2.4.4 Component Base Class Updates

**File:** `include/icarus/core/Component.hpp`

Update Component base to track registered interface:

```cpp
template <typename Scalar>
class Component {
public:
    // Lifecycle methods (existing)
    virtual void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) = 0;
    virtual void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) = 0;
    virtual void Step(Scalar t, Scalar dt) = 0;

    // Interface introspection (NEW)
    [[nodiscard]] std::vector<std::string> GetOutputNames() const;
    [[nodiscard]] std::vector<std::string> GetInputNames() const;
    [[nodiscard]] std::vector<std::string> GetParamNames() const;

    // Validation helper
    [[nodiscard]] std::vector<std::string> GetUnwiredInputs() const;

protected:
    // Tracked during Provision
    std::vector<std::string> registered_outputs_;
    std::vector<std::string> registered_inputs_;
    std::vector<std::string> registered_params_;
};
```

**Exit Criteria:**
- [ ] Components can query their own interface
- [ ] Useful for debugging and testing

---

### 2.4.5 Simulator Wiring API

**File:** `include/icarus/sim/Simulator.hpp`

Add wiring and access methods to Simulator:

```cpp
template <typename Scalar>
class Simulator {
public:
    // === Wiring API ===

    /// Wire a single input to a source
    void Wire(const std::string& input, const std::string& source);

    /// Load wiring from YAML config
    void LoadWiring(const std::string& path);
    void LoadWiring(const WiringConfig& cfg);

    /// Validate all inputs are wired (call after Stage)
    void ValidateWiring() const;

    // === Signal Access API ===

    /// Get any signal value by name
    template <typename T>
    T Get(const std::string& signal_name) const;

    /// Set any signal value by name
    template <typename T>
    void Set(const std::string& signal_name, const T& value);

    /// Get signal metadata
    SignalInfo GetSignalInfo(const std::string& name) const;

    /// Get all signal names matching pattern
    std::vector<std::string> GetSignals(const std::string& pattern = "*") const;

    // === Data Dictionary ===
    void GenerateDataDictionary(const std::string& path) const;
    DataDictionary GetDataDictionary() const;

private:
    WiringConfig wiring_config_;
    Backplane<Scalar> backplane_;
};
```

**Exit Criteria:**
- [ ] `Wire()` stores wiring for later application
- [ ] `LoadWiring()` parses YAML wiring config
- [ ] `ValidateWiring()` throws with list of unwired inputs
- [ ] `Get<T>()`/`Set<T>()` work for any registered signal

---

### 2.4.6 Wiring Configuration Format

**File:** `include/icarus/io/WiringConfig.hpp`

```cpp
#pragma once

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace icarus {

/**
 * @brief Wiring configuration mapping inputs to sources
 */
class WiringConfig {
public:
    /// Load from YAML file
    static WiringConfig FromFile(const std::string& path);

    /// Load from YAML node
    static WiringConfig FromYAML(const YAML::Node& node);

    /// Get source for an input (throws if not found)
    [[nodiscard]] std::string GetSource(const std::string& input) const;

    /// Check if input has wiring specified
    [[nodiscard]] bool HasWiring(const std::string& input) const;

    /// Add wiring programmatically
    void AddWiring(const std::string& input, const std::string& source);

    /// Get all wirings
    [[nodiscard]] const std::unordered_map<std::string, std::string>&
        GetAllWirings() const;

private:
    // input_name -> source_name
    std::unordered_map<std::string, std::string> wirings_;
};

} // namespace icarus
```

**YAML Format:**

```yaml
# wiring.yaml
wiring:
  # Component-level wiring
  X15.MainEngine:
    altitude: "Environment.Atmosphere.altitude"
    throttle: "X15.GNC.throttle_cmd"
    mach: "Environment.Atmosphere.mach"

  X15.Aero:
    velocity: "X15.EOM.velocity"
    density: "Environment.Atmosphere.density"

  # Or fully-qualified names
  # X15.MainEngine.altitude: "Environment.Atmosphere.altitude"
```

**Exit Criteria:**
- [ ] YAML parsing works for both nested and flat formats
- [ ] Error messages include line numbers for invalid config
- [ ] Programmatic wiring addition works

---

### 2.4.7 Data Dictionary Generation

**File:** `include/icarus/io/DataDictionary.hpp`

```cpp
#pragma once

#include <icarus/signal/SignalInfo.hpp>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Complete catalog of simulation interface
 */
struct DataDictionary {
    struct ComponentEntry {
        std::string name;        // e.g., "X15.MainEngine"
        std::string type;        // e.g., "JetEngine"

        std::vector<SignalInfo> outputs;
        std::vector<SignalInfo> inputs;
        std::vector<SignalInfo> parameters;
    };

    std::vector<ComponentEntry> components;

    // Summary stats
    std::size_t total_outputs = 0;
    std::size_t total_inputs = 0;
    std::size_t total_parameters = 0;
    std::size_t integrable_states = 0;
    std::size_t unwired_inputs = 0;

    /// Export to YAML file
    void ToYAML(const std::string& path) const;

    /// Export to JSON file
    void ToJSON(const std::string& path) const;
};

/**
 * @brief Signal metadata
 */
struct SignalInfo {
    std::string name;           // Local name (e.g., "thrust")
    std::string full_name;      // Full name (e.g., "X15.MainEngine.thrust")
    std::string type_name;      // "Scalar", "Vec3<Scalar>", etc.
    std::string units;
    std::string description;

    SignalType signal_type;     // Output, Input, Parameter

    // For inputs only
    std::string wired_to;

    // For parameters only
    std::string initial_value;  // String representation

    // For outputs with state
    bool integrable = false;
};

enum class SignalType {
    Output,
    Input,
    Parameter
};

} // namespace icarus
```

**Exit Criteria:**
- [ ] YAML export matches format in architecture doc
- [ ] JSON export for programmatic tooling
- [ ] Summary statistics computed correctly
- [ ] Unwired inputs tracked

---

### 2.4.8 Update Existing Components

Update `PointMass3DOF` and `Gravity` to use new interface pattern:

**File:** `components/dynamics/PointMass3DOF.hpp`

```cpp
template <typename Scalar>
class PointMass3DOF : public Component<Scalar> {
    // Outputs
    Vec3<Scalar> position_;
    Vec3<Scalar> velocity_;

    // Inputs (NEW: explicit registration)
    Vec3Input<Scalar> force_;

    // Parameters (NEW: Scalar-typed)
    Scalar mass_;

public:
    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Outputs
        bp.register_output("position", &position_, "m", "Position vector");
        bp.register_output("velocity", &velocity_, "m/s", "Velocity vector");

        // Inputs (declare port, don't wire yet)
        bp.register_input("force", &force_, "N", "Applied force vector");

        // Parameters
        bp.register_param("mass", &mass_,
                          cfg.get<Scalar>("mass", Scalar{1.0}),
                          "kg", "Point mass");
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Wire inputs from external config
        bp.wire_inputs();
    }

    void Step(Scalar t, Scalar dt) override {
        Vec3<Scalar> f = force_.get();  // Read wired input
        Vec3<Scalar> accel = vulcan::dynamics::point_mass_acceleration(f, mass_);
        // ...
    }
};
```

**Exit Criteria:**
- [ ] `PointMass3DOF` uses `register_input()` for force
- [ ] `PointMass3DOF` uses `register_param()` for mass
- [ ] `Gravity` component updated similarly
- [ ] All tests pass with new interface

---

### 2.4.9 Integration Tests

**File:** `tests/integration/component_interface_test.cpp`

```cpp
TEST(ComponentInterface, InputWiring) {
    Simulator<double> sim;

    auto gravity = std::make_unique<Gravity<double>>();
    auto point_mass = std::make_unique<PointMass3DOF<double>>();

    sim.AddComponent(std::move(gravity));
    sim.AddComponent(std::move(point_mass));

    // Wire inputs
    sim.Wire("PointMass3DOF.force", "Gravity.force");

    sim.Provision();
    sim.Stage();

    // Validate wiring
    EXPECT_NO_THROW(sim.ValidateWiring());
    EXPECT_EQ(sim.GetDataDictionary().unwired_inputs, 0);
}

TEST(ComponentInterface, UnwiredInputDetection) {
    Simulator<double> sim;
    auto point_mass = std::make_unique<PointMass3DOF<double>>();
    sim.AddComponent(std::move(point_mass));

    sim.Provision();
    sim.Stage();

    // Should detect unwired input
    EXPECT_THROW(sim.ValidateWiring(), UnwiredInputError);

    auto unwired = sim.GetDataDictionary().unwired_inputs;
    EXPECT_EQ(unwired, 1);
}

TEST(ComponentInterface, SignalAccess) {
    Simulator<double> sim;
    // ... setup ...

    // Get output
    auto pos = sim.Get<Vec3<double>>("PointMass3DOF.position");

    // Set parameter
    sim.Set("PointMass3DOF.mass", 100.0);
    EXPECT_DOUBLE_EQ(sim.Get<double>("PointMass3DOF.mass"), 100.0);
}

TEST(ComponentInterface, DataDictionaryGeneration) {
    Simulator<double> sim;
    // ... setup ...

    auto dict = sim.GetDataDictionary();

    // Check structure
    EXPECT_EQ(dict.components.size(), 2);

    auto& pm = dict.components[0];  // PointMass3DOF
    EXPECT_EQ(pm.outputs.size(), 2);   // position, velocity
    EXPECT_EQ(pm.inputs.size(), 1);    // force
    EXPECT_EQ(pm.parameters.size(), 1); // mass
}
```

**Exit Criteria:**
- [ ] Wiring validation catches unwired inputs
- [ ] Signal access API works for all signal types
- [ ] Data Dictionary contains complete interface
- [ ] Error messages are helpful (suggest similar names)

---

## File Summary

| File | Action | Description |
|:-----|:-------|:------------|
| `include/icarus/signal/InputHandle.hpp` | Create | Input handle template |
| `include/icarus/signal/Registry.hpp` | Extend | Add input/param storage |
| `include/icarus/signal/Backplane.hpp` | Extend | Add new registration methods |
| `include/icarus/signal/SignalInfo.hpp` | Create | Signal metadata struct |
| `include/icarus/core/Component.hpp` | Extend | Interface introspection |
| `include/icarus/sim/Simulator.hpp` | Extend | Wiring and access API |
| `include/icarus/io/WiringConfig.hpp` | Create | Wiring config parser |
| `include/icarus/io/DataDictionary.hpp` | Create | Interface catalog |
| `components/dynamics/PointMass3DOF.hpp` | Update | Use new interface pattern |
| `components/environment/Gravity.hpp` | Update | Use new interface pattern |
| `tests/integration/component_interface_test.cpp` | Create | Integration tests |

---

## Dependencies

```
InputHandle.hpp
       ↓
Registry.hpp ←── SignalInfo.hpp
       ↓
Backplane.hpp ←── WiringConfig.hpp
       ↓
Simulator.hpp ←── DataDictionary.hpp
       ↓
Component updates
       ↓
Integration tests
```

---

## Exit Criteria (Phase 2.4)

- [ ] Components declare inputs with `register_input()`
- [ ] Components declare parameters with `register_param()` (Scalar-typed)
- [ ] Wiring is external to components (YAML or programmatic)
- [ ] `sim.Get<T>()`/`sim.Set<T>()` works for any signal
- [ ] `sim.ValidateWiring()` catches unwired inputs before run
- [ ] Data Dictionary includes outputs, inputs, and parameters
- [ ] `PointMass3DOF` and `Gravity` updated to new pattern
- [ ] All existing tests pass
- [ ] New integration tests pass

---

## Design Decisions

### 1. Scalar-Typed Parameters

Parameters are `Scalar`-typed (not `double`) to maintain symbolic compatibility:

```cpp
Scalar max_thrust_;  // Works with double or casadi::MX
```

This allows:
- Same component code for numeric and symbolic simulation
- Parameters can be optimization variables in trim solver
- Unified get/set API for all signal types

### 2. InputHandle vs Raw Pointer

Using `InputHandle<T>` instead of raw `T*` provides:
- Clear ownership semantics (component owns handle, not data)
- Wiring state tracking (`is_wired()`)
- Better error messages on access before wiring
- Metadata attachment

### 3. Wiring at Stage, Not Provision

Wiring happens at Stage because:
- All outputs must be registered (Provision) before inputs can be wired
- Allows different wiring for same component type in different scenarios
- Config can override default wiring

### 4. Unified Signal Access

Single `Get<T>()`/`Set<T>()` API for all signal types because:
- User doesn't need to know if something is output vs parameter
- Consistent interface for tooling
- Parameters are just slow-changing signals

### 5. Validation Before Run

`ValidateWiring()` is explicit (not automatic) because:
- Some test scenarios intentionally leave inputs unwired
- Allows partial setup for debugging
- User controls when validation happens
