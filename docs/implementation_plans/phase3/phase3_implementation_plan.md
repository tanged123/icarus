# Phase 3: Symbolic Mode Implementation Plan

**Status:** Proposed
**Target:** Dual-mode symbolic/numeric simulation with graph export
**Prerequisites:** Phase 2 complete (State & Integration working with `Simulator<double>`)

---

## Overview

Phase 3 validates that the entire Icarus stack compiles and runs correctly with `SymbolicScalar` (Janus's wrapper around `casadi::MX`) as the scalar backend. This enables automatic differentiation, optimization, and trajectory analysis.

### Key Concerns Addressed

1. **Standardized Execution API**: Current examples duplicate boilerplate. We'll create a unified `SimulationRunner` that works identically for both numeric and symbolic modes.

2. **Signal System Validation**: Analysis confirms the signal system is correctly templated. `SignalRegistry<Scalar>` stores values in `std::deque<Scalar>`, and `TypeTraits<SymbolicScalar>` already exists. No architectural changes needed—only validation.

3. **Janus Compliance Audit**: Systematic scan for `std::` math and if/else branching on scalars.

---

## Architecture Assessment Summary

### Current Status: GREEN (Ready for Symbolic Mode)

| System | Template Status | Janus Compliance | Notes |
|:-------|:----------------|:-----------------|:------|
| SignalRegistry | `<Scalar>` | N/A | `std::deque<Scalar>` storage |
| Simulator | `<Scalar>` | ✓ Uses Janus types | Clean template chain |
| Component | `<Scalar>` | ✓ | Base class correct |
| PointMass3DOF | `<Scalar>` | ✓ | No std:: math |
| PointMassGravity | `<Scalar>` | ✓ | Switch on config, not Scalar |
| Integrator (RK4) | `<Scalar>` | ✓ | JanusVector throughout |
| InputHandle | `<T>` | N/A | Pointer dereferencing only |
| Vec3Handle | `<Scalar>` | ✓ | Templated correctly |

### Potential Risk Areas

1. **Vulcan Physics Functions**: Must verify all Vulcan calls support `SymbolicScalar`
2. **MX Object Lifetime**: `SymbolicScalar` in `std::deque` requires careful memory management
3. **Logger Integration**: Current logger uses `double` formatting—needs conditional handling

---

## Implementation Tasks

### 3.1 Standardized Execution Framework

**Goal:** Eliminate boilerplate, provide consistent API for both modes while preserving user control.

**Key Design Principle:** The builder is a *setup convenience*, not an execution wrapper.
External bindings (Python, MATLAB, C) require step-by-step control, signal peek/poke,
and state access. The Simulator itself must expose these interfaces.

#### 3.1.1 SimulationBuilder Pattern

Create `include/icarus/sim/SimulationBuilder.hpp`:

```cpp
template <typename Scalar>
class SimulationBuilder {
public:
    SimulationBuilder& AddComponent(std::unique_ptr<Component<Scalar>> comp);
    SimulationBuilder& Wire(const std::string& target,
                           const std::map<std::string, std::string>& wiring);
    SimulationBuilder& SetIntegrator(std::unique_ptr<Integrator<Scalar>> integ);
    SimulationBuilder& SetTimeStep(Scalar dt);
    SimulationBuilder& SetEndTime(Scalar t_end);
    SimulationBuilder& EnableProfiling(bool enable = true);
    SimulationBuilder& SetLogFile(const std::string& path);

    // Build and return configured simulator
    Simulator<Scalar> Build();
};
```

**Deliverables:**
- [ ] `include/icarus/sim/SimulationBuilder.hpp`
- [ ] Unit tests for builder pattern
- [ ] Migration guide for existing examples

**Phase 5 Forward-Compatibility:**
The builder API is designed to support config file loading (see [13_configuration.md](../../architecture/13_configuration.md)):
- `LoadScenario(path)` - Load Layer B (entities, wiring, scheduler)
- `LoadServices(path)` - Load Layer F (logging, recording)
- `LoadTrimConfig(path)` - Load Layer E (trim solver)
- `OverrideParam()` / `OverrideInitialCondition()` - Programmatic overrides

These methods are stubbed/documented in Phase 3 but implemented in Phase 5.

#### 3.1.2 Simulator External Interface API

Extend `Simulator<Scalar>` with binding-friendly signal access:

```cpp
template <typename Scalar>
class Simulator {
public:
    // Signal Peek/Poke
    Scalar GetSignal(const std::string& name) const;       // Already exists
    void SetSignal(const std::string& name, Scalar value); // Already exists

    template <typename T> T Peek(const std::string& name) const;
    template <typename T> void Poke(const std::string& name, const T& value);

    std::map<std::string, Scalar> PeekBatch(const std::vector<std::string>& names) const;
    void PokeBatch(const std::map<std::string, Scalar>& values);

    // State Vector Access
    JanusVector<Scalar> GetState() const;
    void SetState(const JanusVector<Scalar>& state);

    // Lifecycle
    void Initialize();  // Provision + Stage
    void Reset();       // Reset to initial conditions
    bool IsInitialized() const;

    // Signal Discovery
    std::vector<std::string> GetSignalNames() const;
    std::vector<std::string> GetInputNames() const;
    std::vector<std::string> GetOutputNames() const;
    std::vector<std::string> GetParameterNames() const;

    // External Callbacks (for HIL/real-time)
    void RegisterInputSource(const std::string& signal, std::function<Scalar()> cb);
    void RegisterOutputObserver(const std::string& signal, std::function<void(Scalar)> cb);
};
```

**Deliverables:**
- [ ] `Peek<T>`/`Poke<T>` template methods
- [ ] `PeekBatch`/`PokeBatch` for efficient bulk access
- [ ] `GetState`/`SetState` for checkpointing
- [ ] `Initialize()` convenience method
- [ ] `Reset()` for re-running with same setup
- [ ] Signal discovery methods
- [ ] Input source/output observer callbacks

#### 3.1.3 SimulationRunner (Optional Convenience)

**NOTE:** SimulationRunner is for "run to completion" scenarios only.
For external bindings (Python, MATLAB, C), use `Simulator` directly with step-by-step control.

Create `include/icarus/sim/SimulationRunner.hpp`:

```cpp
template <typename Scalar>
class SimulationRunner {
public:
    explicit SimulationRunner(Simulator<Scalar>& sim);

    // "Run to completion" execution
    void Run(Scalar dt, Scalar t_end);

    // Configuration
    void SetProgressCallback(std::function<void(Scalar t, Scalar t_end)> cb);
    void SetStepCallback(std::function<void(Scalar t)> cb);

    // Results
    SimulationResults<Scalar> GetResults() const;
};
```

**Use Cases:**
- Batch simulations (Monte Carlo)
- Quick demos/examples
- Cases where external code doesn't inject controls

**Deliverables:**
- [ ] `include/icarus/sim/SimulationRunner.hpp`
- [ ] `SimulationResults<Scalar>` struct for capturing outcomes
- [ ] Unit tests

---

### 3.2 Dual-Mode Validation

**Goal:** Prove `Simulator<SymbolicScalar>` compiles and runs correctly.

#### 3.2.1 Compilation Test

Create `tests/symbolic/symbolic_compilation_test.cpp`:

```cpp
#include <icarus/sim/Simulator.hpp>
#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

// This test MUST compile - verifies template instantiation
TEST(SymbolicMode, SimulatorInstantiation) {
    using Scalar = icarus::SymbolicScalar;

    Simulator<Scalar> sim;  // Must compile

    auto gravity = std::make_unique<PointMassGravity<Scalar>>("Gravity");
    auto vehicle = std::make_unique<PointMass3DOF<Scalar>>(Scalar{450000.0}, "PointMass3DOF");

    sim.AddComponent(std::move(gravity));
    sim.AddComponent(std::move(vehicle));

    // Lifecycle must complete without exceptions
    sim.Provision();
    sim.Stage();
}
```

**Deliverables:**
- [ ] `tests/symbolic/symbolic_compilation_test.cpp`
- [ ] CI job that builds symbolic mode tests
- [ ] Document any compilation errors and fixes

#### 3.2.2 Janus Compliance Audit

Create automated audit script `scripts/janus_audit.sh`:

```bash
#!/bin/bash
# Scan for Janus violations in Icarus codebase

echo "=== Janus Compliance Audit ==="

# 1. Check for std:: math functions
echo -e "\n[1] Checking for std:: math usage..."
grep -rn --include="*.hpp" --include="*.cpp" \
    -E "std::(sin|cos|tan|sqrt|pow|exp|log|abs|fabs|atan|asin|acos)" \
    include/ src/ components/ || echo "  PASS: No std:: math found"

# 2. Check for if/else on Scalar (heuristic)
echo -e "\n[2] Checking for potential if/else on Scalar..."
grep -rn --include="*.hpp" --include="*.cpp" \
    -E "if\s*\([^)]*Scalar" \
    include/ src/ components/ || echo "  PASS: No obvious if/Scalar patterns"

# 3. Check for raw double in templates
echo -e "\n[3] Checking for hardcoded double in template functions..."
grep -rn --include="*.hpp" --include="*.cpp" \
    -E "template.*Scalar.*\n.*\bdouble\b" \
    include/ src/ components/ || echo "  PASS: No hardcoded double in templates"

echo -e "\n=== Audit Complete ==="
```

**Deliverables:**
- [ ] `scripts/janus_audit.sh`
- [ ] Fix any violations found
- [ ] Add audit to CI pipeline

#### 3.2.3 Component Template Verification

For each component, verify:

1. **Constructor takes `Scalar` parameters** (not `double`)
2. **No `std::` math calls** (use `janus::sin`, `janus::cos`, etc.)
3. **No `if/else` on `Scalar` values** (use `janus::where()`)
4. **Outputs declared with `Scalar` type**

| Component | Constructor | Math | Branching | Status |
|:----------|:------------|:-----|:----------|:-------|
| PointMass3DOF | ✓ | ✓ | ✓ | Ready |
| PointMassGravity | ✓ | ✓ | ✓ (switch on config) | Ready |

**Deliverables:**
- [ ] Audit checklist for each component
- [ ] Document any required fixes

---

### 3.3 Graph Export

**Goal:** Extract simulation dynamics as `janus::Function` for optimization.

#### 3.3.1 SymbolicTracer Utility

Create `include/icarus/symbolic/SymbolicTracer.hpp`:

```cpp
#include <janus/core/Function.hpp>

namespace icarus::symbolic {

/**
 * @brief Traces simulation dynamics to produce Janus Function
 *
 * Usage:
 *   auto sim = BuildSymbolicSimulator();
 *   SymbolicTracer tracer(sim);
 *   janus::Function dynamics = tracer.TraceDynamics();
 *
 *   // Evaluate numerically
 *   auto result = dynamics(t0, x0);  // Returns std::vector<janus::NumericMatrix>
 */
class SymbolicTracer {
public:
    explicit SymbolicTracer(Simulator<SymbolicScalar>& sim);

    /**
     * @brief Trace one Step() call to produce dynamics function
     * @return janus::Function with signature (t, x, u) -> (xdot)
     */
    janus::Function TraceDynamics();

    /**
     * @brief Get state variable names
     */
    std::vector<std::string> GetStateNames() const;

    /**
     * @brief Get input (control) signal names
     */
    std::vector<std::string> GetInputNames() const;

private:
    Simulator<SymbolicScalar>& sim_;
};

} // namespace icarus::symbolic
```

**Algorithm:**
1. Create symbolic state vector `x = janus::sym_vec("x", n_states)`
2. Scatter symbolic state to components via existing binding
3. Call `ComputeDerivatives(t_sym)` to populate `X_dot_global_`
4. Gather symbolic derivatives
5. Build `janus::Function("dynamics", {t, x}, {xdot})`

**Deliverables:**
- [ ] `include/icarus/symbolic/SymbolicTracer.hpp`
- [ ] `src/symbolic/SymbolicTracer.cpp`
- [ ] Unit tests verifying function output shape

#### 3.3.2 Simulator::GenerateGraph() Method

Add to `Simulator<Scalar>`:

```cpp
template <typename Scalar>
class Simulator {
public:
    // ... existing methods ...

    /**
     * @brief Generate Janus function representing system dynamics
     * @note Only available when Scalar = SymbolicScalar
     */
    template <typename S = Scalar>
    std::enable_if_t<std::is_same_v<S, SymbolicScalar>, janus::Function>
    GenerateGraph() const;
};
```

**Deliverables:**
- [ ] `GenerateGraph()` implementation using SymbolicTracer
- [ ] SFINAE guard for `<double>` instantiation (compile error if called)
- [ ] Unit test for graph generation

---

### 3.4 Symbolic Test Suite

**Goal:** Verify numeric/symbolic equivalence.

#### 3.4.1 Numeric/Symbolic Comparison Tests

Create `tests/symbolic/numeric_symbolic_comparison_test.cpp`:

```cpp
TEST(SymbolicMode, PointMass3DOF_NumericSymbolicMatch) {
    // 1. Run numeric simulation
    Simulator<double> numeric_sim;
    // ... setup ...
    numeric_sim.Step(dt);
    double numeric_x = numeric_sim.GetSignal("PointMass3DOF.position.x");

    // 2. Build symbolic function
    Simulator<SymbolicScalar> symbolic_sim;
    // ... identical setup ...
    janus::Function dynamics = symbolic_sim.GenerateGraph();

    // 3. Evaluate symbolic function at same inputs
    janus::NumericVector x0 = /* initial state */;
    auto result = dynamics.eval(t, x0);  // Returns janus::NumericMatrix

    // 4. Compare
    EXPECT_NEAR(numeric_x, result(0), 1e-10);
}
```

**Test Cases:**
- [ ] Point mass free fall
- [ ] Circular orbit (validates Vulcan gravity)
- [ ] Multiple components (wiring validation)
- [ ] Multi-step integration (accumulation errors)

**Deliverables:**
- [ ] `tests/symbolic/numeric_symbolic_comparison_test.cpp`
- [ ] Tolerance guidelines for comparison

#### 3.4.2 Derivative Extraction Test

```cpp
TEST(SymbolicMode, DerivativeExtraction) {
    Simulator<SymbolicScalar> sim;
    // ... setup ...

    janus::Function dynamics = sim.GenerateGraph();

    // Extract Jacobian via janus::jacobian (from AutoDiff.hpp)
    // Build symbolic inputs and compute jacobian symbolically
    auto t_sym = janus::sym("t");
    auto x_sym = janus::sym_vec("x", n_states);
    auto xdot_sym = dynamics.eval(t_sym, x_sym);  // Symbolic evaluation
    auto J_sym = janus::jacobian({janus::as_mx(xdot_sym)}, {janus::as_mx(x_sym)});

    // Wrap as function for numeric evaluation
    janus::Function jacobian_fn("jacobian", {t_sym, janus::as_mx(x_sym)}, {J_sym[0]});

    // Evaluate at specific point
    auto J = jacobian_fn.eval(t0, x0);  // Returns janus::NumericMatrix

    // Verify dimensions
    EXPECT_EQ(J.rows(), n_states);
    EXPECT_EQ(J.cols(), n_states);

    // Verify known analytical Jacobian for point mass
    // dxdot/dx for free fall: [[0, 0, 0], [0, 0, 0], [0, 0, 0]] (position)
    // dvdot/dv = [[0, 0, 0], [0, 0, 0], [0, 0, 0]] (velocity)
}
```

**Deliverables:**
- [ ] `tests/symbolic/derivative_extraction_test.cpp`
- [ ] Analytical Jacobian reference for validation

---

### 3.5 Logger Symbolic Mode Support

**Goal:** Logger works correctly when `Scalar = SymbolicScalar`.

#### 3.5.1 Conditional Formatting

The current logger formats `double` values. For symbolic mode, we need:

```cpp
// In MissionLogger or utility header
template <typename Scalar>
std::string FormatScalar(const Scalar& value) {
    if constexpr (std::is_same_v<Scalar, double>) {
        return std::to_string(value);
    } else {
        // For MX, show symbolic expression
        return value.get_str();
    }
}
```

**Deliverables:**
- [ ] `FormatScalar<Scalar>` utility
- [ ] Logger updates to use conditional formatting
- [ ] Test: symbolic sim runs without logger crashes

#### 3.5.2 Symbolic Mode Logger Option

```cpp
// Disable verbose logging in symbolic mode (traces can be huge)
sim.GetLogger().SetSymbolicMode(true);  // Suppresses per-step output
```

**Deliverables:**
- [ ] `SetSymbolicMode()` method
- [ ] Documentation on logger behavior in symbolic mode

---

### 3.6 Example Migration

**Goal:** Update examples to use new standardized API.

#### 3.6.1 Dual-Mode Example

Create `examples/symbolic/symbolic_orbital_demo.cpp`:

```cpp
#include <icarus/sim/SimulationBuilder.hpp>
#include <icarus/symbolic/SymbolicTracer.hpp>

int main() {
    // Demonstrate identical setup for both modes
    auto build_sim = []<typename Scalar>() {
        return SimulationBuilder<Scalar>()
            .AddComponent(std::make_unique<PointMassGravity<Scalar>>("Gravity"))
            .AddComponent(std::make_unique<PointMass3DOF<Scalar>>(Scalar{450000}, "Vehicle"))
            .Wire("Gravity", {{"position.x", "Vehicle.position.x"}, ...})
            .Wire("Vehicle", {{"force.x", "Gravity.force.x"}, ...})
            .SetIntegrator(std::make_unique<RK4Integrator<Scalar>>())
            .Build();
    };

    // Numeric execution
    auto numeric_sim = build_sim.template operator()<double>();
    SimulationRunner(numeric_sim).Run();

    // Symbolic graph export
    auto symbolic_sim = build_sim.template operator()<SymbolicScalar>();
    auto dynamics = symbolic_sim.GenerateGraph();
    std::cout << "Dynamics function: " << dynamics << "\n";
}
```

**Deliverables:**
- [ ] `examples/symbolic/symbolic_orbital_demo.cpp`
- [ ] Update `orbital_demo.cpp` to use SimulationBuilder
- [ ] Update `formation_flight_demo.cpp` to use SimulationBuilder

---

## File Structure

```
include/icarus/
├── sim/
│   ├── SimulationBuilder.hpp    [NEW]
│   ├── SimulationRunner.hpp     [NEW]
│   └── SimulationResults.hpp    [NEW]
├── symbolic/
│   └── SymbolicTracer.hpp       [NEW]
└── util/
    └── ScalarFormat.hpp         [NEW]

src/
└── symbolic/
    └── SymbolicTracer.cpp       [NEW]

tests/symbolic/
├── symbolic_compilation_test.cpp       [NEW]
├── numeric_symbolic_comparison_test.cpp [NEW]
└── derivative_extraction_test.cpp      [NEW]

examples/symbolic/
└── symbolic_orbital_demo.cpp    [NEW]

scripts/
└── janus_audit.sh               [NEW]
```

---

## Exit Criteria

- [ ] `Simulator<SymbolicScalar>` compiles with all existing components
- [ ] `GenerateGraph()` returns valid `janus::Function`
- [ ] Numeric/symbolic outputs match within tolerance (< 1e-10)
- [ ] Jacobian extraction works for point mass dynamics
- [ ] Zero Janus violations in codebase (audit passes)
- [ ] Examples migrated to SimulationBuilder pattern
- [ ] CI includes symbolic mode build and test

---

## Risk Mitigation

| Risk | Mitigation |
|:-----|:-----------|
| Vulcan functions don't support MX | Verify Vulcan templates early; add wrappers if needed |
| MX memory issues in deque | Profile memory; consider MX pooling if needed |
| Logger crashes with MX | Add symbolic mode flag; conditional formatting |
| CI build time increases | Separate symbolic tests into optional CI job |

---

## Dependencies

- **Janus**: Must support all math operations used by Icarus (provides `janus::Function`, `janus::sym()`, `janus::Opti`)
- **Vulcan**: Physics functions must be templated on Scalar
- **CasADi**: Available via Janus (wrapped by `janus::SymbolicScalar`, `janus::Function`)

---

## Estimated Complexity

| Task | New Files | Lines of Code |
|:-----|:----------|:--------------|
| 3.1 SimulationBuilder/Runner | 3 | ~400 |
| 3.2 Dual-Mode Validation | 2 | ~200 |
| 3.3 Graph Export | 2 | ~300 |
| 3.4 Symbolic Test Suite | 3 | ~400 |
| 3.5 Logger Support | 1 | ~100 |
| 3.6 Example Migration | 2 | ~200 |
| **Total** | **13** | **~1600** |

---

## Next Steps

1. **Start with 3.2.1**: Attempt `Simulator<SymbolicScalar>` compilation
2. **Run 3.2.2**: Execute Janus audit to identify violations
3. **Fix any issues** found in steps 1-2
4. **Implement 3.1**: SimulationBuilder pattern (parallelizable with above)
5. **Implement 3.3**: Graph export once compilation passes
6. **Complete 3.4**: Test suite for validation
7. **Migrate examples**: After API stabilizes
