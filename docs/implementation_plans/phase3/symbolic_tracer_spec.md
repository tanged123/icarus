# SymbolicTracer Technical Specification

**Component:** Graph Export for Optimization & AD
**Phase:** 3.3
**Status:** Proposed

---

## Purpose

The SymbolicTracer extracts the computational graph from a `Simulator<SymbolicScalar>` as a `janus::Function`. This enables:

1. **Automatic Differentiation**: Jacobians, Hessians for sensitivity analysis
2. **Optimization**: NLP formulation for trajectory optimization
3. **Code Generation**: Export to C, MATLAB, or other backends
4. **Verification**: Compare symbolic vs numeric outputs

---

## Design

### Core Concept

The tracer works by:
1. Creating symbolic variables for state and time using `janus::sym()`
2. "Playing" these through the simulation's `ComputeDerivatives()` path
3. Capturing the resulting symbolic expressions for state derivatives
4. Packaging as a `janus::Function`

```
                    Simulator<SymbolicScalar>
                          |
     [sym t]─────────────┼───────────────┐
     [sym x]─────────────┼───────────────┤
     [sym u]─────────────┼───────────────┤
                         │               │
                         ▼               │
              ┌──────────────────┐       │
              │ ComputeDerivatives│       │
              │   (symbolic)      │       │
              └────────┬─────────┘       │
                       │                 │
                       ▼                 │
                  [sym xdot]             │
                       │                 │
                       ▼                 ▼
              ┌─────────────────────────────────────┐
              │ janus::Function("dynamics",         │
              │   {t, x, u}, {xdot})                │
              └─────────────────────────────────────┘
```

### Interface

```cpp
#pragma once

#include <icarus/sim/Simulator.hpp>
#include <icarus/core/Types.hpp>

#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>

#include <string>
#include <vector>

namespace icarus::symbolic {

/**
 * @brief Configuration for symbolic tracing
 */
struct TracerConfig {
    bool include_time = true;          ///< Include time as input
    bool include_controls = true;      ///< Include control inputs
    std::string function_name = "dynamics";
    std::vector<std::string> control_signals;  ///< Override auto-detected controls
};

/**
 * @brief Extracts computational graph from symbolic simulator
 *
 * The tracer produces a janus::Function representing the system dynamics:
 *   xdot = f(t, x, u)
 *
 * Where:
 *   - t: scalar time
 *   - x: state vector (all integrated states)
 *   - u: control vector (inputs marked as controls)
 *   - xdot: state derivative vector
 */
class SymbolicTracer {
public:
    /**
     * @brief Construct tracer for a symbolic simulator
     * @param sim Reference to initialized Simulator<SymbolicScalar>
     *
     * The simulator must be in Stage phase (Provision and Stage called).
     */
    explicit SymbolicTracer(Simulator<SymbolicScalar>& sim);

    /**
     * @brief Configure tracing behavior
     */
    SymbolicTracer& Configure(const TracerConfig& config);

    // =========================================================================
    // Graph Extraction
    // =========================================================================

    /**
     * @brief Trace dynamics to produce janus::Function
     * @return Function with signature (t, x, [u]) -> (xdot)
     *
     * The function is suitable for:
     * - Direct evaluation: dynamics(t_val, x_val) -> std::vector<NumericMatrix>
     * - Jacobian via janus::jacobian()
     * - Integration: Use in janus::integrate
     * - NLP formulation: Add as constraint in janus::Opti
     */
    janus::Function TraceDynamics();

    /**
     * @brief Trace outputs (all signals, not just states)
     * @return Function with signature (t, x, [u]) -> (outputs)
     *
     * Useful for computing observables like altitude, velocity magnitude.
     */
    janus::Function TraceOutputs(const std::vector<std::string>& output_signals);

    /**
     * @brief Trace single step including integrator
     * @param integrator_type "rk4", "euler", etc.
     * @param dt Step size
     * @return Function with signature (t, x, [u], dt) -> (x_next)
     */
    janus::Function TraceStep(const std::string& integrator_type,
                              SymbolicScalar dt);

    // =========================================================================
    // Metadata
    // =========================================================================

    /**
     * @brief Get state variable names in order
     */
    std::vector<std::string> GetStateNames() const;

    /**
     * @brief Get state vector size
     */
    size_t GetStateSize() const;

    /**
     * @brief Get control (input) signal names
     */
    std::vector<std::string> GetControlNames() const;

    /**
     * @brief Get control vector size
     */
    size_t GetControlSize() const;

    /**
     * @brief Get output signal names
     */
    std::vector<std::string> GetOutputNames() const;

private:
    Simulator<SymbolicScalar>& sim_;
    TracerConfig config_;

    // Cached symbolic variables (using Janus types)
    SymbolicScalar t_sym_;
    janus::SymbolicVector x_sym_;
    janus::SymbolicVector u_sym_;

    // Internal helpers
    void CreateSymbolicVariables();
    void ScatterSymbolicState();
    void GatherSymbolicDerivatives(janus::SymbolicVector& xdot);
    std::vector<std::string> DetectControlSignals() const;
};

// =============================================================================
// Convenience Functions
// =============================================================================

/**
 * @brief Quick dynamics extraction (uses default config)
 */
inline janus::Function ExtractDynamics(Simulator<SymbolicScalar>& sim) {
    return SymbolicTracer(sim).TraceDynamics();
}

} // namespace icarus::symbolic
```

---

## Implementation Details

### Step 1: Create Symbolic Variables

```cpp
void SymbolicTracer::CreateSymbolicVariables() {
    // Time (scalar symbolic variable)
    t_sym_ = janus::sym("t");

    // State vector (from simulator's state registry)
    size_t n_states = sim_.GetTotalStateSize();
    x_sym_ = janus::sym_vec("x", n_states);

    // Control vector (detected or configured)
    auto control_names = config_.control_signals.empty()
        ? DetectControlSignals()
        : config_.control_signals;
    size_t n_controls = control_names.size();
    u_sym_ = janus::sym_vec("u", n_controls);
}
```

### Step 2: Scatter State to Components

```cpp
void SymbolicTracer::ScatterSymbolicState() {
    // Get simulator's global state pointer (JanusVector<SymbolicScalar>)
    auto& X_global = sim_.GetStateVector();

    // Replace numeric values with symbolic
    for (size_t i = 0; i < X_global.size(); ++i) {
        X_global(i) = x_sym_(i);
    }

    // Scatter to component-owned state pointers
    // (This happens automatically through pointer binding)
}
```

### Step 3: Evaluate Derivatives

```cpp
janus::Function SymbolicTracer::TraceDynamics() {
    CreateSymbolicVariables();
    ScatterSymbolicState();

    // Set simulation time to symbolic
    sim_.SetTime(t_sym_);

    // Compute derivatives (this traces the symbolic graph)
    sim_.ComputeDerivatives(t_sym_);

    // Gather derivatives
    janus::SymbolicVector xdot;
    GatherSymbolicDerivatives(xdot);

    // Build function using janus::Function
    // Convert to SymbolicArg for janus::Function constructor
    std::vector<janus::SymbolicArg> inputs = {t_sym_, x_sym_};

    if (config_.include_controls && u_sym_.size() > 0) {
        inputs.push_back(u_sym_);
    }

    return janus::Function(config_.function_name, inputs, {xdot});
}
```

### Step 4: Gather Derivatives

```cpp
void SymbolicTracer::GatherSymbolicDerivatives(janus::SymbolicVector& xdot) {
    auto& X_dot_global = sim_.GetStateDerivativeVector();

    xdot.resize(X_dot_global.size());
    for (size_t i = 0; i < X_dot_global.size(); ++i) {
        xdot(i) = X_dot_global(i);
    }
}
```

---

## Control Signal Detection

Control signals are inputs that:
1. Are wired from external sources (not other simulation components)
2. Influence the dynamics (appear in derivative computation)

```cpp
std::vector<std::string> SymbolicTracer::DetectControlSignals() const {
    std::vector<std::string> controls;

    // Get all input signals
    auto& dict = sim_.GetDataDictionary();
    for (const auto& [name, desc] : dict) {
        if (desc.kind == SignalKind::Input) {
            // Check if wired to external source
            if (desc.wired_to.empty() ||
                !sim_.HasSignal(desc.wired_to)) {
                controls.push_back(name);
            }
        }
    }

    return controls;
}
```

---

## Usage Examples

### Basic Dynamics Extraction

```cpp
#include <icarus/symbolic/SymbolicTracer.hpp>

// Build symbolic simulator
Simulator<SymbolicScalar> sim;
// ... add components, wire, provision, stage ...

// Extract dynamics
auto dynamics = icarus::symbolic::ExtractDynamics(sim);

// Evaluate at specific point using janus::Function
double t0 = 0.0;
janus::NumericVector x0 = janus::NumericVector::Zero(6);  // 3 pos + 3 vel
x0(0) = 6.778e6;  // Initial radius (Earth + 400km)
x0(4) = 7660.0;   // Circular velocity

auto result = dynamics(t0, x0);  // Returns std::vector<janus::NumericMatrix>
auto xdot = result[0];
std::cout << "State derivatives:\n" << xdot << "\n";
```

### Jacobian for Linearization

```cpp
#include <janus/math/AutoDiff.hpp>

auto dynamics = tracer.TraceDynamics();

// Build Jacobian symbolically
auto t_sym = janus::sym("t");
auto x_sym = janus::sym_vec("x", 6);

// Evaluate dynamics symbolically
auto xdot_sym = dynamics.eval(t_sym, x_sym);

// Compute Jacobian using janus::jacobian
auto J_sym = janus::jacobian({janus::as_mx(xdot_sym)}, {janus::as_mx(x_sym)});

// Wrap as function for numeric evaluation
janus::Function jacobian_fn("A", {t_sym, janus::as_mx(x_sym)}, {J_sym[0]});

// Evaluate at equilibrium point
auto A_numeric = jacobian_fn.eval(t0, x_eq);
std::cout << "Linearized A matrix:\n" << A_numeric << "\n";

// Eigenvalue analysis for stability
// (Use Eigen::EigenSolver)
```

### Trajectory Optimization with Opti

```cpp
#include <janus/optimization/Opti.hpp>

// Extract dynamics
auto f = tracer.TraceDynamics();

// Build NLP using janus::Opti
janus::Opti opti;
int N = 100;  // Horizon
double dt = 1.0;

auto X = opti.variable(6, N+1);  // States
auto U = opti.variable(3, N);    // Controls

// Initial condition
opti.subject_to(X.col(0) == x0);

// Dynamics constraints (direct collocation or shooting)
for (int k = 0; k < N; ++k) {
    auto xk = X.col(k);
    auto uk = U.col(k);

    // Evaluate dynamics symbolically
    auto xdot = f.eval(t0 + k*dt, xk, uk);
    opti.subject_to(X.col(k+1) == xk + dt * xdot);
}

// Objective: minimize fuel (sum of squared controls)
opti.minimize(U.squaredNorm());

// Solve
opti.solver("ipopt");
auto sol = opti.solve();
```

### Code Generation

```cpp
auto dynamics = tracer.TraceDynamics();

// Access underlying CasADi function for code generation
auto& casadi_fn = dynamics.casadi_function();

// Generate C code
casadi_fn.generate("dynamics");  // Creates dynamics.c

// Generate for MATLAB
casadi::Dict opts;
opts["mex"] = true;
casadi_fn.generate("dynamics_mex", opts);
```

---

## Testing Strategy

### Unit Tests

```cpp
// tests/symbolic/symbolic_tracer_test.cpp

TEST(SymbolicTracer, DynamicsFunctionDimensions) {
    auto sim = BuildSymbolicPointMassSim();
    SymbolicTracer tracer(sim);

    auto dynamics = tracer.TraceDynamics();

    // Evaluate to check output dimensions
    // Point mass: 6 states (3 pos + 3 vel)
    janus::NumericVector x0 = janus::NumericVector::Zero(6);
    auto result = dynamics(0.0, x0);

    EXPECT_EQ(result[0].rows(), 6);
    EXPECT_EQ(result[0].cols(), 1);
}

TEST(SymbolicTracer, JacobianExtraction) {
    auto sim = BuildSymbolicPointMassSim();
    auto dynamics = SymbolicTracer(sim).TraceDynamics();

    // Build Jacobian symbolically
    auto t_sym = janus::sym("t");
    auto x_sym = janus::sym_vec("x", 6);
    auto xdot_sym = dynamics.eval(t_sym, x_sym);
    auto J_sym = janus::jacobian({janus::as_mx(xdot_sym)}, {janus::as_mx(x_sym)});

    janus::Function jacobian_fn("J", {t_sym, janus::as_mx(x_sym)}, {J_sym[0]});

    // Evaluate at a point
    janus::NumericVector x0 = janus::NumericVector::Zero(6);
    auto J = jacobian_fn.eval(0.0, x0);

    // Jacobian should be 6x6
    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), 6);
}

TEST(SymbolicTracer, NumericEvaluation) {
    // Setup symbolic sim
    auto sym_sim = BuildSymbolicPointMassSim();
    auto dynamics = SymbolicTracer(sym_sim).TraceDynamics();

    // Setup numeric sim with same config
    auto num_sim = BuildNumericPointMassSim();

    // Initial state
    janus::NumericVector x0 = GetInitialState();
    double t0 = 0.0;

    // Symbolic function evaluation (returns numeric)
    auto result = dynamics(t0, x0);
    auto xdot_sym = result[0];

    // Numeric simulation evaluation
    num_sim.ComputeDerivatives(0.0);
    auto xdot_num = num_sim.GetStateDerivativeVector();

    // Compare
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_NEAR(xdot_sym(i), xdot_num(i), 1e-12);
    }
}
```

---

## Integration with Simulator

Add to `Simulator<Scalar>`:

```cpp
template <typename Scalar>
class Simulator {
public:
    // ... existing interface ...

    /**
     * @brief Generate janus::Function from dynamics
     * @note Only available when Scalar = SymbolicScalar
     *
     * Requires simulator to be in Stage phase.
     */
    template <typename S = Scalar>
    std::enable_if_t<std::is_same_v<S, SymbolicScalar>, janus::Function>
    GenerateGraph() const {
        return symbolic::SymbolicTracer(const_cast<Simulator&>(*this))
            .TraceDynamics();
    }

    // Allow tracer to access internals
    friend class symbolic::SymbolicTracer;

protected:
    // Expose for tracing
    JanusVector<Scalar>& GetStateVector() { return X_global_; }
    JanusVector<Scalar>& GetStateDerivativeVector() { return X_dot_global_; }
};
```

---

## Performance Considerations

1. **Tracing is One-Time**: Graph built once, evaluated many times
2. **Graph Size**: Complex simulations may produce large symbolic graphs
3. **JIT Compilation**: Use `dynamics.casadi_function().generate()` for hot paths
4. **Sparsity**: Janus/CasADi automatically detects sparsity in Jacobians

---

## Future Extensions

1. **Sensitivity Analysis**: Expose parameter sensitivities
2. **Adjoint Mode**: Reverse-mode AD for large state spaces
3. **Event Detection**: Symbolic root-finding for phase transitions
4. **Multi-Rate**: Handle components at different rates
