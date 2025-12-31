# Phase 4 Staging Implementation Plan

**Goal:** Implement trim optimization, linearization, and symbolic graph generation through the `Stage()` lifecycle method.

---

## Overview

The staging system supports two modes:

| Mode | Backend | Use Case | Accuracy |
|:-----|:--------|:---------|:---------|
| **Numeric** | Finite differences | Fast trim, quick linearization | O(h²) |
| **Symbolic** | CasADi AD | Exact Jacobians, graph export | Machine precision |

Both modes use the same configuration (`TrimConfig`, `LinearizationConfig`, `SymbolicsConfig`) and produce compatible outputs.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Simulator::Stage()                      │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Component Stage() calls (existing)                       │
│  2. Wiring validation (existing)                             │
│  3. ─────────── NEW STAGING FEATURES ───────────            │
│     │                                                        │
│     ├── if symbolics.enabled:                                │
│     │   └── SymbolicStager::GenerateGraphs()                │
│     │       ├── Create SymbolicSimulatorCore                │
│     │       ├── Trace dynamics → dynamics_graph_            │
│     │       └── Trace Jacobian → jacobian_                  │
│     │                                                        │
│     ├── if trim.enabled:                                     │
│     │   └── TrimSolver::Solve()                             │
│     │       ├── Numeric: FiniteDifferenceTrim               │
│     │       └── Symbolic: SymbolicTrim (Newton/IPOPT)       │
│     │                                                        │
│     └── if linearization.enabled:                            │
│         └── Linearizer::Compute()                            │
│             ├── Numeric: FiniteDifferenceLinearizer         │
│             └── Symbolic: SymbolicLinearizer                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## File Structure

```
include/icarus/staging/
├── StagingTypes.hpp           # LinearModel, TrimResult structs
├── TrimSolver.hpp             # Abstract trim interface + implementations
├── Linearizer.hpp             # Abstract linearizer + implementations
├── SymbolicSimulatorCore.hpp  # Lightweight symbolic component manager
└── SymbolicStager.hpp         # Symbolic graph generation
```

---

## Part 1: Core Types

### File: `include/icarus/staging/StagingTypes.hpp`

```cpp
#pragma once

#include <icarus/core/Types.hpp>
#include <janus/core/Function.hpp>
#include <string>
#include <vector>
#include <optional>

namespace icarus::staging {

/**
 * @brief Result of trim optimization
 */
struct TrimResult {
    bool converged = false;
    int iterations = 0;
    double residual_norm = 0.0;
    std::string message;

    /// Final control values (control_name -> value)
    std::unordered_map<std::string, double> controls;

    /// Final derivative residuals (derivative_name -> value)
    std::unordered_map<std::string, double> residuals;
};

/**
 * @brief Linear state-space model
 *
 * ẋ = Ax + Bu
 * y = Cx + Du
 */
struct LinearModel {
    Eigen::MatrixXd A;  ///< State matrix (n_states × n_states)
    Eigen::MatrixXd B;  ///< Input matrix (n_states × n_inputs)
    Eigen::MatrixXd C;  ///< Output matrix (n_outputs × n_states)
    Eigen::MatrixXd D;  ///< Feedthrough matrix (n_outputs × n_inputs)

    std::vector<std::string> state_names;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;

    /// Operating point where linearization was performed
    Eigen::VectorXd x0;
    Eigen::VectorXd u0;
    double t0 = 0.0;

    // Export methods
    void ExportMatlab(const std::string& path) const;
    void ExportNumPy(const std::string& path) const;
    void ExportJSON(const std::string& path) const;

    /// Check controllability rank
    [[nodiscard]] int ControllabilityRank() const;

    /// Check observability rank
    [[nodiscard]] int ObservabilityRank() const;
};

/**
 * @brief Symbolic dynamics representation
 */
struct SymbolicDynamics {
    std::optional<janus::Function> dynamics;   ///< f(t, x) -> xdot
    std::optional<janus::Function> jacobian_x; ///< df/dx
    std::optional<janus::Function> jacobian_u; ///< df/du (if controls specified)

    std::vector<std::string> state_names;
    std::vector<std::string> control_names;
};

} // namespace icarus::staging
```

---

## Part 2: Numeric Trim (Finite Differences)

### Approach

Uses the existing numeric `Simulator` with finite difference Jacobians:

1. Define residual function: `F(u) = selected_derivatives`
2. Compute Jacobian `dF/du` via finite differences
3. Newton iteration: `u_new = u - J⁻¹ * F(u)`
4. Apply bounds via projection

### File: `include/icarus/staging/TrimSolver.hpp`

```cpp
#pragma once

#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/staging/StagingTypes.hpp>
#include <functional>

namespace icarus::staging {

// Forward declaration
class Simulator;

/**
 * @brief Abstract trim solver interface
 */
class TrimSolver {
public:
    virtual ~TrimSolver() = default;

    /**
     * @brief Solve trim problem
     * @param sim Simulator to trim (will be modified)
     * @param config Trim configuration
     * @return Trim result
     */
    virtual TrimResult Solve(Simulator& sim, const TrimConfig& config) = 0;
};

/**
 * @brief Numeric trim using finite differences
 *
 * Simple and robust. Uses central differences for Jacobian.
 * Works with existing double-typed Simulator.
 */
class FiniteDifferenceTrim : public TrimSolver {
public:
    struct Options {
        double step_size = 1e-7;      ///< Finite difference step
        double tolerance = 1e-6;       ///< Convergence tolerance
        int max_iterations = 100;      ///< Maximum Newton iterations
        double damping = 1.0;          ///< Newton step damping (0-1]
        bool verbose = false;
    };

    explicit FiniteDifferenceTrim(Options opts = {}) : opts_(opts) {}

    TrimResult Solve(Simulator& sim, const TrimConfig& config) override;

private:
    Options opts_;

    /// Evaluate residual (selected derivatives)
    Eigen::VectorXd EvaluateResidual(
        Simulator& sim,
        const std::vector<std::string>& zero_derivatives,
        double t);

    /// Compute Jacobian via central differences
    Eigen::MatrixXd ComputeJacobian(
        Simulator& sim,
        const std::vector<std::string>& control_signals,
        const std::vector<std::string>& zero_derivatives,
        double t);

    /// Apply control bounds (projection)
    void ApplyBounds(
        Eigen::VectorXd& u,
        const std::vector<std::string>& control_signals,
        const std::unordered_map<std::string, std::pair<double, double>>& bounds);
};

/**
 * @brief Symbolic trim using janus::NewtonSolver
 *
 * Requires symbolic components. Provides exact Jacobians.
 * Falls back to IPOPT for inequality constraints.
 */
class SymbolicTrim : public TrimSolver {
public:
    TrimResult Solve(Simulator& sim, const TrimConfig& config) override;

private:
    /// Build symbolic residual function
    janus::Function BuildResidualFunction(
        SymbolicSimulatorCore& sym_sim,
        const TrimConfig& config);
};

} // namespace icarus::staging
```

### Implementation: `FiniteDifferenceTrim::Solve()`

```cpp
TrimResult FiniteDifferenceTrim::Solve(Simulator& sim, const TrimConfig& config) {
    TrimResult result;

    const auto& controls = config.control_signals;
    const auto& derivs = config.zero_derivatives;
    const int n_controls = static_cast<int>(controls.size());
    const int n_residuals = static_cast<int>(derivs.size());

    // Get current control values as initial guess
    Eigen::VectorXd u(n_controls);
    for (int i = 0; i < n_controls; ++i) {
        auto it = config.initial_guesses.find(controls[i]);
        u(i) = (it != config.initial_guesses.end())
               ? it->second
               : sim.Peek(controls[i]);
    }

    double t = sim.Time();

    // Newton iteration
    for (int iter = 0; iter < opts_.max_iterations; ++iter) {
        // Apply current controls
        for (int i = 0; i < n_controls; ++i) {
            sim.Poke(controls[i], u(i));
        }

        // Evaluate residual
        Eigen::VectorXd F = EvaluateResidual(sim, derivs, t);
        double norm = F.norm();

        if (opts_.verbose) {
            std::cout << "Trim iter " << iter << ": ||F|| = " << norm << "\n";
        }

        // Check convergence
        if (norm < opts_.tolerance) {
            result.converged = true;
            result.iterations = iter;
            result.residual_norm = norm;
            result.message = "Converged";
            break;
        }

        // Compute Jacobian
        Eigen::MatrixXd J = ComputeJacobian(sim, controls, derivs, t);

        // Solve J * du = -F
        Eigen::VectorXd du = J.colPivHouseholderQr().solve(-F);

        // Update with damping
        u += opts_.damping * du;

        // Apply bounds
        ApplyBounds(u, controls, config.control_bounds);

        result.iterations = iter + 1;
        result.residual_norm = norm;
    }

    if (!result.converged) {
        result.message = "Max iterations reached";
    }

    // Store final values
    for (int i = 0; i < n_controls; ++i) {
        result.controls[controls[i]] = u(i);
        sim.Poke(controls[i], u(i));  // Apply final controls
    }

    // Store final residuals
    Eigen::VectorXd F_final = EvaluateResidual(sim, derivs, t);
    for (int i = 0; i < n_residuals; ++i) {
        result.residuals[derivs[i]] = F_final(i);
    }

    return result;
}

Eigen::VectorXd FiniteDifferenceTrim::EvaluateResidual(
    Simulator& sim,
    const std::vector<std::string>& zero_derivatives,
    double t)
{
    // Compute all derivatives
    sim.ComputeDerivatives(t);

    // Extract selected derivatives
    const int n = static_cast<int>(zero_derivatives.size());
    Eigen::VectorXd F(n);
    for (int i = 0; i < n; ++i) {
        F(i) = sim.Peek(zero_derivatives[i]);
    }
    return F;
}

Eigen::MatrixXd FiniteDifferenceTrim::ComputeJacobian(
    Simulator& sim,
    const std::vector<std::string>& control_signals,
    const std::vector<std::string>& zero_derivatives,
    double t)
{
    const int n_controls = static_cast<int>(control_signals.size());
    const int n_residuals = static_cast<int>(zero_derivatives.size());
    const double h = opts_.step_size;

    Eigen::MatrixXd J(n_residuals, n_controls);

    for (int j = 0; j < n_controls; ++j) {
        double u0 = sim.Peek(control_signals[j]);

        // Forward perturbation
        sim.Poke(control_signals[j], u0 + h);
        Eigen::VectorXd F_plus = EvaluateResidual(sim, zero_derivatives, t);

        // Backward perturbation
        sim.Poke(control_signals[j], u0 - h);
        Eigen::VectorXd F_minus = EvaluateResidual(sim, zero_derivatives, t);

        // Central difference
        J.col(j) = (F_plus - F_minus) / (2.0 * h);

        // Restore original
        sim.Poke(control_signals[j], u0);
    }

    return J;
}
```

---

## Part 3: Symbolic Trim

### Prerequisites

1. **Dual Component Registration** - Components must be registered for both `double` and `SymbolicScalar`
2. **SymbolicSimulatorCore** - Lightweight symbolic component manager

### File: `include/icarus/staging/SymbolicSimulatorCore.hpp`

```cpp
#pragma once

#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentFactory.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/sim/SimulatorConfig.hpp>

namespace icarus::staging {

/**
 * @brief Lightweight symbolic simulator for graph extraction
 *
 * Creates symbolic components from the same configs as the numeric sim.
 * Used internally during Stage() for trim/linearization/graph export.
 */
class SymbolicSimulatorCore {
public:
    /**
     * @brief Create from simulator config
     * @throws ConfigError if symbolic components can't be created
     */
    explicit SymbolicSimulatorCore(const SimulatorConfig& config);

    /// Set state vector (symbolic)
    void SetState(const JanusVector<SymbolicScalar>& x);

    /// Set time (symbolic)
    void SetTime(SymbolicScalar t);

    /// Compute derivatives symbolically
    JanusVector<SymbolicScalar> ComputeDerivatives();

    /// Get total state size
    [[nodiscard]] std::size_t GetStateSize() const;

    /// Read signal value (symbolic)
    [[nodiscard]] SymbolicScalar GetSignal(const std::string& name) const;

    /// Write signal value (symbolic)
    void SetSignal(const std::string& name, SymbolicScalar value);

    /// Get all registered signal names
    [[nodiscard]] std::vector<std::string> GetSignalNames() const;

private:
    std::vector<std::unique_ptr<Component<SymbolicScalar>>> components_;
    SignalRegistry<SymbolicScalar> registry_;
    Backplane<SymbolicScalar> backplane_;

    JanusVector<SymbolicScalar> state_;
    JanusVector<SymbolicScalar> derivatives_;
    SymbolicScalar time_;
};

} // namespace icarus::staging
```

### Updated Registration Macro

```cpp
// In ComponentFactory.hpp

/**
 * @brief Register component for BOTH double and SymbolicScalar
 */
#define ICARUS_REGISTER_COMPONENT(ComponentType)                                    \
    namespace {                                                                     \
    static bool _reg_double_##ComponentType = []() {                                \
        ::icarus::ComponentFactory<double>::Instance().Register(                    \
            #ComponentType, [](const ::icarus::ComponentConfig &config) {           \
                auto comp = std::make_unique<ComponentType<double>>(                \
                    config.name, config.entity);                                    \
                comp->SetConfig(config);                                            \
                return comp;                                                        \
            });                                                                     \
        return true;                                                                \
    }();                                                                            \
    static bool _reg_symbolic_##ComponentType = []() {                              \
        ::icarus::ComponentFactory<SymbolicScalar>::Instance().Register(            \
            #ComponentType, [](const ::icarus::ComponentConfig &config) {           \
                auto comp = std::make_unique<ComponentType<SymbolicScalar>>(        \
                    config.name, config.entity);                                    \
                comp->SetConfig(config);                                            \
                return comp;                                                        \
            });                                                                     \
        return true;                                                                \
    }();                                                                            \
    }
```

### Symbolic Trim Implementation

```cpp
TrimResult SymbolicTrim::Solve(Simulator& sim, const TrimConfig& config) {
    // Create symbolic simulator from same config
    SymbolicSimulatorCore sym_sim(sim.GetConfig());

    // Build symbolic residual function: F(u) -> derivatives
    janus::Function F = BuildResidualFunction(sym_sim, config);

    // Configure Newton solver
    janus::RootFinderOptions opts;
    opts.abstol = config.tolerance;
    opts.max_iter = config.max_iterations;
    opts.line_search = true;

    janus::NewtonSolver solver(F, opts);

    // Get initial guess
    const int n_controls = static_cast<int>(config.control_signals.size());
    Eigen::VectorXd u0(n_controls);
    for (int i = 0; i < n_controls; ++i) {
        auto it = config.initial_guesses.find(config.control_signals[i]);
        u0(i) = (it != config.initial_guesses.end())
                ? it->second
                : sim.Peek(config.control_signals[i]);
    }

    // Solve
    auto root_result = solver.solve(u0);

    // Convert to TrimResult
    TrimResult result;
    result.converged = root_result.converged;
    result.iterations = root_result.iterations;
    result.message = root_result.message;

    if (result.converged) {
        for (int i = 0; i < n_controls; ++i) {
            result.controls[config.control_signals[i]] = root_result.x(i);
            sim.Poke(config.control_signals[i], root_result.x(i));
        }
    }

    return result;
}

janus::Function SymbolicTrim::BuildResidualFunction(
    SymbolicSimulatorCore& sym_sim,
    const TrimConfig& config)
{
    const int n_controls = static_cast<int>(config.control_signals.size());
    const int n_residuals = static_cast<int>(config.zero_derivatives.size());

    // Create symbolic control variables
    auto [u_vec, u_mx] = janus::sym_vec_pair("u", n_controls);

    // Apply symbolic controls to simulator
    for (int i = 0; i < n_controls; ++i) {
        sym_sim.SetSignal(config.control_signals[i], u_vec(i));
    }

    // Compute derivatives symbolically
    sym_sim.ComputeDerivatives();

    // Gather residuals
    std::vector<SymbolicScalar> residual_elements;
    for (const auto& deriv_name : config.zero_derivatives) {
        residual_elements.push_back(sym_sim.GetSignal(deriv_name));
    }

    SymbolicScalar residuals = SymbolicScalar::vertcat(residual_elements);

    return janus::Function("trim_residual", {u_mx}, {residuals});
}
```

---

## Part 4: Linearization

### File: `include/icarus/staging/Linearizer.hpp`

```cpp
#pragma once

#include <icarus/staging/StagingTypes.hpp>
#include <icarus/sim/SimulatorConfig.hpp>

namespace icarus::staging {

class Simulator;

/**
 * @brief Abstract linearizer interface
 */
class Linearizer {
public:
    virtual ~Linearizer() = default;

    /**
     * @brief Compute linear model at current operating point
     */
    virtual LinearModel Compute(
        Simulator& sim,
        const LinearizationConfig& config) = 0;
};

/**
 * @brief Linearizer using finite differences
 */
class FiniteDifferenceLinearizer : public Linearizer {
public:
    struct Options {
        double step_size = 1e-7;
    };

    explicit FiniteDifferenceLinearizer(Options opts = {}) : opts_(opts) {}

    LinearModel Compute(Simulator& sim, const LinearizationConfig& config) override;

private:
    Options opts_;
};

/**
 * @brief Linearizer using symbolic Jacobians
 */
class SymbolicLinearizer : public Linearizer {
public:
    LinearModel Compute(Simulator& sim, const LinearizationConfig& config) override;
};

} // namespace icarus::staging
```

### Implementation: `FiniteDifferenceLinearizer::Compute()`

```cpp
LinearModel FiniteDifferenceLinearizer::Compute(
    Simulator& sim,
    const LinearizationConfig& config)
{
    LinearModel model;
    model.state_names = config.states;
    model.input_names = config.inputs;
    model.output_names = config.outputs;

    const int nx = static_cast<int>(config.states.size());
    const int nu = static_cast<int>(config.inputs.size());
    const int ny = static_cast<int>(config.outputs.size());
    const double h = opts_.step_size;
    const double t = sim.Time();

    // Store operating point
    model.x0.resize(nx);
    for (int i = 0; i < nx; ++i) {
        model.x0(i) = sim.Peek(config.states[i]);
    }
    model.u0.resize(nu);
    for (int i = 0; i < nu; ++i) {
        model.u0(i) = sim.Peek(config.inputs[i]);
    }
    model.t0 = t;

    // Helper: get state derivatives
    auto get_xdot = [&]() {
        sim.ComputeDerivatives(t);
        Eigen::VectorXd xdot(nx);
        for (int i = 0; i < nx; ++i) {
            // Assuming derivative signal is state_name + "_dot"
            // Or we need a mapping from states to their derivatives
            xdot(i) = sim.Peek(config.states[i] + "_dot");
        }
        return xdot;
    };

    // Helper: get outputs
    auto get_y = [&]() {
        Eigen::VectorXd y(ny);
        for (int i = 0; i < ny; ++i) {
            y(i) = sim.Peek(config.outputs[i]);
        }
        return y;
    };

    // Compute A = df/dx (central differences)
    model.A.resize(nx, nx);
    for (int j = 0; j < nx; ++j) {
        double x0_j = sim.Peek(config.states[j]);

        sim.Poke(config.states[j], x0_j + h);
        Eigen::VectorXd f_plus = get_xdot();

        sim.Poke(config.states[j], x0_j - h);
        Eigen::VectorXd f_minus = get_xdot();

        model.A.col(j) = (f_plus - f_minus) / (2.0 * h);

        sim.Poke(config.states[j], x0_j);  // Restore
    }

    // Compute B = df/du
    model.B.resize(nx, nu);
    for (int j = 0; j < nu; ++j) {
        double u0_j = sim.Peek(config.inputs[j]);

        sim.Poke(config.inputs[j], u0_j + h);
        Eigen::VectorXd f_plus = get_xdot();

        sim.Poke(config.inputs[j], u0_j - h);
        Eigen::VectorXd f_minus = get_xdot();

        model.B.col(j) = (f_plus - f_minus) / (2.0 * h);

        sim.Poke(config.inputs[j], u0_j);  // Restore
    }

    // Compute C = dg/dx
    model.C.resize(ny, nx);
    for (int j = 0; j < nx; ++j) {
        double x0_j = sim.Peek(config.states[j]);

        sim.Poke(config.states[j], x0_j + h);
        sim.ComputeDerivatives(t);  // May need to update outputs
        Eigen::VectorXd y_plus = get_y();

        sim.Poke(config.states[j], x0_j - h);
        sim.ComputeDerivatives(t);
        Eigen::VectorXd y_minus = get_y();

        model.C.col(j) = (y_plus - y_minus) / (2.0 * h);

        sim.Poke(config.states[j], x0_j);  // Restore
    }

    // Compute D = dg/du
    model.D.resize(ny, nu);
    for (int j = 0; j < nu; ++j) {
        double u0_j = sim.Peek(config.inputs[j]);

        sim.Poke(config.inputs[j], u0_j + h);
        sim.ComputeDerivatives(t);
        Eigen::VectorXd y_plus = get_y();

        sim.Poke(config.inputs[j], u0_j - h);
        sim.ComputeDerivatives(t);
        Eigen::VectorXd y_minus = get_y();

        model.D.col(j) = (y_plus - y_minus) / (2.0 * h);

        sim.Poke(config.inputs[j], u0_j);  // Restore
    }

    return model;
}
```

---

## Part 5: Linear Model Export

### MATLAB Export

```cpp
void LinearModel::ExportMatlab(const std::string& path) const {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw IOError("Failed to open file: " + path);
    }

    file << "% Linear state-space model\n";
    file << "% Generated by Icarus " << Version() << "\n";
    file << "% Operating point: t = " << t0 << "\n\n";

    // Write matrices
    auto write_matrix = [&](const std::string& name, const Eigen::MatrixXd& M) {
        file << name << " = [\n";
        for (int i = 0; i < M.rows(); ++i) {
            file << "  ";
            for (int j = 0; j < M.cols(); ++j) {
                file << std::setprecision(15) << M(i, j);
                if (j < M.cols() - 1) file << ", ";
            }
            file << ";\n";
        }
        file << "];\n\n";
    };

    write_matrix("A", A);
    write_matrix("B", B);
    write_matrix("C", C);
    write_matrix("D", D);

    // Create ss object
    file << "sys = ss(A, B, C, D);\n";

    // State/input/output names
    file << "sys.StateName = {";
    for (size_t i = 0; i < state_names.size(); ++i) {
        file << "'" << state_names[i] << "'";
        if (i < state_names.size() - 1) file << ", ";
    }
    file << "};\n";

    // Similar for input_names, output_names...
}
```

### NumPy Export

```cpp
void LinearModel::ExportNumPy(const std::string& path) const {
    std::ofstream file(path);

    file << "import numpy as np\n\n";

    auto write_matrix = [&](const std::string& name, const Eigen::MatrixXd& M) {
        file << name << " = np.array([\n";
        for (int i = 0; i < M.rows(); ++i) {
            file << "    [";
            for (int j = 0; j < M.cols(); ++j) {
                file << std::setprecision(15) << M(i, j);
                if (j < M.cols() - 1) file << ", ";
            }
            file << "]";
            if (i < M.rows() - 1) file << ",";
            file << "\n";
        }
        file << "])\n\n";
    };

    write_matrix("A", A);
    write_matrix("B", B);
    write_matrix("C", C);
    write_matrix("D", D);

    // Metadata
    file << "state_names = " << "[";
    for (size_t i = 0; i < state_names.size(); ++i) {
        file << "'" << state_names[i] << "'";
        if (i < state_names.size() - 1) file << ", ";
    }
    file << "]\n";
}
```

---

## Part 6: Updated Stage() Implementation

```cpp
inline void Simulator::Stage() {
    // ... existing component staging and wiring validation ...

    // =========================================================================
    // NEW: Symbolic Graph Generation
    // =========================================================================
    if (config_.staging.symbolics.enabled) {
        try {
            staging::SymbolicSimulatorCore sym_sim(config_);
            staging::SymbolicStager stager(sym_sim);

            auto sym_dynamics = stager.GenerateDynamics();
            dynamics_graph_ = sym_dynamics.dynamics;

            if (config_.staging.symbolics.generate_jacobian) {
                jacobian_ = sym_dynamics.jacobian_x;
            }

            logger_.Info("Symbolic graphs generated: {} states",
                        sym_sim.GetStateSize());
        } catch (const Error& e) {
            logger_.Warn("Symbolic generation failed: {}", e.what());
            // Continue - symbolic is optional
        }
    }

    // =========================================================================
    // NEW: Trim Optimization
    // =========================================================================
    if (config_.staging.trim.enabled) {
        std::unique_ptr<staging::TrimSolver> solver;

        if (config_.staging.trim.method == "newton" &&
            !config_.staging.symbolics.enabled) {
            // Numeric mode
            solver = std::make_unique<staging::FiniteDifferenceTrim>();
        } else {
            // Symbolic mode (or IPOPT)
            solver = std::make_unique<staging::SymbolicTrim>();
        }

        auto result = solver->Solve(*this, config_.staging.trim);

        if (result.converged) {
            logger_.Info("Trim converged in {} iterations, ||F|| = {:.2e}",
                        result.iterations, result.residual_norm);
        } else {
            throw TrimError("Trim failed: " + result.message);
        }

        trim_result_ = result;  // Store for later access
    }

    // =========================================================================
    // NEW: Linearization
    // =========================================================================
    if (config_.staging.linearization.enabled) {
        std::unique_ptr<staging::Linearizer> linearizer;

        if (config_.staging.symbolics.enabled) {
            linearizer = std::make_unique<staging::SymbolicLinearizer>();
        } else {
            linearizer = std::make_unique<staging::FiniteDifferenceLinearizer>();
        }

        linear_model_ = linearizer->Compute(*this, config_.staging.linearization);

        logger_.Info("Linearization complete: A is {}x{}, rank(Wc) = {}",
                    linear_model_->A.rows(), linear_model_->A.cols(),
                    linear_model_->ControllabilityRank());

        // Export if configured
        const auto& lin = config_.staging.linearization;
        if (lin.export_matlab && !lin.output_dir.empty()) {
            linear_model_->ExportMatlab(lin.output_dir + "/linear_model.m");
        }
        if (lin.export_numpy && !lin.output_dir.empty()) {
            linear_model_->ExportNumPy(lin.output_dir + "/linear_model.py");
        }
        if (lin.export_json && !lin.output_dir.empty()) {
            linear_model_->ExportJSON(lin.output_dir + "/linear_model.json");
        }
    }

    // ... rest of existing Stage() ...
}
```

---

## Part 7: Implementation Order

### Phase A: Core Infrastructure
1. **A.1** Create `include/icarus/staging/` directory
2. **A.2** Implement `StagingTypes.hpp` (LinearModel, TrimResult)
3. **A.3** Add `linear_model_` and `trim_result_` to Simulator

### Phase B: Numeric Mode (No Symbolic Dependencies)
4. **B.1** Implement `FiniteDifferenceTrim`
5. **B.2** Implement `FiniteDifferenceLinearizer`
6. **B.3** Implement `LinearModel::Export*()` methods
7. **B.4** Update `Stage()` to use numeric trim/linearization
8. **B.5** Add unit tests for numeric mode

### Phase C: Symbolic Mode
9. **C.1** Update `ICARUS_REGISTER_COMPONENT` for dual registration
10. **C.2** Implement `SymbolicSimulatorCore`
11. **C.3** Implement `SymbolicTrim`
12. **C.4** Implement `SymbolicLinearizer`
13. **C.5** Implement `SymbolicStager` (graph export)
14. **C.6** Add unit tests for symbolic mode

### Phase D: Integration & Testing
15. **D.1** Integration test: trim → linearization → step
16. **D.2** YAML configuration tests
17. **D.3** Export format verification tests
18. **D.4** Documentation updates

---

## Exit Criteria

### Numeric Mode
- [ ] `FiniteDifferenceTrim` converges on test cases
- [ ] `FiniteDifferenceLinearizer` produces correct A, B, C, D
- [ ] Linear model exports to MATLAB, NumPy, JSON
- [ ] Stage() integrates trim and linearization

### Symbolic Mode
- [ ] Dual component registration works
- [ ] `SymbolicSimulatorCore` creates symbolic components
- [ ] `SymbolicTrim` produces same results as numeric (within tolerance)
- [ ] `SymbolicLinearizer` produces exact Jacobians
- [ ] Symbolic dynamics graphs exportable as `janus::Function`

### Integration
- [ ] Trim result applied to numeric sim before linearization
- [ ] Linearization at trim point produces stable A matrix (if applicable)
- [ ] Full pipeline: YAML → Stage() → trim → linearize → Step()
