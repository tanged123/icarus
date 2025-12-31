#pragma once

/**
 * @file TrimSolver.hpp
 * @brief Trim optimization solvers (numeric and symbolic)
 *
 * Part of Phase 4: Staging Implementation.
 *
 * Trim finds control values that zero selected derivatives, achieving
 * equilibrium/trim conditions. Two modes:
 *   - Numeric (FiniteDifferenceTrim): Uses central differences for Jacobians
 *   - Symbolic (SymbolicTrim): Uses exact Jacobians via janus::NewtonSolver
 */

#include <icarus/core/Error.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/staging/StagingTypes.hpp>

// Include SymbolicSimulatorCore here (outside namespace)
// to avoid namespace resolution issues
#include <icarus/staging/SymbolicSimulatorCore.hpp>

#include <janus/core/Function.hpp>
#include <janus/math/RootFinding.hpp>

#include <Eigen/Dense>
#include <cmath>

#include <memory>
#include <string>
#include <vector>

namespace icarus {
// Forward declaration
class Simulator;
} // namespace icarus

namespace icarus::staging {

// Forward declaration
class SymbolicSimulatorCore;

// =============================================================================
// TrimSolver Interface
// =============================================================================

/**
 * @brief Abstract trim solver interface
 */
class TrimSolver {
  public:
    virtual ~TrimSolver() = default;

    /**
     * @brief Solve trim problem
     *
     * Finds control values that zero the specified derivatives.
     * Modifies the simulator's signal values to the trim solution.
     *
     * @param sim Simulator to trim (will be modified)
     * @param config Trim configuration
     * @return TrimResult with convergence info and final values
     */
    virtual ::icarus::staging::TrimResult Solve(::icarus::Simulator &sim,
                                                const TrimConfig &config) = 0;
};

// =============================================================================
// FiniteDifferenceTrim
// =============================================================================

/**
 * @brief Numeric trim using finite differences
 *
 * Simple and robust Newton solver using central differences for Jacobian.
 * Works with the existing double-typed Simulator without symbolic components.
 *
 * Algorithm:
 *   1. Evaluate residual F(u) = [selected derivatives]
 *   2. Compute Jacobian J = dF/du via central differences
 *   3. Newton step: u_new = u - damping * J^(-1) * F
 *   4. Apply bounds (projection)
 *   5. Repeat until ||F|| < tolerance
 */
class FiniteDifferenceTrim : public TrimSolver {
  public:
    struct Options {
        double step_size{1e-7};  ///< Finite difference step size
        double tolerance{1e-6};  ///< Convergence tolerance on residual norm
        int max_iterations{100}; ///< Maximum Newton iterations
        double damping{1.0};     ///< Newton step damping factor (0, 1]
        bool verbose{false};     ///< Print iteration progress
    };

    FiniteDifferenceTrim() : opts_{} {}
    explicit FiniteDifferenceTrim(Options opts) : opts_(std::move(opts)) {}

    ::icarus::staging::TrimResult Solve(::icarus::Simulator &sim,
                                        const TrimConfig &config) override;

  private:
    Options opts_;

    /// Evaluate residual vector (selected derivatives)
    Eigen::VectorXd EvaluateResidual(::icarus::Simulator &sim,
                                     const std::vector<std::string> &zero_derivatives, double t);

    /// Compute Jacobian via central differences
    Eigen::MatrixXd ComputeJacobian(::icarus::Simulator &sim,
                                    const std::vector<std::string> &control_signals,
                                    const std::vector<std::string> &zero_derivatives, double t);

    /// Apply control bounds via projection
    void ApplyBounds(Eigen::VectorXd &u, const std::vector<std::string> &control_signals,
                     const std::unordered_map<std::string, std::pair<double, double>> &bounds);
};

// =============================================================================
// SymbolicTrim
// =============================================================================

/**
 * @brief Symbolic trim using janus::NewtonSolver
 *
 * Requires symbolic components (SymbolicSimulatorCore).
 * Provides exact Jacobians via automatic differentiation.
 *
 * For problems with inequality constraints, can fall back to IPOPT.
 */
class SymbolicTrim : public TrimSolver {
  public:
    ::icarus::staging::TrimResult Solve(::icarus::Simulator &sim,
                                        const TrimConfig &config) override;

  private:
    /// Build symbolic residual function F(u) -> derivatives
    janus::Function BuildResidualFunction(SymbolicSimulatorCore &sym_sim, const TrimConfig &config);
};

// =============================================================================
// Factory
// =============================================================================

/**
 * @brief Create appropriate trim solver based on configuration
 *
 * Returns FiniteDifferenceTrim for numeric mode,
 * SymbolicTrim for symbolic mode (requires symbolic components).
 */
inline std::unique_ptr<TrimSolver> CreateTrimSolver(const TrimConfig &config,
                                                    bool symbolic_enabled) {
    if (symbolic_enabled && config.method == "newton") {
        return std::make_unique<SymbolicTrim>();
    }
    // Default to finite differences (works without symbolic)
    FiniteDifferenceTrim::Options opts;
    opts.tolerance = config.tolerance;
    opts.max_iterations = config.max_iterations;
    return std::make_unique<FiniteDifferenceTrim>(opts);
}

} // namespace icarus::staging

// =============================================================================
// Implementation
// =============================================================================
// Note: Full implementation requires Simulator definition.
// Include this header after Simulator.hpp, or implement in .cpp file.

#include <icarus/sim/Simulator.hpp>

namespace icarus::staging {

inline ::icarus::staging::TrimResult FiniteDifferenceTrim::Solve(::icarus::Simulator &sim,
                                                                 const TrimConfig &config) {
    ::icarus::staging::TrimResult result;

    const auto &controls = config.control_signals;
    const auto &derivs = config.zero_derivatives;
    const int n_controls = static_cast<int>(controls.size());
    const int n_residuals = static_cast<int>(derivs.size());

    if (n_controls == 0) {
        result.message = "No control signals specified";
        return result;
    }
    if (n_residuals == 0) {
        result.message = "No zero_derivatives specified";
        return result;
    }

    // Log start
    std::vector<std::pair<std::string, double>> targets;
    targets.reserve(n_residuals);
    for (const auto &name : derivs) {
        targets.emplace_back(name, 0.0);
    }
    sim.GetLogger().LogTrimStart("finite-difference", targets);

    // Initialize controls from config or current values
    Eigen::VectorXd u(n_controls);
    for (int i = 0; i < n_controls; ++i) {
        auto it = config.initial_guesses.find(controls[i]);
        if (it != config.initial_guesses.end()) {
            u(i) = it->second;
        } else {
            u(i) = sim.Peek(controls[i]);
        }
    }

    const double t = sim.Time();

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
            sim.GetLogger().LogTrimIteration(iter, norm);
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

        // Solve J * du = -F using SVD-based pseudo-inverse for robustness
        // This handles rank-deficient, underdetermined, and overdetermined cases
        // automatically with tolerance-based singular value thresholding.
        //
        // SVD tolerance: singular values below this fraction of the largest
        // are treated as zero, providing numerical stability for near-singular
        // Jacobians. Default threshold is O(machine_epsilon * max_dim).
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Set threshold for singular values (relative to largest)
        // Values below threshold * max_singular_value are treated as zero
        constexpr double svd_tolerance = 1e-10;
        svd.setThreshold(svd_tolerance);

        // SVD.solve() automatically computes:
        // - Minimum-norm solution for underdetermined systems (n_controls > n_residuals)
        // - Least-squares solution for overdetermined systems (n_residuals > n_controls)
        // - Exact solution for square full-rank systems
        Eigen::VectorXd du = svd.solve(-F);

        // Update with damping
        u += opts_.damping * du;

        // Apply bounds
        ApplyBounds(u, controls, config.control_bounds);

        result.iterations = iter + 1;
        result.residual_norm = norm;
    }

    if (!result.converged) {
        result.message =
            "Max iterations reached (residual = " + std::to_string(result.residual_norm) + ")";
    }

    // Store final values and apply to simulator
    for (int i = 0; i < n_controls; ++i) {
        result.controls[controls[i]] = u(i);
        sim.Poke(controls[i], u(i));
    }

    // Store final residuals
    Eigen::VectorXd F_final = EvaluateResidual(sim, derivs, t);
    for (int i = 0; i < n_residuals; ++i) {
        result.residuals[derivs[i]] = F_final(i);
    }

    if (result.converged) {
        sim.GetLogger().LogTrimConverged(result.iterations);
    } else {
        sim.GetLogger().LogTrimFailed(result.message);
    }

    return result;
}

inline Eigen::VectorXd
FiniteDifferenceTrim::EvaluateResidual(::icarus::Simulator &sim,
                                       const std::vector<std::string> &zero_derivatives, double t) {
    // Compute all derivatives at current state
    sim.ComputeDerivatives(t);

    // Extract selected derivatives
    const int n = static_cast<int>(zero_derivatives.size());
    Eigen::VectorXd F(n);
    for (int i = 0; i < n; ++i) {
        F(i) = sim.Peek(zero_derivatives[i]);
    }
    return F;
}

inline Eigen::MatrixXd
FiniteDifferenceTrim::ComputeJacobian(::icarus::Simulator &sim,
                                      const std::vector<std::string> &control_signals,
                                      const std::vector<std::string> &zero_derivatives, double t) {
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

        // Central difference: dF/du ≈ (F(u+h) - F(u-h)) / (2h)
        J.col(j) = (F_plus - F_minus) / (2.0 * h);

        // Restore original value
        sim.Poke(control_signals[j], u0);
    }

    return J;
}

inline void FiniteDifferenceTrim::ApplyBounds(
    Eigen::VectorXd &u, const std::vector<std::string> &control_signals,
    const std::unordered_map<std::string, std::pair<double, double>> &bounds) {
    for (int i = 0; i < static_cast<int>(control_signals.size()); ++i) {
        auto it = bounds.find(control_signals[i]);
        if (it != bounds.end()) {
            const auto &[lo, hi] = it->second;
            u(i) = std::clamp(u(i), lo, hi);
        }
    }
}

// =============================================================================
// SymbolicTrim Implementation
// =============================================================================

inline ::icarus::staging::TrimResult SymbolicTrim::Solve(::icarus::Simulator &sim,
                                                         const TrimConfig &config) {
    ::icarus::staging::TrimResult result;

    const auto &controls = config.control_signals;
    const auto &derivs = config.zero_derivatives;
    const int n_controls = static_cast<int>(controls.size());
    const int n_residuals = static_cast<int>(derivs.size());

    if (n_controls == 0) {
        result.message = "No control signals specified";
        return result;
    }
    if (n_residuals == 0) {
        result.message = "No zero_derivatives specified";
        return result;
    }

    // Log start
    std::vector<std::pair<std::string, double>> targets;
    targets.reserve(n_residuals);
    for (const auto &name : derivs) {
        targets.emplace_back(name, 0.0);
    }
    sim.GetLogger().LogTrimStart("symbolic-newton", targets);

    // Create symbolic simulator from same config
    try {
        SymbolicSimulatorCore sym_sim(sim.GetConfig());

        // Build symbolic residual function
        janus::Function F = BuildResidualFunction(sym_sim, config);

        // Configure Newton solver
        janus::RootFinderOptions opts;
        opts.abstol = config.tolerance;
        opts.max_iter = config.max_iterations;
        opts.line_search = true;
        opts.verbose = false;

        janus::NewtonSolver solver(F, opts);

        // Get initial guess from config or current simulator values
        Eigen::VectorXd u0(n_controls);
        for (int i = 0; i < n_controls; ++i) {
            auto it = config.initial_guesses.find(controls[i]);
            if (it != config.initial_guesses.end()) {
                u0(i) = it->second;
            } else {
                u0(i) = sim.Peek(controls[i]);
            }
        }

        // Solve
        auto root_result = solver.solve(u0);

        // Convert to TrimResult
        result.converged = root_result.converged;
        result.iterations = root_result.iterations;
        result.message = root_result.message;

        if (result.converged) {
            // Apply solution to numeric simulator
            for (int i = 0; i < n_controls; ++i) {
                double val = root_result.x(i);
                result.controls[controls[i]] = val;
                sim.Poke(controls[i], val);
            }

            // Evaluate final residuals
            double t = sim.Time();
            sim.ComputeDerivatives(t);
            for (int i = 0; i < n_residuals; ++i) {
                result.residuals[derivs[i]] = sim.Peek(derivs[i]);
            }

            // Compute residual norm
            double norm = 0.0;
            for (const auto &[name, val] : result.residuals) {
                norm += val * val;
            }
            result.residual_norm = std::sqrt(norm);

            sim.GetLogger().LogTrimConverged(result.iterations);
        } else {
            sim.GetLogger().LogTrimFailed(result.message);
        }

    } catch (const Error &e) {
        result.converged = false;
        result.message = std::string("Symbolic trim failed: ") + e.what();
        sim.GetLogger().LogTrimFailed(result.message);
    } catch (const std::exception &e) {
        result.converged = false;
        result.message = std::string("Symbolic trim failed: ") + e.what();
        sim.GetLogger().LogTrimFailed(result.message);
    }

    return result;
}

inline janus::Function SymbolicTrim::BuildResidualFunction(SymbolicSimulatorCore &sym_sim,
                                                           const TrimConfig &config) {
    using Scalar = janus::SymbolicScalar;

    const int n_controls = static_cast<int>(config.control_signals.size());
    const int n_residuals = static_cast<int>(config.zero_derivatives.size());
    const std::size_t n_states = sym_sim.GetStateSize();

    // Create symbolic control variables
    auto [u_vec, u_mx] = janus::sym_vec_pair("u", n_controls);

    // Create symbolic state and time (use current numeric values as base)
    auto [x_vec, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states));
    Scalar t_sym = janus::sym("t");

    // Set symbolic state and time
    sym_sim.SetState(x_vec);
    sym_sim.SetTime(t_sym);

    // Apply symbolic controls to simulator
    for (int i = 0; i < n_controls; ++i) {
        sym_sim.SetSignal(config.control_signals[i], u_vec(i));
    }

    // Compute derivatives symbolically (traces the graph)
    sym_sim.ComputeDerivatives();

    // Gather residuals (the derivatives we want to be zero)
    std::vector<Scalar> residual_elements;
    residual_elements.reserve(n_residuals);
    for (const auto &deriv_name : config.zero_derivatives) {
        if (!sym_sim.HasSignal(deriv_name)) {
            throw ConfigError("SymbolicTrim: Unknown derivative signal '" + deriv_name + "'");
        }
        residual_elements.push_back(sym_sim.GetSignal(deriv_name));
    }

    // Concatenate into single output vector
    Scalar residuals = Scalar::vertcat(residual_elements);

    // Build function: F(u) -> residuals
    // Note: x and t are fixed parameters (not optimized), u is the decision variable
    return janus::Function("trim_residual", {u_mx}, {residuals});
}

} // namespace icarus::staging
