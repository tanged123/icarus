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

// SymbolicTrim implementation is deferred until SymbolicSimulatorCore is available
inline ::icarus::staging::TrimResult SymbolicTrim::Solve(::icarus::Simulator & /*sim*/,
                                                         const TrimConfig & /*config*/) {
    ::icarus::staging::TrimResult result;
    result.message = "SymbolicTrim requires SymbolicSimulatorCore (not yet implemented)";
    return result;
}

inline janus::Function SymbolicTrim::BuildResidualFunction(SymbolicSimulatorCore & /*sym_sim*/,
                                                           const TrimConfig & /*config*/) {
    throw NotImplementedError("SymbolicTrim::BuildResidualFunction");
}

} // namespace icarus::staging
