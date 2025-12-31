#pragma once

/**
 * @file Linearizer.hpp
 * @brief Linearization of dynamics around operating point
 *
 * Part of Phase 4: Staging Implementation.
 *
 * Computes linear state-space model (A, B, C, D) at current operating point.
 * Two modes:
 *   - Numeric (FiniteDifferenceLinearizer): Uses central differences
 *   - Symbolic (SymbolicLinearizer): Uses exact Jacobians via janus::jacobian
 */

#include <icarus/core/Error.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/staging/StagingTypes.hpp>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace icarus {
// Forward declaration - Simulator is defined in sim/Simulator.hpp
class Simulator;
} // namespace icarus

namespace icarus::staging {

// Forward declaration
class SymbolicSimulatorCore;

// =============================================================================
// Linearizer Interface
// =============================================================================

/**
 * @brief Abstract linearizer interface
 */
class Linearizer {
  public:
    virtual ~Linearizer() = default;

    /**
     * @brief Compute linear model at current operating point
     *
     * The simulator should be at the desired operating point (e.g., after trim).
     *
     * @param sim Simulator at operating point
     * @param config Linearization configuration
     * @return LinearModel with A, B, C, D matrices
     */
    virtual ::icarus::staging::LinearModel Compute(::icarus::Simulator &sim,
                                                   const LinearizationConfig &config) = 0;
};

// =============================================================================
// FiniteDifferenceLinearizer
// =============================================================================

/**
 * @brief Linearizer using finite differences
 *
 * Computes Jacobians via central differences.
 * Simple and works with any numeric simulator.
 *
 * Accuracy is O(h²) where h is the step size.
 */
class FiniteDifferenceLinearizer : public Linearizer {
  public:
    struct Options {
        double step_size{1e-7}; ///< Finite difference step size
    };

    FiniteDifferenceLinearizer() : opts_{} {}
    explicit FiniteDifferenceLinearizer(Options opts) : opts_(std::move(opts)) {}

    ::icarus::staging::LinearModel Compute(::icarus::Simulator &sim,
                                           const LinearizationConfig &config) override;

  private:
    Options opts_;
};

// =============================================================================
// SymbolicLinearizer
// =============================================================================

/**
 * @brief Linearizer using symbolic Jacobians
 *
 * Uses janus::jacobian() for exact derivatives.
 * Requires symbolic components (SymbolicSimulatorCore).
 */
class SymbolicLinearizer : public Linearizer {
  public:
    ::icarus::staging::LinearModel Compute(::icarus::Simulator &sim,
                                           const LinearizationConfig &config) override;
};

// =============================================================================
// Factory
// =============================================================================

/**
 * @brief Create appropriate linearizer based on configuration
 */
inline std::unique_ptr<Linearizer> CreateLinearizer(bool symbolic_enabled) {
    if (symbolic_enabled) {
        return std::make_unique<SymbolicLinearizer>();
    }
    return std::make_unique<FiniteDifferenceLinearizer>();
}

} // namespace icarus::staging

// =============================================================================
// Implementation
// =============================================================================

#include <icarus/sim/Simulator.hpp>

namespace icarus::staging {

inline ::icarus::staging::LinearModel
FiniteDifferenceLinearizer::Compute(::icarus::Simulator &sim, const LinearizationConfig &config) {
    ::icarus::staging::LinearModel model;
    model.state_names = config.states;
    model.input_names = config.inputs;
    model.output_names = config.outputs;

    const int nx = static_cast<int>(config.states.size());
    const int nu = static_cast<int>(config.inputs.size());
    const int ny = static_cast<int>(config.outputs.size());
    const double h = opts_.step_size;
    const double t = sim.Time();

    if (nx == 0) {
        throw ConfigError("Linearization requires at least one state");
    }

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
    // Compute A = df/dx (central differences on derivatives)
    model.A.resize(nx, nx);
    Eigen::VectorXd state_backup = sim.GetState();

    for (int j = 0; j < nx; ++j) {
        // Get original state value
        Eigen::VectorXd state_plus = state_backup;
        Eigen::VectorXd state_minus = state_backup;

        state_plus(j) += h;
        state_minus(j) -= h;

        // Forward perturbation
        sim.SetState(state_plus);
        Eigen::VectorXd xdot_plus = sim.ComputeDerivatives(t);

        // Backward perturbation
        sim.SetState(state_minus);
        Eigen::VectorXd xdot_minus = sim.ComputeDerivatives(t);

        // Central difference
        model.A.col(j) = (xdot_plus - xdot_minus) / (2.0 * h);
    }

    // Restore state
    sim.SetState(state_backup);

    // Compute B = df/du
    model.B.resize(nx, nu);
    for (int j = 0; j < nu; ++j) {
        double u0_j = sim.Peek(config.inputs[j]);

        // Forward perturbation
        sim.Poke(config.inputs[j], u0_j + h);
        Eigen::VectorXd xdot_plus = sim.ComputeDerivatives(t);

        // Backward perturbation
        sim.Poke(config.inputs[j], u0_j - h);
        Eigen::VectorXd xdot_minus = sim.ComputeDerivatives(t);

        // Central difference
        model.B.col(j) = (xdot_plus - xdot_minus) / (2.0 * h);

        // Restore
        sim.Poke(config.inputs[j], u0_j);
    }

    // Helper: get outputs
    auto get_y = [&]() {
        Eigen::VectorXd y(ny);
        for (int i = 0; i < ny; ++i) {
            y(i) = sim.Peek(config.outputs[i]);
        }
        return y;
    };

    // Compute C = dg/dx
    model.C.resize(ny, nx);
    if (ny > 0) {
        for (int j = 0; j < nx; ++j) {
            Eigen::VectorXd state_plus = state_backup;
            Eigen::VectorXd state_minus = state_backup;

            state_plus(j) += h;
            state_minus(j) -= h;

            // Forward perturbation
            sim.SetState(state_plus);
            sim.ComputeDerivatives(t); // Update outputs
            Eigen::VectorXd y_plus = get_y();

            // Backward perturbation
            sim.SetState(state_minus);
            sim.ComputeDerivatives(t);
            Eigen::VectorXd y_minus = get_y();

            // Central difference
            model.C.col(j) = (y_plus - y_minus) / (2.0 * h);
        }

        // Restore state
        sim.SetState(state_backup);
    }

    // Compute D = dg/du
    model.D.resize(ny, nu);
    if (ny > 0 && nu > 0) {
        for (int j = 0; j < nu; ++j) {
            double u0_j = sim.Peek(config.inputs[j]);

            // Forward perturbation
            sim.Poke(config.inputs[j], u0_j + h);
            sim.ComputeDerivatives(t);
            Eigen::VectorXd y_plus = get_y();

            // Backward perturbation
            sim.Poke(config.inputs[j], u0_j - h);
            sim.ComputeDerivatives(t);
            Eigen::VectorXd y_minus = get_y();

            // Central difference
            model.D.col(j) = (y_plus - y_minus) / (2.0 * h);

            // Restore
            sim.Poke(config.inputs[j], u0_j);
        }
    }

    // Restore final state
    sim.SetState(state_backup);

    return model;
}

// SymbolicLinearizer implementation is deferred until SymbolicSimulatorCore is available
inline ::icarus::staging::LinearModel
SymbolicLinearizer::Compute(::icarus::Simulator & /*sim*/, const LinearizationConfig & /*config*/) {
    throw NotImplementedError("SymbolicLinearizer requires SymbolicSimulatorCore");
}

} // namespace icarus::staging
