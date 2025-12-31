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

// Include SymbolicSimulatorCore and AutoDiff here (outside namespace)
// to avoid namespace resolution issues
#include <icarus/staging/SymbolicSimulatorCore.hpp>
#include <janus/math/AutoDiff.hpp>

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

// =============================================================================
// SymbolicLinearizer Implementation
// =============================================================================

inline ::icarus::staging::LinearModel
SymbolicLinearizer::Compute(::icarus::Simulator &sim, const LinearizationConfig &config) {
    using Scalar = janus::SymbolicScalar;

    ::icarus::staging::LinearModel model;
    model.state_names = config.states;
    model.input_names = config.inputs;
    model.output_names = config.outputs;

    const int nx = static_cast<int>(config.states.size());
    const int nu = static_cast<int>(config.inputs.size());
    const int ny = static_cast<int>(config.outputs.size());

    if (nx == 0) {
        throw ConfigError("SymbolicLinearizer: requires at least one state");
    }

    // Store operating point from numeric simulator
    model.t0 = sim.Time();
    model.x0.resize(nx);
    for (int i = 0; i < nx; ++i) {
        model.x0(i) = sim.Peek(config.states[i]);
    }
    model.u0.resize(nu);
    for (int i = 0; i < nu; ++i) {
        model.u0(i) = sim.Peek(config.inputs[i]);
    }

    // Create symbolic simulator
    SymbolicSimulatorCore sym_sim(sim.GetConfig());

    const std::size_t n_states_total = sym_sim.GetStateSize();

    // Create symbolic variables - MUST use sym_vec_pair to get pure symbolic MX
    auto [x_vec, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states_total));
    Scalar t_sym = janus::sym("t");

    // Create symbolic inputs
    std::vector<Scalar> u_elements;
    Scalar u_mx;
    if (nu > 0) {
        auto [u_v, u_m] = janus::sym_vec_pair("u", nu);
        for (int i = 0; i < nu; ++i) {
            u_elements.push_back(u_v(i));
        }
        u_mx = u_m;
    }

    // Set symbolic state and time
    sym_sim.SetState(x_vec);
    sym_sim.SetTime(t_sym);

    // Apply symbolic inputs
    for (int i = 0; i < nu; ++i) {
        sym_sim.SetSignal(config.inputs[i], u_elements[i]);
    }

    // Compute derivatives symbolically
    JanusVector<Scalar> xdot_vec = sym_sim.ComputeDerivatives();

    // Find indices of selected states in global state vector
    std::vector<int> state_indices;
    state_indices.reserve(nx);
    for (const auto &slice : sym_sim.GetStateLayout()) {
        for (std::size_t i = 0; i < slice.size; ++i) {
            std::string state_name = slice.component_name + ".state[" + std::to_string(i) + "]";
            for (int j = 0; j < nx; ++j) {
                if (config.states[j] == state_name ||
                    config.states[j] == slice.component_name + ".x[" + std::to_string(i) + "]") {
                    state_indices.push_back(static_cast<int>(slice.offset + i));
                    break;
                }
            }
        }
    }

    // If we couldn't match by name, assume states are in order of the state vector
    if (state_indices.size() != static_cast<std::size_t>(nx)) {
        state_indices.clear();
        for (int i = 0; i < nx && i < static_cast<int>(n_states_total); ++i) {
            state_indices.push_back(i);
        }
    }

    // Build symbolic xdot vector for selected states
    std::vector<Scalar> xdot_selected;
    xdot_selected.reserve(nx);
    for (int idx : state_indices) {
        xdot_selected.push_back(xdot_vec(idx));
    }
    Scalar xdot_mx = Scalar::vertcat(xdot_selected);

    // Compute A = df/dx using symbolic Jacobian
    // Use the full x_mx symbol (pure MX) for differentiation, then extract relevant columns
    Scalar A_full_sym = janus::jacobian({xdot_mx}, {x_mx});

    // Compute B = df/du
    Scalar B_sym;
    if (nu > 0) {
        B_sym = janus::jacobian({xdot_mx}, {u_mx});
    }

    // Build symbolic outputs
    std::vector<Scalar> y_elements;
    y_elements.reserve(ny);
    for (const auto &output_name : config.outputs) {
        if (sym_sim.HasSignal(output_name)) {
            y_elements.push_back(sym_sim.GetSignal(output_name));
        } else {
            throw ConfigError("SymbolicLinearizer: Unknown output signal '" + output_name + "'");
        }
    }

    Scalar C_full_sym, D_sym;
    if (ny > 0) {
        Scalar y_mx = Scalar::vertcat(y_elements);

        // Compute C = dg/dx (full Jacobian)
        C_full_sym = janus::jacobian({y_mx}, {x_mx});

        // Compute D = dg/du
        if (nu > 0) {
            D_sym = janus::jacobian({y_mx}, {u_mx});
        }
    }

    // Create janus::Functions for evaluation
    std::vector<janus::SymbolicArg> all_inputs;
    all_inputs.push_back(t_sym);
    all_inputs.push_back(x_mx);
    if (nu > 0) {
        all_inputs.push_back(u_mx);
    }

    janus::Function A_full_func("A_full", all_inputs, {A_full_sym});

    // Get full state from numeric simulator for evaluation
    Eigen::VectorXd full_state = sim.GetState();
    // Pad if necessary
    if (full_state.size() < static_cast<Eigen::Index>(n_states_total)) {
        Eigen::VectorXd padded = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(n_states_total));
        padded.head(full_state.size()) = full_state;
        full_state = padded;
    }

    // Evaluate A matrix (full) and extract selected columns
    // Call function with properly typed arguments: (t0, x, [u])
    std::vector<Eigen::MatrixXd> A_full_result;
    if (nu > 0) {
        A_full_result = A_full_func(model.t0, full_state, model.u0);
    } else {
        A_full_result = A_full_func(model.t0, full_state);
    }

    model.A.resize(nx, nx);
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < nx; ++j) {
            int col_idx = state_indices[j]; // Map to full state column
            model.A(i, j) = A_full_result[0](i, col_idx);
        }
    }

    // Evaluate B matrix
    if (nu > 0) {
        janus::Function B_func("B", all_inputs, {B_sym});
        auto B_result = B_func(model.t0, full_state, model.u0);
        model.B.resize(nx, nu);
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                model.B(i, j) = B_result[0](i, j);
            }
        }
    } else {
        model.B.resize(nx, 0);
    }

    // Evaluate C matrix (full) and extract selected columns
    if (ny > 0) {
        janus::Function C_full_func("C_full", all_inputs, {C_full_sym});
        std::vector<Eigen::MatrixXd> C_full_result;
        if (nu > 0) {
            C_full_result = C_full_func(model.t0, full_state, model.u0);
        } else {
            C_full_result = C_full_func(model.t0, full_state);
        }
        model.C.resize(ny, nx);
        for (int i = 0; i < ny; ++i) {
            for (int j = 0; j < nx; ++j) {
                int col_idx = state_indices[j]; // Map to full state column
                model.C(i, j) = C_full_result[0](i, col_idx);
            }
        }
    } else {
        model.C.resize(0, nx);
    }

    // Evaluate D matrix
    if (ny > 0 && nu > 0) {
        janus::Function D_func("D", all_inputs, {D_sym});
        auto D_result = D_func(model.t0, full_state, model.u0);
        model.D.resize(ny, nu);
        for (int i = 0; i < ny; ++i) {
            for (int j = 0; j < nu; ++j) {
                model.D(i, j) = D_result[0](i, j);
            }
        }
    } else {
        model.D.resize(ny, nu);
        model.D.setZero();
    }

    return model;
}

} // namespace icarus::staging
