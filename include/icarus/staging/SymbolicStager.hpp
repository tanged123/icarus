#pragma once

/**
 * @file SymbolicStager.hpp
 * @brief Symbolic graph generation during Stage()
 *
 * Part of Phase 4: Staging Implementation (Phase C.5)
 *
 * Extracts computational graphs from symbolic simulator for:
 *   - Dynamics: f(t, x) -> xdot
 *   - Jacobian: df/dx
 *   - Integration step: step(t, x, dt) -> x_next
 */

#include <icarus/core/CoreTypes.hpp>
#include <icarus/staging/StagingTypes.hpp>
#include <icarus/staging/SymbolicSimulatorCore.hpp>

#include <metis/core/Function.hpp>
#include <metis/math/AutoDiff.hpp>

#include <string>
#include <vector>

namespace icarus::staging {

/**
 * @brief Configuration for symbolic graph generation
 */
struct SymbolicStagerConfig {
    bool generate_dynamics = true; ///< Generate dynamics function f(t, x) -> xdot
    bool generate_jacobian = true; ///< Generate state Jacobian df/dx
    bool generate_step = false;    ///< Generate discrete step function
    double step_dt = 0.01;         ///< Step size for discrete step function
    std::string function_name = "dynamics";
    bool include_time = true;                 ///< Include time as explicit input
    std::vector<std::string> control_signals; ///< Signals to treat as controls (inputs)
};

/**
 * @brief Symbolic graph generator
 *
 * Creates metis::Function objects representing the simulation dynamics.
 * These can be used for:
 *   - Numerical evaluation
 *   - Automatic differentiation (Jacobian, Hessian)
 *   - NLP formulation for trajectory optimization
 *   - Code generation
 */
class SymbolicStager {
  public:
    using Scalar = metis::SymbolicScalar;

    /**
     * @brief Construct stager with symbolic simulator
     *
     * @param sym_sim Reference to initialized SymbolicSimulatorCore
     */
    explicit SymbolicStager(SymbolicSimulatorCore &sym_sim) : sym_sim_(sym_sim) {}

    /**
     * @brief Generate symbolic dynamics representation
     *
     * Creates dynamics function f(t, x) -> xdot and optionally computes Jacobians.
     *
     * @param config Stager configuration
     * @return SymbolicDynamics containing functions and metadata
     */
    SymbolicDynamics GenerateDynamics(const SymbolicStagerConfig &config = {}) {
        SymbolicDynamics result;

        const std::size_t n_states = sym_sim_.GetStateSize();
        const int n_controls = static_cast<int>(config.control_signals.size());

        // Create symbolic variables
        auto [x_vec, x_mx] = metis::sym_vec_pair("x", static_cast<int>(n_states));
        Scalar t_sym = metis::sym("t");

        // Create symbolic controls if specified
        std::vector<Scalar> u_elements;
        Scalar u_mx;
        if (n_controls > 0) {
            auto [u_v, u_m] = metis::sym_vec_pair("u", n_controls);
            for (int i = 0; i < n_controls; ++i) {
                u_elements.push_back(u_v(i));
            }
            u_mx = u_m;
        }

        // Set symbolic state and time
        sym_sim_.SetState(x_vec);
        sym_sim_.SetTime(t_sym);

        // Apply symbolic controls
        for (int i = 0; i < n_controls; ++i) {
            sym_sim_.SetSignal(config.control_signals[i], u_elements[i]);
        }

        // Compute derivatives symbolically
        MetisVector<Scalar> xdot_vec = sym_sim_.ComputeDerivatives();

        // Convert to single MX column vector
        std::vector<Scalar> xdot_elements;
        xdot_elements.reserve(n_states);
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(n_states); ++i) {
            xdot_elements.push_back(xdot_vec(i));
        }
        Scalar xdot_mx = Scalar::vertcat(xdot_elements);

        // Build dynamics function inputs
        std::vector<metis::SymbolicArg> inputs;
        if (config.include_time) {
            inputs.push_back(t_sym);
        }
        inputs.push_back(x_mx);
        if (n_controls > 0) {
            inputs.push_back(u_mx);
        }

        // Create dynamics function: f(t, x, [u]) -> xdot
        if (config.generate_dynamics) {
            result.dynamics = metis::Function(config.function_name, inputs, {xdot_mx});
        }

        // Create Jacobian function: J(t, x, [u]) -> df/dx
        if (config.generate_jacobian) {
            Scalar J_sym = metis::jacobian({xdot_mx}, {x_mx});
            result.jacobian_x = metis::Function("jacobian_x", inputs, {J_sym});
        }

        // Create control Jacobian if controls specified
        if (n_controls > 0 && config.generate_jacobian) {
            Scalar Ju_sym = metis::jacobian({xdot_mx}, {u_mx});
            result.jacobian_u = metis::Function("jacobian_u", inputs, {Ju_sym});
        }

        // Store metadata
        result.state_names = GetStateNames();
        result.control_names = config.control_signals;

        return result;
    }

    /**
     * @brief Generate discrete-time step function
     *
     * Creates function step(t, x) -> x_next using RK4 integration.
     *
     * @param dt Step size
     * @return metis::Function representing one integration step
     */
    metis::Function GenerateStepFunction(double dt) {
        const std::size_t n_states = sym_sim_.GetStateSize();

        // Create symbolic variables
        auto [x_vec, x_mx] = metis::sym_vec_pair("x", static_cast<int>(n_states));
        Scalar t_sym = metis::sym("t");
        Scalar dt_sym(dt);

        // RK4 integration
        auto compute_xdot = [this](const MetisVector<Scalar> &x, Scalar t) {
            sym_sim_.SetState(x);
            sym_sim_.SetTime(t);
            return sym_sim_.ComputeDerivatives();
        };

        // k1 = f(t, x)
        MetisVector<Scalar> k1 = compute_xdot(x_vec, t_sym);

        // k2 = f(t + dt/2, x + dt/2 * k1)
        MetisVector<Scalar> x_mid1 = x_vec + dt_sym * Scalar(0.5) * k1;
        MetisVector<Scalar> k2 = compute_xdot(x_mid1, t_sym + dt_sym * Scalar(0.5));

        // k3 = f(t + dt/2, x + dt/2 * k2)
        MetisVector<Scalar> x_mid2 = x_vec + dt_sym * Scalar(0.5) * k2;
        MetisVector<Scalar> k3 = compute_xdot(x_mid2, t_sym + dt_sym * Scalar(0.5));

        // k4 = f(t + dt, x + dt * k3)
        MetisVector<Scalar> x_end = x_vec + dt_sym * k3;
        MetisVector<Scalar> k4 = compute_xdot(x_end, t_sym + dt_sym);

        // x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        MetisVector<Scalar> x_next =
            x_vec + dt_sym * (Scalar(1.0 / 6.0) * k1 + Scalar(1.0 / 3.0) * k2 +
                              Scalar(1.0 / 3.0) * k3 + Scalar(1.0 / 6.0) * k4);

        // Convert to single MX column vector
        std::vector<Scalar> x_next_elements;
        x_next_elements.reserve(n_states);
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(n_states); ++i) {
            x_next_elements.push_back(x_next(i));
        }
        Scalar x_next_mx = Scalar::vertcat(x_next_elements);

        return metis::Function("step", {t_sym, x_mx}, {x_next_mx});
    }

    /**
     * @brief Get state variable names in order
     */
    [[nodiscard]] std::vector<std::string> GetStateNames() const {
        std::vector<std::string> names;
        const auto &bindings = sym_sim_.GetStateBindings();
        names.reserve(bindings.size());
        for (const auto &binding : bindings) {
            names.push_back(binding.name);
        }
        return names;
    }

    /**
     * @brief Get total state size
     */
    [[nodiscard]] std::size_t GetStateSize() const { return sym_sim_.GetStateSize(); }

  private:
    SymbolicSimulatorCore &sym_sim_;
};

// =============================================================================
// Convenience Functions
// =============================================================================

/**
 * @brief Generate dynamics graph from simulator config
 *
 * Creates a SymbolicSimulatorCore and extracts dynamics graph.
 *
 * @param config Simulator configuration
 * @return SymbolicDynamics with dynamics and Jacobian functions
 */
inline SymbolicDynamics GenerateSymbolicDynamics(const SimulatorConfig &config) {
    SymbolicSimulatorCore sym_sim(config);
    SymbolicStager stager(sym_sim);
    return stager.GenerateDynamics();
}

/**
 * @brief Generate dynamics graph with custom options
 *
 * @param config Simulator configuration
 * @param stager_config Stager configuration
 * @return SymbolicDynamics with dynamics and Jacobian functions
 */
inline SymbolicDynamics GenerateSymbolicDynamics(const SimulatorConfig &config,
                                                 const SymbolicStagerConfig &stager_config) {
    SymbolicSimulatorCore sym_sim(config);
    SymbolicStager stager(sym_sim);
    return stager.GenerateDynamics(stager_config);
}

} // namespace icarus::staging
