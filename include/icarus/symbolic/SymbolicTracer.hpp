#pragma once

/**
 * @file SymbolicTracer.hpp
 * @brief Extracts computational graph from symbolic simulator
 *
 * Part of Phase 3.3: Graph Export.
 * Traces simulation dynamics to produce janus::Function for AD & optimization.
 */

#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/sim/Simulator.hpp>

#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>

#include <string>
#include <vector>

namespace icarus::symbolic {

/**
 * @brief Configuration for symbolic tracing
 */
struct TracerConfig {
    bool include_time = true;     ///< Include time as input
    bool include_controls = true; ///< Include control inputs
    std::string function_name = "dynamics";
    std::vector<std::string> control_signals; ///< Override auto-detected controls
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
 *
 * Usage:
 *   Simulator<SymbolicScalar> sim;
 *   // ... setup components, provision, stage ...
 *
 *   SymbolicTracer tracer(sim);
 *   auto dynamics = tracer.TraceDynamics();
 *
 *   // Evaluate numerically
 *   auto result = dynamics(t0, x0);  // Returns std::vector<NumericMatrix>
 *
 *   // Or compute Jacobian
 *   auto J = janus::jacobian({dynamics.eval(t_sym, x_sym)}, {x_sym});
 */
class SymbolicTracer {
  public:
    /**
     * @brief Construct tracer for a symbolic simulator
     * @param sim Reference to initialized Simulator<SymbolicScalar>
     *
     * The simulator must be in Staged phase (Provision and Stage called).
     * @throws LifecycleError if simulator is not staged
     */
    explicit SymbolicTracer(Simulator<SymbolicScalar> &sim) : sim_(sim) {
        if (!sim_.IsInitialized()) {
            throw LifecycleError(LifecyclePhase::Stage,
                                 "SymbolicTracer requires a Staged simulator");
        }
    }

    /**
     * @brief Configure tracing behavior
     * @param config Tracing configuration
     */
    SymbolicTracer &Configure(const TracerConfig &config) {
        config_ = config;
        return *this;
    }

    // =========================================================================
    // Graph Extraction
    // =========================================================================

    /**
     * @brief Trace dynamics to produce janus::Function
     * @return Function with signature (t, x) -> (xdot)
     *
     * The function is suitable for:
     * - Direct evaluation: dynamics(t_val, x_val) -> std::vector<NumericMatrix>
     * - Jacobian via janus::jacobian()
     * - Integration: Use in janus::solve_ivp
     * - NLP formulation: Add as constraint in janus::Opti
     */
    janus::Function TraceDynamics() {
        std::size_t n_states = sim_.GetTotalStateSize();

        // Create symbolic time (scalar)
        t_mx_ = janus::sym("t");

        // Use sym_vec_pair to get both:
        // - x_sym_ (SymbolicVector) for templated simulator code
        // - x_mx_ (raw MX) for janus::Function inputs (CasADi needs original symbol)
        auto [x_sym, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states));
        x_mx_ = x_mx; // Store for later use

        // Set simulator state using SymbolicVector (works with templates)
        sim_.SetState(x_sym);

        // Set simulation time to symbolic
        sim_.SetTime(t_mx_);

        // Compute derivatives (this traces the symbolic graph)
        sim_.ComputeDerivatives(t_mx_);

        // Gather derivatives
        auto xdot_mx = GatherSymbolicDerivatives(n_states);

        // Build janus::Function with the ORIGINAL MX symbols as inputs
        std::vector<janus::SymbolicArg> inputs;
        if (config_.include_time) {
            inputs.push_back(t_mx_);
        }
        inputs.push_back(x_mx_); // Use the raw MX, not the Eigen wrapper

        std::vector<janus::SymbolicArg> outputs = {xdot_mx};

        return janus::Function(config_.function_name, inputs, outputs);
    }

    /**
     * @brief Trace outputs (all signals, not just states)
     * @param output_signals Names of signals to include in output
     * @return Function with signature (t, x) -> (outputs)
     *
     * Useful for computing observables like altitude, velocity magnitude.
     */
    janus::Function TraceOutputs(const std::vector<std::string> &output_signals) {
        std::size_t n_states = sim_.GetTotalStateSize();

        t_mx_ = janus::sym("t");
        auto [x_sym, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states));
        x_mx_ = x_mx;

        sim_.SetState(x_sym);
        sim_.SetTime(t_mx_);
        sim_.ComputeDerivatives(t_mx_);

        // Gather requested outputs into a single column vector
        std::vector<SymbolicScalar> output_elements;
        for (const auto &name : output_signals) {
            output_elements.push_back(sim_.GetSignal(name));
        }

        // Vertcat into single MX
        SymbolicScalar outputs_mx = SymbolicScalar::vertcat(output_elements);

        std::vector<janus::SymbolicArg> inputs = {t_mx_, x_mx_};
        std::vector<janus::SymbolicArg> output_args = {outputs_mx};

        return janus::Function("outputs", inputs, output_args);
    }

    /**
     * @brief Trace a single integration step (discrete-time dynamics)
     * @param dt_value Step size (numeric value)
     * @return Function with signature (t, x) -> (x_next)
     *
     * The function represents: x_{k+1} = step(t_k, x_k, dt)
     * Uses the same integrator configured in the Simulator (e.g., RK4).
     * Useful for discrete optimization and MPC.
     */
    janus::Function TraceStep(double dt_value) {
        std::size_t n_states = sim_.GetTotalStateSize();

        t_mx_ = janus::sym("t");
        auto [x_sym, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states));
        x_mx_ = x_mx;

        // Create derivative function for the integrator
        // This function will be called at each intermediate stage
        auto deriv_func =
            [this](SymbolicScalar t_eval,
                   const JanusVector<SymbolicScalar> &x_eval) -> JanusVector<SymbolicScalar> {
            // Set simulator state to the evaluation point
            sim_.SetState(x_eval);
            sim_.SetTime(t_eval);

            // Compute derivatives
            return sim_.ComputeDerivatives(t_eval);
        };

        // Use the simulator's integrator to perform the step
        auto *integrator = sim_.GetIntegrator();
        JanusVector<SymbolicScalar> x_next_vec =
            integrator->Step(deriv_func, x_sym, t_mx_, SymbolicScalar(dt_value));

        // Convert result back to MX column vector
        std::vector<SymbolicScalar> x_next_elements;
        x_next_elements.reserve(n_states);
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(n_states); ++i) {
            x_next_elements.push_back(x_next_vec(i));
        }
        SymbolicScalar x_next = SymbolicScalar::vertcat(x_next_elements);

        std::vector<janus::SymbolicArg> inputs = {t_mx_, x_mx_};
        std::vector<janus::SymbolicArg> outputs = {x_next};

        return janus::Function("step", inputs, outputs);
    }

    // =========================================================================
    // Metadata
    // =========================================================================

    /**
     * @brief Get state variable names in order
     */
    [[nodiscard]] std::vector<std::string> GetStateNames() const {
        std::vector<std::string> names;
        auto layout = sim_.GetStateLayout();
        for (const auto &slice : layout) {
            std::string comp_name = slice.owner->FullName();
            for (std::size_t i = 0; i < slice.size; ++i) {
                names.push_back(comp_name + ".state[" + std::to_string(i) + "]");
            }
        }
        return names;
    }

    /**
     * @brief Get state vector size
     */
    [[nodiscard]] std::size_t GetStateSize() const { return sim_.GetTotalStateSize(); }

    /**
     * @brief Get control (input) signal names
     */
    [[nodiscard]] std::vector<std::string> GetControlNames() const {
        if (!config_.control_signals.empty()) {
            return config_.control_signals;
        }
        return DetectControlSignals();
    }

    /**
     * @brief Get control vector size
     */
    [[nodiscard]] std::size_t GetControlSize() const { return GetControlNames().size(); }

    /**
     * @brief Get output signal names (all registered signals)
     */
    [[nodiscard]] std::vector<std::string> GetOutputNames() const { return sim_.GetSignalNames(); }

  private:
    Simulator<SymbolicScalar> &sim_;
    TracerConfig config_;

    // Pure symbolic MX variables (not Eigen matrices)
    SymbolicScalar t_mx_;
    SymbolicScalar x_mx_;

    /**
     * @brief Gather symbolic derivatives from simulator and return as MX
     */
    SymbolicScalar GatherSymbolicDerivatives(std::size_t n_states) {
        const auto &X_dot_global = sim_.GetDerivatives();

        // Collect elements into vector for vertcat
        std::vector<SymbolicScalar> xdot_elements;
        xdot_elements.reserve(n_states);
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(n_states); ++i) {
            xdot_elements.push_back(X_dot_global(i));
        }

        // Vertcat into single MX column vector
        return SymbolicScalar::vertcat(xdot_elements);
    }

    /**
     * @brief Detect control signals (inputs not wired to internal sources)
     */
    [[nodiscard]] std::vector<std::string> DetectControlSignals() const {
        // For now, return empty - all inputs are internal
        // In future, this would scan for unwired external inputs
        return {};
    }
};

// =============================================================================
// Convenience Functions
// =============================================================================

/**
 * @brief Quick dynamics extraction (uses default config)
 */
inline janus::Function ExtractDynamics(Simulator<SymbolicScalar> &sim) {
    return SymbolicTracer(sim).TraceDynamics();
}

/**
 * @brief Extract the state Jacobian df/dx as a janus::Function
 * @param sim Staged symbolic simulator
 * @return Function with signature (t, x) -> (J) where J is the n×n Jacobian matrix
 */
inline janus::Function ExtractStateJacobian(Simulator<SymbolicScalar> &sim) {
    std::size_t n_states = sim.GetTotalStateSize();

    // Create symbolic variables using sym_vec_pair
    auto t_sym = janus::sym("t");
    auto [x_vec, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states));

    // Get dynamics function
    auto dynamics = ExtractDynamics(sim);

    // Evaluate dynamics symbolically with the raw MX
    auto xdot_result = dynamics.eval(t_sym, x_mx);

    // Compute Jacobian
    auto J_sym = janus::jacobian({janus::as_mx(xdot_result)}, {x_mx});

    return janus::Function("jacobian", {t_sym, x_mx}, {J_sym});
}

} // namespace icarus::symbolic
