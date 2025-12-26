#pragma once

/**
 * @file SimulationBuilder.hpp
 * @brief Fluent builder for Simulator configuration
 *
 * Part of Phase 3: Symbolic Mode.
 * Provides a clean API for simulation setup that works identically
 * for both numeric (double) and symbolic (SymbolicScalar) modes.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/sim/Integrator.hpp>
#include <icarus/sim/RK4Integrator.hpp>
#include <icarus/sim/Simulator.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace icarus {

/**
 * @brief Fluent builder for Simulator configuration
 *
 * Usage:
 *   auto sim = SimulationBuilder<double>()
 *       .AddComponent(std::make_unique<Gravity>("Gravity"))
 *       .AddComponent(std::make_unique<Vehicle>(mass, "Vehicle"))
 *       .Wire("Gravity", {{"position.x", "Vehicle.position.x"}})
 *       .Wire("Vehicle", {{"force.x", "Gravity.force.x"}})
 *       .SetIntegrator<RK4Integrator>()
 *       .EnableProfiling()
 *       .Build();
 *
 * @tparam Scalar The numeric type (double or SymbolicScalar)
 */
template <typename Scalar> class SimulationBuilder {
  public:
    SimulationBuilder() = default;

    // =========================================================================
    // Component Configuration
    // =========================================================================

    /**
     * @brief Add a component to the simulation
     * @param component Owned component instance
     * @return Reference to builder for chaining
     *
     * Components are added in order; execution order follows addition order.
     */
    SimulationBuilder &AddComponent(std::unique_ptr<Component<Scalar>> component) {
        components_.push_back(std::move(component));
        return *this;
    }

    /**
     * @brief Convenience: construct component in-place
     * @tparam ComponentT Component type to construct
     * @tparam Args Constructor argument types
     * @param args Constructor arguments forwarded to ComponentT
     */
    template <typename ComponentT, typename... Args>
    SimulationBuilder &AddComponent(Args &&...args) {
        components_.push_back(std::make_unique<ComponentT>(std::forward<Args>(args)...));
        return *this;
    }

    // =========================================================================
    // Wiring Configuration
    // =========================================================================

    /**
     * @brief Wire inputs for a target component
     * @param target_component Component name (with optional entity prefix)
     * @param wiring Map of input_name -> source_signal_name
     * @return Reference to builder for chaining
     *
     * Example:
     *   .Wire("Gravity", {
     *       {"position.x", "Vehicle.position.x"},
     *       {"mass", "Vehicle.mass"}
     *   })
     */
    SimulationBuilder &Wire(const std::string &target_component,
                            const std::map<std::string, std::string> &wiring) {
        wiring_.emplace_back(target_component, wiring);
        return *this;
    }

    /**
     * @brief Wire a single input
     * @param target_input Full path: "Component.input_name"
     * @param source_output Full path: "Component.output_name"
     */
    SimulationBuilder &Wire(const std::string &target_input, const std::string &source_output) {
        // Parse component name from target_input
        auto dot_pos = target_input.find('.');
        if (dot_pos == std::string::npos) {
            throw ConfigurationError("Wire() target must be in format 'Component.input': " +
                                     target_input);
        }
        std::string component = target_input.substr(0, dot_pos);
        std::string input = target_input.substr(dot_pos + 1);
        wiring_.emplace_back(component, std::map<std::string, std::string>{{input, source_output}});
        return *this;
    }

    // =========================================================================
    // Integrator Configuration
    // =========================================================================

    /**
     * @brief Set the integrator instance
     * @param integrator Owned integrator instance
     */
    SimulationBuilder &SetIntegrator(std::unique_ptr<Integrator<Scalar>> integrator) {
        integrator_ = std::move(integrator);
        return *this;
    }

    /**
     * @brief Convenience: construct integrator in-place
     * @tparam IntegratorT Integrator type (e.g., RK4Integrator)
     * @tparam Args Constructor argument types
     */
    template <typename IntegratorT, typename... Args>
    SimulationBuilder &SetIntegrator(Args &&...args) {
        integrator_ = std::make_unique<IntegratorT>(std::forward<Args>(args)...);
        return *this;
    }

    // =========================================================================
    // Simulation Parameters
    // =========================================================================

    /**
     * @brief Set the default time step
     * @param dt Time step in seconds
     */
    SimulationBuilder &SetTimeStep(Scalar dt) {
        dt_ = dt;
        return *this;
    }

    /**
     * @brief Set the simulation end time
     * @param t_end End time in seconds
     */
    SimulationBuilder &SetEndTime(Scalar t_end) {
        t_end_ = t_end;
        return *this;
    }

    // =========================================================================
    // Logging & Profiling
    // =========================================================================

    /**
     * @brief Enable component execution profiling
     */
    SimulationBuilder &EnableProfiling(bool enable = true) {
        profiling_enabled_ = enable;
        return *this;
    }

    /**
     * @brief Set log file path
     * @param path Path to log file (empty disables file logging)
     */
    SimulationBuilder &SetLogFile(const std::string &path) {
        log_file_ = path;
        return *this;
    }

    /**
     * @brief Disable all logging (useful for symbolic mode)
     */
    SimulationBuilder &DisableLogging() {
        logging_disabled_ = true;
        return *this;
    }

    // =========================================================================
    // Build
    // =========================================================================

    /**
     * @brief Build the configured simulator
     * @return Fully constructed Simulator (components added, NOT provisioned)
     *
     * After Build():
     * - All components added to simulator
     * - Wiring configuration stored (applied at Stage)
     * - Integrator set
     *
     * Caller must still call Provision() and Stage(), or use BuildAndInitialize().
     */
    Simulator<Scalar> Build() {
        Simulator<Scalar> sim;

        // Configure logging
        if (logging_disabled_) {
            sim.SetQuietMode(true);
        }
        if (!log_file_.empty()) {
            sim.SetLogFile(log_file_);
        }
        if (profiling_enabled_) {
            sim.SetProfilingEnabled(true);
        }

        // Add components
        for (auto &comp : components_) {
            sim.AddComponent(std::move(comp));
        }
        components_.clear();

        // Set integrator (default to RK4 if not specified)
        if (integrator_) {
            sim.SetIntegrator(std::move(integrator_));
        }
        // else: Simulator already defaults to RK4

        // Set nominal timestep
        sim.SetNominalDt(dt_);

        // Store wiring (applied during Stage)
        for (const auto &[target, wiring_map] : wiring_) {
            sim.SetWiring(target, wiring_map);
        }

        return sim;
    }

    /**
     * @brief Build and initialize (Provision + Stage)
     * @return Ready-to-run Simulator
     */
    Simulator<Scalar> BuildAndInitialize() {
        Simulator<Scalar> sim = Build();
        sim.Provision();
        sim.Stage();
        return sim;
    }

    // =========================================================================
    // Configuration Getters (for inspection)
    // =========================================================================

    /**
     * @brief Get the configured time step
     */
    [[nodiscard]] Scalar GetTimeStep() const { return dt_; }

    /**
     * @brief Get the configured end time
     */
    [[nodiscard]] Scalar GetEndTime() const { return t_end_; }

    // =========================================================================
    // Phase 5 Forward-Compatible Stubs
    // =========================================================================

    /**
     * @brief Load scenario configuration (Layer B)
     * @param path Path to scenario YAML/JSON
     * @throws NotImplementedError (Phase 5)
     */
    SimulationBuilder &LoadScenario(const std::string & /*path*/) {
        throw NotImplementedError("LoadScenario() is not yet implemented (Phase 5)");
    }

    /**
     * @brief Load services configuration (Layer F)
     * @param path Path to services YAML/JSON
     * @throws NotImplementedError (Phase 5)
     */
    SimulationBuilder &LoadServices(const std::string & /*path*/) {
        throw NotImplementedError("LoadServices() is not yet implemented (Phase 5)");
    }

    /**
     * @brief Load trim/optimization configuration (Layer E)
     * @param path Path to trim config YAML/JSON
     * @throws NotImplementedError (Phase 5)
     */
    SimulationBuilder &LoadTrimConfig(const std::string & /*path*/) {
        throw NotImplementedError("LoadTrimConfig() is not yet implemented (Phase 5)");
    }

    /**
     * @brief Override a parameter after loading config
     * @throws NotImplementedError (Phase 5)
     */
    SimulationBuilder &OverrideParam(const std::string & /*signal_path*/, Scalar /*value*/) {
        throw NotImplementedError("OverrideParam() is not yet implemented (Phase 5)");
    }

    /**
     * @brief Override initial condition after loading config
     * @throws NotImplementedError (Phase 5)
     */
    SimulationBuilder &OverrideInitialCondition(const std::string & /*state_path*/,
                                                Scalar /*value*/) {
        throw NotImplementedError("OverrideInitialCondition() is not yet implemented (Phase 5)");
    }

    /**
     * @brief Set configuration search paths
     * @throws NotImplementedError (Phase 5)
     */
    SimulationBuilder &SetConfigPaths(const std::vector<std::string> & /*paths*/) {
        throw NotImplementedError("SetConfigPaths() is not yet implemented (Phase 5)");
    }

  private:
    std::vector<std::unique_ptr<Component<Scalar>>> components_;
    std::vector<std::pair<std::string, std::map<std::string, std::string>>> wiring_;
    std::unique_ptr<Integrator<Scalar>> integrator_;

    Scalar dt_ = Scalar{0.01};
    Scalar t_end_ = Scalar{1.0};
    bool profiling_enabled_ = false;
    std::string log_file_;
    bool logging_disabled_ = false;
};

} // namespace icarus
