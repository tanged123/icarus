#pragma once

/**
 * @file DataDictionary.hpp
 * @brief Complete catalog of simulation interface
 *
 * Part of Phase 2.4: Component Interface System.
 * Provides structured documentation of all signals, inputs, parameters, and config.
 */

#include <cstddef>
#include <icarus/signal/Signal.hpp>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Complete catalog of simulation interface
 *
 * The DataDictionary provides a complete snapshot of the simulation's
 * interface, including all registered outputs, inputs, parameters, and config.
 */
struct DataDictionary {
    /**
     * @brief Entry for a single component
     */
    struct ComponentEntry {
        std::string name; ///< Full component name (e.g., "X15.MainEngine")
        std::string type; ///< Component type (e.g., "JetEngine")

        std::vector<SignalDescriptor> outputs;    ///< Output signals
        std::vector<SignalDescriptor> inputs;     ///< Input ports
        std::vector<SignalDescriptor> parameters; ///< Scalar parameters (optimizable)
        std::vector<SignalDescriptor> config;     ///< Discrete config (not optimizable)
    };

    /// All registered components
    std::vector<ComponentEntry> components;

    // Summary statistics
    std::size_t total_outputs = 0;
    std::size_t total_inputs = 0;
    std::size_t total_parameters = 0;
    std::size_t total_config = 0;
    std::size_t integrable_states = 0;
    std::size_t unwired_inputs = 0;

    /**
     * @brief Compute summary statistics from components
     */
    void ComputeStats() {
        total_outputs = 0;
        total_inputs = 0;
        total_parameters = 0;
        total_config = 0;
        integrable_states = 0;
        unwired_inputs = 0;

        for (const auto &comp : components) {
            total_outputs += comp.outputs.size();
            total_inputs += comp.inputs.size();
            total_parameters += comp.parameters.size();
            total_config += comp.config.size();

            for (const auto &output : comp.outputs) {
                if (output.is_state) {
                    ++integrable_states;
                }
            }

            for (const auto &input : comp.inputs) {
                if (input.wired_to.empty()) {
                    ++unwired_inputs;
                }
            }
        }
    }
};

} // namespace icarus
