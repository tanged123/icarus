#pragma once

/**
 * @file DataDictionary.hpp
 * @brief Complete catalog of simulation interface
 *
 * Part of Phase 2.4: Component Interface System.
 * Provides structured documentation of all signals, inputs, parameters, and config.
 */

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <icarus/core/Error.hpp>
#include <icarus/signal/Signal.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

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

    /**
     * @brief Export to YAML file
     * @param path Output file path
     */
    void ToYAML(const std::string &path) const {
        YAML::Emitter out;
        out << YAML::BeginMap;

        // Summary section
        out << YAML::Key << "summary" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "total_outputs" << YAML::Value << total_outputs;
        out << YAML::Key << "total_inputs" << YAML::Value << total_inputs;
        out << YAML::Key << "total_parameters" << YAML::Value << total_parameters;
        out << YAML::Key << "total_config" << YAML::Value << total_config;
        out << YAML::Key << "integrable_states" << YAML::Value << integrable_states;
        out << YAML::Key << "unwired_inputs" << YAML::Value << unwired_inputs;
        out << YAML::EndMap;

        // Components section
        out << YAML::Key << "components" << YAML::Value << YAML::BeginSeq;
        for (const auto &comp : components) {
            out << YAML::BeginMap;
            out << YAML::Key << "name" << YAML::Value << comp.name;
            out << YAML::Key << "type" << YAML::Value << comp.type;

            auto emit_signals = [&out](const std::string &key,
                                       const std::vector<SignalDescriptor> &signals) {
                if (!signals.empty()) {
                    out << YAML::Key << key << YAML::Value << YAML::BeginSeq;
                    for (const auto &sig : signals) {
                        out << YAML::BeginMap;
                        out << YAML::Key << "name" << YAML::Value << sig.name;
                        out << YAML::Key << "unit" << YAML::Value << sig.unit;
                        out << YAML::Key << "description" << YAML::Value << sig.description;
                        if (!sig.wired_to.empty()) {
                            out << YAML::Key << "wired_to" << YAML::Value << sig.wired_to;
                        }
                        if (sig.is_state) {
                            out << YAML::Key << "is_state" << YAML::Value << true;
                        }
                        out << YAML::EndMap;
                    }
                    out << YAML::EndSeq;
                }
            };

            emit_signals("outputs", comp.outputs);
            emit_signals("inputs", comp.inputs);
            emit_signals("parameters", comp.parameters);
            emit_signals("config", comp.config);

            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;

        std::ofstream file(path);
        if (!file.is_open()) {
            throw IOError("Failed to open file for writing: '" + path +
                          "': " + std::strerror(errno));
        }

        file << out.c_str();

        if (file.fail()) {
            throw IOError("Failed to write to file: '" + path + "': " + std::strerror(errno));
        }

        file.close();
    }

    /**
     * @brief Export to JSON file
     * @param path Output file path
     */
    void ToJSON(const std::string &path) const {
        nlohmann::json j;

        // Summary
        j["summary"]["total_outputs"] = total_outputs;
        j["summary"]["total_inputs"] = total_inputs;
        j["summary"]["total_parameters"] = total_parameters;
        j["summary"]["total_config"] = total_config;
        j["summary"]["integrable_states"] = integrable_states;
        j["summary"]["unwired_inputs"] = unwired_inputs;

        // Components
        j["components"] = nlohmann::json::array();
        for (const auto &comp : components) {
            nlohmann::json jcomp;
            jcomp["name"] = comp.name;
            jcomp["type"] = comp.type;

            auto to_json_signals = [](const std::vector<SignalDescriptor> &signals) {
                nlohmann::json arr = nlohmann::json::array();
                for (const auto &sig : signals) {
                    nlohmann::json jsig;
                    jsig["name"] = sig.name;
                    jsig["unit"] = sig.unit;
                    jsig["description"] = sig.description;
                    if (!sig.wired_to.empty()) {
                        jsig["wired_to"] = sig.wired_to;
                    }
                    if (sig.is_state) {
                        jsig["is_state"] = true;
                    }
                    arr.push_back(jsig);
                }
                return arr;
            };

            jcomp["outputs"] = to_json_signals(comp.outputs);
            jcomp["inputs"] = to_json_signals(comp.inputs);
            jcomp["parameters"] = to_json_signals(comp.parameters);
            jcomp["config"] = to_json_signals(comp.config);

            j["components"].push_back(jcomp);
        }

        std::ofstream file(path);
        if (!file.is_open()) {
            throw IOError("Failed to open file for writing: '" + path +
                          "': " + std::strerror(errno));
        }

        file << j.dump(2);

        if (file.fail()) {
            throw IOError("Failed to write to file: '" + path + "': " + std::strerror(errno));
        }

        file.close();
    }
};

} // namespace icarus
