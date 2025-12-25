#pragma once

/**
 * @file WiringConfig.hpp
 * @brief Wiring configuration for component input connections
 *
 * Part of Phase 2.4: Component Interface System.
 * Maps input ports to their source signals.
 */

#include <fstream>
#include <icarus/core/Error.hpp>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace icarus {

/**
 * @brief Wiring configuration mapping inputs to sources
 *
 * Used to configure how component inputs are connected to output signals.
 * Can be populated programmatically or from YAML configuration.
 *
 * YAML Format:
 * @code
 * wiring:
 *   # Fully-qualified format
 *   PointMass3DOF.force: "Gravity.force"
 *
 *   # Or nested format
 *   PointMass3DOF:
 *     force: "Gravity.force"
 * @endcode
 */
class WiringConfig {
  public:
    WiringConfig() = default;

    /**
     * @brief Load wiring from YAML file
     *
     * @param path Path to YAML file
     * @return WiringConfig populated from file
     * @throws ConfigError if file cannot be read or parsed
     */
    static WiringConfig FromFile(const std::string &path) {
        try {
            YAML::Node node = YAML::LoadFile(path);
            return FromYAML(node);
        } catch (const YAML::Exception &e) {
            throw ConfigError("Failed to parse wiring file '" + path + "': " + e.what());
        }
    }

    /**
     * @brief Load wiring from YAML node
     *
     * Supports two formats:
     * 1. Flat: `"Component.input": "Source.output"`
     * 2. Nested: `Component: { input: "Source.output" }`
     *
     * @param node YAML node (expects "wiring" key or direct content)
     * @return WiringConfig populated from YAML
     */
    static WiringConfig FromYAML(const YAML::Node &node) {
        WiringConfig config;

        // Handle root-level "wiring" key or direct content
        YAML::Node wiring_node = node["wiring"] ? node["wiring"] : node;

        if (!wiring_node.IsMap()) {
            return config; // Empty config if not a map
        }

        for (const auto &entry : wiring_node) {
            std::string key = entry.first.as<std::string>();

            if (entry.second.IsScalar()) {
                // Flat format: "Component.input": "Source.output"
                config.AddWiring(key, entry.second.as<std::string>());
            } else if (entry.second.IsMap()) {
                // Nested format: Component: { input: "Source.output" }
                for (const auto &sub_entry : entry.second) {
                    std::string input_name = key + "." + sub_entry.first.as<std::string>();
                    config.AddWiring(input_name, sub_entry.second.as<std::string>());
                }
            }
        }

        return config;
    }

    /**
     * @brief Add a wiring connection
     *
     * @param input_name Full name of the input port
     * @param source_name Full name of the source signal
     */
    void AddWiring(const std::string &input_name, const std::string &source_name) {
        wirings_[input_name] = source_name;
    }

    /**
     * @brief Get the source signal for an input
     *
     * @param input_name Full name of the input port
     * @return Source signal name
     * @throws WiringError if input not found
     */
    [[nodiscard]] std::string GetSource(const std::string &input_name) const {
        auto it = wirings_.find(input_name);
        if (it == wirings_.end()) {
            throw WiringError("No wiring for input: '" + input_name + "'");
        }
        return it->second;
    }

    /**
     * @brief Check if input has wiring specified
     */
    [[nodiscard]] bool HasWiring(const std::string &input_name) const {
        return wirings_.contains(input_name);
    }

    /**
     * @brief Get all wirings
     */
    [[nodiscard]] const std::unordered_map<std::string, std::string> &GetAllWirings() const {
        return wirings_;
    }

    /**
     * @brief Get number of wirings
     */
    [[nodiscard]] std::size_t size() const { return wirings_.size(); }

    /**
     * @brief Check if empty
     */
    [[nodiscard]] bool empty() const { return wirings_.empty(); }

    /**
     * @brief Clear all wirings
     */
    void clear() { wirings_.clear(); }

  private:
    // input_name -> source_name
    std::unordered_map<std::string, std::string> wirings_;
};

} // namespace icarus
