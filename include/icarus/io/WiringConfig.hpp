#pragma once

/**
 * @file WiringConfig.hpp
 * @brief Wiring configuration for component input connections
 *
 * Part of Phase 2.4: Component Interface System.
 * Maps input ports to their source signals.
 */

#include <icarus/core/Error.hpp>
#include <string>
#include <unordered_map>

namespace icarus {

/**
 * @brief Wiring configuration mapping inputs to sources
 *
 * Used to configure how component inputs are connected to output signals.
 * Can be populated programmatically or from YAML configuration.
 */
class WiringConfig {
  public:
    WiringConfig() = default;

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
