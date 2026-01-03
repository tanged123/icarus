#pragma once

/**
 * @file PhaseManager.hpp
 * @brief Flight phase transition management
 *
 * Part of Phase 6.1: Phase Manager.
 * Coordinates flight phase transitions as a pure orchestration concern.
 * Components remain phase-agnostic - phase logic is applied externally.
 */

#include <icarus/core/ConditionParser.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/signal/Registry.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Configuration for a single phase transition
 */
struct PhaseTransition {
    int32_t from_phase = -1; ///< Source phase (-1 = any phase)
    int32_t to_phase = 0;    ///< Destination phase
    std::string condition;   ///< Condition expression (e.g., "fuel_mass < 0.01")

    PhaseTransition() = default;
    PhaseTransition(int32_t from, int32_t to, std::string cond)
        : from_phase(from), to_phase(to), condition(std::move(cond)) {}
};

/**
 * @brief Configuration for phase management
 */
struct PhaseConfig {
    /// Phase name to integer mapping (e.g., {"GROUND": 0, "BOOST": 1, ...})
    std::map<std::string, int32_t> definitions;

    /// Initial phase name
    std::string initial_phase;

    /// Transition rules
    std::vector<PhaseTransition> transitions;

    /// Entity prefix for phase signal (e.g., "Vehicle" -> "Vehicle.phase")
    std::string entity_prefix;

    /// Get phase integer from name, or nullopt if not found
    [[nodiscard]] std::optional<int32_t> GetPhaseValue(const std::string &name) const {
        auto it = definitions.find(name);
        if (it != definitions.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    /// Get phase name from integer, or empty string if not found
    [[nodiscard]] std::string GetPhaseName(int32_t value) const {
        for (const auto &[name, val] : definitions) {
            if (val == value) {
                return name;
            }
        }
        return "";
    }
};

/**
 * @brief Manages flight phase transitions
 *
 * The PhaseManager tracks the current phase and evaluates transition conditions
 * after each simulation step. It follows no-cascade semantics: at most one
 * transition fires per step.
 *
 * Example usage:
 * @code
 * PhaseConfig config;
 * config.definitions = {{"GROUND", 0}, {"BOOST", 1}, {"COAST", 2}};
 * config.initial_phase = "GROUND";
 * config.transitions = {
 *     {0, 1, "Propulsion.ignition == 1"},
 *     {1, 2, "Propulsion.fuel_mass < 0.01"}
 * };
 *
 * PhaseManager<double> pm;
 * pm.Configure(config);
 *
 * // After each Step():
 * pm.EvaluateTransitions(registry);
 * if (pm.PhaseChangedThisStep()) {
 *     // Handle phase change
 * }
 * @endcode
 */
template <typename Scalar> class PhaseManager {
  public:
    PhaseManager() = default;

    /**
     * @brief Configure phase manager with phase definitions and transitions
     *
     * @param config Phase configuration
     * @throws ConfigError if initial phase not found or transitions invalid
     */
    void Configure(const PhaseConfig &config) {
        config_ = config;

        // Validate and set initial phase
        auto initial_value = config_.GetPhaseValue(config_.initial_phase);
        if (!initial_value.has_value()) {
            throw ConfigError("PhaseManager", "initial_phase '" + config_.initial_phase +
                                                  "' not found in definitions");
        }
        current_phase_ = *initial_value;
        previous_phase_ = current_phase_;

        // Pre-compile all transition conditions
        ConditionParser<Scalar> parser;
        compiled_conditions_.clear();
        compiled_conditions_.reserve(config_.transitions.size());

        for (size_t i = 0; i < config_.transitions.size(); ++i) {
            const auto &trans = config_.transitions[i];
            try {
                compiled_conditions_.push_back(parser.Parse(trans.condition));
            } catch (const ConditionError &e) {
                throw ConfigError("PhaseManager", "invalid condition in transition " +
                                                      std::to_string(i) + ": " + e.what());
            }
        }

        configured_ = true;
    }

    /**
     * @brief Evaluate transition conditions and potentially change phase
     *
     * Called after each Step(). Evaluates all applicable transitions in order
     * and fires the first one whose condition is true. No-cascade: at most
     * one transition per call.
     *
     * @param registry Signal registry for condition evaluation
     */
    void EvaluateTransitions(const SignalRegistry<Scalar> &registry) {
        if (!configured_) {
            return;
        }

        previous_phase_ = current_phase_;
        phase_changed_ = false;

        for (size_t i = 0; i < config_.transitions.size(); ++i) {
            const auto &trans = config_.transitions[i];

            // Check if transition applies to current phase
            if (trans.from_phase != -1 && trans.from_phase != current_phase_) {
                continue;
            }

            // Evaluate condition
            try {
                if (compiled_conditions_[i].Evaluate(registry)) {
                    // Transition fires
                    current_phase_ = trans.to_phase;
                    phase_changed_ = true;
                    break; // No cascade: stop after first transition
                }
            } catch (const SignalNotFoundError &) {
                // Signal not found - condition cannot be evaluated
                // Skip this transition silently (signal may not exist yet)
            }
        }
    }

    /**
     * @brief Get current phase value
     */
    [[nodiscard]] int32_t CurrentPhase() const { return current_phase_; }

    /**
     * @brief Get current phase name
     */
    [[nodiscard]] std::string CurrentPhaseName() const {
        return config_.GetPhaseName(current_phase_);
    }

    /**
     * @brief Get previous phase value (before last EvaluateTransitions)
     */
    [[nodiscard]] int32_t PreviousPhase() const { return previous_phase_; }

    /**
     * @brief Check if phase changed in last EvaluateTransitions call
     */
    [[nodiscard]] bool PhaseChangedThisStep() const { return phase_changed_; }

    /**
     * @brief Check if phase manager is configured
     */
    [[nodiscard]] bool IsConfigured() const { return configured_; }

    /**
     * @brief Get phase signal path for this entity
     *
     * @return Full signal path (e.g., "Vehicle.phase")
     */
    [[nodiscard]] std::string GetPhaseSignalPath() const {
        if (config_.entity_prefix.empty()) {
            return "phase";
        }
        return config_.entity_prefix + ".phase";
    }

    /**
     * @brief Get the phase configuration
     */
    [[nodiscard]] const PhaseConfig &GetConfig() const { return config_; }

    /**
     * @brief Reset to initial phase
     */
    void Reset() {
        if (configured_) {
            auto initial_value = config_.GetPhaseValue(config_.initial_phase);
            if (initial_value.has_value()) {
                current_phase_ = *initial_value;
                previous_phase_ = current_phase_;
                phase_changed_ = false;
            }
        }
    }

    /**
     * @brief Force set phase (for testing or initialization)
     *
     * @param phase_name Phase name to set
     * @throws ConfigError if phase name not found
     */
    void SetPhase(const std::string &phase_name) {
        auto value = config_.GetPhaseValue(phase_name);
        if (!value.has_value()) {
            throw ConfigError("PhaseManager", "phase '" + phase_name + "' not found");
        }
        previous_phase_ = current_phase_;
        current_phase_ = *value;
        phase_changed_ = (current_phase_ != previous_phase_);
    }

    /**
     * @brief Force set phase by value (for testing or initialization)
     */
    void SetPhaseValue(int32_t value) {
        previous_phase_ = current_phase_;
        current_phase_ = value;
        phase_changed_ = (current_phase_ != previous_phase_);
    }

  private:
    PhaseConfig config_;
    std::vector<CompiledCondition<Scalar>> compiled_conditions_;

    int32_t current_phase_ = 0;
    int32_t previous_phase_ = 0;
    bool phase_changed_ = false;
    bool configured_ = false;
};

} // namespace icarus
