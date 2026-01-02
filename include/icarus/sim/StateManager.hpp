#pragma once

/**
 * @file StateManager.hpp
 * @brief Manages state integration via unified signal model
 *
 * Phase 6: Unified Signal Model
 *
 * StateManager discovers integrable states from the SignalRegistry
 * instead of scanning components. States ARE signals - components
 * register them during Provision() using register_state().
 *
 * This class IS templated on Scalar for dual-backend support.
 */

#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Registry.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace icarus {

// =============================================================================
// StateBinding (Phase 6)
// =============================================================================

/**
 * @brief Binding to a single scalar state value and its derivative
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> struct StateBinding {
    std::string name;                 ///< Signal name (for debugging)
    Scalar *value_ptr = nullptr;      ///< Pointer to state value
    Scalar *derivative_ptr = nullptr; ///< Pointer to derivative
};

// =============================================================================
// StateManager (Phase 6: Unified Signal Model)
// =============================================================================

/**
 * @brief Manages state integration via signal discovery
 *
 * Discovers integrable states from the SignalRegistry during Stage().
 * Components register states with register_state() during Provision(),
 * and StateManager builds the integration vector by gathering pointers.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class StateManager {
  public:
    StateManager() = default;

    /**
     * @brief Discover integrable states from the registry
     *
     * Queries the registry for all state pairs registered via register_state().
     * Builds internal binding vector with pointers to each state/derivative.
     *
     * @param registry SignalRegistry containing registered states
     */
    void DiscoverStates(const SignalRegistry<Scalar> &registry) {
        bindings_.clear();

        // Get all state pairs from the registry
        const auto &pairs = registry.get_state_pairs();

        for (const auto &pair : pairs) {
            StateBinding<Scalar> binding;
            binding.name = pair.value_name;
            binding.value_ptr = static_cast<Scalar *>(pair.value_ptr);
            binding.derivative_ptr = static_cast<Scalar *>(pair.derivative_ptr);
            bindings_.push_back(binding);
        }

        discovered_ = true;
    }

    /**
     * @brief Zero all derivatives
     *
     * Called before each Step() to ensure components accumulate derivatives.
     */
    void ZeroDerivatives() {
        for (auto &binding : bindings_) {
            *binding.derivative_ptr = Scalar{0};
        }
    }

    /**
     * @brief Get total state vector size
     */
    [[nodiscard]] std::size_t TotalSize() const { return bindings_.size(); }

    /**
     * @brief Get copy of current state as a vector
     *
     * Gathers state values from component-owned storage into a JanusVector.
     */
    [[nodiscard]] JanusVector<Scalar> GetState() const {
        JanusVector<Scalar> X(static_cast<Eigen::Index>(bindings_.size()));
        for (std::size_t i = 0; i < bindings_.size(); ++i) {
            X(static_cast<Eigen::Index>(i)) = *bindings_[i].value_ptr;
        }
        return X;
    }

    /**
     * @brief Set state from external vector
     *
     * Scatters values from JanusVector back to component-owned storage.
     *
     * @param X New state vector (must match size)
     */
    void SetState(const JanusVector<Scalar> &X) {
        if (static_cast<std::size_t>(X.size()) != bindings_.size()) {
            throw std::invalid_argument("State vector size mismatch: expected " +
                                        std::to_string(bindings_.size()) + ", got " +
                                        std::to_string(X.size()));
        }
        for (std::size_t i = 0; i < bindings_.size(); ++i) {
            *bindings_[i].value_ptr = X(static_cast<Eigen::Index>(i));
        }
    }

    /**
     * @brief Get copy of current derivatives as a vector
     *
     * Gathers derivative values from component-owned storage.
     */
    [[nodiscard]] JanusVector<Scalar> GetDerivatives() const {
        JanusVector<Scalar> X_dot(static_cast<Eigen::Index>(bindings_.size()));
        for (std::size_t i = 0; i < bindings_.size(); ++i) {
            X_dot(static_cast<Eigen::Index>(i)) = *bindings_[i].derivative_ptr;
        }
        return X_dot;
    }

    /**
     * @brief Get state bindings (for introspection)
     */
    [[nodiscard]] const std::vector<StateBinding<Scalar>> &GetBindings() const { return bindings_; }

    /**
     * @brief Check if states have been discovered
     */
    [[nodiscard]] bool IsAllocated() const { return discovered_; }

  private:
    std::vector<StateBinding<Scalar>> bindings_; ///< Pointers to state/derivative pairs
    bool discovered_ = false;                    ///< True after DiscoverStates() called
};

} // namespace icarus
