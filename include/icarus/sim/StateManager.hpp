#pragma once

/**
 * @file StateManager.hpp
 * @brief Manages global state vectors and component bindings
 *
 * Part of Phase 4.0.7: Internal Managers
 *
 * StateManager handles:
 * - Allocation of global state vector (X) and derivative vector (X_dot)
 * - Binding component state slices to the global vectors
 * - Providing get/set access to the full state
 *
 * This class IS templated on Scalar for dual-backend support.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/Types.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace icarus {

// =============================================================================
// StateSlice
// =============================================================================

/**
 * @brief Describes a component's slice of the global state vector
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> struct StateSlice {
    std::string component_name; ///< Fully qualified name (Entity.Component)
    std::size_t offset = 0;     ///< Starting index in global state vector
    std::size_t size = 0;       ///< Number of states for this component
};

// =============================================================================
// StateManager
// =============================================================================

/**
 * @brief Manages global state vectors and component bindings
 *
 * Centralizes state vector management that was previously spread
 * across Simulator methods.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class StateManager {
  public:
    StateManager() = default;

    /**
     * @brief Allocate global state vectors from component list
     *
     * Scans all components, computes total state size, and allocates
     * the global state (X) and derivative (X_dot) vectors.
     *
     * @param components List of provisioned components
     */
    void AllocateState(const std::vector<std::unique_ptr<Component<Scalar>>> &components) {
        // Compute total size and build layout
        layout_.clear();
        std::size_t total_size = 0;

        for (const auto &comp : components) {
            if (comp) {
                std::size_t comp_size = comp->StateSize();
                if (comp_size > 0) {
                    StateSlice<Scalar> slice;
                    slice.component_name = comp->FullName();
                    slice.offset = total_size;
                    slice.size = comp_size;
                    layout_.push_back(slice);
                    total_size += comp_size;
                }
            }
        }

        // Allocate vectors
        X_global_ = JanusVector<Scalar>::Zero(static_cast<int>(total_size));
        X_dot_global_ = JanusVector<Scalar>::Zero(static_cast<int>(total_size));

        allocated_ = true;
    }

    /**
     * @brief Bind component state pointers to global vectors
     *
     * Each component receives pointers to its slice of X and X_dot.
     * Must be called after AllocateState().
     *
     * @param components List of components to bind
     */
    void BindComponents(std::vector<std::unique_ptr<Component<Scalar>>> &components) {
        // Iterate layout_ to use stored offsets (computed during AllocateState)
        for (const auto &slice : layout_) {
            // Find matching component by name
            Component<Scalar> *comp_ptr = nullptr;
            for (auto &comp : components) {
                if (comp && comp->FullName() == slice.component_name) {
                    comp_ptr = comp.get();
                    break;
                }
            }

            if (comp_ptr == nullptr) {
                throw std::runtime_error("StateManager::BindComponents: component '" +
                                         slice.component_name +
                                         "' from layout not found in component vector");
            }

            // Use stored offset and size from layout_
            Scalar *x_ptr = X_global_.data() + slice.offset;
            Scalar *x_dot_ptr = X_dot_global_.data() + slice.offset;
            comp_ptr->BindState(x_ptr, x_dot_ptr, slice.size);
        }
    }

    /**
     * @brief Zero all derivatives
     *
     * Called before each Step() to ensure components accumulate derivatives.
     */
    void ZeroDerivatives() { X_dot_global_.setZero(); }

    /**
     * @brief Get total state vector size
     */
    [[nodiscard]] std::size_t TotalSize() const {
        return static_cast<std::size_t>(X_global_.size());
    }

    /**
     * @brief Get copy of current state
     */
    [[nodiscard]] JanusVector<Scalar> GetState() const { return X_global_; }

    /**
     * @brief Set state from external vector
     *
     * @param X New state vector (must match size)
     */
    void SetState(const JanusVector<Scalar> &X) {
        if (X.size() != X_global_.size()) {
            throw std::invalid_argument("State vector size mismatch: expected " +
                                        std::to_string(X_global_.size()) + ", got " +
                                        std::to_string(X.size()));
        }
        X_global_ = X;
    }

    /**
     * @brief Get reference to derivative vector
     */
    [[nodiscard]] const JanusVector<Scalar> &GetDerivatives() const { return X_dot_global_; }

    /**
     * @brief Get mutable reference to derivative vector
     */
    [[nodiscard]] JanusVector<Scalar> &GetDerivativesMutable() { return X_dot_global_; }

    /**
     * @brief Get reference to state vector
     */
    [[nodiscard]] JanusVector<Scalar> &GetStateMutable() { return X_global_; }

    /**
     * @brief Get state layout (component slices)
     */
    [[nodiscard]] const std::vector<StateSlice<Scalar>> &GetLayout() const { return layout_; }

    /**
     * @brief Check if state has been allocated
     */
    [[nodiscard]] bool IsAllocated() const { return allocated_; }

  private:
    JanusVector<Scalar> X_global_;           ///< Global state vector
    JanusVector<Scalar> X_dot_global_;       ///< Global derivative vector
    std::vector<StateSlice<Scalar>> layout_; ///< Per-component layout info
    bool allocated_ = false;                 ///< Explicit allocation tracking
};

} // namespace icarus
