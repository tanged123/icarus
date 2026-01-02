#pragma once

/**
 * @file StatefulComponent.hpp
 * @brief Test component with integrated state for testing state management
 *
 * Implements simple exponential decay dynamics: x_dot = -x
 *
 * Updated for Phase 6: Uses unified signal model with register_state().
 */

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>

namespace icarus {

/**
 * @brief Component with integrated state for testing state management
 *
 * Implements simple exponential decay dynamics: x_dot = -x
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class StatefulComponent : public Component<Scalar> {
  public:
    explicit StatefulComponent(std::string name, std::size_t state_size = 3)
        : name_(std::move(name)), state_size_(state_size) {
        // Initialize state vectors with zeros
        state_.resize(state_size_);
        state_dot_.resize(state_size_);
        for (std::size_t i = 0; i < state_size_; ++i) {
            state_[i] = Scalar{0};
            state_dot_[i] = Scalar{0};
        }
    }

    [[nodiscard]] std::string Name() const override { return name_; }

    /**
     * @brief Get the number of state variables (for testing purposes)
     *
     * Note: This is NOT an override - StateSize() was removed from Component base.
     */
    [[nodiscard]] std::size_t GetStateSize() const { return state_size_; }

    void Provision(Backplane<Scalar> &bp) override {
        // Register states using the unified signal model
        // Each scalar state creates a value output and a derivative signal
        for (std::size_t i = 0; i < state_size_; ++i) {
            std::string name = "state_" + std::to_string(i);
            bp.template register_state<Scalar>(name, &state_[i], &state_dot_[i], "",
                                               "Test state " + std::to_string(i));
        }

        // Register output for observability
        bp.template register_output<Scalar>("state_bound", &state_bound_, "", "State bound flag");
    }

    void Stage(Backplane<Scalar> &) override {
        // Mark as bound (for compatibility with tests)
        state_bound_ = Scalar{1};
    }

    void Step(Scalar /*t*/, Scalar /*dt*/) override {
        // Simple dynamics: x_dot = -x (exponential decay)
        for (std::size_t i = 0; i < state_size_; ++i) {
            state_dot_[i] = -state_[i];
        }
    }

    // Accessors for testing
    [[nodiscard]] Scalar *GetStatePtr() { return state_.data(); }
    [[nodiscard]] Scalar *GetStateDotPtr() { return state_dot_.data(); }
    [[nodiscard]] bool IsBound() const { return state_bound_ > Scalar{0}; }

    // Direct state access for testing
    [[nodiscard]] const std::vector<Scalar> &GetState() const { return state_; }
    [[nodiscard]] const std::vector<Scalar> &GetStateDot() const { return state_dot_; }

    void SetState(std::size_t idx, Scalar value) {
        if (idx < state_size_) {
            state_[idx] = value;
        }
    }

  private:
    std::string name_;
    std::size_t state_size_;
    std::vector<Scalar> state_;
    std::vector<Scalar> state_dot_;
    Scalar state_bound_{0};
};

} // namespace icarus
