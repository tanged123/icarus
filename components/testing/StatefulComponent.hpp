#pragma once

/**
 * @file StatefulComponent.hpp
 * @brief Test component with integrated state for testing state management
 *
 * Implements simple exponential decay dynamics: x_dot = -x
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
        : name_(std::move(name)), state_size_(state_size) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::size_t StateSize() const override { return state_size_; }

    void Provision(Backplane<Scalar> &bp, const ComponentConfig & /*config*/) override {
        // Register output for observability
        bp.register_output("state_bound", &state_bound_);
    }

    void Stage(Backplane<Scalar> & /*bp*/, const ComponentConfig & /*config*/) override {
        // Nothing beyond state binding
    }

    void BindState(Scalar *state, Scalar *state_dot, std::size_t size) override {
        if (size != state_size_) {
            throw StateSizeMismatchError(state_size_, size);
        }
        state_ptr_ = state;
        state_dot_ptr_ = state_dot;
        state_bound_ = Scalar{1};
    }

    void Step(Scalar /*t*/, Scalar /*dt*/) override {
        // Simple dynamics: x_dot = -x (exponential decay)
        for (std::size_t i = 0; i < state_size_; ++i) {
            state_dot_ptr_[i] = -state_ptr_[i];
        }
    }

    // Accessors for testing
    [[nodiscard]] Scalar *GetStatePtr() { return state_ptr_; }
    [[nodiscard]] Scalar *GetStateDotPtr() { return state_dot_ptr_; }
    [[nodiscard]] bool IsBound() const {
        return state_ptr_ != nullptr && state_dot_ptr_ != nullptr;
    }

  private:
    std::string name_;
    std::size_t state_size_;
    Scalar *state_ptr_ = nullptr;
    Scalar *state_dot_ptr_ = nullptr;
    Scalar state_bound_{0};
};

} // namespace icarus
