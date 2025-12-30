#pragma once

/**
 * @file DummyComponent.hpp
 * @brief Minimal component for testing lifecycle
 *
 * Part of Phase 1.4: Component Base.
 * A simple component that outputs a counter and time for testing.
 */

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>

namespace icarus {

/**
 * @brief Minimal component for testing lifecycle
 *
 * Outputs a single signal that increments each step.
 */
template <typename Scalar> class DummyComponent : public Component<Scalar> {
  public:
    explicit DummyComponent(std::string name = "Dummy", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    void Provision(Backplane<Scalar> &bp) override {
        bp.register_output("counter", &counter_, "", "Step counter");
        bp.register_output("time", &last_time_, "s", "Last step time");
    }

    void Stage(Backplane<Scalar> &) override {
        counter_ = Scalar{0};
        last_time_ = Scalar{0};
    }

    void Step(Scalar t, Scalar /*dt*/) override {
        counter_ = counter_ + Scalar{1};
        last_time_ = t;
    }

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "DummyComponent"; }

    [[nodiscard]] std::vector<SignalDecl> DeclareOutputs() const override {
        return {{"counter", "", "Step counter", false}, {"time", "s", "Last step time", false}};
    }

    // Expose values for testing
    [[nodiscard]] Scalar counter() const { return counter_; }
    [[nodiscard]] Scalar last_time() const { return last_time_; }

  private:
    std::string name_;
    std::string entity_;
    Scalar counter_{};
    Scalar last_time_{};
};

} // namespace icarus
