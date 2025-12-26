
#include <gtest/gtest.h>
#include <icarus/icarus.hpp>
#include <testing/StatefulComponent.hpp>

// Only compiling this test if Janus Symbolic support is enabled?
// Assuming yes since SymbolicScalar is used.

namespace icarus {
namespace {

// Test component
template <typename Scalar> class SimpleStateComponent : public Component<Scalar> {
  public:
    SimpleStateComponent(const std::string &name) : name_(name) {}

    std::string Name() const override { return name_; }

    void Provision(Backplane<Scalar> &bp, const ComponentConfig &config) override {}

    void Stage(Backplane<Scalar> &bp, const ComponentConfig &config) override {
        if (x_ptr_) {
            *x_ptr_ = 10.0;
        }
    }

    std::size_t StateSize() const override { return 1; }

    void BindState(Scalar *x, Scalar *dx, std::size_t n) override {
        x_ptr_ = x;
        dx_ptr_ = dx;
    }

    void Step(Scalar t, Scalar dt) override {
        if (dx_ptr_ && x_ptr_) {
            // dx/dt = -x
            *dx_ptr_ = -(*x_ptr_);
        }
    }

  private:
    std::string name_;
    Scalar *x_ptr_ = nullptr;
    Scalar *dx_ptr_ = nullptr;
};

// Use SymbolicScalar for this test
using Scalar = SymbolicScalar;

TEST(SimulatorSymbolicTest, GenerateGraphNoSideEffects) {
    // if (!janus::initialized()) {
    //     GTEST_SKIP() << "Janus not initialized (or no CASADI support)";
    // }

    auto sim = SimulationBuilder<Scalar>()
                   .AddComponent(std::make_unique<SimpleStateComponent<Scalar>>("TestComp"))
                   .BuildAndInitialize();

    // Verify initial state
    auto X = sim.GetState();
    // In symbolic mode, 10.0 is a constant expression
    // We can evaluate it or check if it's constant
    // For simplicity, let's just assume it starts at 10.0 and t=0

    // Actually, we can poke a numeric value into the symbolic scalar?
    // Janus SymbolicScalar wraps CasADi MX.
    // If we just check that the pointer/object is *different* from a symbol?
    // Or we can just check if X(0) is a symbol "x_0"?

    // Let's set a distinct time and state
    Scalar t_test = 5.0;
    sim.SetTime(t_test);

    // We can't easily "SetState" to a purely numeric value in Symbolic mode IF the components only
    // work with symbols? But SymbolicScalar SHOULD handle numeric constants.
    JanusVector<Scalar> X_set(1);
    X_set(0) = 42.0;
    sim.SetState(X_set);

    // Check precondition
    // Check precondition - casting to double should SUCCEED for constant values
    double val_pre = static_cast<double>(sim.Time());
    EXPECT_DOUBLE_EQ(val_pre, 5.0);

    // Okay, safer way:
    // GenerateGraph replaces state with SYMBOLS "x".
    // If we verify that AFTER GenerateGraph, state is still 42.0 (constant), then we are good.

    auto graph = sim.GenerateGraph();

    // Verify Post-condition
    // 1. Time should be 5.0
    // 2. State should be 42.0

    // Using janus::eval or just printing?
    // Let's rely on internal representation check if possible, or just re-generation.

    // If side-effects persisted, Time would be a symbol "t".
    // And State would be a symbol "x".

    // Check Time
    Scalar t_after = sim.Time();
    // A symbol "t" usually has a name. A constant 5.0 does not (or name is null/empty?).
    // Or we can try to evaluate it.

    // If we can construct a Function that returns the time and evaluate it with dummy inputs.
    // But simpler: just check if it IS NOT a symbol.
    // Janus SymbolicScalar has `is_symbolic()`? No...
    // But we know GenerateGraph sets time to `janus::sym("t")`.

    // Let's assume if we can evaluate it to a double without providing inputs, it's a constant.
    // BUT SymbolicScalar conversion to double throws if it has symbols.

    try {
        double t_val = static_cast<double>(t_after); // Should succeed if it's 5.0
        EXPECT_DOUBLE_EQ(t_val, 5.0);
    } catch (...) {
        FAIL() << "Simulation Time became symbolic after GenerateGraph()";
    }

    try {
        JanusVector<Scalar> X_after = sim.GetState();
        double x_val = static_cast<double>(X_after(0)); // Should succeed if it's 42.0
        EXPECT_DOUBLE_EQ(x_val, 42.0);
    } catch (...) {
        FAIL() << "Simulation State became symbolic after GenerateGraph()";
    }
}

} // namespace
} // namespace icarus
