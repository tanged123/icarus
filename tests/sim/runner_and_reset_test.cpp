
#include <gtest/gtest.h>
#include <icarus/icarus.hpp>
#include <testing/StatefulComponent.hpp>

namespace icarus {
namespace {

// Test component that sets initial condition
// Test component that sets initial condition
template <typename Scalar> class IcComponent : public Component<Scalar> {
  public:
    IcComponent(const std::string &name, Scalar ic_value) : name_(name), ic_value_(ic_value) {
        // Allocate state
        x_val_ = ic_value;
    }

    std::string Name() const override { return name_; }

    void Provision(Backplane<Scalar> &bp, const ComponentConfig &config) override {
        // Register local member as output
        bp.template register_output<Scalar>("state", &x_val_, "My Unit");
    }

    void Stage(Backplane<Scalar> &bp, const ComponentConfig &config) override {
        if (x_ptr_) {
            *x_ptr_ = ic_value_;
        }
        // We can resolve our own output to test resolution, or just trust x_val_
        // But for the test logic "output_ = ...", let's just update x_val_ in Step
    }

    std::size_t StateSize() const override { return 1; }

    void BindState(Scalar *x, Scalar *dx, std::size_t n) override {
        x_ptr_ = x;
        dx_ptr_ = dx;
        // Initialize state from internal value if needed, or vice-versa
        if (x_ptr_)
            *x_ptr_ = ic_value_;
    }

    void Step(Scalar t, Scalar dt) override {
        if (dx_ptr_) {
            *dx_ptr_ = 1.0; // dx/dt = 1
        }
        // Sync state to output (if they were different, but here "state" output IS x_val_?)
        // Actually, usually output is a computed value or state itself.
        // Let's make the "state" output be bound to x_val_ which we update from x_ptr_
        if (x_ptr_) {
            x_val_ = *x_ptr_;
        }
    }

  private:
    std::string name_;
    Scalar ic_value_;
    Scalar x_val_; // The storage for the output
    Scalar *x_ptr_ = nullptr;
    Scalar *dx_ptr_ = nullptr;
};

TEST(SimulatorTest, ResetRestoresInitialConditions) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent(std::make_unique<IcComponent<double>>("Test", 10.0))
                   .BuildAndInitialize();

    // Verify initial state
    auto X = sim.GetState();
    EXPECT_DOUBLE_EQ(X(0), 10.0);
    EXPECT_DOUBLE_EQ(sim.Time(), 0.0);

    // Run for a bit
    sim.Step(1.0);
    X = sim.GetState();
    EXPECT_DOUBLE_EQ(X(0), 11.0); // 10 + 1*1
    EXPECT_DOUBLE_EQ(sim.Time(), 1.0);

    // Reset
    sim.Reset();

    // Verify reset state
    X = sim.GetState();
    // Prior to fix, this would be 0.0. With fix, it should be 10.0
    EXPECT_DOUBLE_EQ(X(0), 10.0);
    EXPECT_DOUBLE_EQ(sim.Time(), 0.0);
}

TEST(SimulationRunnerTest, HistoryRecording) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent(std::make_unique<IcComponent<double>>("Test", 0.0))
                   .BuildAndInitialize();

    SimulationRunner<double> runner(sim);
    runner.EnableHistory(true);

    // Run for 5 steps: 0.0, 0.1, 0.2, 0.3, 0.4, 0.5
    runner.Run(0.1, 0.5); // approx 5 steps

    const auto &history = runner.GetHistory();
    EXPECT_GE(history.size(), 5);

    // Check first point
    EXPECT_DOUBLE_EQ(history.front().first, 0.0);
    EXPECT_TRUE(history.front().second.count("Test.state"));

    // Check values increase
    double prev_val = history.front().second.at("Test.state");
    for (size_t i = 1; i < history.size(); ++i) {
        double val = history[i].second.at("Test.state");
        EXPECT_GT(val, prev_val);
        prev_val = val;
    }
}

} // namespace
} // namespace icarus
