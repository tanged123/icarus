/**
 * @file test_managers.cpp
 * @brief Unit tests for StateManager and IntegrationManager
 *
 * Part of Phase 4.0.7: Internal Managers
 */

#include <gtest/gtest.h>
#include <icarus/sim/IntegrationManager.hpp>
#include <icarus/sim/StateManager.hpp>

namespace icarus {
namespace {

// =============================================================================
// StateManager Tests
// =============================================================================

TEST(StateManagerTest, DefaultConstruction) {
    StateManager<double> sm;
    EXPECT_EQ(sm.TotalSize(), 0u);
    EXPECT_TRUE(sm.GetBindings().empty());
}

TEST(StateManagerTest, ZeroDerivatives) {
    StateManager<double> sm;
    // Even with no allocation, ZeroDerivatives should not crash
    sm.ZeroDerivatives();
}

TEST(StateManagerTest, GetSetState) {
    StateManager<double> sm;
    // Without allocation, state is empty
    auto state = sm.GetState();
    EXPECT_EQ(state.size(), 0);
}

// =============================================================================
// IntegrationManager Tests
// =============================================================================

TEST(IntegrationManagerTest, DefaultConstruction) {
    IntegrationManager<double> im;
    EXPECT_FALSE(im.IsConfigured());
}

TEST(IntegrationManagerTest, ConfigureDefault) {
    IntegrationManager<double> im;
    im.ConfigureDefault();

    EXPECT_TRUE(im.IsConfigured());
    EXPECT_EQ(im.Type(), IntegratorType::RK4);
    EXPECT_EQ(im.Name(), "RK4");
    EXPECT_FALSE(im.IsAdaptive());
}

TEST(IntegrationManagerTest, ConfigureFromType) {
    IntegrationManager<double> im;
    im.Configure(IntegratorType::RK45);

    EXPECT_TRUE(im.IsConfigured());
    EXPECT_EQ(im.Type(), IntegratorType::RK45);
    EXPECT_TRUE(im.IsAdaptive());
}

TEST(IntegrationManagerTest, ConfigureFromString) {
    IntegrationManager<double> im;
    im.Configure("RK2");

    EXPECT_TRUE(im.IsConfigured());
    EXPECT_EQ(im.Type(), IntegratorType::RK2);
}

TEST(IntegrationManagerTest, ConfigureFromConfig) {
    IntegrationManager<double> im;
    IntegratorConfig<double> cfg;
    cfg.type = IntegratorType::Euler;
    im.Configure(cfg);

    EXPECT_TRUE(im.IsConfigured());
    EXPECT_EQ(im.Type(), IntegratorType::Euler);
}

TEST(IntegrationManagerTest, StepWithoutConfigureThrows) {
    IntegrationManager<double> im;
    JanusVector<double> X(3);
    X.setZero();

    auto deriv_func = [](double /*t*/, const JanusVector<double> &x) { return x; };

    EXPECT_THROW((void)im.Step(deriv_func, X, 0.0, 0.01), std::runtime_error);
}

TEST(IntegrationManagerTest, FixedStep) {
    IntegrationManager<double> im;
    im.Configure(IntegratorType::Euler);

    JanusVector<double> X(1);
    X(0) = 1.0;

    // dx/dt = x  =>  x(t) = e^t
    // Euler: x_new = x + dt * x = x * (1 + dt)
    auto deriv_func = [](double /*t*/, const JanusVector<double> &x) { return x; };

    auto X_new = im.Step(deriv_func, X, 0.0, 0.1);
    EXPECT_NEAR(X_new(0), 1.1, 1e-10);
}

TEST(IntegrationManagerTest, AdaptiveStepOnNonAdaptiveThrows) {
    IntegrationManager<double> im;
    im.Configure(IntegratorType::RK4); // Not adaptive

    JanusVector<double> X(1);
    X(0) = 1.0;

    auto deriv_func = [](double /*t*/, const JanusVector<double> &x) { return x; };

    EXPECT_THROW((void)im.AdaptiveStep(deriv_func, X, 0.0, 0.1), std::runtime_error);
}

TEST(IntegrationManagerTest, AdaptiveStepOnRK45Works) {
    IntegrationManager<double> im;
    im.Configure(IntegratorType::RK45);

    JanusVector<double> X(1);
    X(0) = 1.0;

    auto deriv_func = [](double /*t*/, const JanusVector<double> &x) { return x; };

    auto result = im.AdaptiveStep(deriv_func, X, 0.0, 0.1);
    EXPECT_TRUE(result.accepted);
    EXPECT_GT(result.state(0), 1.0); // Should have increased
}

} // namespace
} // namespace icarus
