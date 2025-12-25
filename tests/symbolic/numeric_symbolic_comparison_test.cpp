/**
 * @file numeric_symbolic_comparison_test.cpp
 * @brief Tests verifying numeric and symbolic modes produce identical results
 *
 * Part of Phase 3.4: Symbolic Test Suite
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// Include test components
#include <testing/StatefulComponent.hpp>

namespace icarus {
namespace {

constexpr double TOLERANCE = 1e-10;

// =============================================================================
// Basic Numeric/Symbolic Equivalence
// =============================================================================

TEST(NumericSymbolicComparison, PointMassDynamicsMatch) {
    // Build numeric simulator
    Simulator<double> num_sim;
    num_sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 3));
    num_sim.Initialize();

    // Build symbolic simulator
    Simulator<SymbolicScalar> sym_sim;
    sym_sim.AddComponent(std::make_unique<StatefulComponent<SymbolicScalar>>("Test", 3));
    sym_sim.Initialize();

    // Extract dynamics from symbolic
    auto dynamics = symbolic::ExtractDynamics(sym_sim);

    // Test at several points
    std::vector<std::array<double, 3>> test_points = {
        {1.0, 2.0, 3.0}, {-1.0, 0.5, 2.5}, {100.0, -50.0, 25.0}};

    for (const auto &point : test_points) {
        // Set state in numeric sim
        janus::NumericVector x_num(3);
        x_num << point[0], point[1], point[2];
        num_sim.SetState(x_num);

        // Compute numeric derivatives
        num_sim.ComputeDerivatives(0.0);
        auto xdot_num = num_sim.GetDerivatives();

        // Evaluate symbolic dynamics
        auto sym_result = dynamics(0.0, x_num);
        auto xdot_sym = sym_result[0];

        // Compare
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(xdot_num(i), xdot_sym(i), TOLERANCE)
                << "Mismatch at index " << i << " for point [" << point[0] << ", " << point[1]
                << ", " << point[2] << "]";
        }
    }
}

TEST(NumericSymbolicComparison, MultiStepAccumulation) {
    // Run both numeric and symbolic (TraceStep) for multiple steps
    // and verify final states match. Both now use RK4.

    // Numeric
    Simulator<double> num_sim;
    num_sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 2));
    num_sim.Initialize();
    janus::NumericVector x0(2);
    x0 << 10.0, 5.0;
    num_sim.SetState(x0);

    const double dt = 0.01;
    const int n_steps = 100;

    for (int i = 0; i < n_steps; ++i) {
        num_sim.Step(dt);
    }
    auto final_num = num_sim.GetState();

    // Symbolic (extract and evaluate step function - now uses RK4)
    Simulator<SymbolicScalar> sym_sim;
    sym_sim.AddComponent(std::make_unique<StatefulComponent<SymbolicScalar>>("Test", 2));
    sym_sim.Initialize();

    auto step_fn = symbolic::SymbolicTracer(sym_sim).TraceStep(dt);

    // Iterate using symbolic function
    janus::NumericVector x = x0;
    double t = 0.0;
    for (int i = 0; i < n_steps; ++i) {
        auto result = step_fn(t, x);
        x = result[0];
        t += dt;
    }

    // Compare - both should match since both use RK4
    for (int i = 0; i < 2; ++i) {
        EXPECT_NEAR(final_num(i), x(i), 1e-10) << "Multi-step mismatch at index " << i;
    }
}

// =============================================================================
// Jacobian Verification
// =============================================================================

TEST(NumericSymbolicComparison, JacobianMatchesFiniteDifference) {
    // For StatefulComponent: xdot = -x
    // The Jacobian should be -I (negative identity)

    Simulator<SymbolicScalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<SymbolicScalar>>("Test", 3));
    sim.Initialize();

    auto jacobian_fn = symbolic::ExtractStateJacobian(sim);

    // Evaluate at a point
    janus::NumericVector x0(3);
    x0 << 1.0, 2.0, 3.0;

    auto J_result = jacobian_fn(0.0, x0);
    auto J = J_result[0];

    // Verify J = -I
    EXPECT_EQ(J.rows(), 3);
    EXPECT_EQ(J.cols(), 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = (i == j) ? -1.0 : 0.0;
            EXPECT_NEAR(J(i, j), expected, TOLERANCE)
                << "Jacobian mismatch at (" << i << ", " << j << ")";
        }
    }
}

TEST(NumericSymbolicComparison, FiniteDifferenceVerification) {
    // Verify Jacobian using finite differences

    Simulator<double> num_sim;
    num_sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 2));
    num_sim.Initialize();

    Simulator<SymbolicScalar> sym_sim;
    sym_sim.AddComponent(std::make_unique<StatefulComponent<SymbolicScalar>>("Test", 2));
    sym_sim.Initialize();

    auto jacobian_fn = symbolic::ExtractStateJacobian(sym_sim);

    janus::NumericVector x0(2);
    x0 << 1.0, 2.0;

    // Compute symbolic Jacobian
    auto J_sym_result = jacobian_fn(0.0, x0);
    auto J_sym = J_sym_result[0];

    // Compute finite difference Jacobian
    const double eps = 1e-7;
    janus::NumericMatrix J_fd(2, 2);

    for (int j = 0; j < 2; ++j) {
        janus::NumericVector x_plus = x0;
        janus::NumericVector x_minus = x0;
        x_plus(j) += eps;
        x_minus(j) -= eps;

        num_sim.SetState(x_plus);
        num_sim.ComputeDerivatives(0.0);
        auto f_plus = num_sim.GetDerivatives();

        num_sim.SetState(x_minus);
        num_sim.ComputeDerivatives(0.0);
        auto f_minus = num_sim.GetDerivatives();

        for (int i = 0; i < 2; ++i) {
            J_fd(i, j) = (f_plus(i) - f_minus(i)) / (2.0 * eps);
        }
    }

    // Compare
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            EXPECT_NEAR(J_sym(i, j), J_fd(i, j), 1e-5)
                << "FD verification mismatch at (" << i << ", " << j << ")";
        }
    }
}

// =============================================================================
// TraceStep Verification
// =============================================================================

TEST(NumericSymbolicComparison, TraceStepMatchesIntegrator) {
    // Verify TraceStep produces same result as numeric RK4 step

    const double dt = 0.1;

    // Numeric simulator with RK4
    Simulator<double> num_sim;
    num_sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 2));
    num_sim.Initialize();

    janus::NumericVector x0(2);
    x0 << 10.0, 5.0;
    num_sim.SetState(x0);
    num_sim.Step(dt);
    auto x_numeric = num_sim.GetState();

    // Symbolic TraceStep (now uses RK4)
    Simulator<SymbolicScalar> sym_sim;
    sym_sim.AddComponent(std::make_unique<StatefulComponent<SymbolicScalar>>("Test", 2));
    sym_sim.Initialize();

    auto step_fn = symbolic::SymbolicTracer(sym_sim).TraceStep(dt);

    auto step_result = step_fn(0.0, x0);
    auto x_symbolic = step_result[0];

    // Both use RK4, so should match exactly
    EXPECT_NEAR(x_symbolic(0), x_numeric(0), TOLERANCE);
    EXPECT_NEAR(x_symbolic(1), x_numeric(1), TOLERANCE);
}

// =============================================================================
// Input/Output Callback Tests
// =============================================================================

TEST(SimulatorCallbacks, RegisterInputSourceBasic) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 2));
    sim.Initialize();

    EXPECT_EQ(sim.NumInputSources(), 0);

    sim.RegisterInputSource("some.signal", [](const std::string &) { return 42.0; });

    EXPECT_EQ(sim.NumInputSources(), 1);
}

TEST(SimulatorCallbacks, RegisterOutputObserverBasic) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 2));
    sim.Initialize();

    EXPECT_EQ(sim.NumOutputObservers(), 0);

    sim.RegisterOutputObserver("some.signal", [](const std::string &, const double &) {
        // Observer callback
    });

    EXPECT_EQ(sim.NumOutputObservers(), 1);
}

TEST(SimulatorCallbacks, UnregisterRemovesCallback) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 2));
    sim.Initialize();

    sim.RegisterInputSource("a", [](const std::string &) { return 0.0; });
    sim.RegisterInputSource("b", [](const std::string &) { return 0.0; });
    EXPECT_EQ(sim.NumInputSources(), 2);

    sim.UnregisterInputSource("a");
    EXPECT_EQ(sim.NumInputSources(), 1);

    sim.UnregisterInputSource("b");
    EXPECT_EQ(sim.NumInputSources(), 0);
}

TEST(SimulatorCallbacks, MultipleCallbacksTracking) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 2));
    sim.Initialize();

    EXPECT_EQ(sim.NumInputSources(), 0);
    EXPECT_EQ(sim.NumOutputObservers(), 0);

    sim.RegisterInputSource("input1", [](const std::string &) { return 0.0; });
    sim.RegisterInputSource("input2", [](const std::string &) { return 0.0; });
    sim.RegisterInputSource("input3", [](const std::string &) { return 0.0; });
    EXPECT_EQ(sim.NumInputSources(), 3);

    sim.RegisterOutputObserver("output1", [](const std::string &, const double &) {});
    sim.RegisterOutputObserver("output2", [](const std::string &, const double &) {});
    EXPECT_EQ(sim.NumOutputObservers(), 2);

    // Unregister one of each
    sim.UnregisterInputSource("input2");
    sim.UnregisterOutputObserver("output1");

    EXPECT_EQ(sim.NumInputSources(), 2);
    EXPECT_EQ(sim.NumOutputObservers(), 1);
}

} // namespace
} // namespace icarus
