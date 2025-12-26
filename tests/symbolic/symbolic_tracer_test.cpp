/**
 * @file symbolic_tracer_test.cpp
 * @brief Tests for SymbolicTracer (Phase 3.3)
 *
 * Tests graph extraction from symbolic simulator.
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// Include test components
#include <testing/StatefulComponent.hpp>

namespace icarus {
namespace {

using Scalar = SymbolicScalar;

// =============================================================================
// Basic Tracer Construction
// =============================================================================

TEST(SymbolicTracer, Construction) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 3));
    sim.Initialize();

    // Should not throw
    symbolic::SymbolicTracer tracer(sim);
    EXPECT_EQ(tracer.GetStateSize(), 3);
}

TEST(SymbolicTracer, RequiresStagedSimulator) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 3));
    // NOT initialized

    EXPECT_THROW((symbolic::SymbolicTracer(sim)), LifecycleError);
}

// =============================================================================
// Dynamics Tracing
// =============================================================================

TEST(SymbolicTracer, TraceDynamicsReturnsFunction) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 2));
    sim.Initialize();

    symbolic::SymbolicTracer tracer(sim);
    auto dynamics = tracer.TraceDynamics();

    // Access underlying CasADi function for metadata
    const auto &casadi_fn = dynamics.casadi_function();
    EXPECT_EQ(casadi_fn.name(), "dynamics");
    EXPECT_EQ(casadi_fn.n_in(), 2);  // t, x
    EXPECT_EQ(casadi_fn.n_out(), 1); // xdot
}

TEST(SymbolicTracer, TraceDynamicsCorrectDimensions) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("A", 3));
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("B", 4));
    sim.Initialize();

    symbolic::SymbolicTracer tracer(sim);
    auto dynamics = tracer.TraceDynamics();

    // Evaluate at a point to check output dimension
    double t0 = 0.0;
    janus::NumericVector x0 = janus::NumericVector::Zero(7);
    x0 << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

    auto result = dynamics(t0, x0);
    ASSERT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].rows(), 7);
    EXPECT_EQ(result[0].cols(), 1);
}

TEST(SymbolicTracer, TraceDynamicsMatchesNumericEvaluation) {
    // StatefulComponent has dynamics: xdot = -x
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 3));
    sim.Initialize();

    symbolic::SymbolicTracer tracer(sim);
    auto dynamics = tracer.TraceDynamics();

    // Evaluate dynamics at specific point
    double t0 = 0.0;
    janus::NumericVector x0(3);
    x0 << 1.0, 2.0, 3.0;

    auto result = dynamics(t0, x0);
    auto xdot = result[0];

    // StatefulComponent sets xdot = -x
    EXPECT_NEAR(xdot(0), -1.0, 1e-10);
    EXPECT_NEAR(xdot(1), -2.0, 1e-10);
    EXPECT_NEAR(xdot(2), -3.0, 1e-10);
}

// =============================================================================
// Metadata Tests
// =============================================================================

TEST(SymbolicTracer, GetStateSize) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("A", 5));
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("B", 3));
    sim.Initialize();

    symbolic::SymbolicTracer tracer(sim);
    EXPECT_EQ(tracer.GetStateSize(), 8);
}

TEST(SymbolicTracer, GetStateNames) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 2));
    sim.Initialize();

    symbolic::SymbolicTracer tracer(sim);
    auto names = tracer.GetStateNames();

    EXPECT_EQ(names.size(), 2);
}

// =============================================================================
// Configuration Tests
// =============================================================================

TEST(SymbolicTracer, ConfigureCustomFunctionName) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 2));
    sim.Initialize();

    symbolic::TracerConfig config;
    config.function_name = "my_dynamics";

    symbolic::SymbolicTracer tracer(sim);
    tracer.Configure(config);

    auto dynamics = tracer.TraceDynamics();
    EXPECT_EQ(dynamics.casadi_function().name(), "my_dynamics");
}

TEST(SymbolicTracer, ConfigureWithoutTime) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 2));
    sim.Initialize();

    symbolic::TracerConfig config;
    config.include_time = false;

    symbolic::SymbolicTracer tracer(sim);
    tracer.Configure(config);

    auto dynamics = tracer.TraceDynamics();
    EXPECT_EQ(dynamics.casadi_function().n_in(), 1); // Just x (no t)
}

// =============================================================================
// Convenience Function Tests
// =============================================================================

TEST(SymbolicTracer, ExtractDynamicsConvenienceFunction) {
    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 4));
    sim.Initialize();

    // One-liner extraction
    auto dynamics = symbolic::ExtractDynamics(sim);

    const auto &casadi_fn = dynamics.casadi_function();
    EXPECT_EQ(casadi_fn.name(), "dynamics");
    EXPECT_EQ(casadi_fn.n_in(), 2);
    EXPECT_EQ(casadi_fn.n_out(), 1);
}

// =============================================================================
// Jacobian Extraction Test
// =============================================================================

TEST(SymbolicTracer, JacobianExtraction) {
    using namespace janus;

    Simulator<Scalar> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<Scalar>>("Test", 2));
    sim.Initialize();

    auto dynamics = symbolic::ExtractDynamics(sim);

    // Create PURE symbolic inputs for Jacobian computation
    // Must use sym(name, rows, cols) for pure MX, not sym_vec() which returns Eigen<MX>
    auto t_sym = sym("t");
    auto x_sym = sym("x", 2, 1); // Pure MX column vector

    // Evaluate dynamics symbolically - returns vector of SymbolicMatrix
    auto xdot_result = dynamics.eval(t_sym, x_sym);

    // Compute Jacobian: d(xdot)/dx
    // janus::jacobian returns a SymbolicScalar (MX) directly
    auto J_sym = jacobian({as_mx(xdot_result)}, {x_sym});

    // Wrap Jacobian as function for numeric evaluation
    janus::Function jacobian_fn("jacobian", {t_sym, x_sym}, {J_sym});

    // Evaluate at a point
    double t0 = 0.0;
    NumericVector x0(2);
    x0 << 1.0, 2.0;

    auto J_result = jacobian_fn(t0, x0);
    auto J = J_result[0];

    // For StatefulComponent with xdot = -x, Jacobian should be -I
    EXPECT_EQ(J.rows(), 2);
    EXPECT_EQ(J.cols(), 2);
    EXPECT_NEAR(J(0, 0), -1.0, 1e-10);
    EXPECT_NEAR(J(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(J(1, 0), 0.0, 1e-10);
    EXPECT_NEAR(J(1, 1), -1.0, 1e-10);
}

} // namespace
} // namespace icarus
