/**
 * @file test_integrator.cpp
 * @brief Tests for Phase 2.2: Integrator Interface
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

#include "testing/StatefulComponent.hpp"

#include <cmath>

using namespace icarus;

// =============================================================================
// Analytical Test Cases
// =============================================================================

// Exponential decay: dx/dt = -x, x(0) = 1 => x(t) = e^(-t)
template <typename Scalar>
JanusVector<Scalar> exponential_decay(Scalar /*t*/, const JanusVector<Scalar> &x) {
    return -x;
}

// Harmonic oscillator: x'' + ω²x = 0
// State: [x, v], dx/dt = v, dv/dt = -ω²x
template <typename Scalar>
JanusVector<Scalar> harmonic_oscillator(Scalar /*t*/, const JanusVector<Scalar> &x) {
    const Scalar omega_sq = Scalar{4.0}; // ω = 2
    JanusVector<Scalar> dx(2);
    dx[0] = x[1];
    dx[1] = -omega_sq * x[0];
    return dx;
}

// Free fall: y'' = -g
// State: [y, v], dy/dt = v, dv/dt = -g
template <typename Scalar>
JanusVector<Scalar> free_fall(Scalar /*t*/, const JanusVector<Scalar> &x) {
    const Scalar g = Scalar{9.81};
    JanusVector<Scalar> dx(2);
    dx[0] = x[1]; // dy/dt = v
    dx[1] = -g;   // dv/dt = -g
    return dx;
}

// =============================================================================
// RK4 Tests
// =============================================================================

TEST(RK4Integrator, ExponentialDecay) {
    RK4Integrator<double> rk4;
    JanusVector<double> x(1);
    x[0] = 1.0;

    double t = 0.0;
    double dt = 0.01;

    // Integrate to t = 1.0
    for (int i = 0; i < 100; ++i) {
        x = rk4.Step(exponential_decay<double>, x, t, dt);
        t += dt;
    }

    // x(1) = e^(-1) ≈ 0.3679
    EXPECT_NEAR(x[0], std::exp(-1.0), 1e-6);
}

TEST(RK4Integrator, HarmonicOscillator) {
    RK4Integrator<double> rk4;
    JanusVector<double> x(2);
    x[0] = 1.0; // Initial position
    x[1] = 0.0; // Initial velocity

    double t = 0.0;
    double dt = 0.001;
    double omega = 2.0;
    double period = 2.0 * M_PI / omega;

    // Integrate one full period
    int steps = static_cast<int>(period / dt);
    for (int i = 0; i < steps; ++i) {
        x = rk4.Step(harmonic_oscillator<double>, x, t, dt);
        t += dt;
    }

    // Should return to initial state after one period (with small numerical drift)
    EXPECT_NEAR(x[0], 1.0, 1e-3);
    EXPECT_NEAR(x[1], 0.0, 1e-2);
}

TEST(RK4Integrator, FreeFall) {
    RK4Integrator<double> rk4;
    JanusVector<double> x(2);
    double y0 = 100.0; // Initial height
    double v0 = 0.0;   // Initial velocity
    x[0] = y0;
    x[1] = v0;

    double t = 0.0;
    double dt = 0.01;
    double g = 9.81;

    // Integrate to t = 2.0
    for (int i = 0; i < 200; ++i) {
        x = rk4.Step(free_fall<double>, x, t, dt);
        t += dt;
    }

    // Analytical: y(t) = y0 + v0*t - 0.5*g*t²
    double y_analytical = y0 + v0 * t - 0.5 * g * t * t;
    double v_analytical = v0 - g * t;

    EXPECT_NEAR(x[0], y_analytical, 1e-6);
    EXPECT_NEAR(x[1], v_analytical, 1e-6);
}

TEST(RK4Integrator, OrderVerification) {
    RK4Integrator<double> rk4;
    EXPECT_EQ(rk4.Order(), 4);
    EXPECT_EQ(rk4.Name(), "RK4");
    EXPECT_FALSE(rk4.IsAdaptive());
    EXPECT_EQ(rk4.Type(), IntegratorType::RK4);
}

// =============================================================================
// Euler and RK2 Tests
// =============================================================================

TEST(EulerIntegrator, OrderVerification) {
    EulerIntegrator<double> euler;
    EXPECT_EQ(euler.Order(), 1);
    EXPECT_EQ(euler.Name(), "Euler");
    EXPECT_FALSE(euler.IsAdaptive());
    EXPECT_EQ(euler.Type(), IntegratorType::Euler);
}

TEST(RK2Integrator, OrderVerification) {
    RK2Integrator<double> rk2;
    EXPECT_EQ(rk2.Order(), 2);
    EXPECT_EQ(rk2.Name(), "RK2");
    EXPECT_FALSE(rk2.IsAdaptive());
    EXPECT_EQ(rk2.Type(), IntegratorType::RK2);
}

TEST(EulerIntegrator, ExponentialDecay) {
    EulerIntegrator<double> euler;
    JanusVector<double> x(1);
    x[0] = 1.0;

    double t = 0.0;
    double dt = 0.001; // Smaller step for lower order

    // Integrate to t = 1.0
    for (int i = 0; i < 1000; ++i) {
        x = euler.Step(exponential_decay<double>, x, t, dt);
        t += dt;
    }

    // Euler is 1st order, expect less accuracy
    EXPECT_NEAR(x[0], std::exp(-1.0), 1e-2);
}

// =============================================================================
// RK45 Tests
// =============================================================================

TEST(RK45Integrator, ExponentialDecay) {
    RK45Integrator<double> rk45(1e-8, 1e-8);
    JanusVector<double> x(1);
    x[0] = 1.0;

    double t = 0.0;
    double dt = 0.1;

    // Integrate to t = 1.0 using fixed number of steps
    for (int i = 0; i < 10; ++i) {
        x = rk45.Step(exponential_decay<double>, x, t, dt);
        t += dt;
    }

    EXPECT_NEAR(x[0], std::exp(-1.0), 1e-5);
}

TEST(RK45Integrator, AdaptiveStep) {
    RK45Integrator<double> rk45(1e-6, 1e-6);
    JanusVector<double> x(1);
    x[0] = 1.0;

    double t = 0.0;
    double dt = 0.1;

    auto result = rk45.AdaptiveStep(exponential_decay<double>, x, t, dt);

    EXPECT_GT(result.state.size(), 0);
    EXPECT_TRUE(result.accepted); // Should accept with reasonable tolerance
    EXPECT_LT(result.error_estimate, 1e-4);
}

TEST(RK45Integrator, StepSizeSuggestion) {
    RK45Integrator<double> rk45(1e-6, 1e-6);

    // If error < tolerance, suggest larger step
    double dt_new = rk45.SuggestDt(0.01, 1e-8, 1e-6);
    EXPECT_GT(dt_new, 0.01);

    // If error > tolerance, suggest smaller step
    dt_new = rk45.SuggestDt(0.01, 1e-4, 1e-6);
    EXPECT_LT(dt_new, 0.01);
}

TEST(RK45Integrator, Statistics) {
    RK45Integrator<double> rk45(1e-6, 1e-6);
    JanusVector<double> x(1);
    x[0] = 1.0;

    rk45.ResetStatistics();

    // Several steps
    for (int i = 0; i < 10; ++i) {
        rk45.AdaptiveStep(exponential_decay<double>, x, 0.0, 0.1);
    }

    auto stats = rk45.GetStatistics();
    EXPECT_GT(stats.accepted_steps, 0u);
}

TEST(RK45Integrator, OrderVerification) {
    RK45Integrator<double> rk45;
    EXPECT_EQ(rk45.Order(), 5);
    EXPECT_EQ(rk45.Name(), "RK45");
    EXPECT_TRUE(rk45.IsAdaptive());
    EXPECT_EQ(rk45.Type(), IntegratorType::RK45);
}

// =============================================================================
// IntegratorConfig Tests
// =============================================================================

TEST(IntegratorConfig, TypeEnumToString) {
    EXPECT_EQ(to_string(IntegratorType::Euler), "Euler");
    EXPECT_EQ(to_string(IntegratorType::RK2), "RK2");
    EXPECT_EQ(to_string(IntegratorType::RK4), "RK4");
    EXPECT_EQ(to_string(IntegratorType::RK45), "RK45");
}

TEST(IntegratorConfig, ParseTypeFromString) {
    EXPECT_EQ(parse_integrator_type("Euler"), IntegratorType::Euler);
    EXPECT_EQ(parse_integrator_type("euler"), IntegratorType::Euler);
    EXPECT_EQ(parse_integrator_type("RK4"), IntegratorType::RK4);
    EXPECT_EQ(parse_integrator_type("rk45"), IntegratorType::RK45);

    EXPECT_THROW(parse_integrator_type("invalid"), std::invalid_argument);
}

TEST(IntegratorConfig, DefaultConfig) {
    auto config = IntegratorConfig<double>::RK4Default();
    EXPECT_EQ(config.type, IntegratorType::RK4);
}

TEST(IntegratorConfig, AdaptiveConfig) {
    auto config = IntegratorConfig<double>::RK45Adaptive(1e-9, 1e-8);
    EXPECT_EQ(config.type, IntegratorType::RK45);
    EXPECT_DOUBLE_EQ(config.abs_tol, 1e-9);
    EXPECT_DOUBLE_EQ(config.rel_tol, 1e-8);
}

// =============================================================================
// IntegratorFactory Tests
// =============================================================================

TEST(IntegratorFactory, CreateFromType) {
    auto euler = IntegratorFactory<double>::Create(
        IntegratorConfig<double>::ForMethod(IntegratorType::Euler));
    EXPECT_EQ(euler->Name(), "Euler");
    EXPECT_EQ(euler->Order(), 1);

    auto rk4 =
        IntegratorFactory<double>::Create(IntegratorConfig<double>::ForMethod(IntegratorType::RK4));
    EXPECT_EQ(rk4->Name(), "RK4");
    EXPECT_EQ(rk4->Order(), 4);
}

TEST(IntegratorFactory, CreateFromString) {
    auto integrator = IntegratorFactory<double>::Create("rk45");
    EXPECT_EQ(integrator->Name(), "RK45");
    EXPECT_TRUE(integrator->IsAdaptive());
}

TEST(IntegratorFactory, CreateDefault) {
    auto integrator = IntegratorFactory<double>::CreateDefault();
    EXPECT_EQ(integrator->Name(), "RK4");
}

TEST(IntegratorFactory, RK45ConfigApplied) {
    IntegratorConfig<double> config;
    config.type = IntegratorType::RK45;
    config.abs_tol = 1e-10;
    config.rel_tol = 1e-9;
    config.min_dt = 1e-15;
    config.max_dt = 0.5;

    auto integrator = IntegratorFactory<double>::Create(config);
    auto *rk45 = dynamic_cast<RK45Integrator<double> *>(integrator.get());

    ASSERT_NE(rk45, nullptr);
    EXPECT_DOUBLE_EQ(rk45->GetAbsTol(), 1e-10);
    EXPECT_DOUBLE_EQ(rk45->GetRelTol(), 1e-9);
    EXPECT_DOUBLE_EQ(rk45->GetMinDt(), 1e-15);
    EXPECT_DOUBLE_EQ(rk45->GetMaxDt(), 0.5);
}

// =============================================================================
// Simulator Integration Tests
// =============================================================================

TEST(SimulatorIntegrator, DefaultIsRK4) {
    Simulator<double> sim;

    // Integrator should be valid from construction
    EXPECT_NE(sim.GetIntegrator(), nullptr);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK4);
    EXPECT_EQ(sim.GetIntegrator()->Name(), "RK4");
}

TEST(SimulatorIntegrator, SetByEnum) {
    Simulator<double> sim;

    sim.SetIntegrator(IntegratorType::RK2);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK2);

    sim.SetIntegrator(IntegratorType::RK4);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK4);
}

TEST(SimulatorIntegrator, SetByString) {
    Simulator<double> sim;

    sim.SetIntegrator("euler");
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::Euler);

    sim.SetIntegrator("RK45");
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK45);
}

TEST(SimulatorIntegrator, SetByConfig) {
    Simulator<double> sim;

    auto config = IntegratorConfig<double>::RK45Adaptive(1e-10, 1e-10);
    sim.SetIntegrator(config);

    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK45);
    EXPECT_DOUBLE_EQ(sim.GetIntegratorConfig().abs_tol, 1e-10);
}

TEST(SimulatorIntegrator, SetByUniquePtr) {
    Simulator<double> sim;

    sim.SetIntegrator(std::make_unique<EulerIntegrator<double>>());
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::Euler);
    EXPECT_EQ(sim.GetIntegrator()->Name(), "Euler");
}

TEST(SimulatorIntegrator, FreeFallWithRK4) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("TestComponent"));
    sim.SetIntegrator(std::make_unique<RK4Integrator<double>>());

    sim.Provision();
    sim.Stage();

    // Verify initial state
    EXPECT_EQ(sim.GetTotalStateSize(), 3u);

    // Integrate for 1 second
    double dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        sim.Step(dt);
    }

    // Check final time
    double t = sim.Time();
    EXPECT_NEAR(t, 1.0, 1e-10);
}

TEST(SimulatorIntegrator, RuntimeSwitching) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("TestComponent"));
    sim.Provision();
    sim.Stage();

    // Start with Euler
    sim.SetIntegrator(IntegratorType::Euler);
    sim.Step(0.01);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::Euler);

    // Switch to RK4 mid-simulation
    sim.SetIntegrator(IntegratorType::RK4);
    sim.Step(0.01);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK4);

    // Switch to RK45
    sim.SetIntegrator(IntegratorType::RK45);
    sim.Step(0.01);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK45);
}

TEST(SimulatorIntegrator, AllMethodsProduceResults) {
    // Verify all integrators work end-to-end
    std::vector<IntegratorType> methods = {IntegratorType::Euler, IntegratorType::RK2,
                                           IntegratorType::RK4, IntegratorType::RK45};

    for (auto method : methods) {
        Simulator<double> sim;
        sim.AddComponent(std::make_unique<StatefulComponent<double>>("TestComponent"));
        sim.SetIntegrator(method);
        sim.Provision();
        sim.Stage();

        // Run 10 steps
        for (int i = 0; i < 10; ++i) {
            sim.Step(0.01);
        }

        // Should have advanced time
        EXPECT_GT(sim.Time(), 0.0) << "Failed for " << to_string(method);
    }
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(IntegratorSymbolic, RK4Compiles) {
    using MX = casadi::MX;

    RK4Integrator<MX> rk4;
    auto x = janus::sym_vec("x", 2);
    auto t = janus::sym("t");
    auto dt = janus::sym("dt");

    // Use symbolic omega for proper tracing
    auto omega_sq = MX(4.0);

    auto x_next = rk4.Step(
        [&omega_sq](MX /*t*/, const JanusVector<MX> &x) {
            JanusVector<MX> dx(2);
            dx[0] = x[1];
            dx[1] = -omega_sq * x[0];
            return dx;
        },
        x, t, dt);

    // Should produce valid symbolic result
    EXPECT_EQ(x_next.size(), 2);
}

TEST(IntegratorSymbolic, EulerCompiles) {
    using MX = casadi::MX;

    EulerIntegrator<MX> euler;
    auto x = janus::sym_vec("x", 1);
    auto t = janus::sym("t");
    auto dt = janus::sym("dt");

    auto x_next = euler.Step([](MX /*t*/, const JanusVector<MX> &x) { return -x; }, x, t, dt);

    casadi::Function step_fn("step", {janus::to_mx(x), t, dt}, {janus::to_mx(x_next)});

    std::vector<casadi::DM> args = {casadi::DM(1.0), casadi::DM(0.0), casadi::DM(0.01)};
    auto res = step_fn(args);

    EXPECT_EQ(res[0].size1(), 1);
}

TEST(IntegratorSymbolic, SimulatorWithMX) {
    using MX = casadi::MX;

    Simulator<MX> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<MX>>("TestComponent"));
    sim.SetIntegrator(std::make_unique<RK4Integrator<MX>>());

    sim.Provision();
    sim.Stage();

    // Should compile and run symbolic step
    EXPECT_NO_THROW(sim.Step(MX{0.01}));
}
