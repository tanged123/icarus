/**
 * @file test_point_mass.cpp
 * @brief Unit tests for PointMass3DOF and PointMassGravity components
 *
 * Part of Phase 2.3: First Real Component
 * Updated for Phase 6: Unified signal model (states as signals)
 */

#include <gtest/gtest.h>

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/sim/Simulator.hpp>

#include <vulcan/core/Constants.hpp>

#include <cmath>

using namespace icarus;
using namespace icarus::components;

// =============================================================================
// PointMass3DOF Unit Tests
// =============================================================================

TEST(PointMass3DOF, Identity) {
    PointMass3DOF<double> pm("TestMass", "Vehicle");
    EXPECT_EQ(pm.Name(), "TestMass");
    EXPECT_EQ(pm.Entity(), "Vehicle");
    EXPECT_EQ(pm.TypeName(), "PointMass3DOF");
}

TEST(PointMass3DOF, MassProperty) {
    PointMass3DOF<double> pm;
    pm.SetMass(42.0);
    EXPECT_DOUBLE_EQ(pm.GetMass(), 42.0);

    pm.SetMass(100.0);
    EXPECT_DOUBLE_EQ(pm.GetMass(), 100.0);
}

TEST(PointMass3DOF, InitialConditions) {
    PointMass3DOF<double> pm;
    pm.SetInitialPosition(1.0, 2.0, 3.0);
    pm.SetInitialVelocity(4.0, 5.0, 6.0);

    // In Phase 6, states are registered as signals and accessible via accessors
    auto pos = pm.GetPosition();
    EXPECT_DOUBLE_EQ(pos(0), 1.0);
    EXPECT_DOUBLE_EQ(pos(1), 2.0);
    EXPECT_DOUBLE_EQ(pos(2), 3.0);

    auto vel = pm.GetVelocity();
    EXPECT_DOUBLE_EQ(vel(0), 4.0);
    EXPECT_DOUBLE_EQ(vel(1), 5.0);
    EXPECT_DOUBLE_EQ(vel(2), 6.0);
}

TEST(PointMass3DOF, InitialConditionsVec3) {
    PointMass3DOF<double> pm;
    pm.SetInitialPosition(Vec3<double>{1.0, 2.0, 3.0});
    pm.SetInitialVelocity(Vec3<double>{4.0, 5.0, 6.0});

    auto pos = pm.GetPosition();
    EXPECT_DOUBLE_EQ(pos(0), 1.0);

    auto vel = pm.GetVelocity();
    EXPECT_DOUBLE_EQ(vel(0), 4.0);
}

// =============================================================================
// PointMassGravity Unit Tests
// =============================================================================

TEST(PointMassGravity, Identity) {
    PointMassGravity<double> grav("TestGravity", "Environment");
    EXPECT_EQ(grav.Name(), "TestGravity");
    EXPECT_EQ(grav.Entity(), "Environment");
    EXPECT_EQ(grav.TypeName(), "PointMassGravity");
}

TEST(PointMassGravity, ModelSwitch) {
    PointMassGravity<double> grav;

    grav.SetModel(PointMassGravity<double>::Model::Constant);
    EXPECT_EQ(grav.GetModel(), PointMassGravity<double>::Model::Constant);

    grav.SetModel(PointMassGravity<double>::Model::PointMass);
    EXPECT_EQ(grav.GetModel(), PointMassGravity<double>::Model::PointMass);
}

TEST(PointMassGravity, GravitationalParameter) {
    PointMassGravity<double> grav;

    // Default is Earth's mu
    EXPECT_DOUBLE_EQ(grav.GetGravitationalParameter(), vulcan::constants::earth::mu);

    // Can override for other bodies
    double moon_mu = 4.9028e12;
    grav.SetGravitationalParameter(moon_mu);
    EXPECT_DOUBLE_EQ(grav.GetGravitationalParameter(), moon_mu);
}

// =============================================================================
// Integration Tests (Simulator with both components)
// Updated for Phase 4.0.7 API - uses AddRoutes() instead of SetWiring()
// =============================================================================

TEST(PointMassIntegration, FreeFallConstantGravity) {
    // Create simulator
    Simulator sim;

    // Create components
    auto pm = std::make_unique<PointMass3DOF<double>>("PointMass3DOF");
    auto grav = std::make_unique<PointMassGravity<double>>("Gravity");

    // Set mass and constant gravity model for analytical validation
    pm->SetMass(1.0);
    grav->SetModel(PointMassGravity<double>::Model::Constant);

    // Initial conditions: start at z=100m, at rest
    double z0 = 100.0;
    double v0 = 0.0;
    pm->SetInitialPosition(0.0, 0.0, z0);
    pm->SetInitialVelocity(0.0, 0.0, v0);

    // Add components - ORDER MATTERS!
    // Gravity must run first to compute force from current position
    // before PointMass3DOF reads it
    sim.AddComponent(std::move(grav));
    sim.AddComponent(std::move(pm));

    // Wire components using AddRoutes (replaces SetWiring)
    std::vector<signal::SignalRoute> routes = {
        // Gravity reads position from PointMass3DOF
        {"Gravity.position.x", "PointMass3DOF.position.x"},
        {"Gravity.position.y", "PointMass3DOF.position.y"},
        {"Gravity.position.z", "PointMass3DOF.position.z"},
        {"Gravity.mass", "PointMass3DOF.mass"},
        // PointMass3DOF reads force from Gravity
        {"PointMass3DOF.force.x", "Gravity.force.x"},
        {"PointMass3DOF.force.y", "Gravity.force.y"},
        {"PointMass3DOF.force.z", "Gravity.force.z"},
    };
    sim.AddRoutes(routes);

    // Stage runs Provision internally, then stages components
    sim.Stage();

    // Simulate for 2 seconds
    double dt = 0.01;
    double g = vulcan::constants::physics::g0;

    for (int i = 0; i < 200; ++i) {
        sim.Step(dt);
    }

    double t = sim.Time();

    // Get final state
    auto state = sim.GetState();
    double z = state[2];  // Position Z
    double vz = state[5]; // Velocity Z

    // Analytical solution: z(t) = z₀ + v₀t - ½gt²
    double z_exact = z0 + v0 * t - 0.5 * g * t * t;
    double v_exact = v0 - g * t;

    // RK4 should integrate constant acceleration (quadratic) exactly
    // Allow for floating point accumulation error over 200 steps
    EXPECT_NEAR(z, z_exact, 1e-10);
    EXPECT_NEAR(vz, v_exact, 1e-10);
}

TEST(PointMassIntegration, OrbitalEnergyConservation) {
    // =========================================================================
    // Orbital Mechanics Test (ECI Frame)
    // =========================================================================
    // Validate point-mass gravity by checking energy conservation
    // over a partial orbit. For a closed orbit, mechanical energy
    // E = KE + PE = ½mv² - μm/r should remain constant.

    Simulator sim;

    // Circular orbit at 400 km altitude (ISS-like)
    double altitude = 400e3; // m
    double r0 = vulcan::constants::earth::R_eq + altitude;
    double mu = vulcan::constants::earth::mu;

    // Circular orbit velocity: v = sqrt(μ/r)
    double v_circular = std::sqrt(mu / r0);

    // Create components
    double mass = 1000.0; // kg (typical small satellite)
    auto pm = std::make_unique<PointMass3DOF<double>>("PointMass3DOF");
    pm->SetMass(mass);
    auto grav = std::make_unique<PointMassGravity<double>>("Gravity");

    // Use point-mass gravity model (central force)
    grav->SetModel(PointMassGravity<double>::Model::PointMass);

    // Initial conditions: satellite at +X, velocity in +Y (circular orbit in XY plane)
    pm->SetInitialPosition(r0, 0.0, 0.0);
    pm->SetInitialVelocity(0.0, v_circular, 0.0);

    // Add components in correct order (Gravity first!)
    sim.AddComponent(std::move(grav));
    sim.AddComponent(std::move(pm));

    // Wire components using AddRoutes
    std::vector<signal::SignalRoute> routes = {
        {"Gravity.position.x", "PointMass3DOF.position.x"},
        {"Gravity.position.y", "PointMass3DOF.position.y"},
        {"Gravity.position.z", "PointMass3DOF.position.z"},
        {"Gravity.mass", "PointMass3DOF.mass"},
        {"PointMass3DOF.force.x", "Gravity.force.x"},
        {"PointMass3DOF.force.y", "Gravity.force.y"},
        {"PointMass3DOF.force.z", "Gravity.force.z"},
    };
    sim.AddRoutes(routes);

    sim.Stage();

    // Compute initial mechanical energy
    auto compute_energy = [&](const JanusVector<double> &state) {
        double rx = state[0], ry = state[1], rz = state[2];
        double vx = state[3], vy = state[4], vz = state[5];
        double r = std::sqrt(rx * rx + ry * ry + rz * rz);
        double v = std::sqrt(vx * vx + vy * vy + vz * vz);
        double KE = 0.5 * mass * v * v;
        double PE = -mu * mass / r;
        return KE + PE;
    };

    double E_initial = compute_energy(sim.GetState());

    // Orbital period: T = 2π√(r³/μ)
    double T_orbit = 2.0 * M_PI * std::sqrt(r0 * r0 * r0 / mu);

    // Simulate for 1/4 orbit with dt = 1 second
    double dt = 1.0;
    int steps = static_cast<int>(T_orbit / 4.0 / dt);

    for (int i = 0; i < steps; ++i) {
        sim.Step(dt);
    }

    double E_final = compute_energy(sim.GetState());

    // Energy should be conserved to within integration tolerance
    // For RK4 with dt=1s over ~1400 steps (quarter orbit ~92 min),
    // expect relative energy error < 0.1% (conservative bound)
    double relative_energy_error = std::abs((E_final - E_initial) / E_initial);
    EXPECT_LT(relative_energy_error, 1e-3);

    // Also verify we've moved roughly 90 degrees around the orbit
    auto final_state = sim.GetState();
    double rx = final_state[0], ry = final_state[1];
    double angle = std::atan2(ry, rx);    // Should be ~π/2 (90°)
    EXPECT_NEAR(angle, M_PI / 2.0, 0.01); // Within ~0.6°
}
