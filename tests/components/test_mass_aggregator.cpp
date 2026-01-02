/**
 * @file test_mass_aggregator.cpp
 * @brief Unit tests for StaticMass and MassAggregator components
 *
 * Part of Phase 4: Aggregation & 6DOF
 */

#include <gtest/gtest.h>

#include <aggregators/MassAggregator.hpp>
#include <mass/StaticMass.hpp>

#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>

#include <vulcan/mass/MassProperties.hpp>

#include <cmath>

using namespace icarus;
using namespace icarus::components;

// =============================================================================
// StaticMass Unit Tests
// =============================================================================

TEST(StaticMass, Identity) {
    StaticMass<double> sm("Structure", "Vehicle");
    EXPECT_EQ(sm.Name(), "Structure");
    EXPECT_EQ(sm.Entity(), "Vehicle");
    EXPECT_EQ(sm.TypeName(), "StaticMass");
}

TEST(StaticMass, SetMassProperties) {
    StaticMass<double> sm;
    sm.SetMass(100.0);
    sm.SetCG(Vec3<double>{1.0, 0.0, 0.5});
    sm.SetDiagonalInertia(10.0, 20.0, 30.0);

    auto props = sm.GetMassProperties();
    EXPECT_DOUBLE_EQ(props.mass, 100.0);
    EXPECT_DOUBLE_EQ(props.cg(0), 1.0);
    EXPECT_DOUBLE_EQ(props.cg(1), 0.0);
    EXPECT_DOUBLE_EQ(props.cg(2), 0.5);
    EXPECT_DOUBLE_EQ(props.inertia(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(props.inertia(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(props.inertia(2, 2), 30.0);
}

TEST(StaticMass, SetFromMassProperties) {
    StaticMass<double> sm;

    auto props = vulcan::mass::MassProperties<double>::solid_sphere(10.0, 1.0);
    sm.SetFromMassProperties(props);

    auto retrieved = sm.GetMassProperties();
    EXPECT_DOUBLE_EQ(retrieved.mass, props.mass);
    EXPECT_NEAR(retrieved.inertia(0, 0), props.inertia(0, 0), 1e-10);
}

TEST(StaticMass, Provision) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    StaticMass<double> sm("TestMass", "");
    sm.SetMass(50.0);
    sm.SetCG(Vec3<double>{0.5, 0.0, 0.0});
    sm.SetDiagonalInertia(1.0, 2.0, 3.0);

    bp.set_context("", "TestMass");
    sm.Provision(bp);

    // Verify signals were registered
    EXPECT_TRUE(registry.HasOutput("TestMass.mass"));
    EXPECT_TRUE(registry.HasOutput("TestMass.cg.x"));
    EXPECT_TRUE(registry.HasOutput("TestMass.cg.y"));
    EXPECT_TRUE(registry.HasOutput("TestMass.cg.z"));
    EXPECT_TRUE(registry.HasOutput("TestMass.inertia.xx"));
    EXPECT_TRUE(registry.HasOutput("TestMass.inertia.yy"));
    EXPECT_TRUE(registry.HasOutput("TestMass.inertia.zz"));

    // Verify values via resolve
    auto mass_handle = registry.resolve<double>("TestMass.mass");
    EXPECT_DOUBLE_EQ(*mass_handle, 50.0);
}

// =============================================================================
// MassAggregator Unit Tests
// =============================================================================

TEST(MassAggregator, Identity) {
    MassAggregator<double> agg("MassAgg", "Vehicle");
    EXPECT_EQ(agg.Name(), "MassAgg");
    EXPECT_EQ(agg.Entity(), "Vehicle");
    EXPECT_EQ(agg.TypeName(), "MassAggregator");
}

TEST(MassAggregator, AggregatesTwoSources) {
    // Set up registry and backplane
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // Create two static mass sources
    StaticMass<double> mass1("Mass1", "");
    mass1.SetMass(100.0);
    mass1.SetCG(Vec3<double>{0.0, 0.0, 0.0});
    mass1.SetDiagonalInertia(10.0, 10.0, 10.0);

    StaticMass<double> mass2("Mass2", "");
    mass2.SetMass(50.0);
    mass2.SetCG(Vec3<double>{2.0, 0.0, 0.0}); // CG offset from origin
    mass2.SetDiagonalInertia(5.0, 5.0, 5.0);

    // Create aggregator
    MassAggregator<double> agg("MassAgg", "");
    agg.AddSource("Mass1");
    agg.AddSource("Mass2");

    // Provision all components
    bp.set_context("", "Mass1");
    mass1.Provision(bp);

    bp.set_context("", "Mass2");
    mass2.Provision(bp);

    bp.set_context("", "MassAgg");
    agg.Provision(bp);

    // Stage all components
    bp.set_context("", "Mass1");
    mass1.Stage(bp);

    bp.set_context("", "Mass2");
    mass2.Stage(bp);

    bp.set_context("", "MassAgg");
    agg.Stage(bp);

    // Step to compute aggregated values
    agg.Step(0.0, 0.01);

    // Verify aggregated mass
    EXPECT_DOUBLE_EQ(agg.GetTotalMass(), 150.0); // 100 + 50

    // Verify aggregated CG (mass-weighted average)
    // CG = (100*0 + 50*2) / 150 = 100/150 = 0.667
    Vec3<double> expected_cg = Vec3<double>{100.0 / 150.0, 0.0, 0.0};
    auto actual_cg = agg.GetCG();
    EXPECT_NEAR(actual_cg(0), expected_cg(0), 1e-10);
    EXPECT_NEAR(actual_cg(1), expected_cg(1), 1e-10);
    EXPECT_NEAR(actual_cg(2), expected_cg(2), 1e-10);
}

TEST(MassAggregator, ParallelAxisTheorem) {
    // Test that parallel axis theorem is correctly applied when combining
    // two masses with different CG locations

    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // Two point masses at different locations
    StaticMass<double> mass1("Mass1", "");
    mass1.SetMass(10.0);
    mass1.SetCG(Vec3<double>{1.0, 0.0, 0.0});
    mass1.SetDiagonalInertia(0.0, 0.0, 0.0); // Point mass at offset

    StaticMass<double> mass2("Mass2", "");
    mass2.SetMass(10.0);
    mass2.SetCG(Vec3<double>{-1.0, 0.0, 0.0});
    mass2.SetDiagonalInertia(0.0, 0.0, 0.0);

    MassAggregator<double> agg("MassAgg", "");
    agg.AddSource("Mass1");
    agg.AddSource("Mass2");

    bp.set_context("", "Mass1");
    mass1.Provision(bp);
    bp.set_context("", "Mass2");
    mass2.Provision(bp);
    bp.set_context("", "MassAgg");
    agg.Provision(bp);

    bp.set_context("", "Mass1");
    mass1.Stage(bp);
    bp.set_context("", "Mass2");
    mass2.Stage(bp);
    bp.set_context("", "MassAgg");
    agg.Stage(bp);

    agg.Step(0.0, 0.01);

    // Combined mass = 20 kg
    EXPECT_DOUBLE_EQ(agg.GetTotalMass(), 20.0);

    // Combined CG at origin (symmetric)
    auto cg = agg.GetCG();
    EXPECT_NEAR(cg(0), 0.0, 1e-10);
    EXPECT_NEAR(cg(1), 0.0, 1e-10);
    EXPECT_NEAR(cg(2), 0.0, 1e-10);

    // Combined inertia about CG using parallel axis theorem
    // Each mass contributes: I + m*r^2 where r is distance from combined CG
    // I_yy = I_zz = 10*1^2 + 10*1^2 = 20 (rotation about y or z axis)
    // I_xx = 0 (masses on x-axis, rotation about x has no contribution)
    auto props = agg.GetMassProperties();
    EXPECT_NEAR(props.inertia(0, 0), 0.0, 1e-10);  // Ixx
    EXPECT_NEAR(props.inertia(1, 1), 20.0, 1e-10); // Iyy
    EXPECT_NEAR(props.inertia(2, 2), 20.0, 1e-10); // Izz
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(StaticMassSymbolic, BasicCompiles) {
    // Verify symbolic mode compiles
    StaticMass<janus::SymbolicScalar> sm;
    EXPECT_EQ(sm.Name(), "StaticMass");
}

TEST(MassAggregatorSymbolic, BasicCompiles) {
    MassAggregator<janus::SymbolicScalar> agg;
    EXPECT_EQ(agg.Name(), "MassAggregator");
}
