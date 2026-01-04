/**
 * @file test_component_epoch.cpp
 * @brief Tests for component epoch access
 *
 * Verifies that components can access the simulator's epoch
 * after Stage() for time-dependent calculations like ephemeris.
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>
#include <testing/DummyComponent.hpp>

namespace icarus {
namespace {

// =============================================================================
// Component Epoch Access Tests
// =============================================================================

TEST(ComponentEpoch, NullBeforeStage) {
    DummyComponent<double> comp("TestComp");
    // Epoch should be nullptr before staging
    EXPECT_EQ(comp.GetEpoch(), nullptr);
}

TEST(ComponentEpoch, AvailableAfterStage) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Counter"));
    sim.Stage();

    // After staging, component should have epoch access
    // We can't directly access the component's epoch, but we can verify
    // the simulator's epoch accessors work
    EXPECT_GT(sim.JD_TT(), 0.0);
    EXPECT_GT(sim.JD_UTC(), 0.0);
}

TEST(ComponentEpoch, EpochMatchesSimulator) {
    // Create simulator with explicit epoch configuration
    SimulatorConfig config;
    config.name = "EpochTest";
    config.dt = 0.01;
    config.t_end = 1.0;
    config.epoch.system = "UTC";
    config.epoch.reference = "2024-07-15T12:00:00Z";

    auto sim = Simulator::FromConfig(config);
    ASSERT_NE(sim, nullptr);

    sim->Stage();

    // Verify epoch is set
    double jd_tt = sim->JD_TT();
    EXPECT_GT(jd_tt, 2451545.0); // After J2000.0
    EXPECT_LT(jd_tt, 2500000.0); // Reasonable future

    // Take a step and verify epoch advances
    double jd_tt_before = sim->JD_TT();
    sim->Step(1.0);
    double jd_tt_after = sim->JD_TT();

    // 1 second = 1/86400 days
    double expected_delta = 1.0 / 86400.0;
    EXPECT_NEAR(jd_tt_after - jd_tt_before, expected_delta, 1e-10);
}

TEST(ComponentEpoch, TimeScaleConversions) {
    SimulatorConfig config;
    config.name = "TimeScaleTest";
    config.dt = 0.01;
    config.t_end = 1.0;
    config.epoch.system = "UTC";
    config.epoch.reference = "2024-01-01T00:00:00Z";

    auto sim = Simulator::FromConfig(config);
    ASSERT_NE(sim, nullptr);

    sim->Stage();

    // All time scales should return valid Julian Dates
    EXPECT_GT(sim->JD_UTC(), 0.0);
    EXPECT_GT(sim->JD_TAI(), 0.0);
    EXPECT_GT(sim->JD_TT(), 0.0);
    EXPECT_GT(sim->JD_GPS(), 0.0);

    // TAI > UTC (due to leap seconds)
    // TT > TAI (by 32.184s)
    double jd_utc = sim->JD_UTC();
    double jd_tai = sim->JD_TAI();
    double jd_tt = sim->JD_TT();

    EXPECT_GT(jd_tai, jd_utc);
    EXPECT_GT(jd_tt, jd_tai);

    // TT - TAI should be 32.184 seconds = 32.184/86400 days
    double tt_tai_offset_days = 32.184 / 86400.0;
    EXPECT_NEAR(jd_tt - jd_tai, tt_tai_offset_days, 1e-9);
}

TEST(ComponentEpoch, ISO8601String) {
    SimulatorConfig config;
    config.name = "ISO8601Test";
    config.dt = 0.01;
    config.t_end = 1.0;
    config.epoch.system = "UTC";
    config.epoch.reference = "2024-07-15T12:30:00Z";

    auto sim = Simulator::FromConfig(config);
    ASSERT_NE(sim, nullptr);

    sim->Stage();

    std::string iso = sim->ISO8601();
    EXPECT_FALSE(iso.empty());
    EXPECT_TRUE(iso.find("2024") != std::string::npos);
    EXPECT_TRUE(iso.find("07") != std::string::npos);
    EXPECT_TRUE(iso.find("15") != std::string::npos);
}

// =============================================================================
// Symbolic Backend Epoch Test
// =============================================================================

TEST(ComponentEpochSymbolic, EpochPointerSet) {
    SignalRegistry<SymbolicScalar> registry;
    Backplane<SymbolicScalar> bp(registry);

    DummyComponent<SymbolicScalar> comp("SymComp");

    // Before stage, epoch should be null
    EXPECT_EQ(comp.GetEpoch(), nullptr);

    // Create a symbolic epoch and bind it
    auto sym_epoch = vulcan::time::Epoch<SymbolicScalar>();
    bp.set_epoch(&sym_epoch);

    bp.set_context(comp.Entity(), comp.Name());
    comp.Provision(bp);
    bp.bind_epoch_to(comp);
    comp.Stage(bp);

    // After binding, epoch should be accessible
    EXPECT_NE(comp.GetEpoch(), nullptr);
}

} // namespace
} // namespace icarus
