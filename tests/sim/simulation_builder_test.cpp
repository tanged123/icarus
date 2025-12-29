/**
 * @file simulation_builder_test.cpp
 * @brief Tests for SimulationBuilder (Phase 3.1)
 *
 * TEMPORARILY DISABLED: SimulationBuilder<T> needs update for Phase 4.0.7
 * The templated builder API is being replaced with FromConfig pattern.
 *
 * TODO: Rewrite these tests when new builder pattern is established.
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// Include test components
#include <testing/DummyComponent.hpp>
#include <testing/StatefulComponent.hpp>

namespace icarus {
namespace {

// =============================================================================
// Basic Simulator Tests (Updated for Phase 4.0.7)
// =============================================================================

TEST(Simulator, BasicConstruction) {
    Simulator sim;
    EXPECT_EQ(sim.GetPhase(), Phase::Uninitialized);
    EXPECT_EQ(sim.NumComponents(), 0);
}

TEST(Simulator, AddAndStage) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Test"));
    sim.Stage();

    EXPECT_EQ(sim.NumComponents(), 1);
    EXPECT_EQ(sim.GetPhase(), Phase::Staged);
}

TEST(Simulator, Reset) {
    Simulator sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("Test", 3));
    sim.Stage();

    // Step forward
    sim.Step(0.1);
    sim.Step(0.1);
    double t_before = sim.Time();
    EXPECT_GT(t_before, 0.0);

    // Reset
    sim.Reset();
    EXPECT_DOUBLE_EQ(sim.Time(), 0.0);
    EXPECT_EQ(sim.GetPhase(), Phase::Staged);
}

// =============================================================================
// NOTE: SimulationBuilder<T>, SimulationRunner<T>, and related tests
// are disabled pending Phase 4.0.7 API redesign.
// =============================================================================

} // namespace
} // namespace icarus
