/**
 * @file test_state_management.cpp
 * @brief Tests for State Management (Phase 2.1, Updated for Phase 6)
 *
 * Tests unified signal model state registration, state vectors, and integrator
 * compatibility.
 *
 * Updated for Phase 6: Unified signal model where states are registered
 * via register_state() and discovered by StateManager.
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// Include test components from components library
#include <testing/DummyComponent.hpp>
#include <testing/StatefulComponent.hpp>

namespace icarus {
namespace {

// =============================================================================
// Helper: Create JanusVector from values
// =============================================================================

template <typename Scalar> JanusVector<Scalar> make_state(std::initializer_list<double> values) {
    JanusVector<Scalar> v(static_cast<Eigen::Index>(values.size()));
    Eigen::Index i = 0;
    for (double val : values) {
        v[i++] = Scalar{val};
    }
    return v;
}

// =============================================================================
// State Vector Allocation Tests
// =============================================================================

TEST(StateManagement, StateVectorAllocation) {
    Simulator sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 6));
    sim.Stage();

    auto state = sim.GetState();
    EXPECT_EQ(state.size(), 6);
}

TEST(StateManagement, StateVectorZeroInitialized) {
    Simulator sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 4));
    sim.Stage();

    auto state = sim.GetState();
    for (Eigen::Index i = 0; i < state.size(); ++i) {
        EXPECT_DOUBLE_EQ(state[i], 0.0);
    }
}

TEST(StateManagement, EmptyStateVectorWhenNoStatefulComponents) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Stateless"));
    sim.Stage();

    auto state = sim.GetState();
    EXPECT_EQ(state.size(), 0);
}

// =============================================================================
// State Binding Tests (Updated for Phase 6: Unified Signal Model)
// =============================================================================

TEST(StateManagement, ComponentsAreBound) {
    Simulator sim;

    auto compA = std::make_unique<StatefulComponent<double>>("A", 3);
    auto *ptrA = compA.get();

    sim.AddComponent(std::move(compA));
    sim.Stage();

    // In Phase 6, "bound" means states are registered and staged
    EXPECT_TRUE(static_cast<StatefulComponent<double> *>(ptrA)->IsBound());
}

// =============================================================================
// Get/Set State Tests
// =============================================================================

TEST(StateManagement, SetAndGetState) {
    Simulator sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 4));
    sim.Stage();

    // Set state
    auto new_state = make_state<double>({1.0, 2.0, 3.0, 4.0});
    sim.SetState(new_state);

    // Get state
    auto retrieved = sim.GetState();
    EXPECT_EQ(retrieved.size(), 4);
    EXPECT_DOUBLE_EQ(retrieved[0], 1.0);
    EXPECT_DOUBLE_EQ(retrieved[1], 2.0);
    EXPECT_DOUBLE_EQ(retrieved[2], 3.0);
    EXPECT_DOUBLE_EQ(retrieved[3], 4.0);
}

TEST(StateManagement, SetStateSizeMismatchThrows) {
    Simulator sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 4));
    sim.Stage();

    // Wrong size should throw
    auto wrong_size = make_state<double>({1.0, 2.0});
    EXPECT_THROW(sim.SetState(wrong_size), std::invalid_argument);
}

TEST(StateManagement, StatePointersAccessGlobalVector) {
    Simulator sim;

    auto comp = std::make_unique<StatefulComponent<double>>("A", 3);
    auto *ptr = comp.get();

    sim.AddComponent(std::move(comp));
    sim.Stage();

    // Set state via Simulator
    auto new_state = make_state<double>({10.0, 20.0, 30.0});
    sim.SetState(new_state);

    // Component's internal state should see updated values
    // (states ARE the component-owned storage in Phase 6)
    auto comp_state = static_cast<StatefulComponent<double> *>(ptr)->GetState();
    EXPECT_DOUBLE_EQ(comp_state[0], 10.0);
    EXPECT_DOUBLE_EQ(comp_state[1], 20.0);
    EXPECT_DOUBLE_EQ(comp_state[2], 30.0);
}

// =============================================================================
// Derivative Computation Tests
// =============================================================================

TEST(StateManagement, ComputeDerivatives) {
    Simulator sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 3));
    sim.Stage();

    // Set initial state
    auto X0 = make_state<double>({1.0, 2.0, 3.0});
    sim.SetState(X0);

    // Compute derivatives (x_dot = -x for StatefulComponent)
    auto X_dot = sim.ComputeDerivatives(0.0);

    EXPECT_EQ(X_dot.size(), 3);
    EXPECT_DOUBLE_EQ(X_dot[0], -1.0);
    EXPECT_DOUBLE_EQ(X_dot[1], -2.0);
    EXPECT_DOUBLE_EQ(X_dot[2], -3.0);
}

// =============================================================================
// Phase 6: State Registration Tests
// =============================================================================

TEST(StateManagement, StatesSizeAccessor) {
    // Phase 6: StatefulComponent no longer has StateSize() as override,
    // but provides GetStateSize() for testing purposes
    StatefulComponent<double> comp("Test", 5);
    EXPECT_EQ(comp.GetStateSize(), 5);
}

TEST(StateManagement, StateRegistrationCreatesSignals) {
    // Verify that register_state creates both value and derivative signals
    Simulator sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 2));
    sim.Stage();

    // The StatefulComponent registers states named "state_0", "state_1", etc.
    // Signals should be "A.state_0" and "A.state_0_dot"
    // Use Peek to verify signals exist (will throw if not found)
    EXPECT_NO_THROW((void)sim.Peek("A.state_0"));
    EXPECT_NO_THROW((void)sim.Peek("A.state_0_dot"));
    EXPECT_NO_THROW((void)sim.Peek("A.state_1"));
    EXPECT_NO_THROW((void)sim.Peek("A.state_1_dot"));
}

// =============================================================================
// Symbolic Mode Tests (Components only - Simulator is no longer templated)
// =============================================================================

TEST(StateManagementSymbolic, AllocationCompiles) {
    using MX = janus::SymbolicScalar;

    // Only test component-level symbolic capability
    StatefulComponent<MX> comp("A", 4);
    EXPECT_EQ(comp.GetStateSize(), 4);
}

// NOTE: Simulator<MX> tests removed - Simulator is no longer templated.
// Symbolic mode is handled internally during Stage() for analysis.

} // namespace
} // namespace icarus
