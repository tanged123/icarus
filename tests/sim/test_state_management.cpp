/**
 * @file test_state_management.cpp
 * @brief Tests for State Management (Phase 2.1)
 *
 * Tests global state vectors, pointer-based scatter/gather, and integrator
 * compatibility.
 *
 * Updated for Phase 4.0.7 non-templated Simulator API.
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
// State Binding Tests
// =============================================================================

TEST(StateManagement, ComponentsAreBound) {
    Simulator sim;

    auto compA = std::make_unique<StatefulComponent<double>>("A", 3);
    auto *ptrA = compA.get();

    sim.AddComponent(std::move(compA));
    sim.Stage();

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

    // Component pointer should see updated values
    auto *state_ptr = static_cast<StatefulComponent<double> *>(ptr)->GetStatePtr();
    EXPECT_DOUBLE_EQ(state_ptr[0], 10.0);
    EXPECT_DOUBLE_EQ(state_ptr[1], 20.0);
    EXPECT_DOUBLE_EQ(state_ptr[2], 30.0);
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
// Error Handling Tests
// =============================================================================

TEST(StateManagement, StateSizeMismatchInBindStateThrows) {
    StatefulComponent<double> comp("Test", 5);

    double state[3];
    double state_dot[3];

    EXPECT_THROW(comp.BindState(state, state_dot, 3), // Wrong size (expects 5)
                 StateSizeMismatchError);
}

TEST(StateManagement, StateSizeMismatchErrorDetails) {
    try {
        StatefulComponent<double> comp("Test", 5);
        double state[3], state_dot[3];
        comp.BindState(state, state_dot, 3);
        FAIL() << "Expected StateSizeMismatchError";
    } catch (const StateSizeMismatchError &e) {
        EXPECT_EQ(e.expected(), 5);
        EXPECT_EQ(e.actual(), 3);
    }
}

// =============================================================================
// HasState Tests
// =============================================================================

TEST(Component, HasStateFalseForStateless) {
    DummyComponent<double> comp;
    EXPECT_FALSE(comp.HasState());
}

TEST(Component, HasStateTrueForStateful) {
    StatefulComponent<double> comp("Test", 3);
    EXPECT_TRUE(comp.HasState());
}

// =============================================================================
// Symbolic Mode Tests (Components only - Simulator is no longer templated)
// =============================================================================

TEST(StateManagementSymbolic, AllocationCompiles) {
    using MX = janus::SymbolicScalar;

    // Only test component-level symbolic capability
    StatefulComponent<MX> comp("A", 4);
    EXPECT_EQ(comp.StateSize(), 4);
}

// NOTE: Simulator<MX> tests removed - Simulator is no longer templated.
// Symbolic mode is handled internally during Stage() for analysis.

} // namespace
} // namespace icarus
