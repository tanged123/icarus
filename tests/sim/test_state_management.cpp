/**
 * @file test_state_management.cpp
 * @brief Tests for State Management (Phase 2.1)
 *
 * Tests global state vectors, pointer-based scatter/gather, and integrator
 * compatibility.
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
// State Size Calculation Tests
// =============================================================================

TEST(StateManagement, TotalStateSizeZeroWithNoComponents) {
    Simulator<double> sim;
    EXPECT_EQ(sim.GetTotalStateSize(), 0);
}

TEST(StateManagement, TotalStateSizeWithStatelessComponents) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("NoState1"));
    sim.AddComponent(std::make_unique<DummyComponent<double>>("NoState2"));
    EXPECT_EQ(sim.GetTotalStateSize(), 0);
}

TEST(StateManagement, TotalStateSizeCalculation) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 3));
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("B", 5));
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("C", 2));
    EXPECT_EQ(sim.GetTotalStateSize(), 10);
}

TEST(StateManagement, TotalStateSizeMixedComponents) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Stateless"));
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("Stateful", 4));
    EXPECT_EQ(sim.GetTotalStateSize(), 4);
}

// =============================================================================
// State Vector Allocation Tests
// =============================================================================

TEST(StateManagement, StateVectorAllocation) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 6));
    sim.Provision();
    sim.Stage();

    auto state = sim.GetState();
    EXPECT_EQ(state.size(), 6);
}

TEST(StateManagement, StateVectorZeroInitialized) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 4));
    sim.Provision();
    sim.Stage();

    auto state = sim.GetState();
    for (Eigen::Index i = 0; i < state.size(); ++i) {
        EXPECT_DOUBLE_EQ(state[i], 0.0);
    }
}

TEST(StateManagement, EmptyStateVectorWhenNoStatefulComponents) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Stateless"));
    sim.Provision();
    sim.Stage();

    auto state = sim.GetState();
    EXPECT_EQ(state.size(), 0);
}

// =============================================================================
// State Binding Tests
// =============================================================================

TEST(StateManagement, StateBindingCorrectOffsets) {
    Simulator<double> sim;

    auto compA = std::make_unique<StatefulComponent<double>>("A", 3);
    auto compB = std::make_unique<StatefulComponent<double>>("B", 4);
    auto *ptrA = compA.get();
    auto *ptrB = compB.get();

    sim.AddComponent(std::move(compA));
    sim.AddComponent(std::move(compB));
    sim.Provision();
    sim.Stage();

    // Verify layout
    auto layout = sim.GetStateLayout();
    EXPECT_EQ(layout.size(), 2);
    EXPECT_EQ(layout[0].owner, ptrA);
    EXPECT_EQ(layout[0].offset, 0);
    EXPECT_EQ(layout[0].size, 3);
    EXPECT_EQ(layout[1].owner, ptrB);
    EXPECT_EQ(layout[1].offset, 3);
    EXPECT_EQ(layout[1].size, 4);
}

TEST(StateManagement, StatePointersSeparate) {
    Simulator<double> sim;

    auto compA = std::make_unique<StatefulComponent<double>>("A", 3);
    auto compB = std::make_unique<StatefulComponent<double>>("B", 4);
    auto *ptrA = compA.get();
    auto *ptrB = compB.get();

    sim.AddComponent(std::move(compA));
    sim.AddComponent(std::move(compB));
    sim.Provision();
    sim.Stage();

    // Pointers should be offset by A's state size
    auto *stateA = static_cast<StatefulComponent<double> *>(ptrA)->GetStatePtr();
    auto *stateB = static_cast<StatefulComponent<double> *>(ptrB)->GetStatePtr();

    EXPECT_EQ(stateB - stateA, 3); // B starts 3 elements after A
}

TEST(StateManagement, ComponentsAreBound) {
    Simulator<double> sim;

    auto compA = std::make_unique<StatefulComponent<double>>("A", 3);
    auto *ptrA = compA.get();

    sim.AddComponent(std::move(compA));
    sim.Provision();
    sim.Stage();

    EXPECT_TRUE(static_cast<StatefulComponent<double> *>(ptrA)->IsBound());
}

TEST(StateManagement, ZeroStateComponentNotInLayout) {
    Simulator<double> sim;

    // DummyComponent has StateSize() = 0
    sim.AddComponent(std::make_unique<DummyComponent<double>>("NoState"));
    sim.AddComponent(std::make_unique<StatefulComponent<double>>("WithState", 3));

    sim.Provision();
    sim.Stage();

    // Only WithState should appear in layout
    auto layout = sim.GetStateLayout();
    EXPECT_EQ(layout.size(), 1);
    EXPECT_EQ(layout[0].size, 3);

    // Total state size is 3
    EXPECT_EQ(sim.GetTotalStateSize(), 3);
}

// =============================================================================
// Get/Set State Tests
// =============================================================================

TEST(StateManagement, SetAndGetState) {
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 4));
    sim.Provision();
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
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 4));
    sim.Provision();
    sim.Stage();

    // Wrong size should throw
    auto wrong_size = make_state<double>({1.0, 2.0});
    EXPECT_THROW(sim.SetState(wrong_size), StateError);
}

TEST(StateManagement, StatePointersAccessGlobalVector) {
    Simulator<double> sim;

    auto comp = std::make_unique<StatefulComponent<double>>("A", 3);
    auto *ptr = comp.get();

    sim.AddComponent(std::move(comp));
    sim.Provision();
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
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 3));
    sim.Provision();
    sim.Stage();

    // Set initial state
    auto X0 = make_state<double>({1.0, 2.0, 3.0});
    sim.SetState(X0);

    // Compute derivatives (x_dot = -x for StatefulComponent)
    const auto &X_dot = sim.ComputeDerivatives(0.0);

    EXPECT_EQ(X_dot.size(), 3);
    EXPECT_DOUBLE_EQ(X_dot[0], -1.0);
    EXPECT_DOUBLE_EQ(X_dot[1], -2.0);
    EXPECT_DOUBLE_EQ(X_dot[2], -3.0);
}

TEST(StateManagement, GetDerivativesAfterCompute) {
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 2));
    sim.Provision();
    sim.Stage();

    auto X0 = make_state<double>({5.0, 10.0});
    sim.SetState(X0);

    sim.ComputeDerivatives(0.0);
    const auto &X_dot = sim.GetDerivatives();

    EXPECT_EQ(X_dot.size(), 2);
    EXPECT_DOUBLE_EQ(X_dot[0], -5.0);
    EXPECT_DOUBLE_EQ(X_dot[1], -10.0);
}

TEST(StateManagement, DerivativesZeroedEachCompute) {
    Simulator<double> sim;

    sim.AddComponent(std::make_unique<StatefulComponent<double>>("A", 2));
    sim.Provision();
    sim.Stage();

    // First compute
    auto X1 = make_state<double>({1.0, 2.0});
    sim.SetState(X1);
    sim.ComputeDerivatives(0.0);

    // Second compute with different state
    auto X2 = make_state<double>({10.0, 20.0});
    sim.SetState(X2);
    const auto &X_dot = sim.ComputeDerivatives(0.0);

    // Should be fresh derivatives, not accumulated
    EXPECT_DOUBLE_EQ(X_dot[0], -10.0);
    EXPECT_DOUBLE_EQ(X_dot[1], -20.0);
}

// =============================================================================
// Nominal Timestep Tests
// =============================================================================

TEST(StateManagement, DefaultNominalDt) {
    Simulator<double> sim;
    EXPECT_DOUBLE_EQ(sim.GetNominalDt(), 0.01);
}

TEST(StateManagement, SetNominalDt) {
    Simulator<double> sim;
    sim.SetNominalDt(0.005);
    EXPECT_DOUBLE_EQ(sim.GetNominalDt(), 0.005);
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
// Symbolic Mode Tests
// =============================================================================

TEST(StateManagementSymbolic, AllocationCompiles) {
    using MX = casadi::MX;

    Simulator<MX> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<MX>>("A", 4));
    sim.Provision();
    sim.Stage();

    auto state = sim.GetState();
    EXPECT_EQ(state.size(), 4);
}

TEST(StateManagementSymbolic, DerivativeComputation) {
    using MX = casadi::MX;

    Simulator<MX> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<MX>>("A", 2));
    sim.Provision();
    sim.Stage();

    // Create symbolic state vector
    JanusVector<MX> X(2);
    X[0] = MX::sym("x0");
    X[1] = MX::sym("x1");
    sim.SetState(X);

    // Compute derivatives (symbolic)
    const auto &X_dot = sim.ComputeDerivatives(MX{0});

    EXPECT_EQ(X_dot.size(), 2);
    // Verify symbolic expressions are created (should be -x0, -x1)
}

TEST(StateManagementSymbolic, LayoutMetadata) {
    using MX = casadi::MX;

    Simulator<MX> sim;
    sim.AddComponent(std::make_unique<StatefulComponent<MX>>("A", 3));
    sim.AddComponent(std::make_unique<StatefulComponent<MX>>("B", 2));
    sim.Provision();
    sim.Stage();

    auto layout = sim.GetStateLayout();
    EXPECT_EQ(layout.size(), 2);
    EXPECT_EQ(layout[0].offset, 0);
    EXPECT_EQ(layout[0].size, 3);
    EXPECT_EQ(layout[1].offset, 3);
    EXPECT_EQ(layout[1].size, 2);
}

} // namespace
} // namespace icarus
