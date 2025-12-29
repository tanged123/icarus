/**
 * @file test_component.cpp
 * @brief Tests for Component Base (Phase 1.4)
 *
 * Tests Backplane, Component lifecycle, and Simulator integration.
 * Updated for Phase 4.0.7 non-templated Simulator API.
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// Include DummyComponent from components library
#include <testing/DummyComponent.hpp>

namespace icarus {
namespace {

// =============================================================================
// Backplane Tests
// =============================================================================

TEST(Backplane, ContextManagement) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    bp.set_context("Vehicle", "Engine");
    EXPECT_EQ(bp.full_prefix(), "Vehicle.Engine");

    bp.clear_context();
    EXPECT_EQ(bp.full_prefix(), "");
}

TEST(Backplane, ContextWithoutEntity) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    bp.set_context("", "Environment");
    EXPECT_EQ(bp.full_prefix(), "Environment");
}

TEST(Backplane, FullNameGeneration) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double value = 42.0;
    bp.set_context("X15", "Propulsion");
    bp.register_output("thrust", &value, "N", "Thrust output");

    EXPECT_TRUE(registry.HasSignal("X15.Propulsion.thrust"));
}

TEST(Backplane, RegisterOutput) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double thrust = 1000.0;
    bp.set_context("", "Engine");
    bp.register_output("thrust", &thrust, "N", "Engine thrust");

    EXPECT_TRUE(registry.HasSignal("Engine.thrust"));

    auto handle = registry.resolve<double>("Engine.thrust");
    EXPECT_DOUBLE_EQ(*handle, 1000.0);

    // Modify through original
    thrust = 2000.0;
    EXPECT_DOUBLE_EQ(*handle, 2000.0);
}

TEST(Backplane, RegisterVec3) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    Vec3<double> position(1.0, 2.0, 3.0);
    bp.set_context("", "Nav");
    bp.register_output_vec3("position", &position, "m", "Position");

    EXPECT_TRUE(registry.HasSignal("Nav.position.x"));
    EXPECT_TRUE(registry.HasSignal("Nav.position.y"));
    EXPECT_TRUE(registry.HasSignal("Nav.position.z"));

    EXPECT_DOUBLE_EQ(*registry.resolve<double>("Nav.position.x"), 1.0);
    EXPECT_DOUBLE_EQ(*registry.resolve<double>("Nav.position.y"), 2.0);
    EXPECT_DOUBLE_EQ(*registry.resolve<double>("Nav.position.z"), 3.0);
}

TEST(Backplane, Resolve) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double value = 100.0;
    registry.register_output("source.signal", &value);

    auto handle = bp.resolve<double>("source.signal");
    EXPECT_DOUBLE_EQ(*handle, 100.0);
}

TEST(Backplane, DependencyTracking) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double val1 = 1.0, val2 = 2.0;
    bp.set_context("", "Producer");
    bp.register_output("out1", &val1);
    bp.register_output("out2", &val2);

    EXPECT_EQ(bp.registered_outputs().size(), 2);
    EXPECT_EQ(bp.registered_outputs()[0], "Producer.out1");
    EXPECT_EQ(bp.registered_outputs()[1], "Producer.out2");

    bp.clear_tracking();
    EXPECT_EQ(bp.registered_outputs().size(), 0);

    // Now resolve some inputs
    bp.set_context("", "Consumer");
    (void)bp.resolve<double>("Producer.out1");
    (void)bp.resolve<double>("Producer.out2");

    EXPECT_EQ(bp.resolved_inputs().size(), 2);
    EXPECT_EQ(bp.resolved_inputs()[0], "Producer.out1");
    EXPECT_EQ(bp.resolved_inputs()[1], "Producer.out2");
}

TEST(Backplane, HasSignal) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double value = 0.0;
    registry.register_output("existing.signal", &value);

    EXPECT_TRUE(bp.has_signal("existing.signal"));
    EXPECT_FALSE(bp.has_signal("nonexistent.signal"));
}

// =============================================================================
// Component Tests
// =============================================================================

TEST(Component, FullNameWithEntity) {
    DummyComponent<double> comp("MainEngine", "Rocket");
    EXPECT_EQ(comp.FullName(), "Rocket.MainEngine");
}

TEST(Component, FullNameWithoutEntity) {
    DummyComponent<double> comp("Environment", "");
    EXPECT_EQ(comp.FullName(), "Environment");
}

TEST(Component, TypeName) {
    DummyComponent<double> comp("Tracker", "Vehicle");
    EXPECT_EQ(comp.TypeName(), "DummyComponent");
}

TEST(Component, DeclareOutputs) {
    DummyComponent<double> comp;
    auto outputs = comp.DeclareOutputs();
    EXPECT_EQ(outputs.size(), 2);
    EXPECT_EQ(outputs[0].name, "counter");
    EXPECT_EQ(outputs[1].name, "time");
}

TEST(Component, LifecycleStateInitial) {
    DummyComponent<double> comp;
    EXPECT_FALSE(comp.IsProvisioned());
    EXPECT_FALSE(comp.IsStaged());
}

// =============================================================================
// DummyComponent Lifecycle Tests
// =============================================================================

TEST(DummyComponent, Lifecycle) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    DummyComponent<double> comp("Counter", "Test");

    // Provision
    bp.set_context(comp.Entity(), comp.Name());
    ComponentConfig config;
    config.name = comp.Name();
    config.entity = comp.Entity();
    comp.Provision(bp, config);

    EXPECT_TRUE(registry.HasSignal("Test.Counter.counter"));
    EXPECT_TRUE(registry.HasSignal("Test.Counter.time"));

    // Stage
    comp.Stage(bp, config);

    EXPECT_DOUBLE_EQ(comp.counter(), 0.0);
    EXPECT_DOUBLE_EQ(comp.last_time(), 0.0);

    // Step
    comp.Step(1.0, 0.01);
    EXPECT_DOUBLE_EQ(comp.counter(), 1.0);
    EXPECT_DOUBLE_EQ(comp.last_time(), 1.0);

    comp.Step(1.01, 0.01);
    EXPECT_DOUBLE_EQ(comp.counter(), 2.0);
    EXPECT_DOUBLE_EQ(comp.last_time(), 1.01);
}

// =============================================================================
// Simulator Integration Tests (Updated for new API)
// =============================================================================

TEST(Simulator, AddComponent) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Test"));
    EXPECT_EQ(sim.NumComponents(), 1);
}

TEST(Simulator, ProvisionStage) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Counter"));

    // Note: Provision() is now private, called via Configure/Stage
    sim.Stage();
    EXPECT_EQ(sim.GetPhase(), Phase::Staged);
    EXPECT_TRUE(sim.GetBackplane().has_signal("Counter.counter"));
    EXPECT_TRUE(sim.GetBackplane().has_signal("Counter.time"));
}

TEST(Simulator, Step) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Counter"));

    sim.Stage();

    sim.Step(0.01);
    EXPECT_EQ(sim.GetPhase(), Phase::Running);
    EXPECT_DOUBLE_EQ(sim.Time(), 0.01);

    // Access signal through backplane
    auto handle = sim.GetBackplane().resolve<double>("Counter.counter");
    EXPECT_DOUBLE_EQ(*handle, 1.0);

    sim.Step(0.01);
    EXPECT_DOUBLE_EQ(*handle, 2.0);
    EXPECT_DOUBLE_EQ(sim.Time(), 0.02);
}

TEST(Simulator, MultipleComponents) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Counter1"));
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Counter2"));

    sim.Stage();

    EXPECT_TRUE(sim.GetBackplane().has_signal("Counter1.counter"));
    EXPECT_TRUE(sim.GetBackplane().has_signal("Counter2.counter"));

    sim.Step(0.01);

    EXPECT_DOUBLE_EQ(*sim.GetBackplane().resolve<double>("Counter1.counter"), 1.0);
    EXPECT_DOUBLE_EQ(*sim.GetBackplane().resolve<double>("Counter2.counter"), 1.0);
}

TEST(Simulator, EntityNamespacing) {
    Simulator sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Engine", "Stage1"));
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Engine", "Stage2"));

    sim.Stage();

    EXPECT_TRUE(sim.GetBackplane().has_signal("Stage1.Engine.counter"));
    EXPECT_TRUE(sim.GetBackplane().has_signal("Stage2.Engine.counter"));
}

// =============================================================================
// Symbolic Backend Tests (Components only - Simulator<MX> removed)
// =============================================================================

TEST(ComponentSymbolic, DummyLifecycle) {
    SignalRegistry<SymbolicScalar> registry;
    Backplane<SymbolicScalar> bp(registry);

    DummyComponent<SymbolicScalar> comp("SymCounter");

    bp.set_context(comp.Entity(), comp.Name());
    ComponentConfig config;
    config.name = comp.Name();
    comp.Provision(bp, config);
    comp.Stage(bp, config);

    EXPECT_TRUE(registry.HasSignal("SymCounter.counter"));
    EXPECT_TRUE(registry.HasSignal("SymCounter.time"));
}

// NOTE: SimulatorSymbolic tests removed - Simulator is no longer templated.
// Symbolic mode is handled internally during Stage() for analysis.

} // namespace
} // namespace icarus
