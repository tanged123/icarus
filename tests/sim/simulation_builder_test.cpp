/**
 * @file simulation_builder_test.cpp
 * @brief Tests for SimulationBuilder (Phase 3.1)
 *
 * Tests the fluent builder API for simulation configuration.
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// Include test components
#include <testing/DummyComponent.hpp>
#include <testing/StatefulComponent.hpp>

namespace icarus {
namespace {

// =============================================================================
// Basic Construction Tests
// =============================================================================

TEST(SimulationBuilder, DefaultConstruction) {
    SimulationBuilder<double> builder;
    auto sim = builder.Build();

    EXPECT_EQ(sim.NumComponents(), 0);
    EXPECT_EQ(sim.GetPhase(), Phase::Uninitialized);
}

TEST(SimulationBuilder, AddComponentMove) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent(std::make_unique<DummyComponent<double>>("Test"))
                   .Build();

    EXPECT_EQ(sim.NumComponents(), 1);
}

TEST(SimulationBuilder, AddComponentInPlace) {
    auto sim = SimulationBuilder<double>().AddComponent<DummyComponent<double>>("InPlace").Build();

    EXPECT_EQ(sim.NumComponents(), 1);
}

TEST(SimulationBuilder, MultipleComponents) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("A")
                   .AddComponent<DummyComponent<double>>("B")
                   .AddComponent<DummyComponent<double>>("C")
                   .Build();

    EXPECT_EQ(sim.NumComponents(), 3);
}

// =============================================================================
// BuildAndInitialize Tests
// =============================================================================

TEST(SimulationBuilder, BuildAndInitializeProvisions) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("Test")
                   .BuildAndInitialize();

    EXPECT_EQ(sim.GetPhase(), Phase::Staged);
    EXPECT_TRUE(sim.IsInitialized());
}

TEST(SimulationBuilder, BuildAndInitializeWithStatefulComponent) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<StatefulComponent<double>>("Stateful", 3)
                   .BuildAndInitialize();

    EXPECT_EQ(sim.GetPhase(), Phase::Staged);
    EXPECT_EQ(sim.GetTotalStateSize(), 3);
}

// =============================================================================
// Wiring Tests
// =============================================================================

TEST(SimulationBuilder, WiringConfiguration) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("Source")
                   .AddComponent<DummyComponent<double>>("Sink")
                   .Wire("Sink", {{"input", "Source.output"}})
                   .Build();

    // Wiring is stored but not applied until Stage
    EXPECT_EQ(sim.NumComponents(), 2);
}

TEST(SimulationBuilder, SingleWireOverload) {
    // Test the Wire(target_input, source_output) overload
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("Source")
                   .AddComponent<DummyComponent<double>>("Sink")
                   .Wire("Sink.input", "Source.output")
                   .Build();

    EXPECT_EQ(sim.NumComponents(), 2);
}

// =============================================================================
// Configuration Tests
// =============================================================================

TEST(SimulationBuilder, SetIntegrator) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("Test")
                   .SetIntegrator<RK4Integrator<double>>()
                   .Build();

    EXPECT_NE(sim.GetIntegrator(), nullptr);
}

TEST(SimulationBuilder, SetTimeStep) {
    SimulationBuilder<double> builder;
    builder.SetTimeStep(0.005);

    EXPECT_DOUBLE_EQ(builder.GetTimeStep(), 0.005);
}

TEST(SimulationBuilder, SetEndTime) {
    SimulationBuilder<double> builder;
    builder.SetEndTime(100.0);

    EXPECT_DOUBLE_EQ(builder.GetEndTime(), 100.0);
}

TEST(SimulationBuilder, EnableProfiling) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("Test")
                   .EnableProfiling()
                   .Build();

    // Profiling should be enabled (check logger state)
    EXPECT_TRUE(sim.GetLogger().IsProfilingEnabled());
}

TEST(SimulationBuilder, DisableLogging) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("Test")
                   .DisableLogging()
                   .Build();

    // In quiet mode
    EXPECT_EQ(sim.NumComponents(), 1);
}

// =============================================================================
// Phase 5 Stubs (should throw NotImplementedError)
// =============================================================================

TEST(SimulationBuilder, LoadScenarioThrows) {
    SimulationBuilder<double> builder;
    EXPECT_THROW(builder.LoadScenario("test.yaml"), NotImplementedError);
}

TEST(SimulationBuilder, LoadServicesThrows) {
    SimulationBuilder<double> builder;
    EXPECT_THROW(builder.LoadServices("services.yaml"), NotImplementedError);
}

TEST(SimulationBuilder, LoadTrimConfigThrows) {
    SimulationBuilder<double> builder;
    EXPECT_THROW(builder.LoadTrimConfig("trim.yaml"), NotImplementedError);
}

TEST(SimulationBuilder, OverrideParamThrows) {
    SimulationBuilder<double> builder;
    EXPECT_THROW(builder.OverrideParam("test.param", 1.0), NotImplementedError);
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(SimulationBuilder, SymbolicInstantiation) {
    using Scalar = SymbolicScalar;

    // This must compile without errors
    auto sim = SimulationBuilder<Scalar>().AddComponent<DummyComponent<Scalar>>("Test").Build();

    EXPECT_EQ(sim.NumComponents(), 1);
}

TEST(SimulationBuilder, SymbolicBuildAndInitialize) {
    using Scalar = SymbolicScalar;

    auto sim = SimulationBuilder<Scalar>()
                   .AddComponent<StatefulComponent<Scalar>>("Stateful", 4)
                   .BuildAndInitialize();

    EXPECT_TRUE(sim.IsInitialized());
    EXPECT_EQ(sim.GetTotalStateSize(), 4);
}

// =============================================================================
// SimulationRunner Tests
// =============================================================================

TEST(SimulationRunner, BasicRun) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<StatefulComponent<double>>("Test", 2)
                   .BuildAndInitialize();

    SimulationRunner<double> runner(sim);
    runner.Run(0.1, 1.0);

    auto results = runner.GetResults();
    EXPECT_TRUE(results.completed);
    // Allow small overrun due to floating-point precision in while (t < t_end)
    EXPECT_GE(results.sim_time_final, 1.0);
    EXPECT_LE(results.sim_time_final, 1.2); // At most one extra step
    EXPECT_GT(results.total_steps, 0);
}

TEST(SimulationRunner, ProgressCallback) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<StatefulComponent<double>>("Test", 2)
                   .BuildAndInitialize();

    int callback_count = 0;
    SimulationRunner<double> runner(sim);
    runner.SetProgressCallback(
        [&callback_count](double /*t*/, double /*t_end*/) { callback_count++; });
    runner.Run(0.1, 0.5);

    // Should have been called once per step
    EXPECT_GT(callback_count, 0);
}

TEST(SimulationRunner, StepCallback) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<StatefulComponent<double>>("Test", 2)
                   .BuildAndInitialize();

    std::vector<double> times;
    SimulationRunner<double> runner(sim);
    runner.SetStepCallback([&times](double t) { times.push_back(t); });
    runner.Run(0.25, 1.0);

    // Should have recorded times at each step
    EXPECT_EQ(times.size(), 4);
}

// =============================================================================
// Simulator External Interface Tests
// =============================================================================

TEST(SimulatorExternalInterface, Initialize) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<DummyComponent<double>>("Test"));

    EXPECT_FALSE(sim.IsInitialized());
    sim.Initialize();
    EXPECT_TRUE(sim.IsInitialized());
}

TEST(SimulatorExternalInterface, Reset) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<StatefulComponent<double>>("Test", 3)
                   .BuildAndInitialize();

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

TEST(SimulatorExternalInterface, SetTime) {
    auto sim = SimulationBuilder<double>()
                   .AddComponent<DummyComponent<double>>("Test")
                   .BuildAndInitialize();

    sim.SetTime(42.0);
    EXPECT_DOUBLE_EQ(sim.Time(), 42.0);
}

TEST(SimulatorExternalInterface, PeekBatch) {
    Simulator<double> sim;
    auto comp = std::make_unique<DummyComponent<double>>("Test");
    sim.AddComponent(std::move(comp));
    sim.Initialize();

    // DummyComponent registers an output signal
    auto names = sim.GetSignalNames();
    if (!names.empty()) {
        auto batch = sim.PeekBatch(names);
        EXPECT_EQ(batch.size(), names.size());
    }
}

TEST(SimulatorExternalInterface, PokeBatch) {
    Simulator<double> sim;
    auto comp = std::make_unique<DummyComponent<double>>("Test");
    sim.AddComponent(std::move(comp));
    sim.Initialize();

    // Get signal names and poke values
    auto names = sim.GetSignalNames();
    if (!names.empty()) {
        std::map<std::string, double> values;
        for (const auto &name : names) {
            values[name] = 123.0;
        }
        EXPECT_NO_THROW(sim.PokeBatch(values));
    }
}

} // namespace
} // namespace icarus
