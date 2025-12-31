/**
 * @file test_config_structs.cpp
 * @brief Unit tests for Phase 4.0.7 configuration structs
 *
 * Part of Phase 4.0.7: Configuration Infrastructure
 */

#include <gtest/gtest.h>
#include <icarus/io/SimulationLoader.hpp>
#include <icarus/sim/SimulatorConfig.hpp>

namespace icarus {
namespace {

// =============================================================================
// TrimConfig Tests
// =============================================================================

TEST(TrimConfigTest, DefaultIsDisabled) {
    auto cfg = TrimConfig::Default();
    EXPECT_FALSE(cfg.enabled);
    EXPECT_TRUE(cfg.zero_derivatives.empty());
    EXPECT_TRUE(cfg.control_signals.empty());
    EXPECT_EQ(cfg.method, "newton");
    EXPECT_DOUBLE_EQ(cfg.tolerance, 1e-6);
}

TEST(TrimConfigTest, ValidateReturnsEmptyWhenDisabled) {
    TrimConfig cfg;
    cfg.enabled = false;
    auto errors = cfg.Validate();
    EXPECT_TRUE(errors.empty());
}

TEST(TrimConfigTest, ValidateRequiresDerivativesWhenEnabled) {
    TrimConfig cfg;
    cfg.enabled = true;
    cfg.control_signals = {"throttle"};
    auto errors = cfg.Validate();
    EXPECT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("zero_derivatives") != std::string::npos);
}

TEST(TrimConfigTest, ValidateRequiresControlsWhenEnabled) {
    TrimConfig cfg;
    cfg.enabled = true;
    cfg.zero_derivatives = {"velocity_dot"};
    auto errors = cfg.Validate();
    EXPECT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("control_signals") != std::string::npos);
}

TEST(TrimConfigTest, ValidateRejectsUnknownMethod) {
    TrimConfig cfg;
    cfg.enabled = true;
    cfg.zero_derivatives = {"velocity_dot"};
    cfg.control_signals = {"throttle"};
    cfg.method = "gradient_descent"; // Invalid
    auto errors = cfg.Validate();
    EXPECT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("method") != std::string::npos);
}

// =============================================================================
// LinearizationConfig Tests
// =============================================================================

TEST(LinearizationConfigTest, DefaultIsDisabled) {
    auto cfg = LinearizationConfig::Default();
    EXPECT_FALSE(cfg.enabled);
    EXPECT_TRUE(cfg.states.empty());
    EXPECT_FALSE(cfg.export_matlab);
}

TEST(LinearizationConfigTest, ValidateRequiresStatesWhenEnabled) {
    LinearizationConfig cfg;
    cfg.enabled = true;
    auto errors = cfg.Validate();
    EXPECT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("states") != std::string::npos);
}

// =============================================================================
// SymbolicsConfig Tests
// =============================================================================

TEST(SymbolicsConfigTest, DefaultIsDisabled) {
    auto cfg = SymbolicsConfig::Default();
    EXPECT_FALSE(cfg.enabled);
    EXPECT_TRUE(cfg.generate_dynamics);
    EXPECT_FALSE(cfg.generate_jacobian);
}

// =============================================================================
// StageConfig Tests
// =============================================================================

TEST(StageConfigTest, DefaultHasValidationEnabled) {
    auto cfg = StageConfig::Default();
    EXPECT_TRUE(cfg.validate_wiring);
    EXPECT_TRUE(cfg.warn_on_unwired);
    EXPECT_FALSE(cfg.trim.enabled);
    EXPECT_FALSE(cfg.linearization.enabled);
}

TEST(StageConfigTest, ValidatePropagatesSubErrors) {
    StageConfig cfg;
    cfg.trim.enabled = true; // Missing derivatives & controls
    auto errors = cfg.Validate();
    EXPECT_GE(errors.size(), 2u);
}

// =============================================================================
// SchedulerConfig Tests
// =============================================================================

TEST(SchedulerConfigTest, DefaultHasSingleGroup) {
    auto cfg = SchedulerConfig::Default();
    EXPECT_EQ(cfg.groups.size(), 1u);
    EXPECT_EQ(cfg.groups[0].name, "default");
    EXPECT_DOUBLE_EQ(cfg.groups[0].rate_hz, 400.0);
}

TEST(SchedulerConfigTest, MaxRateReturnsHighestRate) {
    SchedulerConfig cfg;
    cfg.groups.emplace_back("slow", 100.0);
    cfg.groups.emplace_back("fast", 1000.0);
    cfg.groups.emplace_back("medium", 400.0);
    EXPECT_DOUBLE_EQ(cfg.MaxRate(), 1000.0);
}

TEST(SchedulerConfigTest, MaxRateReturnsFallbackWhenEmpty) {
    SchedulerConfig cfg;
    cfg.groups.clear();
    EXPECT_DOUBLE_EQ(cfg.MaxRate(), 400.0);
}

TEST(SchedulerConfigTest, ValidateRejectsEmptyGroups) {
    SchedulerConfig cfg;
    cfg.groups.clear();
    auto errors = cfg.Validate();
    EXPECT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("No scheduler groups") != std::string::npos);
}

TEST(SchedulerConfigTest, ValidateRejectsInvalidRate) {
    SchedulerConfig cfg;
    cfg.groups.emplace_back("bad", -100.0);
    auto errors = cfg.Validate();
    EXPECT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("invalid rate") != std::string::npos);
}

TEST(SchedulerConfigTest, FromFileThrowsNotImplemented) {
    // FromFile is not implemented - use SimulationLoader::Load() instead
    EXPECT_THROW((void)SchedulerConfig::FromFile("nonexistent.yaml"), NotImplementedError);
}

// =============================================================================
// OutputConfig Tests
// =============================================================================

TEST(OutputConfigTest, DefaultValues) {
    auto cfg = OutputConfig::Default();
    EXPECT_EQ(cfg.directory, "./output");
    EXPECT_TRUE(cfg.data_dictionary);
    EXPECT_EQ(cfg.data_dictionary_format, "yaml");
    EXPECT_TRUE(cfg.telemetry);
    EXPECT_EQ(cfg.telemetry_format, "hdf5");
    EXPECT_FALSE(cfg.timing_report);
}

// =============================================================================
// SimulatorConfig Tests
// =============================================================================

TEST(SimulatorConfigTest, DefaultValues) {
    auto cfg = SimulatorConfig::Default();
    EXPECT_EQ(cfg.name, "Simulation");
    EXPECT_EQ(cfg.version, icarus::Version());
    EXPECT_DOUBLE_EQ(cfg.t_start, 0.0);
    EXPECT_DOUBLE_EQ(cfg.t_end, 100.0);
    EXPECT_DOUBLE_EQ(cfg.dt, 0.01);
    EXPECT_DOUBLE_EQ(cfg.reference_epoch_jd, 2451545.0); // J2000.0
}

TEST(SimulatorConfigTest, ValidateReturnsEmptyForDefault) {
    auto cfg = SimulatorConfig::Default();
    auto errors = cfg.Validate();
    EXPECT_TRUE(errors.empty());
}

TEST(SimulatorConfigTest, ValidateRejectsInvalidTime) {
    SimulatorConfig cfg;
    cfg.t_start = 100.0;
    cfg.t_end = 50.0; // Invalid: end < start
    auto errors = cfg.Validate();
    EXPECT_GE(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("t_end") != std::string::npos);
}

TEST(SimulatorConfigTest, ValidateRejectsNegativeDt) {
    SimulatorConfig cfg;
    cfg.dt = -0.01;
    auto errors = cfg.Validate();
    EXPECT_GE(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("dt") != std::string::npos);
}

TEST(SimulatorConfigTest, ValidatePropagatesSubErrors) {
    SimulatorConfig cfg;
    cfg.scheduler.groups.clear(); // Invalid scheduler
    auto errors = cfg.Validate();
    EXPECT_GE(errors.size(), 1u);
}

TEST(SimulatorConfigTest, FromFileThrowsOnMissingFile) {
    EXPECT_THROW((void)SimulatorConfig::FromFile("nonexistent.yaml"), ConfigError);
}

TEST(SimulatorConfigTest, FromFilesThrowsNotImplemented) {
    // FromFiles is not implemented - use SimulationLoader::Load() instead
    EXPECT_THROW((void)SimulatorConfig::FromFiles("master.yaml", "routes.yaml"),
                 NotImplementedError);
}

// =============================================================================
// TopologyConfig Tests
// =============================================================================

TEST(TopologyConfigTest, DefaultValues) {
    TopologyConfig cfg;
    EXPECT_EQ(cfg.mode, SchedulingMode::Explicit);
    EXPECT_EQ(cfg.cycle_detection, TopologyConfig::CycleHandling::Error);
    EXPECT_TRUE(cfg.log_order);
}

// =============================================================================
// GroupMember Tests
// =============================================================================

TEST(GroupMemberTest, DefaultConstruction) {
    GroupMember member;
    EXPECT_TRUE(member.component.empty());
    EXPECT_EQ(member.priority, 0);
}

TEST(GroupMemberTest, ValueConstruction) {
    GroupMember member("MyComponent", 5);
    EXPECT_EQ(member.component, "MyComponent");
    EXPECT_EQ(member.priority, 5);
}

// =============================================================================
// SchedulerGroupConfig Tests
// =============================================================================

TEST(SchedulerGroupConfigTest, DefaultConstruction) {
    SchedulerGroupConfig group;
    EXPECT_TRUE(group.name.empty());
    EXPECT_DOUBLE_EQ(group.rate_hz, 400.0);
    EXPECT_EQ(group.priority, 0);
}

TEST(SchedulerGroupConfigTest, ValueConstruction) {
    SchedulerGroupConfig group("fast_group", 1000.0, 1);
    EXPECT_EQ(group.name, "fast_group");
    EXPECT_DOUBLE_EQ(group.rate_hz, 1000.0);
    EXPECT_EQ(group.priority, 1);
}

// =============================================================================
// IntegratorConfig<double> Tests (for SimulatorConfig usage)
// =============================================================================

TEST(IntegratorConfigDoubleTest, RK4DefaultValues) {
    auto cfg = IntegratorConfig<double>::RK4Default();
    EXPECT_EQ(cfg.type, IntegratorType::RK4);
    EXPECT_DOUBLE_EQ(cfg.abs_tol, 1e-6);
    EXPECT_DOUBLE_EQ(cfg.rel_tol, 1e-6);
}

TEST(IntegratorConfigDoubleTest, RK45AdaptiveValues) {
    auto cfg = IntegratorConfig<double>::RK45Adaptive(1e-8, 1e-7);
    EXPECT_EQ(cfg.type, IntegratorType::RK45);
    EXPECT_DOUBLE_EQ(cfg.abs_tol, 1e-8);
    EXPECT_DOUBLE_EQ(cfg.rel_tol, 1e-7);
}

TEST(IntegratorConfigDoubleTest, SimulatorConfigUsesIntegratorConfig) {
    SimulatorConfig cfg;
    EXPECT_EQ(cfg.integrator.type, IntegratorType::RK4);
}

// =============================================================================
// SignalRoute Tests
// =============================================================================

TEST(SignalRouteTest, DefaultValues) {
    signal::SignalRoute route;
    EXPECT_TRUE(route.input_path.empty());
    EXPECT_TRUE(route.output_path.empty());
    EXPECT_DOUBLE_EQ(route.gain, 1.0);
    EXPECT_DOUBLE_EQ(route.offset, 0.0);
    EXPECT_DOUBLE_EQ(route.delay, 0.0);
}

TEST(SignalRouteTest, ConstructorWithValues) {
    signal::SignalRoute route("input.path", "output.path", 2.0, 0.5, 0.01);
    EXPECT_EQ(route.input_path, "input.path");
    EXPECT_EQ(route.output_path, "output.path");
    EXPECT_DOUBLE_EQ(route.gain, 2.0);
    EXPECT_DOUBLE_EQ(route.offset, 0.5);
    EXPECT_DOUBLE_EQ(route.delay, 0.01);
}

TEST(SignalRouteTest, ValidateReturnsEmptyForValid) {
    signal::SignalRoute route("in", "out");
    auto errors = route.Validate();
    EXPECT_TRUE(errors.empty());
}

TEST(SignalRouteTest, ValidateRejectsMissingPaths) {
    signal::SignalRoute route;
    auto errors = route.Validate();
    EXPECT_EQ(errors.size(), 2u);
}

TEST(SignalRouteTest, ValidateRejectsNegativeDelay) {
    signal::SignalRoute route("in", "out", 1.0, 0.0, -0.1);
    auto errors = route.Validate();
    EXPECT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("delay") != std::string::npos);
}

} // namespace
} // namespace icarus
