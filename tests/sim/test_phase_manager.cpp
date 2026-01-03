#include <gtest/gtest.h>
#include <icarus/signal/Registry.hpp>
#include <icarus/sim/PhaseManager.hpp>

using namespace icarus;

// ============================================================================
// PhaseConfig Tests
// ============================================================================

TEST(PhaseConfig, GetPhaseValue) {
    PhaseConfig config;
    config.definitions = {{"GROUND", 0}, {"BOOST", 1}, {"COAST", 2}};

    EXPECT_EQ(config.GetPhaseValue("GROUND"), 0);
    EXPECT_EQ(config.GetPhaseValue("BOOST"), 1);
    EXPECT_EQ(config.GetPhaseValue("COAST"), 2);
    EXPECT_FALSE(config.GetPhaseValue("INVALID").has_value());
}

TEST(PhaseConfig, GetPhaseName) {
    PhaseConfig config;
    config.definitions = {{"GROUND", 0}, {"BOOST", 1}, {"COAST", 2}};

    EXPECT_EQ(config.GetPhaseName(0), "GROUND");
    EXPECT_EQ(config.GetPhaseName(1), "BOOST");
    EXPECT_EQ(config.GetPhaseName(2), "COAST");
    EXPECT_EQ(config.GetPhaseName(99), ""); // Not found
}

// ============================================================================
// PhaseManager Configuration Tests
// ============================================================================

TEST(PhaseManager, DefaultState) {
    PhaseManager<double> pm;

    EXPECT_FALSE(pm.IsConfigured());
    EXPECT_EQ(pm.CurrentPhase(), 0);
    EXPECT_FALSE(pm.PhaseChangedThisStep());
}

TEST(PhaseManager, Configure) {
    PhaseManager<double> pm;
    PhaseConfig config;
    config.definitions = {{"GROUND", 0}, {"BOOST", 1}};
    config.initial_phase = "GROUND";
    config.entity_prefix = "Vehicle";

    pm.Configure(config);

    EXPECT_TRUE(pm.IsConfigured());
    EXPECT_EQ(pm.CurrentPhase(), 0);
    EXPECT_EQ(pm.CurrentPhaseName(), "GROUND");
    EXPECT_EQ(pm.GetPhaseSignalPath(), "Vehicle.phase");
}

TEST(PhaseManager, ConfigureInvalidInitialPhase) {
    PhaseManager<double> pm;
    PhaseConfig config;
    config.definitions = {{"GROUND", 0}, {"BOOST", 1}};
    config.initial_phase = "INVALID";

    EXPECT_THROW(pm.Configure(config), ConfigError);
}

TEST(PhaseManager, ConfigureInvalidCondition) {
    PhaseManager<double> pm;
    PhaseConfig config;
    config.definitions = {{"GROUND", 0}, {"BOOST", 1}};
    config.initial_phase = "GROUND";
    config.transitions = {{0, 1, "invalid @ syntax"}};

    EXPECT_THROW(pm.Configure(config), ConfigError);
}

// ============================================================================
// Phase Transition Tests
// ============================================================================

class PhaseManagerTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Set up registry with test signals
        registry_.set_current_component("Propulsion");
        registry_.register_output("Propulsion.ignition", &ignition_, "", "Ignition signal");
        registry_.register_output("Propulsion.fuel_mass", &fuel_mass_, "kg", "Fuel mass");
        registry_.clear_current_component();

        registry_.set_current_component("Vehicle");
        registry_.register_output("Vehicle.altitude", &altitude_, "m", "Altitude");
        registry_.clear_current_component();

        // Configure phase manager
        config_.definitions = {
            {"GROUND", 0}, {"BOOST", 1}, {"COAST", 2}, {"DESCENT", 3}, {"LANDED", 4}};
        config_.initial_phase = "GROUND";
        config_.entity_prefix = "Vehicle";
        config_.transitions = {
            {0, 1, "Propulsion.ignition == 1"},    // GROUND -> BOOST on ignition
            {1, 2, "Propulsion.fuel_mass < 0.01"}, // BOOST -> COAST on fuel depletion
            {2, 3, "Vehicle.altitude < 10000"},    // COAST -> DESCENT on low altitude
            {3, 4, "Vehicle.altitude < 1"}         // DESCENT -> LANDED on touchdown
        };
    }

    SignalRegistry<double> registry_;
    PhaseConfig config_;

    double ignition_ = 0.0;
    double fuel_mass_ = 100.0;
    double altitude_ = 50000.0;
};

TEST_F(PhaseManagerTest, NoTransitionWhenConditionFalse) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    // Ignition is 0, should stay in GROUND
    pm.EvaluateTransitions(registry_);

    EXPECT_EQ(pm.CurrentPhase(), 0);
    EXPECT_EQ(pm.CurrentPhaseName(), "GROUND");
    EXPECT_FALSE(pm.PhaseChangedThisStep());
}

TEST_F(PhaseManagerTest, TransitionWhenConditionTrue) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    // Set ignition
    ignition_ = 1.0;
    pm.EvaluateTransitions(registry_);

    EXPECT_EQ(pm.CurrentPhase(), 1);
    EXPECT_EQ(pm.CurrentPhaseName(), "BOOST");
    EXPECT_TRUE(pm.PhaseChangedThisStep());
    EXPECT_EQ(pm.PreviousPhase(), 0);
}

TEST_F(PhaseManagerTest, NoCascadeWithinSingleStep) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    // Set conditions for multiple transitions at once
    ignition_ = 1.0;    // GROUND -> BOOST
    fuel_mass_ = 0.001; // BOOST -> COAST (but shouldn't fire same step)

    pm.EvaluateTransitions(registry_);

    // Should only transition to BOOST, not cascade to COAST
    EXPECT_EQ(pm.CurrentPhase(), 1);
    EXPECT_EQ(pm.CurrentPhaseName(), "BOOST");
}

TEST_F(PhaseManagerTest, SequentialTransitions) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    // Step 1: GROUND -> BOOST
    ignition_ = 1.0;
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "BOOST");

    // Step 2: BOOST -> COAST
    fuel_mass_ = 0.001;
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "COAST");

    // Step 3: COAST -> DESCENT
    altitude_ = 5000.0;
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "DESCENT");

    // Step 4: DESCENT -> LANDED
    altitude_ = 0.5;
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "LANDED");
}

TEST_F(PhaseManagerTest, TransitionFromAnyPhase) {
    // Create config with any-phase transition at higher priority (first)
    PhaseConfig config;
    config.definitions = {{"A", 0}, {"B", 1}, {"EMERGENCY", 2}};
    config.initial_phase = "A";
    config.entity_prefix = "Vehicle";
    config.transitions = {
        {-1, 2, "Vehicle.altitude < 0"},    // Emergency: any -> EMERGENCY (highest priority)
        {0, 1, "Propulsion.ignition == 1"}, // A -> B
    };

    PhaseManager<double> pm;
    pm.Configure(config);

    // Transition A -> B
    ignition_ = 1.0;
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "B");

    // Altitude goes negative - should trigger any->EMERGENCY even from B
    altitude_ = -10.0;
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "EMERGENCY");
}

TEST_F(PhaseManagerTest, TransitionOrderMatters) {
    // Create config with overlapping conditions
    PhaseConfig config;
    config.definitions = {{"A", 0}, {"B", 1}, {"C", 2}};
    config.initial_phase = "A";
    config.transitions = {
        {0, 1, "Propulsion.fuel_mass < 50"},  // A -> B
        {0, 2, "Propulsion.fuel_mass < 100"}, // A -> C
    };

    PhaseManager<double> pm;
    pm.Configure(config);

    // Both conditions true, but first one should win
    fuel_mass_ = 25.0;
    pm.EvaluateTransitions(registry_);

    EXPECT_EQ(pm.CurrentPhaseName(), "B"); // First matching transition
}

TEST_F(PhaseManagerTest, PhaseChangedFlag) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    // No transition
    pm.EvaluateTransitions(registry_);
    EXPECT_FALSE(pm.PhaseChangedThisStep());

    // Transition
    ignition_ = 1.0;
    pm.EvaluateTransitions(registry_);
    EXPECT_TRUE(pm.PhaseChangedThisStep());

    // No transition (stay in BOOST)
    pm.EvaluateTransitions(registry_);
    EXPECT_FALSE(pm.PhaseChangedThisStep());
}

// ============================================================================
// Reset and Manual Phase Setting
// ============================================================================

TEST_F(PhaseManagerTest, Reset) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    // Transition to BOOST
    ignition_ = 1.0;
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "BOOST");

    // Reset should go back to GROUND
    pm.Reset();
    EXPECT_EQ(pm.CurrentPhaseName(), "GROUND");
    EXPECT_FALSE(pm.PhaseChangedThisStep());
}

TEST_F(PhaseManagerTest, SetPhaseByName) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    pm.SetPhase("COAST");
    EXPECT_EQ(pm.CurrentPhaseName(), "COAST");
    EXPECT_EQ(pm.CurrentPhase(), 2);
    EXPECT_TRUE(pm.PhaseChangedThisStep());
}

TEST_F(PhaseManagerTest, SetPhaseByNameInvalid) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    EXPECT_THROW(pm.SetPhase("INVALID"), ConfigError);
}

TEST_F(PhaseManagerTest, SetPhaseByValue) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    pm.SetPhaseValue(3);
    EXPECT_EQ(pm.CurrentPhase(), 3);
    EXPECT_EQ(pm.CurrentPhaseName(), "DESCENT");
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(PhaseManagerTest, EvaluateWithoutConfigure) {
    PhaseManager<double> pm;

    // Should not crash, just do nothing
    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhase(), 0);
}

TEST_F(PhaseManagerTest, NoTransitions) {
    PhaseConfig config;
    config.definitions = {{"ONLY", 0}};
    config.initial_phase = "ONLY";
    // No transitions

    PhaseManager<double> pm;
    pm.Configure(config);

    pm.EvaluateTransitions(registry_);
    EXPECT_EQ(pm.CurrentPhaseName(), "ONLY");
    EXPECT_FALSE(pm.PhaseChangedThisStep());
}

TEST_F(PhaseManagerTest, EmptyEntityPrefix) {
    PhaseConfig config;
    config.definitions = {{"A", 0}};
    config.initial_phase = "A";
    config.entity_prefix = ""; // No prefix

    PhaseManager<double> pm;
    pm.Configure(config);

    EXPECT_EQ(pm.GetPhaseSignalPath(), "phase");
}

TEST_F(PhaseManagerTest, SignalNotFoundSkipsTransition) {
    PhaseConfig config;
    config.definitions = {{"A", 0}, {"B", 1}};
    config.initial_phase = "A";
    config.transitions = {{0, 1, "NonExistent.signal > 0"}};

    PhaseManager<double> pm;
    pm.Configure(config);

    // Should not throw, just skip the transition
    EXPECT_NO_THROW(pm.EvaluateTransitions(registry_));
    EXPECT_EQ(pm.CurrentPhaseName(), "A");
}

// ============================================================================
// Config Access
// ============================================================================

TEST_F(PhaseManagerTest, GetConfig) {
    PhaseManager<double> pm;
    pm.Configure(config_);

    const auto &retrieved = pm.GetConfig();
    EXPECT_EQ(retrieved.initial_phase, "GROUND");
    EXPECT_EQ(retrieved.definitions.size(), 5u);
    EXPECT_EQ(retrieved.transitions.size(), 4u);
}
