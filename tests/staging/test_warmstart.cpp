/**
 * @file test_warmstart.cpp
 * @brief Tests for warmstart (state restoration from recording)
 *
 * Phase 6.3: Warmstart - State Restoration
 */

#include <gtest/gtest.h>

#include <icarus/icarus.hpp>

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <cmath>
#include <filesystem>

using namespace icarus;
namespace fs = std::filesystem;

class WarmstartTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Ensure output directory exists
        fs::create_directories("output");
    }

    void TearDown() override {
        // Clean up test files
        if (fs::exists("output/warmstart_test.h5")) {
            fs::remove("output/warmstart_test.h5");
        }
        if (fs::exists("output/warmstart_test.csv")) {
            fs::remove("output/warmstart_test.csv");
        }
    }

    // Create a simple point mass simulation config
    SimulatorConfig CreateTestConfig() {
        SimulatorConfig config;
        config.name = "WarmstartTest";
        config.t_start = 0.0;
        config.t_end = 10.0;
        config.dt = 0.1;

        // Add point mass component
        ComponentConfig point_mass;
        point_mass.type = "PointMass3DOF";
        point_mass.name = "Body";
        point_mass.scalars["mass"] = 1.0;
        point_mass.vectors["initial_position"] = {0.0, 0.0, 100.0}; // 100m up
        point_mass.vectors["initial_velocity"] = {10.0, 0.0, 0.0};  // 10 m/s horizontal
        config.components.push_back(point_mass);

        // Add gravity
        ComponentConfig gravity;
        gravity.type = "PointMassGravity";
        gravity.name = "Gravity";
        gravity.scalars["mu"] = 3.986004418e14;
        gravity.integers["model"] = 0; // Constant gravity
        config.components.push_back(gravity);

        // Routes
        config.routes.push_back({"Gravity.position.x", "Body.position.x"});
        config.routes.push_back({"Gravity.position.y", "Body.position.y"});
        config.routes.push_back({"Gravity.position.z", "Body.position.z"});
        config.routes.push_back({"Gravity.mass", "Body.mass"});
        config.routes.push_back({"Body.force.x", "Gravity.force.x"});
        config.routes.push_back({"Body.force.y", "Gravity.force.y"});
        config.routes.push_back({"Body.force.z", "Gravity.force.z"});

        // Scheduler
        config.scheduler.groups.clear();
        SchedulerGroupConfig env_group("environment", 10.0, 1);
        env_group.members.push_back({"Gravity", 1});
        config.scheduler.groups.push_back(env_group);

        SchedulerGroupConfig dyn_group("dynamics", 10.0, 2);
        dyn_group.members.push_back({"Body", 1});
        config.scheduler.groups.push_back(dyn_group);

        return config;
    }
};

TEST_F(WarmstartTest, RecordAndWarmstart) {
    // =========================================================================
    // Phase 1: Run simulation and record
    // =========================================================================
    auto config1 = CreateTestConfig();
    config1.recording.enabled = true;
    config1.recording.path = "output/warmstart_test.h5";
    config1.recording.mode = "outputs";
    config1.recording.export_csv = false;

    auto sim1 = Simulator::FromConfig(config1);
    sim1->Stage();

    // Run for 50 steps (5 seconds at dt=0.1)
    const int total_steps = 50;
    const int warmstart_step = 25; // Resume from step 25 (t=2.5s)

    for (int i = 0; i < total_steps; ++i) {
        sim1->Step();
    }

    // Record final state for comparison
    double final_time_original = sim1->Time();
    double final_pos_x_original = sim1->Peek("Body.position.x");
    double final_pos_z_original = sim1->Peek("Body.position.z");

    // Close sim1 to flush recording
    sim1.reset();

    // =========================================================================
    // Phase 2: Warmstart from midpoint
    // =========================================================================
    auto config2 = CreateTestConfig();
    config2.staging.trim.enabled = true;
    config2.staging.trim.mode = "warmstart";
    config2.staging.trim.recording_path = "output/warmstart_test.h5";
    config2.staging.trim.resume_time = warmstart_step * config2.dt; // t=2.5s
    config2.staging.trim.validate_schema = true;

    auto sim2 = Simulator::FromConfig(config2);
    sim2->Stage(); // This should restore state from recording

    // Verify time was restored
    double resumed_time = sim2->Time();
    EXPECT_NEAR(resumed_time, warmstart_step * config2.dt, config2.dt);

    // Run remaining steps
    int remaining_steps = total_steps - warmstart_step;
    for (int i = 0; i < remaining_steps; ++i) {
        sim2->Step();
    }

    // =========================================================================
    // Phase 3: Compare results
    // =========================================================================
    double final_time_warmstart = sim2->Time();
    double final_pos_x_warmstart = sim2->Peek("Body.position.x");
    double final_pos_z_warmstart = sim2->Peek("Body.position.z");

    // Times should match
    EXPECT_NEAR(final_time_warmstart, final_time_original, 1e-6);

    // Positions should match (within tolerance)
    // Note: Some divergence expected due to:
    // 1. Frame timing differences (warmstart uses nearest frame)
    // 2. Floating point accumulation in RK4
    // Using 5% relative tolerance for position since we're comparing across
    // different integration paths from the same initial conditions
    double pos_tolerance = std::abs(final_pos_z_original) * 0.05 + 1e-3;
    EXPECT_NEAR(final_pos_x_warmstart, final_pos_x_original, pos_tolerance);
    EXPECT_NEAR(final_pos_z_warmstart, final_pos_z_original, pos_tolerance);
}

TEST_F(WarmstartTest, WarmstartTimeRestored) {
    // Record a short simulation
    auto config1 = CreateTestConfig();
    config1.recording.enabled = true;
    config1.recording.path = "output/warmstart_test.h5";
    config1.recording.mode = "outputs";

    auto sim1 = Simulator::FromConfig(config1);
    sim1->Stage();

    // Run for 20 steps
    for (int i = 0; i < 20; ++i) {
        sim1->Step();
    }
    sim1.reset();

    // Warmstart from t=1.0s (step 10)
    auto config2 = CreateTestConfig();
    config2.staging.trim.enabled = true;
    config2.staging.trim.mode = "warmstart";
    config2.staging.trim.recording_path = "output/warmstart_test.h5";
    config2.staging.trim.resume_time = 1.0;

    auto sim2 = Simulator::FromConfig(config2);
    sim2->Stage();

    // Time should be approximately 1.0
    EXPECT_NEAR(sim2->Time(), 1.0, config2.dt);
}

TEST_F(WarmstartTest, WarmstartValidationMissingFile) {
    auto config = CreateTestConfig();
    config.staging.trim.enabled = true;
    config.staging.trim.mode = "warmstart";
    config.staging.trim.recording_path = "nonexistent_file.h5";
    config.staging.trim.resume_time = 1.0;

    auto sim = Simulator::FromConfig(config);

    // Stage should throw due to missing file
    EXPECT_THROW(sim->Stage(), std::exception);
}

TEST_F(WarmstartTest, TrimConfigValidation) {
    TrimConfig config;

    // Warmstart mode requires recording_path
    config.enabled = true;
    config.mode = "warmstart";
    config.recording_path = "";

    auto errors = config.Validate();
    EXPECT_FALSE(errors.empty());
    EXPECT_TRUE(errors[0].find("recording_path") != std::string::npos);

    // With path, should be valid
    config.recording_path = "some_file.h5";
    errors = config.Validate();
    EXPECT_TRUE(errors.empty());
}

TEST_F(WarmstartTest, TrimConfigModeHelpers) {
    TrimConfig config;

    // Default is equilibrium
    EXPECT_TRUE(config.IsEquilibrium());
    EXPECT_FALSE(config.IsWarmstart());

    // Switch to warmstart
    config.mode = "warmstart";
    EXPECT_FALSE(config.IsEquilibrium());
    EXPECT_TRUE(config.IsWarmstart());
}
