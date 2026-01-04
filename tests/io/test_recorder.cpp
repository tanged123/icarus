/**
 * @file test_recorder.cpp
 * @brief Tests for HDF5Recorder wrapping Vulcan's telemetry system
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <icarus/io/HDF5Recorder.hpp>
#include <icarus/io/RecordingReader.hpp>
#include <icarus/sim/SimulatorConfig.hpp>

namespace fs = std::filesystem;
using namespace icarus;

class RecorderTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Clean up any previous test files
        if (fs::exists(test_file_)) {
            fs::remove(test_file_);
        }
    }

    void TearDown() override {
        // Clean up test file
        if (fs::exists(test_file_)) {
            fs::remove(test_file_);
        }
    }

    std::string test_file_ = "/tmp/test_recording.h5";
};

TEST_F(RecorderTest, RecordAndReadBasicSignals) {
    // Create a registry with some test signals
    SignalRegistry<double> registry;

    // Component-owned storage
    double position_x = 0.0, position_y = 0.0, position_z = 0.0;
    double velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;
    int32_t phase = 0;

    registry.set_current_component("TestComponent");
    registry.register_output("position.x", &position_x, "m");
    registry.register_output("position.y", &position_y, "m");
    registry.register_output("position.z", &position_z, "m");
    registry.register_output("velocity.x", &velocity_x, "m/s");
    registry.register_output("velocity.y", &velocity_y, "m/s");
    registry.register_output("velocity.z", &velocity_z, "m/s");
    registry.register_output("phase", &phase);
    registry.clear_current_component();

    // Create recorder and open file
    RecordingConfig config;
    config.enabled = true;
    config.path = test_file_;
    config.mode = "outputs";

    HDF5Recorder recorder(registry, config);
    recorder.Open("");

    EXPECT_EQ(recorder.RecordedSignals().size(), 7);

    // Record 100 frames
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.01;

        // Update signal values
        position_x = 1000.0 + t * 100.0;
        position_y = 2000.0 + t * 50.0;
        position_z = 3000.0 + t * 25.0;
        velocity_x = 100.0;
        velocity_y = 50.0;
        velocity_z = 25.0;
        phase = (t < 0.5) ? 1 : 2;

        recorder.Record(t);
    }

    EXPECT_EQ(recorder.FrameCount(), 100);
    recorder.Close();

    // Read back and verify
    RecordingReader reader(test_file_);

    EXPECT_EQ(reader.frame_count(), 100);

    auto times = reader.times();
    EXPECT_EQ(times.size(), 100);
    EXPECT_NEAR(times[0], 0.0, 1e-9);
    EXPECT_NEAR(times[99], 0.99, 1e-9);

    auto pos_x = reader.read_double("position.x");
    EXPECT_EQ(pos_x.size(), 100);
    EXPECT_NEAR(pos_x[0], 1000.0, 1e-9);
    EXPECT_NEAR(pos_x[50], 1000.0 + 0.5 * 100.0, 1e-9);

    auto phases = reader.read_int32("phase");
    EXPECT_EQ(phases.size(), 100);
    EXPECT_EQ(phases[0], 1);
    EXPECT_EQ(phases[99], 2);
}

TEST_F(RecorderTest, PatternFiltering) {
    SignalRegistry<double> registry;

    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    double vel_x = 0.0, vel_y = 0.0, vel_z = 0.0;
    double internal = 0.0;

    registry.set_current_component("Satellite");
    registry.register_output("Satellite.position.x", &pos_x, "m");
    registry.register_output("Satellite.position.y", &pos_y, "m");
    registry.register_output("Satellite.position.z", &pos_z, "m");
    registry.register_output("Satellite.velocity.x", &vel_x, "m/s");
    registry.register_output("Satellite.velocity.y", &vel_y, "m/s");
    registry.register_output("Satellite.velocity.z", &vel_z, "m/s");
    registry.register_output("Satellite.internal", &internal);
    registry.clear_current_component();

    // Only record position signals
    RecordingConfig config;
    config.enabled = true;
    config.path = test_file_;
    config.mode = "signals";
    config.include = {"position"};

    HDF5Recorder recorder(registry, config);
    recorder.Open("");

    // Should only have position signals
    EXPECT_EQ(recorder.RecordedSignals().size(), 3);

    pos_x = 100.0;
    pos_y = 200.0;
    pos_z = 300.0;
    vel_x = 10.0;
    recorder.Record(0.0);
    recorder.Close();

    // Verify only position was recorded
    RecordingReader reader(test_file_);
    auto names = reader.signal_names();

    bool has_position_x =
        std::find(names.begin(), names.end(), "Satellite.position.x") != names.end();
    bool has_velocity_x =
        std::find(names.begin(), names.end(), "Satellite.velocity.x") != names.end();

    EXPECT_TRUE(has_position_x);
    EXPECT_FALSE(has_velocity_x);
}

TEST_F(RecorderTest, ExcludePatterns) {
    SignalRegistry<double> registry;

    double pos_x = 0.0, pos_dot_x = 0.0;

    registry.set_current_component("Test");
    registry.register_output("pos.x", &pos_x, "m");
    registry.register_output("pos_dot.x", &pos_dot_x, "m/s"); // derivative
    registry.clear_current_component();

    // Exclude derivatives by default
    RecordingConfig config;
    config.enabled = true;
    config.path = test_file_;
    config.mode = "outputs";
    config.include_derivatives = false;

    HDF5Recorder recorder(registry, config);
    recorder.Open("");

    // Should exclude _dot signals
    auto recorded = recorder.RecordedSignals();
    EXPECT_EQ(recorded.size(), 1);
    EXPECT_EQ(recorded[0], "pos.x");

    recorder.Record(0.0);
    recorder.Close();
}

TEST_F(RecorderTest, Vec3Signals) {
    SignalRegistry<double> registry;

    Vec3<double> position = Vec3<double>::Zero();
    Vec3<double> velocity = Vec3<double>::Zero();

    registry.set_current_component("Body");
    registry.register_output_vec3("Body.position", &position, "m");
    registry.register_output_vec3("Body.velocity", &velocity, "m/s");
    registry.clear_current_component();

    RecordingConfig config;
    config.enabled = true;
    config.path = test_file_;
    config.mode = "outputs";

    HDF5Recorder recorder(registry, config);
    recorder.Open("");

    // Record a few frames
    for (int i = 0; i < 10; ++i) {
        position = Vec3<double>(i * 100.0, i * 200.0, i * 300.0);
        velocity = Vec3<double>(100.0, 200.0, 300.0);
        recorder.Record(i * 0.1);
    }
    recorder.Close();

    // Read back as vec3
    RecordingReader reader(test_file_);
    auto positions = reader.read_vec3("Body.position");

    EXPECT_EQ(positions.size(), 10);
    EXPECT_NEAR(positions[5].x(), 500.0, 1e-9);
    EXPECT_NEAR(positions[5].y(), 1000.0, 1e-9);
    EXPECT_NEAR(positions[5].z(), 1500.0, 1e-9);
}
