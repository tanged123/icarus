/**
 * @file test_physical_component.cpp
 * @brief Tests for PhysicalComponent<Scalar> (Phase 9A)
 *
 * Tests body attachment configuration, frame transformations,
 * and Euler angle / quaternion orientation parsing.
 */

#include <cmath>
#include <gtest/gtest.h>
#include <icarus/icarus.hpp>
#include <testing/DummyComponent.hpp>

namespace icarus {
namespace {

// Concrete test component inheriting from PhysicalComponent
template <typename Scalar> class TestPhysicalComponent : public PhysicalComponent<Scalar> {
  public:
    explicit TestPhysicalComponent(std::string name = "TestPhys", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    void Provision(Backplane<Scalar> &) override {}

    void Stage(Backplane<Scalar> &) override { this->ReadAttachmentFromConfig(); }

    void Step(Scalar, Scalar) override {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "TestPhysicalComponent"; }

    // Expose protected methods for testing
    using PhysicalComponent<Scalar>::SetBodyAttachment;
    using PhysicalComponent<Scalar>::SetBodyPosition;

  private:
    std::string name_;
    std::string entity_;
};

class PhysicalComponentTest : public ::testing::Test {
  protected:
    static constexpr double kTol = 1e-10;
};

// =============================================================================
// Default/Base Component Tests
// =============================================================================

TEST_F(PhysicalComponentTest, DefaultAttachment) {
    TestPhysicalComponent<double> comp;

    // Before any configuration, no attachment
    EXPECT_FALSE(comp.HasBodyAttachment());

    // Default position is zero
    auto pos = comp.GetBodyPosition();
    EXPECT_NEAR(pos(0), 0.0, kTol);
    EXPECT_NEAR(pos(1), 0.0, kTol);
    EXPECT_NEAR(pos(2), 0.0, kTol);

    // Default orientation is identity
    auto q = comp.GetBodyOrientation();
    EXPECT_NEAR(q.w, 1.0, kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
    EXPECT_NEAR(q.z, 0.0, kTol);
}

TEST_F(PhysicalComponentTest, BaseComponentNoAttachment) {
    // Verify base Component class returns defaults
    DummyComponent<double> base_comp("Dummy");

    EXPECT_FALSE(base_comp.HasBodyAttachment());

    auto pos = base_comp.GetBodyPosition();
    EXPECT_EQ(pos, Vec3<double>::Zero());

    auto q = base_comp.GetBodyOrientation();
    EXPECT_NEAR(q.w, 1.0, kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
}

// =============================================================================
// Position Configuration Tests
// =============================================================================

TEST_F(PhysicalComponentTest, PositionOnly) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    config.entity = "Vehicle";
    config.type = "TestPhysicalComponent";
    config.vectors["body_position"] = {1.0, 2.0, 3.0};

    TestPhysicalComponent<double> comp("Test", "Vehicle");
    comp.SetConfig(config);

    bp.set_context("Vehicle", "Test");
    comp.Stage(bp);

    EXPECT_TRUE(comp.HasBodyAttachment());

    auto pos = comp.GetBodyPosition();
    EXPECT_NEAR(pos(0), 1.0, kTol);
    EXPECT_NEAR(pos(1), 2.0, kTol);
    EXPECT_NEAR(pos(2), 3.0, kTol);

    // Orientation should still be identity
    auto q = comp.GetBodyOrientation();
    EXPECT_NEAR(q.w, 1.0, kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
    EXPECT_NEAR(q.z, 0.0, kTol);
}

// =============================================================================
// Quaternion Orientation Tests
// =============================================================================

TEST_F(PhysicalComponentTest, QuaternionOrientation) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    config.entity = "Vehicle";
    config.vectors["body_position"] = {0.0, 0.0, 10.0};

    // 90 deg rotation about Y-axis
    double angle = M_PI / 2.0;
    config.vectors["body_orientation"] = {std::cos(angle / 2), 0.0, std::sin(angle / 2), 0.0};

    TestPhysicalComponent<double> comp("Test", "Vehicle");
    comp.SetConfig(config);

    bp.set_context("Vehicle", "Test");
    comp.Stage(bp);

    EXPECT_TRUE(comp.HasBodyAttachment());

    auto q = comp.GetBodyOrientation();
    EXPECT_NEAR(q.w, std::cos(M_PI / 4), kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, std::sin(M_PI / 4), kTol);
    EXPECT_NEAR(q.z, 0.0, kTol);
}

TEST_F(PhysicalComponentTest, QuaternionNormalization) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    // Non-unit quaternion (will be normalized)
    config.vectors["body_orientation"] = {2.0, 0.0, 0.0, 0.0};

    TestPhysicalComponent<double> comp("Test");
    comp.SetConfig(config);

    bp.set_context("", "Test");
    comp.Stage(bp);

    auto q = comp.GetBodyOrientation();
    // Should be normalized to identity
    EXPECT_NEAR(q.w, 1.0, kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
    EXPECT_NEAR(q.z, 0.0, kTol);
}

// =============================================================================
// Euler Angle Orientation Tests
// =============================================================================

TEST_F(PhysicalComponentTest, EulerOrientationPitchMinus90) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    config.vectors["body_position"] = {0.0, 0.0, 10.0};
    // pitch -90 deg: +X_local -> -Z_body
    config.vectors["body_orientation_euler_zyx"] = {0.0, -90.0, 0.0};

    TestPhysicalComponent<double> comp("Test");
    comp.SetConfig(config);

    bp.set_context("", "Test");
    comp.Stage(bp);

    EXPECT_TRUE(comp.HasBodyAttachment());

    // A -90 deg pitch about Y should rotate +X to -Z in body frame
    Vec3<double> local_x{1.0, 0.0, 0.0};
    Vec3<double> body_vec = comp.TransformToBodyFrame(local_x);

    EXPECT_NEAR(body_vec(0), 0.0, kTol);
    EXPECT_NEAR(body_vec(1), 0.0, kTol);
    EXPECT_NEAR(body_vec(2), -1.0, kTol);
}

TEST_F(PhysicalComponentTest, EulerOrientationYaw90) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    // yaw 90 deg: +X_body -> +Y_local
    config.vectors["body_orientation_euler_zyx"] = {90.0, 0.0, 0.0};

    TestPhysicalComponent<double> comp("Test");
    comp.SetConfig(config);

    bp.set_context("", "Test");
    comp.Stage(bp);

    // Yaw 90 deg means body X axis aligns with component local Y
    Vec3<double> body_x{1.0, 0.0, 0.0};
    Vec3<double> local = comp.TransformToLocalFrame(body_x);

    EXPECT_NEAR(local(0), 0.0, kTol);
    EXPECT_NEAR(local(1), 1.0, kTol);
    EXPECT_NEAR(local(2), 0.0, kTol);

    // Round-trip back to body frame
    Vec3<double> back = comp.TransformToBodyFrame(local);
    EXPECT_NEAR(back(0), 1.0, kTol);
    EXPECT_NEAR(back(1), 0.0, kTol);
    EXPECT_NEAR(back(2), 0.0, kTol);
}

TEST_F(PhysicalComponentTest, EulerOrientationRoll45) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    // roll 45 deg
    config.vectors["body_orientation_euler_zyx"] = {0.0, 0.0, 45.0};

    TestPhysicalComponent<double> comp("Test");
    comp.SetConfig(config);

    bp.set_context("", "Test");
    comp.Stage(bp);

    // Roll 45 deg about X: body Y -> component local (Y rotated toward Z)
    Vec3<double> body_y{0.0, 1.0, 0.0};
    Vec3<double> local = comp.TransformToLocalFrame(body_y);

    double c45 = std::cos(M_PI / 4.0);
    double s45 = std::sin(M_PI / 4.0);

    EXPECT_NEAR(local(0), 0.0, kTol);
    EXPECT_NEAR(local(1), c45, kTol);
    EXPECT_NEAR(local(2), s45, kTol);
}

// =============================================================================
// Euler Precedence Test
// =============================================================================

TEST_F(PhysicalComponentTest, EulerTakesPrecedence) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    // Both specified - Euler should win
    config.vectors["body_orientation"] = {1.0, 0.0, 0.0, 0.0};       // identity
    config.vectors["body_orientation_euler_zyx"] = {90.0, 0.0, 0.0}; // yaw 90 deg

    TestPhysicalComponent<double> comp("Test");
    comp.SetConfig(config);

    bp.set_context("", "Test");
    comp.Stage(bp);

    // Should be 90 deg yaw, not identity
    auto q = comp.GetBodyOrientation();
    EXPECT_NEAR(q.w, std::cos(M_PI / 4), kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
    EXPECT_NEAR(q.z, std::sin(M_PI / 4), kTol);
}

// =============================================================================
// Transform Helper Tests
// =============================================================================

TEST_F(PhysicalComponentTest, GetComponentToBodyRotation) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    config.vectors["body_orientation_euler_zyx"] = {90.0, 0.0, 0.0}; // yaw 90 deg

    TestPhysicalComponent<double> comp("Test");
    comp.SetConfig(config);

    bp.set_context("", "Test");
    comp.Stage(bp);

    auto q_body_to_comp = comp.GetBodyOrientation();
    auto q_comp_to_body = comp.GetComponentToBodyRotation();

    // Should be conjugate
    EXPECT_NEAR(q_comp_to_body.w, q_body_to_comp.w, kTol);
    EXPECT_NEAR(q_comp_to_body.x, -q_body_to_comp.x, kTol);
    EXPECT_NEAR(q_comp_to_body.y, -q_body_to_comp.y, kTol);
    EXPECT_NEAR(q_comp_to_body.z, -q_body_to_comp.z, kTol);
}

TEST_F(PhysicalComponentTest, TransformRoundTrip) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    ComponentConfig config;
    config.name = "Test";
    // Combined rotation: yaw 30, pitch 20, roll 10
    config.vectors["body_orientation_euler_zyx"] = {30.0, 20.0, 10.0};

    TestPhysicalComponent<double> comp("Test");
    comp.SetConfig(config);

    bp.set_context("", "Test");
    comp.Stage(bp);

    // Arbitrary vector
    Vec3<double> v_body{1.5, -2.3, 4.7};

    // Round trip: body -> local -> body
    Vec3<double> v_local = comp.TransformToLocalFrame(v_body);
    Vec3<double> v_back = comp.TransformToBodyFrame(v_local);

    EXPECT_NEAR(v_back(0), v_body(0), kTol);
    EXPECT_NEAR(v_back(1), v_body(1), kTol);
    EXPECT_NEAR(v_back(2), v_body(2), kTol);
}

// =============================================================================
// Programmatic Attachment Tests
// =============================================================================

TEST_F(PhysicalComponentTest, ProgrammaticAttachment) {
    TestPhysicalComponent<double> comp("Test");

    EXPECT_FALSE(comp.HasBodyAttachment());

    Vec3<double> pos{5.0, 0.0, 0.0};
    auto q = janus::Quaternion<double>::from_axis_angle({0, 1, 0}, M_PI / 4);

    comp.SetBodyAttachment(pos, q);

    EXPECT_TRUE(comp.HasBodyAttachment());
    EXPECT_EQ(comp.GetBodyPosition(), pos);

    auto q_out = comp.GetBodyOrientation();
    EXPECT_NEAR(q_out.w, q.w, kTol);
    EXPECT_NEAR(q_out.x, q.x, kTol);
    EXPECT_NEAR(q_out.y, q.y, kTol);
    EXPECT_NEAR(q_out.z, q.z, kTol);
}

TEST_F(PhysicalComponentTest, SetBodyPositionOnly) {
    TestPhysicalComponent<double> comp("Test");

    EXPECT_FALSE(comp.HasBodyAttachment());

    Vec3<double> pos{1.0, 2.0, 3.0};
    comp.SetBodyPosition(pos);

    EXPECT_TRUE(comp.HasBodyAttachment());
    EXPECT_EQ(comp.GetBodyPosition(), pos);

    // Orientation should still be identity
    auto q = comp.GetBodyOrientation();
    EXPECT_NEAR(q.w, 1.0, kTol);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
    EXPECT_NEAR(q.z, 0.0, kTol);
}

// =============================================================================
// Symbolic Compatibility Test
// =============================================================================

TEST_F(PhysicalComponentTest, SymbolicCompatibility) {
    // This test ensures the template compiles with casadi::MX
    TestPhysicalComponent<SymbolicScalar> comp("SymTest");

    EXPECT_FALSE(comp.HasBodyAttachment());

    auto q = comp.GetBodyOrientation();
    // Just verify it compiles and returns something
    EXPECT_EQ(q.w.rows(), 1);
}

// =============================================================================
// Dynamic Orientation Tests (Thrust Vectoring)
// =============================================================================

/**
 * @brief A gimballed force component that updates orientation dynamically.
 *
 * Demonstrates thrust vectoring pattern:
 * - Base mount orientation set from config
 * - Gimbal pitch/yaw angles read as input signals
 * - Total orientation = base_mount * gimbal_rotation
 * - Force output in local frame (+X direction)
 */
template <typename Scalar> class GimballedForceComponent : public PhysicalComponent<Scalar> {
  public:
    explicit GimballedForceComponent(std::string name = "GimbalForce", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    void Provision(Backplane<Scalar> &bp) override {
        // Outputs: force in local frame
        bp.register_output_vec3("force_local", &force_local_, "N", "Force in local frame");
        // Also output force in body frame for verification
        bp.register_output_vec3("force_body", &force_body_, "N", "Force in body frame");
    }

    void Stage(Backplane<Scalar> &bp) override {
        // Read base mount orientation from config
        this->ReadAttachmentFromConfig();
        base_mount_orientation_ = this->body_orientation_;

        // Read thrust magnitude from config
        thrust_magnitude_ = this->read_param("thrust", 1000.0);

        // Wire gimbal angle inputs (optional - default to 0)
        if (bp.has_signal("gimbal.pitch_deg")) {
            gimbal_pitch_handle_ = bp.template resolve<Scalar>("gimbal.pitch_deg");
        }
        if (bp.has_signal("gimbal.yaw_deg")) {
            gimbal_yaw_handle_ = bp.template resolve<Scalar>("gimbal.yaw_deg");
        }

        // Initialize force
        force_local_ = Vec3<Scalar>{thrust_magnitude_, Scalar{0}, Scalar{0}};
        force_body_ = this->TransformToBodyFrame(force_local_);
    }

    void Step(Scalar /*t*/, Scalar /*dt*/) override {
        // Read gimbal angles (degrees)
        Scalar pitch_deg = gimbal_pitch_handle_.valid() ? *gimbal_pitch_handle_ : Scalar{0};
        Scalar yaw_deg = gimbal_yaw_handle_.valid() ? *gimbal_yaw_handle_ : Scalar{0};

        // Convert to radians
        constexpr double deg2rad = M_PI / 180.0;
        Scalar pitch_rad = pitch_deg * Scalar{deg2rad};
        Scalar yaw_rad = yaw_deg * Scalar{deg2rad};

        // Compute gimbal rotation (pitch then yaw at the gimbal)
        // Gimbal rotates the nozzle: pitch about Y, yaw about Z
        auto gimbal_rotation = janus::Quaternion<Scalar>::from_euler(Scalar{0}, pitch_rad, yaw_rad);

        // Total orientation = gimbal * base_mount (quaternion convention: right-to-left)
        // This composes: first base_mount (body->gimbal), then gimbal (gimbal->nozzle)
        // Result: body -> nozzle orientation
        this->body_orientation_ = gimbal_rotation * base_mount_orientation_;

        // Force is always along local +X
        force_local_ = Vec3<Scalar>{thrust_magnitude_, Scalar{0}, Scalar{0}};

        // Transform to body frame using current orientation
        force_body_ = this->TransformToBodyFrame(force_local_);
    }

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "GimballedForceComponent"; }

    // Accessors for testing
    [[nodiscard]] Vec3<Scalar> GetForceLocal() const { return force_local_; }
    [[nodiscard]] Vec3<Scalar> GetForceBody() const { return force_body_; }
    [[nodiscard]] Scalar GetThrustMagnitude() const { return thrust_magnitude_; }

  private:
    std::string name_;
    std::string entity_;

    // Base mount orientation (from config, doesn't change)
    janus::Quaternion<Scalar> base_mount_orientation_;

    // Thrust magnitude
    Scalar thrust_magnitude_{1000.0};

    // Gimbal input handles
    SignalHandle<Scalar> gimbal_pitch_handle_;
    SignalHandle<Scalar> gimbal_yaw_handle_;

    // Output signals
    Vec3<Scalar> force_local_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> force_body_ = Vec3<Scalar>::Zero();
};

TEST_F(PhysicalComponentTest, DynamicOrientationUpdate) {
    // Test that orientation can be updated dynamically in Step()
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    // Create gimbal command signals
    double gimbal_pitch = 0.0;
    double gimbal_yaw = 0.0;
    registry.register_output("gimbal.pitch_deg", &gimbal_pitch);
    registry.register_output("gimbal.yaw_deg", &gimbal_yaw);

    // Configure gimballed force component
    // Base mount: pitch -90 deg (thrust along -Z when gimbal is zero)
    ComponentConfig config;
    config.name = "Engine";
    config.entity = "Vehicle";
    config.vectors["body_position"] = {0.0, 0.0, 5.0};
    config.vectors["body_orientation_euler_zyx"] = {0.0, -90.0, 0.0}; // +X_local -> -Z_body
    config.scalars["thrust"] = 1000.0;

    GimballedForceComponent<double> engine("Engine", "Vehicle");
    engine.SetConfig(config);

    bp.set_context("Vehicle", "Engine");
    engine.Provision(bp);
    engine.Stage(bp);

    // Initial state: gimbal at zero
    engine.Step(0.0, 0.01);

    // Force should be along -Z body (base mount pitch -90)
    auto force0 = engine.GetForceBody();
    EXPECT_NEAR(force0(0), 0.0, kTol);
    EXPECT_NEAR(force0(1), 0.0, kTol);
    EXPECT_NEAR(force0(2), -1000.0, kTol);

    // Now gimbal pitch +10 deg (nozzle pitches, thrust vector changes)
    gimbal_pitch = 10.0;
    engine.Step(0.01, 0.01);

    auto force1 = engine.GetForceBody();
    // With gimbal pitch, thrust gets a +X component
    // The force should no longer be purely -Z
    EXPECT_GT(std::abs(force1(0)), 100.0); // Significant X component
    EXPECT_NEAR(force1(1), 0.0, kTol);     // No Y component
    // Z component reduced but still dominant
    EXPECT_LT(force1(2), -900.0);

    // Verify force magnitude is preserved
    double mag1 = std::sqrt(force1(0) * force1(0) + force1(1) * force1(1) + force1(2) * force1(2));
    EXPECT_NEAR(mag1, 1000.0, kTol);

    // Gimbal yaw +15 deg
    gimbal_pitch = 0.0;
    gimbal_yaw = 15.0;
    engine.Step(0.02, 0.01);

    auto force2 = engine.GetForceBody();
    // With gimbal yaw, thrust gets a Y component
    EXPECT_NEAR(force2(0), 0.0, 1.0);      // Small X
    EXPECT_GT(std::abs(force2(1)), 200.0); // Significant Y component
    EXPECT_LT(force2(2), -900.0);          // Still mostly -Z

    // Verify magnitude preserved
    double mag2 = std::sqrt(force2(0) * force2(0) + force2(1) * force2(1) + force2(2) * force2(2));
    EXPECT_NEAR(mag2, 1000.0, kTol);
}

TEST_F(PhysicalComponentTest, DynamicOrientationWithCombinedGimbal) {
    // Test combined pitch and yaw gimbal
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double gimbal_pitch = 0.0;
    double gimbal_yaw = 0.0;
    registry.register_output("gimbal.pitch_deg", &gimbal_pitch);
    registry.register_output("gimbal.yaw_deg", &gimbal_yaw);

    // Engine mounted pointing straight down (-Z)
    ComponentConfig config;
    config.name = "Engine";
    config.vectors["body_position"] = {0.0, 0.0, 5.0};
    config.vectors["body_orientation_euler_zyx"] = {0.0, -90.0, 0.0};
    config.scalars["thrust"] = 1000.0;

    GimballedForceComponent<double> engine("Engine");
    engine.SetConfig(config);

    bp.set_context("", "Engine");
    engine.Provision(bp);
    engine.Stage(bp);

    // Combined gimbal: pitch 30, yaw 45
    gimbal_pitch = 30.0;
    gimbal_yaw = 45.0;
    engine.Step(0.0, 0.01);

    auto force = engine.GetForceBody();

    // Verify magnitude preserved (this is the key invariant)
    double mag = std::sqrt(force(0) * force(0) + force(1) * force(1) + force(2) * force(2));
    EXPECT_NEAR(mag, 1000.0, kTol);

    // With significant pitch and yaw, all three components should be non-trivial
    EXPECT_GT(std::abs(force(0)), 100.0);
    EXPECT_GT(std::abs(force(1)), 100.0);
    EXPECT_GT(std::abs(force(2)), 500.0); // Z still dominant
}

TEST_F(PhysicalComponentTest, GimbalReturnToZero) {
    // Test that returning gimbal to zero restores original thrust direction
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double gimbal_pitch = 0.0;
    double gimbal_yaw = 0.0;
    registry.register_output("gimbal.pitch_deg", &gimbal_pitch);
    registry.register_output("gimbal.yaw_deg", &gimbal_yaw);

    ComponentConfig config;
    config.name = "Engine";
    config.vectors["body_orientation_euler_zyx"] = {0.0, -90.0, 0.0};
    config.scalars["thrust"] = 1000.0;

    GimballedForceComponent<double> engine("Engine");
    engine.SetConfig(config);

    bp.set_context("", "Engine");
    engine.Provision(bp);
    engine.Stage(bp);

    // Initial force
    engine.Step(0.0, 0.01);
    auto force_initial = engine.GetForceBody();

    // Deflect gimbal
    gimbal_pitch = 20.0;
    gimbal_yaw = 30.0;
    engine.Step(0.01, 0.01);
    auto force_deflected = engine.GetForceBody();

    // Return to zero
    gimbal_pitch = 0.0;
    gimbal_yaw = 0.0;
    engine.Step(0.02, 0.01);
    auto force_returned = engine.GetForceBody();

    // Force should match initial exactly
    EXPECT_NEAR(force_returned(0), force_initial(0), kTol);
    EXPECT_NEAR(force_returned(1), force_initial(1), kTol);
    EXPECT_NEAR(force_returned(2), force_initial(2), kTol);

    // And should be different from deflected
    EXPECT_GT(std::abs(force_deflected(0) - force_initial(0)), 100.0);
}

} // namespace
} // namespace icarus
