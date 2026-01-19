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

} // namespace
} // namespace icarus
