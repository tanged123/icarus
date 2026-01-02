/**
 * @file test_rigid_body_6dof.cpp
 * @brief Unit tests for RigidBody6DOF component
 *
 * Part of Phase 4.4: RigidBody6DOF component
 */

#include <gtest/gtest.h>

#include <aggregators/ForceAggregator.hpp>
#include <aggregators/MassAggregator.hpp>
#include <dynamics/RigidBody6DOF.hpp>
#include <mass/StaticMass.hpp>

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/SignalRouter.hpp>

#include <janus/math/Quaternion.hpp>

#include <cmath>

using namespace icarus;
using namespace icarus::components;

// =============================================================================
// Test Helper Components
// =============================================================================

/**
 * @brief Simple force/moment source for testing
 */
template <typename Scalar> class TestForceMomentSource : public Component<Scalar> {
  public:
    explicit TestForceMomentSource(std::string name = "TestForce", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "TestForceMomentSource"; }
    [[nodiscard]] std::size_t StateSize() const override { return 0; }

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_output_vec3<Scalar>("force", &force_, "N", "Force vector");
        bp.template register_output_vec3<Scalar>("moment", &moment_, "N*m", "Moment vector");
    }

    void Stage(Backplane<Scalar> &) override {}
    void Step(Scalar, Scalar) override {}

    void SetForce(const Vec3<Scalar> &f) { force_ = f; }
    void SetMoment(const Vec3<Scalar> &m) { moment_ = m; }

  private:
    std::string name_;
    std::string entity_;
    Vec3<Scalar> force_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> moment_ = Vec3<Scalar>::Zero();
};

/**
 * @brief Mass properties source for testing
 */
template <typename Scalar> class TestMassSource : public Component<Scalar> {
  public:
    explicit TestMassSource(std::string name = "TestMass", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "TestMassSource"; }
    [[nodiscard]] std::size_t StateSize() const override { return 0; }

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_output<Scalar>("total_mass", &mass_, "kg", "Total mass");
        bp.template register_output_vec3<Scalar>("cg", &cg_, "m", "CG position");
        bp.template register_output<Scalar>("inertia.xx", &Ixx_, "kg*m^2", "Ixx");
        bp.template register_output<Scalar>("inertia.yy", &Iyy_, "kg*m^2", "Iyy");
        bp.template register_output<Scalar>("inertia.zz", &Izz_, "kg*m^2", "Izz");
        bp.template register_output<Scalar>("inertia.xy", &Ixy_, "kg*m^2", "Ixy");
        bp.template register_output<Scalar>("inertia.xz", &Ixz_, "kg*m^2", "Ixz");
        bp.template register_output<Scalar>("inertia.yz", &Iyz_, "kg*m^2", "Iyz");
    }

    void Stage(Backplane<Scalar> &) override {}
    void Step(Scalar, Scalar) override {}

    void SetMass(Scalar m) { mass_ = m; }
    void SetCG(const Vec3<Scalar> &cg) { cg_ = cg; }
    void SetInertia(Scalar Ixx, Scalar Iyy, Scalar Izz, Scalar Ixy = Scalar(0),
                    Scalar Ixz = Scalar(0), Scalar Iyz = Scalar(0)) {
        Ixx_ = Ixx;
        Iyy_ = Iyy;
        Izz_ = Izz;
        Ixy_ = Ixy;
        Ixz_ = Ixz;
        Iyz_ = Iyz;
    }

  private:
    std::string name_;
    std::string entity_;
    Scalar mass_{1.0};
    Vec3<Scalar> cg_ = Vec3<Scalar>::Zero();
    Scalar Ixx_{1.0}, Iyy_{1.0}, Izz_{1.0};
    Scalar Ixy_{0.0}, Ixz_{0.0}, Iyz_{0.0};
};

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Wire RigidBody6DOF inputs to mass/force sources
 */
void WireRigidBodyInputs(icarus::SignalRegistry<double> &registry, const std::string &rb_name,
                         const std::string &mass_src, const std::string &force_src) {
    // Mass inputs
    registry.template wire_input<double>(rb_name + ".total_mass", mass_src + ".total_mass");
    registry.template wire_input<double>(rb_name + ".inertia.xx", mass_src + ".inertia.xx");
    registry.template wire_input<double>(rb_name + ".inertia.yy", mass_src + ".inertia.yy");
    registry.template wire_input<double>(rb_name + ".inertia.zz", mass_src + ".inertia.zz");
    registry.template wire_input<double>(rb_name + ".inertia.xy", mass_src + ".inertia.xy");
    registry.template wire_input<double>(rb_name + ".inertia.xz", mass_src + ".inertia.xz");
    registry.template wire_input<double>(rb_name + ".inertia.yz", mass_src + ".inertia.yz");

    // Force/moment inputs
    registry.template wire_input<double>(rb_name + ".total_force.x", force_src + ".force.x");
    registry.template wire_input<double>(rb_name + ".total_force.y", force_src + ".force.y");
    registry.template wire_input<double>(rb_name + ".total_force.z", force_src + ".force.z");
    registry.template wire_input<double>(rb_name + ".total_moment.x", force_src + ".moment.x");
    registry.template wire_input<double>(rb_name + ".total_moment.y", force_src + ".moment.y");
    registry.template wire_input<double>(rb_name + ".total_moment.z", force_src + ".moment.z");
}

// =============================================================================
// Basic Unit Tests
// =============================================================================

TEST(RigidBody6DOF, StateSize) {
    RigidBody6DOF<double> rb;
    EXPECT_EQ(rb.StateSize(), 13);
    EXPECT_TRUE(rb.HasState());
}

TEST(RigidBody6DOF, Identity) {
    RigidBody6DOF<double> rb("RB", "Vehicle");
    EXPECT_EQ(rb.Name(), "RB");
    EXPECT_EQ(rb.Entity(), "Vehicle");
    EXPECT_EQ(rb.TypeName(), "RigidBody6DOF");
}

TEST(RigidBody6DOF, StateLayout) {
    EXPECT_EQ(RigidBody6DOF<double>::kPosOffset, 0);
    EXPECT_EQ(RigidBody6DOF<double>::kVelOffset, 3);
    EXPECT_EQ(RigidBody6DOF<double>::kQuatOffset, 6);
    EXPECT_EQ(RigidBody6DOF<double>::kOmegaOffset, 10);
}

TEST(RigidBody6DOF, InitialConditionsDefault) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestMassSource<double> mass_src("MassSrc", "");
    mass_src.SetMass(10.0);
    mass_src.SetInertia(1.0, 1.0, 1.0);

    TestForceMomentSource<double> force_src("ForceSrc", "");

    RigidBody6DOF<double> rb("RB", "");

    // Provision
    bp.set_context("", "MassSrc");
    mass_src.Provision(bp);
    bp.set_context("", "ForceSrc");
    force_src.Provision(bp);
    bp.set_context("", "RB");
    rb.Provision(bp);

    // Wire inputs
    WireRigidBodyInputs(registry, "RB", "MassSrc", "ForceSrc");

    // Create state vectors
    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);

    // Bind and stage
    rb.BindState(state.data(), state_dot.data(), 13);
    bp.set_context("", "RB");
    rb.Stage(bp);

    // Check default ICs: identity quaternion, everything else zero
    EXPECT_DOUBLE_EQ(state[0], 0.0);  // pos.x
    EXPECT_DOUBLE_EQ(state[1], 0.0);  // pos.y
    EXPECT_DOUBLE_EQ(state[2], 0.0);  // pos.z
    EXPECT_DOUBLE_EQ(state[3], 0.0);  // vel.x
    EXPECT_DOUBLE_EQ(state[4], 0.0);  // vel.y
    EXPECT_DOUBLE_EQ(state[5], 0.0);  // vel.z
    EXPECT_DOUBLE_EQ(state[6], 1.0);  // quat.w (identity)
    EXPECT_DOUBLE_EQ(state[7], 0.0);  // quat.x
    EXPECT_DOUBLE_EQ(state[8], 0.0);  // quat.y
    EXPECT_DOUBLE_EQ(state[9], 0.0);  // quat.z
    EXPECT_DOUBLE_EQ(state[10], 0.0); // omega.x
    EXPECT_DOUBLE_EQ(state[11], 0.0); // omega.y
    EXPECT_DOUBLE_EQ(state[12], 0.0); // omega.z
}

TEST(RigidBody6DOF, SetInitialConditions) {
    RigidBody6DOF<double> rb("RB", "");

    rb.SetInitialPosition(100.0, 200.0, 300.0);
    rb.SetInitialVelocityBody(10.0, 20.0, 30.0);
    rb.SetInitialAttitude(0.5, 0.5, 0.5, 0.5); // 90 deg rotation
    rb.SetInitialOmegaBody(0.1, 0.2, 0.3);

    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);
    rb.BindState(state.data(), state_dot.data(), 13);

    EXPECT_DOUBLE_EQ(state[0], 100.0);
    EXPECT_DOUBLE_EQ(state[1], 200.0);
    EXPECT_DOUBLE_EQ(state[2], 300.0);
    EXPECT_DOUBLE_EQ(state[3], 10.0);
    EXPECT_DOUBLE_EQ(state[4], 20.0);
    EXPECT_DOUBLE_EQ(state[5], 30.0);
    EXPECT_DOUBLE_EQ(state[6], 0.5);
    EXPECT_DOUBLE_EQ(state[7], 0.5);
    EXPECT_DOUBLE_EQ(state[8], 0.5);
    EXPECT_DOUBLE_EQ(state[9], 0.5);
    EXPECT_DOUBLE_EQ(state[10], 0.1);
    EXPECT_DOUBLE_EQ(state[11], 0.2);
    EXPECT_DOUBLE_EQ(state[12], 0.3);
}

// =============================================================================
// Dynamics Tests
// =============================================================================

TEST(RigidBody6DOF, FreefallTranslation) {
    // Rigid body in freefall: constant downward force, no rotation
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    double mass = 10.0; // kg
    double g = 9.81;    // m/s^2
    double weight = mass * g;

    TestMassSource<double> mass_src("MassSrc", "");
    mass_src.SetMass(mass);
    mass_src.SetInertia(1.0, 1.0, 1.0);

    TestForceMomentSource<double> force_src("ForceSrc", "");
    force_src.SetForce(Vec3<double>{0.0, 0.0, -weight}); // Gravity in -Z

    RigidBody6DOF<double> rb("RB", "");
    rb.SetInitialPosition(0.0, 0.0, 1000.0);

    // Provision
    bp.set_context("", "MassSrc");
    mass_src.Provision(bp);
    bp.set_context("", "ForceSrc");
    force_src.Provision(bp);
    bp.set_context("", "RB");
    rb.Provision(bp);

    // Wire inputs
    WireRigidBodyInputs(registry, "RB", "MassSrc", "ForceSrc");

    // Create state vectors
    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);

    rb.BindState(state.data(), state_dot.data(), 13);

    bp.set_context("", "MassSrc");
    mass_src.Stage(bp);
    bp.set_context("", "ForceSrc");
    force_src.Stage(bp);
    bp.set_context("", "RB");
    rb.Stage(bp);

    // Step to get derivatives
    rb.Step(0.0, 0.01);

    // Position derivatives = velocity (should be zero initially)
    EXPECT_NEAR(state_dot[0], 0.0, 1e-10); // dx/dt = 0
    EXPECT_NEAR(state_dot[1], 0.0, 1e-10); // dy/dt = 0
    EXPECT_NEAR(state_dot[2], 0.0, 1e-10); // dz/dt = 0

    // Velocity derivatives = acceleration = F/m - omega x v = -g (no rotation)
    // Since velocity is zero and omega is zero, transport term is zero
    EXPECT_NEAR(state_dot[3], 0.0, 1e-10); // dv_x/dt = 0
    EXPECT_NEAR(state_dot[4], 0.0, 1e-10); // dv_y/dt = 0
    EXPECT_NEAR(state_dot[5], -g, 1e-10);  // dv_z/dt = -g

    // Quaternion derivatives = 0 (no angular velocity)
    EXPECT_NEAR(state_dot[6], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[7], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[8], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[9], 0.0, 1e-10);

    // Angular velocity derivatives = 0 (no moment, no omega)
    EXPECT_NEAR(state_dot[10], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[11], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[12], 0.0, 1e-10);
}

TEST(RigidBody6DOF, TorqueFreeRotation) {
    // Torque-free rotation about principal axis - angular velocity should be constant
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestMassSource<double> mass_src("MassSrc", "");
    mass_src.SetMass(10.0);
    mass_src.SetInertia(1.0, 1.0, 1.0); // Symmetric inertia

    TestForceMomentSource<double> force_src("ForceSrc", "");
    // No force, no moment

    RigidBody6DOF<double> rb("RB", "");
    rb.SetInitialOmegaBody(0.0, 0.0, 1.0); // Spin about Z-axis

    // Provision
    bp.set_context("", "MassSrc");
    mass_src.Provision(bp);
    bp.set_context("", "ForceSrc");
    force_src.Provision(bp);
    bp.set_context("", "RB");
    rb.Provision(bp);

    // Wire inputs
    WireRigidBodyInputs(registry, "RB", "MassSrc", "ForceSrc");

    // Create state vectors
    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);

    rb.BindState(state.data(), state_dot.data(), 13);

    bp.set_context("", "MassSrc");
    mass_src.Stage(bp);
    bp.set_context("", "ForceSrc");
    force_src.Stage(bp);
    bp.set_context("", "RB");
    rb.Stage(bp);

    // Step
    rb.Step(0.0, 0.01);

    // Angular acceleration should be zero (torque-free, spinning about principal axis)
    EXPECT_NEAR(state_dot[10], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[11], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[12], 0.0, 1e-10);

    // Quaternion should be changing due to omega_z = 1 rad/s
    // For identity quaternion and omega = [0,0,1]:
    // q_dot = 0.5 * q * (0, omega) = 0.5 * (1,0,0,0) * (0,0,0,1)
    //       = 0.5 * (0, 0, 0, 1) = (0, 0, 0, 0.5)
    EXPECT_NEAR(state_dot[6], 0.0, 1e-10); // dw/dt = 0
    EXPECT_NEAR(state_dot[7], 0.0, 1e-10); // dx/dt = 0
    EXPECT_NEAR(state_dot[8], 0.0, 1e-10); // dy/dt = 0
    EXPECT_NEAR(state_dot[9], 0.5, 1e-10); // dz/dt = 0.5
}

TEST(RigidBody6DOF, ConstantTorque) {
    // Apply constant torque, should see angular acceleration
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    double I = 2.0;  // kg*m^2
    double M = 10.0; // N*m

    TestMassSource<double> mass_src("MassSrc", "");
    mass_src.SetMass(10.0);
    mass_src.SetInertia(I, I, I);

    TestForceMomentSource<double> force_src("ForceSrc", "");
    force_src.SetMoment(Vec3<double>{M, 0.0, 0.0}); // Torque about X-axis

    RigidBody6DOF<double> rb("RB", "");

    // Provision
    bp.set_context("", "MassSrc");
    mass_src.Provision(bp);
    bp.set_context("", "ForceSrc");
    force_src.Provision(bp);
    bp.set_context("", "RB");
    rb.Provision(bp);

    // Wire inputs
    WireRigidBodyInputs(registry, "RB", "MassSrc", "ForceSrc");

    // Create state vectors
    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);

    rb.BindState(state.data(), state_dot.data(), 13);

    bp.set_context("", "MassSrc");
    mass_src.Stage(bp);
    bp.set_context("", "ForceSrc");
    force_src.Stage(bp);
    bp.set_context("", "RB");
    rb.Stage(bp);

    // Step
    rb.Step(0.0, 0.01);

    // Angular acceleration = M/I = 10/2 = 5 rad/s^2 about X
    EXPECT_NEAR(state_dot[10], M / I, 1e-10); // d(omega_x)/dt = 5
    EXPECT_NEAR(state_dot[11], 0.0, 1e-10);
    EXPECT_NEAR(state_dot[12], 0.0, 1e-10);
}

TEST(RigidBody6DOF, BodyVelocityWithRotation) {
    // Body moving in body +X at 10 m/s, rotated 90 deg about Z
    // Reference frame velocity should be in +Y
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestMassSource<double> mass_src("MassSrc", "");
    mass_src.SetMass(10.0);
    mass_src.SetInertia(1.0, 1.0, 1.0);

    TestForceMomentSource<double> force_src("ForceSrc", "");

    RigidBody6DOF<double> rb("RB", "");
    rb.SetInitialVelocityBody(10.0, 0.0, 0.0); // 10 m/s in body +X

    // 90 degree rotation about Z: quat = (cos(45), 0, 0, sin(45))
    double angle = M_PI / 2.0;
    double c = std::cos(angle / 2.0);
    double s = std::sin(angle / 2.0);
    rb.SetInitialAttitude(c, 0.0, 0.0, s);

    // Provision
    bp.set_context("", "MassSrc");
    mass_src.Provision(bp);
    bp.set_context("", "ForceSrc");
    force_src.Provision(bp);
    bp.set_context("", "RB");
    rb.Provision(bp);

    // Wire inputs
    WireRigidBodyInputs(registry, "RB", "MassSrc", "ForceSrc");

    // Create state vectors
    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);

    rb.BindState(state.data(), state_dot.data(), 13);

    bp.set_context("", "MassSrc");
    mass_src.Stage(bp);
    bp.set_context("", "ForceSrc");
    force_src.Stage(bp);
    bp.set_context("", "RB");
    rb.Stage(bp);

    // Step
    rb.Step(0.0, 0.01);

    // Position derivatives = R * v_body
    // With 90 deg rotation about Z, body +X maps to reference +Y
    EXPECT_NEAR(state_dot[0], 0.0, 1e-10);  // dx/dt ~ 0
    EXPECT_NEAR(state_dot[1], 10.0, 1e-10); // dy/dt ~ 10
    EXPECT_NEAR(state_dot[2], 0.0, 1e-10);  // dz/dt ~ 0
}

// =============================================================================
// Angular Momentum Conservation Test
// =============================================================================

TEST(RigidBody6DOF, AngularMomentumConservation) {
    // Asymmetric tumbling body with no external torque
    // Angular momentum should be conserved
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // Asymmetric inertia tensor (like a brick)
    double Ix = 1.0, Iy = 2.0, Iz = 3.0;

    TestMassSource<double> mass_src("MassSrc", "");
    mass_src.SetMass(10.0);
    mass_src.SetInertia(Ix, Iy, Iz);

    TestForceMomentSource<double> force_src("ForceSrc", "");
    // No torque

    RigidBody6DOF<double> rb("RB", "");
    // Initial spin with components in all axes
    Vec3<double> omega0{0.5, 0.3, 0.1};
    rb.SetInitialOmegaBody(omega0(0), omega0(1), omega0(2));

    // Provision
    bp.set_context("", "MassSrc");
    mass_src.Provision(bp);
    bp.set_context("", "ForceSrc");
    force_src.Provision(bp);
    bp.set_context("", "RB");
    rb.Provision(bp);

    // Wire inputs
    WireRigidBodyInputs(registry, "RB", "MassSrc", "ForceSrc");

    // Create state vectors
    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);

    rb.BindState(state.data(), state_dot.data(), 13);

    bp.set_context("", "MassSrc");
    mass_src.Stage(bp);
    bp.set_context("", "ForceSrc");
    force_src.Stage(bp);
    bp.set_context("", "RB");
    rb.Stage(bp);

    // Calculate initial angular momentum in body frame
    Mat3<double> I;
    I << Ix, 0, 0, 0, Iy, 0, 0, 0, Iz;
    Vec3<double> H0 = I * omega0;
    double H0_mag = H0.norm();

    // Integrate for a while using simple Euler (just for magnitude check)
    double dt = 0.0001; // Small timestep for accuracy
    double t = 0.0;
    int steps = 10000;

    for (int i = 0; i < steps; ++i) {
        rb.Step(t, dt);

        // Simple Euler integration
        for (size_t j = 0; j < 13; ++j) {
            state[j] += state_dot[j] * dt;
        }

        t += dt;
    }

    // Get final omega
    Vec3<double> omega_final{state[10], state[11], state[12]};
    Vec3<double> H_final = I * omega_final;
    double H_final_mag = H_final.norm();

    // Angular momentum magnitude should be conserved (within numerical tolerance)
    // Using larger tolerance due to Euler integration
    EXPECT_NEAR(H_final_mag, H0_mag, 0.01); // 1% tolerance for Euler
}

// =============================================================================
// Output Signal Tests
// =============================================================================

TEST(RigidBody6DOF, OutputsUpdated) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestMassSource<double> mass_src("MassSrc", "");
    mass_src.SetMass(10.0);
    mass_src.SetInertia(1.0, 1.0, 1.0);

    TestForceMomentSource<double> force_src("ForceSrc", "");

    RigidBody6DOF<double> rb("RB", "");
    rb.SetInitialPosition(100.0, 200.0, 300.0);
    rb.SetInitialVelocityBody(1.0, 2.0, 3.0);
    rb.SetInitialOmegaBody(0.1, 0.2, 0.3);

    // Provision
    bp.set_context("", "MassSrc");
    mass_src.Provision(bp);
    bp.set_context("", "ForceSrc");
    force_src.Provision(bp);
    bp.set_context("", "RB");
    rb.Provision(bp);

    // Wire inputs
    WireRigidBodyInputs(registry, "RB", "MassSrc", "ForceSrc");

    // Create state vectors
    std::vector<double> state(13, 0.0);
    std::vector<double> state_dot(13, 0.0);

    rb.BindState(state.data(), state_dot.data(), 13);

    bp.set_context("", "MassSrc");
    mass_src.Stage(bp);
    bp.set_context("", "ForceSrc");
    force_src.Stage(bp);
    bp.set_context("", "RB");
    rb.Stage(bp);

    // PreStep should publish outputs
    rb.PreStep(0.0, 0.01);

    // Check accessor outputs
    auto pos = rb.GetPosition();
    EXPECT_DOUBLE_EQ(pos(0), 100.0);
    EXPECT_DOUBLE_EQ(pos(1), 200.0);
    EXPECT_DOUBLE_EQ(pos(2), 300.0);

    auto vel = rb.GetVelocityBody();
    EXPECT_DOUBLE_EQ(vel(0), 1.0);
    EXPECT_DOUBLE_EQ(vel(1), 2.0);
    EXPECT_DOUBLE_EQ(vel(2), 3.0);

    auto omega = rb.GetOmegaBody();
    EXPECT_DOUBLE_EQ(omega(0), 0.1);
    EXPECT_DOUBLE_EQ(omega(1), 0.2);
    EXPECT_DOUBLE_EQ(omega(2), 0.3);

    auto quat = rb.GetAttitude();
    EXPECT_DOUBLE_EQ(quat.w, 1.0); // Identity
    EXPECT_DOUBLE_EQ(quat.x, 0.0);
    EXPECT_DOUBLE_EQ(quat.y, 0.0);
    EXPECT_DOUBLE_EQ(quat.z, 0.0);
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(RigidBody6DOFSymbolic, BasicCompiles) {
    RigidBody6DOF<janus::SymbolicScalar> rb;
    EXPECT_EQ(rb.StateSize(), 13);
}

TEST(RigidBody6DOFSymbolic, SetInitialConditions) {
    RigidBody6DOF<janus::SymbolicScalar> rb("RB", "");

    // Should compile with symbolic scalars
    janus::SymbolicScalar x = janus::SymbolicScalar(1.0);
    janus::SymbolicScalar y = janus::SymbolicScalar(2.0);
    janus::SymbolicScalar z = janus::SymbolicScalar(3.0);

    rb.SetInitialPosition(x, y, z);
    rb.SetInitialVelocityBody(x, y, z);
    rb.SetInitialOmegaBody(x, y, z);

    EXPECT_EQ(rb.StateSize(), 13);
}
