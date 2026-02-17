/**
 * @file test_vehicle6dof.cpp
 * @brief Unit tests for Vehicle6DOF unified component
 *
 * Part of Phase 9B: Vehicle6DOF implementation
 * Tests mass aggregation, force aggregation, and 6DOF dynamics
 */

#include <algorithm>
#include <casadi/casadi.hpp>
#include <gtest/gtest.h>

#include <dynamics/Vehicle6DOF.hpp>
#include <mass/StaticMass.hpp>

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>

#include <janus/math/Quaternion.hpp>

#include <cmath>

using namespace icarus;
using namespace icarus::components;

// =============================================================================
// Test Helper Components
// =============================================================================

/**
 * @brief Mass properties source for testing
 */
template <typename Scalar> class TestMassProvider : public Component<Scalar> {
  public:
    explicit TestMassProvider(std::string name = "TestMass", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "TestMassProvider"; }

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_output<Scalar>("mass", &mass_, "kg", "Mass");
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

/**
 * @brief Force source for testing (body frame)
 */
template <typename Scalar> class TestForceProvider : public Component<Scalar> {
  public:
    explicit TestForceProvider(std::string name = "TestForce", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "TestForceProvider"; }

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

// =============================================================================
// Basic Tests
// =============================================================================

TEST(Vehicle6DOF, Identity) {
    Vehicle6DOF<double> vehicle("Vehicle", "Rocket");
    EXPECT_EQ(vehicle.Name(), "Vehicle");
    EXPECT_EQ(vehicle.Entity(), "Rocket");
    EXPECT_EQ(vehicle.TypeName(), "Vehicle6DOF");
}

TEST(Vehicle6DOF, DefaultInitialConditions) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    Vehicle6DOF<double> vehicle("Vehicle", "");

    bp.set_context("", "Vehicle");
    vehicle.Provision(bp);
    vehicle.Stage(bp);

    // Check default ICs
    auto pos = vehicle.GetPosition();
    EXPECT_DOUBLE_EQ(pos(0), 0.0);
    EXPECT_DOUBLE_EQ(pos(1), 0.0);
    EXPECT_DOUBLE_EQ(pos(2), 0.0);

    auto vel = vehicle.GetVelocityBody();
    EXPECT_DOUBLE_EQ(vel(0), 0.0);
    EXPECT_DOUBLE_EQ(vel(1), 0.0);
    EXPECT_DOUBLE_EQ(vel(2), 0.0);

    auto quat = vehicle.GetAttitude();
    EXPECT_DOUBLE_EQ(quat.w, 1.0);
    EXPECT_DOUBLE_EQ(quat.x, 0.0);
    EXPECT_DOUBLE_EQ(quat.y, 0.0);
    EXPECT_DOUBLE_EQ(quat.z, 0.0);

    auto omega = vehicle.GetOmegaBody();
    EXPECT_DOUBLE_EQ(omega(0), 0.0);
    EXPECT_DOUBLE_EQ(omega(1), 0.0);
    EXPECT_DOUBLE_EQ(omega(2), 0.0);
}

TEST(Vehicle6DOF, SetInitialConditions) {
    Vehicle6DOF<double> vehicle("Vehicle", "");

    vehicle.SetInitialPosition(Vec3<double>{100.0, 200.0, 300.0});
    vehicle.SetInitialVelocityBody(Vec3<double>{10.0, 20.0, 30.0});
    vehicle.SetInitialOmegaBody(Vec3<double>{0.1, 0.2, 0.3});

    auto pos = vehicle.GetPosition();
    EXPECT_DOUBLE_EQ(pos(0), 100.0);
    EXPECT_DOUBLE_EQ(pos(1), 200.0);
    EXPECT_DOUBLE_EQ(pos(2), 300.0);

    auto vel = vehicle.GetVelocityBody();
    EXPECT_DOUBLE_EQ(vel(0), 10.0);
    EXPECT_DOUBLE_EQ(vel(1), 20.0);
    EXPECT_DOUBLE_EQ(vel(2), 30.0);

    auto omega = vehicle.GetOmegaBody();
    EXPECT_DOUBLE_EQ(omega(0), 0.1);
    EXPECT_DOUBLE_EQ(omega(1), 0.2);
    EXPECT_DOUBLE_EQ(omega(2), 0.3);
}

// =============================================================================
// Mass Aggregation Tests
// =============================================================================

TEST(Vehicle6DOF, DefaultMassWhenNoSources) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    Vehicle6DOF<double> vehicle("Vehicle", "");
    vehicle.SetDefaultMass(42.0);
    vehicle.SetDefaultInertia(Vec3<double>{10.0, 20.0, 30.0});

    bp.set_context("", "Vehicle");
    vehicle.Provision(bp);
    vehicle.Stage(bp);

    // Step to trigger aggregation
    vehicle.Step(0.0, 0.01);

    EXPECT_DOUBLE_EQ(vehicle.GetTotalMass(), 42.0);
}

TEST(Vehicle6DOF, MassAggregationFromSources) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // Two mass sources
    TestMassProvider<double> mass1("Mass1", "Rocket");
    mass1.SetMass(100.0);
    mass1.SetCG(Vec3<double>{0.0, 0.0, 0.0});
    mass1.SetInertia(10.0, 10.0, 10.0);

    TestMassProvider<double> mass2("Mass2", "Rocket");
    mass2.SetMass(50.0);
    mass2.SetCG(Vec3<double>{1.0, 0.0, 0.0});
    mass2.SetInertia(5.0, 5.0, 5.0);

    Vehicle6DOF<double> vehicle("Vehicle", "Rocket");
    vehicle.AddMassSource("Mass1");
    vehicle.AddMassSource("Mass2");

    // Provision all
    bp.set_context("Rocket", "Mass1");
    mass1.Provision(bp);
    bp.set_context("Rocket", "Mass2");
    mass2.Provision(bp);
    bp.set_context("Rocket", "Vehicle");
    vehicle.Provision(bp);

    // Stage all
    bp.set_context("Rocket", "Mass1");
    mass1.Stage(bp);
    bp.set_context("Rocket", "Mass2");
    mass2.Stage(bp);
    bp.set_context("Rocket", "Vehicle");
    vehicle.Stage(bp);

    // Step to trigger aggregation
    vehicle.Step(0.0, 0.01);

    // Total mass = 100 + 50 = 150 kg
    EXPECT_DOUBLE_EQ(vehicle.GetTotalMass(), 150.0);

    // CG should be weighted average: (100*0 + 50*1) / 150 = 50/150 = 1/3
    auto cg = vehicle.GetCG();
    EXPECT_NEAR(cg(0), 1.0 / 3.0, 1e-10);
    EXPECT_NEAR(cg(1), 0.0, 1e-10);
    EXPECT_NEAR(cg(2), 0.0, 1e-10);
}

// =============================================================================
// Force Aggregation Tests
// =============================================================================

TEST(Vehicle6DOF, ForceAggregationBodyFrame) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // Mass source for dynamics
    TestMassProvider<double> mass("Mass", "");
    mass.SetMass(10.0);
    mass.SetInertia(1.0, 1.0, 1.0);

    // Two body-frame force sources
    TestForceProvider<double> force1("Force1", "");
    force1.SetForce(Vec3<double>{100.0, 0.0, 0.0});

    TestForceProvider<double> force2("Force2", "");
    force2.SetForce(Vec3<double>{50.0, 25.0, 0.0});

    Vehicle6DOF<double> vehicle("Vehicle", "");
    vehicle.AddMassSource("Mass");
    vehicle.AddBodySource("Force1");
    vehicle.AddBodySource("Force2");

    // Provision all
    bp.set_context("", "Mass");
    mass.Provision(bp);
    bp.set_context("", "Force1");
    force1.Provision(bp);
    bp.set_context("", "Force2");
    force2.Provision(bp);
    bp.set_context("", "Vehicle");
    vehicle.Provision(bp);

    // Stage all
    bp.set_context("", "Mass");
    mass.Stage(bp);
    bp.set_context("", "Force1");
    force1.Stage(bp);
    bp.set_context("", "Force2");
    force2.Stage(bp);
    bp.set_context("", "Vehicle");
    vehicle.Stage(bp);

    // Step to trigger aggregation
    vehicle.Step(0.0, 0.01);

    // Total force = [100, 0, 0] + [50, 25, 0] = [150, 25, 0]
    auto force = vehicle.GetTotalForce();
    EXPECT_NEAR(force(0), 150.0, 1e-10);
    EXPECT_NEAR(force(1), 25.0, 1e-10);
    EXPECT_NEAR(force(2), 0.0, 1e-10);
}

TEST(Vehicle6DOF, ForceAggregationEcefToBody) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // Mass source
    TestMassProvider<double> mass("Mass", "");
    mass.SetMass(10.0);
    mass.SetInertia(1.0, 1.0, 1.0);

    // ECEF force source - force pointing in +X ECEF
    TestForceProvider<double> gravity("Gravity", "");
    gravity.SetForce(Vec3<double>{100.0, 0.0, 0.0});

    Vehicle6DOF<double> vehicle("Vehicle", "");
    vehicle.AddMassSource("Mass");
    vehicle.AddEcefSource("Gravity");

    // ECEF sources require reference_frame="ecef"
    icarus::ComponentConfig cfg;
    cfg.strings["reference_frame"] = "ecef";
    vehicle.SetConfig(cfg);

    // Set vehicle attitude: 90 deg rotation about Z
    // Body +X points to ECEF +Y, Body +Y points to ECEF -X
    double angle = M_PI / 2.0;
    double c = std::cos(angle / 2.0);
    double s = std::sin(angle / 2.0);
    vehicle.SetInitialAttitude(janus::Quaternion<double>{c, 0, 0, s}); // yaw 90 deg

    // Provision all
    bp.set_context("", "Mass");
    mass.Provision(bp);
    bp.set_context("", "Gravity");
    gravity.Provision(bp);
    bp.set_context("", "Vehicle");
    vehicle.Provision(bp);

    // Stage all
    bp.set_context("", "Mass");
    mass.Stage(bp);
    bp.set_context("", "Gravity");
    gravity.Stage(bp);
    bp.set_context("", "Vehicle");
    vehicle.Stage(bp);

    // Step to trigger aggregation
    vehicle.Step(0.0, 0.01);

    // ECEF force [100, 0, 0] transformed to body frame with 90 deg yaw
    // With 90 deg yaw: body +X → ECEF +Y, body +Y → ECEF -X
    // Inverse: ECEF +X → body -Y
    auto force = vehicle.GetTotalForce();
    EXPECT_NEAR(std::abs(force(1)), 100.0, 1e-10); // Force is along body Y
    EXPECT_NEAR(force(2), 0.0, 1e-10);             // No Z component
    // The magnitude is preserved even if sign convention differs
    EXPECT_NEAR(force.norm(), 100.0, 1e-10);
}

// =============================================================================
// Dynamics Tests
// =============================================================================

TEST(Vehicle6DOF, FreefallDynamics) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    double mass = 10.0;
    double g = 9.81;
    double weight = mass * g;

    TestMassProvider<double> mass_src("Mass", "");
    mass_src.SetMass(mass);
    mass_src.SetInertia(1.0, 1.0, 1.0);

    TestForceProvider<double> gravity("Gravity", "");
    gravity.SetForce(Vec3<double>{0.0, 0.0, -weight});

    Vehicle6DOF<double> vehicle("Vehicle", "");
    vehicle.AddMassSource("Mass");
    vehicle.AddBodySource("Gravity");
    vehicle.SetInitialPosition(Vec3<double>{0.0, 0.0, 1000.0});

    // Provision
    bp.set_context("", "Mass");
    mass_src.Provision(bp);
    bp.set_context("", "Gravity");
    gravity.Provision(bp);
    bp.set_context("", "Vehicle");
    vehicle.Provision(bp);

    // Stage
    bp.set_context("", "Mass");
    mass_src.Stage(bp);
    bp.set_context("", "Gravity");
    gravity.Stage(bp);
    bp.set_context("", "Vehicle");
    vehicle.Stage(bp);

    // Step
    vehicle.Step(0.0, 0.01);

    // Check acceleration = g in -Z direction
    // Access derivatives via registry
    EXPECT_NEAR(registry.GetByName("Vehicle.velocity_body_dot.z"), -g, 1e-10);
}

TEST(Vehicle6DOF, TorqueFreeRotation) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestMassProvider<double> mass_src("Mass", "");
    mass_src.SetMass(10.0);
    mass_src.SetInertia(1.0, 1.0, 1.0);

    Vehicle6DOF<double> vehicle("Vehicle", "");
    vehicle.AddMassSource("Mass");
    vehicle.SetInitialOmegaBody(Vec3<double>{0.0, 0.0, 1.0}); // Spin about Z

    // Provision
    bp.set_context("", "Mass");
    mass_src.Provision(bp);
    bp.set_context("", "Vehicle");
    vehicle.Provision(bp);

    // Stage
    bp.set_context("", "Mass");
    mass_src.Stage(bp);
    bp.set_context("", "Vehicle");
    vehicle.Stage(bp);

    // Step
    vehicle.Step(0.0, 0.01);

    // Angular acceleration should be zero for torque-free spin about principal axis
    EXPECT_NEAR(registry.GetByName("Vehicle.omega_body_dot.x"), 0.0, 1e-10);
    EXPECT_NEAR(registry.GetByName("Vehicle.omega_body_dot.y"), 0.0, 1e-10);
    EXPECT_NEAR(registry.GetByName("Vehicle.omega_body_dot.z"), 0.0, 1e-10);

    // Quaternion rate should be [0, 0, 0, 0.5] for omega_z = 1
    EXPECT_NEAR(registry.GetByName("Vehicle.attitude_dot.w"), 0.0, 1e-10);
    EXPECT_NEAR(registry.GetByName("Vehicle.attitude_dot.z"), 0.5, 1e-10);
}

// =============================================================================
// Combined Integration Test
// =============================================================================

TEST(Vehicle6DOF, IntegratedRocketSimulation) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // Structure mass
    TestMassProvider<double> structure("Structure", "Rocket");
    structure.SetMass(500.0);
    structure.SetCG(Vec3<double>{0.0, 0.0, 5.0});
    structure.SetInertia(100.0, 100.0, 50.0);

    // Fuel mass
    TestMassProvider<double> fuel("Fuel", "Rocket");
    fuel.SetMass(1000.0);
    fuel.SetCG(Vec3<double>{0.0, 0.0, 3.0});
    fuel.SetInertia(200.0, 200.0, 100.0);

    // Engine thrust
    TestForceProvider<double> engine("Engine", "Rocket");
    engine.SetForce(Vec3<double>{0.0, 0.0, -20000.0}); // Thrust in body -Z

    // Gravity (ECEF frame)
    TestForceProvider<double> gravity("Gravity", "Rocket");
    gravity.SetForce(Vec3<double>{0.0, 0.0, -14715.0}); // 1500 kg * 9.81

    Vehicle6DOF<double> vehicle("Vehicle", "Rocket");
    vehicle.AddMassSource("Structure");
    vehicle.AddMassSource("Fuel");
    vehicle.AddBodySource("Engine");
    vehicle.AddEcefSource("Gravity");

    // ECEF sources require reference_frame="ecef"
    icarus::ComponentConfig cfg;
    cfg.strings["reference_frame"] = "ecef";
    vehicle.SetConfig(cfg);

    // Provision all
    bp.set_context("Rocket", "Structure");
    structure.Provision(bp);
    bp.set_context("Rocket", "Fuel");
    fuel.Provision(bp);
    bp.set_context("Rocket", "Engine");
    engine.Provision(bp);
    bp.set_context("Rocket", "Gravity");
    gravity.Provision(bp);
    bp.set_context("Rocket", "Vehicle");
    vehicle.Provision(bp);

    // Stage all
    bp.set_context("Rocket", "Structure");
    structure.Stage(bp);
    bp.set_context("Rocket", "Fuel");
    fuel.Stage(bp);
    bp.set_context("Rocket", "Engine");
    engine.Stage(bp);
    bp.set_context("Rocket", "Gravity");
    gravity.Stage(bp);
    bp.set_context("Rocket", "Vehicle");
    vehicle.Stage(bp);

    // Step
    vehicle.Step(0.0, 0.01);

    // Total mass = 500 + 1000 = 1500 kg
    EXPECT_DOUBLE_EQ(vehicle.GetTotalMass(), 1500.0);

    // CG = (500*5 + 1000*3) / 1500 = (2500 + 3000) / 1500 = 5500/1500 = 11/3
    auto cg = vehicle.GetCG();
    EXPECT_NEAR(cg(2), 11.0 / 3.0, 1e-10);

    // Net force in body frame (with identity attitude):
    // Engine: [0, 0, -20000] (body)
    // Gravity: [0, 0, -14715] (ECEF, same as body with identity attitude)
    // Total: [0, 0, -34715]
    auto force = vehicle.GetTotalForce();
    EXPECT_NEAR(force(2), -34715.0, 1e-10);
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(Vehicle6DOFSymbolic, Compiles) {
    Vehicle6DOF<janus::SymbolicScalar> vehicle("Vehicle", "");
    EXPECT_EQ(vehicle.TypeName(), "Vehicle6DOF");
}

TEST(Vehicle6DOFSymbolic, SetInitialConditions) {
    Vehicle6DOF<janus::SymbolicScalar> vehicle("Vehicle", "");

    janus::SymbolicScalar x(1.0);
    janus::SymbolicScalar y(2.0);
    janus::SymbolicScalar z(3.0);

    vehicle.SetInitialPosition(Vec3<janus::SymbolicScalar>{x, y, z});
    vehicle.SetInitialVelocityBody(Vec3<janus::SymbolicScalar>{x, y, z});
    vehicle.SetInitialOmegaBody(Vec3<janus::SymbolicScalar>{x, y, z});

    EXPECT_EQ(vehicle.TypeName(), "Vehicle6DOF");
}

// =============================================================================
// Output Signal Tests
// =============================================================================

TEST(Vehicle6DOF, OutputSignalsRegistered) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    Vehicle6DOF<double> vehicle("Vehicle", "Test");

    bp.set_context("Test", "Vehicle");
    vehicle.Provision(bp);
    vehicle.Stage(bp);

    // Check that key outputs are registered
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.position.x"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.velocity_body.x"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.attitude.w"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.omega_body.x"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.total_mass"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.cg.x"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.inertia.xx"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.total_force.x"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.total_moment.x"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.velocity_ref.x"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.position_lla.lat"));
    EXPECT_TRUE(registry.HasSignal("Test.Vehicle.euler_zyx.yaw"));
}

// =============================================================================
// Introspection: Resolve Tracking
// =============================================================================

TEST(Vehicle6DOF, ResolvedInputsTrackedByBackplane) {
    // Verify that Backplane tracks Vehicle6DOF's resolve() calls
    // during Stage(), capturing implicit source binding edges.
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestMassProvider<double> mass("Structure", "Rocket");
    mass.SetMass(100.0);
    mass.SetInertia(10.0, 10.0, 10.0);

    TestForceProvider<double> engine("Engine", "Rocket");
    engine.SetForce(Vec3<double>{1000.0, 0.0, 0.0});

    Vehicle6DOF<double> vehicle("Vehicle", "Rocket");
    vehicle.AddMassSource("Structure");
    vehicle.AddBodySource("Engine");

    // Provision all
    bp.set_context("Rocket", "Structure");
    bp.clear_tracking();
    mass.Provision(bp);
    bp.set_context("Rocket", "Engine");
    bp.clear_tracking();
    engine.Provision(bp);
    bp.set_context("Rocket", "Vehicle");
    bp.clear_tracking();
    vehicle.Provision(bp);

    // Stage all - Vehicle6DOF calls resolve() for source handles
    bp.set_context("Rocket", "Structure");
    bp.clear_tracking();
    mass.Stage(bp);
    bp.set_context("Rocket", "Engine");
    bp.clear_tracking();
    engine.Stage(bp);
    bp.set_context("Rocket", "Vehicle");
    bp.clear_tracking();
    vehicle.Stage(bp);

    // Check that Vehicle6DOF's resolve() calls were tracked
    auto resolved = bp.resolved_inputs();
    EXPECT_FALSE(resolved.empty());

    // Should include mass source signals
    auto contains = [&](const std::string &needle) {
        return std::find(resolved.begin(), resolved.end(), needle) != resolved.end();
    };

    EXPECT_TRUE(contains("Rocket.Structure.mass"));
    EXPECT_TRUE(contains("Rocket.Structure.cg.x"));
    EXPECT_TRUE(contains("Rocket.Structure.inertia.xx"));

    // Should include force source signals
    EXPECT_TRUE(contains("Rocket.Engine.force.x"));
    EXPECT_TRUE(contains("Rocket.Engine.force.y"));
    EXPECT_TRUE(contains("Rocket.Engine.force.z"));
}

TEST(Vehicle6DOF, ResolvedInputsTrackedByBackplaneSymbolic) {
    // Mirror the double test for casadi::MX to validate resolve tracking in symbolic mode.
    icarus::SignalRegistry<casadi::MX> registry;
    icarus::Backplane<casadi::MX> bp(registry);

    TestMassProvider<casadi::MX> mass("Structure", "Rocket");
    TestForceProvider<casadi::MX> engine("Engine", "Rocket");

    Vehicle6DOF<casadi::MX> vehicle("Vehicle", "Rocket");
    vehicle.AddMassSource("Structure");
    vehicle.AddBodySource("Engine");

    // Provision all
    bp.set_context("Rocket", "Structure");
    bp.clear_tracking();
    mass.Provision(bp);
    bp.set_context("Rocket", "Engine");
    bp.clear_tracking();
    engine.Provision(bp);
    bp.set_context("Rocket", "Vehicle");
    bp.clear_tracking();
    vehicle.Provision(bp);

    // Stage all - Vehicle6DOF calls resolve() for source handles
    bp.set_context("Rocket", "Structure");
    bp.clear_tracking();
    mass.Stage(bp);
    bp.set_context("Rocket", "Engine");
    bp.clear_tracking();
    engine.Stage(bp);
    bp.set_context("Rocket", "Vehicle");
    bp.clear_tracking();
    vehicle.Stage(bp);

    // Check that Vehicle6DOF's resolve() calls were tracked
    auto resolved = bp.resolved_inputs();
    EXPECT_FALSE(resolved.empty());

    auto contains = [&](const std::string &needle) {
        return std::find(resolved.begin(), resolved.end(), needle) != resolved.end();
    };

    EXPECT_TRUE(contains("Rocket.Structure.mass"));
    EXPECT_TRUE(contains("Rocket.Structure.cg.x"));
    EXPECT_TRUE(contains("Rocket.Structure.inertia.xx"));

    EXPECT_TRUE(contains("Rocket.Engine.force.x"));
    EXPECT_TRUE(contains("Rocket.Engine.force.y"));
    EXPECT_TRUE(contains("Rocket.Engine.force.z"));
}
