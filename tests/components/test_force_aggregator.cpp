/**
 * @file test_force_aggregator.cpp
 * @brief Unit tests for ForceAggregator component
 *
 * Part of Phase 4: Aggregation & 6DOF
 */

#include <gtest/gtest.h>

#include <aggregators/ForceAggregator.hpp>
#include <aggregators/MassAggregator.hpp>
#include <mass/StaticMass.hpp>

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/SignalRouter.hpp>

#include <cmath>

using namespace icarus;
using namespace icarus::components;

// =============================================================================
// Test Force Source Component
// =============================================================================

/**
 * @brief Simple force source for testing
 *
 * Publishes configurable force, moment, and application point signals.
 */
template <typename Scalar> class TestForceSource : public Component<Scalar> {
  public:
    explicit TestForceSource(std::string name = "TestForce", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "TestForceSource"; }
    [[nodiscard]] std::size_t StateSize() const override { return 0; }

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_output_vec3<Scalar>("force", &force_, "N", "Force vector");
        if (publish_moment_) {
            bp.template register_output_vec3<Scalar>("moment", &moment_, "N*m", "Moment vector");
        }
        if (publish_app_point_) {
            bp.template register_output_vec3<Scalar>("application_point", &app_point_, "m",
                                                     "Application point");
        }
    }

    void Stage(Backplane<Scalar> &) override {}
    void Step(Scalar, Scalar) override {}

    // Configuration
    void SetForce(const Vec3<Scalar> &f) { force_ = f; }
    void SetMoment(const Vec3<Scalar> &m) {
        moment_ = m;
        publish_moment_ = true;
    }
    void SetApplicationPoint(const Vec3<Scalar> &p) {
        app_point_ = p;
        publish_app_point_ = true;
    }

  private:
    std::string name_;
    std::string entity_;
    Vec3<Scalar> force_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> moment_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> app_point_ = Vec3<Scalar>::Zero();
    bool publish_moment_ = false;
    bool publish_app_point_ = false;
};

/**
 * @brief CG source for testing (provides cg.x, cg.y, cg.z)
 */
template <typename Scalar> class TestCGSource : public Component<Scalar> {
  public:
    explicit TestCGSource(std::string name = "CGSource", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "TestCGSource"; }
    [[nodiscard]] std::size_t StateSize() const override { return 0; }

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_output_vec3<Scalar>("cg", &cg_, "m", "CG position");
    }

    void Stage(Backplane<Scalar> &) override {}
    void Step(Scalar, Scalar) override {}

    void SetCG(const Vec3<Scalar> &cg) { cg_ = cg; }

  private:
    std::string name_;
    std::string entity_;
    Vec3<Scalar> cg_ = Vec3<Scalar>::Zero();
};

// =============================================================================
// Helper to wire CG inputs
// =============================================================================

void WireCGInputs(icarus::SignalRegistry<double> &registry, const std::string &agg_name,
                  const std::string &cg_source_name) {
    registry.template wire_input<double>(agg_name + ".cg.x", cg_source_name + ".cg.x");
    registry.template wire_input<double>(agg_name + ".cg.y", cg_source_name + ".cg.y");
    registry.template wire_input<double>(agg_name + ".cg.z", cg_source_name + ".cg.z");
}

// =============================================================================
// ForceAggregator Unit Tests
// =============================================================================

TEST(ForceAggregator, StateSize) {
    ForceAggregator<double> agg;
    EXPECT_EQ(agg.StateSize(), 0);
    EXPECT_FALSE(agg.HasState());
}

TEST(ForceAggregator, Identity) {
    ForceAggregator<double> agg("ForceAgg", "Vehicle");
    EXPECT_EQ(agg.Name(), "ForceAgg");
    EXPECT_EQ(agg.Entity(), "Vehicle");
    EXPECT_EQ(agg.TypeName(), "ForceAggregator");
}

TEST(ForceAggregator, EmptySourcesReturnsZero) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // CG source
    TestCGSource<double> cg_src("CGSource", "");
    cg_src.SetCG(Vec3<double>{0.0, 0.0, 0.0});

    bp.set_context("", "CGSource");
    cg_src.Provision(bp);

    // Aggregator with no sources
    ForceAggregator<double> agg("ForceAgg", "");

    bp.set_context("", "ForceAgg");
    agg.Provision(bp);

    // Wire CG inputs
    WireCGInputs(registry, "ForceAgg", "CGSource");

    agg.Stage(bp);
    agg.Step(0.0, 0.01);

    EXPECT_DOUBLE_EQ(agg.GetTotalForce()(0), 0.0);
    EXPECT_DOUBLE_EQ(agg.GetTotalForce()(1), 0.0);
    EXPECT_DOUBLE_EQ(agg.GetTotalForce()(2), 0.0);
    EXPECT_DOUBLE_EQ(agg.GetTotalMoment()(0), 0.0);
    EXPECT_DOUBLE_EQ(agg.GetTotalMoment()(1), 0.0);
    EXPECT_DOUBLE_EQ(agg.GetTotalMoment()(2), 0.0);
}

TEST(ForceAggregator, SingleForceAtCG) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // CG at origin
    TestCGSource<double> cg_src("CGSource", "");
    cg_src.SetCG(Vec3<double>{0.0, 0.0, 0.0});

    // Force source at CG (no application point offset)
    TestForceSource<double> force1("Force1", "");
    force1.SetForce(Vec3<double>{100.0, 0.0, -50.0});

    // Aggregator
    ForceAggregator<double> agg("ForceAgg", "");
    agg.AddSource("Force1");

    // Provision all
    bp.set_context("", "CGSource");
    cg_src.Provision(bp);

    bp.set_context("", "Force1");
    force1.Provision(bp);

    bp.set_context("", "ForceAgg");
    agg.Provision(bp);

    // Wire CG inputs
    WireCGInputs(registry, "ForceAgg", "CGSource");

    // Stage all
    bp.set_context("", "CGSource");
    cg_src.Stage(bp);

    bp.set_context("", "Force1");
    force1.Stage(bp);

    bp.set_context("", "ForceAgg");
    agg.Stage(bp);

    // Step
    agg.Step(0.0, 0.01);

    // Force should pass through
    auto total_force = agg.GetTotalForce();
    EXPECT_DOUBLE_EQ(total_force(0), 100.0);
    EXPECT_DOUBLE_EQ(total_force(1), 0.0);
    EXPECT_DOUBLE_EQ(total_force(2), -50.0);

    // No moment (force at CG)
    auto total_moment = agg.GetTotalMoment();
    EXPECT_NEAR(total_moment(0), 0.0, 1e-10);
    EXPECT_NEAR(total_moment(1), 0.0, 1e-10);
    EXPECT_NEAR(total_moment(2), 0.0, 1e-10);
}

TEST(ForceAggregator, TwoForcesSummed) {
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestCGSource<double> cg_src("CGSource", "");
    cg_src.SetCG(Vec3<double>::Zero());

    TestForceSource<double> force1("Force1", "");
    force1.SetForce(Vec3<double>{100.0, 0.0, 0.0});

    TestForceSource<double> force2("Force2", "");
    force2.SetForce(Vec3<double>{50.0, 25.0, -10.0});

    ForceAggregator<double> agg("ForceAgg", "");
    agg.AddSource("Force1");
    agg.AddSource("Force2");

    // Provision
    bp.set_context("", "CGSource");
    cg_src.Provision(bp);
    bp.set_context("", "Force1");
    force1.Provision(bp);
    bp.set_context("", "Force2");
    force2.Provision(bp);
    bp.set_context("", "ForceAgg");
    agg.Provision(bp);

    // Wire CG inputs
    WireCGInputs(registry, "ForceAgg", "CGSource");

    // Stage
    bp.set_context("", "CGSource");
    cg_src.Stage(bp);
    bp.set_context("", "Force1");
    force1.Stage(bp);
    bp.set_context("", "Force2");
    force2.Stage(bp);
    bp.set_context("", "ForceAgg");
    agg.Stage(bp);

    agg.Step(0.0, 0.01);

    auto total_force = agg.GetTotalForce();
    EXPECT_DOUBLE_EQ(total_force(0), 150.0); // 100 + 50
    EXPECT_DOUBLE_EQ(total_force(1), 25.0);  // 0 + 25
    EXPECT_DOUBLE_EQ(total_force(2), -10.0); // 0 + -10
}

TEST(ForceAggregator, MomentTransferFromOffset) {
    // Force applied at offset from CG creates a moment
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // CG at origin
    TestCGSource<double> cg_src("CGSource", "");
    cg_src.SetCG(Vec3<double>::Zero());

    // Force applied at offset position
    // Force = [0, 0, -100] N at application point [1, 0, 0] m
    // Moment = r × F = [1,0,0] × [0,0,-100] = [0, 100, 0]
    TestForceSource<double> force1("Force1", "");
    force1.SetForce(Vec3<double>{0.0, 0.0, -100.0});
    force1.SetApplicationPoint(Vec3<double>{1.0, 0.0, 0.0});

    ForceAggregator<double> agg("ForceAgg", "");
    agg.AddSource("Force1");

    bp.set_context("", "CGSource");
    cg_src.Provision(bp);
    bp.set_context("", "Force1");
    force1.Provision(bp);
    bp.set_context("", "ForceAgg");
    agg.Provision(bp);

    // Wire CG inputs
    WireCGInputs(registry, "ForceAgg", "CGSource");

    bp.set_context("", "CGSource");
    cg_src.Stage(bp);
    bp.set_context("", "Force1");
    force1.Stage(bp);
    bp.set_context("", "ForceAgg");
    agg.Stage(bp);

    agg.Step(0.0, 0.01);

    auto total_force = agg.GetTotalForce();
    EXPECT_DOUBLE_EQ(total_force(0), 0.0);
    EXPECT_DOUBLE_EQ(total_force(1), 0.0);
    EXPECT_DOUBLE_EQ(total_force(2), -100.0);

    // r × F = [1,0,0] × [0,0,-100] = [0*(-100) - 0*0, 0*0 - 1*(-100), 1*0 - 0*0]
    //       = [0, 100, 0]
    auto total_moment = agg.GetTotalMoment();
    EXPECT_NEAR(total_moment(0), 0.0, 1e-10);
    EXPECT_NEAR(total_moment(1), 100.0, 1e-10);
    EXPECT_NEAR(total_moment(2), 0.0, 1e-10);
}

TEST(ForceAggregator, MomentTransferWithCGOffset) {
    // CG not at origin - moment transfer uses (app_point - cg)
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    // CG at [0.5, 0, 0]
    TestCGSource<double> cg_src("CGSource", "");
    cg_src.SetCG(Vec3<double>{0.5, 0.0, 0.0});

    // Force at [1, 0, 0], so r = [1,0,0] - [0.5,0,0] = [0.5, 0, 0]
    // Force = [0, 0, -100]
    // Moment = [0.5,0,0] × [0,0,-100] = [0, 50, 0]
    TestForceSource<double> force1("Force1", "");
    force1.SetForce(Vec3<double>{0.0, 0.0, -100.0});
    force1.SetApplicationPoint(Vec3<double>{1.0, 0.0, 0.0});

    ForceAggregator<double> agg("ForceAgg", "");
    agg.AddSource("Force1");

    bp.set_context("", "CGSource");
    cg_src.Provision(bp);
    bp.set_context("", "Force1");
    force1.Provision(bp);
    bp.set_context("", "ForceAgg");
    agg.Provision(bp);

    // Wire CG inputs
    WireCGInputs(registry, "ForceAgg", "CGSource");

    bp.set_context("", "CGSource");
    cg_src.Stage(bp);
    bp.set_context("", "Force1");
    force1.Stage(bp);
    bp.set_context("", "ForceAgg");
    agg.Stage(bp);

    agg.Step(0.0, 0.01);

    auto total_moment = agg.GetTotalMoment();
    EXPECT_NEAR(total_moment(0), 0.0, 1e-10);
    EXPECT_NEAR(total_moment(1), 50.0, 1e-10); // Half the moment due to shorter arm
    EXPECT_NEAR(total_moment(2), 0.0, 1e-10);
}

TEST(ForceAggregator, ExplicitMomentAdded) {
    // Source provides explicit moment (e.g., reaction wheel or aero moment)
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestCGSource<double> cg_src("CGSource", "");
    cg_src.SetCG(Vec3<double>::Zero());

    TestForceSource<double> force1("Force1", "");
    force1.SetForce(Vec3<double>{10.0, 0.0, 0.0});
    force1.SetMoment(Vec3<double>{0.0, 0.0, 5.0}); // Pure torque

    ForceAggregator<double> agg("ForceAgg", "");
    agg.AddSource("Force1");

    bp.set_context("", "CGSource");
    cg_src.Provision(bp);
    bp.set_context("", "Force1");
    force1.Provision(bp);
    bp.set_context("", "ForceAgg");
    agg.Provision(bp);

    // Wire CG inputs
    WireCGInputs(registry, "ForceAgg", "CGSource");

    bp.set_context("", "CGSource");
    cg_src.Stage(bp);
    bp.set_context("", "Force1");
    force1.Stage(bp);
    bp.set_context("", "ForceAgg");
    agg.Stage(bp);

    agg.Step(0.0, 0.01);

    auto total_moment = agg.GetTotalMoment();
    EXPECT_NEAR(total_moment(0), 0.0, 1e-10);
    EXPECT_NEAR(total_moment(1), 0.0, 1e-10);
    EXPECT_NEAR(total_moment(2), 5.0, 1e-10); // Just the explicit moment
}

TEST(ForceAggregator, CombinedMomentAndTransfer) {
    // Source provides both explicit moment AND application point
    icarus::SignalRegistry<double> registry;
    icarus::Backplane<double> bp(registry);

    TestCGSource<double> cg_src("CGSource", "");
    cg_src.SetCG(Vec3<double>::Zero());

    // Force at [1,0,0], creates moment [0,100,0] from transfer
    // Plus explicit moment [0,0,20]
    TestForceSource<double> force1("Force1", "");
    force1.SetForce(Vec3<double>{0.0, 0.0, -100.0});
    force1.SetMoment(Vec3<double>{0.0, 0.0, 20.0});
    force1.SetApplicationPoint(Vec3<double>{1.0, 0.0, 0.0});

    ForceAggregator<double> agg("ForceAgg", "");
    agg.AddSource("Force1");

    bp.set_context("", "CGSource");
    cg_src.Provision(bp);
    bp.set_context("", "Force1");
    force1.Provision(bp);
    bp.set_context("", "ForceAgg");
    agg.Provision(bp);

    // Wire CG inputs
    WireCGInputs(registry, "ForceAgg", "CGSource");

    bp.set_context("", "CGSource");
    cg_src.Stage(bp);
    bp.set_context("", "Force1");
    force1.Stage(bp);
    bp.set_context("", "ForceAgg");
    agg.Stage(bp);

    agg.Step(0.0, 0.01);

    auto total_moment = agg.GetTotalMoment();
    EXPECT_NEAR(total_moment(0), 0.0, 1e-10);
    EXPECT_NEAR(total_moment(1), 100.0, 1e-10); // From transfer
    EXPECT_NEAR(total_moment(2), 20.0, 1e-10);  // From explicit moment
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(ForceAggregatorSymbolic, BasicCompiles) {
    ForceAggregator<janus::SymbolicScalar> agg;
    EXPECT_EQ(agg.StateSize(), 0);
}
