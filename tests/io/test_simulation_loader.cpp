/**
 * @file test_simulation_loader.cpp
 * @brief Unit tests for SimulationLoader YAML parsing
 *
 * Part of Phase 4.0.7: Configuration Infrastructure
 */

#include <gtest/gtest.h>
#include <icarus/io/SimulationLoader.hpp>

namespace icarus::io {
namespace {

// =============================================================================
// Parse Tests (from string)
// =============================================================================

TEST(SimulationLoaderTest, ParseMinimalConfig) {
    const char *yaml = R"(
components:
  - type: TestComponent
    name: Test
)";
    auto cfg = SimulationLoader::Parse(yaml);
    EXPECT_EQ(cfg.components.size(), 1u);
    EXPECT_EQ(cfg.components[0].type, "TestComponent");
    EXPECT_EQ(cfg.components[0].name, "Test");
}

TEST(SimulationLoaderTest, ParseSimulationSection) {
    const char *yaml = R"(
simulation:
  name: "My Simulation"
  version: "2.0.0"
  description: "Test description"
components:
  - type: Test
    name: T1
)";
    auto cfg = SimulationLoader::Parse(yaml);
    EXPECT_EQ(cfg.name, "My Simulation");
    EXPECT_EQ(cfg.version, "2.0.0");
    EXPECT_EQ(cfg.description, "Test description");
}

TEST(SimulationLoaderTest, ParseTimeSection) {
    const char *yaml = R"(
time:
  start: 10.0
  end: 200.0
  dt: 0.001
  reference_epoch_jd: 2451550.0
components:
  - type: Test
    name: T1
)";
    auto cfg = SimulationLoader::Parse(yaml);
    EXPECT_DOUBLE_EQ(cfg.t_start, 10.0);
    EXPECT_DOUBLE_EQ(cfg.t_end, 200.0);
    EXPECT_DOUBLE_EQ(cfg.dt, 0.001);
    EXPECT_DOUBLE_EQ(cfg.reference_epoch_jd, 2451550.0);
}

TEST(SimulationLoaderTest, ParseComponentWithScalars) {
    const char *yaml = R"(
components:
  - type: PointMassGravity
    name: Gravity
    scalars:
      mu: 3.986e14
      radius: 6.371e6
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.components.size(), 1u);
    EXPECT_DOUBLE_EQ(cfg.components[0].scalars.at("mu"), 3.986e14);
    EXPECT_DOUBLE_EQ(cfg.components[0].scalars.at("radius"), 6.371e6);
}

TEST(SimulationLoaderTest, ParseComponentWithVectors) {
    const char *yaml = R"(
components:
  - type: RigidBody
    name: Body
    vectors:
      position: [1.0, 2.0, 3.0]
      velocity: [4.0, 5.0, 6.0]
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.components.size(), 1u);
    auto pos = cfg.components[0].vectors.at("position");
    ASSERT_EQ(pos.size(), 3u);
    EXPECT_DOUBLE_EQ(pos[0], 1.0);
    EXPECT_DOUBLE_EQ(pos[1], 2.0);
    EXPECT_DOUBLE_EQ(pos[2], 3.0);
}

TEST(SimulationLoaderTest, ParseRoutes) {
    const char *yaml = R"(
components:
  - type: Test
    name: T1
routes:
  - input: A.signal
    output: B.signal
    gain: 2.0
    offset: 1.5
    delay: 0.01
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.routes.size(), 1u);
    EXPECT_EQ(cfg.routes[0].input_path, "A.signal");
    EXPECT_EQ(cfg.routes[0].output_path, "B.signal");
    EXPECT_DOUBLE_EQ(cfg.routes[0].gain, 2.0);
    EXPECT_DOUBLE_EQ(cfg.routes[0].offset, 1.5);
    EXPECT_DOUBLE_EQ(cfg.routes[0].delay, 0.01);
}

TEST(SimulationLoaderTest, ParseSchedulerGroups) {
    const char *yaml = R"(
components:
  - type: Test
    name: T1
scheduler:
  groups:
    - name: fast
      rate_hz: 1000
      priority: 1
      members:
        - component: A
          priority: 1
        - component: B
          priority: 2
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.scheduler.groups.size(), 1u);
    EXPECT_EQ(cfg.scheduler.groups[0].name, "fast");
    EXPECT_DOUBLE_EQ(cfg.scheduler.groups[0].rate_hz, 1000.0);
    ASSERT_EQ(cfg.scheduler.groups[0].members.size(), 2u);
    EXPECT_EQ(cfg.scheduler.groups[0].members[0].component, "A");
    EXPECT_EQ(cfg.scheduler.groups[0].members[1].component, "B");
}

TEST(SimulationLoaderTest, ParseIntegrator) {
    const char *yaml = R"(
components:
  - type: Test
    name: T1
integrator:
  type: RK45
  abs_tol: 1e-8
  rel_tol: 1e-6
)";
    auto cfg = SimulationLoader::Parse(yaml);
    EXPECT_EQ(cfg.integrator.type, IntegratorType::RK45);
    EXPECT_DOUBLE_EQ(cfg.integrator.abs_tol, 1e-8);
    EXPECT_DOUBLE_EQ(cfg.integrator.rel_tol, 1e-6);
}

// =============================================================================
// Error Tests
// =============================================================================

TEST(SimulationLoaderTest, ThrowsOnMissingComponentsAndEntities) {
    const char *yaml = R"(
simulation:
  name: Empty
)";
    EXPECT_THROW(SimulationLoader::Parse(yaml), ConfigError);
}

TEST(SimulationLoaderTest, ThrowsOnMissingRequiredComponentType) {
    const char *yaml = R"(
components:
  - name: NoType
)";
    EXPECT_THROW(SimulationLoader::Parse(yaml), std::exception);
}

// =============================================================================
// Entity Tests
// =============================================================================

TEST(SimulationLoaderTest, ParseInlineEntity) {
    const char *yaml = R"(
entities:
  - name: Leader
    entity:
      name: Rocket
      components:
        - type: PointMass
          name: EOM
          scalars:
            mass: 1000.0
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.components.size(), 1u);
    // Component should have entity prefix
    EXPECT_EQ(cfg.components[0].entity, "Leader");
    EXPECT_EQ(cfg.components[0].type, "PointMass");
    EXPECT_EQ(cfg.components[0].name, "EOM");
}

TEST(SimulationLoaderTest, EntityRoutesExpandedWithPrefix) {
    const char *yaml = R"(
entities:
  - name: MyEntity
    entity:
      name: Test
      components:
        - type: A
          name: CompA
        - type: B
          name: CompB
      routes:
        - input: CompA.in
          output: CompB.out
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.routes.size(), 1u);
    // Routes should have entity prefix
    EXPECT_EQ(cfg.routes[0].input_path, "MyEntity.CompA.in");
    EXPECT_EQ(cfg.routes[0].output_path, "MyEntity.CompB.out");
}

TEST(SimulationLoaderTest, EntityOverridesMerged) {
    const char *yaml = R"(
entities:
  - name: Instance1
    entity:
      components:
        - type: Dynamics
          name: EOM
          scalars:
            mass: 500.0
            drag: 0.1
    overrides:
      EOM:
        scalars:
          mass: 1000.0
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.components.size(), 1u);
    // Overridden value
    EXPECT_DOUBLE_EQ(cfg.components[0].scalars.at("mass"), 1000.0);
    // Non-overridden value preserved
    EXPECT_DOUBLE_EQ(cfg.components[0].scalars.at("drag"), 0.1);
}

TEST(SimulationLoaderTest, MultipleEntityInstances) {
    const char *yaml = R"(
entities:
  - name: Leader
    entity:
      components:
        - type: Rocket
          name: EOM
  - name: Follower
    entity:
      components:
        - type: Rocket
          name: EOM
)";
    auto cfg = SimulationLoader::Parse(yaml);
    ASSERT_EQ(cfg.components.size(), 2u);
    EXPECT_EQ(cfg.components[0].entity, "Leader");
    EXPECT_EQ(cfg.components[1].entity, "Follower");
}

} // namespace
} // namespace icarus::io
