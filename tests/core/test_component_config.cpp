/**
 * @file test_component_config.cpp
 * @brief Unit tests for ComponentConfig typed accessors
 *
 * Part of Phase 4.0: Configuration Infrastructure (4.0.10)
 */

#include <gtest/gtest.h>
#include <icarus/core/ComponentConfig.hpp>

namespace icarus {
namespace {

class ComponentConfigTest : public ::testing::Test {
  protected:
    ComponentConfig cfg;

    void SetUp() override {
        cfg.name = "TestComponent";
        cfg.entity = "TestEntity";
        cfg.type = "TestType";
    }
};

// =============================================================================
// Scalar (double) Tests
// =============================================================================

TEST_F(ComponentConfigTest, RequireDoubleThrowsOnMissing) {
    EXPECT_THROW(cfg.Require<double>("missing"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireDoubleReturnsValue) {
    cfg.scalars["mass"] = 1000.0;
    EXPECT_DOUBLE_EQ(cfg.Require<double>("mass"), 1000.0);
}

TEST_F(ComponentConfigTest, GetDoubleReturnsDefault) {
    EXPECT_DOUBLE_EQ(cfg.Get<double>("missing", 42.0), 42.0);
}

TEST_F(ComponentConfigTest, GetDoubleReturnsValue) {
    cfg.scalars["cd"] = 0.5;
    EXPECT_DOUBLE_EQ(cfg.Get<double>("cd", 0.0), 0.5);
}

TEST_F(ComponentConfigTest, HasDoubleReturnsFalse) { EXPECT_FALSE(cfg.Has<double>("missing")); }

TEST_F(ComponentConfigTest, HasDoubleReturnsTrue) {
    cfg.scalars["density"] = 1.225;
    EXPECT_TRUE(cfg.Has<double>("density"));
}

// =============================================================================
// Integer Tests
// =============================================================================

TEST_F(ComponentConfigTest, RequireIntThrowsOnMissing) {
    EXPECT_THROW(cfg.Require<int>("missing"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireIntReturnsValue) {
    cfg.integers["count"] = 42;
    EXPECT_EQ(cfg.Require<int>("count"), 42);
}

TEST_F(ComponentConfigTest, GetIntReturnsDefault) { EXPECT_EQ(cfg.Get<int>("missing", 10), 10); }

TEST_F(ComponentConfigTest, GetInt64ReturnsValue) {
    cfg.integers["large"] = 9223372036854775807LL;
    EXPECT_EQ(cfg.Require<int64_t>("large"), 9223372036854775807LL);
}

// =============================================================================
// Boolean Tests
// =============================================================================

TEST_F(ComponentConfigTest, RequireBoolThrowsOnMissing) {
    EXPECT_THROW(cfg.Require<bool>("missing"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireBoolReturnsValue) {
    cfg.booleans["enabled"] = true;
    EXPECT_TRUE(cfg.Require<bool>("enabled"));
}

TEST_F(ComponentConfigTest, GetBoolReturnsDefault) {
    EXPECT_FALSE(cfg.Get<bool>("missing", false));
}

// =============================================================================
// String Tests
// =============================================================================

TEST_F(ComponentConfigTest, RequireStringThrowsOnMissing) {
    EXPECT_THROW(cfg.Require<std::string>("missing"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireStringReturnsValue) {
    cfg.strings["model"] = "F-22";
    EXPECT_EQ(cfg.Require<std::string>("model"), "F-22");
}

TEST_F(ComponentConfigTest, GetStringReturnsDefault) {
    EXPECT_EQ(cfg.Get<std::string>("missing", "default"), "default");
}

TEST_F(ComponentConfigTest, HasStringReturnsFalse) {
    EXPECT_FALSE(cfg.Has<std::string>("missing"));
}

TEST_F(ComponentConfigTest, HasStringReturnsTrue) {
    cfg.strings["name"] = "test";
    EXPECT_TRUE(cfg.Has<std::string>("name"));
}

// =============================================================================
// Vec3<double> Tests
// =============================================================================

TEST_F(ComponentConfigTest, RequireVec3ThrowsOnMissing) {
    EXPECT_THROW(cfg.Require<Vec3<double>>("missing"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireVec3ThrowsOnWrongSize) {
    cfg.vectors["position"] = {1.0, 2.0}; // Only 2 elements
    EXPECT_THROW(cfg.Require<Vec3<double>>("position"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireVec3ReturnsValue) {
    cfg.vectors["position"] = {1.0, 2.0, 3.0};
    auto v = cfg.Require<Vec3<double>>("position");
    EXPECT_DOUBLE_EQ(v(0), 1.0);
    EXPECT_DOUBLE_EQ(v(1), 2.0);
    EXPECT_DOUBLE_EQ(v(2), 3.0);
}

TEST_F(ComponentConfigTest, GetVec3ReturnsDefault) {
    Vec3<double> def{4.0, 5.0, 6.0};
    auto v = cfg.Get<Vec3<double>>("missing", def);
    EXPECT_DOUBLE_EQ(v(0), 4.0);
    EXPECT_DOUBLE_EQ(v(1), 5.0);
    EXPECT_DOUBLE_EQ(v(2), 6.0);
}

TEST_F(ComponentConfigTest, HasVec3ReturnsFalse) { EXPECT_FALSE(cfg.Has<Vec3<double>>("missing")); }

TEST_F(ComponentConfigTest, HasVec3ReturnsTrueForValid) {
    cfg.vectors["velocity"] = {10.0, 20.0, 30.0};
    EXPECT_TRUE(cfg.Has<Vec3<double>>("velocity"));
}

TEST_F(ComponentConfigTest, HasVec3ReturnsFalseForWrongSize) {
    cfg.vectors["bad"] = {1.0, 2.0}; // Wrong size
    EXPECT_FALSE(cfg.Has<Vec3<double>>("bad"));
}

// =============================================================================
// Vec4<double> Tests (Quaternion)
// =============================================================================

TEST_F(ComponentConfigTest, RequireVec4ThrowsOnMissing) {
    EXPECT_THROW(cfg.Require<Vec4<double>>("missing"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireVec4ReturnsValue) {
    cfg.vectors["attitude"] = {1.0, 0.0, 0.0, 0.0};
    auto q = cfg.Require<Vec4<double>>("attitude");
    EXPECT_DOUBLE_EQ(q(0), 1.0); // w
    EXPECT_DOUBLE_EQ(q(1), 0.0); // x
    EXPECT_DOUBLE_EQ(q(2), 0.0); // y
    EXPECT_DOUBLE_EQ(q(3), 0.0); // z
}

// =============================================================================
// std::vector<double> Tests
// =============================================================================

TEST_F(ComponentConfigTest, RequireVectorDoubleThrowsOnMissing) {
    EXPECT_THROW(cfg.Require<std::vector<double>>("missing"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireVectorDoubleReturnsFromArrays) {
    cfg.arrays["coefficients"] = {0.1, 0.2, 0.3, 0.4, 0.5};
    auto v = cfg.Require<std::vector<double>>("coefficients");
    ASSERT_EQ(v.size(), 5);
    EXPECT_DOUBLE_EQ(v[0], 0.1);
    EXPECT_DOUBLE_EQ(v[4], 0.5);
}

TEST_F(ComponentConfigTest, RequireVectorDoubleReturnsFromVectors) {
    cfg.vectors["inertia"] = {100.0, 200.0, 50.0};
    auto v = cfg.Require<std::vector<double>>("inertia");
    ASSERT_EQ(v.size(), 3);
    EXPECT_DOUBLE_EQ(v[0], 100.0);
}

TEST_F(ComponentConfigTest, HasVectorDoubleChecksArraysAndVectors) {
    EXPECT_FALSE(cfg.Has<std::vector<double>>("missing"));

    cfg.arrays["arr"] = {1.0};
    EXPECT_TRUE(cfg.Has<std::vector<double>>("arr"));

    cfg.vectors["vec"] = {2.0};
    EXPECT_TRUE(cfg.Has<std::vector<double>>("vec"));
}

// =============================================================================
// std::vector<std::string> (sources) Tests
// =============================================================================

TEST_F(ComponentConfigTest, RequireSourcesThrowsOnEmpty) {
    EXPECT_THROW(cfg.Require<std::vector<std::string>>("sources"), ConfigError);
}

TEST_F(ComponentConfigTest, RequireSourcesReturnsValue) {
    cfg.sources = {"source1", "source2", "source3"};
    auto s = cfg.Require<std::vector<std::string>>("sources");
    ASSERT_EQ(s.size(), 3);
    EXPECT_EQ(s[0], "source1");
    EXPECT_EQ(s[2], "source3");
}

TEST_F(ComponentConfigTest, GetSourcesReturnsDefault) {
    std::vector<std::string> def = {"default"};
    auto s = cfg.Get<std::vector<std::string>>("sources", def);
    ASSERT_EQ(s.size(), 1);
    EXPECT_EQ(s[0], "default");
}

TEST_F(ComponentConfigTest, HasSourcesReturnsFalse) {
    EXPECT_FALSE(cfg.Has<std::vector<std::string>>("sources"));
}

TEST_F(ComponentConfigTest, HasSourcesReturnsTrue) {
    cfg.sources = {"a", "b"};
    EXPECT_TRUE(cfg.Has<std::vector<std::string>>("sources"));
}

// =============================================================================
// FullPath Tests
// =============================================================================

TEST_F(ComponentConfigTest, FullPathWithEntity) {
    cfg.entity = "Vehicle";
    cfg.name = "Engine";
    EXPECT_EQ(cfg.FullPath(), "Vehicle.Engine");
}

TEST_F(ComponentConfigTest, FullPathWithoutEntity) {
    cfg.entity = "";
    cfg.name = "Environment";
    EXPECT_EQ(cfg.FullPath(), "Environment");
}

// =============================================================================
// Error Message Tests
// =============================================================================

TEST_F(ComponentConfigTest, ConfigErrorContainsComponentPath) {
    try {
        cfg.Require<double>("nonexistent");
        FAIL() << "Expected ConfigError";
    } catch (const ConfigError &e) {
        std::string msg = e.what();
        EXPECT_TRUE(msg.find("TestEntity.TestComponent") != std::string::npos);
        EXPECT_TRUE(msg.find("nonexistent") != std::string::npos);
    }
}

} // namespace
} // namespace icarus
