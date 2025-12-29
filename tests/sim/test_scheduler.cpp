/**
 * @file test_scheduler.cpp
 * @brief Unit tests for Scheduler and SchedulerConfig
 *
 * Part of Phase 4.0.7: Scheduler Integration
 */

#include <gtest/gtest.h>
#include <icarus/sim/Scheduler.hpp>

namespace icarus {
namespace {

// =============================================================================
// SchedulerConfig Validation Tests
// =============================================================================

TEST(SchedulerConfigTest, ValidateEmptyGroupsReturnsError) {
    SchedulerConfig cfg;
    cfg.groups.clear();

    auto errors = cfg.Validate();
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors[0], "No scheduler groups defined");
}

TEST(SchedulerConfigTest, ValidateDuplicateGroupNamesReturnsError) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 400.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 200.0, 2}); // Duplicate

    auto errors = cfg.Validate();
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors[0], "Duplicate group name: fast");
}

TEST(SchedulerConfigTest, ValidateDuplicateComponentReturnsError) {
    SchedulerConfig cfg;
    SchedulerGroupConfig g1{"group1", 400.0, 1};
    g1.members = {{"ComponentA", 1}};
    cfg.groups.push_back(g1);

    SchedulerGroupConfig g2{"group2", 200.0, 2};
    g2.members = {{"ComponentA", 1}}; // Same component in two groups
    cfg.groups.push_back(g2);

    auto errors = cfg.Validate();
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors[0], "Component 'ComponentA' assigned to multiple groups");
}

TEST(SchedulerConfigTest, ValidateInvalidRateReturnsError) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"bad", -100.0, 1});

    auto errors = cfg.Validate();
    ASSERT_GE(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("invalid rate") != std::string::npos);
}

TEST(SchedulerConfigTest, ValidateValidConfigReturnsNoErrors) {
    SchedulerConfig cfg;
    SchedulerGroupConfig g1{"sensors", 400.0, 1};
    g1.members = {{"IMU", 1}, {"GPS", 2}};
    cfg.groups.push_back(g1);

    SchedulerGroupConfig g2{"dynamics", 200.0, 2};
    g2.members = {{"EOM", 1}};
    cfg.groups.push_back(g2);

    auto errors = cfg.Validate();
    EXPECT_TRUE(errors.empty());
}

// =============================================================================
// Global Timing Validation Tests
// =============================================================================

TEST(SchedulerConfigTest, ValidateGlobalTimingRateExceedsSimRate) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 1600.0, 1});

    // Simulation runs at 800 Hz, but group wants 1600 Hz
    auto errors = cfg.ValidateGlobalTiming(800.0);
    ASSERT_GE(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("exceeds simulation rate") != std::string::npos);
}

TEST(SchedulerConfigTest, ValidateGlobalTimingNonIntegerDivisor) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"weird", 513.0, 1});

    // 1600 / 513 = 3.12... not an integer
    auto errors = cfg.ValidateGlobalTiming(1600.0);
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_TRUE(errors[0].find("not an integer divisor") != std::string::npos);
}

TEST(SchedulerConfigTest, ValidateGlobalTimingValidDivisors) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 1600.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"medium", 400.0, 2});
    cfg.groups.push_back(SchedulerGroupConfig{"slow", 200.0, 3});

    auto errors = cfg.ValidateGlobalTiming(1600.0);
    EXPECT_TRUE(errors.empty());
}

// =============================================================================
// Frame Divisor Tests
// =============================================================================

TEST(SchedulerConfigTest, ComputeFrameDivisors) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 1600.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"medium", 400.0, 2});
    cfg.groups.push_back(SchedulerGroupConfig{"slow", 200.0, 3});

    cfg.ComputeFrameDivisors(1600.0);

    EXPECT_EQ(cfg.group_frame_divisors["fast"], 1);
    EXPECT_EQ(cfg.group_frame_divisors["medium"], 4);
    EXPECT_EQ(cfg.group_frame_divisors["slow"], 8);
}

// =============================================================================
// MaxRate Tests
// =============================================================================

TEST(SchedulerConfigTest, MaxRateReturnsHighestRate) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"slow", 200.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 1600.0, 2});
    cfg.groups.push_back(SchedulerGroupConfig{"medium", 400.0, 3});

    EXPECT_DOUBLE_EQ(cfg.MaxRate(), 1600.0);
}

TEST(SchedulerConfigTest, MaxRateEmptyGroupsReturnsDefault) {
    SchedulerConfig cfg;
    cfg.groups.clear();

    EXPECT_DOUBLE_EQ(cfg.MaxRate(), 400.0); // Default fallback
}

// =============================================================================
// Scheduler Execution Tests
// =============================================================================

TEST(SchedulerTest, ConfigureSortsGroupsByPriority) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"third", 400.0, 3});
    cfg.groups.push_back(SchedulerGroupConfig{"first", 400.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"second", 400.0, 2});

    Scheduler scheduler;
    scheduler.Configure(cfg, 400.0);

    auto groups = scheduler.GetGroups();
    ASSERT_EQ(groups.size(), 3u);
    EXPECT_EQ(groups[0].name, "first");
    EXPECT_EQ(groups[1].name, "second");
    EXPECT_EQ(groups[2].name, "third");
}

TEST(SchedulerTest, ConfigureSortsMembersByPriority) {
    SchedulerConfig cfg;
    SchedulerGroupConfig g{"group", 400.0, 1};
    g.members = {{"C", 3}, {"A", 1}, {"B", 2}};
    cfg.groups.push_back(g);

    Scheduler scheduler;
    scheduler.Configure(cfg, 400.0);

    auto members = scheduler.GetMembersForGroup("group");
    ASSERT_EQ(members.size(), 3u);
    EXPECT_EQ(members[0], "A");
    EXPECT_EQ(members[1], "B");
    EXPECT_EQ(members[2], "C");
}

TEST(SchedulerTest, GetGroupsForFrameMultiRate) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 1600.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"slow", 400.0, 2});

    Scheduler scheduler;
    scheduler.Configure(cfg, 1600.0);

    // Frame 0: both groups run
    auto groups0 = scheduler.GetGroupsForFrame(0);
    EXPECT_EQ(groups0.size(), 2u);

    // Frame 1: only fast runs
    auto groups1 = scheduler.GetGroupsForFrame(1);
    ASSERT_EQ(groups1.size(), 1u);
    EXPECT_EQ(groups1[0], "fast");

    // Frame 4: both groups run again
    auto groups4 = scheduler.GetGroupsForFrame(4);
    EXPECT_EQ(groups4.size(), 2u);
}

TEST(SchedulerTest, GetGroupDt) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 1600.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"slow", 400.0, 2});

    Scheduler scheduler;
    scheduler.Configure(cfg, 1600.0);

    // Fast group: dt = 1/1600 = 0.000625
    EXPECT_DOUBLE_EQ(scheduler.GetGroupDt("fast"), 1.0 / 1600.0);

    // Slow group: dt = 1/400 = 0.0025
    EXPECT_DOUBLE_EQ(scheduler.GetGroupDt("slow"), 1.0 / 400.0);
}

TEST(SchedulerTest, GetFrameDivisor) {
    SchedulerConfig cfg;
    cfg.groups.push_back(SchedulerGroupConfig{"fast", 1600.0, 1});
    cfg.groups.push_back(SchedulerGroupConfig{"slow", 200.0, 2});

    Scheduler scheduler;
    scheduler.Configure(cfg, 1600.0);

    EXPECT_EQ(scheduler.GetFrameDivisor("fast"), 1);
    EXPECT_EQ(scheduler.GetFrameDivisor("slow"), 8);
}

} // namespace
} // namespace icarus
