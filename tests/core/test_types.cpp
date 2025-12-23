#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// ============================================
// Types Tests
// ============================================

TEST(Types, VersionString) {
    EXPECT_STREQ(icarus::Version(), "0.1.0");
    EXPECT_STREQ(icarus::VersionString(), "0.1.0");
}

TEST(Types, VersionComponents) {
    EXPECT_EQ(icarus::VersionMajor(), 0);
    EXPECT_EQ(icarus::VersionMinor(), 1);
    EXPECT_EQ(icarus::VersionPatch(), 0);
}

TEST(Types, PhaseEnum) {
    // Verify Phase enum values exist
    auto phase = icarus::Phase::Uninitialized;
    EXPECT_EQ(phase, icarus::Phase::Uninitialized);

    phase = icarus::Phase::Provisioned;
    EXPECT_EQ(phase, icarus::Phase::Provisioned);

    phase = icarus::Phase::Staged;
    EXPECT_EQ(phase, icarus::Phase::Staged);

    phase = icarus::Phase::Running;
    EXPECT_EQ(phase, icarus::Phase::Running);

    phase = icarus::Phase::Completed;
    EXPECT_EQ(phase, icarus::Phase::Completed);
}

// ============================================
// Signal Tests
// ============================================

TEST(Signal, SignalTypeEnum) {
    EXPECT_EQ(static_cast<uint8_t>(icarus::SignalType::Float64), 0);
    EXPECT_EQ(static_cast<uint8_t>(icarus::SignalType::Int32), 1);
    EXPECT_EQ(static_cast<uint8_t>(icarus::SignalType::Int64), 2);
}

TEST(Signal, SignalLifecycleEnum) {
    EXPECT_EQ(static_cast<uint8_t>(icarus::SignalLifecycle::Static), 0);
    EXPECT_EQ(static_cast<uint8_t>(icarus::SignalLifecycle::Dynamic), 1);
}

TEST(Signal, SignalDescriptor) {
    icarus::SignalDescriptor desc;
    desc.name = "test.signal";
    desc.unit = "m/s";
    desc.type = icarus::SignalType::Float64;
    desc.lifecycle = icarus::SignalLifecycle::Dynamic;
    desc.description = "Test signal";
    desc.is_state = true;

    EXPECT_EQ(desc.name, "test.signal");
    EXPECT_EQ(desc.unit, "m/s");
    EXPECT_EQ(desc.type, icarus::SignalType::Float64);
    EXPECT_EQ(desc.lifecycle, icarus::SignalLifecycle::Dynamic);
    EXPECT_TRUE(desc.is_state);
}

// ============================================
// SignalRegistry Tests
// ============================================

TEST(SignalRegistry, RegisterAndResolve) {
    icarus::SignalRegistry<double> registry;

    icarus::SignalDescriptor desc;
    desc.name = "entity.component.output";
    desc.type = icarus::SignalType::Float64;

    auto index = registry.RegisterSignal(desc);
    EXPECT_EQ(index, 0);

    auto resolved = registry.Resolve("entity.component.output");
    EXPECT_EQ(resolved, index);
}

TEST(SignalRegistry, SetAndGet) {
    icarus::SignalRegistry<double> registry;

    icarus::SignalDescriptor desc;
    desc.name = "test.value";
    desc.type = icarus::SignalType::Float64;

    auto index = registry.RegisterSignal(desc);
    registry.Set(index, 42.0);

    EXPECT_DOUBLE_EQ(registry.Get(index), 42.0);
    EXPECT_DOUBLE_EQ(registry.GetByName("test.value"), 42.0);
}

TEST(SignalRegistry, DuplicateRegistrationThrows) {
    icarus::SignalRegistry<double> registry;

    icarus::SignalDescriptor desc;
    desc.name = "duplicate.signal";
    desc.type = icarus::SignalType::Float64;

    registry.RegisterSignal(desc);
    EXPECT_THROW(registry.RegisterSignal(desc), icarus::SignalError);
}

TEST(SignalRegistry, UnknownSignalThrows) {
    icarus::SignalRegistry<double> registry;
    EXPECT_THROW((void)registry.Resolve("nonexistent"), icarus::SignalError);
}

// ============================================
// Simulator Tests
// ============================================

TEST(Simulator, DefaultConstruction) {
    icarus::Simulator<double> sim;
    EXPECT_EQ(sim.GetPhase(), icarus::Phase::Uninitialized);
    EXPECT_EQ(sim.NumComponents(), 0);
}
