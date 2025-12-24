#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

// ============================================
// Types Tests
// ============================================

TEST(Types, VersionString) {
    EXPECT_STREQ(icarus::Version(), ICARUS_VERSION_STR(ICARUS_VERSION_MAJOR, ICARUS_VERSION_MINOR,
                                                       ICARUS_VERSION_PATCH));
}

TEST(Types, VersionComponents) {
    EXPECT_EQ(icarus::VersionMajor(), ICARUS_VERSION_MAJOR);
    EXPECT_EQ(icarus::VersionMinor(), ICARUS_VERSION_MINOR);
    EXPECT_EQ(icarus::VersionPatch(), ICARUS_VERSION_PATCH);
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
    // Verify SignalType values exist (re-exported from Vulcan)
    auto type = icarus::SignalType::Double;
    EXPECT_EQ(type, icarus::SignalType::Double);

    type = icarus::SignalType::Int32;
    EXPECT_EQ(type, icarus::SignalType::Int32);

    type = icarus::SignalType::Int64;
    EXPECT_EQ(type, icarus::SignalType::Int64);
}

TEST(Signal, SignalLifecycleEnum) {
    // Verify SignalLifecycle values exist (re-exported from Vulcan)
    auto lc = icarus::SignalLifecycle::Static;
    EXPECT_EQ(lc, icarus::SignalLifecycle::Static);

    lc = icarus::SignalLifecycle::Dynamic;
    EXPECT_EQ(lc, icarus::SignalLifecycle::Dynamic);
}

TEST(Signal, SignalDescriptor) {
    icarus::SignalDescriptor desc;
    desc.name = "test.signal";
    desc.unit = "m/s";
    desc.type = icarus::SignalType::Double;
    desc.lifecycle = icarus::SignalLifecycle::Dynamic;
    desc.description = "Test signal";
    desc.is_state = true;

    EXPECT_EQ(desc.name, "test.signal");
    EXPECT_EQ(desc.unit, "m/s");
    EXPECT_EQ(desc.type, icarus::SignalType::Double);
    EXPECT_EQ(desc.lifecycle, icarus::SignalLifecycle::Dynamic);
    EXPECT_TRUE(desc.is_state);
    EXPECT_EQ(desc.size_bytes(), 8); // Match Vulcan's wire format
}

// ============================================
// SignalRegistry Tests
// ============================================

TEST(SignalRegistry, RegisterAndResolve) {
    icarus::SignalRegistry<double> registry;

    icarus::SignalDescriptor desc;
    desc.name = "entity.component.output";
    desc.type = icarus::SignalType::Double;

    auto index = registry.RegisterSignal(desc);
    EXPECT_EQ(index, 0);

    auto resolved = registry.Resolve("entity.component.output");
    EXPECT_EQ(resolved, index);
}

TEST(SignalRegistry, SetAndGet) {
    icarus::SignalRegistry<double> registry;

    icarus::SignalDescriptor desc;
    desc.name = "test.value";
    desc.type = icarus::SignalType::Double;

    auto index = registry.RegisterSignal(desc);
    registry.Set(index, 42.0);

    EXPECT_DOUBLE_EQ(registry.Get(index), 42.0);
    EXPECT_DOUBLE_EQ(registry.GetByName("test.value"), 42.0);
}

TEST(SignalRegistry, DuplicateRegistrationThrows) {
    icarus::SignalRegistry<double> registry;

    icarus::SignalDescriptor desc;
    desc.name = "duplicate.signal";
    desc.type = icarus::SignalType::Double;

    registry.RegisterSignal(desc);
    EXPECT_THROW(registry.RegisterSignal(desc), icarus::SignalError);
}

TEST(SignalRegistry, UnknownSignalThrows) {
    icarus::SignalRegistry<double> registry;
    EXPECT_THROW((void)registry.Resolve("nonexistent"), icarus::SignalNotFoundError);
}

// ============================================
// Simulator Tests
// ============================================

TEST(Simulator, DefaultConstruction) {
    icarus::Simulator<double> sim;
    EXPECT_EQ(sim.GetPhase(), icarus::Phase::Uninitialized);
    EXPECT_EQ(sim.NumComponents(), 0);
}

// ============================================
// Symbolic Backend Tests (SymbolicScalar)
// ============================================

TEST(SignalRegistrySymbolic, RegisterAndResolve) {
    icarus::SignalRegistry<janus::SymbolicScalar> registry;

    icarus::SignalDescriptor desc;
    desc.name = "symbolic.signal";
    desc.type = icarus::SignalType::Double;

    auto index = registry.RegisterSignal(desc);
    EXPECT_EQ(index, 0);

    auto resolved = registry.Resolve("symbolic.signal");
    EXPECT_EQ(resolved, index);
}

TEST(SignalRegistrySymbolic, SetAndGetSymbolic) {
    icarus::SignalRegistry<janus::SymbolicScalar> registry;

    icarus::SignalDescriptor desc;
    desc.name = "symbolic.value";
    desc.type = icarus::SignalType::Double;

    auto index = registry.RegisterSignal(desc);

    // Set a symbolic value
    janus::SymbolicScalar sym_val = janus::sym("x");
    registry.Set(index, sym_val);

    // Verify it's a valid symbolic expression
    auto retrieved = registry.Get(index);
    EXPECT_TRUE(retrieved.is_symbolic());
}

TEST(SimulatorSymbolic, DefaultConstruction) {
    icarus::Simulator<janus::SymbolicScalar> sim;
    EXPECT_EQ(sim.GetPhase(), icarus::Phase::Uninitialized);
    EXPECT_EQ(sim.NumComponents(), 0);
}
