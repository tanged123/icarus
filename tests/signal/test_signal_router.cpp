/**
 * @file test_signal_router.cpp
 * @brief Unit tests for SignalRouter with gain support
 *
 * Part of Phase 4.0: Configuration Infrastructure (4.0.11)
 */

#include <gtest/gtest.h>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/SignalRouter.hpp>

namespace icarus {
namespace signal {
namespace {

class SignalRouterTest : public ::testing::Test {
  protected:
    SignalRegistry<double> registry;
    Backplane<double> backplane{registry};
    SignalRouter<double> router;

    // Test signal storage
    double source_value = 100.0;
    InputHandle<double> input_handle;

    void SetUp() override {
        // Register an output signal
        // Note: set_current_component sets ownership, signal name is the full path
        registry.set_current_component("Source");
        registry.register_output<double>("Source.value", &source_value, "V", "Test source");

        // Register an input
        registry.set_current_component("Dest");
        registry.register_input<double>("Dest.input", &input_handle, "V", "Test input");
        registry.clear_current_component();
    }
};

// =============================================================================
// Route Management Tests
// =============================================================================

TEST_F(SignalRouterTest, AddRouteIncrementsSize) {
    EXPECT_EQ(router.Size(), 0);

    router.AddRoute("Dest.Input.value", "Source.Output.value");
    EXPECT_EQ(router.Size(), 1);

    router.AddRoute("Another.Input", "Another.Output", 2.0);
    EXPECT_EQ(router.Size(), 2);
}

TEST_F(SignalRouterTest, ClearRemovesAllRoutes) {
    router.AddRoute("A", "B");
    router.AddRoute("C", "D");
    EXPECT_EQ(router.Size(), 2);

    router.Clear();
    EXPECT_EQ(router.Size(), 0);
}

TEST_F(SignalRouterTest, GetRoutesReturnsAllRoutes) {
    router.AddRoute("in1", "out1", 1.0);
    router.AddRoute("in2", "out2", 2.5);

    const auto &routes = router.GetRoutes();
    ASSERT_EQ(routes.size(), 2);

    EXPECT_EQ(routes[0].input_path, "in1");
    EXPECT_EQ(routes[0].output_path, "out1");
    EXPECT_DOUBLE_EQ(routes[0].gain, 1.0);

    EXPECT_EQ(routes[1].input_path, "in2");
    EXPECT_EQ(routes[1].output_path, "out2");
    EXPECT_DOUBLE_EQ(routes[1].gain, 2.5);
}

// =============================================================================
// SignalRoute Struct Tests
// =============================================================================

TEST(SignalRouteTest, DefaultGainIsOne) {
    SignalRoute route;
    EXPECT_DOUBLE_EQ(route.gain, 1.0);
}

TEST(SignalRouteTest, ConstructorSetsValues) {
    SignalRoute route("input.path", "output.path", 0.5);
    EXPECT_EQ(route.input_path, "input.path");
    EXPECT_EQ(route.output_path, "output.path");
    EXPECT_DOUBLE_EQ(route.gain, 0.5);
}

// =============================================================================
// Validation Tests
// =============================================================================

TEST_F(SignalRouterTest, ValidateRoutesReturnsEmptyForValidRoutes) {
    router.AddRoute("Dest.input", "Source.value");

    auto errors = router.ValidateRoutes(backplane);
    EXPECT_TRUE(errors.empty());
}

TEST_F(SignalRouterTest, ValidateRoutesReportsMissingOutput) {
    router.AddRoute("Dest.input", "NonExistent.Signal");

    auto errors = router.ValidateRoutes(backplane);
    ASSERT_EQ(errors.size(), 1);
    EXPECT_TRUE(errors[0].find("NonExistent.Signal") != std::string::npos);
    EXPECT_TRUE(errors[0].find("Output not found") != std::string::npos);
}

TEST_F(SignalRouterTest, ValidateRoutesReportsMissingInput) {
    router.AddRoute("NonExistent.Input", "Source.value");

    auto errors = router.ValidateRoutes(backplane);
    ASSERT_EQ(errors.size(), 1);
    EXPECT_TRUE(errors[0].find("NonExistent.Input") != std::string::npos);
    EXPECT_TRUE(errors[0].find("Input not found") != std::string::npos);
}

TEST_F(SignalRouterTest, ValidateRoutesReportsMultipleErrors) {
    router.AddRoute("Bad.Input", "Bad.Output");

    auto errors = router.ValidateRoutes(backplane);
    EXPECT_EQ(errors.size(), 2); // Both input and output missing
}

// =============================================================================
// Apply Routes Tests
// =============================================================================

TEST_F(SignalRouterTest, ApplyRoutesThrowsOnInvalidRoutes) {
    router.AddRoute("Bad.Input", "Bad.Output");

    EXPECT_THROW(router.ApplyRoutes(backplane), RoutingError);
}

TEST_F(SignalRouterTest, ApplyRoutesWiresValidRoutes) {
    router.AddRoute("Dest.input", "Source.value");

    // Should not throw
    EXPECT_NO_THROW(router.ApplyRoutes(backplane));

    // Verify the input is now wired
    EXPECT_TRUE(input_handle.is_wired());
    EXPECT_EQ(input_handle.wired_to(), "Source.value");
}

// =============================================================================
// Unwired Input Detection Tests
// =============================================================================

TEST_F(SignalRouterTest, GetUnwiredInputsReturnsUnwired) {
    // Don't add any routes - input should be unwired
    auto unwired = router.GetUnwiredInputs(backplane);

    // Should have exactly one unwired input
    ASSERT_EQ(unwired.size(), 1);
    EXPECT_EQ(unwired[0], "Dest.input");
}

TEST_F(SignalRouterTest, GetUnwiredInputsExcludesWired) {
    router.AddRoute("Dest.input", "Source.value");

    auto unwired = router.GetUnwiredInputs(backplane);
    EXPECT_TRUE(unwired.empty());
}

} // namespace
} // namespace signal
} // namespace icarus
