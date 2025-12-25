/**
 * @file test_signal.cpp
 * @brief Tests for Signal Backplane (Phase 1.3)
 *
 * Tests TypeTraits, SignalHandle, SignalRegistry, Vec3Handle,
 * and error handling for the signal backplane.
 */

#include <gtest/gtest.h>
#include <icarus/icarus.hpp>

namespace icarus {
namespace {

// =============================================================================
// TypeTraits Tests
// =============================================================================

TEST(TypeTraits, Double) {
    EXPECT_EQ(TypeTraits<double>::type_id, SignalType::Double);
    EXPECT_STREQ(TypeTraits<double>::name, "Double");
}

TEST(TypeTraits, Int32) {
    EXPECT_EQ(TypeTraits<int32_t>::type_id, SignalType::Int32);
    EXPECT_STREQ(TypeTraits<int32_t>::name, "Int32");
}

TEST(TypeTraits, Int64) {
    EXPECT_EQ(TypeTraits<int64_t>::type_id, SignalType::Int64);
    EXPECT_STREQ(TypeTraits<int64_t>::name, "Int64");
}

// =============================================================================
// SignalHandle Tests
// =============================================================================

TEST(SignalHandle, DefaultConstruction) {
    SignalHandle<double> handle;
    EXPECT_FALSE(handle.valid());
    EXPECT_FALSE(static_cast<bool>(handle));
}

TEST(SignalHandle, Access) {
    double value = 42.0;
    SignalDescriptor desc{.name = "test.signal",
                          .type = SignalType::Double,
                          .lifecycle = SignalLifecycle::Dynamic,
                          .unit = "m/s",
                          .description = "Test signal"};

    SignalHandle<double> handle(&value, &desc);

    EXPECT_TRUE(handle.valid());
    EXPECT_TRUE(static_cast<bool>(handle));

    // Read access
    EXPECT_DOUBLE_EQ(*handle, 42.0);

    // Write access
    *handle = 100.0;
    EXPECT_DOUBLE_EQ(value, 100.0);

    // Pointer access
    EXPECT_EQ(handle.ptr(), &value);
}

TEST(SignalHandle, Metadata) {
    double value = 0.0;
    SignalDescriptor desc{.name = "thrust.magnitude",
                          .type = SignalType::Double,
                          .lifecycle = SignalLifecycle::Dynamic,
                          .unit = "N",
                          .description = "Thrust magnitude"};

    SignalHandle<double> handle(&value, &desc);

    EXPECT_EQ(handle.name(), "thrust.magnitude");
    EXPECT_EQ(handle.unit(), "N");
    EXPECT_EQ(handle.lifecycle(), SignalLifecycle::Dynamic);
    EXPECT_EQ(handle.descriptor(), &desc);
}

// =============================================================================
// SignalRegistry - Legacy API Tests
// =============================================================================

TEST(SignalRegistry, LegacyRegisterAndResolve) {
    SignalRegistry<double> registry;

    SignalDescriptor desc{.name = "vehicle.velocity",
                          .type = SignalType::Double,
                          .lifecycle = SignalLifecycle::Dynamic,
                          .unit = "m/s",
                          .description = "Vehicle velocity"};

    auto index = registry.RegisterSignal(desc);
    EXPECT_EQ(index, 0);
    EXPECT_EQ(registry.Size(), 1);
    EXPECT_TRUE(registry.HasSignal("vehicle.velocity"));

    auto resolved_index = registry.Resolve("vehicle.velocity");
    EXPECT_EQ(resolved_index, index);
}

TEST(SignalRegistry, LegacyGetSet) {
    SignalRegistry<double> registry;

    SignalDescriptor desc{.name = "altitude",
                          .type = SignalType::Double,
                          .lifecycle = SignalLifecycle::Dynamic,
                          .unit = "m",
                          .description = "Altitude"};

    auto index = registry.RegisterSignal(desc);

    registry.Set(index, 10000.0);
    EXPECT_DOUBLE_EQ(registry.Get(index), 10000.0);

    registry.SetByName("altitude", 20000.0);
    EXPECT_DOUBLE_EQ(registry.GetByName("altitude"), 20000.0);
}

// =============================================================================
// SignalRegistry - Pointer-Based Registration
// =============================================================================

TEST(SignalRegistry, RegisterOutput) {
    SignalRegistry<double> registry;

    double thrust = 1000.0;
    registry.set_current_component("Propulsion");
    registry.register_output("thrust", &thrust, "N", "Engine thrust");

    EXPECT_TRUE(registry.HasSignal("thrust"));

    // Verify the pointer binding
    auto handle = registry.resolve<double>("thrust");
    EXPECT_DOUBLE_EQ(*handle, 1000.0);

    // Modify through the original variable
    thrust = 2000.0;
    EXPECT_DOUBLE_EQ(*handle, 2000.0);

    // Modify through handle
    *handle = 3000.0;
    EXPECT_DOUBLE_EQ(thrust, 3000.0);
}

TEST(SignalRegistry, RegisterStatic) {
    SignalRegistry<double> registry;

    const double gravity = 9.81;
    registry.register_static("gravity", &gravity, "m/s^2", "Standard gravity");

    EXPECT_TRUE(registry.HasSignal("gravity"));

    auto desc = registry.get_descriptor("gravity");
    ASSERT_NE(desc, nullptr);
    EXPECT_EQ(desc->lifecycle, SignalLifecycle::Static);
}

TEST(SignalRegistry, OwnerTracking) {
    SignalRegistry<double> registry;

    double val = 0.0;
    registry.set_current_component("Engine");
    registry.register_output("signal1", &val, "m");

    registry.set_current_component("Guidance");
    double val2 = 0.0;
    registry.register_output("signal2", &val2, "m");

    auto desc1 = registry.get_descriptor("signal1");
    auto desc2 = registry.get_descriptor("signal2");

    ASSERT_NE(desc1, nullptr);
    ASSERT_NE(desc2, nullptr);
    EXPECT_EQ(desc1->owner_component, "Engine");
    EXPECT_EQ(desc2->owner_component, "Guidance");
}

// =============================================================================
// SignalRegistry - Type-Safe Resolution
// =============================================================================

TEST(SignalRegistry, ResolveTyped) {
    SignalRegistry<double> registry;

    double value = 42.0;
    registry.register_output("test", &value);

    auto handle = registry.resolve<double>("test");
    EXPECT_TRUE(handle.valid());
    EXPECT_DOUBLE_EQ(*handle, 42.0);
}

TEST(SignalRegistry, TypeMismatch) {
    SignalRegistry<double> registry;

    double value = 42.0;
    registry.register_output("test", &value);

    // Try to resolve as wrong type
    EXPECT_THROW((void)registry.resolve<int32_t>("test"), TypeMismatchError);
}

TEST(SignalRegistry, SignalNotFound) {
    SignalRegistry<double> registry;

    EXPECT_THROW((void)registry.resolve<double>("nonexistent"), SignalNotFoundError);
    EXPECT_THROW((void)registry.Resolve("nonexistent"), SignalNotFoundError);
}

TEST(SignalRegistry, DuplicateSignal) {
    SignalRegistry<double> registry;

    double val1 = 0.0, val2 = 0.0;
    registry.set_current_component("Component1");
    registry.register_output("test", &val1);

    registry.set_current_component("Component2");
    EXPECT_THROW(registry.register_output("test", &val2), DuplicateSignalError);
}

// =============================================================================
// Vec3Handle Tests
// =============================================================================

TEST(SignalRegistry, RegisterVec3) {
    SignalRegistry<double> registry;

    Vec3<double> position(1.0, 2.0, 3.0);
    registry.register_vec3("position", &position, "m", "Position");

    EXPECT_TRUE(registry.HasSignal("position.x"));
    EXPECT_TRUE(registry.HasSignal("position.y"));
    EXPECT_TRUE(registry.HasSignal("position.z"));

    auto handle_x = registry.resolve<double>("position.x");
    auto handle_y = registry.resolve<double>("position.y");
    auto handle_z = registry.resolve<double>("position.z");

    EXPECT_DOUBLE_EQ(*handle_x, 1.0);
    EXPECT_DOUBLE_EQ(*handle_y, 2.0);
    EXPECT_DOUBLE_EQ(*handle_z, 3.0);
}

TEST(SignalRegistry, ResolveVec3) {
    SignalRegistry<double> registry;

    Vec3<double> velocity(10.0, 20.0, 30.0);
    registry.register_vec3("velocity", &velocity, "m/s");

    auto handle = registry.resolve_vec3<double>("velocity");
    EXPECT_TRUE(handle.valid());

    // Read all components
    Vec3<double> v = handle.get();
    EXPECT_DOUBLE_EQ(v(0), 10.0);
    EXPECT_DOUBLE_EQ(v(1), 20.0);
    EXPECT_DOUBLE_EQ(v(2), 30.0);

    // Write all components
    handle.set(Vec3<double>(100.0, 200.0, 300.0));
    EXPECT_DOUBLE_EQ(velocity(0), 100.0);
    EXPECT_DOUBLE_EQ(velocity(1), 200.0);
    EXPECT_DOUBLE_EQ(velocity(2), 300.0);
}

TEST(Vec3Handle, ReadWrite) {
    Vec3<double> data(1.0, 2.0, 3.0);

    SignalDescriptor desc_x{.name = "v.x", .type = SignalType::Double};
    SignalDescriptor desc_y{.name = "v.y", .type = SignalType::Double};
    SignalDescriptor desc_z{.name = "v.z", .type = SignalType::Double};

    Vec3Handle<double> handle{SignalHandle<double>(&data(0), &desc_x),
                              SignalHandle<double>(&data(1), &desc_y),
                              SignalHandle<double>(&data(2), &desc_z)};

    EXPECT_TRUE(handle.valid());

    Vec3<double> read = handle.get();
    EXPECT_DOUBLE_EQ(read(0), 1.0);
    EXPECT_DOUBLE_EQ(read(1), 2.0);
    EXPECT_DOUBLE_EQ(read(2), 3.0);

    handle.set(Vec3<double>(10.0, 20.0, 30.0));
    EXPECT_DOUBLE_EQ(data(0), 10.0);
    EXPECT_DOUBLE_EQ(data(1), 20.0);
    EXPECT_DOUBLE_EQ(data(2), 30.0);
}

// =============================================================================
// Query/Introspection Tests
// =============================================================================

TEST(SignalRegistry, Query) {
    SignalRegistry<double> registry;

    double a = 0, b = 0, c = 0;
    registry.register_output("engine.thrust", &a);
    registry.register_output("engine.fuel", &b);
    registry.register_output("guidance.target", &c);

    auto engine_signals = registry.query("engine\\..*");
    EXPECT_EQ(engine_signals.size(), 2);

    auto all_signals = registry.query(".*");
    EXPECT_EQ(all_signals.size(), 3);
}

TEST(SignalRegistry, GetDescriptor) {
    SignalRegistry<double> registry;

    double val = 0.0;
    registry.register_output("test", &val, "kg", "Test signal");

    auto desc = registry.get_descriptor("test");
    ASSERT_NE(desc, nullptr);
    EXPECT_EQ(desc->name, "test");
    EXPECT_EQ(desc->unit, "kg");

    EXPECT_EQ(registry.get_descriptor("nonexistent"), nullptr);
}

// =============================================================================
// Legacy API backward compatibility
// =============================================================================

TEST(SignalRegistry, LegacyWithOwnerTracking) {
    SignalRegistry<double> registry;

    registry.set_current_component("TestComponent");

    SignalDescriptor desc{.name = "legacy.signal",
                          .type = SignalType::Double,
                          .lifecycle = SignalLifecycle::Dynamic,
                          .unit = "m"};

    registry.RegisterSignal(desc);

    auto stored_desc = registry.get_descriptor("legacy.signal");
    ASSERT_NE(stored_desc, nullptr);
    EXPECT_EQ(stored_desc->owner_component, "TestComponent");

    // Verify pointer binding works with legacy API too
    auto handle = registry.resolve<double>("legacy.signal");
    EXPECT_TRUE(handle.valid());
}

// =============================================================================
// Symbolic Backend Tests (SymbolicScalar)
// =============================================================================

TEST(SignalRegistrySymbolic, LegacyAPI) {
    // SignalRegistry<SymbolicScalar> stores SymbolicScalar values
    SignalRegistry<SymbolicScalar> registry;

    SignalDescriptor desc{.name = "symbolic.signal",
                          .type = SignalType::Double,
                          .lifecycle = SignalLifecycle::Dynamic};

    auto index = registry.RegisterSignal(desc);
    EXPECT_EQ(index, 0);

    // Set a symbolic value
    SymbolicScalar sym_val = sym("x");
    registry.Set(index, sym_val);

    // Verify it's symbolic
    auto retrieved = registry.Get(index);
    EXPECT_TRUE(retrieved.is_symbolic());
}

TEST(SignalRegistrySymbolic, GetSetSymbolic) {
    SignalRegistry<SymbolicScalar> registry;

    SignalDescriptor desc{.name = "symbolic.value",
                          .type = SignalType::Double,
                          .lifecycle = SignalLifecycle::Dynamic};

    auto index = registry.RegisterSignal(desc);

    // Create and set a symbolic expression
    SymbolicScalar x = sym("x");
    SymbolicScalar expr = x * x + 2.0 * x + 1.0; // (x+1)^2
    registry.Set(index, expr);

    // Retrieve and verify it's not constant (has symbolic dependencies)
    auto retrieved = registry.Get(index);
    EXPECT_FALSE(retrieved.is_constant());
}

TEST(SignalRegistrySymbolic, MultipleSymbolicSignals) {
    SignalRegistry<SymbolicScalar> registry;

    SignalDescriptor desc1{.name = "state.x", .type = SignalType::Double};
    SignalDescriptor desc2{.name = "state.y", .type = SignalType::Double};
    SignalDescriptor desc3{.name = "state.z", .type = SignalType::Double};

    auto idx_x = registry.RegisterSignal(desc1);
    auto idx_y = registry.RegisterSignal(desc2);
    auto idx_z = registry.RegisterSignal(desc3);

    // Set symbolic values
    registry.Set(idx_x, sym("x"));
    registry.Set(idx_y, sym("y"));
    registry.Set(idx_z, sym("z"));

    // Verify all are symbolic
    EXPECT_TRUE(registry.Get(idx_x).is_symbolic());
    EXPECT_TRUE(registry.Get(idx_y).is_symbolic());
    EXPECT_TRUE(registry.Get(idx_z).is_symbolic());
}

TEST(SignalRegistrySymbolic, SymbolicExpressionPropagation) {
    SignalRegistry<SymbolicScalar> registry;

    SignalDescriptor desc{
        .name = "result", .type = SignalType::Double, .lifecycle = SignalLifecycle::Dynamic};

    auto index = registry.RegisterSignal(desc);

    // Build a symbolic computation graph
    SymbolicScalar x = sym("x");
    SymbolicScalar y = sym("y");
    SymbolicScalar result = janus::sin(x) + janus::cos(y);

    registry.Set(index, result);

    // The stored value should not be constant (has symbolic dependencies)
    auto stored = registry.Get(index);
    EXPECT_FALSE(stored.is_constant());
}

TEST(SignalRegistrySymbolic, FunctionEvaluation) {
    // This test demonstrates the full symbolic workflow:
    // 1. Create symbolic signals in registry
    // 2. Build expressions using registry values
    // 3. Create a janus::Function and evaluate numerically

    SignalRegistry<SymbolicScalar> registry;

    SignalDescriptor desc_x{.name = "x", .type = SignalType::Double};
    SignalDescriptor desc_y{.name = "y", .type = SignalType::Double};
    SignalDescriptor desc_out{.name = "output", .type = SignalType::Double};

    auto idx_x = registry.RegisterSignal(desc_x);
    auto idx_y = registry.RegisterSignal(desc_y);
    auto idx_out = registry.RegisterSignal(desc_out);

    // Set x and y as symbolic inputs
    SymbolicScalar sym_x = sym("x");
    SymbolicScalar sym_y = sym("y");
    registry.Set(idx_x, sym_x);
    registry.Set(idx_y, sym_y);

    // Compute output expression: f(x,y) = x * y + x
    SymbolicScalar x_val = registry.Get(idx_x);
    SymbolicScalar y_val = registry.Get(idx_y);
    SymbolicScalar out_expr = x_val * y_val + x_val;
    registry.Set(idx_out, out_expr);

    // Create a janus::Function to evaluate the expression
    janus::Function f("registry_func", {sym_x, sym_y}, {registry.Get(idx_out)});

    // Evaluate: f(2, 3) = 2*3 + 2 = 8
    auto res = f(2.0, 3.0);
    ASSERT_EQ(res.size(), 1);
    EXPECT_NEAR(res[0](0, 0), 8.0, 1e-9);

    // Evaluate: f(3, 4) = 3*4 + 3 = 15
    auto res2 = f(3.0, 4.0);
    EXPECT_NEAR(res2[0](0, 0), 15.0, 1e-9);
}

TEST(SignalRegistrySymbolic, GradientComputation) {
    // Test automatic differentiation through registry signals

    SignalRegistry<SymbolicScalar> registry;

    SignalDescriptor desc{.name = "param", .type = SignalType::Double};
    auto idx = registry.RegisterSignal(desc);

    // Set parameter as symbolic
    SymbolicScalar param = sym("param");
    registry.Set(idx, param);

    // Build expression: f(param) = param^2
    SymbolicScalar val = registry.Get(idx);
    SymbolicScalar f_expr = val * val;

    // Compute gradient: df/dparam = 2*param
    auto grad = janus::jacobian(f_expr, param);

    // Create function to evaluate the gradient
    janus::Function grad_func("grad", {param}, {grad});

    // At param=3: gradient = 2*3 = 6
    auto res = grad_func(3.0);
    EXPECT_NEAR(res[0](0, 0), 6.0, 1e-9);

    // At param=5: gradient = 2*5 = 10
    auto res2 = grad_func(5.0);
    EXPECT_NEAR(res2[0](0, 0), 10.0, 1e-9);
}

// =============================================================================
// Phase 2.4: InputHandle Tests
// =============================================================================

TEST(InputHandle, DefaultConstruction) {
    InputHandle<double> handle;
    EXPECT_FALSE(handle.is_wired());
    EXPECT_TRUE(handle.name().empty());
    EXPECT_TRUE(handle.wired_to().empty());
}

TEST(InputHandle, UnwiredAccessThrows) {
    InputHandle<double> handle;
    EXPECT_THROW((void)handle.get(), UnwiredInputError);
}

TEST(InputHandle, WiredAccess) {
    // Simulate wiring through registry
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    // Register an output
    double source_value = 42.0;
    bp.set_context("", "Source");
    bp.register_output("value", &source_value, "m", "Source value");

    // Register an input
    InputHandle<double> input;
    bp.set_context("", "Consumer");
    bp.register_input("input", &input, "m", "Input port");

    // Wire the input
    bp.wire_input<double>("Consumer.input", "Source.value");

    // Access the wired value
    EXPECT_TRUE(input.is_wired());
    EXPECT_DOUBLE_EQ(input.get(), 42.0);
    EXPECT_EQ(input.wired_to(), "Source.value");

    // Source changes propagate
    source_value = 100.0;
    EXPECT_DOUBLE_EQ(input.get(), 100.0);
}

// =============================================================================
// Phase 2.4: Parameter Registration Tests
// =============================================================================

TEST(SignalRegistry, RegisterParam) {
    SignalRegistry<double> registry;

    double mass = 0.0;
    registry.set_current_component("Vehicle");
    registry.register_param("mass", &mass, 1000.0, "kg", "Vehicle mass");

    EXPECT_TRUE(registry.has_param("mass"));
    EXPECT_DOUBLE_EQ(mass, 1000.0); // Initial value applied

    auto params = registry.get_params();
    EXPECT_EQ(params.size(), 1);
    EXPECT_EQ(params[0].kind, SignalKind::Parameter);
    EXPECT_TRUE(params[0].is_optimizable);
}

TEST(Backplane, RegisterParam) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double thrust = 0.0;
    bp.set_context("X15", "Engine");
    bp.register_param("max_thrust", &thrust, 50000.0, "N", "Max thrust");

    EXPECT_TRUE(registry.has_param("X15.Engine.max_thrust"));
    EXPECT_DOUBLE_EQ(thrust, 50000.0);
}

// =============================================================================
// Phase 2.4: Config Registration Tests
// =============================================================================

TEST(SignalRegistry, RegisterConfigInt) {
    SignalRegistry<double> registry;

    int mode = 0;
    registry.set_current_component("Controller");
    registry.register_config("mode", &mode, 3, "Control mode");

    EXPECT_TRUE(registry.has_config("mode"));
    EXPECT_EQ(mode, 3); // Initial value applied

    auto config = registry.get_config();
    EXPECT_EQ(config.size(), 1);
    EXPECT_EQ(config[0].kind, SignalKind::Config);
    EXPECT_FALSE(config[0].is_optimizable);
}

TEST(SignalRegistry, RegisterConfigBool) {
    SignalRegistry<double> registry;

    bool enabled = false;
    registry.register_config("enabled", &enabled, true, "Enable flag");

    EXPECT_TRUE(registry.has_config("enabled"));
    EXPECT_TRUE(enabled);
}

TEST(Backplane, RegisterConfig) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    int num_engines = 0;
    bp.set_context("Vehicle", "Propulsion");
    bp.register_config("num_engines", &num_engines, 4, "Number of engines");

    EXPECT_TRUE(registry.has_config("Vehicle.Propulsion.num_engines"));
    EXPECT_EQ(num_engines, 4);
}

// =============================================================================
// Phase 2.4: Wiring Validation Tests
// =============================================================================

TEST(SignalRegistry, GetUnwiredInputs) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    // Register source
    double source = 0.0;
    bp.set_context("", "Source");
    bp.register_output("output", &source);

    // Register two inputs
    InputHandle<double> input1, input2;
    bp.set_context("", "Consumer");
    bp.register_input("in1", &input1, "m");
    bp.register_input("in2", &input2, "m");

    // Wire only one
    bp.wire_input<double>("Consumer.in1", "Source.output");

    auto unwired = registry.get_unwired_inputs();
    EXPECT_EQ(unwired.size(), 1);
    EXPECT_EQ(unwired[0], "Consumer.in2");
}

TEST(SignalRegistry, ValidateWiringThrows) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double source = 0.0;
    bp.set_context("", "Source");
    bp.register_output("output", &source);

    InputHandle<double> input;
    bp.set_context("", "Consumer");
    bp.register_input("input", &input);

    // Should throw with unwired input
    EXPECT_THROW(registry.validate_wiring(), WiringError);
}

TEST(SignalRegistry, ValidateWiringPasses) {
    SignalRegistry<double> registry;
    Backplane<double> bp(registry);

    double source = 0.0;
    bp.set_context("", "Source");
    bp.register_output("output", &source);

    InputHandle<double> input;
    bp.set_context("", "Consumer");
    bp.register_input("input", &input);
    bp.wire_input<double>("Consumer.input", "Source.output");

    // Should not throw
    EXPECT_NO_THROW(registry.validate_wiring());
}

// =============================================================================
// Phase 2.4: WiringConfig Tests
// =============================================================================

TEST(WiringConfig, AddAndGet) {
    WiringConfig config;

    config.AddWiring("Consumer.input", "Source.output");

    EXPECT_TRUE(config.HasWiring("Consumer.input"));
    EXPECT_EQ(config.GetSource("Consumer.input"), "Source.output");
    EXPECT_FALSE(config.HasWiring("Other.input"));
}

TEST(WiringConfig, GetSourceThrows) {
    WiringConfig config;

    EXPECT_THROW((void)config.GetSource("nonexistent"), WiringError);
}

// =============================================================================
// Phase 2.4: SignalKind Tests
// =============================================================================

TEST(SignalKind, ToString) {
    EXPECT_STREQ(to_string(SignalKind::Output), "Output");
    EXPECT_STREQ(to_string(SignalKind::Input), "Input");
    EXPECT_STREQ(to_string(SignalKind::Parameter), "Parameter");
    EXPECT_STREQ(to_string(SignalKind::Config), "Config");
}

// =============================================================================
// Phase 2.4: DataDictionary Tests
// =============================================================================

TEST(DataDictionary, ComputeStats) {
    DataDictionary dict;

    DataDictionary::ComponentEntry comp;
    comp.name = "Test";
    comp.type = "TestType";

    SignalDescriptor output1, output2, input1, param1, cfg1;
    output1.kind = SignalKind::Output;
    output1.is_state = true;
    output2.kind = SignalKind::Output;
    output2.is_state = false;
    input1.kind = SignalKind::Input;
    input1.wired_to = ""; // Unwired
    param1.kind = SignalKind::Parameter;
    cfg1.kind = SignalKind::Config;

    comp.outputs = {output1, output2};
    comp.inputs = {input1};
    comp.parameters = {param1};
    comp.config = {cfg1};

    dict.components.push_back(comp);
    dict.ComputeStats();

    EXPECT_EQ(dict.total_outputs, 2);
    EXPECT_EQ(dict.total_inputs, 1);
    EXPECT_EQ(dict.total_parameters, 1);
    EXPECT_EQ(dict.total_config, 1);
    EXPECT_EQ(dict.integrable_states, 1);
    EXPECT_EQ(dict.unwired_inputs, 1);
}

} // namespace
} // namespace icarus
