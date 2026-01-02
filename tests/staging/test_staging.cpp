/**
 * @file test_staging.cpp
 * @brief Tests for Phase 4 Staging: TrimSolver and Linearizer
 *
 * Tests finite-difference trim and linearization using simple dynamical systems
 * with known equilibrium points and analytically-derivable Jacobians.
 */

#include <gtest/gtest.h>

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/sim/Simulator.hpp>
#include <icarus/staging/Linearizer.hpp>
#include <icarus/staging/StagingTypes.hpp>
#include <icarus/staging/TrimSolver.hpp>

#include <cmath>

using namespace icarus;
using namespace icarus::staging;

// =============================================================================
// Test Component: Mass-Spring-Damper
// =============================================================================
// A simple 1D system: mx'' + cx' + kx = F
// State: [x, v] where x=position, v=velocity
// Dynamics: x' = v, v' = (F - cx' - kx) / m
// Equilibrium: x_eq = F/k, v_eq = 0
// Jacobians: A = [[0, 1], [-k/m, -c/m]], B = [[0], [1/m]]

template <typename Scalar> class MassSpringDamper : public Component<Scalar> {
  public:
    MassSpringDamper(const std::string &name = "MSD", const std::string &entity = "")
        : name_(name), entity_(entity) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "MassSpringDamper"; }

    void Provision(Backplane<Scalar> &bp) override {
        // Register states using unified signal model
        bp.template register_state<Scalar>("position", &position_val_, &position_dot_val_, "m",
                                           "Position state");
        bp.template register_state<Scalar>("velocity", &velocity_val_, &velocity_dot_val_, "m/s",
                                           "Velocity state");

        // Control input
        bp.template register_input<Scalar>("force", &force_input_);

        // Parameters (exposed as outputs for visibility)
        bp.template register_output<Scalar>("mass", &mass_);
        bp.template register_output<Scalar>("spring_k", &spring_k_);
        bp.template register_output<Scalar>("damping_c", &damping_c_);
    }

    void Stage(Backplane<Scalar> & /*bp*/) override {
        // Apply initial conditions
        position_val_ = x0_;
        velocity_val_ = v0_;
    }

    void Step(Scalar /*t*/, Scalar /*dt*/) override {
        // Read input (may be unwired, use default if so)
        Scalar F = force_input_.is_wired() ? force_input_.get() : Scalar{0.0};

        // Compute derivatives
        // In unified signal model, states are owned by component
        position_dot_val_ = velocity_val_;
        velocity_dot_val_ = (F - damping_c_ * velocity_val_ - spring_k_ * position_val_) / mass_;
    }

    void SetParameters(Scalar mass, Scalar spring_k, Scalar damping_c) {
        mass_ = mass;
        spring_k_ = spring_k;
        damping_c_ = damping_c;
    }

    void SetInitialConditions(Scalar x0, Scalar v0) {
        x0_ = x0;
        v0_ = v0;
    }

  private:
    std::string name_;
    std::string entity_;

    // Parameters
    Scalar mass_ = 1.0;
    Scalar spring_k_ = 10.0;
    Scalar damping_c_ = 1.0;

    // Initial conditions
    Scalar x0_ = 0.0;
    Scalar v0_ = 0.0;

    // State storage (owned by component in Phase 6)
    Scalar position_val_ = 0.0;
    Scalar velocity_val_ = 0.0;
    Scalar position_dot_val_ = 0.0;
    Scalar velocity_dot_val_ = 0.0;

    // Input handle
    InputHandle<Scalar> force_input_;
};

// =============================================================================
// Test Component: Constant Force Source
// =============================================================================

template <typename Scalar> class ConstantForce : public Component<Scalar> {
  public:
    ConstantForce(const std::string &name = "Force", const std::string &entity = "")
        : name_(name), entity_(entity) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "ConstantForce"; }

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_output<Scalar>("force", &force_val_);
        force_val_ = force_value_;
    }

    void Stage(Backplane<Scalar> & /*bp*/) override { force_val_ = force_value_; }

    void Step(Scalar /*t*/, Scalar /*dt*/) override {
        // NOTE: Don't overwrite force_val_ here because:
        // 1. The simulator may have Poked a different value for trim/linearization
        // 2. The force is already set via Stage or SetForce
        // This allows Poke to modify the output for derivative computation.
    }

    void SetForce(Scalar value) {
        force_value_ = value;
        force_val_ = value; // Also update output immediately
    }

  private:
    std::string name_;
    std::string entity_;
    Scalar force_value_ = 0.0;
    Scalar force_val_ = 0.0;
};

// =============================================================================
// LinearModel Unit Tests
// =============================================================================

TEST(LinearModel, ControllabilityRank_Controllable) {
    // Controllable system: A = [[0, 1], [0, 0]], B = [[0], [1]]
    // Controllability matrix: [B, AB] = [[0, 1], [1, 0]] -> rank 2
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 0, 1, 0, 0;
    model.B.resize(2, 1);
    model.B << 0, 1;
    model.C.resize(0, 2);
    model.D.resize(0, 1);

    EXPECT_EQ(model.ControllabilityRank(), 2);
}

TEST(LinearModel, ControllabilityRank_Uncontrollable) {
    // Uncontrollable system: A = [[1, 0], [0, 2]], B = [[1], [0]]
    // Controllability matrix: [B, AB] = [[1, 1], [0, 0]] -> rank 1
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 1, 0, 0, 2;
    model.B.resize(2, 1);
    model.B << 1, 0;
    model.C.resize(0, 2);
    model.D.resize(0, 1);

    EXPECT_EQ(model.ControllabilityRank(), 1);
}

TEST(LinearModel, ObservabilityRank_Observable) {
    // Observable system: A = [[0, 1], [0, 0]], C = [[1, 0]]
    // Observability matrix: [C; CA] = [[1, 0], [0, 1]] -> rank 2
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 0, 1, 0, 0;
    model.B.resize(2, 1);
    model.B << 0, 1;
    model.C.resize(1, 2);
    model.C << 1, 0;
    model.D.resize(1, 1);
    model.D << 0;

    EXPECT_EQ(model.ObservabilityRank(), 2);
}

TEST(LinearModel, IsStable_Stable) {
    // Stable system: eigenvalues at -1 and -2
    // A = [[-3, 0], [0, -1]] (diagonal, eigenvalues on diagonal)
    LinearModel model;
    model.A.resize(2, 2);
    model.A << -3, 0, 0, -1;

    EXPECT_TRUE(model.IsStable());
}

TEST(LinearModel, IsStable_Unstable) {
    // Unstable system: one eigenvalue at +1
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 1, 0, 0, -1;

    EXPECT_FALSE(model.IsStable());
}

TEST(LinearModel, IsStable_Marginal) {
    // Marginally stable: eigenvalue at 0
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 0, 1, 0, 0;

    EXPECT_FALSE(model.IsStable()); // Marginal counts as unstable
}

TEST(LinearModel, Eigenvalues) {
    // A = [[-1, 0], [0, -2]] -> eigenvalues at -1, -2
    LinearModel model;
    model.A.resize(2, 2);
    model.A << -1, 0, 0, -2;

    auto eigs = model.Eigenvalues();
    ASSERT_EQ(eigs.size(), 2);

    // Sort eigenvalues by real part for comparison
    std::vector<double> real_parts = {eigs(0).real(), eigs(1).real()};
    std::sort(real_parts.begin(), real_parts.end());

    EXPECT_NEAR(real_parts[0], -2.0, 1e-10);
    EXPECT_NEAR(real_parts[1], -1.0, 1e-10);
}

// =============================================================================
// LinearModel Export Tests
// =============================================================================

TEST(LinearModel, ExportMatlab) {
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 1, 2, 3, 4;
    model.B.resize(2, 1);
    model.B << 5, 6;
    model.C.resize(1, 2);
    model.C << 7, 8;
    model.D.resize(1, 1);
    model.D << 0;
    model.x0.resize(2);
    model.x0 << 0, 0;
    model.u0.resize(1);
    model.u0 << 0;
    model.state_names = {"x1", "x2"};
    model.input_names = {"u1"};
    model.output_names = {"y1"};
    model.t0 = 0.0;

    // Export to temp file
    std::string path = "/tmp/test_linear_model.m";
    EXPECT_NO_THROW(model.ExportMatlab(path));

    // Verify file was created
    std::ifstream file(path);
    EXPECT_TRUE(file.good());

    // Read contents
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    // Check for expected content
    EXPECT_NE(content.find("A ="), std::string::npos);
    EXPECT_NE(content.find("B ="), std::string::npos);
    EXPECT_NE(content.find("sys = ss"), std::string::npos);
    EXPECT_NE(content.find("'x1'"), std::string::npos);
}

TEST(LinearModel, ExportNumPy) {
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 1, 2, 3, 4;
    model.B.resize(2, 1);
    model.B << 5, 6;
    model.C.resize(1, 2);
    model.C << 7, 8;
    model.D.resize(1, 1);
    model.D << 0;
    model.x0.resize(2);
    model.x0 << 0, 0;
    model.u0.resize(1);
    model.u0 << 0;
    model.state_names = {"x1", "x2"};
    model.input_names = {"u1"};
    model.output_names = {"y1"};
    model.t0 = 0.0;

    std::string path = "/tmp/test_linear_model.py";
    EXPECT_NO_THROW(model.ExportNumPy(path));

    std::ifstream file(path);
    EXPECT_TRUE(file.good());

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    EXPECT_NE(content.find("import numpy"), std::string::npos);
    EXPECT_NE(content.find("A = np.array"), std::string::npos);
    EXPECT_NE(content.find("state_names"), std::string::npos);
}

TEST(LinearModel, ExportJSON) {
    LinearModel model;
    model.A.resize(2, 2);
    model.A << 1, 2, 3, 4;
    model.B.resize(2, 1);
    model.B << 5, 6;
    model.C.resize(1, 2);
    model.C << 7, 8;
    model.D.resize(1, 1);
    model.D << 0;
    model.x0.resize(2);
    model.x0 << 0, 0;
    model.u0.resize(1);
    model.u0 << 0;
    model.state_names = {"x1", "x2"};
    model.input_names = {"u1"};
    model.output_names = {"y1"};
    model.t0 = 0.0;

    std::string path = "/tmp/test_linear_model.json";
    EXPECT_NO_THROW(model.ExportJSON(path));

    std::ifstream file(path);
    EXPECT_TRUE(file.good());

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    EXPECT_NE(content.find("\"A\":"), std::string::npos);
    EXPECT_NE(content.find("\"state_names\":"), std::string::npos);
}

// =============================================================================
// TrimResult Tests
// =============================================================================

TEST(TrimResult, DefaultConstruction) {
    TrimResult result;
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_DOUBLE_EQ(result.residual_norm, 0.0);
    EXPECT_TRUE(result.message.empty());
    EXPECT_TRUE(result.controls.empty());
    EXPECT_TRUE(result.residuals.empty());
}

// =============================================================================
// FiniteDifferenceTrim Tests
// =============================================================================

class TrimTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Create a mass-spring-damper system
        msd_ = std::make_unique<MassSpringDamper<double>>("MSD");
        force_ = std::make_unique<ConstantForce<double>>("Force");

        // Parameters: m=1, k=10, c=1
        // Equilibrium at F=50: x_eq = F/k = 5.0, v_eq = 0
        msd_->SetParameters(1.0, 10.0, 1.0);
        msd_->SetInitialConditions(0.0, 0.0);
        force_->SetForce(50.0); // This gives equilibrium at x=5
    }

    std::unique_ptr<MassSpringDamper<double>> msd_;
    std::unique_ptr<ConstantForce<double>> force_;
};

TEST_F(TrimTest, FiniteDifferenceTrim_ConvergesToEquilibrium) {
    // Create simulator
    Simulator sim;

    // Add components - for this test, we find the force that gives zero acceleration
    // at a given position. Set initial position to 5.0.
    msd_->SetInitialConditions(5.0, 0.0); // At desired equilibrium position
    force_->SetForce(0.0);                // Initial force guess

    sim.AddComponent(std::move(force_));
    sim.AddComponent(std::move(msd_));

    // Wire components
    std::vector<signal::SignalRoute> routes = {
        {"MSD.force", "Force.force"},
    };
    sim.AddRoutes(routes);

    // Stage
    sim.Stage();

    // Trim: find force that gives zero acceleration at current position
    // At x=5, v=0, we need F = k*x = 10*5 = 50 for equilibrium
    TrimConfig config;
    config.enabled = true;
    config.zero_derivatives = {"MSD.velocity_dot"}; // Zero acceleration
    config.control_signals = {"Force.force"};       // Adjust force to achieve equilibrium
    config.tolerance = 1e-8;
    config.max_iterations = 50;
    config.initial_guesses["Force.force"] = 10.0; // Start with wrong force

    FiniteDifferenceTrim::Options opts;
    opts.tolerance = config.tolerance;
    opts.max_iterations = config.max_iterations;
    opts.verbose = false;

    FiniteDifferenceTrim solver(opts);
    auto result = solver.Solve(sim, config);

    // Should converge
    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.residual_norm, 1e-6);

    // Equilibrium force should be F = k*x = 10*5 = 50.0
    EXPECT_NEAR(result.controls.at("Force.force"), 50.0, 1e-4);
}

TEST_F(TrimTest, FiniteDifferenceTrim_NoControlSignals) {
    Simulator sim;

    sim.AddComponent(std::move(force_));
    sim.AddComponent(std::move(msd_));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    TrimConfig config;
    config.enabled = true;
    config.zero_derivatives = {"MSD.velocity_dot"};
    config.control_signals = {}; // No controls

    FiniteDifferenceTrim solver;
    auto result = solver.Solve(sim, config);

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.message, "No control signals specified");
}

TEST_F(TrimTest, FiniteDifferenceTrim_NoDerivatives) {
    Simulator sim;

    sim.AddComponent(std::move(force_));
    sim.AddComponent(std::move(msd_));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    TrimConfig config;
    config.enabled = true;
    config.zero_derivatives = {};              // No derivatives to zero
    config.control_signals = {"MSD.position"}; // Has controls

    FiniteDifferenceTrim solver;
    auto result = solver.Solve(sim, config);

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.message, "No zero_derivatives specified");
}

// =============================================================================
// FiniteDifferenceLinearizer Tests
// =============================================================================

class LinearizerTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Create mass-spring-damper system
        // m=1, k=10, c=2
        // At equilibrium (x=x_eq, v=0), the A matrix is:
        // A = [[0, 1], [-k/m, -c/m]] = [[0, 1], [-10, -2]]
        msd_ = std::make_unique<MassSpringDamper<double>>("MSD");
        force_ = std::make_unique<ConstantForce<double>>("Force");

        msd_->SetParameters(1.0, 10.0, 2.0);  // m=1, k=10, c=2
        msd_->SetInitialConditions(5.0, 0.0); // Start at equilibrium
        force_->SetForce(50.0);               // F=50 gives x_eq=5
    }

    std::unique_ptr<MassSpringDamper<double>> msd_;
    std::unique_ptr<ConstantForce<double>> force_;
};

TEST_F(LinearizerTest, FiniteDifferenceLinearizer_ComputesCorrectA) {
    Simulator sim;

    sim.AddComponent(std::move(force_));
    sim.AddComponent(std::move(msd_));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    // Configure linearization
    LinearizationConfig config;
    config.enabled = true;
    config.states = {"MSD.position", "MSD.velocity"};
    config.inputs = {};  // No inputs for this test
    config.outputs = {}; // No outputs for this test

    FiniteDifferenceLinearizer linearizer;
    auto model = linearizer.Compute(sim, config);

    // Check A matrix dimensions
    ASSERT_EQ(model.A.rows(), 2);
    ASSERT_EQ(model.A.cols(), 2);

    // Expected A matrix for mass-spring-damper:
    // A = [[0, 1], [-k/m, -c/m]] = [[0, 1], [-10, -2]]
    EXPECT_NEAR(model.A(0, 0), 0.0, 1e-4);
    EXPECT_NEAR(model.A(0, 1), 1.0, 1e-4);
    EXPECT_NEAR(model.A(1, 0), -10.0, 1e-4);
    EXPECT_NEAR(model.A(1, 1), -2.0, 1e-4);

    // Check stability (should be stable: eigenvalues at -1±3i)
    EXPECT_TRUE(model.IsStable());
}

TEST_F(LinearizerTest, FiniteDifferenceLinearizer_ControllabilityWithB) {
    // For this test, we treat force as an input
    // B = [[0], [1/m]] = [[0], [1]]

    Simulator sim;

    // We need to make force a controllable input
    // For simplicity, just verify B matrix computation

    auto msd = std::make_unique<MassSpringDamper<double>>("MSD");
    auto force = std::make_unique<ConstantForce<double>>("Force");

    msd->SetParameters(1.0, 10.0, 2.0);
    msd->SetInitialConditions(5.0, 0.0);
    force->SetForce(50.0);

    sim.AddComponent(std::move(force));
    sim.AddComponent(std::move(msd));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    LinearizationConfig config;
    config.enabled = true;
    config.states = {"MSD.position", "MSD.velocity"};
    config.inputs = {"Force.force"}; // Force is the input
    config.outputs = {};

    FiniteDifferenceLinearizer linearizer;
    auto model = linearizer.Compute(sim, config);

    // Check B matrix dimensions
    ASSERT_EQ(model.B.rows(), 2);
    ASSERT_EQ(model.B.cols(), 1);

    // B = [[0], [1/m]] = [[0], [1]]
    // Note: The linearizer computes d(x_dot)/d(u), where x_dot = [v, (F-cv-kx)/m]
    // d(v_dot)/dF = 1/m = 1
    EXPECT_NEAR(model.B(0, 0), 0.0, 1e-4);
    EXPECT_NEAR(model.B(1, 0), 1.0, 1e-4);

    // Check controllability rank (should be 2 = full rank)
    EXPECT_EQ(model.ControllabilityRank(), 2);
}

TEST_F(LinearizerTest, FiniteDifferenceLinearizer_OutputMatrix) {
    Simulator sim;

    auto msd = std::make_unique<MassSpringDamper<double>>("MSD");
    auto force = std::make_unique<ConstantForce<double>>("Force");

    msd->SetParameters(1.0, 10.0, 2.0);
    msd->SetInitialConditions(5.0, 0.0);
    force->SetForce(50.0);

    sim.AddComponent(std::move(force));
    sim.AddComponent(std::move(msd));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    LinearizationConfig config;
    config.enabled = true;
    config.states = {"MSD.position", "MSD.velocity"};
    config.inputs = {};
    config.outputs = {"MSD.position"}; // Output is position

    FiniteDifferenceLinearizer linearizer;
    auto model = linearizer.Compute(sim, config);

    // C matrix: y = position = [1, 0] * [x, v]'
    ASSERT_EQ(model.C.rows(), 1);
    ASSERT_EQ(model.C.cols(), 2);
    EXPECT_NEAR(model.C(0, 0), 1.0, 1e-4);
    EXPECT_NEAR(model.C(0, 1), 0.0, 1e-4);
}

TEST_F(LinearizerTest, FiniteDifferenceLinearizer_OperatingPoint) {
    Simulator sim;

    auto msd = std::make_unique<MassSpringDamper<double>>("MSD");
    auto force = std::make_unique<ConstantForce<double>>("Force");

    msd->SetParameters(1.0, 10.0, 2.0);
    msd->SetInitialConditions(5.0, 0.5); // Position=5, velocity=0.5
    force->SetForce(50.0);

    sim.AddComponent(std::move(force));
    sim.AddComponent(std::move(msd));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    LinearizationConfig config;
    config.enabled = true;
    config.states = {"MSD.position", "MSD.velocity"};
    config.inputs = {"Force.force"};
    config.outputs = {};

    FiniteDifferenceLinearizer linearizer;
    auto model = linearizer.Compute(sim, config);

    // Check operating point is stored
    ASSERT_EQ(model.x0.size(), 2);
    EXPECT_NEAR(model.x0(0), 5.0, 1e-10);
    EXPECT_NEAR(model.x0(1), 0.5, 1e-10);

    ASSERT_EQ(model.u0.size(), 1);
    EXPECT_NEAR(model.u0(0), 50.0, 1e-10);

    // Check state names
    EXPECT_EQ(model.state_names.size(), 2u);
    EXPECT_EQ(model.state_names[0], "MSD.position");
    EXPECT_EQ(model.state_names[1], "MSD.velocity");

    // Check input names
    EXPECT_EQ(model.input_names.size(), 1u);
    EXPECT_EQ(model.input_names[0], "Force.force");
}

// =============================================================================
// Factory Tests
// =============================================================================

TEST(StagingFactory, CreateTrimSolver_Numeric) {
    TrimConfig config;
    config.method = "newton";

    auto solver = CreateTrimSolver(config, false); // symbolic_enabled = false
    EXPECT_NE(solver, nullptr);

    // Should be FiniteDifferenceTrim
    auto *fd_trim = dynamic_cast<FiniteDifferenceTrim *>(solver.get());
    EXPECT_NE(fd_trim, nullptr);
}

TEST(StagingFactory, CreateTrimSolver_Symbolic) {
    TrimConfig config;
    config.method = "newton";

    auto solver = CreateTrimSolver(config, true); // symbolic_enabled = true
    EXPECT_NE(solver, nullptr);

    // Should be SymbolicTrim
    auto *sym_trim = dynamic_cast<SymbolicTrim *>(solver.get());
    EXPECT_NE(sym_trim, nullptr);
}

TEST(StagingFactory, CreateLinearizer_Numeric) {
    auto linearizer = CreateLinearizer(false);
    EXPECT_NE(linearizer, nullptr);

    auto *fd_lin = dynamic_cast<FiniteDifferenceLinearizer *>(linearizer.get());
    EXPECT_NE(fd_lin, nullptr);
}

TEST(StagingFactory, CreateLinearizer_Symbolic) {
    auto linearizer = CreateLinearizer(true);
    EXPECT_NE(linearizer, nullptr);

    auto *sym_lin = dynamic_cast<SymbolicLinearizer *>(linearizer.get());
    EXPECT_NE(sym_lin, nullptr);
}

// =============================================================================
// Integration Tests (Trim + Linearization)
// =============================================================================

TEST(StagingIntegration, TrimThenLinearize) {
    // Full workflow: trim to equilibrium, then linearize
    Simulator sim;

    auto msd = std::make_unique<MassSpringDamper<double>>("MSD");
    auto force = std::make_unique<ConstantForce<double>>("Force");

    // m=1, k=10, c=2
    msd->SetParameters(1.0, 10.0, 2.0);
    msd->SetInitialConditions(5.0, 0.0); // Set at equilibrium position
    force->SetForce(0.0);                // Start with wrong force

    sim.AddComponent(std::move(force));
    sim.AddComponent(std::move(msd));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    // Step 1: Trim to find the force that balances the spring at x=5
    TrimConfig trim_config;
    trim_config.enabled = true;
    trim_config.zero_derivatives = {"MSD.velocity_dot"};
    trim_config.control_signals = {"Force.force"}; // Adjust force
    trim_config.tolerance = 1e-8;
    trim_config.max_iterations = 50;
    trim_config.initial_guesses["Force.force"] = 10.0;

    FiniteDifferenceTrim::Options trim_opts;
    trim_opts.tolerance = trim_config.tolerance;
    trim_opts.max_iterations = trim_config.max_iterations;

    FiniteDifferenceTrim trim_solver(trim_opts);
    auto trim_result = trim_solver.Solve(sim, trim_config);

    EXPECT_TRUE(trim_result.converged);
    // At x=5, equilibrium force is F = k*x = 10*5 = 50
    EXPECT_NEAR(trim_result.controls.at("Force.force"), 50.0, 1e-4);

    // Step 2: Linearize at the trim point
    LinearizationConfig lin_config;
    lin_config.enabled = true;
    lin_config.states = {"MSD.position", "MSD.velocity"};
    lin_config.inputs = {"Force.force"};
    lin_config.outputs = {"MSD.position"};

    FiniteDifferenceLinearizer linearizer;
    auto model = linearizer.Compute(sim, lin_config);

    // Verify linearization at equilibrium
    EXPECT_NEAR(model.x0(0), 5.0, 1e-4);       // x = 5 at equilibrium
    EXPECT_NEAR(model.A(1, 0), -10.0, 1e-4);   // -k/m = -10
    EXPECT_NEAR(model.A(1, 1), -2.0, 1e-4);    // -c/m = -2
    EXPECT_EQ(model.ControllabilityRank(), 2); // Fully controllable
    EXPECT_TRUE(model.IsStable());             // Stable system
}

// =============================================================================
// SymbolicTrim Tests
// =============================================================================

// Symbolic trim requires proper component registration for dual-backend
// Full integration testing deferred to dedicated symbolic test file
TEST(SymbolicTrim, SymbolicCreation) {
    // Test placeholder - symbolic components require ICARUS_REGISTER_COMPONENT
    // in built-in components to work. Test will be enabled once component
    // library fully supports symbolic mode.
    Simulator sim;

    auto msd = std::make_unique<MassSpringDamper<double>>("MSD");
    auto force = std::make_unique<ConstantForce<double>>("Force");

    msd->SetParameters(1.0, 10.0, 2.0);
    force->SetForce(50.0);

    sim.AddComponent(std::move(force));
    sim.AddComponent(std::move(msd));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    TrimConfig config;
    config.enabled = true;
    config.zero_derivatives = {"MSD.velocity_dot"};
    config.control_signals = {"MSD.position"};

    SymbolicTrim solver;
    auto result = solver.Solve(sim, config);

    // Actual behavior: attempts symbolic trim (may fail if components not registered)
    // When components support symbolic mode, this should converge
}

// =============================================================================
// SymbolicLinearizer Tests
// =============================================================================

// Symbolic linearization requires proper component registration for dual-backend
// Full integration testing deferred to dedicated symbolic test file
TEST(SymbolicLinearizer, SymbolicJacobian) {
    // Test placeholder - symbolic components require ICARUS_REGISTER_COMPONENT
    // in built-in components to work. Test will be enabled once component
    // library fully supports symbolic mode.
    Simulator sim;

    auto msd = std::make_unique<MassSpringDamper<double>>("MSD");
    auto force = std::make_unique<ConstantForce<double>>("Force");

    msd->SetParameters(1.0, 10.0, 2.0);
    force->SetForce(50.0);

    sim.AddComponent(std::move(force));
    sim.AddComponent(std::move(msd));

    std::vector<signal::SignalRoute> routes = {{"MSD.force", "Force.force"}};
    sim.AddRoutes(routes);
    sim.Stage();

    LinearizationConfig config;
    config.enabled = true;
    config.states = {"MSD.position", "MSD.velocity"};
    config.inputs = {};
    config.outputs = {};

    SymbolicLinearizer linearizer;
    // When components support symbolic mode, this should compute exact Jacobians
}
