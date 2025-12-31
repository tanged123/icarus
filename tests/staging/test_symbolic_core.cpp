/**
 * @file test_symbolic_core.cpp
 * @brief Unit tests for SymbolicSimulatorCore and SymbolicStager
 *
 * Part of Phase 4: Staging Implementation (Phase C.6)
 *
 * Tests the symbolic graph generation infrastructure.
 */

#include <gtest/gtest.h>

#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentFactory.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/staging/SymbolicSimulatorCore.hpp>
#include <icarus/staging/SymbolicStager.hpp>

#include <janus/core/JanusTypes.hpp>

using namespace icarus;
using namespace icarus::staging;

// =============================================================================
// Simple Test Component - Dual Backend Compatible
// =============================================================================

/**
 * @brief Simple linear dynamics for testing: xdot = a*x + b*u
 *
 * State: x (scalar)
 * Input: u (scalar)
 * Params: a, b
 *
 * This component works in both numeric and symbolic modes.
 */
template <typename Scalar> class LinearDynamics : public Component<Scalar> {
  public:
    explicit LinearDynamics(std::string name = "LinearDynamics", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "LinearDynamics"; }
    [[nodiscard]] std::size_t StateSize() const override { return 1; }

    void Provision(Backplane<Scalar> &bp) override {
        // Output
        bp.template register_output<Scalar>("x", &x_, "m", "Position state");
        bp.template register_output<Scalar>("xdot", &xdot_, "m/s", "State derivative");

        // Input
        bp.template register_input<Scalar>("u", &u_input_, "N", "Control input");

        // Parameters
        bp.register_param("a", &a_, Scalar{-1.0}, "", "State gain");
        bp.register_param("b", &b_, Scalar{1.0}, "", "Input gain");
    }

    void Stage(Backplane<Scalar> &) override {
        // Set initial condition
        if (x_ptr_ != nullptr) {
            *x_ptr_ = x0_;
        }
    }

    void Step(Scalar /*t*/, Scalar /*dt*/) override {
        // Read state
        x_ = (x_ptr_ != nullptr) ? *x_ptr_ : Scalar{0};

        // Read input
        Scalar u = u_input_.get();

        // Compute derivative: xdot = a*x + b*u
        xdot_ = a_ * x_ + b_ * u;

        // Write derivative
        if (xdot_ptr_ != nullptr) {
            *xdot_ptr_ = xdot_;
        }
    }

    void SetInitialCondition(double x0) { x0_ = Scalar{x0}; }
    void SetParameters(double a, double b) {
        a_ = Scalar{a};
        b_ = Scalar{b};
    }

  private:
    std::string name_;
    std::string entity_;

    // State binding
    Scalar *x_ptr_ = nullptr;
    Scalar *xdot_ptr_ = nullptr;
    Scalar x0_{0.0};

    // Internal values
    Scalar x_{0.0};
    Scalar xdot_{0.0};

    // Input handle
    InputHandle<Scalar> u_input_;

    // Parameters
    Scalar a_{-1.0};
    Scalar b_{1.0};

    void BindState(Scalar *x, Scalar *xdot, std::size_t /*size*/) override {
        x_ptr_ = x;
        xdot_ptr_ = xdot;
        // Initialize state
        if (x_ptr_ != nullptr) {
            *x_ptr_ = x0_;
        }
    }
};

/**
 * @brief Simple constant input source
 */
template <typename Scalar> class ConstantInput : public Component<Scalar> {
  public:
    explicit ConstantInput(std::string name = "ConstantInput", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "ConstantInput"; }
    [[nodiscard]] std::size_t StateSize() const override { return 0; }

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_output<Scalar>("value", &value_, "", "Output value");
    }

    void Stage(Backplane<Scalar> &) override {}
    void Step(Scalar /*t*/, Scalar /*dt*/) override {}

    void SetValue(double val) { value_ = Scalar{val}; }

  private:
    std::string name_;
    std::string entity_;
    Scalar value_{0.0};
};

// Register components for dual-backend support
ICARUS_REGISTER_COMPONENT(LinearDynamics);
ICARUS_REGISTER_COMPONENT(ConstantInput);

// =============================================================================
// SymbolicSimulatorCore Tests
// =============================================================================

class SymbolicSimulatorCoreTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Create a simple config with our test components
        config_.dt = 0.01;
        config_.t_end = 1.0;

        // Add components
        ComponentConfig dynamics_cfg;
        dynamics_cfg.name = "Dynamics";
        dynamics_cfg.type = "LinearDynamics";
        dynamics_cfg.entity = "";
        config_.components.push_back(dynamics_cfg);

        ComponentConfig input_cfg;
        input_cfg.name = "Input";
        input_cfg.type = "ConstantInput";
        input_cfg.entity = "";
        config_.components.push_back(input_cfg);

        // Add route: Dynamics.u <- Input.value
        signal::SignalRoute route("Dynamics.u", "Input.value");
        config_.routes.push_back(route);
    }

    SimulatorConfig config_;
};

TEST_F(SymbolicSimulatorCoreTest, Creation) {
    EXPECT_NO_THROW({
        SymbolicSimulatorCore sym_sim(config_);
        EXPECT_EQ(sym_sim.GetStateSize(), 1);
        EXPECT_EQ(sym_sim.NumComponents(), 2);
    });
}

TEST_F(SymbolicSimulatorCoreTest, GetComponent) {
    SymbolicSimulatorCore sym_sim(config_);

    auto *dynamics = sym_sim.GetComponent("Dynamics");
    EXPECT_NE(dynamics, nullptr);
    EXPECT_EQ(dynamics->Name(), "Dynamics");
    EXPECT_EQ(dynamics->TypeName(), "LinearDynamics");

    auto *input = sym_sim.GetComponent("Input");
    EXPECT_NE(input, nullptr);
    EXPECT_EQ(input->Name(), "Input");
}

TEST_F(SymbolicSimulatorCoreTest, SymbolicState) {
    SymbolicSimulatorCore sym_sim(config_);

    // Create symbolic state
    auto [x_vec, x_mx] = janus::sym_vec_pair("x", 1);

    // Set state
    EXPECT_NO_THROW(sym_sim.SetState(x_vec));

    // Compute derivatives
    auto xdot = sym_sim.ComputeDerivatives();
    EXPECT_EQ(xdot.size(), 1);
}

TEST_F(SymbolicSimulatorCoreTest, SignalAccess) {
    SymbolicSimulatorCore sym_sim(config_);

    // Check signals exist
    EXPECT_TRUE(sym_sim.HasSignal("Dynamics.x"));
    EXPECT_TRUE(sym_sim.HasSignal("Input.value"));

    // Get signal names
    auto names = sym_sim.GetSignalNames();
    EXPECT_GE(names.size(), 2);
}

TEST_F(SymbolicSimulatorCoreTest, StateLayout) {
    SymbolicSimulatorCore sym_sim(config_);

    const auto &layout = sym_sim.GetStateLayout();
    EXPECT_EQ(layout.size(), 1); // Only Dynamics has state

    EXPECT_EQ(layout[0].component_name, "Dynamics");
    EXPECT_EQ(layout[0].offset, 0);
    EXPECT_EQ(layout[0].size, 1);
}

// =============================================================================
// SymbolicStager Tests
// =============================================================================

TEST_F(SymbolicSimulatorCoreTest, StagerDynamicsGeneration) {
    SymbolicSimulatorCore sym_sim(config_);
    SymbolicStager stager(sym_sim);

    SymbolicStagerConfig stager_config;
    stager_config.generate_dynamics = true;
    stager_config.generate_jacobian = true;
    stager_config.include_time = true;

    auto dynamics = stager.GenerateDynamics(stager_config);

    // Check dynamics function was created
    ASSERT_TRUE(dynamics.dynamics.has_value());
    EXPECT_TRUE(dynamics.dynamics->casadi_function().n_in() >= 1);
    EXPECT_TRUE(dynamics.dynamics->casadi_function().n_out() >= 1);
}

TEST_F(SymbolicSimulatorCoreTest, StagerJacobianGeneration) {
    SymbolicSimulatorCore sym_sim(config_);
    SymbolicStager stager(sym_sim);

    SymbolicStagerConfig stager_config;
    stager_config.generate_jacobian = true;
    stager_config.include_time = false; // Simpler function

    auto dynamics = stager.GenerateDynamics(stager_config);

    // Check Jacobian was created
    ASSERT_TRUE(dynamics.jacobian_x.has_value());
    EXPECT_TRUE(dynamics.jacobian_x->casadi_function().n_out() >= 1);
}

TEST_F(SymbolicSimulatorCoreTest, StagerStateNames) {
    SymbolicSimulatorCore sym_sim(config_);
    SymbolicStager stager(sym_sim);

    auto names = stager.GetStateNames();
    EXPECT_EQ(names.size(), 1);
    EXPECT_NE(names[0].find("Dynamics"), std::string::npos);
}

TEST_F(SymbolicSimulatorCoreTest, StagerWithControls) {
    SymbolicSimulatorCore sym_sim(config_);
    SymbolicStager stager(sym_sim);

    SymbolicStagerConfig stager_config;
    stager_config.generate_dynamics = true;
    stager_config.generate_jacobian = true;
    stager_config.control_signals = {"Input.value"};
    stager_config.include_time = true;

    auto dynamics = stager.GenerateDynamics(stager_config);

    // With controls, we should have control Jacobian
    ASSERT_TRUE(dynamics.dynamics.has_value());
    EXPECT_TRUE(dynamics.dynamics->casadi_function().n_in() >= 2); // t, x, [u]
}

// =============================================================================
// Function Evaluation Tests
// =============================================================================

TEST_F(SymbolicSimulatorCoreTest, DynamicsEvaluation) {
    SymbolicSimulatorCore sym_sim(config_);
    SymbolicStager stager(sym_sim);

    SymbolicStagerConfig stager_config;
    stager_config.generate_dynamics = true;
    stager_config.include_time = false;

    auto dynamics = stager.GenerateDynamics(stager_config);

    // Evaluate dynamics at a specific point
    // f(x=1) should give xdot = a*x + b*u = -1*1 + 1*0 = -1
    Eigen::VectorXd x(1);
    x << 1.0;

    ASSERT_TRUE(dynamics.dynamics.has_value());
    auto result = (*dynamics.dynamics)(x);
    EXPECT_EQ(result.size(), 1);
    EXPECT_EQ(result[0].rows(), 1);
}
