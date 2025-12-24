# Phase 2.3: First Real Component Implementation Plan

**Status:** Proposed
**Target:** End-to-end simulation with `PointMass3DOF` and `PointMassGravity` components

---

## Overview

Phase 2.3 delivers the **first real simulation** — a point mass falling under gravity. This validates the entire Phase 1 + Phase 2 infrastructure end-to-end:

1. **Component lifecycle** — Provision → Stage → Step
2. **State management** — Pointer-based scatter/gather
3. **Signal backplane** — Component I/O wiring
4. **Integrator** — RK4 advancing state over time
5. **Vulcan integration** — Using physics utilities for gravity

Two components demonstrate the architecture:

| Component | Role | State | Inputs | Outputs |
|:----------|:-----|:------|:-------|:--------|
| **PointMass3DOF** | Dynamics | 6 (pos + vel) | `acceleration` | `position`, `velocity` |
| **PointMassGravity** | Environment | 0 | `position` | `acceleration` |

```
┌──────────────────────────────────────────────────────────────────┐
│                    PHASE 2.3 COMPONENT DIAGRAM                    │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│   PointMassGravity (Stateless)                                  │
│   ┌─────────────────────────────┐                               │
│   │ position ─────────────────┐ │                               │
│   │                           │ │                               │
│   │   g = -μ/|r|² * r̂        │ │                               │
│   │                           ▼ │                               │
│   │                    acceleration ──────────────┐             │
│   └─────────────────────────────┘                 │             │
│                                                   │             │
│   PointMass3DOF (Stateful)                        │             │
│   ┌─────────────────────────────┐                 │             │
│   │ ┌─────────────────────────┐ │                 │             │
│   │ │ State: r, v (6 vars)    │ │◄────────────────┘             │
│   │ └─────────────────────────┘ │                               │
│   │                             │                               │
│   │   ṙ = v                     │                               │
│   │   v̇ = acceleration         │                               │
│   │                             │                               │
│   │   Outputs: position, velocity                               │
│   └─────────────────────────────┘                               │
│                                                                  │
│   Integrator (RK4)                                              │
│   ┌─────────────────────────────┐                               │
│   │ X_new = rk4_step(f, X, t, dt)                               │
│   │ where f = ComputeDerivatives()                              │
│   └─────────────────────────────┘                               │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## Architecture Alignment

### Component Protocol

Per [02_component_protocol.md](../../architecture/02_component_protocol.md):

| Lifecycle Method | PointMass3DOF | PointMassGravity |
|:-----------------|:--------------|:-----------------|
| **Provision** | Register outputs: position, velocity | Register output: acceleration |
| **Stage** | Resolve input: acceleration; Apply IC | Resolve input: position |
| **Step** | Read accel, write pos/vel derivatives | Read position, compute gravity |

### State Ownership

Per [09_memory_state_ownership.md](../../architecture/09_memory_state_ownership.md):

```
X_global_ (6 elements)
┌───────────────────────────────────────────────────┐
│ rx │ ry │ rz │ vx │ vy │ vz │
│ 0  │ 1  │ 2  │ 3  │ 4  │ 5  │
└───────────────────────────────────────────────────┘
        ▲                       ▲
        │                       │
  PointMass3DOF.r_         PointMass3DOF.v_
  (pointer to X[0:3])      (pointer to X[3:6])
```

### Vulcan Usage

Per [08_vulcan_integration.md](../../architecture/08_vulcan_integration.md):

```cpp
// Gravity computation (Vulcan provides, Icarus uses)
Vec3<Scalar> g = vulcan::gravity::point_mass(r_ecef, mu);

// Vulcan is stateless — pure functions only
```

---

## Exit Criteria (From Main Plan)

- [ ] Simulate a falling object (point mass under gravity)
- [ ] Verify against analytical solution: `y(t) = y₀ + v₀t - ½gt²`
- [ ] State correctly scattered/gathered through integrator

---

## Component Design

### 1. PointMass3DOF Component

The core dynamics component with 6 state variables (position + velocity).

#### File: `components/dynamics/PointMass3DOF.hpp`

```cpp
#pragma once

/**
 * @file PointMass3DOF.hpp
 * @brief 3-DOF point mass dynamics component
 *
 * Part of Phase 2.3: First Real Component
 */

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/core/Types.hpp>
#include <janus/types.hpp>

namespace icarus {
namespace components {

/**
 * @brief 3-DOF point mass translational dynamics
 *
 * State vector: [rx, ry, rz, vx, vy, vz] (6 variables)
 *
 * Equations of motion:
 *   dr/dt = v           (kinematics)
 *   dv/dt = a           (Newton's 2nd law, F/m = a)
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar>
class PointMass3DOF : public Component<Scalar> {
public:
    static constexpr std::size_t kStateSize = 6;
    static constexpr std::size_t kPosOffset = 0;
    static constexpr std::size_t kVelOffset = 3;

    /**
     * @brief Construct with optional name and entity
     */
    explicit PointMass3DOF(std::string name = "PointMass3DOF",
                          std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "PointMass3DOF"; }

    [[nodiscard]] std::size_t StateSize() const override { return kStateSize; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register outputs: position, velocity
     */
    void Provision(Backplane<Scalar>& bp, const ComponentConfig&) override {
        // Position (ECEF or inertial frame)
        bp.register_output("position.x", &position_x_, "m", "Position X component");
        bp.register_output("position.y", &position_y_, "m", "Position Y component");
        bp.register_output("position.z", &position_z_, "m", "Position Z component");

        // Velocity (ECEF or inertial frame)
        bp.register_output("velocity.x", &velocity_x_, "m/s", "Velocity X component");
        bp.register_output("velocity.y", &velocity_y_, "m/s", "Velocity Y component");
        bp.register_output("velocity.z", &velocity_z_, "m/s", "Velocity Z component");
    }

    /**
     * @brief Wire inputs and apply initial conditions
     */
    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Resolve acceleration input (from gravity or other force sources)
        accel_x_ = bp.template resolve<Scalar>("Gravity.acceleration.x");
        accel_y_ = bp.template resolve<Scalar>("Gravity.acceleration.y");
        accel_z_ = bp.template resolve<Scalar>("Gravity.acceleration.z");

        // Apply initial conditions from config (or defaults)
        if (cfg.has("initial_position.x")) {
            ic_pos_x_ = cfg.get<Scalar>("initial_position.x");
            ic_pos_y_ = cfg.get<Scalar>("initial_position.y");
            ic_pos_z_ = cfg.get<Scalar>("initial_position.z");
        }
        if (cfg.has("initial_velocity.x")) {
            ic_vel_x_ = cfg.get<Scalar>("initial_velocity.x");
            ic_vel_y_ = cfg.get<Scalar>("initial_velocity.y");
            ic_vel_z_ = cfg.get<Scalar>("initial_velocity.z");
        }
    }

    /**
     * @brief Bind to global state vector slices
     */
    void BindState(Scalar* state, Scalar* state_dot, std::size_t size) override {
        if (size != kStateSize) {
            throw StateSizeMismatchError(kStateSize, size);
        }

        // Position pointers
        state_pos_ = state + kPosOffset;
        state_dot_pos_ = state_dot + kPosOffset;

        // Velocity pointers
        state_vel_ = state + kVelOffset;
        state_dot_vel_ = state_dot + kVelOffset;

        // Apply initial conditions to state
        state_pos_[0] = ic_pos_x_;
        state_pos_[1] = ic_pos_y_;
        state_pos_[2] = ic_pos_z_;
        state_vel_[0] = ic_vel_x_;
        state_vel_[1] = ic_vel_y_;
        state_vel_[2] = ic_vel_z_;
    }

    /**
     * @brief Compute derivatives (kinematics and dynamics)
     *
     * Called every integration substep. Reads from state pointers,
     * writes derivatives to state_dot pointers.
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;  // Time not needed for simple dynamics
        (void)dt; // dt not needed (integrator handles step)

        // Read current state (via pointers to X_global_)
        Scalar rx = state_pos_[0];
        Scalar ry = state_pos_[1];
        Scalar rz = state_pos_[2];
        Scalar vx = state_vel_[0];
        Scalar vy = state_vel_[1];
        Scalar vz = state_vel_[2];

        // Read acceleration input (from gravity component)
        Scalar ax = *accel_x_;
        Scalar ay = *accel_y_;
        Scalar az = *accel_z_;

        // Write derivatives
        // dr/dt = v (kinematics)
        state_dot_pos_[0] = vx;
        state_dot_pos_[1] = vy;
        state_dot_pos_[2] = vz;

        // dv/dt = a (Newton's 2nd law)
        state_dot_vel_[0] = ax;
        state_dot_vel_[1] = ay;
        state_dot_vel_[2] = az;

        // Update outputs (for other components and logging)
        position_x_ = rx;
        position_y_ = ry;
        position_z_ = rz;
        velocity_x_ = vx;
        velocity_y_ = vy;
        velocity_z_ = vz;
    }

    // =========================================================================
    // Accessors (for testing)
    // =========================================================================

    [[nodiscard]] Scalar GetPositionX() const { return position_x_; }
    [[nodiscard]] Scalar GetPositionY() const { return position_y_; }
    [[nodiscard]] Scalar GetPositionZ() const { return position_z_; }
    [[nodiscard]] Scalar GetVelocityX() const { return velocity_x_; }
    [[nodiscard]] Scalar GetVelocityY() const { return velocity_y_; }
    [[nodiscard]] Scalar GetVelocityZ() const { return velocity_z_; }

    // Set initial conditions programmatically
    void SetInitialPosition(Scalar x, Scalar y, Scalar z) {
        ic_pos_x_ = x;
        ic_pos_y_ = y;
        ic_pos_z_ = z;
    }

    void SetInitialVelocity(Scalar vx, Scalar vy, Scalar vz) {
        ic_vel_x_ = vx;
        ic_vel_y_ = vy;
        ic_vel_z_ = vz;
    }

private:
    // Identity
    std::string name_;
    std::string entity_;

    // Initial conditions (applied in BindState)
    Scalar ic_pos_x_{0};
    Scalar ic_pos_y_{0};
    Scalar ic_pos_z_{0};
    Scalar ic_vel_x_{0};
    Scalar ic_vel_y_{0};
    Scalar ic_vel_z_{0};

    // State pointers (bound in BindState, point into X_global_)
    Scalar* state_pos_ = nullptr;
    Scalar* state_vel_ = nullptr;
    Scalar* state_dot_pos_ = nullptr;
    Scalar* state_dot_vel_ = nullptr;

    // Input handles (resolved in Stage)
    const Scalar* accel_x_ = nullptr;
    const Scalar* accel_y_ = nullptr;
    const Scalar* accel_z_ = nullptr;

    // Output values (published via register_output)
    Scalar position_x_{0};
    Scalar position_y_{0};
    Scalar position_z_{0};
    Scalar velocity_x_{0};
    Scalar velocity_y_{0};
    Scalar velocity_z_{0};
};

} // namespace components
} // namespace icarus
```

---

### 2. PointMassGravity Component

Stateless component that computes gravitational acceleration.

#### File: `components/environment/PointMassGravity.hpp`

```cpp
#pragma once

/**
 * @file PointMassGravity.hpp
 * @brief Simple point-mass gravity model component
 *
 * Part of Phase 2.3: First Real Component
 */

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/core/Types.hpp>
#include <vulcan/gravity/PointMass.hpp>
#include <vulcan/constants/Earth.hpp>
#include <janus/math/Functions.hpp>

namespace icarus {
namespace components {

/**
 * @brief Simple round Earth gravity model
 *
 * Computes gravitational acceleration using point-mass approximation:
 *   g = -μ/|r|² * r̂
 *
 * where μ = GM (gravitational parameter) and r is position vector from
 * Earth's center.
 *
 * For Phase 2.3 validation, can also use simplified constant gravity:
 *   g = [0, 0, -9.81] m/s² (in local vertical frame)
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar>
class PointMassGravity : public Component<Scalar> {
public:
    /**
     * @brief Gravity model mode
     */
    enum class Mode {
        ConstantDownward,  ///< g = [0, 0, -9.81] (for validation)
        PointMass,         ///< g = -μ/|r|² * r̂ (realistic)
    };

    /**
     * @brief Construct with optional name and mode
     */
    explicit PointMassGravity(std::string name = "Gravity",
                             std::string entity = "",
                             Mode mode = Mode::ConstantDownward)
        : name_(std::move(name)), entity_(std::move(entity)), mode_(mode) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "PointMassGravity"; }

    [[nodiscard]] std::size_t StateSize() const override { return 0; }  // Stateless

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register output: acceleration
     */
    void Provision(Backplane<Scalar>& bp, const ComponentConfig&) override {
        bp.register_output("acceleration.x", &accel_x_, "m/s²", "Gravity X");
        bp.register_output("acceleration.y", &accel_y_, "m/s²", "Gravity Y");
        bp.register_output("acceleration.z", &accel_z_, "m/s²", "Gravity Z");
    }

    /**
     * @brief Wire input: position from dynamics component
     */
    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Resolve position input
        pos_x_ = bp.template resolve<Scalar>("PointMass3DOF.position.x");
        pos_y_ = bp.template resolve<Scalar>("PointMass3DOF.position.y");
        pos_z_ = bp.template resolve<Scalar>("PointMass3DOF.position.z");

        // Override mode from config if specified
        if (cfg.has("mode")) {
            std::string mode_str = cfg.get<std::string>("mode");
            if (mode_str == "constant" || mode_str == "ConstantDownward") {
                mode_ = Mode::ConstantDownward;
            } else if (mode_str == "point_mass" || mode_str == "PointMass") {
                mode_ = Mode::PointMass;
            }
        }

        // Override gravitational parameter if specified
        if (cfg.has("mu")) {
            mu_ = cfg.get<Scalar>("mu");
        }

        // Override constant gravity magnitude if specified
        if (cfg.has("g")) {
            g_constant_ = cfg.get<Scalar>("g");
        }
    }

    /**
     * @brief Compute gravitational acceleration
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        if (mode_ == Mode::ConstantDownward) {
            // Simple constant gravity (Z-down convention)
            // Useful for validating against analytical solution
            accel_x_ = Scalar{0};
            accel_y_ = Scalar{0};
            accel_z_ = -g_constant_;  // Negative = downward
        } else {
            // Point-mass gravity model
            Scalar rx = *pos_x_;
            Scalar ry = *pos_y_;
            Scalar rz = *pos_z_;

            // |r| = sqrt(rx² + ry² + rz²)
            Scalar r_mag_sq = rx * rx + ry * ry + rz * rz;
            Scalar r_mag = janus::sqrt(r_mag_sq);

            // g = -μ/|r|³ * r  (note: |r|³ not |r|² because we multiply by r not r̂)
            Scalar factor = -mu_ / (r_mag * r_mag_sq);

            accel_x_ = factor * rx;
            accel_y_ = factor * ry;
            accel_z_ = factor * rz;
        }
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    void SetMode(Mode mode) { mode_ = mode; }
    [[nodiscard]] Mode GetMode() const { return mode_; }

    void SetGravitationalParameter(Scalar mu) { mu_ = mu; }
    void SetConstantGravity(Scalar g) { g_constant_ = g; }

private:
    // Identity
    std::string name_;
    std::string entity_;

    // Configuration
    Mode mode_ = Mode::ConstantDownward;
    Scalar mu_ = Scalar{3.986004418e14};  // Earth GM [m³/s²]
    Scalar g_constant_ = Scalar{9.80665}; // Standard gravity [m/s²]

    // Input handles (resolved in Stage)
    const Scalar* pos_x_ = nullptr;
    const Scalar* pos_y_ = nullptr;
    const Scalar* pos_z_ = nullptr;

    // Output values
    Scalar accel_x_{0};
    Scalar accel_y_{0};
    Scalar accel_z_{0};
};

} // namespace components
} // namespace icarus
```

---

## Integration Example

### File: `examples/falling_mass/main.cpp`

```cpp
/**
 * @file main.cpp
 * @brief Phase 2.3 validation: Falling mass under constant gravity
 *
 * Validates:
 * - Component lifecycle (Provision → Stage → Step)
 * - State management (pointer-based scatter/gather)
 * - Signal wiring (gravity → dynamics)
 * - Integrator (RK4)
 * - Analytical verification: y(t) = y₀ + v₀t - ½gt²
 */

#include <icarus/icarus.hpp>
#include <icarus/components/PointMass3DOF.hpp>
#include <icarus/components/PointMassGravity.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace icarus;
using namespace icarus::components;

int main() {
    std::cout << "=== Phase 2.3 Validation: Falling Mass ===\n\n";

    // =========================================================================
    // Simulation Setup
    // =========================================================================

    // Create simulator
    Simulator<double> sim;

    // Create components
    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    auto point_mass = std::make_unique<PointMass3DOF<double>>("PointMass3DOF");

    // Configure gravity for constant downward (Z-down frame)
    gravity->SetMode(PointMassGravity<double>::Mode::ConstantDownward);

    // Set initial conditions
    // Start at z = 100m, falling from rest
    double y0 = 100.0;  // Initial height [m]
    double v0 = 0.0;    // Initial velocity [m/s]
    point_mass->SetInitialPosition(0.0, 0.0, y0);
    point_mass->SetInitialVelocity(0.0, 0.0, v0);

    // Add components (order matters for signal resolution)
    // PointMass3DOF must provision first so Gravity can resolve its position
    sim.AddComponent(std::move(point_mass));
    sim.AddComponent(std::move(gravity));

    // =========================================================================
    // Lifecycle: Provision → Stage
    // =========================================================================

    sim.Provision();
    sim.Stage();

    // =========================================================================
    // Run Simulation
    // =========================================================================

    double dt = 0.01;      // 10ms timestep
    double t_end = 4.0;    // Run for 4 seconds
    double g = 9.80665;    // Gravity [m/s²]

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Time [s]     z [m]      v_z [m/s]   z_exact    error [m]\n";
    std::cout << "  --------------------------------------------------------\n";

    // Sample output at intervals
    int output_interval = 50;  // Every 0.5s
    int step = 0;

    while (sim.Time() < t_end) {
        // Get current state
        double t = sim.Time();
        double z = sim.GetSignal("PointMass3DOF.position.z");
        double vz = sim.GetSignal("PointMass3DOF.velocity.z");

        // Analytical solution: z(t) = z₀ + v₀t - ½gt²
        double z_exact = y0 + v0 * t - 0.5 * g * t * t;
        double error = std::abs(z - z_exact);

        if (step % output_interval == 0) {
            std::cout << "  " << std::setw(8) << t
                      << "  " << std::setw(10) << z
                      << "  " << std::setw(10) << vz
                      << "  " << std::setw(10) << z_exact
                      << "  " << std::setw(10) << error
                      << "\n";
        }

        // Step simulation
        sim.Step(dt);
        ++step;
    }

    // =========================================================================
    // Final Verification
    // =========================================================================

    double t_final = sim.Time();
    double z_final = sim.GetSignal("PointMass3DOF.position.z");
    double vz_final = sim.GetSignal("PointMass3DOF.velocity.z");

    double z_exact_final = y0 + v0 * t_final - 0.5 * g * t_final * t_final;
    double v_exact_final = v0 - g * t_final;

    double z_error = std::abs(z_final - z_exact_final);
    double v_error = std::abs(vz_final - v_exact_final);

    std::cout << "\n=== Final State (t = " << t_final << " s) ===\n";
    std::cout << "  Position:  z = " << z_final << " m  (exact: " << z_exact_final << " m)\n";
    std::cout << "  Velocity:  v = " << vz_final << " m/s  (exact: " << v_exact_final << " m/s)\n";
    std::cout << "  Position error: " << z_error << " m\n";
    std::cout << "  Velocity error: " << v_error << " m/s\n";

    // Verify accuracy
    bool passed = (z_error < 1e-4) && (v_error < 1e-4);
    std::cout << "\n=== Validation: " << (passed ? "PASSED" : "FAILED") << " ===\n";

    return passed ? 0 : 1;
}
```

---

## Unit Tests

### File: `tests/components/test_point_mass_3dof.cpp`

```cpp
#include <gtest/gtest.h>
#include <icarus/icarus.hpp>
#include <icarus/components/PointMass3DOF.hpp>
#include <icarus/components/PointMassGravity.hpp>
#include <cmath>

using namespace icarus;
using namespace icarus::components;

// ---------------------------------------------------------------------------
// PointMass3DOF Unit Tests
// ---------------------------------------------------------------------------

TEST(PointMass3DOF, StateSize) {
    PointMass3DOF<double> pm;
    EXPECT_EQ(pm.StateSize(), 6);
    EXPECT_TRUE(pm.HasState());
}

TEST(PointMass3DOF, Identity) {
    PointMass3DOF<double> pm("TestMass", "Vehicle");
    EXPECT_EQ(pm.Name(), "TestMass");
    EXPECT_EQ(pm.Entity(), "Vehicle");
    EXPECT_EQ(pm.TypeName(), "PointMass3DOF");
}

TEST(PointMass3DOF, InitialConditions) {
    PointMass3DOF<double> pm;
    pm.SetInitialPosition(1.0, 2.0, 3.0);
    pm.SetInitialVelocity(4.0, 5.0, 6.0);

    // Bind state manually to verify IC application
    double state[6];
    double state_dot[6];
    pm.BindState(state, state_dot, 6);

    EXPECT_DOUBLE_EQ(state[0], 1.0);
    EXPECT_DOUBLE_EQ(state[1], 2.0);
    EXPECT_DOUBLE_EQ(state[2], 3.0);
    EXPECT_DOUBLE_EQ(state[3], 4.0);
    EXPECT_DOUBLE_EQ(state[4], 5.0);
    EXPECT_DOUBLE_EQ(state[5], 6.0);
}

TEST(PointMass3DOF, StateSizeMismatchThrows) {
    PointMass3DOF<double> pm;
    double state[4];
    double state_dot[4];

    EXPECT_THROW(pm.BindState(state, state_dot, 4), StateSizeMismatchError);
}

// ---------------------------------------------------------------------------
// PointMassGravity Unit Tests
// ---------------------------------------------------------------------------

TEST(PointMassGravity, StateSize) {
    PointMassGravity<double> grav;
    EXPECT_EQ(grav.StateSize(), 0);
    EXPECT_FALSE(grav.HasState());
}

TEST(PointMassGravity, Identity) {
    PointMassGravity<double> grav("TestGravity", "Environment");
    EXPECT_EQ(grav.Name(), "TestGravity");
    EXPECT_EQ(grav.Entity(), "Environment");
    EXPECT_EQ(grav.TypeName(), "PointMassGravity");
}

TEST(PointMassGravity, ModeSwitch) {
    PointMassGravity<double> grav;

    grav.SetMode(PointMassGravity<double>::Mode::ConstantDownward);
    EXPECT_EQ(grav.GetMode(), PointMassGravity<double>::Mode::ConstantDownward);

    grav.SetMode(PointMassGravity<double>::Mode::PointMass);
    EXPECT_EQ(grav.GetMode(), PointMassGravity<double>::Mode::PointMass);
}

// ---------------------------------------------------------------------------
// Integration Tests
// ---------------------------------------------------------------------------

TEST(PointMass3DOFIntegration, FreeFallValidation) {
    Simulator<double> sim;

    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    auto point_mass = std::make_unique<PointMass3DOF<double>>("PointMass3DOF");

    gravity->SetMode(PointMassGravity<double>::Mode::ConstantDownward);

    double y0 = 100.0;
    double v0 = 0.0;
    point_mass->SetInitialPosition(0.0, 0.0, y0);
    point_mass->SetInitialVelocity(0.0, 0.0, v0);

    sim.AddComponent(std::move(point_mass));
    sim.AddComponent(std::move(gravity));
    sim.Provision();
    sim.Stage();

    double dt = 0.01;
    double g = 9.80665;

    // Run for 2 seconds
    for (int i = 0; i < 200; ++i) {
        sim.Step(dt);
    }

    double t = sim.Time();
    double z = sim.GetSignal("PointMass3DOF.position.z");
    double vz = sim.GetSignal("PointMass3DOF.velocity.z");

    // Analytical: z(t) = z₀ + v₀t - ½gt²
    double z_exact = y0 + v0 * t - 0.5 * g * t * t;
    double v_exact = v0 - g * t;

    EXPECT_NEAR(z, z_exact, 1e-4);
    EXPECT_NEAR(vz, v_exact, 1e-4);
}

TEST(PointMass3DOFIntegration, ProjectileMotion) {
    Simulator<double> sim;

    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    auto point_mass = std::make_unique<PointMass3DOF<double>>("PointMass3DOF");

    gravity->SetMode(PointMassGravity<double>::Mode::ConstantDownward);

    // Launch at 45 degrees with v = 10 m/s
    double v_initial = 10.0;
    double angle = M_PI / 4.0;  // 45 degrees
    double vx0 = v_initial * std::cos(angle);
    double vz0 = v_initial * std::sin(angle);

    point_mass->SetInitialPosition(0.0, 0.0, 0.0);
    point_mass->SetInitialVelocity(vx0, 0.0, vz0);

    sim.AddComponent(std::move(point_mass));
    sim.AddComponent(std::move(gravity));
    sim.Provision();
    sim.Stage();

    double dt = 0.001;  // Small timestep for accuracy
    double g = 9.80665;

    // Time of flight: t = 2 * vz0 / g
    double t_flight = 2.0 * vz0 / g;

    // Run until projectile lands
    while (sim.Time() < t_flight + 0.1) {
        sim.Step(dt);
    }

    // Check range: R = v²sin(2θ)/g
    double range_exact = v_initial * v_initial * std::sin(2 * angle) / g;
    double x_final = sim.GetSignal("PointMass3DOF.position.x");

    // Should be close to analytical range (within 1%)
    EXPECT_NEAR(x_final, range_exact, range_exact * 0.01);
}

TEST(PointMass3DOFIntegration, EnergyConservation) {
    // In free fall with no friction, mechanical energy should be conserved
    Simulator<double> sim;

    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    auto point_mass = std::make_unique<PointMass3DOF<double>>("PointMass3DOF");

    gravity->SetMode(PointMassGravity<double>::Mode::ConstantDownward);

    double z0 = 100.0;
    double v0 = 10.0;  // Initial downward velocity
    point_mass->SetInitialPosition(0.0, 0.0, z0);
    point_mass->SetInitialVelocity(0.0, 0.0, v0);

    sim.AddComponent(std::move(point_mass));
    sim.AddComponent(std::move(gravity));
    sim.Provision();
    sim.Stage();

    double g = 9.80665;

    // Initial energy: E = ½v² + gh (per unit mass)
    double E_initial = 0.5 * v0 * v0 + g * z0;

    double dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        sim.Step(dt);

        double z = sim.GetSignal("PointMass3DOF.position.z");
        double vz = sim.GetSignal("PointMass3DOF.velocity.z");

        double E_current = 0.5 * vz * vz + g * z;

        // Energy should be conserved (within numerical tolerance)
        EXPECT_NEAR(E_current, E_initial, 1e-3);
    }
}

// ---------------------------------------------------------------------------
// Symbolic Mode Tests
// ---------------------------------------------------------------------------

TEST(PointMass3DOFSymbolic, CompilationTest) {
    using MX = casadi::MX;

    Simulator<MX> sim;

    auto gravity = std::make_unique<PointMassGravity<MX>>("Gravity");
    auto point_mass = std::make_unique<PointMass3DOF<MX>>("PointMass3DOF");

    sim.AddComponent(std::move(point_mass));
    sim.AddComponent(std::move(gravity));

    EXPECT_NO_THROW(sim.Provision());
    EXPECT_NO_THROW(sim.Stage());
    EXPECT_NO_THROW(sim.Step(MX{0.01}));
}
```

---

## Implementation Checklist

### Task 2.3a: PointMass3DOF Component

- [ ] Create `components/dynamics/` directory
- [ ] Create `PointMass3DOF.hpp` header
- [ ] Implement `StateSize()` returning 6
- [ ] Implement `Provision()` registering position/velocity outputs
- [ ] Implement `Stage()` resolving acceleration input
- [ ] Implement `BindState()` with IC application
- [ ] Implement `Step()` computing kinematics/dynamics
- [ ] Add IC setters (`SetInitialPosition`, `SetInitialVelocity`)
- [ ] Add state accessors for testing

### Task 2.3b: PointMassGravity Component

- [ ] Create `components/environment/` directory
- [ ] Create `PointMassGravity.hpp` header
- [ ] Implement `StateSize()` returning 0 (stateless)
- [ ] Implement `Provision()` registering acceleration output
- [ ] Implement `Stage()` resolving position input
- [ ] Implement `Step()` with constant gravity mode
- [ ] Implement `Step()` with point-mass gravity mode
- [ ] Add mode switching API

### Task 2.3c: Example Application

- [ ] Create `examples/falling_mass/` directory
- [ ] Create `main.cpp` with simulation setup
- [ ] Add CMakeLists.txt for example
- [ ] Implement analytical verification output
- [ ] Add to top-level examples build

### Task 2.3d: Unit Tests

- [ ] Create `tests/components/test_point_mass_3dof.cpp`
- [ ] Add PointMass3DOF identity/state size tests
- [ ] Add PointMass3DOF initial condition tests
- [ ] Add PointMassGravity identity/mode tests
- [ ] Add free fall analytical validation test
- [ ] Add projectile motion test
- [ ] Add energy conservation test
- [ ] Add symbolic mode compilation test
- [ ] Update `tests/CMakeLists.txt`

### Task 2.3e: Build Integration

- [ ] Update `components/CMakeLists.txt` with new components
- [ ] Update `include/icarus/icarus.hpp` with component headers
- [ ] Add `include/icarus/components/PointMass3DOF.hpp` (forwarding header)
- [ ] Add `include/icarus/components/PointMassGravity.hpp` (forwarding header)
- [ ] Verify `./scripts/build.sh` succeeds
- [ ] Verify `./scripts/test.sh` all pass
- [ ] Verify `./scripts/run_examples.sh` includes falling_mass

### Task 2.3f: Documentation

- [ ] Update main implementation plan checkboxes
- [ ] Mark Phase 2.3 complete when all exit criteria pass

---

## Design Decisions

### 1. Component Ordering

**Decision:** PointMass3DOF provisions before PointMassGravity so gravity can resolve position.

**Rationale:**
- Signal resolution requires outputs to exist
- In Provision, PointMass3DOF creates `position.{x,y,z}` signals
- In Stage, PointMassGravity resolves those signals
- Component add order determines provision/stage order

**Alternative:** Use explicit dependency declaration (future enhancement).

### 2. Constant Gravity Mode

**Decision:** Include `ConstantDownward` mode for analytical validation.

**Rationale:**
- Free-fall analytical solution assumes constant `g`
- Easier to validate RK4 accuracy
- Point-mass gravity has different solution (not closed-form)
- Switch to point-mass for realistic simulations

### 3. Separate Position and Velocity Outputs

**Decision:** Register individual scalar outputs (`position.x`, etc.) rather than Vec3.

**Rationale:**
- Simpler signal resolution for Phase 2.3
- Vec3 signal support (`register_vec3`) is a Phase 5+ enhancement
- Individual scalars work with current Backplane API

**Future:** Add `bp.register_vec3("position", &position_vec_)` convenience.

### 4. Initial Conditions in BindState

**Decision:** Apply initial conditions when state is bound, not in Stage.

**Rationale:**
- State pointers aren't available until `BindState()` is called
- IC values set via setters or config before Stage
- BindState is the first moment we have write access to X_global_

### 5. Z-Down Coordinate Frame

**Decision:** Use Z-positive-up, gravity is negative-Z.

**Rationale:**
- Matches common aerospace conventions (NED: Z-down is alternative)
- Analytical solution `z = z₀ - ½gt²` is intuitive
- Can switch to ECEF for orbital simulations

---

## Janus Compatibility Checklist

- [ ] All component code templated on `Scalar`
- [ ] Use `janus::sqrt()` instead of `std::sqrt()` in gravity computation
- [ ] No `if/else` branching on `Scalar` values in traced code
- [ ] Mode switching uses structural branching (not Scalar-dependent)
- [ ] Verify `Simulator<casadi::MX>` compiles and runs
- [ ] Verify symbolic step produces valid MX expressions

---

## Exit Criteria Verification

### 1. Simulate a falling object

**Test:** `PointMass3DOFIntegration.FreeFallValidation`

- Creates Simulator with PointMass3DOF + PointMassGravity
- Runs for 2 seconds with dt=0.01
- Verifies position and velocity match expectations

### 2. Verify against analytical solution

**Analytical Solution:**
```
z(t) = z₀ + v₀t - ½gt²
v(t) = v₀ - gt
```

**Verification:**
```cpp
double z_exact = y0 + v0 * t - 0.5 * g * t * t;
double v_exact = v0 - g * t;
EXPECT_NEAR(z, z_exact, 1e-4);
EXPECT_NEAR(vz, v_exact, 1e-4);
```

### 3. State scattered/gathered correctly

**Verification:**
- `PointMass3DOF::BindState()` receives pointers into X_global_
- `Step()` reads from state pointers, writes derivatives
- Integrator calls `ComputeDerivatives()`, state updates correctly
- Test: `StateBindingCorrectOffsets` from Phase 2.1

---

## Dependencies

| Dependency | Purpose | Status |
|:-----------|:--------|:-------|
| Phase 2.1 | State management | ✅ Complete |
| Phase 2.2 | Integrator interface | ✅ Complete |
| Phase 1.4 | Component base | ✅ Complete |
| Phase 1.5 | Simulator shell | ✅ Complete |
| Vulcan gravity | Point-mass gravity | ✅ Available |

---

## Next Steps (Phase 3)

After Phase 2.3 completes, Phase 3 (Symbolic Mode) will:

1. Instantiate `Simulator<casadi::MX>` with same components
2. Verify symbolic compilation (no template violations)
3. Extract dynamics as `casadi::Function`
4. Compare numeric/symbolic outputs
5. Generate Jacobians via AD

The `PointMass3DOF` component becomes the first **dual-mode validated** component in the codebase.

---

## References

| Topic | Document |
|:------|:---------|
| Component protocol | [02_component_protocol.md](../../architecture/02_component_protocol.md) |
| State ownership | [09_memory_state_ownership.md](../../architecture/09_memory_state_ownership.md) |
| Vulcan integration | [08_vulcan_integration.md](../../architecture/08_vulcan_integration.md) |
| Lifecycle | [04_lifecycle.md](../../architecture/04_lifecycle.md) |
| Janus integration | [07_janus_integration.md](../../architecture/07_janus_integration.md) |
| Symbolic constraints | [21_symbolic_constraints.md](../../architecture/21_symbolic_constraints.md) |
