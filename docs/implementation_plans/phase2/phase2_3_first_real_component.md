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
5. **Vulcan integration** — Using physics utilities for gravity and dynamics

Two components demonstrate the architecture:

| Component | Role | State | Inputs | Outputs | Vulcan Functions |
|:----------|:-----|:------|:-------|:--------|:-----------------|
| **PointMass3DOF** | Dynamics | 6 (pos + vel) | `force` | `position`, `velocity` | `point_mass_acceleration()` (ECI) or `point_mass_acceleration_ecef()` |
| **PointMassGravity** | Environment | 0 | `position`, `mass` | `force` | `gravity::point_mass::acceleration()` (ECEF-compatible) |

```
┌──────────────────────────────────────────────────────────────────┐
│                    PHASE 2.3 COMPONENT DIAGRAM                    │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│   PointMassGravity (Stateless)                                  │
│   ┌─────────────────────────────┐                               │
│   │ position ─────────────────┐ │                               │
│   │ mass ────────────────────┐│ │                               │
│   │                          ││ │                               │
│   │  Vulcan: gravity::point_mass::acceleration(r)               │
│   │  force = mass * g        ▼▼ │                               │
│   │                      force ───────────────────┐             │
│   └─────────────────────────────┘                 │             │
│                                                   │             │
│   PointMass3DOF (Stateful)                        │             │
│   ┌─────────────────────────────┐                 │             │
│   │ ┌─────────────────────────┐ │                 │             │
│   │ │ State: r, v (6 vars)    │ │◄────────────────┘             │
│   │ └─────────────────────────┘ │                               │
│   │                             │                               │
│   │  Vulcan: point_mass_acceleration(force, mass)               │
│   │  ṙ = v                      │                               │
│   │  v̇ = a = F/m               │                               │
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

Vulcan provides stateless physics functions that components wrap:

**Gravity Model** (`vulcan/gravity/PointMass.hpp`):

```cpp
namespace vulcan::gravity::point_mass {
    // Gravitational acceleration: g = -μ/r³ · r
    template <typename Scalar>
    Vec3<Scalar> acceleration(const Vec3<Scalar>& r_ecef,
                             double mu = constants::earth::mu);

    // Gravitational potential: U = -μ/r
    template <typename Scalar>
    Scalar potential(const Vec3<Scalar>& r_ecef,
                    double mu = constants::earth::mu);
}
```

**Point Mass Dynamics** (`vulcan/dynamics/PointMass.hpp`):

```cpp
namespace vulcan::dynamics {
    // Inertial frame acceleration: a = F/m
    template <typename Scalar>
    Vec3<Scalar> point_mass_acceleration(const Vec3<Scalar>& force,
                                        const Scalar& mass);

    // ECEF frame with Coriolis & centrifugal terms
    template <typename Scalar>
    Vec3<Scalar> point_mass_acceleration_ecef(const Vec3<Scalar>& position,
                                             const Vec3<Scalar>& velocity,
                                             const Vec3<Scalar>& force,
                                             const Scalar& mass,
                                             const Vec3<Scalar>& omega_earth);

    // Kinetic/potential energy
    template <typename Scalar>
    Scalar specific_energy(const Vec3<Scalar>& position,
                          const Vec3<Scalar>& velocity,
                          const Scalar& mu);
}
```

**Coordinate Frames** (`vulcan/coordinates/`):

```cpp
// LLA ↔ ECEF conversions
template <typename Scalar>
LLA<Scalar> ecef_to_lla(const Vec3<Scalar>& r, const EarthModel& m);

template <typename Scalar>
Vec3<Scalar> lla_to_ecef(const LLA<Scalar>& lla, const EarthModel& m);

// Local frames (NED, ENU)
template <typename Scalar>
CoordinateFrame<Scalar> local_ned_at(const Vec3<Scalar>& r_ecef);
```

**Constants** (`vulcan/core/Constants.hpp`):

```cpp
namespace vulcan::constants::earth {
    inline constexpr double mu = 3.986004418e14;    // GM [m³/s²]
    inline constexpr double R_eq = 6378137.0;       // Equatorial radius [m]
    inline constexpr double omega = 7.2921159e-5;   // Rotation rate [rad/s]
    inline constexpr double g0 = 9.80665;           // Standard gravity [m/s²]
}
```

---

## Coordinate Frame Strategy

> [!IMPORTANT]
> Coordinate frame consistency is critical. All components must agree on which frame state is propagated in.

### Frame Options

| Frame | Description | Dynamics Function | When to Use |
|:------|:------------|:------------------|:------------|
| **ECI** | Earth-Centered Inertial | `point_mass_acceleration(F, m)` | Orbital, no Earth rotation effects |
| **ECEF** | Earth-Centered Earth-Fixed | `point_mass_acceleration_ecef(r, v, F, m, ω)` | Ground-referenced, includes Coriolis |
| **Local/NED** | Local vertical frame | Custom formulation | Atmospheric flight, short duration |

### Phase 2.3 Approach: Dual-Mode Support

To maximize validation coverage and educational value:

1. **Validation Mode (Constant Gravity)**: Uses a simplified **local vertical frame** where:
   - Position is in a local Cartesian frame (X-East, Y-North, Z-Up or similar)
   - Gravity is constant: `g = [0, 0, -g0]`
   - No frame rotation effects
   - Validates against analytical solution: `z(t) = z₀ + v₀t - ½gt²`

2. **Orbital Mode (Point-Mass Gravity)**: Uses **ECI frame** where:
   - Position/velocity are inertial (non-rotating)
   - Gravity: `g = -μ/r³ · r` (central force, same formula in any frame)
   - Uses `point_mass_acceleration(F, m)` without fictitious forces
   - Validates via energy conservation

> [!NOTE]
> Future phases will add ECEF propagation with `point_mass_acceleration_ecef()` for
> Earth-relative trajectory analysis (e.g., launch trajectories, reentry).

### Frame Consistency Rules

```
┌─────────────────────────────────────────────────────────────────────┐
│                    FRAME CONSISTENCY FLOW                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   PointMassGravity.Step()                                          │
│   ├─ Reads: position (frame F)                                     │
│   ├─ Computes: g = vulcan::gravity::...(r)      // Output in F     │
│   └─ Outputs: force = m * g                     // Force in F      │
│                        │                                           │
│                        ▼                                           │
│   PointMass3DOF.Step()                                             │
│   ├─ Reads: force (frame F)                                        │
│   ├─ Computes: a = F/m (ECI) or with ω×v, ω×ω×r (ECEF)            │
│   └─ Writes: dr/dt = v, dv/dt = a               // Derivs in F    │
│                                                                     │
│   Integration                                                       │
│   └─ Updates: r, v via RK4                      // State in F     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Why Not ECEF for Phase 2.3?

ECEF propagation requires:

- Coriolis term: `-2ω × v`
- Centrifugal term: `-ω × (ω × r)`
- Earth rotation vector: `ω = [0, 0, 7.2921159e-5]` rad/s

This adds complexity without improving the core validation goals of Phase 2.3.
ECI is simpler and sufficient for demonstrating:

- Component lifecycle
- State management
- Signal wiring
- Integrator correctness

ECEF support will be added in a later phase focused on coordinate frame infrastructure.

---

## Exit Criteria (From Main Plan)

- [ ] Simulate a falling object (point mass under gravity)
- [ ] Verify against analytical solution: `y(t) = y₀ + v₀t - ½gt²`
- [ ] State correctly scattered/gathered through integrator

---

## Component Design

### 1. PointMass3DOF Component

The core dynamics component with 6 state variables (position + velocity). Uses Vulcan's `point_mass_acceleration()` for physics in **inertial frames** (ECI or local).

> [!NOTE]
> This component uses inertial-frame dynamics (no Coriolis/centrifugal terms).
> For ECEF propagation, a future variant will use `point_mass_acceleration_ecef()`.

#### File: `components/dynamics/PointMass3DOF.hpp`

```cpp
#pragma once

/**
 * @file PointMass3DOF.hpp
 * @brief 3-DOF point mass dynamics component (inertial frame)
 *
 * Wraps Vulcan's point_mass_acceleration() for translational dynamics.
 * Suitable for ECI orbital dynamics or local-frame validation.
 *
 * Frame: Inertial (ECI) or local Cartesian (for validation)
 * NOT suitable for ECEF without adding Coriolis/centrifugal terms.
 *
 * Part of Phase 2.3: First Real Component
 */

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/core/Types.hpp>
#include <vulcan/dynamics/PointMass.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace icarus {
namespace components {

/**
 * @brief 3-DOF point mass translational dynamics (inertial frame)
 *
 * State vector: [rx, ry, rz, vx, vy, vz] (6 variables)
 *
 * Frame Assumption: This component assumes an INERTIAL reference frame.
 * - For orbital dynamics: use ECI coordinates
 * - For validation tests: use local Cartesian (Z-up)
 *
 * Uses Vulcan for physics:
 *   - vulcan::dynamics::point_mass_acceleration(force, mass) → a = F/m
 *
 * Equations of motion:
 *   dr/dt = v           (kinematics)
 *   dv/dt = a = F/m     (Newton's 2nd law via Vulcan)
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
     * @brief Construct with mass and optional name/entity
     */
    explicit PointMass3DOF(Scalar mass = Scalar{1.0},
                          std::string name = "PointMass3DOF",
                          std::string entity = "")
        : mass_(mass), name_(std::move(name)), entity_(std::move(entity)) {}

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
     * @brief Register outputs: position, velocity, mass
     */
    void Provision(Backplane<Scalar>& bp, const ComponentConfig&) override {
        // Position (in inertial frame: ECI for orbital, local for validation)
        bp.register_output("position.x", &position_.x(), "m", "Position X");
        bp.register_output("position.y", &position_.y(), "m", "Position Y");
        bp.register_output("position.z", &position_.z(), "m", "Position Z");

        // Velocity (in same inertial frame as position)
        bp.register_output("velocity.x", &velocity_.x(), "m/s", "Velocity X");
        bp.register_output("velocity.y", &velocity_.y(), "m/s", "Velocity Y");
        bp.register_output("velocity.z", &velocity_.z(), "m/s", "Velocity Z");

        // Mass (for gravity component to compute force)
        bp.register_output("mass", &mass_, "kg", "Point mass");
    }

    /**
     * @brief Wire force input and apply initial conditions
     */
    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Resolve force input (from gravity and other force sources)
        force_x_ = bp.template resolve<Scalar>("Gravity.force.x");
        force_y_ = bp.template resolve<Scalar>("Gravity.force.y");
        force_z_ = bp.template resolve<Scalar>("Gravity.force.z");

        // Apply initial conditions from config (or use programmatic values)
        if (cfg.has("mass")) {
            mass_ = cfg.template get<Scalar>("mass");
        }
    }

    /**
     * @brief Bind to global state vector slices
     */
    void BindState(Scalar* state, Scalar* state_dot, std::size_t size) override {
        if (size != kStateSize) {
            throw StateSizeMismatchError(kStateSize, size);
        }

        // Store pointers to state slices
        state_pos_ = state + kPosOffset;
        state_dot_pos_ = state_dot + kPosOffset;
        state_vel_ = state + kVelOffset;
        state_dot_vel_ = state_dot + kVelOffset;

        // Apply initial conditions to state
        state_pos_[0] = ic_position_.x();
        state_pos_[1] = ic_position_.y();
        state_pos_[2] = ic_position_.z();
        state_vel_[0] = ic_velocity_.x();
        state_vel_[1] = ic_velocity_.y();
        state_vel_[2] = ic_velocity_.z();
    }

    /**
     * @brief Compute derivatives using Vulcan dynamics
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read current state into Vec3 (for Vulcan API)
        vulcan::Vec3<Scalar> pos{state_pos_[0], state_pos_[1], state_pos_[2]};
        vulcan::Vec3<Scalar> vel{state_vel_[0], state_vel_[1], state_vel_[2]};

        // Read force input
        vulcan::Vec3<Scalar> force{*force_x_, *force_y_, *force_z_};

        // Compute acceleration using Vulcan
        vulcan::Vec3<Scalar> accel = vulcan::dynamics::point_mass_acceleration(
            force, mass_);

        // Write derivatives
        // dr/dt = v (kinematics)
        state_dot_pos_[0] = vel.x();
        state_dot_pos_[1] = vel.y();
        state_dot_pos_[2] = vel.z();

        // dv/dt = a = F/m (Vulcan computes this)
        state_dot_vel_[0] = accel.x();
        state_dot_vel_[1] = accel.y();
        state_dot_vel_[2] = accel.z();

        // Update outputs for other components
        position_ = pos;
        velocity_ = vel;
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    void SetMass(Scalar mass) { mass_ = mass; }
    [[nodiscard]] Scalar GetMass() const { return mass_; }

    void SetInitialPosition(const vulcan::Vec3<Scalar>& pos) { ic_position_ = pos; }
    void SetInitialPosition(Scalar x, Scalar y, Scalar z) {
        ic_position_ = vulcan::Vec3<Scalar>{x, y, z};
    }

    void SetInitialVelocity(const vulcan::Vec3<Scalar>& vel) { ic_velocity_ = vel; }
    void SetInitialVelocity(Scalar vx, Scalar vy, Scalar vz) {
        ic_velocity_ = vulcan::Vec3<Scalar>{vx, vy, vz};
    }

    // Accessors
    [[nodiscard]] vulcan::Vec3<Scalar> GetPosition() const { return position_; }
    [[nodiscard]] vulcan::Vec3<Scalar> GetVelocity() const { return velocity_; }

private:
    // Identity
    std::string name_;
    std::string entity_;

    // Mass property
    Scalar mass_{1.0};

    // Initial conditions
    vulcan::Vec3<Scalar> ic_position_{Scalar{0}, Scalar{0}, Scalar{0}};
    vulcan::Vec3<Scalar> ic_velocity_{Scalar{0}, Scalar{0}, Scalar{0}};

    // State pointers (bound in BindState)
    Scalar* state_pos_ = nullptr;
    Scalar* state_vel_ = nullptr;
    Scalar* state_dot_pos_ = nullptr;
    Scalar* state_dot_vel_ = nullptr;

    // Input handles (resolved in Stage)
    const Scalar* force_x_ = nullptr;
    const Scalar* force_y_ = nullptr;
    const Scalar* force_z_ = nullptr;

    // Output values
    vulcan::Vec3<Scalar> position_{Scalar{0}, Scalar{0}, Scalar{0}};
    vulcan::Vec3<Scalar> velocity_{Scalar{0}, Scalar{0}, Scalar{0}};
};

} // namespace components
} // namespace icarus
```

---

### 2. PointMassGravity Component

Stateless component that computes gravitational force using Vulcan's `gravity::point_mass::acceleration()`.

> [!NOTE]
> Frame considerations by gravity model:
>
> - **Constant**: Assumes local vertical frame (Z-up), returns `[0, 0, -g0]`
> - **PointMass/J2**: Expects position from Earth center, returns acceleration toward center (works in ECI or ECEF)

#### File: `components/environment/PointMassGravity.hpp`

```cpp
#pragma once

/**
 * @file PointMassGravity.hpp
 * @brief Gravity model component using Vulcan's point-mass gravity
 *
 * Wraps vulcan::gravity::point_mass::acceleration() for gravity computation.
 *
 * Frame Considerations:
 * - Constant model: Assumes local vertical frame (Z-up)
 * - PointMass/J2 models: Input/output vectors relative to Earth center
 *   (valid in both ECI and ECEF since gravity is a central force)
 *
 * Part of Phase 2.3: First Real Component
 */

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/core/Types.hpp>
#include <vulcan/gravity/PointMass.hpp>
#include <vulcan/gravity/J2.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace icarus {
namespace components {

/**
 * @brief Gravity model component using Vulcan physics
 *
 * Uses Vulcan's gravity models:
 *   - vulcan::gravity::point_mass::acceleration(r) → g = -μ/r³ · r
 *   - vulcan::gravity::j2::acceleration(r) → includes J2 oblateness (optional)
 *
 * Outputs gravitational **force** (not acceleration) so it can be summed
 * with other forces before passing to the dynamics component.
 *
 * For Phase 2.3 validation, also supports simplified constant gravity.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar>
class PointMassGravity : public Component<Scalar> {
public:
    /**
     * @brief Gravity model fidelity
     */
    enum class Model {
        Constant,   ///< g = [0, 0, -g0] (for analytical validation)
        PointMass,  ///< vulcan::gravity::point_mass::acceleration()
        J2,         ///< vulcan::gravity::j2::acceleration()
    };

    /**
     * @brief Construct with optional name and model
     */
    explicit PointMassGravity(std::string name = "Gravity",
                             std::string entity = "",
                             Model model = Model::Constant)
        : name_(std::move(name)), entity_(std::move(entity)), model_(model) {}

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
     * @brief Register outputs: force (N)
     */
    void Provision(Backplane<Scalar>& bp, const ComponentConfig&) override {
        // Output force (not acceleration) - allows summing multiple force sources
        bp.register_output("force.x", &force_.x(), "N", "Gravity force X");
        bp.register_output("force.y", &force_.y(), "N", "Gravity force Y");
        bp.register_output("force.z", &force_.z(), "N", "Gravity force Z");

        // Also output acceleration for convenience/logging
        bp.register_output("acceleration.x", &accel_.x(), "m/s²", "Gravity accel X");
        bp.register_output("acceleration.y", &accel_.y(), "m/s²", "Gravity accel Y");
        bp.register_output("acceleration.z", &accel_.z(), "m/s²", "Gravity accel Z");
    }

    /**
     * @brief Wire inputs: position and mass from dynamics component
     */
    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Resolve position input (from dynamics component)
        pos_x_ = bp.template resolve<Scalar>("PointMass3DOF.position.x");
        pos_y_ = bp.template resolve<Scalar>("PointMass3DOF.position.y");
        pos_z_ = bp.template resolve<Scalar>("PointMass3DOF.position.z");

        // Resolve mass input (for force = mass * acceleration)
        mass_ = bp.template resolve<Scalar>("PointMass3DOF.mass");

        // Override model from config if specified
        if (cfg.has("model")) {
            std::string model_str = cfg.template get<std::string>("model");
            if (model_str == "constant") {
                model_ = Model::Constant;
            } else if (model_str == "point_mass") {
                model_ = Model::PointMass;
            } else if (model_str == "j2") {
                model_ = Model::J2;
            }
        }

        // Override gravitational parameter if specified
        if (cfg.has("mu")) {
            mu_ = cfg.template get<double>("mu");
        }
    }

    /**
     * @brief Compute gravitational force using Vulcan
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read position into Vec3
        vulcan::Vec3<Scalar> pos{*pos_x_, *pos_y_, *pos_z_};
        Scalar m = *mass_;

        // Compute acceleration based on model
        switch (model_) {
            case Model::Constant:
                // Local vertical frame: Z-up, constant gravity downward
                // Used for analytical validation: z(t) = z0 - ½gt²
                // NOTE: Only valid with local-frame dynamics, NOT for orbital sims
                accel_ = vulcan::Vec3<Scalar>{
                    Scalar{0},
                    Scalar{0},
                    -Scalar{vulcan::constants::earth::g0}
                };
                break;

            case Model::PointMass:
                // Central gravity toward Earth center: g = -μ/r³ · r
                // Works in ECI or ECEF (central force, same formula)
                // Position must be from Earth center [m]
                accel_ = vulcan::gravity::point_mass::acceleration(pos, mu_);
                break;

            case Model::J2:
                // J2 gravity model (includes Earth oblateness)
                // Adds latitude-dependent perturbation on top of point-mass
                // Position must be from Earth center [m]
                accel_ = vulcan::gravity::j2::acceleration(pos, mu_);
                break;
        }

        // Compute force: F = m * a
        force_ = m * accel_;
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    void SetModel(Model model) { model_ = model; }
    [[nodiscard]] Model GetModel() const { return model_; }

    void SetGravitationalParameter(double mu) { mu_ = mu; }
    [[nodiscard]] double GetGravitationalParameter() const { return mu_; }

    // Accessors
    [[nodiscard]] vulcan::Vec3<Scalar> GetAcceleration() const { return accel_; }
    [[nodiscard]] vulcan::Vec3<Scalar> GetForce() const { return force_; }

private:
    // Identity
    std::string name_;
    std::string entity_;

    // Configuration
    Model model_ = Model::Constant;
    double mu_ = vulcan::constants::earth::mu;

    // Input handles (resolved in Stage)
    const Scalar* pos_x_ = nullptr;
    const Scalar* pos_y_ = nullptr;
    const Scalar* pos_z_ = nullptr;
    const Scalar* mass_ = nullptr;

    // Output values
    vulcan::Vec3<Scalar> accel_{Scalar{0}, Scalar{0}, Scalar{0}};
    vulcan::Vec3<Scalar> force_{Scalar{0}, Scalar{0}, Scalar{0}};
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
 * FRAME: Local vertical (Z-up, constant gravity)
 * This is NOT orbital dynamics - it's a simplified validation case.
 *
 * Validates:
 * - Component lifecycle (Provision → Stage → Step)
 * - State management (pointer-based scatter/gather)
 * - Signal wiring (gravity → dynamics)
 * - Vulcan physics integration
 * - Integrator (RK4)
 * - Analytical verification: z(t) = z₀ + v₀t - ½gt²
 */

#include <icarus/icarus.hpp>
#include <icarus/components/PointMass3DOF.hpp>
#include <icarus/components/PointMassGravity.hpp>
#include <vulcan/core/Constants.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace icarus;
using namespace icarus::components;

int main() {
    std::cout << "=== Phase 2.3 Validation: Falling Mass ===\n\n";

    // =========================================================================
    // Simulation Setup (Local Vertical Frame)
    // =========================================================================
    // Frame: Local Cartesian with Z pointing UP (not ECEF/ECI)
    // Gravity: Constant -g0 in Z direction
    // Valid for: Short-duration, ground-relative motion

    Simulator<double> sim;

    // Create dynamics component with mass
    double mass = 1.0;  // kg
    auto point_mass = std::make_unique<PointMass3DOF<double>>(mass, "PointMass3DOF");

    // Set initial conditions: start at z = 100m, falling from rest
    double z0 = 100.0;  // Initial height [m]
    double v0 = 0.0;    // Initial velocity [m/s]
    point_mass->SetInitialPosition(0.0, 0.0, z0);
    point_mass->SetInitialVelocity(0.0, 0.0, v0);

    // Create gravity component using Vulcan's constant model
    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    gravity->SetModel(PointMassGravity<double>::Model::Constant);

    // Add components (order matters: dynamics first so gravity can resolve position)
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
    double g = vulcan::constants::earth::g0;  // Use Vulcan's constant

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Time [s]     z [m]      v_z [m/s]   z_exact    error [m]\n";
    std::cout << "  --------------------------------------------------------\n";

    int output_interval = 50;  // Every 0.5s
    int step = 0;

    while (sim.Time() < t_end) {
        double t = sim.Time();
        double z = sim.GetSignal("PointMass3DOF.position.z");
        double vz = sim.GetSignal("PointMass3DOF.velocity.z");

        // Analytical solution: z(t) = z₀ + v₀t - ½gt²
        double z_exact = z0 + v0 * t - 0.5 * g * t * t;
        double error = std::abs(z - z_exact);

        if (step % output_interval == 0) {
            std::cout << "  " << std::setw(8) << t
                      << "  " << std::setw(10) << z
                      << "  " << std::setw(10) << vz
                      << "  " << std::setw(10) << z_exact
                      << "  " << std::setw(10) << error
                      << "\n";
        }

        sim.Step(dt);
        ++step;
    }

    // =========================================================================
    // Final Verification
    // =========================================================================

    double t_final = sim.Time();
    double z_final = sim.GetSignal("PointMass3DOF.position.z");
    double vz_final = sim.GetSignal("PointMass3DOF.velocity.z");

    double z_exact_final = z0 + v0 * t_final - 0.5 * g * t_final * t_final;
    double v_exact_final = v0 - g * t_final;

    double z_error = std::abs(z_final - z_exact_final);
    double v_error = std::abs(vz_final - v_exact_final);

    std::cout << "\n=== Final State (t = " << t_final << " s) ===\n";
    std::cout << "  Position:  z = " << z_final << " m  (exact: " << z_exact_final << " m)\n";
    std::cout << "  Velocity:  v = " << vz_final << " m/s  (exact: " << v_exact_final << " m/s)\n";
    std::cout << "  Position error: " << z_error << " m\n";
    std::cout << "  Velocity error: " << v_error << " m/s\n";

    bool passed = (z_error < 1e-4) && (v_error < 1e-4);
    std::cout << "\n=== Validation: " << (passed ? "PASSED" : "FAILED") << " ===\n";

    return passed ? 0 : 1;
}
```

---

## Orbital Simulation Example

For demonstrating Vulcan's point-mass gravity model with realistic orbital dynamics **in ECI frame**.

### File: `examples/orbital_decay/main.cpp`

```cpp
/**
 * @file main.cpp
 * @brief Orbital dynamics using Vulcan's point-mass gravity
 *
 * FRAME: Earth-Centered Inertial (ECI)
 * Position/velocity are in non-rotating inertial coordinates.
 * Central gravity: g = -μ/r³ · r (same formula works in ECI and ECEF)
 *
 * Demonstrates:
 * - Vulcan's gravity::point_mass::acceleration() for orbital dynamics
 * - ECI frame propagation (no Coriolis/centrifugal needed)
 * - Energy conservation verification
 */

#include <icarus/icarus.hpp>
#include <icarus/components/PointMass3DOF.hpp>
#include <icarus/components/PointMassGravity.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/dynamics/PointMass.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace icarus;
using namespace icarus::components;

int main() {
    std::cout << "=== Orbital Dynamics with Vulcan Point-Mass Gravity ===\n\n";

    // =========================================================================
    // Simulation Setup (ECI Frame)
    // =========================================================================
    // Frame: Earth-Centered Inertial (non-rotating)
    // Gravity: Central force toward Earth center
    // Dynamics: point_mass_acceleration() (inertial, no fictitious forces)

    Simulator<double> sim;

    // Circular orbit at 400 km altitude (ISS-like)
    // Position is from Earth center in ECI coordinates
    double altitude = 400e3;  // m
    double r0 = vulcan::constants::earth::R_eq + altitude;
    double mu = vulcan::constants::earth::mu;

    // Circular orbit velocity: v = sqrt(μ/r)
    double v_circular = std::sqrt(mu / r0);

    // Create components
    double mass = 1000.0;  // kg
    auto sat = std::make_unique<PointMass3DOF<double>>(mass, "Satellite");
    sat->SetInitialPosition(r0, 0.0, 0.0);      // Start on +X axis
    sat->SetInitialVelocity(0.0, v_circular, 0.0);  // Velocity in +Y direction

    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    gravity->SetModel(PointMassGravity<double>::Model::PointMass);

    sim.AddComponent(std::move(sat));
    sim.AddComponent(std::move(gravity));

    sim.Provision();
    sim.Stage();

    // Orbital period: T = 2π√(r³/μ)
    double T_orbit = 2.0 * M_PI * std::sqrt(r0 * r0 * r0 / mu);
    std::cout << "Orbital period: " << T_orbit / 60.0 << " minutes\n";
    std::cout << "Circular velocity: " << v_circular << " m/s\n\n";

    // Compute initial specific energy
    double x0 = sim.GetSignal("Satellite.position.x");
    double y0 = sim.GetSignal("Satellite.position.y");
    double z0 = sim.GetSignal("Satellite.position.z");
    double vx0 = sim.GetSignal("Satellite.velocity.x");
    double vy0 = sim.GetSignal("Satellite.velocity.y");
    double vz0 = sim.GetSignal("Satellite.velocity.z");

    vulcan::Vec3<double> r_init{x0, y0, z0};
    vulcan::Vec3<double> v_init{vx0, vy0, vz0};
    double E0 = vulcan::dynamics::specific_energy(r_init, v_init, mu);
    std::cout << "Initial specific energy: " << E0 << " J/kg\n\n";

    // Simulate one orbit
    double dt = 10.0;  // 10 second timesteps
    int steps = static_cast<int>(T_orbit / dt);

    for (int i = 0; i < steps; ++i) {
        sim.Step(dt);
    }

    // Verify energy conservation
    double x = sim.GetSignal("Satellite.position.x");
    double y = sim.GetSignal("Satellite.position.y");
    double z = sim.GetSignal("Satellite.position.z");
    double vx = sim.GetSignal("Satellite.velocity.x");
    double vy = sim.GetSignal("Satellite.velocity.y");
    double vz = sim.GetSignal("Satellite.velocity.z");

    vulcan::Vec3<double> r_final{x, y, z};
    vulcan::Vec3<double> v_final{vx, vy, vz};
    double E_final = vulcan::dynamics::specific_energy(r_final, v_final, mu);

    double E_error = std::abs(E_final - E0) / std::abs(E0);

    std::cout << "Final specific energy: " << E_final << " J/kg\n";
    std::cout << "Energy error (relative): " << E_error * 100 << " %\n";

    bool passed = E_error < 1e-6;  // Less than 0.0001% error
    std::cout << "\n=== Energy Conservation: " << (passed ? "PASSED" : "FAILED") << " ===\n";

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
#include <vulcan/core/Constants.hpp>
#include <vulcan/dynamics/PointMass.hpp>
#include <cmath>

using namespace icarus;
using namespace icarus::components;

// ---------------------------------------------------------------------------
// PointMass3DOF Unit Tests
// ---------------------------------------------------------------------------

TEST(PointMass3DOF, StateSize) {
    PointMass3DOF<double> pm(1.0);
    EXPECT_EQ(pm.StateSize(), 6);
    EXPECT_TRUE(pm.HasState());
}

TEST(PointMass3DOF, Identity) {
    PointMass3DOF<double> pm(1.0, "TestMass", "Vehicle");
    EXPECT_EQ(pm.Name(), "TestMass");
    EXPECT_EQ(pm.Entity(), "Vehicle");
    EXPECT_EQ(pm.TypeName(), "PointMass3DOF");
}

TEST(PointMass3DOF, MassProperty) {
    PointMass3DOF<double> pm(42.0);
    EXPECT_DOUBLE_EQ(pm.GetMass(), 42.0);

    pm.SetMass(100.0);
    EXPECT_DOUBLE_EQ(pm.GetMass(), 100.0);
}

TEST(PointMass3DOF, InitialConditions) {
    PointMass3DOF<double> pm(1.0);
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

TEST(PointMass3DOF, InitialConditionsVec3) {
    PointMass3DOF<double> pm(1.0);
    pm.SetInitialPosition(vulcan::Vec3<double>{1.0, 2.0, 3.0});
    pm.SetInitialVelocity(vulcan::Vec3<double>{4.0, 5.0, 6.0});

    double state[6];
    double state_dot[6];
    pm.BindState(state, state_dot, 6);

    EXPECT_DOUBLE_EQ(state[0], 1.0);
    EXPECT_DOUBLE_EQ(state[3], 4.0);
}

TEST(PointMass3DOF, StateSizeMismatchThrows) {
    PointMass3DOF<double> pm(1.0);
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

TEST(PointMassGravity, ModelSwitch) {
    PointMassGravity<double> grav;

    grav.SetModel(PointMassGravity<double>::Model::Constant);
    EXPECT_EQ(grav.GetModel(), PointMassGravity<double>::Model::Constant);

    grav.SetModel(PointMassGravity<double>::Model::PointMass);
    EXPECT_EQ(grav.GetModel(), PointMassGravity<double>::Model::PointMass);

    grav.SetModel(PointMassGravity<double>::Model::J2);
    EXPECT_EQ(grav.GetModel(), PointMassGravity<double>::Model::J2);
}

TEST(PointMassGravity, GravitationalParameter) {
    PointMassGravity<double> grav;

    // Default is Earth's mu
    EXPECT_DOUBLE_EQ(grav.GetGravitationalParameter(), vulcan::constants::earth::mu);

    // Can override for other bodies
    double moon_mu = 4.9028e12;
    grav.SetGravitationalParameter(moon_mu);
    EXPECT_DOUBLE_EQ(grav.GetGravitationalParameter(), moon_mu);
}

// ---------------------------------------------------------------------------
// Integration Tests (Using Vulcan Physics)
// ---------------------------------------------------------------------------

TEST(PointMass3DOFIntegration, FreeFallValidation) {
    Simulator<double> sim;

    double mass = 1.0;
    auto point_mass = std::make_unique<PointMass3DOF<double>>(mass, "PointMass3DOF");
    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");

    gravity->SetModel(PointMassGravity<double>::Model::Constant);

    double z0 = 100.0;
    double v0 = 0.0;
    point_mass->SetInitialPosition(0.0, 0.0, z0);
    point_mass->SetInitialVelocity(0.0, 0.0, v0);

    sim.AddComponent(std::move(point_mass));
    sim.AddComponent(std::move(gravity));
    sim.Provision();
    sim.Stage();

    double dt = 0.01;
    double g = vulcan::constants::earth::g0;

    // Run for 2 seconds
    for (int i = 0; i < 200; ++i) {
        sim.Step(dt);
    }

    double t = sim.Time();
    double z = sim.GetSignal("PointMass3DOF.position.z");
    double vz = sim.GetSignal("PointMass3DOF.velocity.z");

    // Analytical: z(t) = z₀ + v₀t - ½gt²
    double z_exact = z0 + v0 * t - 0.5 * g * t * t;
    double v_exact = v0 - g * t;

    EXPECT_NEAR(z, z_exact, 1e-4);
    EXPECT_NEAR(vz, v_exact, 1e-4);
}

TEST(PointMass3DOFIntegration, ProjectileMotion) {
    Simulator<double> sim;

    double mass = 1.0;
    auto point_mass = std::make_unique<PointMass3DOF<double>>(mass, "PointMass3DOF");
    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");

    gravity->SetModel(PointMassGravity<double>::Model::Constant);

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
    double g = vulcan::constants::earth::g0;

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

TEST(PointMass3DOFIntegration, OrbitalEnergyConservation) {
    // Test energy conservation with Vulcan's point-mass gravity
    Simulator<double> sim;

    double mu = vulcan::constants::earth::mu;
    double r0 = vulcan::constants::earth::R_eq + 400e3;  // 400 km altitude
    double v_circular = std::sqrt(mu / r0);

    double mass = 1000.0;
    auto sat = std::make_unique<PointMass3DOF<double>>(mass, "Sat");
    sat->SetInitialPosition(r0, 0.0, 0.0);
    sat->SetInitialVelocity(0.0, v_circular, 0.0);

    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    gravity->SetModel(PointMassGravity<double>::Model::PointMass);

    sim.AddComponent(std::move(sat));
    sim.AddComponent(std::move(gravity));
    sim.Provision();
    sim.Stage();

    // Initial specific energy
    vulcan::Vec3<double> r_init{r0, 0.0, 0.0};
    vulcan::Vec3<double> v_init{0.0, v_circular, 0.0};
    double E0 = vulcan::dynamics::specific_energy(r_init, v_init, mu);

    // Simulate 1/4 orbit
    double T_orbit = 2.0 * M_PI * std::sqrt(r0 * r0 * r0 / mu);
    double dt = 10.0;
    int steps = static_cast<int>(T_orbit / 4.0 / dt);

    for (int i = 0; i < steps; ++i) {
        sim.Step(dt);
    }

    // Final energy
    double x = sim.GetSignal("Sat.position.x");
    double y = sim.GetSignal("Sat.position.y");
    double z = sim.GetSignal("Sat.position.z");
    double vx = sim.GetSignal("Sat.velocity.x");
    double vy = sim.GetSignal("Sat.velocity.y");
    double vz = sim.GetSignal("Sat.velocity.z");

    vulcan::Vec3<double> r_final{x, y, z};
    vulcan::Vec3<double> v_final{vx, vy, vz};
    double E_final = vulcan::dynamics::specific_energy(r_final, v_final, mu);

    // Energy should be conserved (within 0.01%)
    double E_error = std::abs(E_final - E0) / std::abs(E0);
    EXPECT_LT(E_error, 1e-4);
}

TEST(PointMass3DOFIntegration, VulcanAccelerationUsed) {
    // Verify that Vulcan's point_mass_acceleration is actually being used
    Simulator<double> sim;

    double mass = 2.0;  // Non-unit mass
    auto pm = std::make_unique<PointMass3DOF<double>>(mass, "PointMass3DOF");
    auto grav = std::make_unique<PointMassGravity<double>>("Gravity");

    grav->SetModel(PointMassGravity<double>::Model::Constant);

    pm->SetInitialPosition(0.0, 0.0, 100.0);
    pm->SetInitialVelocity(0.0, 0.0, 0.0);

    sim.AddComponent(std::move(pm));
    sim.AddComponent(std::move(grav));
    sim.Provision();
    sim.Stage();

    // Take one step
    sim.Step(0.01);

    // Check that acceleration is g (not g*mass or g/mass)
    // After dt, v = v0 + a*dt = 0 + (-g)*0.01
    double vz = sim.GetSignal("PointMass3DOF.velocity.z");
    double expected_vz = -vulcan::constants::earth::g0 * 0.01;

    // Vulcan's point_mass_acceleration divides by mass, so acceleration is g
    EXPECT_NEAR(vz, expected_vz, 1e-6);
}

// ---------------------------------------------------------------------------
// Symbolic Mode Tests
// ---------------------------------------------------------------------------

TEST(PointMass3DOFSymbolic, CompilationTest) {
    using MX = casadi::MX;

    Simulator<MX> sim;

    auto gravity = std::make_unique<PointMassGravity<MX>>("Gravity");
    auto point_mass = std::make_unique<PointMass3DOF<MX>>(MX{1.0}, "PointMass3DOF");

    sim.AddComponent(std::move(point_mass));
    sim.AddComponent(std::move(gravity));

    EXPECT_NO_THROW(sim.Provision());
    EXPECT_NO_THROW(sim.Stage());
    EXPECT_NO_THROW(sim.Step(MX{0.01}));
}

TEST(PointMass3DOFSymbolic, VulcanFunctionsCompile) {
    using MX = casadi::MX;

    // Verify Vulcan functions work with symbolic types
    vulcan::Vec3<MX> r{MX::sym("rx"), MX::sym("ry"), MX::sym("rz")};
    vulcan::Vec3<MX> force{MX::sym("fx"), MX::sym("fy"), MX::sym("fz")};
    MX mass = MX::sym("m");

    // Point mass acceleration
    auto accel = vulcan::dynamics::point_mass_acceleration(force, mass);
    EXPECT_EQ(accel.size(), 3);

    // Gravity acceleration
    auto grav = vulcan::gravity::point_mass::acceleration(r);
    EXPECT_EQ(grav.size(), 3);
}
```

---

## Implementation Checklist

### Task 2.3a: PointMass3DOF Component

- [ ] Create `components/dynamics/` directory
- [ ] Create `PointMass3DOF.hpp` header
- [ ] Include Vulcan headers: `vulcan/dynamics/PointMass.hpp`, `vulcan/core/VulcanTypes.hpp`
- [ ] Implement `StateSize()` returning 6
- [ ] Implement `Provision()` registering position/velocity/mass outputs
- [ ] Implement `Stage()` resolving force input
- [ ] Implement `BindState()` with IC application
- [ ] Implement `Step()` using `vulcan::dynamics::point_mass_acceleration()`
- [ ] Use `vulcan::Vec3<Scalar>` for position/velocity types
- [ ] Add mass property with setter/getter
- [ ] Add IC setters for `vulcan::Vec3<Scalar>` overloads
- [ ] Add state accessors for testing

### Task 2.3b: PointMassGravity Component

- [ ] Create `components/environment/` directory
- [ ] Create `PointMassGravity.hpp` header
- [ ] Include Vulcan headers: `vulcan/gravity/PointMass.hpp`, `vulcan/gravity/J2.hpp`, `vulcan/core/Constants.hpp`
- [ ] Implement `StateSize()` returning 0 (stateless)
- [ ] Implement `Provision()` registering force and acceleration outputs
- [ ] Implement `Stage()` resolving position and mass inputs
- [ ] Implement Model enum: `Constant`, `PointMass`, `J2`
- [ ] Implement `Step()` with Constant mode using `vulcan::constants::earth::g0`
- [ ] Implement `Step()` with PointMass mode using `vulcan::gravity::point_mass::acceleration()`
- [ ] Implement `Step()` with J2 mode using `vulcan::gravity::j2::acceleration()`
- [ ] Compute force from acceleration: `F = m * a`
- [ ] Add gravitational parameter setter for other celestial bodies

### Task 2.3c: Example Applications

- [ ] Create `examples/falling_mass/` directory
- [ ] Create `main.cpp` with constant gravity validation
- [ ] Use `vulcan::constants::earth::g0` for analytical verification
- [ ] Add CMakeLists.txt for example
- [ ] Create `examples/orbital_decay/` directory (optional)
- [ ] Create `main.cpp` with orbital energy conservation demo
- [ ] Use `vulcan::dynamics::specific_energy()` for verification
- [ ] Add to top-level examples build

### Task 2.3d: Unit Tests

- [ ] Create `tests/components/test_point_mass_3dof.cpp`
- [ ] Include Vulcan test utilities: `vulcan/core/Constants.hpp`, `vulcan/dynamics/PointMass.hpp`
- [ ] Add PointMass3DOF identity/state size tests
- [ ] Add PointMass3DOF mass property tests
- [ ] Add PointMass3DOF initial condition tests (scalar and Vec3)
- [ ] Add PointMassGravity identity/model tests
- [ ] Add PointMassGravity gravitational parameter tests
- [ ] Add free fall analytical validation test (using `vulcan::constants::earth::g0`)
- [ ] Add projectile motion test
- [ ] Add orbital energy conservation test (using `vulcan::dynamics::specific_energy()`)
- [ ] Add Vulcan acceleration verification test
- [ ] Add symbolic mode compilation test
- [ ] Add Vulcan symbolic compatibility test
- [ ] Update `tests/CMakeLists.txt`

### Task 2.3e: Build Integration

- [ ] Update `components/CMakeLists.txt` with new components
- [ ] Ensure Vulcan include paths are available
- [ ] Update `include/icarus/icarus.hpp` with component headers
- [ ] Add `include/icarus/components/PointMass3DOF.hpp` (forwarding header)
- [ ] Add `include/icarus/components/PointMassGravity.hpp` (forwarding header)
- [ ] Verify `./scripts/build.sh` succeeds
- [ ] Verify `./scripts/test.sh` all pass
- [ ] Verify `./scripts/run_examples.sh` includes examples

### Task 2.3f: Documentation

- [ ] Update main implementation plan checkboxes
- [ ] Mark Phase 2.3 complete when all exit criteria pass

---

## Design Decisions

### 1. Wrap Vulcan Functions, Don't Reimplement

**Decision:** Components wrap Vulcan's physics functions rather than reimplementing physics.

**Rationale:**

- Vulcan already provides tested, symbolic-compatible implementations
- `vulcan::dynamics::point_mass_acceleration()` handles F/m correctly
- `vulcan::gravity::point_mass::acceleration()` uses proper formula
- `vulcan::constants::earth::*` provides authoritative constants
- Reduces code duplication and ensures consistency

**Implementation:**

```cpp
// In PointMass3DOF::Step()
vulcan::Vec3<Scalar> accel = vulcan::dynamics::point_mass_acceleration(force, mass_);

// In PointMassGravity::Step()
accel_ = vulcan::gravity::point_mass::acceleration(pos, mu_);
```

### 2. Force-Based Interface (Not Acceleration)

**Decision:** Gravity component outputs **force** (N), not acceleration (m/s²).

**Rationale:**

- Force is extensive (scales with mass), acceleration is intensive
- Multiple force sources can be summed directly
- Dynamics component uses `vulcan::dynamics::point_mass_acceleration(force, mass)`
- Matches aerospace convention where forces are aggregated

**Flow:**

```
Gravity.force = mass * gravity::point_mass::acceleration(pos)
Dynamics: accel = point_mass_acceleration(force, mass)  // = force/mass
```

### 3. Three Gravity Models

**Decision:** Support `Constant`, `PointMass`, and `J2` models via enum.

**Rationale:**

- `Constant` mode for analytical validation (closed-form solution)
- `PointMass` mode for orbital dynamics
- `J2` mode for higher fidelity (Earth oblateness)
- All use Vulcan's templated implementations
- Easy to extend to J2J4, spherical harmonics later

### 4. Use Vulcan Types Throughout

**Decision:** Use `vulcan::Vec3<Scalar>` for position/velocity internally.

**Rationale:**

- Vulcan functions expect `Vec3<Scalar>` parameters
- Avoids repeated construction/destruction
- Cleaner code than manual x/y/z handling
- Still expose individual scalars for signal backplane

### 5. Component Ordering

**Decision:** PointMass3DOF provisions before PointMassGravity.

**Rationale:**

- Signal resolution requires outputs to exist
- PointMass3DOF creates `position.{x,y,z}` and `mass` signals
- PointMassGravity resolves those signals in Stage
- Component add order determines provision/stage order

### 6. Initial Conditions in BindState

**Decision:** Apply initial conditions when state is bound.

**Rationale:**

- State pointers aren't available until `BindState()` is called
- IC values set via setters or config before Stage
- BindState is the first moment we have write access to X_global_

### 7. Mass as Output Signal

**Decision:** PointMass3DOF outputs mass as a signal for gravity to read.

**Rationale:**

- Decouples components (gravity doesn't need to know about dynamics internals)
- Mass could change during simulation (future: propellant consumption)
- Follows Icarus pattern: all inter-component data flows through signals

### 8. Frame Selection Strategy (ECI for Phase 2.3)

**Decision:** Use inertial frames (ECI or local) for Phase 2.3; defer ECEF to later phase.

**Rationale:**

- ECI avoids Coriolis/centrifugal complexity (`point_mass_acceleration()` is simpler)
- Central gravity formula `g = -μ/r³ · r` is identical in ECI and ECEF
- Validation tests use local vertical frame (constant gravity, Z-up)
- Orbital tests use ECI (energy conservation is frame-invariant)
- ECEF requires `point_mass_acceleration_ecef()` with ω vector — added complexity

**Frame Compatibility Matrix:**

| Gravity Model | Dynamics Function | Frame | Use Case |
|:--------------|:------------------|:------|:---------|
| Constant | `point_mass_acceleration()` | Local (Z-up) | Validation |
| PointMass | `point_mass_acceleration()` | ECI | Orbital dynamics |
| J2 | `point_mass_acceleration()` | ECI | Orbital (high-fidelity) |
| *Future* | `point_mass_acceleration_ecef()` | ECEF | Launch, reentry |

**Future Work:**

- Add `PointMass3DOF_ECEF` variant or frame-selection option
- Integrate with Vulcan's `CoordinateFrame` for transformations
- Consider automatic frame tagging on signals

---

## Janus Compatibility Checklist

- [ ] All component code templated on `Scalar`
- [ ] Use `vulcan::Vec3<Scalar>` (not `std::vector<double>`)
- [ ] Use Vulcan physics functions (already Janus-compatible)
- [ ] No `std::` math in component code (Vulcan handles this internally)
- [ ] No `if/else` branching on `Scalar` values in traced code
- [ ] Mode switching uses structural branching (enum, not Scalar-dependent)
- [ ] Verify Vulcan functions compile with `casadi::MX`:
  - [ ] `vulcan::dynamics::point_mass_acceleration<MX>()`
  - [ ] `vulcan::gravity::point_mass::acceleration<MX>()`
  - [ ] `vulcan::gravity::j2::acceleration<MX>()`
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
