# Phase 9B: Vehicle6DOF Unified Model Implementation

**Status:** Proposed
**Prerequisites:** Phase 9A (PhysicalComponent class), Phase 6 (unified signal model)
**Related:** [unified_vehicle_component.md](../unified_vehicle_component.md)

---

## Overview

This document provides a detailed implementation plan for the `Vehicle6DOF` component that unifies 6DOF dynamics, mass aggregation, and force aggregation into a single component.

### Goals

1. Merge MassAggregator + ForceAggregator + RigidBody6DOF into one unified component
2. Reduce signal routing complexity (internal aggregation eliminates many routes)
3. Implement frame-aware force and mass aggregation
4. Support body and ECEF force frames

### Non-Goals

- **Auto-discovery of providers** (deferred to Phase 10 - see [Future: Auto-Discovery](#future-auto-discovery))
- Removing existing separate components (keep for backwards compatibility)
- Performance optimization (defer to Phase 10)

### Why Not Auto-Discovery Now?

Auto-discovery (scanning for signal patterns like `{comp}.force_body.*`) adds complexity:
1. Requires new Backplane discovery API
2. ComponentRegistry for querying component instances
3. Complex pattern matching logic
4. Harder to debug when discovery doesn't find expected sources

The explicit approach matches how users already configure MassAggregator and ForceAggregator. Migration is straightforward: combine three component configs into one.

---

## Architecture Summary

### Before: Three Separate Components

```yaml
# Current: 3 components + many routes
components:
  - type: MassAggregator
    name: Mass
    sources: [Structure, FuelTank]

  - type: ForceAggregator
    name: Forces
    body_sources: [Engine]
    ecef_sources: [Gravity]

  - type: RigidBody6DOF
    name: EOM
    # ... initial conditions ...

routes:
  # 6+ routes for Mass→Forces
  # 6+ routes for Mass→EOM
  # 6+ routes for Forces→EOM
  # 4 routes for EOM attitude→Forces
  # ... many more ...
```

### After: One Unified Component

```yaml
# Unified: 1 component, minimal routes
components:
  - type: Vehicle6DOF
    name: Vehicle
    entity: Rocket

    # Mass aggregation (replaces MassAggregator)
    mass_sources: [Structure, FuelTank]

    # Force aggregation (replaces ForceAggregator)
    body_sources: [Engine]
    ecef_sources: [Gravity]

    # 6DOF dynamics (replaces RigidBody6DOF)
    initial_lla: [0.0, 0.0, 100000.0]
    initial_euler_zyx: [0.0, 90.0, 0.0]
    initial_velocity_body: [0, 0, 0]
    initial_omega_body: [0, 0, 0]
    reference_frame: ecef

routes:
  # Only external inputs/outputs need routing
  - input: Rocket.FuelTank.mass_flow_rate
    output: Rocket.Engine.mass_flow_rate

  - input: Rocket.Gravity.position.x
    output: Rocket.Vehicle.position.x
  # ... position.y, position.z ...

  - input: Rocket.Gravity.mass
    output: Rocket.Vehicle.total_mass
```

### Component Relationships

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              Entity: "Rocket"                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                 │
│   │ StaticMass  │    │  FuelTank   │    │ RocketEngine│                 │
│   │  (mass)     │    │   (mass)    │    │   (force)   │                 │
│   └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                 │
│          │                  │                   │                        │
│          │ mass             │ mass              │ force                  │
│          │ cg               │ cg                │                        │
│          │ inertia          │ inertia           │                        │
│          │                  │                   │                        │
│          └────────────┬─────┴───────────────────┘                        │
│                       │ (explicit config)                                │
│                       ▼                                                  │
│              ┌────────────────┐                                          │
│              │   Vehicle6DOF   │                                          │
│              │                 │    ┌─────────────┐                      │
│              │ mass_sources:   │◄───│   Gravity   │ force_ecef           │
│              │  [Structure,    │    └─────────────┘                      │
│              │   FuelTank]     │                                          │
│              │                 │                                          │
│              │ body_sources:   │                                          │
│              │  [Engine]       │                                          │
│              │                 │                                          │
│              │ ecef_sources:   │                                          │
│              │  [Gravity]      │                                          │
│              │                 │                                          │
│              │ → Aggregation   │                                          │
│              │ → 6DOF dynamics │                                          │
│              └────────────────┘                                          │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Implementation Tasks

### Task 1: Vehicle6DOF Component Header

**File:** `include/icarus/components/dynamics/Vehicle6DOF.hpp`

```cpp
#pragma once

#include "icarus/core/Component.hpp"
#include "icarus/signal/Handle.hpp"
#include "vulcan/dynamics/RigidBody.hpp"
#include "vulcan/mass/MassProperties.hpp"
#include "janus/math/Quaternion.hpp"
#include <vector>
#include <string>

namespace icarus {

/**
 * @brief Unified 6DOF vehicle dynamics with mass/force aggregation.
 *
 * Vehicle6DOF combines:
 * - Mass property aggregation (from MassAggregator)
 * - Force/moment aggregation (from ForceAggregator)
 * - 13-state quaternion rigid body dynamics (from RigidBody6DOF)
 *
 * ## Configuration
 *
 * ```yaml
 * - type: Vehicle6DOF
 *   name: Vehicle
 *   entity: Rocket
 *
 *   # Mass sources (component names in same entity)
 *   mass_sources: [Structure, FuelTank]
 *
 *   # Force sources by frame
 *   body_sources: [Engine]           # Forces in body frame
 *   ecef_sources: [Gravity]          # Forces in ECEF frame
 *
 *   # Initial conditions
 *   initial_lla: [lat_deg, lon_deg, alt_m]
 *   initial_euler_zyx: [yaw_deg, pitch_deg, roll_deg]
 *   initial_velocity_body: [vx, vy, vz]
 *   initial_omega_body: [wx, wy, wz]
 *
 *   # Reference frame
 *   reference_frame: "ecef"       # "eci" or "ecef"
 *
 *   # Default mass (if no sources specified)
 *   default_mass: 1.0
 *   default_inertia: [1.0, 1.0, 1.0]  # Diagonal Ixx, Iyy, Izz
 * ```
 *
 * ## Outputs
 *
 * **States:**
 * - `position.x/y/z` [m] — Position in reference frame
 * - `velocity_body.x/y/z` [m/s] — Velocity in body frame
 * - `attitude.w/x/y/z` — Quaternion (body-to-reference)
 * - `omega_body.x/y/z` [rad/s] — Angular velocity in body frame
 *
 * **Derived:**
 * - `velocity_ref.x/y/z` [m/s] — Velocity in reference frame
 * - `position_lla.lat/lon/alt` — Geodetic coordinates
 * - `euler_zyx.yaw/pitch/roll` [rad] — Euler angles
 *
 * **Aggregated:**
 * - `total_mass` [kg]
 * - `cg.x/y/z` [m]
 * - `inertia.xx/yy/zz/xy/xz/yz` [kg*m^2]
 * - `total_force.x/y/z` [N]
 * - `total_moment.x/y/z` [N*m]
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar>
class Vehicle6DOF : public Component<Scalar> {
public:
    static std::string TypeName() { return "Vehicle6DOF"; }

    void Provision(Backplane<Scalar>& bp) override;
    void Stage(Backplane<Scalar>& bp) override;
    void Step(Scalar t, Scalar dt) override;

    std::vector<SignalDecl> DeclareOutputs() const override;

    // Accessors
    [[nodiscard]] Vec3<Scalar> GetPosition() const { return position_; }
    [[nodiscard]] Vec3<Scalar> GetVelocityBody() const { return velocity_body_; }
    [[nodiscard]] janus::Quaternion<Scalar> GetAttitude() const;
    [[nodiscard]] Scalar GetTotalMass() const { return total_mass_; }

private:
    // --- Mass source handles ---
    struct MassSource {
        std::string name;
        SignalHandle<Scalar> mass;
        SignalHandle<Scalar> cg_x, cg_y, cg_z;
        SignalHandle<Scalar> inertia_xx, inertia_yy, inertia_zz;
        SignalHandle<Scalar> inertia_xy, inertia_xz, inertia_yz;
    };
    std::vector<MassSource> mass_sources_;

    // --- Force source handles ---
    struct ForceSource {
        std::string name;
        SignalHandle<Scalar> force_x, force_y, force_z;
        SignalHandle<Scalar> moment_x, moment_y, moment_z;
        bool has_moment = false;
        bool is_ecef = false;  // true = ECEF frame, false = body frame
    };
    std::vector<ForceSource> force_sources_;

    // --- 13-State Variables ---
    Vec3<Scalar> position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> position_dot_ = Vec3<Scalar>::Zero();

    Vec3<Scalar> velocity_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_body_dot_ = Vec3<Scalar>::Zero();

    Vec4<Scalar> attitude_ = Vec4<Scalar>{1, 0, 0, 0};  // [w, x, y, z]
    Vec4<Scalar> attitude_dot_ = Vec4<Scalar>::Zero();

    Vec3<Scalar> omega_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> omega_body_dot_ = Vec3<Scalar>::Zero();

    // --- Aggregated Properties ---
    Scalar total_mass_ = Scalar(1);
    Vec3<Scalar> total_cg_ = Vec3<Scalar>::Zero();
    Scalar inertia_xx_, inertia_yy_, inertia_zz_;
    Scalar inertia_xy_, inertia_xz_, inertia_yz_;

    Vec3<Scalar> total_force_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> total_moment_body_ = Vec3<Scalar>::Zero();

    // --- Derived Outputs ---
    Vec3<Scalar> velocity_ref_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> position_lla_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> euler_zyx_ = Vec3<Scalar>::Zero();

    // --- Configuration ---
    std::string reference_frame_ = "ecef";
    Scalar default_mass_ = Scalar(1);
    Vec3<Scalar> default_inertia_diag_ = Vec3<Scalar>{1, 1, 1};

    // --- Internal Methods ---
    void ApplyInitialConditions();
    void ResolveMassHandles(Backplane<Scalar>& bp);
    void ResolveForceHandles(Backplane<Scalar>& bp);
    void AggregateMass();
    void AggregateForces();
    void ComputeDynamics();
    void ComputeDerivedOutputs();
    [[nodiscard]] Mat3<Scalar> GetInertiaMatrix() const;
};

}  // namespace icarus
```

### Task 2: Vehicle6DOF Implementation

**File:** `src/components/dynamics/Vehicle6DOF.cpp` (template instantiations) or inline in header.

Key implementation details:

#### Provision - Register States and Outputs

```cpp
template <typename Scalar>
void Vehicle6DOF<Scalar>::Provision(Backplane<Scalar>& bp) {
    // States
    bp.register_state_vec3("position", &position_, &position_dot_, "m", "Position");
    bp.register_state_vec3("velocity_body", &velocity_body_, &velocity_body_dot_,
                          "m/s", "Body velocity");
    bp.register_state_quat("attitude", &attitude_, &attitude_dot_, "",
                          "Body-to-ref quaternion");
    bp.register_state_vec3("omega_body", &omega_body_, &omega_body_dot_,
                          "rad/s", "Body angular velocity");

    // Aggregated outputs
    bp.template register_output<Scalar>("total_mass", &total_mass_, "kg", "Total mass");
    bp.template register_output_vec3<Scalar>("cg", &total_cg_, "m", "Center of gravity");
    // ... inertia outputs ...
    bp.template register_output_vec3<Scalar>("total_force", &total_force_body_, "N", "Body force");
    bp.template register_output_vec3<Scalar>("total_moment", &total_moment_body_, "N*m", "Body moment");

    // Derived outputs
    bp.template register_output_vec3<Scalar>("velocity_ref", &velocity_ref_, "m/s", "Ref velocity");
    // ... LLA, euler outputs ...
}
```

#### Stage - Read Config, Resolve Handles

```cpp
template <typename Scalar>
void Vehicle6DOF<Scalar>::Stage(Backplane<Scalar>& bp) {
    const auto& config = this->GetConfig();

    // Read source lists from config
    auto mass_source_names = config.template Get<std::vector<std::string>>("mass_sources", {});
    auto body_source_names = config.template Get<std::vector<std::string>>("body_sources", {});
    auto ecef_source_names = config.template Get<std::vector<std::string>>("ecef_sources", {});

    // Resolve handles for each mass source
    for (const auto& name : mass_source_names) {
        ResolveMassSource(bp, name);
    }

    // Resolve handles for body-frame force sources
    for (const auto& name : body_source_names) {
        ResolveForceSource(bp, name, /*is_ecef=*/false);
    }

    // Resolve handles for ECEF-frame force sources
    for (const auto& name : ecef_source_names) {
        ResolveForceSource(bp, name, /*is_ecef=*/true);
    }

    ApplyInitialConditions();
}
```

#### Step - Aggregate and Compute Dynamics

```cpp
template <typename Scalar>
void Vehicle6DOF<Scalar>::Step(Scalar t, Scalar dt) {
    (void)t; (void)dt;

    AggregateMass();      // Sum mass properties from all sources
    AggregateForces();    // Sum forces, transform ECEF→body
    ComputeDynamics();    // 6DOF equations of motion
    ComputeDerivedOutputs();  // LLA, euler, etc.
}
```

### Task 3: Handle Resolution Helpers

```cpp
template <typename Scalar>
void Vehicle6DOF<Scalar>::ResolveMassSource(Backplane<Scalar>& bp, const std::string& name) {
    std::string entity = this->Entity();
    std::string prefix = entity + "." + name + ".";

    MassSource src;
    src.name = name;
    src.mass = bp.template resolve<Scalar>(prefix + "mass");
    src.cg_x = bp.template resolve<Scalar>(prefix + "cg.x");
    src.cg_y = bp.template resolve<Scalar>(prefix + "cg.y");
    src.cg_z = bp.template resolve<Scalar>(prefix + "cg.z");
    src.inertia_xx = bp.template resolve<Scalar>(prefix + "inertia.xx");
    src.inertia_yy = bp.template resolve<Scalar>(prefix + "inertia.yy");
    src.inertia_zz = bp.template resolve<Scalar>(prefix + "inertia.zz");
    src.inertia_xy = bp.template resolve<Scalar>(prefix + "inertia.xy");
    src.inertia_xz = bp.template resolve<Scalar>(prefix + "inertia.xz");
    src.inertia_yz = bp.template resolve<Scalar>(prefix + "inertia.yz");

    mass_sources_.push_back(std::move(src));
}

template <typename Scalar>
void Vehicle6DOF<Scalar>::ResolveForceSource(Backplane<Scalar>& bp, const std::string& name,
                                              bool is_ecef) {
    std::string entity = this->Entity();
    std::string prefix = entity + "." + name + ".";

    // Determine signal names based on frame
    std::string force_signal = is_ecef ? "force_ecef" : "force";
    // Note: Most body-frame components use "force", not "force_body"
    // Could also check for "thrust" as alternative

    ForceSource src;
    src.name = name;
    src.is_ecef = is_ecef;

    src.force_x = bp.template resolve<Scalar>(prefix + force_signal + ".x");
    src.force_y = bp.template resolve<Scalar>(prefix + force_signal + ".y");
    src.force_z = bp.template resolve<Scalar>(prefix + force_signal + ".z");

    // Optional moments
    if (bp.has_signal(prefix + "moment.x")) {
        src.moment_x = bp.template resolve<Scalar>(prefix + "moment.x");
        src.moment_y = bp.template resolve<Scalar>(prefix + "moment.y");
        src.moment_z = bp.template resolve<Scalar>(prefix + "moment.z");
        src.has_moment = true;
    }

    force_sources_.push_back(std::move(src));
}
```

### Task 4: Aggregation Methods

```cpp
template <typename Scalar>
void Vehicle6DOF<Scalar>::AggregateMass() {
    if (mass_sources_.empty()) {
        // Use defaults
        total_mass_ = default_mass_;
        total_cg_ = Vec3<Scalar>::Zero();
        inertia_xx_ = default_inertia_diag_(0);
        inertia_yy_ = default_inertia_diag_(1);
        inertia_zz_ = default_inertia_diag_(2);
        inertia_xy_ = inertia_xz_ = inertia_yz_ = Scalar(0);
        return;
    }

    // Use vulcan::mass::MassProperties aggregation
    vulcan::mass::MassProperties<Scalar> total;
    for (const auto& src : mass_sources_) {
        vulcan::mass::MassProperties<Scalar> props;
        props.mass = *src.mass;
        props.cg = Vec3<Scalar>{*src.cg_x, *src.cg_y, *src.cg_z};
        props.inertia(0,0) = *src.inertia_xx;
        props.inertia(1,1) = *src.inertia_yy;
        props.inertia(2,2) = *src.inertia_zz;
        props.inertia(0,1) = props.inertia(1,0) = *src.inertia_xy;
        props.inertia(0,2) = props.inertia(2,0) = *src.inertia_xz;
        props.inertia(1,2) = props.inertia(2,1) = *src.inertia_yz;

        total = total + props;  // Vulcan aggregation
    }

    // Write outputs
    total_mass_ = total.mass;
    total_cg_ = total.cg;
    inertia_xx_ = total.inertia(0,0);
    // ... etc ...
}

template <typename Scalar>
void Vehicle6DOF<Scalar>::AggregateForces() {
    Vec3<Scalar> sum_force = Vec3<Scalar>::Zero();
    Vec3<Scalar> sum_moment = Vec3<Scalar>::Zero();

    janus::Quaternion<Scalar> q_body_to_ref{
        attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

    for (const auto& src : force_sources_) {
        Vec3<Scalar> F{*src.force_x, *src.force_y, *src.force_z};

        if (src.is_ecef) {
            // Transform ECEF → body
            F = q_body_to_ref.conjugate().rotate(F);
        }

        sum_force += F;

        if (src.has_moment) {
            Vec3<Scalar> M{*src.moment_x, *src.moment_y, *src.moment_z};
            sum_moment += M;
        }
    }

    total_force_body_ = sum_force;
    total_moment_body_ = sum_moment;
}
```

### Task 5: Unit Tests

**File:** `tests/components/dynamics/test_vehicle6dof.cpp`

Key test cases:
1. Mass aggregation matches standalone MassAggregator
2. Force aggregation matches standalone ForceAggregator
3. 6DOF dynamics integration matches RigidBody6DOF
4. ECEF→body force transformation
5. Default mass fallback when no sources
6. Symbolic mode compilation (casadi::MX)

### Task 6: Register in ComponentFactory

**File:** `src/sim/ComponentFactory.cpp`

```cpp
factory.Register<Vehicle6DOF>("Vehicle6DOF");
```

---

## Migration Guide

### Converting rocket_ascent.yaml

**Before:**
```yaml
components:
  - type: MassAggregator
    name: Mass
    entity: Rocket
    sources: [Structure, FuelTank]

  - type: ForceAggregator
    name: Forces
    entity: Rocket
    body_sources: [Engine]
    ecef_sources: [Gravity]

  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    vectors:
      initial_lla: [0.0, 0.0, 100000.0]
      initial_euler_zyx: [0.0, 90.0, 0.0]
      # ... etc ...

routes:
  # Many routes for Mass→Forces, Mass→EOM, Forces→EOM, EOM→Forces
```

**After:**
```yaml
components:
  - type: Vehicle6DOF
    name: Vehicle
    entity: Rocket

    # Combined from MassAggregator
    mass_sources: [Structure, FuelTank]

    # Combined from ForceAggregator
    body_sources: [Engine]
    ecef_sources: [Gravity]

    # Combined from RigidBody6DOF
    vectors:
      initial_lla: [0.0, 0.0, 100000.0]
      initial_euler_zyx: [0.0, 90.0, 0.0]
      initial_velocity_body: [0, 0, 0]
      initial_omega_body: [0, 0, 0]
    strings:
      reference_frame: ecef

routes:
  # Only external connections needed:
  - input: Rocket.FuelTank.mass_flow_rate
    output: Rocket.Engine.mass_flow_rate

  - input: Rocket.Gravity.position.x
    output: Rocket.Vehicle.position.x
  - input: Rocket.Gravity.position.y
    output: Rocket.Vehicle.position.y
  - input: Rocket.Gravity.position.z
    output: Rocket.Vehicle.position.z

  - input: Rocket.Gravity.mass
    output: Rocket.Vehicle.total_mass
```

**Eliminated routes:**
- All Mass→EOM routes (6 inertia + 1 total_mass)
- All Mass→Forces routes (3 cg)
- All Forces→EOM routes (6 force/moment)
- EOM attitude→Forces routes (4 quaternion)

---

## Validation Criteria

1. **Aggregation matches**: Same mass/force results as separate aggregators
2. **Dynamics match**: Same trajectories as RigidBody6DOF with manual wiring
3. **Frame transforms**: ECEF→body transformation verified
4. **Symbolic compilation**: Templates compile for casadi::MX
5. **Config compatibility**: Works with existing signal names (no `_local` suffix required)

---

## Future: Auto-Discovery

**Deferred to Phase 10**

If we want auto-discovery later, the approach would be:

1. **Signal pattern scanning**: Scan all signals in entity for patterns like:
   - `{comp}.mass` + `{comp}.cg.*` + `{comp}.inertia.*` → mass source
   - `{comp}.force.*` or `{comp}.thrust.*` → body force source
   - `{comp}.force_ecef.*` → ECEF force source

2. **Backplane discovery API**:
   ```cpp
   std::vector<std::string> components_in_entity(const std::string& entity);
   std::vector<std::string> signals_with_prefix(const std::string& prefix);
   ```

3. **Exclusion lists**: Allow `exclude_mass_sources: [SomeComp]` to opt-out

4. **Frame-aware signals**: Components output `force_local` with attachment info, Vehicle6DOF transforms to body frame.

This adds complexity and should only be done if the explicit approach becomes burdensome for large vehicle models.

---

## Implementation Checklist

### Phase 9B-1: Core Component
- [ ] Create `Vehicle6DOF.hpp` with handle structs
- [ ] Implement Provision (state + output registration)
- [ ] Implement Stage (config parsing + handle resolution)
- [ ] Implement Step (aggregation + dynamics)

### Phase 9B-2: Aggregation
- [ ] Mass aggregation using vulcan::mass
- [ ] Body-frame force summation
- [ ] ECEF→body force transformation

### Phase 9B-3: Dynamics
- [ ] 6DOF equations (reuse from RigidBody6DOF)
- [ ] Derived outputs (LLA, euler, velocity_ref)

### Phase 9B-4: Testing
- [ ] Unit tests for aggregation
- [ ] Integration test comparing to separate components
- [ ] Symbolic mode compilation test

### Phase 9B-5: Integration
- [ ] Register in ComponentFactory
- [ ] Create example config using Vehicle6DOF
- [ ] Update documentation

---

## Related Documents

- [unified_vehicle_component.md](../unified_vehicle_component.md) — Overall design
- [phase9_physical_component.md](./phase9_physical_component.md) — PhysicalComponent (Part A)
- [02_component_protocol.md](../../architecture/02_component_protocol.md) — Component lifecycle
