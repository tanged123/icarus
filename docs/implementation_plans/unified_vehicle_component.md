# Unified Vehicle Component: Refined Design

## Executive Summary

This document refines the unified vehicle component design by combining the **automatic aggregation semantics** of hierarchical "system model" architectures with **IODA's flat, composable signal model**.

**Key Insight**: Components self-declare their physical characteristics (mass, forces, body attachment) through standard signal patterns. A `Vehicle6DOF` component discovers and aggregates these automatically within its entity namespace.

---

## Problem Recap

The current architecture requires 26+ manual routes to connect:

1. `RigidBody6DOF` — owns 13-state dynamics
2. `ForceAggregator` — sums forces/moments
3. `MassAggregator` — aggregates mass properties

The old "system model" approach had automatic aggregation but violated IODA's flat peer model.

**Goal**: Get automatic aggregation while preserving IODA compliance.

---

## Design Philosophy: "Opt-In Aggregation via Convention"

Rather than hierarchical ownership, we use **signal conventions** to enable auto-discovery:

1. **Components remain IODA peers** — no parent/child ownership
2. **Components self-declare capabilities** via standard signal patterns
3. **Vehicle6DOF discovers providers** within its entity namespace
4. **Configuration override** available when auto-discovery is insufficient

This is similar to how Go uses interface satisfaction by method signature rather than explicit declaration.

---

## Component Frame Concept (Key Design Decision)

### The Problem with Body-Frame Outputs

Consider a thruster mounted at 45° on a rocket. In the **current design**, the component would need to:
1. Know its mounting orientation
2. Transform its thrust vector from local frame to body frame
3. Output the result in body frame

This has problems:
- **Duplicated logic**: Every force-producing component handles its own frame transform
- **Coupling**: Component knows about vehicle geometry
- **Less reusable**: Can't reuse the same thruster component at different mount angles

### The Insight: Component Local Frame

Every physical component naturally operates in its **own local frame**:
- A thruster produces thrust along its nozzle axis (typically +X or -Z in local frame)
- A fuel tank has mass properties symmetric about its own axis
- A sensor measures in its own boresight direction

The **attachment** (where and how the component is mounted) is configuration, not computed physics.

### Two-Frame Model

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Vehicle Body Frame                              │
│                                                                              │
│    Body Origin (0,0,0)                                                       │
│         │                                                                    │
│         │  r_body = [2, 0, 5] m                                              │
│         │  (attachment position)                                             │
│         ▼                                                                    │
│    ┌─────────────────────┐                                                   │
│    │   Component Frame   │◄── q_comp_to_body (attachment orientation)        │
│    │                     │                                                   │
│    │   Origin: [0,0,0]   │    Component outputs in LOCAL frame:              │
│    │   +X: thrust axis   │    - force_local = [thrust, 0, 0]                 │
│    │   +Z: "up" locally  │    - moment_local = [0, 0, 0]                     │
│    │                     │    - mass_local, cg_local, inertia_local          │
│    └─────────────────────┘                                                   │
│                                                                              │
│    Vehicle6DOF transforms to body frame during aggregation:                  │
│    F_body = q_comp_to_body.rotate(F_local)                                   │
│    M_body = r_body × F_body + q_comp_to_body.rotate(M_local)                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Class Hierarchy: PhysicalComponent

**Decision**: Use a `PhysicalComponent` intermediate class for components with body attachment.

```cpp
// Base Component - no attachment (Environment, schedulers, etc.)
template <typename Scalar>
class Component {
public:
    // Virtual accessors - base class returns "no attachment"
    [[nodiscard]] virtual bool HasBodyAttachment() const { return false; }
    [[nodiscard]] virtual Vec3<Scalar> GetBodyPosition() const {
        return Vec3<Scalar>::Zero();
    }
    [[nodiscard]] virtual janus::Quaternion<Scalar> GetBodyOrientation() const {
        return janus::Quaternion<Scalar>::identity();
    }

    // ... existing lifecycle methods ...
};

// Physical Component - adds body attachment (mass sources, force sources, sensors)
template <typename Scalar>
class PhysicalComponent : public Component<Scalar> {
protected:
    Vec3<Scalar> body_position_ = Vec3<Scalar>::Zero();
    janus::Quaternion<Scalar> body_orientation_;  // body-to-component, identity default
    bool has_body_attachment_ = false;

    /**
     * @brief Read attachment from config (call in Stage)
     *
     * Supports both quaternion and Euler angle specification:
     *   body_position: [x, y, z]                    # meters
     *   body_orientation: [w, x, y, z]              # quaternion (body-to-component)
     *   body_orientation_euler_zyx: [yaw, pitch, roll]  # degrees (alternative)
     */
    void ReadAttachmentFromConfig() {
        body_position_ = this->read_param_vec3("body_position", Vec3<Scalar>::Zero());

        // Check for Euler angles first (more user-friendly)
        if (this->GetConfig().template Has<Vec3<double>>("body_orientation_euler_zyx")) {
            auto euler_deg = this->GetConfig().template Get<Vec3<double>>(
                "body_orientation_euler_zyx", Vec3<double>::Zero());

            // Convert degrees to radians and build quaternion
            double yaw = euler_deg(0) * deg2rad;
            double pitch = euler_deg(1) * deg2rad;
            double roll = euler_deg(2) * deg2rad;
            body_orientation_ = janus::Quaternion<Scalar>::from_euler_zyx(yaw, pitch, roll);
        } else {
            // Quaternion format
            auto q = this->read_param_vec4("body_orientation", Vec4<Scalar>{1,0,0,0});
            body_orientation_ = janus::Quaternion<Scalar>{q(0), q(1), q(2), q(3)};
        }

        has_body_attachment_ = true;
    }

public:
    [[nodiscard]] bool HasBodyAttachment() const override { return has_body_attachment_; }
    [[nodiscard]] Vec3<Scalar> GetBodyPosition() const override { return body_position_; }
    [[nodiscard]] janus::Quaternion<Scalar> GetBodyOrientation() const override {
        return body_orientation_;
    }
};
```

**Usage:**
```cpp
// Physical components inherit from PhysicalComponent
template <typename Scalar>
class RocketEngine : public PhysicalComponent<Scalar> { ... };

template <typename Scalar>
class StaticMass : public PhysicalComponent<Scalar> { ... };

// Non-physical components inherit directly from Component
template <typename Scalar>
class GravityModel : public Component<Scalar> { ... };

template <typename Scalar>
class AtmosphereModel : public Component<Scalar> { ... };
```

**Benefits:**
- Virtual methods in base Component enable uniform querying by Vehicle6DOF
- No storage overhead for non-physical components
- Clear semantic distinction between physical and non-physical components
- Euler angle support built-in for user convenience

### Orientation Convention: Body-to-Component

**Decision**: `body_orientation` represents the rotation **from body frame to component frame**.

**Rationale (CAD Perspective)**:
When mounting a GPS sensor that needs to face "up" (+Z in NED):
1. Start with GPS at body origin, aligned with body axes
2. Rotate 90° pitch (about Y): `body_orientation_euler_zyx: [0, 90, 0]`
3. Now GPS local +Z points along body +X (forward → up)

The Euler angles describe "how I rotated this component from the body frame."

**Frame Transformation Math**:
```cpp
// body_orientation = R_body_to_comp (transforms body vectors to component frame)
//
// To transform component outputs TO body frame, use the conjugate (inverse):
//   F_body = body_orientation.conjugate().rotate(F_local)
//   M_body = body_orientation.conjugate().rotate(M_local)
//
// To transform body vectors TO component frame (e.g., for sensors):
//   v_local = body_orientation.rotate(v_body)
```

**YAML Examples**:
```yaml
# Thruster pointing along body -Z (typical rocket main engine)
# Component +X is thrust axis, needs to point to body -Z
# Rotate 90° about Y, then 180° about X (or equivalently, -90° about Y for +X → -Z)
body_orientation_euler_zyx: [0.0, -90.0, 0.0]

# GPS facing "up" (body +Z in aircraft NED body frame)
# Component +Z is boresight, needs to point to body -Z (down in NED = up in world)
body_orientation_euler_zyx: [0.0, 0.0, 180.0]  # Roll 180° to flip

# Star tracker on side of spacecraft
body_orientation_euler_zyx: [90.0, 0.0, 0.0]   # Yaw 90° to face +Y
```

---

## Signal Conventions for Physical Components

### Mass Provider Convention

A component that provides mass outputs these signals **in its local frame**:

```
{entity}.{component}.mass           [kg]      - Total mass
{entity}.{component}.cg_local.x/y/z [m]       - CG in COMPONENT frame
{entity}.{component}.inertia_local.xx/yy/zz/xy/xz/yz  [kg*m^2] - Inertia about local CG

# Attachment (from config, optionally exposed as signals)
{entity}.{component}.attachment.position.x/y/z      [m]  - Position in body frame
{entity}.{component}.attachment.orientation.w/x/y/z [-]  - Quaternion (comp→body)
```

Vehicle6DOF transforms to body frame:
```cpp
// CG in body frame
Vec3<Scalar> cg_body = body_position + body_orientation.rotate(cg_local);

// Inertia transformation (rotation + parallel axis theorem)
auto mass_props_body = transform_mass_properties(mass_props_local,
                                                  body_position,
                                                  body_orientation);
```

### Force Provider Convention

A component that provides forces outputs **in its local frame**:

```
{entity}.{component}.force_local.x/y/z    [N]     - Force in COMPONENT frame
{entity}.{component}.moment_local.x/y/z   [N*m]   - Moment about component origin (optional)

# Attachment (from config)
{entity}.{component}.attachment.position.x/y/z      [m]
{entity}.{component}.attachment.orientation.w/x/y/z [-]
```

Vehicle6DOF transforms to body frame:
```cpp
// Transform force to body frame
Vec3<Scalar> F_body = body_orientation.rotate(F_local);

// Transform moment and add moment arm contribution
Vec3<Scalar> M_body = body_orientation.rotate(M_local)
                    + (body_position - cg).cross(F_body);
```

### Environmental Force Convention (No Attachment)

Environmental forces (gravity, drag, etc.) that act at CG or in global frames:

```
{entity}.{component}.force_ecef.x/y/z    [N]   - Force in ECEF frame (gravity)
# OR
{entity}.{component}.force_body.x/y/z    [N]   - Force in body frame (aero)
```

These have no attachment — they're applied directly without transformation.

### Discovery Mechanism

During `Stage()`, Vehicle6DOF:

1. Queries backplane for all components in its entity namespace
2. For each component, checks signal patterns to identify providers
3. Resolves handles and queries attachment via base Component API
4. Builds handle bundles with frame transformation info

```cpp
void DiscoverProviders(Backplane<Scalar>& bp) {
    auto components = bp.components_in_entity(entity_);

    for (const auto& comp_name : components) {
        if (comp_name == name_) continue;  // Skip self

        std::string prefix = entity_ + "." + comp_name + ".";
        Component<Scalar>* comp_ptr = bp.get_component(entity_, comp_name);

        // Check for mass provider pattern (local frame)
        if (bp.has_signal(prefix + "mass") &&
            bp.has_signal(prefix + "cg_local.x") &&
            bp.has_signal(prefix + "inertia_local.xx")) {

            MassSourceBundle bundle;
            bundle.name = comp_name;
            bundle.mass = bp.resolve<Scalar>(prefix + "mass");
            bundle.cg_local = resolve_vec3(bp, prefix + "cg_local");
            bundle.inertia_local = resolve_inertia(bp, prefix + "inertia_local");

            // Get attachment from component (base class API)
            if (comp_ptr && comp_ptr->HasBodyAttachment()) {
                bundle.body_position = comp_ptr->GetBodyPosition();
                bundle.body_orientation = comp_ptr->GetBodyOrientation();
                bundle.has_attachment = true;
            }

            mass_sources_.push_back(bundle);
        }

        // Check for local-frame force provider
        if (bp.has_signal(prefix + "force_local.x")) {
            ForceSourceBundle bundle;
            bundle.name = comp_name;
            bundle.force_local = resolve_vec3(bp, prefix + "force_local");
            bundle.moment_local = resolve_vec3_optional(bp, prefix + "moment_local");
            bundle.frame_type = ForceFrame::Local;

            if (comp_ptr && comp_ptr->HasBodyAttachment()) {
                bundle.body_position = comp_ptr->GetBodyPosition();
                bundle.body_orientation = comp_ptr->GetBodyOrientation();
            }

            force_sources_.push_back(bundle);
        }

        // Check for body-frame force (no transform needed)
        else if (bp.has_signal(prefix + "force_body.x")) {
            ForceSourceBundle bundle;
            bundle.name = comp_name;
            bundle.force = resolve_vec3(bp, prefix + "force_body");
            bundle.frame_type = ForceFrame::Body;
            force_sources_.push_back(bundle);
        }

        // Check for ECEF-frame force (transform via attitude)
        else if (bp.has_signal(prefix + "force_ecef.x")) {
            ForceSourceBundle bundle;
            bundle.name = comp_name;
            bundle.force = resolve_vec3(bp, prefix + "force_ecef");
            bundle.frame_type = ForceFrame::ECEF;
            force_sources_.push_back(bundle);
        }
    }
}
```

---

## Component Architecture

### Vehicle6DOF Component

```cpp
template <typename Scalar>
class Vehicle6DOF : public Component<Scalar> {
    // === Identity ===
    std::string name_;
    std::string entity_;

    // === 6DOF State (13 variables) ===
    Vec3<Scalar> position_;           // ECEF position [m]
    Vec3<Scalar> velocity_body_;      // Body-frame velocity [m/s]
    Vec4<Scalar> attitude_;           // Quaternion (body-to-ECEF)
    Vec3<Scalar> omega_body_;         // Angular velocity [rad/s]

    // === Derivatives ===
    Vec3<Scalar> position_dot_;
    Vec3<Scalar> velocity_body_dot_;
    Vec4<Scalar> attitude_dot_;
    Vec3<Scalar> omega_body_dot_;

    // === Aggregated Properties (internal, not wired) ===
    vulcan::mass::MassProperties<Scalar> total_mass_props_;
    Vec3<Scalar> total_force_body_;
    Vec3<Scalar> total_moment_body_;

    // === Discovered Providers ===
    std::vector<MassSourceHandles<Scalar>> mass_sources_;
    std::vector<ForceSourceHandles<Scalar>> body_force_sources_;
    std::vector<ForceSourceHandles<Scalar>> ecef_force_sources_;

    // === Configuration ===
    bool auto_discover_ = true;  // Default: discover providers automatically
    std::vector<std::string> explicit_mass_sources_;   // Override: explicit list
    std::vector<std::string> explicit_force_sources_;  // Override: explicit list
};
```

### Lifecycle Implementation

#### Provision Phase

```cpp
void Provision(Backplane<Scalar>& bp) override {
    // === Register State Outputs ===
    bp.register_state_vec3<Scalar>("position", &position_, &position_dot_, "m", "ECEF position");
    bp.register_state_vec3<Scalar>("velocity_body", &velocity_body_, &velocity_body_dot_, "m/s", "Body velocity");
    bp.register_state_quat<Scalar>("attitude", &attitude_, &attitude_dot_, "", "Body-to-ECEF quaternion");
    bp.register_state_vec3<Scalar>("omega_body", &omega_body_, &omega_body_dot_, "rad/s", "Angular velocity");

    // === Register Derived Outputs ===
    bp.register_output_vec3<Scalar>("velocity_ref", &velocity_ref_, "m/s", "ECEF velocity");
    bp.register_output<Scalar>("position_lla.lat", &position_lla_(0), "rad", "Latitude");
    bp.register_output<Scalar>("position_lla.lon", &position_lla_(1), "rad", "Longitude");
    bp.register_output<Scalar>("position_lla.alt", &position_lla_(2), "m", "Altitude");
    // ... NED velocity, Euler angles ...

    // === Register Aggregated Mass Outputs (for consumers) ===
    bp.register_output<Scalar>("total_mass", &total_mass_, "kg", "Aggregated mass");
    bp.register_output_vec3<Scalar>("cg", &total_cg_, "m", "Aggregated CG");
    bp.register_output<Scalar>("inertia.xx", &total_inertia_xx_, "kg*m^2", "Aggregated Ixx");
    // ... other inertia components ...

    // === Register Aggregated Force Outputs (for debugging/telemetry) ===
    bp.register_output_vec3<Scalar>("total_force", &total_force_body_, "N", "Total body force");
    bp.register_output_vec3<Scalar>("total_moment", &total_moment_body_, "N*m", "Total body moment");

    // NOTE: No inputs registered! Vehicle discovers providers via signals.
}
```

#### Stage Phase (Discovery)

```cpp
void Stage(Backplane<Scalar>& bp) override {
    const auto& config = GetConfig();

    // Configuration options
    auto_discover_ = read_param("auto_discover", true);
    explicit_mass_sources_ = config.Get<std::vector<std::string>>("mass_sources", {});
    explicit_force_sources_ = config.Get<std::vector<std::string>>("force_sources", {});

    // Apply initial conditions (position, velocity, attitude)
    ApplyInitialConditions(config);

    // === Discovery Phase ===
    if (auto_discover_) {
        DiscoverMassProviders(bp);
        DiscoverForceProviders(bp);
    }

    // Explicit sources override/supplement auto-discovery
    for (const auto& name : explicit_mass_sources_) {
        ResolveMassSource(bp, name);
    }
    for (const auto& name : explicit_force_sources_) {
        ResolveForceSource(bp, name);
    }
}

void DiscoverMassProviders(Backplane<Scalar>& bp) {
    // Get all component names in our entity
    auto components = bp.components_in_entity(entity_);

    for (const auto& comp_name : components) {
        if (comp_name == name_) continue;  // Skip self

        std::string prefix = entity_ + "." + comp_name + ".";

        // Check for mass provider pattern
        if (bp.has_signal(prefix + "mass") &&
            bp.has_signal(prefix + "cg.x") &&
            bp.has_signal(prefix + "inertia.xx")) {

            ResolveMassSource(bp, comp_name);
        }
    }
}

void DiscoverForceProviders(Backplane<Scalar>& bp) {
    auto components = bp.components_in_entity(entity_);

    for (const auto& comp_name : components) {
        if (comp_name == name_) continue;

        std::string prefix = entity_ + "." + comp_name + ".";

        // Check for force provider pattern
        if (bp.has_signal(prefix + "force.x") &&
            bp.has_signal(prefix + "force.y") &&
            bp.has_signal(prefix + "force.z")) {

            // Determine frame (default to body if not specified)
            auto frame = bp.has_signal(prefix + "force_frame")
                ? GetFrameFromSignal(bp, prefix + "force_frame")
                : ForceFrame::Body;

            ResolveForceSource(bp, comp_name, frame);
        }
    }
}
```

#### Step Phase (Aggregation + Dynamics)

```cpp
void Step(Scalar t, Scalar dt) override {
    (void)t; (void)dt;

    // === 1. Aggregate Mass Properties ===
    AggregateMass();

    // === 2. Aggregate Forces/Moments ===
    AggregateForces();

    // === 3. Compute 6DOF Derivatives ===
    ComputeDynamics();

    // === 4. Update Derived Outputs ===
    ComputeCoordinateFrameOutputs();
}

void AggregateMass() {
    if (mass_sources_.empty()) {
        total_mass_props_ = default_mass_props_;
        return;
    }

    // Aggregate mass properties, transforming each from local to body frame
    vulcan::mass::MassProperties<Scalar> total;
    total.mass = Scalar(0);
    total.cg = Vec3<Scalar>::Zero();
    total.inertia = Mat3<Scalar>::Zero();

    for (const auto& src : mass_sources_) {
        // Get mass properties in component local frame
        vulcan::mass::MassProperties<Scalar> local;
        local.mass = *src.mass;
        local.cg = src.GetCgLocal();
        local.inertia = src.GetInertiaLocal();

        // Transform to body frame
        vulcan::mass::MassProperties<Scalar> body;
        if (src.has_attachment) {
            // body_orientation is body-to-component, so use conjugate for comp-to-body
            auto R_comp_to_body = src.body_orientation.conjugate();

            // CG in body frame: r_body + R_comp_to_body * cg_local
            body.cg = src.body_position + R_comp_to_body.rotate(local.cg);

            // Inertia transformation:
            // 1. Rotate inertia tensor: I_rotated = R * I_local * R^T
            // 2. Parallel axis theorem: I_body = I_rotated + m * (d^2 * I - d ⊗ d)
            body.inertia = vulcan::mass::transform_inertia(
                local.inertia, local.mass, R_comp_to_body, body.cg);
            body.mass = local.mass;
        } else {
            // No attachment - assume local frame IS body frame
            body = local;
        }

        // Aggregate using Vulcan's MassProperties::operator+
        total = total + body;
    }

    total_mass_props_ = total;
    total_mass_ = total.mass;
    total_cg_ = total.cg;
    // ... write inertia outputs ...
}

void AggregateForces() {
    Vec3<Scalar> sum_force = Vec3<Scalar>::Zero();
    Vec3<Scalar> sum_moment = Vec3<Scalar>::Zero();

    // Vehicle attitude for ECEF transforms
    janus::Quaternion<Scalar> q_body_to_ecef{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

    for (const auto& src : force_sources_) {
        Vec3<Scalar> F_body;
        Vec3<Scalar> M_body;
        Vec3<Scalar> r_app;  // Application point in body frame

        switch (src.frame_type) {
            case ForceFrame::Local: {
                // Force in component local frame - transform via attachment
                Vec3<Scalar> F_local = src.GetForceLocal();
                Vec3<Scalar> M_local = src.GetMomentLocal();

                // body_orientation is body-to-component, use conjugate for comp-to-body
                auto R_comp_to_body = src.body_orientation.conjugate();

                // Transform force: F_body = R_comp_to_body * F_local
                F_body = R_comp_to_body.rotate(F_local);

                // Transform moment and compute application point
                M_body = R_comp_to_body.rotate(M_local);
                r_app = src.body_position;  // Force applied at component origin
                break;
            }

            case ForceFrame::Body: {
                // Already in body frame - no transform needed
                F_body = src.GetForce();
                M_body = Vec3<Scalar>::Zero();
                r_app = total_mass_props_.cg;  // Applied at CG (no moment arm)
                break;
            }

            case ForceFrame::ECEF: {
                // Transform from ECEF to body using vehicle attitude
                Vec3<Scalar> F_ecef = src.GetForce();
                F_body = q_body_to_ecef.conjugate().rotate(F_ecef);
                M_body = Vec3<Scalar>::Zero();
                r_app = total_mass_props_.cg;  // Applied at CG
                break;
            }
        }

        // Sum forces
        sum_force += F_body;

        // Moment about CG: M_cg = M_app + (r_app - r_cg) × F_body
        Vec3<Scalar> lever_arm = r_app - total_mass_props_.cg;
        sum_moment += M_body + lever_arm.cross(F_body);
    }

    total_force_body_ = sum_force;
    total_moment_body_ = sum_moment;
}

void ComputeDynamics() {
    janus::Quaternion<Scalar> quat{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

    vulcan::dynamics::RigidBodyState<Scalar> state;
    state.position = position_;
    state.velocity_body = velocity_body_;
    state.attitude = quat;
    state.omega_body = omega_body_;

    auto derivs = vulcan::dynamics::compute_6dof_derivatives(
        state, total_force_body_, total_moment_body_, total_mass_props_);

    position_dot_ = derivs.position_dot;
    velocity_body_dot_ = derivs.velocity_dot;
    attitude_dot_ = Vec4<Scalar>{derivs.attitude_dot.w, derivs.attitude_dot.x,
                                  derivs.attitude_dot.y, derivs.attitude_dot.z};
    omega_body_dot_ = derivs.omega_dot;
}
```

---

## YAML Configuration

### Minimal (Auto-Discovery)

```yaml
components:
  # Vehicle automatically discovers mass/force providers in "Rocket" entity
  - type: Vehicle6DOF
    name: Vehicle
    entity: Rocket
    initial_lla: [28.5, -80.6, 0.0]        # Cape Canaveral
    initial_euler_zyx: [90.0, 85.0, 0.0]   # Pointing East, 85° pitch up
    initial_velocity_body: [0.0, 0.0, 0.0]

  # Mass provider - outputs in LOCAL frame, Vehicle transforms to body
  - type: StaticMass
    name: Structure
    entity: Rocket
    body_position: [0.0, 0.0, 0.0]         # At body origin
    body_orientation: [1.0, 0.0, 0.0, 0.0] # Identity (aligned with body)
    mass: 1000.0
    cg_local: [0.0, 0.0, 2.0]              # CG 2m along local Z
    inertia_local: [100.0, 100.0, 50.0, 0.0, 0.0, 0.0]

  # Fuel tank at specific body location
  - type: FuelTank
    name: Propellant
    entity: Rocket
    body_position: [0.0, 0.0, 5.0]         # 5m along body Z
    body_orientation: [1.0, 0.0, 0.0, 0.0] # Aligned with body
    initial_mass: 9000.0
    cg_local: [0.0, 0.0, 0.0]              # CG at tank origin
    # ... tank geometry for inertia ...

  # Main engine - thrust along LOCAL +X, mounted at body [0,0,10]
  - type: RocketEngine
    name: MainEngine
    entity: Rocket
    body_position: [0.0, 0.0, 10.0]        # Nozzle location in body frame
    body_orientation_euler_zyx: [0.0, -90.0, 0.0]  # Pitch -90° so local +X → body -Z
    thrust: 100000.0
    # Outputs: force_local = [thrust, 0, 0] (always along nozzle axis)
    # Vehicle transforms: F_body = R.rotate([thrust, 0, 0]) = [0, 0, -thrust]

  # Vernier engine - canted 10° for roll control
  - type: RocketEngine
    name: Vernier1
    entity: Rocket
    body_position: [1.0, 0.0, 9.5]         # Offset from centerline
    body_orientation_euler_zyx: [0.0, -80.0, 0.0]  # 10° cant from vertical
    thrust: 5000.0
    # Same component type, different mounting = different body-frame force

  # Gravity - acts in ECEF frame, no attachment needed
  - type: SphericalGravity
    name: Gravity
    entity: Rocket
    # Outputs: force_ecef.x/y/z (no body_position/orientation needed)
```

### Why This Is Better

The **same RocketEngine component** produces the same output (`force_local = [thrust, 0, 0]`) regardless of mounting. The YAML configures WHERE it's mounted, and Vehicle6DOF handles the transformation:

```
MainEngine:
  body_position = [0, 0, 10]
  body_orientation = 90° rotation about Y
  force_local = [100000, 0, 0]
  → F_body = [0, 0, -100000]  (thrust along -Z body axis)

Vernier1:
  body_position = [1, 0, 9.5]
  body_orientation = 10° cant
  force_local = [5000, 0, 0]
  → F_body = [~4900, 0, -870]  (mostly -Z with small roll component)
```

### Explicit Override

```yaml
components:
  - type: Vehicle6DOF
    name: Vehicle
    entity: Rocket
    auto_discover: false  # Disable auto-discovery
    mass_sources: [Structure, Propellant]
    force_body_sources: [MainEngine, RCS]
    force_ecef_sources: [Gravity, Drag]
    # ... initial conditions ...
```

### Hybrid (Auto + Explicit Exclusions)

```yaml
components:
  - type: Vehicle6DOF
    name: Vehicle
    entity: Rocket
    auto_discover: true
    exclude_mass_sources: [DebugMass]      # Skip these in discovery
    exclude_force_sources: [TestForce]
    additional_force_sources: [External.Perturbation]  # Include cross-entity
```

---

## Component Self-Declaration Pattern

Components that want to be discoverable simply output the standard signals:

### Example: StaticMass Component

```cpp
template <typename Scalar>
class StaticMass : public Component<Scalar> {
    void Provision(Backplane<Scalar>& bp) override {
        // These signals make us a "mass provider"
        bp.register_output<Scalar>("mass", &mass_, "kg", "Total mass");
        bp.register_output_vec3<Scalar>("cg", &cg_, "m", "CG position");
        bp.register_output<Scalar>("inertia.xx", &Ixx_, "kg*m^2", "Ixx");
        bp.register_output<Scalar>("inertia.yy", &Iyy_, "kg*m^2", "Iyy");
        bp.register_output<Scalar>("inertia.zz", &Izz_, "kg*m^2", "Izz");
        bp.register_output<Scalar>("inertia.xy", &Ixy_, "kg*m^2", "Ixy");
        bp.register_output<Scalar>("inertia.xz", &Ixz_, "kg*m^2", "Ixz");
        bp.register_output<Scalar>("inertia.yz", &Iyz_, "kg*m^2", "Iyz");

        // Optional: body position (for display/debugging)
        bp.register_output_vec3<Scalar>("body_position", &body_pos_, "m", "Body frame position");
    }

    void Step(Scalar, Scalar) override {
        // Static mass never changes
    }
};
```

### Example: RocketEngine Component

```cpp
template <typename Scalar>
class RocketEngine : public Component<Scalar> {
    void Provision(Backplane<Scalar>& bp) override {
        // These signals make us a "force provider"
        bp.register_output_vec3<Scalar>("force", &thrust_vec_, "N", "Thrust force (body)");
        bp.register_output_vec3<Scalar>("moment", &moment_, "N*m", "Thrust moment");
        bp.register_output_vec3<Scalar>("application_point", &nozzle_pos_, "m", "Nozzle location");

        // Frame indicator (helps Vehicle know how to handle us)
        // Note: Could be implicit "body" if not specified

        // Mass flow output (consumed by fuel tank)
        bp.register_output<Scalar>("mass_flow", &mdot_, "kg/s", "Propellant mass flow");

        // Inputs
        bp.register_input<Scalar>("throttle", &throttle_, "", "Throttle command");
    }

    void Step(Scalar, Scalar) override {
        // Compute thrust based on throttle
        Scalar thrust_mag = throttle_.get() * max_thrust_;
        thrust_vec_ = thrust_direction_ * thrust_mag;
        moment_ = Vec3<Scalar>::Zero();  // Thrust through nozzle center
        mdot_ = thrust_mag / (Isp_ * g0_);
    }
};
```

---

## Backplane Extensions Required

### New Methods for Discovery

```cpp
template <typename Scalar>
class Backplane {
public:
    // === Discovery API ===

    /**
     * @brief Get all component names in an entity namespace
     */
    std::vector<std::string> components_in_entity(const std::string& entity) const {
        return registry_.components_in_entity(entity);
    }

    /**
     * @brief Get all signals matching a prefix
     */
    std::vector<std::string> signals_with_prefix(const std::string& prefix) const {
        return registry_.signals_with_prefix(prefix);
    }

    /**
     * @brief Check if signal exists (already implemented)
     */
    bool has_signal(const std::string& name) const;
};
```

### SignalRegistry Extensions

```cpp
template <typename Scalar>
class SignalRegistry {
public:
    // === Discovery Support ===

    std::vector<std::string> components_in_entity(const std::string& entity) const {
        std::set<std::string> components;
        std::string prefix = entity + ".";

        for (const auto& [name, _] : signals_) {
            if (name.starts_with(prefix)) {
                // Extract component name: "Rocket.Engine.force.x" → "Engine"
                auto rest = name.substr(prefix.size());
                auto dot_pos = rest.find('.');
                if (dot_pos != std::string::npos) {
                    components.insert(rest.substr(0, dot_pos));
                }
            }
        }
        return {components.begin(), components.end()};
    }

    std::vector<std::string> signals_with_prefix(const std::string& prefix) const {
        std::vector<std::string> result;
        for (const auto& [name, _] : signals_) {
            if (name.starts_with(prefix)) {
                result.push_back(name);
            }
        }
        return result;
    }
};
```

---

## Comparison: Old system model vs. New Vehicle6DOF

| Aspect | Old system model | IODA Separate | New Vehicle6DOF |
|:-------|:-----------|:--------------|:----------------|
| Component ownership | Hierarchical | Flat peers | Flat peers |
| Mass aggregation | Automatic | Manual wiring | Auto-discovered |
| Force aggregation | Automatic | Manual wiring | Auto-discovered |
| Body positions | Component config | N/A | Component signals |
| Wiring complexity | None | ~26 routes | ~0 routes |
| IODA compliance | No | Yes | Yes |
| Testability | Monolithic | Per-component | Per-component |
| Composability | Low | High | High |

---

## Implementation Phases

### Phase 1: Component Attachment Infrastructure

1. Add virtual accessors to `Component<Scalar>` base class (default "no attachment"):
   - `HasBodyAttachment()` → false
   - `GetBodyPosition()` → zero vector
   - `GetBodyOrientation()` → identity quaternion
2. Create `PhysicalComponent<Scalar>` intermediate class:
   - Inherits from `Component<Scalar>`
   - Adds `body_position_`, `body_orientation_`, `has_body_attachment_` members
   - Overrides virtual accessors
   - Implements `ReadAttachmentFromConfig()` helper with:
     - `body_position: [x, y, z]` support
     - `body_orientation: [w, x, y, z]` quaternion support
     - `body_orientation_euler_zyx: [yaw, pitch, roll]` degrees support
   - Convention: body-to-component orientation
3. Add `vulcan::mass::transform_inertia()` utility for inertia frame transformation
4. Add `janus::Quaternion::from_euler_zyx()` if not already present
5. Unit tests for attachment parsing and transformation

### Phase 2: Backplane Discovery API

1. Add `components_in_entity()` to SignalRegistry
2. Add `signals_with_prefix()` to SignalRegistry
3. Add `get_component()` to retrieve component pointer by name
4. Expose via Backplane facade
5. Unit tests for discovery

### Phase 3: Vehicle6DOF Component

1. Create `components/dynamics/Vehicle6DOF.hpp`
2. Implement state registration (copy from RigidBody6DOF)
3. Implement provider discovery with frame detection:
   - Mass providers: `mass` + `cg_local.*` + `inertia_local.*`
   - Local force providers: `force_local.*`
   - Body force providers: `force_body.*`
   - ECEF force providers: `force_ecef.*`
4. Implement mass aggregation with frame transformation
5. Implement force aggregation with frame transformation
6. Implement 6DOF dynamics (reuse Vulcan)
7. Register aggregated outputs for telemetry

### Phase 4: Update Existing Components

1. Change mass components to inherit from `PhysicalComponent<Scalar>`:
   - `StaticMass`, `FuelTank`, etc.
   - Call `ReadAttachmentFromConfig()` in Stage()
   - Output `cg_local` and `inertia_local` signals (local frame)
2. Change force components to inherit from `PhysicalComponent<Scalar>`:
   - `RocketEngine`, `Thruster`, etc.
   - Output `force_local` (not body frame)
   - Remove internal frame transformation logic (Vehicle handles it)
3. Keep environmental components as `Component<Scalar>`:
   - `GravityModel` → outputs `force_ecef`
   - `AeroModel` → outputs `force_body` (already in body frame)
4. Ensure backwards compatibility (old signal names as aliases if needed)

### Phase 5: Examples and Documentation

1. Create `rocket_vehicle.yaml` demonstrating:
   - Auto-discovery
   - Component attachment (position + orientation)
   - Mixed frame types (local, body, ECEF)
2. Create migration guide from separate components
3. Update architecture documentation
4. Add signal convention reference document

---

## Design Decisions (Resolved)

### 1. Class Hierarchy: PhysicalComponent

**Decision**: Use `PhysicalComponent<Scalar>` intermediate class.

- Virtual accessors in base `Component` (default "no attachment")
- `PhysicalComponent` adds storage and `ReadAttachmentFromConfig()` helper
- Physical components (mass/force sources) inherit from `PhysicalComponent`
- Non-physical components (environment, schedulers) inherit from `Component`

### 2. Orientation Convention: Body-to-Component

**Decision**: `body_orientation` represents rotation **from body frame to component frame**.

- Matches CAD intuition: Euler angles describe "how I rotated this component"
- For force transformation: `F_body = body_orientation.conjugate().rotate(F_local)`
- Supports both quaternion and Euler angle config

### 3. Signal Naming for Force Frames

**Decision**: Use naming convention for discovery:
- `force_local.x/y/z` — Component local frame (transform via attachment)
- `force_body.x/y/z` — Vehicle body frame (no transform)
- `force_ecef.x/y/z` — ECEF frame (transform via vehicle attitude)

### 4. Euler Angle Support

**Decision**: Support `body_orientation_euler_zyx: [yaw, pitch, roll]` in degrees.

- Quaternion format also supported: `body_orientation: [w, x, y, z]`
- Euler angles checked first (more user-friendly)

---

## Open Design Questions

### 1. Default Behavior When No Sources Found

When no mass/force sources are discovered:

**Option A: Error (Strict)**
- Fail Stage() with clear error message
- Forces user to configure at least one source

**Option B: Warning + Defaults (Recommended)**
- Log warning about missing sources
- Use configurable default mass properties (e.g., 1 kg point mass)
- Allows incremental development and testing

**Option C: Silent Fallback**
- No warning, just use defaults
- Could hide configuration mistakes

**Recommendation**: Option B. Warn but don't fail.

### 4. Deprecate Aggregator Components?

Should we deprecate `MassAggregator` and `ForceAggregator`?

**Recommendation**: No. Keep them as "power user" components for:
- Non-vehicle aggregation (e.g., ground test stand)
- Cross-entity aggregation
- Debugging individual aggregations
- Gradual migration path

### 5. Multiple Vehicles per Simulation

How do we handle multi-body simulations (e.g., rocket + payload)?

**Recommendation**: Each vehicle has its own entity namespace. Cross-vehicle forces (separation, contact) are separate components that read from both entities.

---

## Related Documents

- [01_core_philosophy.md](../architecture/01_core_philosophy.md) — IODA principles
- [06_entities_namespaces.md](../architecture/06_entities_namespaces.md) — Entity namespace design
- [12_force_aggregation.md](../architecture/12_force_aggregation.md) — Force aggregation theory

---

## Appendix: Migration Example

### Before (26 routes)

```yaml
components:
  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    # ... ICs ...

  - type: MassAggregator
    name: Mass
    entity: Rocket
    sources: [Structure, Propellant]

  - type: ForceAggregator
    name: Forces
    entity: Rocket
    body_sources: [MainEngine]
    ecef_sources: [Gravity]

routes:
  # Mass → Forces (CG for moment transfer)
  - input: Rocket.Forces.cg.x
    output: Rocket.Mass.cg.x
  # ... 2 more ...

  # EOM → Forces (attitude for transform)
  - input: Rocket.Forces.attitude.w
    output: Rocket.EOM.attitude.w
  # ... 3 more ...

  # Forces → EOM
  - input: Rocket.EOM.total_force.x
    output: Rocket.Forces.total_force.x
  # ... 5 more ...

  # Mass → EOM
  - input: Rocket.EOM.total_mass
    output: Rocket.Mass.total_mass
  # ... 6 more ...
```

### After (0 routes)

```yaml
components:
  - type: Vehicle6DOF
    name: Vehicle
    entity: Rocket
    initial_lla: [28.5, -80.6, 0.0]
    initial_euler_zyx: [90.0, 85.0, 0.0]

  - type: StaticMass
    name: Structure
    entity: Rocket
    mass: 1000.0
    cg: [0.0, 0.0, 2.0]
    inertia: [100, 100, 50, 0, 0, 0]

  - type: FuelTank
    name: Propellant
    entity: Rocket
    initial_mass: 9000.0
    cg: [0.0, 0.0, 5.0]

  - type: RocketEngine
    name: MainEngine
    entity: Rocket
    thrust: 100000.0
    application_point: [0.0, 0.0, 10.0]

  - type: SphericalGravity
    name: Gravity
    entity: Rocket

# No routes needed! Vehicle6DOF discovers and aggregates automatically.
```
