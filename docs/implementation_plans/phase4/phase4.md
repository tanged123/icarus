# Phase 4: Aggregation & 6DOF Implementation Plan

**Status:** Proposed
**Goal:** Multi-component force/moment and mass property aggregation with full 6DOF dynamics

---

## Overview

Phase 4 enables **quantity aggregation** where multiple components contribute forces, moments, or mass properties to a single consumer (the equations of motion). This phase implements:

1. **Signal conventions** for force and mass sources
2. **Aggregator components** with explicit config-driven wiring
3. **RigidBody6DOF** dynamics component consuming aggregated quantities
4. **Gold standard component patterns** for all future components

> [!IMPORTANT]
> **No Special APIs.** Aggregation uses standard signals and explicit wiring. No `register_force_source()` or auto-discovery magic. Everything is explicit in config.

> [!NOTE]
> **Gold Standard.** The aggregator components establish the canonical patterns for component development.

---

## Architectural Decisions

### Pure Signal Approach (No Special Registration)

We use **standard signals with conventions**, not special registration APIs:

| Approach | Special Registration | Signal Conventions ✓ |
|:---------|:--------------------|:---------------------|
| APIs | `register_force_source()` | `register_output()` (standard) |
| Discovery | Auto-discovery by type | Explicit wiring in config |
| Coupling | Implicit | Explicit |
| Complexity | Two parallel systems | One unified system |
| Debugging | Hidden connections | Visible in config |

**Decision:** Use standard signals. Aggregators wire explicitly via config.

### Component Categories

Not all components have physical presence. The base `Component<Scalar>` stays minimal:

| Category | Examples | Has Location? | Has Mass? |
|:---------|:---------|:--------------|:----------|
| Physical hardware | Engines, tanks, sensors | Yes | Maybe |
| Environment models | Gravity, atmosphere | No | No |
| Dynamics/EOM | RigidBody6DOF | No | No |
| Aggregators | ForceAggregator | No | No |
| Computational | Navigation, control | No | No |

**Decision:** Components that need location/mass/frame define them. No base class bloat.

### Vulcan MassProperties as Signal Type

Mass sources publish `vulcan::mass::MassProperties<Scalar>` directly:

```cpp
template <typename Scalar>
struct MassProperties {
    Scalar mass;           // [kg]
    Vec3<Scalar> cg;       // CG in body frame [m]
    Mat3<Scalar> inertia;  // About CG [kg·m²]
};
```

**Benefits:**
- Single signal instead of three
- `operator+` handles parallel axis theorem automatically
- Factory constructors for shapes (sphere, cylinder, box)

### Forces Published in Body Frame

Force sources are responsible for transforming their forces to body frame before publishing. This keeps the aggregator simple:

```cpp
// Force source transforms to body frame in Step()
Vec3<Scalar> force_ecef = compute_gravity();
force_body_ = vulcan::coordinates::transform_vector(
    force_ecef, ecef_frame_, body_frame_);

// Aggregator just sums (no frame logic)
total_force_ = gravity_force_.get() + drag_force_.get() + thrust_force_.get();
```

**Benefits:**
- Aggregator is trivial (just summation)
- Frame knowledge stays with the source (where it belongs)
- No frame metadata to track

---

## Implementation Tasks

### 4.1 Signal Conventions

**Goal:** Define standard signal names and types for aggregation.

#### 4.1.1 Mass Source Convention

Mass-contributing components publish a `mass_properties` signal:

```cpp
// In component
vulcan::mass::MassProperties<Scalar> mass_props_;

void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    bp.register_output("mass_properties", &mass_props_, "kg,m,kg*m^2",
                       "Mass, CG, and inertia");
}

void Step(Scalar t, Scalar dt) override {
    mass_props_.mass = current_fuel_mass_;
    mass_props_.cg = fuel_cg_body_;
    mass_props_.inertia = fuel_inertia_;
}
```

#### 4.1.2 Force Source Convention

Force-contributing components publish:
- `force` (Vec3, **in body frame**, required)
- `moment` (Vec3, about CG, body frame, optional)
- `application_point` (Vec3, body frame, optional — for moment calculation by aggregator)

```cpp
// In component
Vec3<Scalar> force_body_;
Vec3<Scalar> application_point_;  // Optional

void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    bp.register_output_vec3("force", &force_body_, "N", "Force in body frame");
    bp.register_output_vec3("application_point", &application_point_, "m",
                            "Force application point in body frame");
}
```

#### 4.1.3 Backplane Extensions

Add support for `MassProperties<Scalar>` and `CoordinateFrame<Scalar>` as signal types:

```cpp
// MassProperties output
bp.register_output("mass_properties", &mass_props_, "kg,m,kg*m^2", "Mass properties");

// MassProperties input
InputHandle<vulcan::mass::MassProperties<Scalar>> mass_input_;
bp.register_input("mass_properties", &mass_input_, "kg,m,kg*m^2", "Mass input");

// CoordinateFrame output
bp.register_output("body_frame", &body_frame_, "", "Body coordinate frame");

// CoordinateFrame input
InputHandle<vulcan::coordinates::CoordinateFrame<Scalar>> frame_input_;
bp.register_input("body_frame", &frame_input_, "", "Body frame input");
```

#### Exit Criteria (4.1)

- [ ] `MassProperties<Scalar>` supported as signal type
- [ ] `CoordinateFrame<Scalar>` supported as signal type
- [ ] Unit tests for new signal types

---

### 4.2 Aggregator Components

**Goal:** Components that sum contributions via explicit wiring.

#### 4.2.1 MassAggregator

**File:** `components/aggregators/MassAggregator.hpp`

**Responsibility:** Sum mass properties from configured sources.

**Config-driven wiring:**
```yaml
MassAggregator:
  name: Mass
  entity: Rocket
  sources:
    - Structure.mass_properties
    - FuelTank.mass_properties
    - Payload.mass_properties
```

**Implementation:**
```cpp
template <typename Scalar>
class MassAggregator : public Component<Scalar> {
    // Inputs: wired from config
    std::vector<InputHandle<vulcan::mass::MassProperties<Scalar>>> sources_;

    // Outputs
    Scalar total_mass_;
    Vec3<Scalar> cg_;
    Mat3<Scalar> inertia_;
    vulcan::mass::MassProperties<Scalar> mass_props_;  // Combined output

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Outputs
        bp.register_output("total_mass", &total_mass_, "kg", "Total mass");
        bp.register_output_vec3("cg", &cg_, "m", "Center of gravity");
        bp.register_output("inertia", &inertia_, "kg*m^2", "Inertia tensor");
        bp.register_output("mass_properties", &mass_props_, "", "Combined mass properties");

        // Create input handles for each source (from config)
        for (const auto& source_path : cfg.mass_sources) {
            sources_.emplace_back();
            bp.register_input(source_path, &sources_.back(), "", "Mass source");
        }
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Wire each source
        for (size_t i = 0; i < cfg.mass_sources.size(); ++i) {
            bp.wire_input(cfg.mass_sources[i], cfg.mass_sources[i]);
        }
    }

    void Step(Scalar t, Scalar dt) override {
        // Use Vulcan's operator+ for aggregation (handles parallel axis)
        vulcan::mass::MassProperties<Scalar> total{};

        for (const auto& src : sources_) {
            total = total + src.get();
        }

        total_mass_ = total.mass;
        cg_ = total.cg;
        inertia_ = total.inertia;
        mass_props_ = total;
    }
};
```

#### 4.2.2 ForceAggregator

**File:** `components/aggregators/ForceAggregator.hpp`

**Responsibility:** Sum forces and compute moments about CG.

**Config-driven wiring:**
```yaml
ForceAggregator:
  name: Forces
  entity: Rocket
  cg_source: Mass.cg
  sources:
    - name: gravity
      force: Gravity.force
      acts_at_cg: true
    - name: drag
      force: Drag.force
      application_point: Drag.application_point
    - name: thrust
      force: Thrust.force
      moment: Thrust.moment
      application_point: Thrust.application_point
```

**Implementation:**
```cpp
template <typename Scalar>
class ForceAggregator : public Component<Scalar> {
    struct ForceInput {
        InputHandle<Vec3<Scalar>> force;
        InputHandle<Vec3<Scalar>> moment;           // Optional
        InputHandle<Vec3<Scalar>> application_point; // Optional
        bool acts_at_cg = false;
    };

    std::vector<ForceInput> sources_;
    InputHandle<Vec3<Scalar>> cg_;

    // Outputs
    Vec3<Scalar> total_force_;
    Vec3<Scalar> total_moment_;

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        bp.register_output_vec3("total_force", &total_force_, "N", "Total force");
        bp.register_output_vec3("total_moment", &total_moment_, "N*m", "Total moment about CG");

        bp.register_input_vec3("cg", &cg_, "m", "Center of gravity");

        // Register inputs for each source
        for (const auto& src_cfg : cfg.force_sources) {
            ForceInput input;
            bp.register_input_vec3(src_cfg.name + ".force", &input.force, "N", "Force");

            if (src_cfg.has_moment) {
                bp.register_input_vec3(src_cfg.name + ".moment", &input.moment, "N*m", "Moment");
            }
            if (src_cfg.has_application_point) {
                bp.register_input_vec3(src_cfg.name + ".app_pt", &input.application_point, "m", "Application point");
            }
            input.acts_at_cg = src_cfg.acts_at_cg;

            sources_.push_back(std::move(input));
        }
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        bp.wire_input("cg", cfg.cg_source);

        for (size_t i = 0; i < cfg.force_sources.size(); ++i) {
            const auto& src_cfg = cfg.force_sources[i];
            bp.wire_input(src_cfg.name + ".force", src_cfg.force_signal);

            if (src_cfg.has_moment) {
                bp.wire_input(src_cfg.name + ".moment", src_cfg.moment_signal);
            }
            if (src_cfg.has_application_point) {
                bp.wire_input(src_cfg.name + ".app_pt", src_cfg.application_point_signal);
            }
        }
    }

    void Step(Scalar t, Scalar dt) override {
        Vec3<Scalar> F_total = Vec3<Scalar>::Zero();
        Vec3<Scalar> M_total = Vec3<Scalar>::Zero();
        Vec3<Scalar> cg = cg_.get();

        for (const auto& src : sources_) {
            Vec3<Scalar> F = src.force.get();
            F_total = F_total + F;

            // Add explicit moment if provided
            if (src.moment.is_connected()) {
                M_total = M_total + src.moment.get();
            }

            // Moment transfer: M = r × F
            if (!src.acts_at_cg && src.application_point.is_connected()) {
                Vec3<Scalar> r = src.application_point.get() - cg;
                M_total = M_total + janus::cross(r, F);
            }
        }

        total_force_ = F_total;
        total_moment_ = M_total;
    }
};
```

#### Exit Criteria (4.2)

- [ ] MassAggregator sums mass properties correctly (parallel axis theorem)
- [ ] ForceAggregator sums forces and computes moments about CG
- [ ] Config-driven wiring works
- [ ] Symbolic mode traces correctly

---

### 4.3 RigidBody6DOF Component

**File:** `components/dynamics/RigidBody6DOF.hpp`

**Responsibility:** Full 6DOF translational and rotational dynamics.

**State Vector (13 states):**
```
[px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
 └─position─┘ └─velocity─┘ └──quaternion──┘ └─ang vel─┘
```

**Inputs (wired from aggregators):**
```yaml
RigidBody6DOF:
  wiring:
    total_force: Forces.total_force
    total_moment: Forces.total_moment
    total_mass: Mass.total_mass
    inertia: Mass.inertia
```

**Implementation:**
```cpp
template <typename Scalar>
class RigidBody6DOF : public Component<Scalar> {
    static constexpr std::size_t kStateSize = 13;

    // Inputs
    InputHandle<Vec3<Scalar>> total_force_;
    InputHandle<Vec3<Scalar>> total_moment_;
    InputHandle<Scalar> total_mass_;
    InputHandle<Mat3<Scalar>> inertia_;

    // State
    Vec3<Scalar> position_;
    Vec3<Scalar> velocity_;
    janus::Quaternion<Scalar> quaternion_;
    Vec3<Scalar> angular_velocity_;

    // Outputs
    vulcan::coordinates::CoordinateFrame<Scalar> body_frame_;
    vulcan::coordinates::CoordinateFrame<Scalar> ned_frame_;

    void Step(Scalar t, Scalar dt) override {
        vulcan::mass::MassProperties<Scalar> mass_props;
        mass_props.mass = total_mass_.get();
        mass_props.cg = Vec3<Scalar>::Zero();  // Forces already about CG
        mass_props.inertia = inertia_.get();

        vulcan::dynamics::RigidBodyState<Scalar> state;
        state.position = position_;
        state.velocity = velocity_;
        state.quaternion = quaternion_;
        state.angular_velocity = angular_velocity_;

        auto derivs = vulcan::dynamics::compute_6dof_derivatives(
            state,
            total_force_.get(),
            total_moment_.get(),
            mass_props
        );

        // Write state derivatives
        position_dot_ = derivs.position_dot;
        velocity_dot_ = derivs.velocity_dot;
        quaternion_dot_ = derivs.quaternion_dot;
        angular_velocity_dot_ = derivs.angular_velocity_dot;

        // Update output frames
        ned_frame_ = vulcan::coordinates::local_ned_at(position_, earth_model_);
        body_frame_ = vulcan::coordinates::body_from_quaternion(ned_frame_, quaternion_);
    }
};
```

#### Exit Criteria (4.3)

- [ ] 13-state quaternion-based 6DOF dynamics
- [ ] Correct translational dynamics: F = ma
- [ ] Correct rotational dynamics: I·ω̇ = M - ω × (I·ω)
- [ ] Quaternion normalization maintained
- [ ] Publishes `body_frame` and `ned_frame` for other components
- [ ] Symbolic mode traces correctly

---

### 4.4 Force/Mass Source Examples

#### 4.4.1 PointMassGravity (Force Source)

Force computed in ECEF, **transformed to body frame before publishing**:

```cpp
template <typename Scalar>
class PointMassGravity : public Component<Scalar> {
    Vec3<Scalar> force_body_;

    // Inputs
    InputHandle<Vec3<Scalar>> position_;
    InputHandle<Scalar> mass_;
    InputHandle<vulcan::coordinates::CoordinateFrame<Scalar>> body_frame_;

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        bp.register_output_vec3("force", &force_body_, "N", "Gravity force in body frame");

        bp.register_input_vec3("position", &position_, "m", "Position ECEF");
        bp.register_input("mass", &mass_, "kg", "Vehicle mass");
        bp.register_input("body_frame", &body_frame_, "", "Body frame");
    }

    void Step(Scalar t, Scalar dt) override {
        Vec3<Scalar> pos = position_.get();
        Scalar m = mass_.get();

        // Compute gravity in ECEF
        Vec3<Scalar> accel_ecef = vulcan::gravity::point_mass(pos, mu_);
        Vec3<Scalar> force_ecef = accel_ecef * m;

        // Transform to body frame
        auto ecef = vulcan::coordinates::CoordinateFrame<Scalar>::ecef();
        force_body_ = vulcan::coordinates::transform_vector(
            force_ecef, ecef, body_frame_.get());
    }
};
```

#### 4.4.2 FuelTank (Mass Source)

Publishes `MassProperties<Scalar>` directly:

```cpp
template <typename Scalar>
class FuelTank : public Component<Scalar> {
    vulcan::mass::MassProperties<Scalar> mass_props_;

    // Config
    Vec3<Scalar> tank_cg_body_;      // Tank CG in body frame
    Mat3<Scalar> tank_inertia_;      // Inertia when full
    Scalar initial_fuel_mass_;

    // State
    Scalar fuel_mass_;

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        bp.register_output("mass_properties", &mass_props_, "", "Tank mass properties");
        bp.register_output("fuel_mass", &fuel_mass_, "kg", "Current fuel mass");
    }

    void Step(Scalar t, Scalar dt) override {
        // Update mass properties based on current fuel
        Scalar fraction = fuel_mass_ / initial_fuel_mass_;

        mass_props_.mass = fuel_mass_;
        mass_props_.cg = tank_cg_body_;  // Assume CG doesn't move (simplification)
        mass_props_.inertia = tank_inertia_ * fraction;  // Scale inertia
    }
};
```

#### 4.4.3 Thrust (Force Source with Application Point)

```cpp
template <typename Scalar>
class Thrust : public Component<Scalar> {
    Vec3<Scalar> force_body_;
    Vec3<Scalar> moment_body_;
    Vec3<Scalar> nozzle_position_;  // In body frame

    // Config
    Vec3<Scalar> thrust_direction_body_;  // Unit vector in body frame

    // Inputs
    InputHandle<Scalar> throttle_;

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        bp.register_output_vec3("force", &force_body_, "N", "Thrust force in body frame");
        bp.register_output_vec3("moment", &moment_body_, "N*m", "Thrust moment in body frame");
        bp.register_output_vec3("application_point", &nozzle_position_, "m",
                                "Nozzle position in body frame");

        bp.register_input("throttle", &throttle_, "", "Throttle command 0-1");
    }

    void Step(Scalar t, Scalar dt) override {
        Scalar thrust_mag = max_thrust_ * throttle_.get();

        // Force already in body frame (thrust direction is body-fixed)
        force_body_ = thrust_direction_body_ * thrust_mag;

        // Moment from thrust misalignment (if any)
        moment_body_ = Vec3<Scalar>::Zero();  // Or compute from gimbal angle
    }
};
```

---

### 4.5 Example Wiring Configuration

Complete example showing how everything connects:

```yaml
# Rocket simulation configuration
entity: Rocket

components:
  # Mass sources
  - type: Structure
    name: Structure
    config:
      mass: 500.0
      cg: [0, 0, 2.0]
      inertia_diagonal: [100, 100, 50]

  - type: FuelTank
    name: FuelTank
    config:
      initial_fuel: 1000.0
      cg: [0, 0, 1.0]

  # Force sources
  - type: PointMassGravity
    name: Gravity
    wiring:
      position: EOM.position
      mass: Mass.total_mass
      body_frame: EOM.body_frame

  - type: Thrust
    name: Thrust
    config:
      max_thrust: 50000.0
      nozzle_position: [0, 0, 0]
      thrust_direction: [0, 0, -1]  # Points "down" in body frame
    wiring:
      throttle: Controller.throttle

  # Aggregators
  - type: MassAggregator
    name: Mass
    sources:
      - Structure.mass_properties
      - FuelTank.mass_properties

  - type: ForceAggregator
    name: Forces
    cg_source: Mass.cg
    sources:
      - name: gravity
        force: Gravity.force
        acts_at_cg: true
      - name: thrust
        force: Thrust.force
        application_point: Thrust.application_point

  # Dynamics
  - type: RigidBody6DOF
    name: EOM
    wiring:
      total_force: Forces.total_force
      total_moment: Forces.total_moment
      total_mass: Mass.total_mass
      inertia: Mass.inertia
```

---

### 4.6 Integration Tests

#### 4.6.1 Tumbling Rigid Body

**File:** `tests/integration/test_tumbling_rigid_body.cpp`

**Scenario:** Rigid body with known inertia, initial angular velocity, no external forces.

**Validation:**
- Angular momentum conservation: L = I·ω constant
- Energy conservation: E = ½ω·I·ω constant
- Quaternion normalization: |q| = 1

#### 4.6.2 Multi-Source Aggregation

**File:** `tests/integration/test_aggregation.cpp`

**Scenario:** Multiple force and mass sources.

**Validation:**
- Total mass equals sum of source masses
- CG computed correctly (mass-weighted average)
- Inertia uses parallel axis theorem
- Total force equals sum of body-frame forces
- Moments computed correctly about CG

---

## Execution Order

Topological sort produces:

```
1. Environment (Atmosphere models)
2. Force Sources (Gravity, Drag, Thrust - compute forces in body frame)
3. Mass Sources (FuelTank - compute current mass properties)
4. MassAggregator (sums mass properties)
5. ForceAggregator (sums forces, computes moments using CG)
6. RigidBody6DOF (integrates state, publishes body_frame)
```

Dependencies are explicit via wiring config.

---

## File Structure

```
include/icarus/
├── core/
│   └── Types.hpp                    # No changes needed
├── signal/
│   └── Backplane.hpp               # Add MassProperties/CoordinateFrame support

components/
├── aggregators/
│   ├── ForceAggregator.hpp         # NEW
│   └── MassAggregator.hpp          # NEW
├── dynamics/
│   ├── PointMass3DOF.hpp           # Existing
│   └── RigidBody6DOF.hpp           # NEW
├── environment/
│   └── PointMassGravity.hpp        # Update: transform to body frame
├── mass/
│   ├── Structure.hpp               # NEW: static mass source
│   └── FuelTank.hpp                # NEW: dynamic mass source
└── propulsion/
    └── Thrust.hpp                   # NEW

tests/
├── unit/
│   ├── test_mass_aggregator.cpp    # NEW
│   └── test_force_aggregator.cpp   # NEW
└── integration/
    ├── test_tumbling_rigid_body.cpp # NEW
    └── test_aggregation.cpp         # NEW
```

---

## Exit Criteria (Phase 4 Complete)

- [ ] **4.1** Signal conventions for mass/force sources
- [ ] **4.2** MassAggregator and ForceAggregator with config-driven wiring
- [ ] **4.3** RigidBody6DOF with quaternion-based dynamics
- [ ] **4.4** Example force/mass source components
- [ ] **4.5** Example wiring configuration
- [ ] **4.6** Integration tests pass
- [ ] All tests pass in both numeric and symbolic mode
- [ ] Symbolic graph extraction works for 6DOF system

---

## Dependencies

| Dependency | Purpose |
|:-----------|:--------|
| `vulcan::mass::MassProperties<Scalar>` | Mass/CG/inertia representation |
| `vulcan::coordinates::CoordinateFrame<Scalar>` | Frame transformations |
| `vulcan::coordinates::transform_vector()` | Vector frame transforms |
| `vulcan::dynamics::compute_6dof_derivatives()` | 6DOF EOM |
| `janus::Quaternion<Scalar>` | Quaternion math |
| `janus::cross()` | Symbolic-safe cross product |

---

## Gold Standard Component Patterns

### Pattern 1: Lifecycle Structure

```cpp
template <typename Scalar>
class ExampleComponent : public Component<Scalar> {
public:
    explicit ExampleComponent(const std::string& name, const std::string& entity = "")
        : Component<Scalar>(name, entity) {}

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // 1. Register outputs
        bp.register_output_vec3("force", &force_, "N", "Force in body frame");

        // 2. Register inputs
        bp.register_input_vec3("position", &position_, "m", "Position");

        // 3. Register parameters (optimizable)
        bp.register_param("Cd", &Cd_, 0.5, "", "Drag coefficient");

        // 4. Register config (discrete)
        bp.register_config("model", &model_int_, 0, "Model selection");
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Wire inputs from config
        for (const auto& [input, source] : cfg.wiring) {
            bp.wire_input(input, source);
        }
    }

    void Step(Scalar t, Scalar dt) override {
        // Read inputs, compute, write outputs
        Vec3<Scalar> pos = position_.get();
        // ... physics ...
        force_ = computed_force;
    }
};
```

### Pattern 2: Force Sources Transform to Body Frame

```cpp
void Step(Scalar t, Scalar dt) override {
    // Compute force in natural frame
    Vec3<Scalar> force_ecef = compute_in_ecef();

    // Transform to body frame BEFORE publishing
    force_body_ = vulcan::coordinates::transform_vector(
        force_ecef, ecef_frame_, body_frame_.get());
}
```

### Pattern 3: Mass Sources Use MassProperties

```cpp
vulcan::mass::MassProperties<Scalar> mass_props_;

void Step(Scalar t, Scalar dt) override {
    mass_props_.mass = current_mass_;
    mass_props_.cg = cg_body_;
    mass_props_.inertia = inertia_about_cg_;
}
```

### Pattern 4: Aggregators Wire Explicitly

```cpp
void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    // All sources listed in config
    for (const auto& source_path : cfg.sources) {
        bp.wire_input(source_path, source_path);
    }
}
```

### Pattern 5: Symbolic Compatibility

- [ ] Template on `Scalar`
- [ ] Use `janus::` math functions
- [ ] Use `janus::where()` for branching
- [ ] No `std::` math in hot path
- [ ] No string lookups in `Step()`

---

## Architecture References

| Topic | Document |
|:------|:---------|
| Aggregation pattern | [12_quantity_aggregation.md](../../architecture/12_quantity_aggregation.md) |
| Vulcan utilities | [08_vulcan_integration.md](../../architecture/08_vulcan_integration.md) |
| Component protocol | [02_component_protocol.md](../../architecture/02_component_protocol.md) |
| Symbolic constraints | [21_symbolic_constraints.md](../../architecture/21_symbolic_constraints.md) |
