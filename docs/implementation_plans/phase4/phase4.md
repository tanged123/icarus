# Phase 4: Aggregation & 6DOF Implementation Plan

**Status:** Blocked on Phase 4.0
**Goal:** Multi-component force/moment and mass property aggregation with full 6DOF dynamics

---

## Prerequisites

> [!CAUTION]
> **Complete Phase 4.0 first.** The configuration infrastructure must be in place before implementing Phase 4 components.

| Prerequisite | Document | Status |
|:-------------|:---------|:-------|
| Config Infrastructure | [phase4_0_config_infrastructure.md](phase4_0_config_infrastructure.md) | Not Started |

Phase 4.0 establishes:
- `ComponentConfig` with typed accessors (`cfg.Require<T>()`, `cfg.Get<T>()`)
- YAML configuration loader
- `ComponentFactory` for type registration
- Refactored existing components (no public setters)

**All Phase 4 components must follow the config-driven pattern.**

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

## Implementation Checklist

### 4.1 Signal Backplane Extensions

- [ ] **4.1.1** Add `MassProperties<Scalar>` support to `Backplane`
  - [ ] `declare_output<vulcan::mass::MassProperties<Scalar>>`
  - [ ] `declare_input<vulcan::mass::MassProperties<Scalar>>`
  - [ ] Update `InputHandle` to work with composite types
- [ ] **4.1.2** Add `Mat3<Scalar>` inertia tensor support
  - [ ] Already supported for 3x3 matrices, verify works for inertia
- [ ] **4.1.3** Unit tests for new signal types
  - [ ] Test MassProperties signal registration/resolution
  - [ ] Test Mat3 inertia signal wiring

---

### 4.2 Mass Source Components

#### 4.2.1 Structure Component (Static Mass)

**File:** `components/mass/Structure.hpp`

- [ ] Create `components/mass/` directory
- [ ] Implement `Structure<Scalar>` component
  - [ ] Constructor with name/entity
  - [ ] `Provision()`: register `mass_properties` output
  - [ ] `Stage()`: apply configuration (mass, CG, inertia from config)
  - [ ] `Step()`: outputs remain constant (static mass)
- [ ] Configuration support:
  - [ ] `mass`: total mass [kg]
  - [ ] `cg`: CG position in body frame [m]
  - [ ] `inertia_diagonal`: [Ixx, Iyy, Izz] [kg·m²]
- [ ] Unit tests (numeric + symbolic)

```cpp
// Usage example
Structure<double> structure("Structure", "Rocket");
structure.SetMass(500.0);
structure.SetCG({0, 0, 2.0});
structure.SetInertiaDiagonal(100, 100, 50);
```

#### 4.2.2 FuelTank Component (Dynamic Mass)

**File:** `components/mass/FuelTank.hpp`

- [ ] Implement `FuelTank<Scalar>` component
  - [ ] Has 1 state variable: `fuel_mass_`
  - [ ] Input: `fuel_flow_rate` [kg/s]
  - [ ] Output: `mass_properties`, `fuel_mass`
  - [ ] `BindState()` for integrator connection
  - [ ] `Step()`: update mass properties based on current fuel
- [ ] Configuration:
  - [ ] `initial_fuel`: starting fuel mass [kg]
  - [ ] `tank_cg`: tank center in body frame [m]
  - [ ] `tank_inertia`: inertia tensor when full [kg·m²]
- [ ] State derivative: `fuel_mass_dot = -fuel_flow_rate_`
- [ ] Unit tests (numeric + symbolic)

---

### 4.3 MassAggregator Component

**File:** `components/aggregators/MassAggregator.hpp`

- [ ] Create `components/aggregators/` directory
- [ ] Implement `MassAggregator<Scalar>` component
  - [ ] Config-driven source wiring (list of source paths)
  - [ ] `Provision()`: register outputs + create input handles for each source
  - [ ] `Stage()`: wire input handles to source paths
  - [ ] `Step()`: aggregate using `vulcan::mass::MassProperties<Scalar>::operator+`
- [ ] Outputs:
  - [ ] `total_mass` (Scalar)
  - [ ] `cg` (Vec3)
  - [ ] `inertia` (Mat3)
  - [ ] `mass_properties` (MassProperties)
- [ ] Unit tests:
  - [ ] Two point masses at different locations → verify CG
  - [ ] Verify parallel axis theorem application
  - [ ] Symbolic mode tracing

**Config example:**

```yaml
MassAggregator:
  name: Mass
  entity: Rocket
  sources:
    - Structure.mass_properties
    - FuelTank.mass_properties
```

---

### 4.4 Force Source Updates

#### 4.4.1 PointMassGravity Update (Existing)

**File:** `components/environment/PointMassGravity.hpp`

- [ ] Verify force output is in body frame (or add body frame transform)
- [ ] Optional: Add `body_frame` input for frame transformation
- [ ] Add `acts_at_cg: true` semantic in config (no application point needed)

#### 4.4.2 AtmosphericDrag Component (NEW)

**File:** `components/environment/AtmosphericDrag.hpp`

- [ ] Implement `AtmosphericDrag<Scalar>` component
  - [ ] Inputs: `velocity`, `altitude`, `body_frame`
  - [ ] Outputs: `force` (Vec3), `application_point` (Vec3)
  - [ ] Parameters: `Cd` (drag coefficient), `area` (reference area)
  - [ ] Uses `vulcan::atmosphere::us76()` for density
  - [ ] Uses `vulcan::aero::dynamic_pressure()` for q
- [ ] Force in body frame: `F = -drag * v_hat`
- [ ] Unit tests (numeric + symbolic)

#### 4.4.3 Thrust Component (NEW)

**File:** `components/propulsion/Thrust.hpp`

- [ ] Implement `Thrust<Scalar>` component
  - [ ] Input: `throttle` [0-1]
  - [ ] Outputs: `force`, `moment`, `application_point`
  - [ ] Config: `max_thrust`, `thrust_direction`, `nozzle_position`
- [ ] Force: `F = max_thrust * throttle * direction`
- [ ] Unit tests (numeric + symbolic)

---

### 4.5 ForceAggregator Component

**File:** `components/aggregators/ForceAggregator.hpp`

- [ ] Implement `ForceAggregator<Scalar>` component
  - [ ] Config-driven source wiring
  - [ ] Input: `cg` from MassAggregator
  - [ ] For each source: `force`, optional `moment`, optional `application_point`
  - [ ] `Step()`: sum forces, compute moments about CG
- [ ] Outputs:
  - [ ] `total_force` (Vec3)
  - [ ] `total_moment` (Vec3)
- [ ] Moment calculation:
  - [ ] If `acts_at_cg`: no moment arm
  - [ ] Else: `M = M_src + (app_point - cg) × F`
- [ ] Unit tests:
  - [ ] Two forces at CG → verify sum
  - [ ] Force with offset → verify moment arm
  - [ ] Symbolic mode tracing

**Config example:**

```yaml
ForceAggregator:
  name: Forces
  entity: Rocket
  cg_source: Mass.cg
  sources:
    - name: gravity
      force: Gravity.force
      acts_at_cg: true
    - name: thrust
      force: Thrust.force
      application_point: Thrust.application_point
```

---

### 4.6 RigidBody6DOF Component

**File:** `components/dynamics/RigidBody6DOF.hpp`

- [ ] Implement `RigidBody6DOF<Scalar>` component
  - [ ] 13 states: [px,py,pz, vx,vy,vz, qw,qx,qy,qz, ωx,ωy,ωz]
  - [ ] Uses `vulcan::dynamics::compute_6dof_derivatives()`
  - [ ] Uses `janus::Quaternion<Scalar>` for attitude
- [ ] Inputs:
  - [ ] `total_force` (Vec3 in body frame)
  - [ ] `total_moment` (Vec3 about CG, body frame)
  - [ ] `total_mass` (Scalar)
  - [ ] `inertia` (Mat3)
- [ ] Outputs:
  - [ ] `position` (Vec3)
  - [ ] `velocity` (Vec3, body or reference frame)
  - [ ] `attitude` (Vec4, quaternion wxyz)
  - [ ] `angular_velocity` (Vec3, body frame)
  - [ ] `body_dcm` (Mat3, body-to-reference rotation)
- [ ] `BindState()`: bind to global state vector
- [ ] `PreStep()`: publish current state to outputs
- [ ] `Step()`: compute derivatives
- [ ] Quaternion normalization (derivative adjustment)
- [ ] Unit tests:
  - [ ] Torque-free tumbling: `L = Iω` conserved
  - [ ] Constant force: linear acceleration
  - [ ] Quaternion norm maintained
  - [ ] Symbolic mode tracing

---

### 4.7 Integration Tests

- [ ] **4.7.1** `test_tumbling_rigid_body.cpp`
  - [ ] No external torque, initial angular velocity
  - [ ] Verify angular momentum conservation
  - [ ] Verify kinetic energy conservation
  - [ ] Verify quaternion normalization
- [ ] **4.7.2** `test_aggregation.cpp`
  - [ ] Multiple mass + force sources
  - [ ] Verify total mass = sum of sources
  - [ ] Verify CG is mass-weighted average
  - [ ] Verify moment calculation about CG
- [ ] **4.7.3** Symbolic graph test for 6DOF
  - [ ] Extract `CasADi::Function` from 6DOF system
  - [ ] Verify derivative shapes match state

---

### 4.8 CMake Updates

- [ ] **4.8.1** `components/CMakeLists.txt`
  - [ ] Add `aggregators/` subdirectory/sources
  - [ ] Add `mass/` subdirectory/sources
  - [ ] Add new propulsion sources
- [ ] **4.8.2** `tests/CMakeLists.txt`
  - [ ] Add `test_mass_aggregator`
  - [ ] Add `test_force_aggregator`
  - [ ] Add `test_rigid_body_6dof`
  - [ ] Add integration tests

---

### 4.9 Examples & Documentation

- [ ] **4.9.1** Create 6DOF rocket example
  - [ ] `examples/rocket_6dof/` directory
  - [ ] Configuration file with all components
  - [ ] Main driver code
- [ ] **4.9.2** Update this document with final implementation notes

---

## Architectural Decisions

### Pure Signal Approach (No Special Registration)

We use **standard signals with conventions**, not special registration APIs:

| Approach | Special Registration | Signal Conventions ✓ |
|:---------|:--------------------|:---------------------|
| APIs | `register_force_source()` | `declare_output()` (standard) |
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
│   └── Backplane.hpp               # Add MassProperties/Mat3 support

components/
├── aggregators/                    # NEW directory
│   ├── ForceAggregator.hpp         # NEW
│   └── MassAggregator.hpp          # NEW
├── dynamics/
│   ├── PointMass3DOF.hpp           # Existing
│   └── RigidBody6DOF.hpp           # NEW
├── environment/
│   ├── AtmosphericDrag.hpp         # NEW
│   └── PointMassGravity.hpp        # Update: verify body frame
├── mass/                           # NEW directory
│   ├── FuelTank.hpp                # NEW
│   └── Structure.hpp               # NEW
└── propulsion/
    └── Thrust.hpp                  # NEW

tests/
├── components/
│   ├── test_mass_aggregator.cpp    # NEW
│   ├── test_force_aggregator.cpp   # NEW
│   └── test_rigid_body_6dof.cpp    # NEW
└── integration/
    └── test_aggregation.cpp        # NEW
```

---

## Dependencies

| Dependency | Purpose | Status |
|:-----------|:--------|:-------|
| `vulcan::mass::MassProperties<Scalar>` | Mass/CG/inertia representation | ✓ Available |
| `vulcan::dynamics::compute_6dof_derivatives()` | 6DOF EOM | ✓ Available |
| `vulcan::dynamics::translational_dynamics()` | Body-frame F=ma | ✓ Available |
| `vulcan::dynamics::rotational_dynamics()` | Euler's equations | ✓ Available |
| `janus::Quaternion<Scalar>` | Quaternion math | ✓ Available |
| `janus::cross()` | Symbolic-safe cross product | ✓ Available |

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

## Gold Standard Component Patterns

### Pattern 1: Lifecycle Structure

See [phase4_0_config_infrastructure.md](phase4_0_config_infrastructure.md) for the authoritative component interface.

```cpp
template <typename Scalar>
class ExampleComponent : public Component<Scalar> {
public:
    explicit ExampleComponent(const std::string& name, const std::string& entity = "")
        : Component<Scalar>(name, entity) {}

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // 1. Request config values (from YAML, possibly overridden)
        Cd_ = cfg.Get<double>("Cd", 0.5);
        model_ = cfg.Get<int>("model", 0);

        // 2. Declare outputs (what this component produces)
        bp.declare_output<Vec3<Scalar>>("force", &force_, "N", "Force in body frame");

        // 3. Declare inputs (what this component consumes)
        bp.declare_input<Vec3<Scalar>>("position", &position_, "m", "Position");

        // NOTE: No wiring here - SignalRouter handles all connections
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Apply initial conditions, bind state, etc.
        // NOTE: No wiring here - SignalRouter has already connected signals
    }

    void Step(Scalar t, Scalar dt) override {
        // Read inputs, compute, write outputs
        Vec3<Scalar> pos = position_.get();
        // ... physics ...
        force_ = computed_force;
    }

private:
    double Cd_ = 0.5;
    int model_ = 0;
    InputHandle<Vec3<Scalar>> position_;
    Vec3<Scalar> force_;
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
