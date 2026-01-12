# Rocket Example Implementation Plan

**Status:** Proposed
**Purpose:** Create working rocket simulation YAML for Hermes integration
**Related:** Hermes sim orchestrator, Vulcan propulsion utilities

---

## Overview

Hermes requires a working rocket YAML example that exposes the following signals via the shared memory backplane:

```
Rocket.EOM.position.*    - Vehicle position (ECI/ECEF)
Rocket.EOM.velocity.*    - Vehicle velocity
Rocket.Engine.thrust     - Current thrust magnitude
Rocket.Mass.total        - Total vehicle mass
```

Additionally, Hermes needs injection points:
```
thrust_cmd               - Throttle command (0-1)
```

---

## Current State Analysis

### Available Vulcan Physics

| File | Functions | Status |
|------|-----------|--------|
| `vulcan/propulsion/Rocket.hpp` | `thrust_from_mdot`, `exhaust_velocity`, `mass_flow_rate`, `delta_v`, `propellant_mass`, `burn_time` | Ready |
| `vulcan/propulsion/AltitudeThrust.hpp` | `altitude_thrust`, `thrust_coefficient` | Ready |
| `vulcan/atmosphere/USSA1976.hpp` | Atmospheric density/pressure vs altitude | Ready |

### Existing Icarus Components

| Component | Signals | Status |
|-----------|---------|--------|
| `ForceAggregator` | `total_force.*`, `total_moment.*` | Ready |
| `MassAggregator` | `total_mass`, `cg.*`, `inertia.*` | Ready |
| `RigidBody6DOF` | `position.*`, `velocity.*`, `attitude.*`, `omega.*` | Ready |
| `PointMassGravity` | `force.*` | Ready |
| `StaticMass` | `mass`, `cg.*`, `inertia.*` | Ready |

### Missing Components

| Component | Purpose | Priority |
|-----------|---------|----------|
| `RocketEngine` | Thrust computation with throttle command | **P0 - Required** |
| `FuelTank` | Propellant mass with depletion | **P0 - Required** |
| `AtmosphereModel` | Ambient pressure for altitude compensation | P2 - Nice to have |

---

## Component Specifications

### 1. RocketEngine Component

**Location:** `components/propulsion/RocketEngine.hpp`

**Description:** Computes thrust force from throttle command and engine parameters. Uses Vulcan `vulcan::propulsion::rocket` utilities for physics.

#### Parameters (YAML config)

```yaml
scalars:
  max_thrust: 50000.0      # Maximum vacuum thrust [N]
  isp_vacuum: 311.0        # Vacuum specific impulse [s]
  isp_sea_level: 282.0     # Sea-level Isp [s] (optional, for altitude comp)
  nozzle_exit_area: 0.5    # Exit area [m²] (optional, for altitude comp)
vectors:
  thrust_direction: [0, 0, -1]    # Unit vector in body frame
  nozzle_position: [0, 0, 0]      # Application point in body frame
```

#### Inputs

| Signal | Type | Unit | Description |
|--------|------|------|-------------|
| `throttle_cmd` | Scalar | - | Throttle command [0, 1] |
| `ambient_pressure` | Scalar | Pa | Atmospheric pressure (optional) |

#### Outputs

| Signal | Type | Unit | Description |
|--------|------|------|-------------|
| `thrust` | Scalar | N | Thrust magnitude |
| `mass_flow_rate` | Scalar | kg/s | Propellant consumption rate |
| `force.x/y/z` | Vec3 | N | Thrust force vector (body frame) |
| `application_point.x/y/z` | Vec3 | m | Nozzle position (for ForceAggregator) |

#### Physics Implementation

```cpp
// Core calculation in Step():
Scalar throttle = janus::clamp(throttle_cmd_.get(), Scalar(0), Scalar(1));

// Exhaust velocity from Isp
Scalar Ve = vulcan::propulsion::rocket::exhaust_velocity(isp_vacuum_);

// Commanded thrust
Scalar F_vac = throttle * max_thrust_;

// Optional: altitude compensation
// F = F_vac - P_atm * A_exit
Scalar F = janus::where(
    has_altitude_compensation_,
    vulcan::propulsion::altitude_thrust(F_vac, ambient_pressure_.get(), 0.0, nozzle_exit_area_),
    F_vac
);

// Mass flow rate
Scalar mdot = vulcan::propulsion::rocket::mass_flow_rate(F, Ve);

// Force vector in body frame
Vec3<Scalar> force = thrust_direction_ * F;
```

#### ForceAggregator Compatibility

Outputs `force.*` and `application_point.*` signals matching ForceAggregator's expected source interface.

---

### 2. FuelTank Component

**Location:** `components/propulsion/FuelTank.hpp`

**Description:** Tracks propellant mass with time-varying depletion. Provides mass properties to MassAggregator.

#### Parameters (YAML config)

```yaml
scalars:
  initial_fuel_mass: 1000.0   # Initial propellant [kg]
  dry_mass: 50.0              # Empty tank mass [kg]
  tank_radius: 0.5            # For inertia computation [m]
  tank_length: 2.0            # For inertia computation [m]
vectors:
  tank_cg: [0, 0, 1.0]        # CG position (body frame) [m]
```

#### Inputs

| Signal | Type | Unit | Description |
|--------|------|------|-------------|
| `mass_flow_rate` | Scalar | kg/s | Consumption rate from engine |

#### Outputs (MassAggregator compatible)

| Signal | Type | Unit | Description |
|--------|------|------|-------------|
| `mass` | Scalar | kg | Current total mass (dry + fuel) |
| `fuel_mass` | Scalar | kg | Current fuel mass |
| `cg.x/y/z` | Vec3 | m | CG position |
| `inertia.xx/yy/zz` | Scalar | kg·m² | Diagonal inertia |
| `inertia.xy/xz/yz` | Scalar | kg·m² | Off-diagonal inertia |

#### State Variables

| State | Unit | Description |
|-------|------|-------------|
| `fuel_mass` | kg | Integrable fuel mass |

#### Physics Implementation

```cpp
// State derivative (in Step):
// d(fuel_mass)/dt = -mass_flow_rate (fuel depletes)
fuel_mass_dot_ = -mass_flow_rate_.get();

// Clamp to non-negative
fuel_mass_dot_ = janus::where(fuel_mass_ <= Scalar(0), Scalar(0), fuel_mass_dot_);

// Total mass
mass_ = dry_mass_ + fuel_mass_;

// Simplified inertia (cylindrical approximation)
Scalar I_radial = (mass_ / 12.0) * (3 * r² + L²);
Scalar I_axial = (mass_ / 2.0) * r²;
```

---

### 3. Rocket Example YAML Structure

**Location:** `config/examples/rocket_ascent.yaml`

#### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Rocket Entity                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐     ┌──────────────┐                          │
│  │  Structure   │     │   FuelTank   │←── mass_flow_rate        │
│  │  (static)    │     │  (depleting) │                          │
│  └──────┬───────┘     └──────┬───────┘                          │
│         │                    │                                   │
│         └────────┬───────────┘                                   │
│                  ▼                                               │
│         ┌────────────────┐                                       │
│         │ MassAggregator │───► total_mass, cg, inertia          │
│         └────────────────┘                                       │
│                  │                                               │
│                  ▼                                               │
│  ┌──────────────┐     ┌──────────────┐                          │
│  │   Gravity    │     │ RocketEngine │←── throttle_cmd          │
│  │ (point mass) │     │   (thrust)   │───► mass_flow_rate       │
│  └──────┬───────┘     └──────┬───────┘                          │
│         │                    │                                   │
│         └────────┬───────────┘                                   │
│                  ▼                                               │
│         ┌────────────────┐                                       │
│         │ForceAggregator │───► total_force, total_moment        │
│         └────────────────┘                                       │
│                  │                                               │
│                  ▼                                               │
│         ┌────────────────┐                                       │
│         │  RigidBody6DOF │───► position, velocity, attitude     │
│         └────────────────┘                                       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

#### Signal Flow

```
throttle_cmd (input) ──────────────────────────────────┐
                                                       ▼
                                              ┌────────────────┐
                                              │  RocketEngine  │
                                              └───────┬────────┘
                                                      │
                    ┌─────────────────────────────────┼──────────────────────┐
                    │ mass_flow_rate                  │ force.*              │
                    ▼                                 ▼                      │
           ┌────────────────┐                ┌────────────────┐              │
           │    FuelTank    │                │ForceAggregator │◄─────────────┤
           └───────┬────────┘                └───────┬────────┘              │
                   │ mass, cg, inertia               │ total_force           │
                   ▼                                 │ total_moment          │
           ┌────────────────┐                        │                       │
           │ MassAggregator │────────────────────────┼───────────────────────┘
           └───────┬────────┘                        │
                   │ total_mass                      │
                   └────────────────┬────────────────┘
                                    ▼
                           ┌────────────────┐
                           │  RigidBody6DOF │
                           └───────┬────────┘
                                   │
                                   ▼
                           position, velocity, attitude
                           (to Hermes backplane)
```

---

## Example YAML Configuration

```yaml
# config/examples/rocket_ascent.yaml
simulation:
  name: "Rocket Ascent Example"
  description: "Single-stage rocket ascent for Hermes integration"
  dt: 0.01
  t_end: 300.0

integrator:
  type: RK4

# =============================================================================
# Components
# =============================================================================
components:
  # ---------------------------------------------------------------------------
  # Mass Sources
  # ---------------------------------------------------------------------------
  - type: StaticMass
    name: Structure
    entity: Rocket
    scalars:
      mass: 500.0
      inertia_xx: 100.0
      inertia_yy: 100.0
      inertia_zz: 50.0
    vectors:
      cg: [0, 0, 2.0]

  - type: FuelTank
    name: FuelTank
    entity: Rocket
    scalars:
      initial_fuel_mass: 1000.0
      dry_mass: 50.0
    vectors:
      tank_cg: [0, 0, 1.0]

  # ---------------------------------------------------------------------------
  # Mass Aggregator
  # ---------------------------------------------------------------------------
  - type: MassAggregator
    name: Mass
    entity: Rocket
    sources: [Structure, FuelTank]

  # ---------------------------------------------------------------------------
  # Force Sources
  # ---------------------------------------------------------------------------
  - type: PointMassGravity
    name: Gravity
    entity: Rocket
    scalars:
      mu: 3.986004418e14

  - type: RocketEngine
    name: Engine
    entity: Rocket
    scalars:
      max_thrust: 50000.0
      isp_vacuum: 311.0
    vectors:
      thrust_direction: [0, 0, 1]     # +Z body = up
      nozzle_position: [0, 0, 0]

  # ---------------------------------------------------------------------------
  # Force Aggregator
  # ---------------------------------------------------------------------------
  - type: ForceAggregator
    name: Forces
    entity: Rocket
    sources: [Gravity, Engine]

  # ---------------------------------------------------------------------------
  # 6DOF Dynamics
  # ---------------------------------------------------------------------------
  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    vectors:
      initial_position: [0, 0, 6.471e6]       # Surface + ~100km
      initial_velocity: [0, 0, 0]             # Vertical launch
      initial_attitude: [1, 0, 0, 0]          # Identity quaternion
      initial_angular_velocity: [0, 0, 0]

# =============================================================================
# Signal Routing
# =============================================================================
routes:
  # Engine inputs
  - input: Rocket.Engine.throttle_cmd
    output: __external__.thrust_cmd           # Hermes injection point

  # Engine → FuelTank
  - input: Rocket.FuelTank.mass_flow_rate
    output: Rocket.Engine.mass_flow_rate

  # MassAggregator → Gravity
  - input: Rocket.Gravity.position.x
    output: Rocket.EOM.position.x
  - input: Rocket.Gravity.position.y
    output: Rocket.EOM.position.y
  - input: Rocket.Gravity.position.z
    output: Rocket.EOM.position.z
  - input: Rocket.Gravity.mass
    output: Rocket.Mass.total_mass

  # MassAggregator → ForceAggregator (CG for moment transfer)
  - input: Rocket.Forces.cg.x
    output: Rocket.Mass.cg.x
  - input: Rocket.Forces.cg.y
    output: Rocket.Mass.cg.y
  - input: Rocket.Forces.cg.z
    output: Rocket.Mass.cg.z

  # ForceAggregator → EOM
  - input: Rocket.EOM.total_force.x
    output: Rocket.Forces.total_force.x
  - input: Rocket.EOM.total_force.y
    output: Rocket.Forces.total_force.y
  - input: Rocket.EOM.total_force.z
    output: Rocket.Forces.total_force.z
  - input: Rocket.EOM.total_moment.x
    output: Rocket.Forces.total_moment.x
  - input: Rocket.EOM.total_moment.y
    output: Rocket.Forces.total_moment.y
  - input: Rocket.EOM.total_moment.z
    output: Rocket.Forces.total_moment.z

  # MassAggregator → EOM
  - input: Rocket.EOM.total_mass
    output: Rocket.Mass.total_mass
  - input: Rocket.EOM.inertia.xx
    output: Rocket.Mass.inertia.xx
  - input: Rocket.EOM.inertia.yy
    output: Rocket.Mass.inertia.yy
  - input: Rocket.EOM.inertia.zz
    output: Rocket.Mass.inertia.zz
  - input: Rocket.EOM.inertia.xy
    output: Rocket.Mass.inertia.xy
  - input: Rocket.EOM.inertia.xz
    output: Rocket.Mass.inertia.xz
  - input: Rocket.EOM.inertia.yz
    output: Rocket.Mass.inertia.yz

# =============================================================================
# Hermes Export Signals
# =============================================================================
exports:
  - Rocket.EOM.position.x
  - Rocket.EOM.position.y
  - Rocket.EOM.position.z
  - Rocket.EOM.velocity.x
  - Rocket.EOM.velocity.y
  - Rocket.EOM.velocity.z
  - Rocket.Engine.thrust
  - Rocket.Mass.total_mass

# =============================================================================
# Scheduler
# =============================================================================
scheduler:
  topology:
    mode: automatic
  groups:
    - name: mass
      rate_hz: 100.0
      priority: 1
      members: [Structure, FuelTank, Mass]
    - name: forces
      rate_hz: 100.0
      priority: 2
      members: [Gravity, Engine, Forces]
    - name: dynamics
      rate_hz: 100.0
      priority: 3
      members: [EOM]
```

---

## Implementation Tasks

### Phase 1: Core Components (P0)

#### Task 1.1: RocketEngine Component
- [ ] Create `components/propulsion/RocketEngine.hpp`
- [ ] Implement throttle-to-thrust calculation using Vulcan utilities
- [ ] Output `force.*` signals compatible with ForceAggregator
- [ ] Output `thrust` scalar for telemetry
- [ ] Output `mass_flow_rate` for fuel depletion
- [ ] Register component in `components/registration.cpp`
- [ ] Unit tests: `tests/components/test_rocket_engine.cpp`

**Key implementation details:**
```cpp
#include <vulcan/propulsion/Rocket.hpp>

// In Step():
Scalar Ve = vulcan::propulsion::rocket::exhaust_velocity(isp_vacuum_);
Scalar F = throttle * max_thrust_;
Scalar mdot = vulcan::propulsion::rocket::mass_flow_rate(F, Ve);
force_ = thrust_direction_ * F;
thrust_ = F;
mass_flow_rate_ = mdot;
```

#### Task 1.2: FuelTank Component (State-Based)
- [ ] Create `components/propulsion/FuelTank.hpp`
- [ ] Implement fuel mass as integrable state
- [ ] Track `d(fuel_mass)/dt = -mass_flow_rate`
- [ ] Output mass properties for MassAggregator
- [ ] Handle fuel exhaustion (clamp at zero)
- [ ] Register component
- [ ] Unit tests: `tests/components/test_fuel_tank.cpp`

**Key implementation details:**
```cpp
// State registration in Provision():
bp.template register_state<Scalar>("fuel_mass", &fuel_mass_, &fuel_mass_dot_, "kg");

// In Step():
fuel_mass_dot_ = -mass_flow_rate_.get();
fuel_mass_dot_ = janus::where(fuel_mass_ <= Scalar(0), Scalar(0), fuel_mass_dot_);
mass_ = dry_mass_ + fuel_mass_;
```

### Phase 2: YAML Example & Integration

#### Task 2.1: Example YAML
- [ ] Create `config/examples/rocket_ascent.yaml`
- [ ] Configure all components with realistic parameters
- [ ] Set up signal routing
- [ ] Define scheduler groups
- [ ] Add export signals for Hermes

#### Task 2.2: Integration Testing
- [ ] Run simulation end-to-end
- [ ] Verify mass depletion over burn
- [ ] Verify thrust produces expected acceleration
- [ ] Verify position/velocity integration
- [ ] Compare against analytical predictions (Tsiolkovsky)

### Phase 3: Hermes Integration (Future)

#### Task 3.1: External Signal Interface
- [ ] Define `__external__` signal namespace for injection
- [ ] Map Hermes shared memory to Icarus signals
- [ ] Test bidirectional signal flow

#### Task 3.2: Altitude Compensation (Optional)
- [ ] Add `AtmosphereModel` component using `vulcan::atmosphere::USSA1976`
- [ ] Wire ambient pressure to RocketEngine
- [ ] Implement Isp interpolation (sea level → vacuum)

---

## Verification Criteria

### Unit Tests

```bash
# Component tests
ctest --test-dir build -R "RocketEngine|FuelTank" -VV
```

| Test | Description |
|------|-------------|
| `RocketEngine_ZeroThrottle` | Zero throttle → zero thrust, zero mdot |
| `RocketEngine_FullThrottle` | Full throttle → max thrust |
| `RocketEngine_MdotConsistency` | mdot = F / Ve |
| `FuelTank_MassDepletion` | Fuel decreases with mdot input |
| `FuelTank_FuelExhaustion` | Clamps at zero, no negative mass |
| `FuelTank_MassProperties` | Outputs correct total mass |

### Integration Tests

```bash
# Full simulation
./build/icarus config/examples/rocket_ascent.yaml
```

| Criterion | Expected |
|-----------|----------|
| Mass depletion | 1550 kg → ~550 kg (after full burn) |
| Delta-V | ~3 km/s (Tsiolkovsky: Ve * ln(m0/mf)) |
| Thrust-to-weight | ~3.2 initially, ~9.0 at burnout |
| Burn time | ~61s (mp / mdot = 1000 / 16.4) |

### Analytical Validation

Using Tsiolkovsky rocket equation:
```
Ve = Isp * g0 = 311 * 9.80665 = 3050 m/s
m0 = 1550 kg (500 structure + 50 tank + 1000 fuel)
mf = 550 kg (500 structure + 50 tank)
ΔV = Ve * ln(m0/mf) = 3050 * ln(2.82) = 3163 m/s
mdot = F / Ve = 50000 / 3050 = 16.4 kg/s
t_burn = 1000 / 16.4 = 61 s
```

---

## Open Questions

1. **Frame conventions**: Should thrust direction be +Z (body up) or -Z (towards ground)?
   - Current proposal: +Z body = thrust up (vehicle nose up during ascent)

2. **Inertia model**: Use simplified cylinder or full tank geometry?
   - Current proposal: Start with simplified, add fidelity later

3. **Altitude compensation**: Required for Phase 1 or defer to Phase 3?
   - Current proposal: Defer (vacuum Isp sufficient for initial validation)

4. **FuelTank inertia variation**: Track CG shift as fuel depletes?
   - Current proposal: Defer (constant CG initially)

---

## File Locations Summary

| File | Purpose |
|------|---------|
| `components/propulsion/RocketEngine.hpp` | Thrust component |
| `components/propulsion/FuelTank.hpp` | Propellant tracking |
| `config/examples/rocket_ascent.yaml` | Example configuration |
| `tests/components/test_rocket_engine.cpp` | Engine tests |
| `tests/components/test_fuel_tank.cpp` | Tank tests |
