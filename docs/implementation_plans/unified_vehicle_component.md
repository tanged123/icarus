# Unified Vehicle Component: Design Analysis

## Problem Statement

The current architecture for a 6DOF rigid body (like the Rocket in `rocket_ascent.yaml`) requires three separate components working in concert:

1. **`RigidBody6DOF`** — Owns 13-state dynamics (position, velocity, attitude, angular velocity)
2. **`ForceAggregator`** — Sums forces/moments from multiple sources, handles frame transforms
3. **`MassAggregator`** — Aggregates mass properties (mass, CG, inertia) from multiple sources

This separation creates significant YAML wiring complexity:

```yaml
# Current: 26+ signal routes just to connect these three components
routes:
  # Mass → ForceAggregator (CG for moment transfer)
  - input: Rocket.Forces.cg.x
    output: Rocket.Mass.cg.x
  # ... (3 routes)
  
  # EOM → ForceAggregator (attitude for ECEF→body transform)
  - input: Rocket.Forces.attitude.w
    output: Rocket.EOM.attitude.w
  # ... (4 routes)
  
  # ForceAggregator → EOM (forces/moments)
  - input: Rocket.EOM.total_force.x
    output: Rocket.Forces.total_force.x
  # ... (6 routes)
  
  # Mass → EOM (mass properties)
  - input: Rocket.EOM.total_mass
    output: Rocket.Mass.total_mass
  # ... (7 routes)
```

> [!IMPORTANT]
> **User's Question**: Should we combine these into a single "Vehicle" or "6DOF Entity" component?

---

## Analysis

### Current Architecture Pros

| Aspect | Benefit |
|:-------|:--------|
| **Separation of Concerns** | Each component has a clear, testable responsibility |
| **Composability** | Can use MassAggregator without EOM (e.g., ground test), or ForceAggregator for non-6DOF bodies |
| **IDOA Compliance** | Follows the "flat simulation" philosophy — all components are peers |
| **Vulcan Alignment** | Each component maps cleanly to Vulcan utilities (`MassProperties`, `compute_6dof_derivatives`) |

### Current Architecture Cons

| Aspect | Pain Point |
|:-------|:-----------|
| **Wiring Verbosity** | ~26 routes for a basic 6DOF body is error-prone |
| **Conceptual Overhead** | Users must understand three components to model one vehicle |
| **Scheduling Complexity** | Must order Mass → Forces → EOM correctly |
| **Cohesion Gap** | Mass, forces, and dynamics are *always* needed together for a 6DOF body |

---

## Design Options

### Option A: Keep Separate Components (Status Quo + Wire Bundles)

**Approach**: Keep the current three components, but add "wire bundle" helpers to reduce YAML verbosity.

```yaml
# New: Wire bundle syntax
wire_bundles:
  - type: 6dof_entity
    entity: Rocket
    mass_aggregator: Mass
    force_aggregator: Forces
    rigid_body: EOM
```

**Pros**:

- No code changes to components
- Maintains composability
- Wire bundles are purely syntactic sugar

**Cons**:

- Still three components conceptually
- Wire bundle logic adds complexity to loader
- Doesn't solve programmatic API verbosity

---

### Option B: Unified "Vehicle6DOF" Component

**Approach**: Create a new component that *contains* the logic of all three:

```cpp
template <typename Scalar>
class Vehicle6DOF : public Component<Scalar> {
    // Owns state: position, velocity, attitude, omega
    // Owns mass aggregation logic
    // Owns force aggregation logic
    // Outputs: all state + LLA/NED/Euler + mass properties
};
```

```yaml
components:
  - type: Vehicle6DOF
    name: Rocket
    mass_sources: [Structure, FuelTank]
    force_body_sources: [Engine]
    force_ecef_sources: [Gravity]
    initial_lla: [0.0, 0.0, 100000.0]
    initial_euler_zyx: [0.0, 90.0, 0.0]
```

**Pros**:

- Single component per vehicle — simple YAML
- All mass/force wiring internal to component
- Matches user mental model ("this is my rocket")

**Cons**:

- **Violates IDOA principles** — component becomes a "god object"
- Cannot use mass aggregation without dynamics
- Code duplication if we also keep the separate components
- Harder to test (monolithic)

---

### Option C: Unified "RigidBody6DOF" with Inline Aggregation (Recommended)

**Approach**: Extend `RigidBody6DOF` to optionally aggregate mass/forces internally when sources are specified, while keeping the aggregator components as standalone utilities.

```yaml
components:
  # Standalone mass sources (unchanged)
  - type: StaticMass
    name: Structure
    entity: Rocket
    ...

  - type: FuelTank
    name: FuelTank
    entity: Rocket
    ...

  # Unified body — aggregates internally
  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    mass_sources: [Structure, FuelTank]    # New: inline mass aggregation
    force_body_sources: [Engine]           # New: inline force aggregation
    force_ecef_sources: [Gravity]
    initial_lla: [0.0, 0.0, 100000.0]
    initial_euler_zyx: [0.0, 90.0, 0.0]
```

**Key Change**: When `mass_sources` or `force_*_sources` are specified, `RigidBody6DOF` performs aggregation internally. When not specified, it expects the traditional input signals (backwards compatible).

**Implementation**:

1. **Stage()**: If `mass_sources` is non-empty:
   - Resolve handles to each mass source's outputs (mass, cg, inertia)
   - Store handles internally (like `MassAggregator` does)

2. **Step()**: If sources are configured:
   - Aggregate mass properties directly (using `vulcan::mass::MassProperties::operator+`)
   - Aggregate forces/moments directly (with frame transform)
   - Use aggregated values instead of reading from input handles

**Pros**:

- Backwards compatible (existing configs still work)
- Reduces wiring to near-zero for common case
- Keeps aggregator components as standalone utilities
- Single component models common "6DOF body" pattern
- Aligns with Vulcan's `compute_6dof_derivatives` signature

**Cons**:

- `RigidBody6DOF` becomes larger (but still focused on one physical concept)
- Two ways to configure the same behavior (input signals vs sources list)

---

### Option D: "Entity" Meta-Component

**Approach**: Create an "Entity" wrapper that composes multiple sub-components, hiding internal wiring.

```yaml
entities:
  - name: Rocket
    template: vehicle_6dof
    config:
      mass_sources: [Structure, FuelTank]
      force_body_sources: [Engine]
      force_ecef_sources: [Gravity]
```

The loader would expand this into the three separate components with auto-generated wiring.

**Pros**:

- Clean YAML ergonomics
- Preserves component separation internally
- Can support different "templates" (vehicle_3dof, point_mass, etc.)

**Cons**:

- Complex loader logic
- Hidden complexity (debugging harder)
- Entity templates are a new concept to learn

---

## Recommendation: Option C

**Extend `RigidBody6DOF` with inline aggregation** is the best balance of:

1. **Simplicity**: One component = one vehicle (common case)
2. **Composability**: Aggregators still available for advanced use
3. **Backwards Compatibility**: Existing configs work unchanged
4. **IDOA Compliance**: Still peers under the hood — just optional internal aggregation

### Proposed Signal Topology (Option C)

```
┌─────────────────────────────────────────────────────────────────────┐
│                          RigidBody6DOF                              │
│                                                                     │
│  ┌─────────────────┐    ┌──────────────────┐    ┌───────────────┐  │
│  │ Mass Sources    │───▶│ Internal Mass    │───▶│ Dynamics      │  │
│  │ (resolved)      │    │ Aggregation      │    │ (Vulcan)      │  │
│  └─────────────────┘    └──────────────────┘    └───────┬───────┘  │
│                                                         │          │
│  ┌─────────────────┐    ┌──────────────────┐            │          │
│  │ Force Sources   │───▶│ Internal Force   │────────────┘          │
│  │ (resolved)      │    │ Aggregation      │                       │
│  └─────────────────┘    └──────────────────┘                       │
│                                                                     │
│  Outputs: position, velocity, attitude, omega, LLA, NED, Euler     │
│           total_mass, cg, inertia (if aggregating internally)      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Alternative Naming

The user mentioned "Vehicle" or "6DOF Entity" as potential names. Options:

| Name | Pros | Cons |
|:-----|:-----|:-----|
| `RigidBody6DOF` (current) | No rename needed, familiar | Doesn't convey aggregation capability |
| `Vehicle6DOF` | Intuitive for aero users | Implies vehicle (not generic rigid body) |
| `Entity6DOF` | Matches "entity" namespace concept | Overloads "Entity" terminology |
| `Body6DOF` | Short, generic | May conflict with "body frame" |

**Suggestion**: Keep `RigidBody6DOF` with extended functionality. Document the aggregation feature prominently.

---

## Implementation Phases

### Phase 1: Add Inline Mass Aggregation to RigidBody6DOF

1. Add `std::vector<std::string> mass_source_names_` config field
2. In `Stage()`: resolve mass source handles if configured
3. In `Step()`: aggregate mass properties if sources exist, else use input handles
4. Output aggregated mass properties (new outputs: `total_mass`, `cg`, `inertia.*`)

### Phase 2: Add Inline Force Aggregation to RigidBody6DOF

1. Add `body_force_sources_` and `ecef_force_sources_` config fields
2. In `Stage()`: resolve force source handles if configured
3. In `Step()`: aggregate forces/moments if sources exist, else use input handles
4. No new outputs (forces/moments are consumed internally)

### Phase 3: Update YAML Examples

1. Create `rocket_ascent_unified.yaml` showing new syntax
2. Update documentation with both patterns (explicit wiring vs inline aggregation)
3. Add migration guide for existing users

### Phase 4: Deprecation (Future)

1. Consider deprecating standalone `MassAggregator` / `ForceAggregator` if unused
2. Or keep them as "power user" components for non-6DOF scenarios

---

## Verification Plan

### Automated Tests

Since this is a design document, no automated tests are written yet. When implementing:

```bash
# Existing test commands
./scripts/test.sh

# Specific component tests
./scripts/test.sh rigid_body
./scripts/test.sh aggregator
```

### Manual Verification

1. Run `rocket_ascent.yaml` with both old and new syntax
2. Compare telemetry outputs (should be identical)
3. Verify symbolic mode still works with new aggregation

---

## Open Questions for User Review

1. **Naming**: Should we rename `RigidBody6DOF` to `Vehicle6DOF` or similar?
2. **Scope**: Should inline aggregation be opt-in (recommended) or the only mode?
3. **Output Scope**: Should aggregated mass/force be exposed as outputs, or kept internal?
4. **Alternative**: Is Option D (Entity templates) worth exploring further?

---

## Related Documents

- [01_core_philosophy.md](file:///home/tanged/sources/icarus/docs/architecture/01_core_philosophy.md)
- [06_entities_namespaces.md](file:///home/tanged/sources/icarus/docs/architecture/06_entities_namespaces.md)
- [12_force_aggregation.md](file:///home/tanged/sources/icarus/docs/architecture/12_force_aggregation.md)

---

## Appendix: Current YAML Wiring (Full)

The following routes are required today for a 6DOF body with mass and force aggregation:

```yaml
routes:
  # Mass → ForceAggregator (CG for moment transfer)
  - input: Rocket.Forces.cg.x
    output: Rocket.Mass.cg.x
  - input: Rocket.Forces.cg.y
    output: Rocket.Mass.cg.y
  - input: Rocket.Forces.cg.z
    output: Rocket.Mass.cg.z

  # EOM → ForceAggregator (attitude for ECEF→body transform)
  - input: Rocket.Forces.attitude.w
    output: Rocket.EOM.attitude.w
  - input: Rocket.Forces.attitude.x
    output: Rocket.EOM.attitude.x
  - input: Rocket.Forces.attitude.y
    output: Rocket.EOM.attitude.y
  - input: Rocket.Forces.attitude.z
    output: Rocket.EOM.attitude.z

  # ForceAggregator → EOM (forces/moments)
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

  # Mass → EOM (mass properties)
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

  # (Plus routes for gravity position, engine mass flow, etc.)
```

With Option C (inline aggregation), this entire block is replaced by:

```yaml
components:
  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    mass_sources: [Structure, FuelTank]
    force_body_sources: [Engine]
    force_ecef_sources: [Gravity]
    # ... initial conditions ...
```
