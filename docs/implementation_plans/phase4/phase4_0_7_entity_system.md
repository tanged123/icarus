# Phase 4.0.7: Entity System

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Single-File vs Multi-File

**Default:** Everything can be defined in ONE file. Multi-file organization is optional.

```yaml
# single_file_sim.yaml - EVERYTHING inline, no external files needed
simulation:
  name: "Simple Orbital Demo"

components:
  - type: PointMassGravity
    name: Gravity
    scalars:
      mu: 3.986004418e14

  - type: PointMass3DOF
    name: Dynamics
    vectors:
      initial_position: [6.871e6, 0, 0]
      initial_velocity: [0, 7500, 0]

routes:
  - input: Gravity.position
    output: Dynamics.position
  - input: Dynamics.force
    output: Gravity.force

integrator:
  type: RK4
```

**Multi-file mode** (for organization only):
```yaml
# simulation.yaml - uses includes for organization
simulation:
  name: "Formation Flight"

entities:
  - template: entities/rocket.yaml  # External file
    name: Leader
```

The loader flattens everything into one config internally. Users choose based on their organization needs.

---

## Overview

Entity templates are **self-contained units** that define their own:
- Components
- Internal signal routes
- Internal scheduler (rate groups within the entity)
- Internal staging/trim configuration

The simulation level then only handles:
- Entity instantiation
- Cross-entity routes
- Global coordination

---

## Master Binding File

The `simulation.yaml` is the top-level entry point. It instantiates entities and handles cross-entity coordination.

```yaml
# simulation.yaml - Master binding file

simulation:
  name: "Formation Flight Simulation"
  version: "1.0.0"

# =========================================================================
# Time Configuration
# =========================================================================
time:
  start: 0.0
  end: 300.0
  dt: 0.01           # Base timestep (entities may have faster internal rates)

# =========================================================================
# Entity Instances
# =========================================================================
entities:
  - template: entities/rocket.yaml
    name: Leader
    overrides:
      EOM:
        vectors:
          initial_position: [0, 0, 6.471e6]
          initial_velocity: [7500, 0, 0]

  - template: entities/rocket.yaml
    name: Follower1
    overrides:
      EOM:
        vectors:
          initial_position: [100, 0, 6.471e6]
      GNC:
        strings:
          mode: "follow"

  - template: entities/ground_station.yaml
    name: KSC

# =========================================================================
# Cross-Entity Routes (between different entities)
# =========================================================================
cross_entity_routes:
  - input: Follower1.GNC.leader_position
    output: Leader.EOM.position

  - input: KSC.Tracker.target_position
    output: Leader.EOM.position

# =========================================================================
# Global Coordination (entity execution order)
# =========================================================================
coordination:
  mode: automatic   # Infer from cross-entity dependencies
  # Or explicit:
  # entity_order: [Leader, Follower1, KSC]

# =========================================================================
# Global Staging Overrides (optional)
# =========================================================================
staging_overrides:
  Leader:
    trim:
      tolerance: 1.0e-8   # Tighter for leader

# =========================================================================
# Integrator, Logging (global)
# Note: Environment models (atmosphere, gravity, winds) are defined as
#       components within entity templates, not as global config.
# =========================================================================
integrator:
  type: RK4

logging:
  console:
    level: Info
  file:
    enabled: true
    path: "./logs/simulation.log"
  telemetry:
    path: "./output/telemetry.h5"
    signals: [Leader.EOM.position, Leader.EOM.velocity]

output:
  directory: "./output"
  data_dictionary: true
```

---

## Entity Template (Self-Contained)

```yaml
# entities/rocket.yaml - Complete entity template

entity:
  name: Rocket
  description: "6DOF rocket with GNC"

  # =========================================================================
  # Components (what this entity is made of)
  # =========================================================================
  components:
    - type: RigidBody6DOF
      name: EOM
      config_file: "./models/6dof_dynamics.yaml"

    - type: PointMassGravity
      name: Gravity
      scalars:
        mu: 3.986004418e14

    - type: Thrust
      name: Engine
      config_file: "./models/rs25_engine.yaml"

    - type: SimpleGNC
      name: GNC

    - type: MassAggregator
      name: Mass
      integers:
        num_sources: 2

    - type: ForceAggregator
      name: Forces
      integers:
        num_sources: 3

  # =========================================================================
  # Internal Routes (within this entity, relative names)
  # =========================================================================
  routes:
    # Gravity
    - input: Gravity.position
      output: EOM.position

    # Force aggregation
    - input: Forces.cg
      output: Mass.cg
    - input: Forces.src0_force
      output: Gravity.force
    - input: Forces.src1_force
      output: Engine.force

    # EOM inputs
    - input: EOM.total_force
      output: Forces.total_force
    - input: EOM.total_moment
      output: Forces.total_moment
    - input: EOM.mass
      output: Mass.total_mass
    - input: EOM.inertia
      output: Mass.inertia

    # GNC
    - input: Engine.throttle
      output: GNC.throttle_cmd

  # =========================================================================
  # Internal Scheduler (hierarchical multi-rate)
  # =========================================================================
  scheduler:
    master_rate_hz: 100

    # Master execution list (runs at 100hz)
    master_list:
      - component: Gravity
        priority: 1
      - component: Engine
        priority: 2
      - group: GNC_group      # <-- Higher-rate group
        priority: 3
      - component: Forces
        priority: 4
      - component: Mass
        priority: 5
      - component: EOM
        priority: 6

    # Higher-rate groups
    groups:
      - name: GNC_group
        rate_hz: 400          # 4× master rate → runs 4 iterations per master frame
        members:
          - component: GNC
            priority: 1

    topology:
      mode: explicit          # automatic ordering is future TODO
      cycle_detection: error
      log_order: true

  # =========================================================================
  # Internal Staging (how to trim THIS entity)
  # =========================================================================
  staging:
    trim:
      enabled: true
      method: newton

      # Derivatives to zero (relative names)
      zero_derivatives:
        - EOM.velocity_dot
        - EOM.angular_rate_dot

      # Controls to adjust (relative names)
      control_signals:
        - GNC.throttle_cmd
        - GNC.pitch_cmd

      # Bounds
      control_bounds:
        GNC.throttle_cmd: [0.0, 1.0]
        GNC.pitch_cmd: [-0.5, 0.5]

    linearization:
      enabled: true
      states: [EOM.position, EOM.velocity, EOM.attitude, EOM.angular_rate]
      inputs: [GNC.throttle_cmd, GNC.pitch_cmd]
      outputs: [EOM.position, EOM.velocity]
```

---

## Override System

Overrides modify **ComponentConfig values** at entity instantiation time. They do NOT affect signal routing (inputs/outputs), which is structural and handled by the `routes:` section.

### What Can Be Overridden

Components receive configuration through `ComponentConfig` (see [phase4_0_config_infrastructure.md](phase4_0_config_infrastructure.md)). Overrides modify the config maps **before** they're passed to `Provision()`:

| ComponentConfig Field | Overrideable | Accessed Via |
|:----------------------|:-------------|:-------------|
| `scalars` | ✅ Yes | `cfg.Get<double>("mass", 1.0)` |
| `vectors` | ✅ Yes | `cfg.Get<Vec3<double>>("initial_position", ...)` |
| `strings` | ✅ Yes | `cfg.Get<std::string>("table_path", "")` |
| `integers` | ✅ Yes | `cfg.Get<int>("num_sources", 1)` |
| `booleans` | ✅ Yes | `cfg.Get<bool>("enable_drag", true)` |
| Signal ports | ❌ No | `bp.declare_input<T>()` / `bp.declare_output<T>()` |

The component has no knowledge of whether a value came from the template default or an instance override - it just calls `cfg.Get<T>()` and receives the final merged value.

### Explicit External Dependencies

**All external dependencies must be requested via `cfg.Get<T>()`.** Components should NOT implicitly load files that aren't in the config. This ensures:

1. **Full introspection** - ComponentConfig shows all dependencies
2. **Overrideable** - can swap files per entity instance
3. **Validation** - paths can be verified before component init
4. **Self-documenting** - no hidden "magic" file expectations

```cpp
// GOOD: Explicit table path in config
void ThrustModel::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // Request config values (these can be overridden per-instance)
    table_path_ = cfg.Get<std::string>("table_path", "./models/thrust.csv");
    interp_mode_ = cfg.Get<int>("interpolation_mode", 0);
    scale_factor_ = cfg.Get<double>("scale_factor", 1.0);

    // Declare signal ports (structural, not overrideable)
    bp.declare_output<Vec3<Scalar>>("force", &force_, "N", "Thrust force");
    bp.declare_input<Scalar>("throttle", &throttle_input_, "nd", "Throttle 0-1");
}

void ThrustModel::Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // Load from the (possibly overridden) path
    thrust_table_ = LoadTable(table_path_);
}

// BAD: Implicit file loading (don't do this)
void BadModel::Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // Hidden dependency - not visible in config, not overrideable
    auto table = LoadTable("./models/hardcoded_table.csv");  // ❌
}
```

### Override Merge Logic

Overrides are merged into `ComponentConfig` before being passed to the component:

```cpp
ComponentConfig MergeOverrides(const ComponentConfig& template_cfg,
                               const ComponentConfig& overrides) {
    ComponentConfig merged = template_cfg;

    // Merge each config category
    for (const auto& [key, value] : overrides.scalars)
        merged.scalars[key] = value;
    for (const auto& [key, value] : overrides.vectors)
        merged.vectors[key] = value;
    for (const auto& [key, value] : overrides.integers)
        merged.integers[key] = value;
    for (const auto& [key, value] : overrides.strings)
        merged.strings[key] = value;
    for (const auto& [key, value] : overrides.booleans)
        merged.booleans[key] = value;

    return merged;
}

// Usage during entity expansion:
for (const auto& instance : entities) {
    auto tmpl = LoadTemplate(instance.template_path);

    for (auto& comp_cfg : tmpl.components) {
        // Merge instance overrides for this component
        if (instance.overrides.count(comp_cfg.name)) {
            comp_cfg = MergeOverrides(comp_cfg, instance.overrides[comp_cfg.name]);
        }

        // Set entity prefix
        comp_cfg.entity = instance.name;

        // Component receives merged config - doesn't know what was overridden
        auto component = factory.Create(comp_cfg);
        component->Provision(bp, comp_cfg);
    }
}
```

---

## Entity Instantiation with Overrides

```yaml
# simulation.yaml

entities:
  - template: entities/rocket.yaml
    name: Leader
    overrides:
      EOM:
        vectors:
          initial_position: [0, 0, 6.471e6]
          initial_velocity: [7500, 0, 0]

  - template: entities/rocket.yaml
    name: Follower1
    overrides:
      EOM:
        vectors:
          initial_position: [100, 0, 6.471e6]
      GNC:
        strings:
          mode: "follow"

  - template: entities/rocket.yaml
    name: Follower2
    overrides:
      EOM:
        vectors:
          initial_position: [-100, 0, 6.471e6]
      GNC:
        strings:
          mode: "follow"

  - template: entities/ground_station.yaml
    name: KSC
    overrides:
      Position:
        vectors:
          lla: [28.5, -80.6, 0]
```

---

## Cross-Entity Routes

```yaml
# simulation.yaml

cross_entity_routes:
  # Followers track Leader
  - input: Follower1.GNC.leader_position
    output: Leader.EOM.position

  - input: Follower2.GNC.leader_position
    output: Leader.EOM.position

  # Ground station tracking
  - input: KSC.Tracker.target0_position
    output: Leader.EOM.position

  - input: KSC.Tracker.target1_position
    output: Follower1.EOM.position
```

---

## Global Coordination

```yaml
# simulation.yaml

coordination:
  # Entity execution order (for cross-entity dependencies)
  entity_order:
    - Leader      # Leader computes first
    - Follower1   # Then followers (they depend on Leader)
    - Follower2
    - KSC         # Ground station last (observes all)

  # Or automatic based on cross-entity route dependencies
  mode: automatic
```

---

## Global Staging Overrides

```yaml
# simulation.yaml

staging:
  # Can override per-entity staging
  Leader:
    trim:
      tolerance: 1.0e-8  # Tighter for leader

  # Or add global constraints
  global:
    # e.g., trim all vehicles to same velocity
    constraints:
      - Leader.EOM.velocity == Follower1.EOM.velocity
```

---

## Expansion Process

When `FromConfig()` is called, the system:

1. **Load entity templates** from files
2. **Instantiate entities** with overrides merged
3. **Expand internal routes** (relative → absolute):
   - `EOM.position` → `Leader.EOM.position`
4. **Merge all routes** (internal + cross-entity)
5. **Build unified scheduler** from entity schedulers + global coordination
6. **Flatten to components** for internal C++ representation

```cpp
// Internal expansion
std::vector<ComponentConfig> expanded_components;
std::vector<SignalRoute> expanded_routes;
SchedulerConfig unified_scheduler;

for (const auto& instance : entities) {
    auto tmpl = LoadTemplate(instance.template_path);

    // Expand components with entity prefix
    for (auto comp : tmpl.components) {
        comp.entity = instance.name;  // Set entity prefix
        MergeOverrides(comp, instance.overrides[comp.name]);
        expanded_components.push_back(comp);
    }

    // Expand routes with entity prefix
    for (auto route : tmpl.routes) {
        route.input_path = instance.name + "." + route.input_path;
        route.output_path = instance.name + "." + route.output_path;
        expanded_routes.push_back(route);
    }

    // Merge scheduler
    unified_scheduler.MergeEntityScheduler(instance.name, tmpl.scheduler);
}

// Add cross-entity routes
for (const auto& route : cross_entity_routes) {
    expanded_routes.push_back(route);
}
```

---

## Swarm Instantiation

For large-scale simulations with many similar entities:

```yaml
# simulation.yaml

swarms:
  - template: entities/quadcopter.yaml
    name_prefix: Drone
    count: 100
```

This generates `Drone_000`, `Drone_001`, ..., `Drone_099` from the template.

### Per-Instance Overrides (Future)

> **Deferred:** Expression-based per-instance configuration (e.g., `${index}` for grid positioning) is future work. Consider Jinja-style templating.

For now, swarms create identical copies. Per-instance differentiation requires:
1. Post-processing the config programmatically, or
2. Using `SetInputSource()` to inject initial conditions at runtime

---

## Entity System Config Structs

```cpp
namespace icarus {

/**
 * @brief Entity template loaded from YAML
 */
struct EntityTemplate {
    std::string name;
    std::string description;

    std::vector<ComponentConfig> components;
    std::vector<SignalRoute> routes;
    SchedulerConfig scheduler;
    StageConfig staging;

    static EntityTemplate FromFile(const std::string& yaml_path);
};

/**
 * @brief Entity instance (template + name + overrides)
 */
struct EntityInstance {
    std::string template_path;
    std::string name;

    // Component name -> config overrides
    std::unordered_map<std::string, ComponentConfig> overrides;
};

/**
 * @brief Swarm configuration for bulk entity spawning
 *
 * Creates count copies of the template with names: prefix_000, prefix_001, ...
 * All copies are identical (per-instance expressions are future work).
 */
struct SwarmConfig {
    std::string template_path;
    std::string name_prefix;
    int count = 1;

    // Future: per-instance overrides with Jinja-style templating
    // Future: swarm-internal routes (neighbor connections)
};

/**
 * @brief Entity system configuration
 */
struct EntitySystemConfig {
    std::vector<EntityInstance> entities;
    std::vector<SwarmConfig> swarms;
    std::vector<SignalRoute> cross_entity_routes;

    // Entity execution order
    std::vector<std::string> entity_order;
    bool auto_order = true;

    /**
     * @brief Expand all entities and swarms to flat component list
     */
    std::tuple<std::vector<ComponentConfig>,
               std::vector<SignalRoute>,
               SchedulerConfig> ExpandAll() const;
};

} // namespace icarus
```

---

## Global Components (Entity-Independent)

> **Note:** Global components may be implemented in a later phase. This section documents the design for future reference.

While entity templates are the primary organizational unit, some components are inherently **simulation-global** rather than entity-specific. These include:

- **Environment models** (atmosphere, gravity, winds, terrain)
- **Epoch/time management**
- **Global coordinate frames**
- **Shared lookup tables or databases**

Since entities are just namespace prefixes in the signal backplane (see [06_entities_namespaces.md](../../architecture/06_entities_namespaces.md)), global components are straightforward: they simply use a reserved namespace like `Global.` or `Environment.`.

### Global Component Declaration

```yaml
# simulation.yaml

# =========================================================================
# Global Components (simulation-wide, not entity-specific)
# =========================================================================
global_components:
  - type: US76Atmosphere
    name: Atmosphere
    config_file: "./models/atmosphere.yaml"

  - type: EGM96Gravity
    name: Gravity
    scalars:
      degree: 20
      order: 20

  - type: HWM14Winds
    name: Winds
    config_file: "./models/winds.yaml"

  - type: SRTM30Terrain
    name: Terrain
    strings:
      data_path: "./data/terrain/"
```

### Signal Namespace

Global components use a flat namespace (no entity prefix):

```
Atmosphere.density
Atmosphere.pressure
Atmosphere.temperature
Gravity.acceleration
Winds.velocity_ned
Terrain.elevation
```

Alternatively, a `Global.` or `Environment.` prefix can be used for clarity:

```
Environment.Atmosphere.density
Environment.Gravity.acceleration
Global.Epoch.julian_date
```

The choice is configuration-level; internally both flatten to the same signal backplane.

### Entity Access to Global Components

Entity components access global signals via cross-entity style routing:

```yaml
# entities/rocket.yaml

routes:
  # Internal routes (relative, within entity)
  - input: EOM.force
    output: Forces.total_force

  # Global component access (absolute paths)
  - input: Aero.density
    output: Atmosphere.density        # Global component
  - input: Aero.wind_velocity
    output: Winds.velocity_ned        # Global component
  - input: Gravity.mu
    output: Gravity.mu                # Global component
```

When the entity template is instantiated, internal routes are prefixed with the entity name, but routes to global components remain as-is:

```
# After expansion for entity "Leader":
Leader.Aero.density      ← Atmosphere.density       (global)
Leader.Aero.wind_velocity ← Winds.velocity_ned       (global)
Leader.EOM.force         ← Leader.Forces.total_force (internal)
```

### Simulation-Level Binding

The `simulation.yaml` shows the complete picture:

```yaml
# simulation.yaml

simulation:
  name: "Multi-Vehicle Atmospheric Flight"

# =========================================================================
# Global Components
# =========================================================================
global_components:
  - type: US76Atmosphere
    name: Atmosphere
  - type: EGM96Gravity
    name: Gravity

# =========================================================================
# Entities
# =========================================================================
entities:
  - template: entities/aircraft.yaml
    name: Leader
  - template: entities/aircraft.yaml
    name: Wingman

# =========================================================================
# Cross-Entity Routes
# =========================================================================
cross_entity_routes:
  - input: Wingman.GNC.leader_position
    output: Leader.EOM.position

# =========================================================================
# Global Routes (entity → global or global → entity)
# =========================================================================
global_routes:
  # All aircraft query the same atmosphere
  - input: Leader.Aero.density
    output: Atmosphere.density
  - input: Wingman.Aero.density
    output: Atmosphere.density

  # Gravity model receives positions
  - input: Gravity.query_position_0
    output: Leader.EOM.position
  - input: Gravity.query_position_1
    output: Wingman.EOM.position
```

### Scheduler Integration

Global components participate in the unified scheduler. They can be assigned to rate groups or have explicit execution order:

```yaml
# simulation.yaml

scheduler:
  # Global components typically run once per frame before entities
  global_execution:
    - component: Atmosphere
      priority: 0
    - component: Gravity
      priority: 1
    - component: Winds
      priority: 2

  # Entity execution follows
  entity_order: [Leader, Wingman]
```

### Global Component Config

Global components use the standard `ComponentConfig` (see [phase4_0_config_infrastructure.md](phase4_0_config_infrastructure.md)) with `entity = ""` (empty string) to indicate they have no entity prefix:

```cpp
namespace icarus {

/**
 * @brief Extended simulation config with global components
 */
struct SimulationConfig {
    std::string name;
    std::string version;

    // Global components use ComponentConfig with entity = ""
    std::vector<ComponentConfig> global_components;

    // Entity instances
    std::vector<EntityInstance> entities;
    std::vector<SwarmConfig> swarms;

    // Routing
    std::vector<SignalRoute> cross_entity_routes;
    std::vector<SignalRoute> global_routes;

    // Scheduler
    SchedulerConfig scheduler;

    // ...
};

} // namespace icarus
```

When loading global components:

```cpp
for (const auto& global_node : yaml["global_components"]) {
    ComponentConfig cfg = io::ConfigLoader::LoadComponent(global_node);
    cfg.entity = "";  // Empty entity = global namespace

    auto component = factory.Create(cfg);
    component->Provision(bp, cfg);
    // Signals registered as "Atmosphere.density" not "SomeEntity.Atmosphere.density"
}
```

### Expansion Process (Updated)

When `FromConfig()` is called, the system:

1. **Load global components** (no prefix)
2. **Load entity templates** from files
3. **Instantiate entities** with overrides merged
4. **Expand internal routes** (relative → absolute with entity prefix)
5. **Add global routes** (already absolute)
6. **Add cross-entity routes** (already absolute)
7. **Build unified scheduler** with global components first

```cpp
std::vector<ComponentConfig> expanded_components;
std::vector<SignalRoute> expanded_routes;

// 1. Global components (entity = "")
for (auto cfg : sim_config.global_components) {
    cfg.entity = "";  // Empty = global namespace
    expanded_components.push_back(cfg);
}

// 2. Entity components (entity = instance name)
for (const auto& instance : sim_config.entities) {
    auto tmpl = LoadTemplate(instance.template_path);

    for (auto cfg : tmpl.components) {
        // Merge instance overrides
        if (instance.overrides.count(cfg.name)) {
            cfg = MergeOverrides(cfg, instance.overrides.at(cfg.name));
        }
        cfg.entity = instance.name;  // Set entity prefix
        expanded_components.push_back(cfg);
    }

    // Expand internal routes (relative → absolute)
    for (auto route : tmpl.routes) {
        route.input_path = instance.name + "." + route.input_path;
        route.output_path = instance.name + "." + route.output_path;
        expanded_routes.push_back(route);
    }
}

// 3. Add global and cross-entity routes (already absolute)
expanded_routes.insert(expanded_routes.end(),
                       sim_config.global_routes.begin(),
                       sim_config.global_routes.end());
expanded_routes.insert(expanded_routes.end(),
                       sim_config.cross_entity_routes.begin(),
                       sim_config.cross_entity_routes.end());
```

### Design Rationale

| Approach | Pros | Cons |
|:---------|:-----|:-----|
| **Global components** | Single instance shared by all entities; natural for environment models | Slightly more complex config structure |
| **Per-entity environment** | Each entity has its own atmosphere/gravity | Redundant computation; harder to ensure consistency |
| **Implicit globals** | Environment "just exists" | Less explicit; harder to configure |

**Conclusion:** Explicit global components provide the right balance of clarity and efficiency. Environment models are inherently shared, and the namespace system naturally supports this.

---

## Exit Criteria

### Entity System (Core)
- [ ] **Single-file mode**: entire sim definable in one YAML (no external files required)
- [ ] **Multi-file mode**: entity templates loadable from external files (optional)
- [ ] `EntityTemplate` with components + internal routes
- [ ] `EntityInstance` with template reference + overrides
- [ ] `SwarmConfig` with count (identical copies, no expressions)
- [ ] `EntitySystemConfig` with `ExpandAll()` expansion
- [ ] Override merging (template defaults + instance overrides)
- [ ] Internal route expansion (relative → absolute: `EOM.force` → `Leader.EOM.force`)
- [ ] Cross-entity route support

### Deferred
- [ ] *(Future)* Swarm per-instance expressions (`${index}`, Jinja-style)
- [ ] *(Future)* Swarm-internal routes (neighbor connections)

### Global Components (Future Phase)
- [ ] `global_components` section in simulation.yaml (uses standard `ComponentConfig` with `entity = ""`)
- [ ] `global_routes` for entity ↔ global component connections
- [ ] Scheduler integration with `global_execution` section
- [ ] Namespace handling (flat `Atmosphere.density` or prefixed `Environment.Atmosphere.density`)
- [ ] Expansion process updated for global components
