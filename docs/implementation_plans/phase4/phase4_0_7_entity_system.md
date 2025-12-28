# Phase 4.0.7: Entity System

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

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

    per_instance:
      EOM:
        vectors:
          initial_position:
            - "${(index % 10) * 5.0}"
            - "${(index / 10) * 5.0}"
            - "100.0"

    # Swarm-internal routes (between swarm members)
    swarm_routes:
      # Each drone knows its neighbors
      - input: "${name}.GNC.neighbor_left"
        output: "${prev_name}.EOM.position"
        skip_first: true

      - input: "${name}.GNC.neighbor_right"
        output: "${next_name}.EOM.position"
        skip_last: true
```

### Swarm Expression Variables

| Variable | Description | Example |
|:---------|:------------|:--------|
| `${index}` | Zero-based index of entity in swarm | `0`, `1`, ..., `99` |
| `${name}` | Full entity name | `Drone_042` |
| `${prev_name}` | Previous entity name | `Drone_041` |
| `${next_name}` | Next entity name | `Drone_043` |

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
 */
struct SwarmConfig {
    std::string template_path;
    std::string name_prefix;
    int count = 1;

    // Per-instance overrides with expression evaluation
    std::unordered_map<std::string, ComponentConfig> per_instance;

    // Swarm-internal routes
    std::vector<SignalRoute> swarm_routes;
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

## Exit Criteria

- [ ] `EntityTemplate` with components + internal routes
- [ ] `EntityInstance` with template reference + overrides
- [ ] `SwarmConfig` with count + per-instance expressions
- [ ] `EntitySystemConfig` with `ExpandAll()` expansion
- [ ] Entity template YAML loading from external files
- [ ] Override merging (template defaults + instance overrides)
- [ ] Swarm expansion with expression evaluation (`${index}`)
- [ ] Internal route expansion (relative → absolute: `EOM.force` → `Leader.EOM.force`)
- [ ] Cross-entity route support
