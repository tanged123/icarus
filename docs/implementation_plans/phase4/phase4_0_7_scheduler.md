# Phase 4.0.7: Scheduler Configuration

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Overview

Scheduling is defined **per-entity** in the entity template. Each entity specifies its own execution order for its internal components. The simulation level only coordinates entity execution order.

---

## Global Timing Model

The simulation has a **global master rate** that all entity rates must conform to.

### Rate Derivation

The simulation dt is **auto-derived** from the fastest rate across all entities:

```
Simulation dt = 1 / max(all entity rates, all group rates)
```

Example:
```yaml
# Entity 1: master 400 Hz, GNC group 1600 Hz
# Entity 2: master 200 Hz
# Entity 3: master 800 Hz

# Auto-derived: simulation runs at 1600 Hz (dt = 0.000625s)
# - Entity 1 master: runs every 4th step (1600/400 = 4)
# - Entity 1 GNC: runs every step (1600/1600 = 1)
# - Entity 2: runs every 8th step (1600/200 = 8)
# - Entity 3: runs every 2nd step (1600/800 = 2)
```

### Rate Constraints

**All rates must be integer divisors of the global simulation rate.**

| Rate | 1600 Hz sim | Valid? |
|:-----|:------------|:-------|
| 1600 Hz | 1600/1600 = 1 | ✅ |
| 800 Hz | 1600/800 = 2 | ✅ |
| 400 Hz | 1600/400 = 4 | ✅ |
| 200 Hz | 1600/200 = 8 | ✅ |
| 513 Hz | 1600/513 = 3.12... | ❌ **Rejected** |
| 333 Hz | 1600/333 = 4.8... | ❌ **Rejected** |

### Validation

Rates are validated at config load time:

```cpp
std::vector<std::string> ValidateGlobalTiming(
    double sim_rate_hz,
    const std::vector<EntityConfig>& entities
) {
    std::vector<std::string> errors;

    for (const auto& entity : entities) {
        for (const auto& group : entity.scheduler.groups) {
            // Group rate cannot exceed simulation rate
            if (group.rate_hz > sim_rate_hz) {
                errors.push_back(
                    "Entity '" + entity.name + "' group '" + group.name +
                    "' rate " + std::to_string(group.rate_hz) +
                    " Hz exceeds simulation rate " +
                    std::to_string(sim_rate_hz) + " Hz"
                );
            }

            // Group rate must be integer divisor of simulation rate
            double ratio = sim_rate_hz / group.rate_hz;
            if (std::abs(ratio - std::round(ratio)) > 1e-9) {
                errors.push_back(
                    "Entity '" + entity.name + "' group '" + group.name +
                    "' rate " + std::to_string(group.rate_hz) +
                    " Hz is not an integer divisor of simulation rate " +
                    std::to_string(sim_rate_hz) + " Hz"
                );
            }
        }
    }

    return errors;
}
```

### Rate Derivation Algorithm

```cpp
double DeriveSimulationRate(const std::vector<EntityConfig>& entities) {
    double max_rate = 0.0;

    for (const auto& entity : entities) {
        for (const auto& group : entity.scheduler.groups) {
            max_rate = std::max(max_rate, group.rate_hz);
        }
    }

    return max_rate;  // Simulation runs at fastest group rate
}
```

---

## Group-Based Scheduling

The scheduler uses a **group-based model** where:

1. Every component belongs to exactly one **group**
2. Each group has a **rate** and **priority** (execution order relative to other groups)
3. Within each group, components have their own **priority** (internal execution order)
4. Entity is just a namespace - no "entity master rate"

```
Simulation at 1600 Hz (auto-derived from fastest group):

Group: sensors (400 Hz, group priority 1)
├─ IMU.Step()    [member priority 1]
└─ GPS.Step()    [member priority 2]

Group: gnc (1600 Hz, group priority 2)
├─ Guidance.Step()   [member priority 1]
└─ Autopilot.Step()  [member priority 2]

Group: dynamics (400 Hz, group priority 3)
├─ Gravity.Step()  [member priority 1]
├─ Engine.Step()   [member priority 2]
├─ Forces.Step()   [member priority 3]
└─ EOM.Step()      [member priority 4]
```

---

## Two Levels of Scheduling

| Level | What | Where Defined |
|:------|:-----|:--------------|
| **Intra-Entity** | Group rates + priorities, component priorities within groups | Entity template (`scheduler:` section) |
| **Inter-Entity** | Which entity runs before another | Simulation (`coordination:` section) |

---

## Scheduling Mode

Currently only **explicit** mode is supported - user defines execution order via priorities.

| Mode | Description | Status |
|:-----|:------------|:-------|
| `explicit` | User-defined execution order | **Implemented** |
| `automatic` | Topological sort based on signal dependencies | Future TODO |

---

## Entity-Level Scheduler (in entity template)

```yaml
# entities/rocket.yaml - Scheduler is INSIDE the entity template

entity:
  name: Rocket

  components:
    # ... component definitions ...

  routes:
    # ... internal routes ...

  # =========================================================================
  # Scheduler (group-based, explicit priorities)
  # =========================================================================
  scheduler:
    # Every component belongs to exactly one group
    # Groups are sorted by group priority, then members by member priority
    groups:
      - name: sensors
        rate_hz: 400
        priority: 1            # Group runs first
        members:
          - component: IMU
            priority: 1
          - component: GPS
            priority: 2

      - name: gnc
        rate_hz: 1600          # Fastest rate → sets simulation dt
        priority: 2            # Group runs second
        members:
          - component: Guidance
            priority: 1
          - component: Autopilot
            priority: 2

      - name: dynamics
        rate_hz: 400
        priority: 3            # Group runs last
        members:
          - component: Gravity
            priority: 1
          - component: Engine
            priority: 2
          - component: Forces
            priority: 3
          - component: EOM
            priority: 4

    # Topology settings
    topology:
      mode: explicit           # automatic ordering is future TODO
      cycle_detection: error
      log_order: true
```

---

## Cross-Entity Signal Reads

When entity B reads a signal from entity A:

- B sees A's value from the **current frame** (whatever is currently populated)
- If A has already run this frame, B sees A's updated value
- If A has not run yet this frame, B sees A's value from the previous frame
- Entity execution order determines which entities see fresh vs stale data

---

## Sub-Rate Components

Components that need to run slower simply belong to a slower group:

```yaml
scheduler:
  groups:
    - name: fast_sensors
      rate_hz: 400
      priority: 1
      members:
        - component: IMU
          priority: 1

    - name: slow_sensors
      rate_hz: 10           # GPS only needs 10 Hz
      priority: 2
      members:
        - component: GPS
          priority: 1

    - name: gnc
      rate_hz: 1600
      priority: 3
      members:
        - component: Guidance
          priority: 1
```

No need for `rate_divisor` - the group's `rate_hz` directly specifies execution rate.

---

## Execution Model

The simulation runs at the global rate (auto-derived from fastest group across all entities). Each group has a computed `frame_divisor` based on its rate vs the simulation rate.

### Global Step

```
GlobalStep(t, dt_global):
    global_frame_count++

    for entity in sorted(entities, by=entity_order):
        ExecuteEntity(entity, t, dt_global, global_frame_count)
```

### Entity Execution

```
ExecuteEntity(entity, t, dt_global, frame_count):
    for group in sorted(entity.scheduler.groups, by=group.priority):
        group_divisor = sim_rate_hz / group.rate_hz

        if frame_count % group_divisor == 0:
            dt_group = dt_global * group_divisor

            for member in sorted(group.members, by=member.priority):
                member.component.Step(t, dt_group)
```

### Example Execution Timeline

```
Simulation at 1600 Hz (dt = 0.625ms)

Entity A groups:
  - sensors:  400 Hz, priority 1  → divisor 4, dt = 2.5ms
  - gnc:      1600 Hz, priority 2 → divisor 1, dt = 0.625ms
  - dynamics: 400 Hz, priority 3  → divisor 4, dt = 2.5ms

Entity B groups:
  - all: 200 Hz, priority 1 → divisor 8, dt = 5ms

Frame 0 (t=0.000):
  Entity A:
    - sensors group runs (0%4==0): IMU.Step(), GPS.Step() with dt=2.5ms
    - gnc group runs (0%1==0): Guidance.Step(), Autopilot.Step() with dt=0.625ms
    - dynamics group runs (0%4==0): Gravity, Engine, Forces, EOM with dt=2.5ms
  Entity B:
    - all group runs (0%8==0): all B components with dt=5ms

Frame 1 (t=0.000625):
  Entity A:
    - sensors skipped (1%4 != 0)
    - gnc runs: Guidance.Step(), Autopilot.Step() with dt=0.625ms
    - dynamics skipped (1%4 != 0)
  Entity B:
    - all skipped (1%8 != 0)

Frame 2-3: similar pattern (only gnc runs each frame)

Frame 4 (t=0.0025):
  Entity A:
    - sensors runs (4%4==0)
    - gnc runs (4%1==0)
    - dynamics runs (4%4==0)
  Entity B:
    - all skipped (4%8 != 0)

Frame 8 (t=0.005):
  Entity A: all groups run
  Entity B: all group runs (8%8==0)
```

---

## Data Freshness Considerations

When high-rate components read from low-rate components:

- **Default**: High-rate component sees the value from the last time low-rate component ran (ZOH - zero-order hold)
- **Future**: Consider interpolation/extrapolation options for smoother data

---

## Simulation-Level Coordination (entity order)

```yaml
# simulation.yaml - Only handles ENTITY order, not component order

coordination:
  mode: automatic   # Infer from cross-entity routes

  # Or explicit entity order:
  # entity_order:
  #   - Leader      # Runs first (no dependencies)
  #   - Follower1   # Depends on Leader.EOM.position
  #   - Follower2   # Depends on Leader.EOM.position
  #   - KSC         # Observes all, runs last
```

---

## SchedulerConfig Struct

```cpp
namespace icarus {

enum class SchedulingMode {
    Automatic,  // Topological sort from signal dependencies (Future TODO)
    Explicit    // User-defined execution order
};

/**
 * @brief A component member within a scheduler group
 */
struct GroupMember {
    std::string component;     // Component name
    int priority = 0;          // Execution order within group (lower = runs first)
};

/**
 * @brief A scheduler group with its own rate and priority
 *
 * Every component belongs to exactly one group.
 * Groups are sorted by priority (group execution order).
 * Within each group, members are sorted by their own priority.
 */
struct SchedulerGroupConfig {
    std::string name;
    double rate_hz = 400.0;    // Must be integer divisor of simulation rate
    int priority = 0;          // Execution order relative to other groups (lower = runs first)

    // Components in this group
    std::vector<GroupMember> members;
};

struct TopologyConfig {
    // Currently only Explicit is implemented; Automatic is future TODO
    SchedulingMode mode = SchedulingMode::Explicit;

    enum class CycleHandling { Error, Warn, BreakAtDelay };
    CycleHandling cycle_detection = CycleHandling::Error;
    bool log_order = true;
};

/**
 * @brief Per-entity scheduler configuration (group-based)
 *
 * Every component belongs to exactly one group.
 * Groups define rate_hz and priority.
 * Simulation dt is auto-derived from the fastest group across all entities.
 */
template <typename Scalar>
struct SchedulerConfig {
    // All scheduler groups for this entity
    // No "master rate" - each group specifies its own rate
    std::vector<SchedulerGroupConfig> groups;

    // Topology/ordering settings
    TopologyConfig topology;

    // =========================================================================
    // Computed at runtime
    // =========================================================================

    // Frame divisor for each group: sim_rate_hz / group.rate_hz
    // Populated by Scheduler::Configure() after global rate derivation
    std::unordered_map<std::string, int> group_frame_divisors;

    // =========================================================================
    // Factory methods
    // =========================================================================

    static SchedulerConfig FromFile(const std::string& yaml_path);
    static SchedulerConfig Default();  // Single group with all components

    // =========================================================================
    // Validation
    // =========================================================================

    /**
     * @brief Validate configuration (local validation, before global rate is known)
     * @return List of error messages (empty if valid)
     */
    std::vector<std::string> Validate() const {
        std::vector<std::string> errors;

        std::unordered_set<std::string> seen_components;
        std::unordered_set<std::string> seen_group_names;

        for (const auto& group : groups) {
            // Check for duplicate group names
            if (seen_group_names.count(group.name)) {
                errors.push_back("Duplicate group name: " + group.name);
            }
            seen_group_names.insert(group.name);

            // Check for duplicate component assignments
            for (const auto& member : group.members) {
                if (seen_components.count(member.component)) {
                    errors.push_back("Component '" + member.component +
                        "' assigned to multiple groups");
                }
                seen_components.insert(member.component);
            }

            // Check rate is positive
            if (group.rate_hz <= 0) {
                errors.push_back("Group '" + group.name +
                    "' has invalid rate: " + std::to_string(group.rate_hz));
            }
        }

        return errors;
    }

    /**
     * @brief Get the maximum rate across all groups
     */
    double MaxRate() const {
        double max_rate = 0.0;
        for (const auto& group : groups) {
            max_rate = std::max(max_rate, group.rate_hz);
        }
        return max_rate;
    }
};

} // namespace icarus
```

---

## Example: Group-Based Scheduler

```cpp
// Build scheduler config programmatically
SchedulerConfig<double> sched;

// Define groups with their rates and priorities
// Groups sorted by priority, members sorted by their priority

// Sensors group: 400 Hz, runs first (priority 1)
SchedulerGroupConfig sensors;
sensors.name = "sensors";
sensors.rate_hz = 400.0;
sensors.priority = 1;
sensors.members = {
    {"IMU", 1},
    {"GPS", 2}
};
sched.groups.push_back(sensors);

// GNC group: 1600 Hz (fastest, sets simulation dt), runs second
SchedulerGroupConfig gnc;
gnc.name = "gnc";
gnc.rate_hz = 1600.0;
gnc.priority = 2;
gnc.members = {
    {"Guidance", 1},
    {"Autopilot", 2}
};
sched.groups.push_back(gnc);

// Dynamics group: 400 Hz, runs last
SchedulerGroupConfig dynamics;
dynamics.name = "dynamics";
dynamics.rate_hz = 400.0;
dynamics.priority = 3;
dynamics.members = {
    {"Gravity", 1},
    {"Engine", 2},
    {"Forces", 3},
    {"EOM", 4}
};
sched.groups.push_back(dynamics);

// Validate local config
auto errors = sched.Validate();
if (!errors.empty()) {
    // Handle errors...
}

// Simulation rate will be auto-derived as 1600 Hz
// - sensors group: frame_divisor = 4 (runs every 4th frame)
// - gnc group: frame_divisor = 1 (runs every frame)
// - dynamics group: frame_divisor = 4 (runs every 4th frame)
```

---

## Integration with Simulator

The scheduler is configured during `FromConfig()` and used during `Step()`:

```cpp
template <typename Scalar>
class Simulator {
private:
    Scheduler<Scalar> scheduler_;
    double sim_rate_hz_;
    int frame_count_ = 0;

    void ProvisionComponents() {
        // ... create components ...

        // Derive global simulation rate from all entity schedulers
        sim_rate_hz_ = DeriveSimulationRate(entities_);

        // Validate all rates are integer divisors
        auto rate_errors = ValidateGlobalTiming(sim_rate_hz_, entities_);
        if (!rate_errors.empty()) {
            throw std::runtime_error("Invalid scheduler rates: " + rate_errors[0]);
        }

        // Build scheduler from config
        scheduler_.Configure(config_.scheduler, sim_rate_hz_);
    }

    void Step(Scalar dt) {
        Scalar dt_global = Scalar{1.0 / sim_rate_hz_};

        // Execute each entity
        for (auto& entity : entities_) {
            ExecuteEntity(entity, time_, dt_global, frame_count_);
        }

        time_ = time_ + dt_global;
        frame_count_++;
    }

    void ExecuteEntity(Entity& entity, Scalar t, Scalar dt_global, int frame_count) {
        // Sort groups by priority
        auto& groups = entity.scheduler.groups;
        std::sort(groups.begin(), groups.end(),
            [](const auto& a, const auto& b) { return a.priority < b.priority; });

        for (const auto& group : groups) {
            int divisor = static_cast<int>(sim_rate_hz_ / group.rate_hz);

            // Only run if this group's frame aligns
            if (frame_count % divisor == 0) {
                Scalar dt_group = dt_global * Scalar{divisor};

                // Sort members by priority
                auto members = group.members;
                std::sort(members.begin(), members.end(),
                    [](const auto& a, const auto& b) { return a.priority < b.priority; });

                for (const auto& member : members) {
                    auto* comp = GetComponent(entity.name + "." + member.component);
                    comp->PreStep(t, dt_group);
                    comp->Step(t, dt_group);
                    comp->PostStep(t, dt_group);
                }
            }
        }
    }
};
```

---

## Exit Criteria

### Global Timing
- [ ] Auto-derive simulation rate from fastest group rate across all entities
- [ ] Validate all group rates are integer divisors of simulation rate
- [ ] Reject invalid rates (e.g., 513 Hz when sim is 1600 Hz) with clear error message
- [ ] Compute frame divisors for each group: `sim_rate_hz / group.rate_hz`

### Scheduler Core
- [ ] `SchedulerConfig` with group-based model (no master_list)
- [ ] `SchedulerGroupConfig` with rate_hz, priority, and members
- [ ] `GroupMember` with component name and priority
- [ ] Group execution order by priority (lower = first)
- [ ] Member execution order within group by priority
- [ ] Frame divisor logic: group runs when `frame_count % divisor == 0`
- [ ] Each component passes group's dt (not global dt)
- [ ] Validation: no duplicate group names, no component in multiple groups
- [ ] Logging of execution order
- [ ] ~~Automatic topology-based ordering~~ (Future TODO)
