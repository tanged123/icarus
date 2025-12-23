# Configuration Architecture

**Related:** [02_component_protocol.md](02_component_protocol.md) | [06_entities_namespaces.md](06_entities_namespaces.md) | [14_trim_optimization.md](14_trim_optimization.md)

---

All runtime behavior is driven by **Configuration Files** (YAML/JSON). The simulation code is generic; the config makes it specific.

---

## 1. Configuration Layers

| Layer | Scope | Example File | Contents |
| :--- | :--- | :--- | :--- |
| **A. Component Params** | Per-component type | `components/jet_engine.yaml` | Defaults, constants, flags, table paths |
| **A'. Entity Definition** | Per-entity type | `entities/x15_vehicle.yaml` | Component bundles, internal wiring, exposed ports |
| **B. World Setup** | Simulation-wide | `scenarios/x15_mission.yaml` | Entity instances, initial conditions |
| **C. Backplane Wiring** | Data routing | (embedded in B or separate) | Inter-entity port mappings |
| **D. Scheduler** | Execution order | (embedded in B or separate) | Rate groups, ordering policy |
| **E. Trim/Optimization** | Stage behavior | `trim/steady_cruise.yaml` | Free vars, targets, solver settings |
| **F. Services** | Logging, I/O | `services.yaml` | Recording, telemetry, bindings |

---

## 2. Layer A: Component Parameters

Each component type can have a **parameter schema**. Instances override defaults.

```yaml
# components/jet_engine.yaml (Component Type Definition)
type: JetEngine
defaults:
  max_thrust: 50000.0      # N
  time_constant: 0.5       # s
  use_afterburner: 0       # 0 - false, 1 - true
tables:
  fuel_flow_map: "data/engine/fuel_flow.csv"
```

---

## 3. Layer A': Entity Definition (Hierarchical Abstraction)

While the runtime is **flat**, humans configure in **hierarchies**. An Entity Definition bundles components into a reusable unit.

```yaml
# entities/x15_vehicle.yaml
entity:
  name: X15
  description: "North American X-15 Research Aircraft"

  # What components make up this entity?
  components:
    - type: RigidBody6DOF
      name: EOM
    - type: JetEngine
      name: MainEngine
      params:
        max_thrust: 75000.0
    - type: AeroBody
      name: Aero
      params:
        reference_area: 18.6  # m^2

  # Internal wiring (within this entity, uses source→target)
  internal_wiring:
    - source: Aero.lift
      target: ForceAggregator.input_aero_lift
    - source: MainEngine.thrust
      target: ForceAggregator.input_propulsion_thrust
    - source: ForceAggregator.total_force
      target: EOM.input_force

  # Exposed ports (for external wiring)
  ports:
    inputs:
      - name: atm_density
        binds_to: Aero.input_density
      - name: gravity
        binds_to: EOM.input_gravity
    outputs:
      - name: position
        binds_to: EOM.output_position
      - name: velocity
        binds_to: EOM.output_velocity
```

**At Load Time:** The entity is "flattened" into the global backplane:
- `X15.EOM.output_position` → `Vehicle.X15.EOM.Position`
- Internal wiring is resolved to direct signal bindings.
- External ports become the entity's public interface.

**Benefits:**
- Develop/test entities in isolation.
- Reuse entity definitions across scenarios.
- Hide internal complexity from scenario authors.

> [!IMPORTANT]
> **Design Decision: Wiring Philosophy (Explicit Source→Target)**
> - All wiring uses explicit `source: X, target: Y` mappings—no auto-wiring or conventions.
> - **Internal wiring** lives in the Entity Definition (encapsulated).
> - **External wiring** (inter-entity) lives in the Scenario/World Setup (Layer C).
> - **No inline transforms.** If you need unit conversion or signal manipulation, add an explicit Converter Component. This keeps the Backplane pure and all logic traceable.

---

## 4. Layer B: World Setup (Scenario Definition)

Defines WHAT exists in the simulation.

```yaml
# scenarios/x15_mission.yaml
scenario:
  name: "X-15 High Altitude Test"

entities:
  - name: X15
    components:
      - type: RigidBody6DOF
        name: EOM
      - type: JetEngine
        name: MainEngine
      - type: AeroBody
        name: Aero

  - name: Environment
    components:
      - type: USA76Atmosphere
        name: Atmosphere
      - type: WGS84Gravity
        name: Gravity

initial_conditions:
  X15.EOM.position: [0, 0, 15000]  # m
  X15.EOM.velocity: [200, 0, 0]   # m/s
```

---

## 5. Layer C: Backplane Wiring

Defines HOW data flows between components using **explicit source→target mappings**.

```yaml
wiring:
  # External wiring: connect entities to each other
  - source: Environment.Atmosphere.Density
    target: X15.Aero.input_density
  - source: Environment.Atmosphere.Temperature
    target: X15.Aero.input_temperature
  - source: Environment.Gravity.acceleration
    target: X15.EOM.input_gravity

  # Inter-component wiring within an entity
  - source: X15.Aero.output_lift
    target: X15.ForceAggregator.input_aero_lift
  - source: X15.MainEngine.output_thrust
    target: X15.ForceAggregator.input_propulsion_thrust
```

> [!IMPORTANT]
> **Wiring Philosophy: Explicit Source→Target.**
> Every signal connection is declared explicitly. This makes data flow traceable, debuggable, and self-documenting. No "magic" auto-wiring—if it's not in the config, it's not connected.

---

## 6. Layer D: Scheduler Configuration

Defines WHEN and in what ORDER components execute.

```yaml
scheduler:
  policy: TOPOLOGICAL  # or MANUAL, RATE_GROUPED

  rate_groups:
    - rate_hz: 1000
      components: [X15.EOM, X15.Aero]
    - rate_hz: 100
      components: [X15.GNC, X15.Autopilot]
    - rate_hz: 10
      components: [Telemetry, Logger]

  # Manual ordering within a rate group (optional)
  execution_order:
    - Environment.Atmosphere
    - Environment.Gravity
    - X15.Aero
    - X15.EOM
```

---

## 7. Configuration Loading Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    Scenario Config                          │
│                (scenarios/x15_mission.yaml)                 │
│                      (Layer B)                              │
└─────────────────────────┬───────────────────────────────────┘
                          │ references
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ Entity Defs     │ │ Trim Config     │ │ Services Config │
│ (Layer A')      │ │ (Layer E)       │ │ (Layer F)       │
│                 │ │                 │ │                 │
│ x15_vehicle.yaml│ │ steady_cruise.  │ │ services.yaml   │
└────────┬────────┘ └─────────────────┘ └─────────────────┘
         │ references
         ▼
┌─────────────────┐
│ Component Defs  │
│ (Layer A)       │
│                 │
│ rocket.yaml     │
│ aero_body.yaml  │
└─────────────────┘
         │
         │ All configs loaded
         ▼
┌─────────────────────────────────────────────────────────────┐
│                     Simulator.Provision()                   │
│  • Instantiate components from Entity Defs                  │
│  • Load tables/assets                                       │
│  • Register all signals on Backplane                        │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                      Simulator.Stage()                      │
│  • Resolve Wiring (Layer C) - internal + external           │
│  • Configure Scheduler (Layer D)                            │
│  • Run Trim/Optimization (Layer E)                          │
│  • Bind signal pointers                                     │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    Simulator.Step() loop                    │
│  • Execute components per Scheduler                         │
│  • Services (Layer F) active: logging, telemetry, recording │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. Configuration Debugging

With 6 configuration layers, tracing "why is my parameter this value?" can be challenging. Icarus provides introspection tools:

### 8.1 Config Flatten

The `icarus config flatten` command outputs a single resolved YAML showing all values and their sources:

```bash
$ icarus config flatten scenarios/x15_mission.yaml

# Output: Resolved configuration with provenance
X15.MainEngine.max_thrust: 75000.0
  # Source: entities/x15_vehicle.yaml:12 (overrides default 50000.0)

X15.Aero.reference_area: 18.6
  # Source: entities/x15_vehicle.yaml:16

Environment.Atmosphere.type: USA76
  # Source: scenarios/x15_mission.yaml:18 (default)
```

### 8.2 Wiring Graph

```bash
$ icarus config graph scenarios/x15_mission.yaml --output wiring.dot

# Generates a DOT graph showing all signal connections
# Visualize with: dot -Tpng wiring.dot -o wiring.png
```

### 8.3 Validation

```bash
$ icarus config validate scenarios/x15_mission.yaml

# Checks for:
# - Missing required parameters
# - Unresolved signal references
# - Type mismatches
# - Circular wiring dependencies
```

> [!TIP]
> Run `icarus config flatten` first when debugging unexpected behavior. It shows exactly which layer set each value.

---

## 9. See Also

- [14_trim_optimization.md](14_trim_optimization.md) - Layer E: Trim solver configuration
- [15_services.md](15_services.md) - Layer F: Services configuration
