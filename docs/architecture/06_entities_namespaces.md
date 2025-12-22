# Entities as Namespaces

**Related:** [01_core_philosophy.md](01_core_philosophy.md) | [03_signal_backplane.md](03_signal_backplane.md) | [13_configuration.md](13_configuration.md)

---

If the simulation is flat, how do we handle multiple vehicles? Or a "Rocket" with "Stages"?

**Entities are Virtual.** They are nothing more than a standardized **Namespace Prefix** in the Signal Backplane.

---

## Example: Multi-Stage Rocket

We don't have a `class Rocket` containing `class Stage`.
We have a config file that generates components with prefixes:

1. `Falcon.Stage1.Engine.Thrust`
2. `Falcon.Stage2.Engine.Thrust`
3. `Falcon.Interstage.SeparationLogic.Active`

The "Separation" logic is just a component that monitors `Stage1` signals and affects `Stage2` signals. It doesn't need to "own" them.

---

## Namespace Convention

```
<Entity>.<Component>.<Signal>
```

Examples:
- `Falcon9.Stage1.Engine.Thrust`
- `Falcon9.Stage2.Engine.Thrust`
- `Falcon9.Interstage.SeparationLogic.Active`
- `Environment.Atmosphere.Density`
- `Environment.Gravity.Acceleration`

---

## Benefits

| Benefit | Description |
|:--------|:------------|
| **No pointer surgery** | Moving a component is just a string rename |
| **Flat registry** | All signals queryable with glob patterns |
| **Implicit grouping** | `Falcon9.*` queries all signals for that entity |
| **No ownership hierarchy** | Components can read/write any signal (by permission) |

---

## Entity Definition Files

While entities are virtual at runtime, they can be defined in configuration for human ergonomics:

```yaml
# entities/falcon9.yaml
entity:
  name: Falcon9

  components:
    - type: RigidBody6DOF
      name: Stage1.EOM
    - type: MerlinEngine
      name: Stage1.Engine
    - type: RigidBody6DOF
      name: Stage2.EOM
    - type: VacuumEngine
      name: Stage2.Engine
    - type: SeparationLogic
      name: Interstage.Separation
```

At load time, this is flattened into individual components with namespaced signals.

See [13_configuration.md](13_configuration.md) for full entity definition syntax.
