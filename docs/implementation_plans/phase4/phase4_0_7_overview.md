# Phase 4.0.7: Simulator Refactor - Overview

**Status:** Proposed
**Prerequisite:** Phase 4.0.1-4.0.6 complete

---

## Sub-Plans

This plan is split into focused sub-documents:

| Document | Description |
|:---------|:------------|
| [phase4_0_7_overview.md](phase4_0_7_overview.md) | Architecture overview, configuration hierarchy (this file) |
| [phase4_0_7_scheduler.md](phase4_0_7_scheduler.md) | Hierarchical multi-rate scheduler configuration |
| [phase4_0_7_entity_system.md](phase4_0_7_entity_system.md) | Entity templates, instantiation, swarms |
| [phase4_0_7_staging.md](phase4_0_7_staging.md) | Trim optimization, linearization, symbolic generation |
| [phase4_0_7_simulator_api.md](phase4_0_7_simulator_api.md) | Simulator API refactor, lifecycle, problem statement |
| [phase4_0_7_config_structs.md](phase4_0_7_config_structs.md) | All configuration struct definitions |
| [phase4_0_7_implementation.md](phase4_0_7_implementation.md) | Implementation order, exit criteria, migration guide |

---

## Configuration Architecture Overview

The simulation configuration is **entity-centric**: entity templates are self-contained units that define their own components, routes, scheduler, and staging configuration. The simulation level only handles entity instantiation and cross-entity coordination.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         simulation.yaml (Master Binding)                    │
│  Entity instances, cross-entity routes, global coordination, environment   │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         │  entities:                      cross_entity_routes:
         │    - template: rocket.yaml        - Follower1.GNC.leader_pos
         │      name: Leader                   → Leader.EOM.position
         │    - template: rocket.yaml
         │      name: Follower1              coordination:
         │                                     entity_order: [Leader, Follower1]
         │
         ├── Entity Templates (self-contained) ────────────────────────────────┐
         │                                                                     │
         ▼                                                                     ▼
┌─────────────────────────────────────────┐    ┌─────────────────────────────────────────┐
│       entities/rocket.yaml              │    │      entities/quadcopter.yaml           │
│  ┌───────────────────────────────────┐  │    │  ┌───────────────────────────────────┐  │
│  │ components:                       │  │    │  │ components:                       │  │
│  │   - EOM (RigidBody6DOF)           │  │    │  │   - EOM, Motor1..4, IMU, GNC      │  │
│  │   - Gravity, Engine, GNC, ...     │  │    │  └───────────────────────────────────┘  │
│  ├───────────────────────────────────┤  │    │  ┌───────────────────────────────────┐  │
│  │ routes: (internal, relative)      │  │    │  │ routes: (internal, relative)      │  │
│  │   - EOM.force ← Forces.total      │  │    │  │   - Motor*.force → Forces.src*    │  │
│  │   - Forces.src0 ← Gravity.force   │  │    │  └───────────────────────────────────┘  │
│  ├───────────────────────────────────┤  │    │  ┌───────────────────────────────────┐  │
│  │ scheduler: (rate groups)          │  │    │  │ scheduler:                        │  │
│  │   - gnc: 100Hz [GNC]              │  │    │  │   - sensors: 1000Hz [IMU]         │  │
│  │   - dynamics: 100Hz [EOM, ...]    │  │    │  │   - motors: 500Hz [Motor*]        │  │
│  ├───────────────────────────────────┤  │    │  └───────────────────────────────────┘  │
│  │ staging: (trim config)            │  │    │  ┌───────────────────────────────────┐  │
│  │   zero: [EOM.velocity_dot]        │  │    │  │ staging:                          │  │
│  │   controls: [GNC.throttle]        │  │    │  │   zero: [EOM.velocity_dot]        │  │
│  └───────────────────────────────────┘  │    │  │   controls: [GNC.collective]      │  │
└─────────────────────────────────────────┘    │  └───────────────────────────────────┘  │
         │                                     └─────────────────────────────────────────┘
         │
         ├── Component Configs (optional external files) ──────────────────────┐
         │                                                                     │
         ▼                                                                     ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ models/6dof.yaml│  │models/engine.yaml│ │ models/gnc.yaml │  │models/motor.yaml│
│ (component cfg) │  │ (component cfg) │  │ (component cfg) │  │ (component cfg) │
└─────────────────┘  └─────────────────┘  └─────────────────┘  └─────────────────┘
```

---

## Entity vs Component Hierarchy

```
Simulation
├── Entity: Leader (template: Rocket)
│   ├── Component: Leader.EOM
│   ├── Component: Leader.Gravity
│   ├── Component: Leader.Engine
│   ├── Component: Leader.GNC
│   ├── Component: Leader.Mass
│   └── Component: Leader.Forces
│
├── Entity: Follower1 (template: Rocket)
│   ├── Component: Follower1.EOM
│   ├── Component: Follower1.Gravity
│   └── ...
│
├── Entity: KSC (template: GroundStation)
│   ├── Component: KSC.Position
│   └── Component: KSC.Tracker
│
└── Swarm: Drone_000..Drone_099 (template: Quadcopter)
    ├── Entity: Drone_000
    │   ├── Component: Drone_000.EOM
    │   └── ...
    └── Entity: Drone_099
        └── ...
```

---

## Configuration Files

| File | Purpose | Location |
|:-----|:--------|:---------|
| `simulation.yaml` | Master binding: entities, cross-entity routes, coordination | **Simulation level** |
| `entities/<name>.yaml` | Entity template: components, routes, scheduler, staging | **Per-entity** |
| `models/<component>.yaml` | Component-specific parameters | **Per-component** |

---

## What's Per-Entity vs Global

| Configuration | Per-Entity Template | Simulation Level |
|:--------------|:--------------------|:-----------------|
| Components | ✅ Defined in template | - |
| Routes (internal) | ✅ Relative names | - |
| Routes (cross-entity) | - | ✅ Full paths |
| Scheduler (rate groups) | ✅ Within entity | - |
| Scheduler (entity order) | - | ✅ Which entity first |
| Staging (trim) | ✅ How to trim this entity | Override tolerances |
| Staging (linearization) | ✅ Per-entity | - |
| Environment | - | ✅ Global selection |
| Integrator | - | ✅ Global |
| Logging | - | ✅ Global |

---

## Two-Layer Architecture

The system maintains two views:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              User View (YAML)                               │
│                         Object-Oriented Entities                            │
├─────────────────────────────────────────────────────────────────────────────┤
│  Entity "Rocket"                    Entity "GroundStation"                  │
│  ├── Components                     ├── Components                          │
│  ├── Internal Routes                ├── Internal Routes                     │
│  ├── Internal Scheduler             └── Internal Scheduler                  │
│  └── Internal Staging                                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                            Simulation Level                                 │
│  ├── Entity Instances (Leader, Follower1, KSC, ...)                        │
│  ├── Cross-Entity Routes                                                   │
│  └── Global Coordination (entity execution order)                          │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼  Flatten / Expand
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Internal View (C++)                               │
│                    Flat List of Components + Signals                        │
├─────────────────────────────────────────────────────────────────────────────┤
│  Components: Leader.EOM, Leader.Gravity, Follower1.EOM, KSC.Tracker, ...   │
│  Signals: Leader.EOM.position, Leader.EOM.velocity, ...                    │
│  Routes: All internal + cross-entity routes with full paths                │
│  Scheduler: Unified schedule from all entity schedulers                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Key Insight:** The user-facing YAML is object-oriented (entities contain components), while the internal C++ backend flattens everything to components + signals on a shared backplane.

---

## Design Rationale

| Aspect | Per-Entity | Global | Winner |
|:-------|:-----------|:-------|:-------|
| Template reusability | ✅ Self-contained | ❌ Needs separate files | Per-Entity |
| Multi-vehicle trim | ✅ Each vehicle trims independently | ❌ One trim for all? | Per-Entity |
| Rate group isolation | ✅ Entity has its own rates | ❌ All rates global | Per-Entity |
| Cross-entity deps | Handled at sim level | ✅ Natural | Global |
| Simplicity for single-vehicle | More files | ✅ All in one place | Global |

**Conclusion:** Per-entity for templates, global only for cross-entity coordination.

---

## Next Steps

See the sub-plans for detailed specifications:

1. **[Scheduler](phase4_0_7_scheduler.md)** - Hierarchical multi-rate execution model
2. **[Entity System](phase4_0_7_entity_system.md)** - Templates, instantiation, swarms
3. **[Staging](phase4_0_7_staging.md)** - Trim, linearization, symbolics
4. **[Simulator API](phase4_0_7_simulator_api.md)** - Clean 4-operation interface
5. **[Config Structs](phase4_0_7_config_structs.md)** - C++ configuration definitions
6. **[Implementation](phase4_0_7_implementation.md)** - Task order and exit criteria
