# Icarus Data-Oriented Architecture (IDOA)

**Status:** Draft / Proposed
**Supersedes:** Hierarchical "Scene Graph" approaches
**Context:** Icarus 6DOF Simulation Engine

## Document Library

This architecture documentation is organized into focused modules for easy navigation.

### Foundation

| Document | Description |
|:---------|:------------|
| [01_core_philosophy.md](01_core_philosophy.md) | The "flat" simulation philosophy and terminology |
| [02_component_protocol.md](02_component_protocol.md) | Register Outputs / Wire Inputs pattern, Data Dictionary |
| [03_signal_backplane.md](03_signal_backplane.md) | Signal registry, storage, type-safe access |
| [04_lifecycle.md](04_lifecycle.md) | Provision / Stage / Step phases and hooks |

### Execution

| Document | Description |
|:---------|:------------|
| [05_execution_model.md](05_execution_model.md) | Scheduler, rate groups, synchronization |
| [06_entities_namespaces.md](06_entities_namespaces.md) | Virtual entities as namespace prefixes |

### Integration

| Document | Description |
|:---------|:------------|
| [07_janus_integration.md](07_janus_integration.md) | Template-first paradigm, dual backend |
| [08_vulcan_integration.md](08_vulcan_integration.md) | Physics utilities, "bricks vs houses" |
| [21_symbolic_constraints.md](21_symbolic_constraints.md) | Janus compatibility rules for components |

### Memory & State

| Document | Description |
|:---------|:------------|
| [09_memory_state_ownership.md](09_memory_state_ownership.md) | Global state vector, scatter/gather, ODE interface |
| [10_entity_lifecycle.md](10_entity_lifecycle.md) | Entity birth, death, separation, state inheritance |
| [12_force_aggregation.md](12_force_aggregation.md) | Force/moment aggregation pattern |

### Configuration

| Document | Description |
|:---------|:------------|
| [13_configuration.md](13_configuration.md) | Configuration layers A-D, entity definitions |
| [14_trim_optimization.md](14_trim_optimization.md) | Trim solver, equilibrium, NLP formulation |
| [15_services.md](15_services.md) | Recording, telemetry, tooling integration |

### Interfaces

| Document | Description |
|:---------|:------------|
| [16_external_bindings.md](16_external_bindings.md) | C API, Python, MATLAB bindings |
| [17_events_phases.md](17_events_phases.md) | Flight phases, event handling, ghosting |

### Operations

| Document | Description |
|:---------|:------------|
| [18_error_handling.md](18_error_handling.md) | Logging, debug mode, tracing |
| [19_determinism_parallelism.md](19_determinism_parallelism.md) | Determinism guarantees, threading strategy |
| [20_recording.md](20_recording.md) | Recording format, schema versioning, warmstart |
| [22_testing.md](22_testing.md) | Unit testing, symbolic mode testing |

### Summary

| Document | Description |
|:---------|:------------|
| [11_summary_benefits.md](11_summary_benefits.md) | Architecture benefits overview |

---

## Quick Reference

### The Defining Rule

> **"The Gravity Model, the Atmosphere, the Spacecraft, and the Fuel Pump are all structural peers."**

There is no "World" object that *contains* the vehicle. There is a simulation that contains a list of components, some of which calculate gravity, and some of which calculate fuel flow.

### Core Lifecycle

```
PROVISION (once)     →  Allocate memory, register signals
    ↓
STAGE (per run)      →  Wire inputs, apply ICs, run trim
    ↓
STEP (per Δt)        →  Read inputs, compute, write outputs
```

### Key Relationships

- An **Entity** (Falcon9) is just a label (namespace prefix)
- A **Component** (EngineComponent) *belongs to* an Entity (conceptually)
- A **Component** *has* **State** (RPM)
- A **Component** *uses* a **Model** (Thermodynamics) to compute derivatives
- A **Component** *reads/writes* **Signals** via the **Backplane**

### Document Dependencies

```
01_core_philosophy
    ├── 02_component_protocol
    │       └── 03_signal_backplane
    ├── 04_lifecycle
    │       └── 05_execution_model
    ├── 06_entities_namespaces
    │       └── 10_entity_lifecycle
    │               └── 17_events_phases
    ├── 07_janus_integration
    │       └── 21_symbolic_constraints
    └── 13_configuration
            └── 14_trim_optimization
```

---

## Related Documents

- `janus/docs/janus_usage_guide.md` - Janus math library usage
- `vulcan/docs/` - Physics utility library documentation
- `design/signal_system_analysis.md` - Signal system design rationale
