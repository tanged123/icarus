# Icarus Implementation Plan

**Status:** Proposed  
**Target:** Core simulation framework with symbolic compatibility

---

## Overview

This plan builds Icarus vertically through 7 phases, validating symbolic mode early (Phase 3) before expanding horizontally. Each phase has clear exit criteria and references to architectural documentation.

---

## Phase 1: Foundation

**Goal:** Minimal working skeleton that compiles and tests.

### Components

| Item | Description | Files |
|:-----|:------------|:------|
| Types & Concepts | `Scalar` template, `SignalType` enum, `Lifecycle` enum, basic type traits | `include/icarus/Types.hpp` |
| Signal & Backplane | Registry, storage vectors, `register_output()`, `resolve()`, type checking | `include/icarus/Signal.hpp`, `include/icarus/Backplane.hpp` |
| Component Base | Abstract `Component<Scalar>` with `Provision()`, `Stage()`, `Step()` signatures | `include/icarus/Component.hpp` |
| Simulator Shell | Owns component list and backplane, calls lifecycle methods in sequence | `include/icarus/Simulator.hpp` |

### Architecture References

- [01_core_philosophy.md](../architecture/01_core_philosophy.md) — Terminology, flat topology
- [02_component_protocol.md](../architecture/02_component_protocol.md) — Component interface contract
- [03_signal_backplane.md](../architecture/03_signal_backplane.md) — Signal registry, type-safe access
- [04_lifecycle.md](../architecture/04_lifecycle.md) — Provision/Stage/Step semantics

### Exit Criteria

- [ ] Can instantiate `Simulator<double>`
- [ ] Add a dummy component
- [ ] Call Provision/Stage/Step
- [ ] Read/write signals via Backplane

---

## Phase 2: State & Integration

**Goal:** Actual differential equations work.

### Components

| Item | Description | Files |
|:-----|:------------|:------|
| StateVector | Global `X_global_`, `X_dot_global_`, scatter/gather via pointer binding | `include/icarus/StateVector.hpp` |
| Integrator Interface | Abstract integrator, RK4 implementation | `include/icarus/integrators/` |
| First Real Component | `PointMass3DOF` with position/velocity state, gravity input | `include/icarus/components/PointMass3DOF.hpp` |

### Architecture References

- [09_memory_state_ownership.md](../architecture/09_memory_state_ownership.md) — Global state, scatter/gather, ODE interface
- [04_lifecycle.md](../architecture/04_lifecycle.md) — State binding in Stage phase

### Exit Criteria

- [ ] Simulate a falling object (point mass under gravity)
- [ ] Verify against analytical solution: `y(t) = y₀ + v₀t - ½gt²`
- [ ] State correctly scattered/gathered through integrator

---

## Phase 3: Symbolic Mode

**Goal:** Same code runs with `casadi::MX`.

### Components

| Item | Description | Files |
|:-----|:------------|:------|
| Dual-Mode Validation | Instantiate `Simulator<MX>`, ensure compilation | `tests/symbolic/` |
| Graph Export | `GenerateGraph()` returns `casadi::Function` | `include/icarus/Simulator.hpp` |
| Symbolic Test Suite | Verify `PointMass3DOF` traces correctly | `tests/symbolic/test_point_mass_symbolic.cpp` |

### Architecture References

- [07_janus_integration.md](../architecture/07_janus_integration.md) — Template-first paradigm, dual backend
- [21_symbolic_constraints.md](../architecture/21_symbolic_constraints.md) — `janus::where()`, `janus::` math, loop rules

### Exit Criteria

- [ ] Extract symbolic dynamics as `casadi::Function`
- [ ] Evaluate numerically, match `Simulator<double>` output
- [ ] No `std::` math or `if/else` on `Scalar` in codebase

> [!IMPORTANT]
> **Do this early!** Symbolic validation in Phase 3 catches template violations before you have 20 components to fix.

---

## Phase 4: Aggregation & 6DOF

**Goal:** Multi-component force/moment aggregation.

### Components

| Item | Description | Files |
|:-----|:------------|:------|
| Force/Mass Registration | `register_force_source()`, `register_mass_source()` with frame metadata | `include/icarus/Backplane.hpp` |
| Aggregators | `ForceAggregator`, `MassAggregator` components | `include/icarus/components/aggregators/` |
| RigidBody6DOF | Full rotational dynamics using Vulcan EOM utilities | `include/icarus/components/RigidBody6DOF.hpp` |

### Architecture References

- [12_quantity_aggregation.md](../architecture/12_quantity_aggregation.md) — Registration pattern, frame transforms, aggregator logic
- [08_vulcan_integration.md](../architecture/08_vulcan_integration.md) — Vulcan EOM utilities

### Exit Criteria

- [ ] Tumbling rigid body with multiple force sources
- [ ] Correct angular momentum conservation
- [ ] Force frame transforms (wind, body, inertial) working

---

## Phase 5: Configuration & Wiring

**Goal:** YAML-driven simulation setup.

### Components

| Item | Description | Files |
|:-----|:------------|:------|
| Config Loader | Parse component definitions, entity bundles, wiring | `include/icarus/config/` |
| Scheduler | Topological sort from dependency graph, rate groups | `include/icarus/Scheduler.hpp` |
| Data Dictionary | Auto-generate signal catalog at Provision | `include/icarus/DataDictionary.hpp` |

### Architecture References

- [13_configuration.md](../architecture/13_configuration.md) — Configuration layers A–F, entity definitions
- [05_execution_model.md](../architecture/05_execution_model.md) — Scheduler, rate groups, multi-rate sync
- [06_entities_namespaces.md](../architecture/06_entities_namespaces.md) — Entity as namespace prefix

### Exit Criteria

- [ ] Load `scenarios/rocket_launch.yaml`
- [ ] Run simulation without hardcoded component setup
- [ ] Data Dictionary exported as JSON/YAML

---

## Phase 6: Events & Recording

**Goal:** Complete simulation lifecycle.

### Components

| Item | Description | Files |
|:-----|:------------|:------|
| Phase Manager | Condition evaluation, phase transitions, ghosting | `include/icarus/PhaseManager.hpp` |
| Recording | `.icarec` writer with schema versioning | `include/icarus/recording/` |
| Warmstart | Load state from recording, continue simulation | `include/icarus/Simulator.hpp` |

### Architecture References

- [17_events_phases.md](../architecture/17_events_phases.md) — Event evaluation, ghosting, derivative gating
- [20_recording.md](../architecture/20_recording.md) — Recording format, schema versioning, warmstart
- [10_entity_lifecycle.md](../architecture/10_entity_lifecycle.md) — Entity birth/death, stage separation

### Exit Criteria

- [ ] Multi-phase rocket simulation with stage separation
- [ ] Full `.icarec` recording with schema
- [ ] Warmstart from mid-flight recording

---

## Phase 7: Trim & External Interfaces

**Goal:** Production-ready entry points.

### Components

| Item | Description | Files |
|:-----|:------------|:------|
| Trim Solver | Symbolic NLP formulation, IPOPT integration | `include/icarus/trim/` |
| C API | `icarus_create()`, `icarus_step()`, `icarus_get_signal()` | `include/icarus/icarus.h` |
| Python Bindings | PyBind11 wrapper | `interfaces/python/` |

### Architecture References

- [14_trim_optimization.md](../architecture/14_trim_optimization.md) — Trim solver, equilibrium, NLP formulation
- [16_external_bindings.md](../architecture/16_external_bindings.md) — C API, Python, MATLAB bindings
- [15_services.md](../architecture/15_services.md) — Telemetry, recording service integration

### Exit Criteria

- [ ] Python script runs trimmed simulation
- [ ] Export results to pandas DataFrame
- [ ] C API usable from external FSW code

---

## Supplementary References

| Topic | Architecture Doc |
|:------|:-----------------|
| Error handling | [18_error_handling.md](../architecture/18_error_handling.md) |
| Determinism | [19_determinism_parallelism.md](../architecture/19_determinism_parallelism.md) |
| Testing patterns | [22_testing.md](../architecture/22_testing.md) |
| External data/tables | [23_external_data.md](../architecture/23_external_data.md) |

---

## Quick Start

**Day 1:**
1. `include/icarus/Types.hpp` + `include/icarus/Signal.hpp`
2. First unit test: signal registration and retrieval
3. Build vertically through Phase 1 → 2 → 3 with one trivial component
4. Symbolic mode validation in Phase 3 before expanding horizontally

```
Phase 1 (Foundation)
    ↓
Phase 2 (State & Integration)
    ↓
Phase 3 (Symbolic Mode)     ← VALIDATE HERE before expanding
    ↓
Phase 4+ (horizontal expansion)
```
