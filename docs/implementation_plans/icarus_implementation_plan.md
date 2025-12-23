# Icarus Implementation Plan

**Status:** Proposed  
**Target:** Core simulation framework with symbolic compatibility

---

## Overview

This plan builds Icarus vertically through 7 implementation phases, validating symbolic mode early (Phase 3) before expanding horizontally. Each phase has clear exit criteria and references to architectural documentation.

> [!IMPORTANT]
> **Build vertically first.** Complete Phases 1→2→3 with one trivial component before expanding horizontally to aggregation and config. Symbolic validation in Phase 3 catches template violations early.

```
Phase 1 (Foundation)
    ↓
Phase 2 (State & Integration)
    ↓
Phase 3 (Symbolic Mode)     ← VALIDATE HERE before expanding
    ↓
Phase 4–7 (horizontal expansion)
```

---

## Phase 1: Foundation

**Goal:** Minimal working skeleton that compiles and tests.

### Tasks

- [x] **1.1 Repository Bootstrap**
  - [x] Set up repository structure (see [Repository Structure](../icarus_bootstrap_guide.md#4-repository-structure))
  - [x] Configure Nix flake with Janus + Vulcan dependencies
  - [x] Configure CMake build system
  - [x] Set up CI/CD workflows
  - [x] Create agent rules (`.cursorrules`, `CLAUDE.md`)

- [x] **1.2 Types & Concepts**
  - [x] `include/icarus/core/Types.hpp` — Scalar template, lifecycle enum
  - [x] `include/icarus/core/Concepts.hpp` — JanusScalar constraints
  - [x] Basic error handling (`include/icarus/core/Error.hpp`)

- [ ] **1.3 Signal Backplane**
  - [ ] `include/icarus/signal/Signal.hpp` — SignalType enum, lifecycle
  - [ ] `include/icarus/signal/Registry.hpp` — `register_output()`, `resolve()`
  - [ ] Type-safe signal access

- [ ] **1.4 Component Base**
  - [ ] Abstract `Component<Scalar>` with `Provision()`, `Stage()`, `Step()`
  - [ ] Optional hooks (`PreStep`, `PostStep`, `OnPhaseEnter`, etc.)

- [ ] **1.5 Simulator Shell**
  - [ ] Top-level `Simulator<Scalar>` class
  - [ ] Owns component list and backplane
  - [ ] Calls lifecycle methods in sequence

### Architecture References

| Topic | Document |
|:------|:---------|
| Flat topology, terminology | [01_core_philosophy.md](../architecture/01_core_philosophy.md) |
| Component interface | [02_component_protocol.md](../architecture/02_component_protocol.md) |
| Signal registry | [03_signal_backplane.md](../architecture/03_signal_backplane.md) |
| Lifecycle phases | [04_lifecycle.md](../architecture/04_lifecycle.md) |

### Exit Criteria

- [ ] Can instantiate `Simulator<double>`, add a dummy component
- [ ] Call Provision/Stage/Step without crashes
- [ ] Read/write signals via Backplane
- [ ] All Phase 1 tests pass

---

## Phase 2: State & Integration

**Goal:** Actual differential equations work.

### Tasks

- [ ] **2.1 State Management**
  - [ ] Global `X_global_`, `X_dot_global_` vectors
  - [ ] State registration and ownership
  - [ ] Scatter/gather via pointer binding to components

- [ ] **2.2 Integrator Interface**
  - [ ] Abstract `Integrator<Scalar>` interface
  - [ ] RK4 implementation
  - [ ] Adaptive RK45 implementation

- [ ] **2.3 First Real Component**
  - [ ] `PointMass3DOF` with position/velocity state
  - [ ] Gravity input, integrated dynamics

### Architecture References

| Topic | Document |
|:------|:---------|
| Global state vector | [09_memory_state_ownership.md](../architecture/09_memory_state_ownership.md) |
| State binding in Stage | [04_lifecycle.md](../architecture/04_lifecycle.md) |

### Exit Criteria

- [ ] Simulate a falling object (point mass under gravity)
- [ ] Verify against analytical solution: `y(t) = y₀ + v₀t - ½gt²`
- [ ] State correctly scattered/gathered through integrator

---

## Phase 3: Symbolic Mode

**Goal:** Same code runs with `casadi::MX`.

### Tasks

- [ ] **3.1 Dual-Mode Validation**
  - [ ] Instantiate `Simulator<casadi::MX>`, ensure compilation
  - [ ] No `std::` math or `if/else` on Scalar in codebase

- [ ] **3.2 Graph Export**
  - [ ] `GenerateGraph()` returns `casadi::Function`
  - [ ] Derivatives extractable via AD

- [ ] **3.3 Symbolic Test Suite**
  - [ ] Verify `PointMass3DOF` traces correctly
  - [ ] Numeric/symbolic output comparison tests

### Architecture References

| Topic | Document |
|:------|:---------|
| Template-first paradigm | [07_janus_integration.md](../architecture/07_janus_integration.md) |
| `janus::where()`, math rules | [21_symbolic_constraints.md](../architecture/21_symbolic_constraints.md) |

### Exit Criteria

- [ ] Extract symbolic dynamics as `casadi::Function`
- [ ] Evaluate numerically, match `Simulator<double>` output
- [ ] Zero template violations in codebase

---

## Phase 4: Aggregation & 6DOF

**Goal:** Multi-component force/moment aggregation.

### Tasks

- [ ] **4.1 Force/Mass Registration**
  - [ ] `register_force_source()`, `register_mass_source()` with frame metadata
  - [ ] Query sources by entity prefix

- [ ] **4.2 Aggregators**
  - [ ] `ForceAggregator` — frame transforms, moment transfer
  - [ ] `MassAggregator` — total mass, CG, inertia tensor

- [ ] **4.3 RigidBody6DOF**
  - [ ] Full rotational dynamics using Vulcan EOM utilities
  - [ ] Consumes aggregated force/moment/inertia

### Architecture References

| Topic | Document |
|:------|:---------|
| Aggregation pattern | [12_quantity_aggregation.md](../architecture/12_quantity_aggregation.md) |
| Vulcan utilities | [08_vulcan_integration.md](../architecture/08_vulcan_integration.md) |

### Exit Criteria

- [ ] Tumbling rigid body with multiple force sources
- [ ] Correct angular momentum conservation
- [ ] Force frame transforms working

---

## Phase 5: Configuration & Scheduling

**Goal:** YAML-driven simulation setup.

### Tasks

- [ ] **5.1 Configuration Loader**
  - [ ] Parse component definitions (Layer A)
  - [ ] Entity bundles (Layer A')
  - [ ] Scenario definitions (Layer B)
  - [ ] Wiring (Layer C)

- [ ] **5.2 Scheduler**
  - [ ] Topological sort from dependency graph
  - [ ] Rate groups for multi-rate simulation (Layer D)

- [ ] **5.3 Data Dictionary**
  - [ ] Auto-generate signal catalog at Provision
  - [ ] Export as JSON/YAML

### Architecture References

| Topic | Document |
|:------|:---------|
| Configuration layers | [13_configuration.md](../architecture/13_configuration.md) |
| Scheduler, rate groups | [05_execution_model.md](../architecture/05_execution_model.md) |
| Entity namespaces | [06_entities_namespaces.md](../architecture/06_entities_namespaces.md) |

### Exit Criteria

- [ ] Load `scenarios/rocket_launch.yaml`
- [ ] Run simulation without hardcoded component setup
- [ ] Data Dictionary exported

---

## Phase 6: Events, Recording & Services

**Goal:** Complete simulation lifecycle.

### Tasks

- [ ] **6.1 Phase Manager**
  - [ ] Condition evaluation (post-Step signal values)
  - [ ] Phase transitions, ghosting, derivative gating
  - [ ] `OnPhaseEnter`/`OnPhaseExit` hooks

- [ ] **6.2 Recording**
  - [ ] HDF5-based `.icarec` writer
  - [ ] Schema versioning
  - [ ] Time system specification (MET/TAI/UTC)

- [ ] **6.3 Warmstart**
  - [ ] Load state from recording
  - [ ] Continue simulation from mid-flight

- [ ] **6.4 Services**
  - [ ] Structured logging (spdlog)
  - [ ] Telemetry service
  - [ ] Debug mode support

### Architecture References

| Topic | Document |
|:------|:---------|
| Events and phases | [17_events_phases.md](../architecture/17_events_phases.md) |
| Recording format | [20_recording.md](../architecture/20_recording.md) |
| Entity lifecycle | [10_entity_lifecycle.md](../architecture/10_entity_lifecycle.md) |
| Error handling | [18_error_handling.md](../architecture/18_error_handling.md) |
| Services | [15_services.md](../architecture/15_services.md) |

### Exit Criteria

- [ ] Multi-phase rocket simulation with stage separation
- [ ] Full `.icarec` recording with schema
- [ ] Warmstart from mid-flight recording

---

## Phase 7: Trim & External Interfaces

**Goal:** Production-ready entry points.

### Tasks

- [ ] **7.1 Trim Solver**
  - [ ] Symbolic NLP formulation
  - [ ] Integration with `janus::Opti` / IPOPT

- [ ] **7.2 C API**
  - [ ] `icarus_create()`, `icarus_step()`, `icarus_get_signal()`
  - [ ] FFI-compatible types

- [ ] **7.3 Python Bindings**
  - [ ] PyBind11 wrapper
  - [ ] `import icarus` workflow

- [ ] **7.4 MATLAB Bindings** (optional)
  - [ ] MEX interface or Python bridge

### Architecture References

| Topic | Document |
|:------|:---------|
| Trim solver | [14_trim_optimization.md](../architecture/14_trim_optimization.md) |
| External bindings | [16_external_bindings.md](../architecture/16_external_bindings.md) |

### Exit Criteria

- [ ] Python script runs trimmed simulation
- [ ] Export results to pandas DataFrame
- [ ] C API usable from external FSW code

---

## Supplementary References

| Topic | Architecture Doc |
|:------|:-----------------|
| Determinism | [19_determinism_parallelism.md](../architecture/19_determinism_parallelism.md) |
| Testing patterns | [22_testing.md](../architecture/22_testing.md) |
| External data/tables | [23_external_data.md](../architecture/23_external_data.md) |
| Quick start | [00a_quick_start.md](../architecture/00a_quick_start.md) |

---

## Quick Start

**Day 1:**
1. `include/icarus/core/Types.hpp` + `include/icarus/signal/Signal.hpp`
2. First unit test: signal registration and retrieval
3. Build vertically through Phase 1 → 2 → 3 with one trivial component
4. Symbolic mode validation in Phase 3 catches template issues early

**See also:** [icarus_bootstrap_guide.md](../icarus_bootstrap_guide.md) for repository setup, Nix configuration, and script templates.
