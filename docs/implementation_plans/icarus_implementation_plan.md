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

- [x] **1.3 Signal Backplane**
  - [x] `include/icarus/signal/Signal.hpp` — SignalType enum, lifecycle
  - [x] `include/icarus/signal/Registry.hpp` — `register_output()`, `resolve()`
  - [x] Type-safe signal access

- [x] **1.4 Component Base**
  - [x] Abstract `Component<Scalar>` with `Provision()`, `Stage()`, `Step()`
  - [x] Optional hooks (`PreStep`, `PostStep`, `OnPhaseEnter`, etc.)

- [x] **1.5 Simulator Shell**
  - [x] Top-level `Simulator<Scalar>` class
  - [x] Owns component list and backplane
  - [x] Calls lifecycle methods in sequence

### Architecture References

| Topic | Document |
|:------|:---------|
| Flat topology, terminology | [01_core_philosophy.md](../architecture/01_core_philosophy.md) |
| Component interface | [02_component_protocol.md](../architecture/02_component_protocol.md) |
| Signal registry | [03_signal_backplane.md](../architecture/03_signal_backplane.md) |
| Lifecycle phases | [04_lifecycle.md](../architecture/04_lifecycle.md) |

### Exit Criteria

- [x] Can instantiate `Simulator<double>`, add a dummy component
- [x] Call Provision/Stage/Step without crashes
- [x] Read/write signals via Backplane
- [x] All Phase 1 tests pass

---

## Phase 2: State & Integration

**Goal:** Actual differential equations work.

### Tasks

- [x] **2.1 State Management**
  - [x] Global `X_global_`, `X_dot_global_` vectors
  - [x] State registration and ownership
  - [x] Scatter/gather via pointer binding to components

- [x] **2.2 Integrator Interface**
  - [x] Abstract `Integrator<Scalar>` interface
  - [x] RK4 implementation
  - [x] Adaptive RK45 implementation

- [x] **2.3 First Real Component**
  - [x] `PointMass3DOF` with position/velocity state, using Vulcan coordinate systems
  - [x] `PointMassGravity` with using Vulcan simple round Earth gravity model
  - [x] Demonstration of connecting components and running a simulation all the way through.

- [x] **2.4 Component Interface System**
  - [x] Explicit input port registration (`register_input()`)
  - [x] Scalar-typed parameter registration (`register_param()`)
  - [x] External wiring configuration (YAML or programmatic)
  - [x] Unified signal access API (`Get<T>()`/`Set<T>()`)
  - [x] Data Dictionary with outputs, inputs, and parameters
  - [x] Pre-run wiring validation

- [x] **2.5 ASCII Logging and Data Dictionary Display**
  - [x] Console abstraction with ANSI color support
  - [x] ASCII box-drawing table formatter
  - [x] Flight Manifest (formatted Data Dictionary at Provision end)
  - [x] Mission Logger with lifecycle phase tracking
  - [x] Mission Debrief with profiling statistics
  - [x] Simulator integration for automatic logging

### Architecture References

| Topic | Document |
|:------|:---------|
| Global state vector | [09_memory_state_ownership.md](../architecture/09_memory_state_ownership.md) |
| State binding in Stage | [04_lifecycle.md](../architecture/04_lifecycle.md) |
| Component interface model | [02_component_protocol.md](../architecture/02_component_protocol.md) |
| Services configuration | [15_services.md](../architecture/15_services.md) |

### Exit Criteria

- [x] Simulate a falling object (point mass under gravity)
- [x] Verify against analytical solution: `y(t) = y₀ + v₀t - ½gt²`
- [x] State correctly scattered/gathered through integrator
- [x] Components use explicit input/parameter registration
- [x] Wiring is external to components
- [x] Any signal accessible via `sim.Get<T>()`/`sim.Set<T>()`
- [x] Flight Manifest displays at Provision end
- [x] Mission Debrief displays at Shutdown with timing stats

---

## Phase 3: Symbolic Mode

**Goal:** Same code runs with `casadi::MX`.

### Tasks

- [x] **3.1 Dual-Mode Validation**
  - [x] Instantiate `Simulator<casadi::MX>`, ensure compilation
  - [x] No `std::` math or `if/else` on Scalar in codebase

- [x] **3.2 Graph Export**
  - [x] `GenerateGraph()` returns `casadi::Function`
  - [x] Derivatives extractable via AD

- [x] **3.3 Symbolic Test Suite**
  - [x] Verify `PointMass3DOF` traces correctly
  - [x] Numeric/symbolic output comparison tests

### Architecture References

| Topic | Document |
|:------|:---------|
| Template-first paradigm | [07_janus_integration.md](../architecture/07_janus_integration.md) |
| `janus::where()`, math rules | [21_symbolic_constraints.md](../architecture/21_symbolic_constraints.md) |

### Exit Criteria

- [x] Extract symbolic dynamics as `janus::Function`
- [x] Evaluate numerically, match `Simulator<double>` output
- [x] Zero template violations in codebase

---

## Phase 4: Aggregation & 6DOF

**Goal:** Multi-component force/moment aggregation with config-driven initialization.

> **See:** [phase4/](implementation_plans/phase4/) for detailed implementation plans.

### Prerequisites

- [x] **4.0 Configuration Infrastructure** ([phase4_0_config_infrastructure.md](implementation_plans/phase4/phase4_0_config_infrastructure.md))
  - [x] `ComponentConfig` with typed accessors
  - [x] YAML configuration loader
  - [x] `ComponentFactory` for type registration
  - [x] Refactor existing components (no public setters)

### Tasks

- [x] **4.1 Signal Conventions**
  - [x] Mass sources publish `MassProperties<Scalar>`
  - [x] Force sources publish `force` in body frame
  - [x] Backplane support for composite types

- [x] **4.2 Aggregators**
  - [x] `MassAggregator` — uses Vulcan `MassProperties::operator+`
  - [x] `ForceAggregator` — sums forces, moment transfer about CG

- [x] **4.3 RigidBody6DOF**
  - [x] 13-state quaternion dynamics using Vulcan EOM
  - [x] Consumes aggregated force/moment/inertia

### Architecture References

| Topic | Document |
|:------|:---------|
| Aggregation pattern | [12_quantity_aggregation.md](../architecture/12_quantity_aggregation.md) |
| Vulcan utilities | [08_vulcan_integration.md](../architecture/08_vulcan_integration.md) |

### Exit Criteria

- [x] Tumbling rigid body with angular momentum conservation
- [x] Multi-source force aggregation with moment transfer
- [x] All components config-driven (no public setters)

---

## Phase 5: Advanced Configuration & Scheduling

**Goal:** Extended configuration layers and multi-rate scheduling.

> **Note:** Basic YAML loading moved to Phase 4.0. Phase 5 extends with advanced features.

### Tasks

- [ ] **5.1 Extended Configuration Layers**
  - [ ] Entity bundles (Layer A') — reusable component groups
  - [ ] Scenario definitions (Layer B) — initial conditions, variants
  - [ ] Include/merge support for config composition

- [ ] **5.2 Scheduler**
  - [ ] Topological sort from dependency graph
  - [ ] Rate groups for multi-rate simulation (Layer D)

- [ ] **5.3 Data Dictionary Export**
  - [ ] Auto-generate signal catalog at Provision
  - [ ] Export as JSON/YAML

### Architecture References

| Topic | Document |
|:------|:---------|
| Configuration layers | [13_configuration.md](../architecture/13_configuration.md) |
| Scheduler, rate groups | [05_execution_model.md](../architecture/05_execution_model.md) |
| Entity namespaces | [06_entities_namespaces.md](../architecture/06_entities_namespaces.md) |

### Exit Criteria

- [ ] Load complex scenario with entity bundles
- [ ] Multi-rate simulation working
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
