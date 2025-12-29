# Phase 4.0.7: Implementation Order & Exit Criteria

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Implementation Scope

### Phase 4.0.7 Focus: Core Infrastructure

This phase establishes the **bones of implementation**:

| Category | In Scope | Status |
|:---------|:---------|:-------|
| Config structs | `SimulatorConfig`, `SchedulerConfig`, `StageConfig`, etc. | Core |
| Entity system | Templates, instances, swarms, expansion | Core |
| Scheduler | Group-based multi-rate execution | Core |
| Simulator lifecycle | `FromConfig()` → `Stage()` → `Step()` | Core |
| Symbolic graphs | Already exists | See [symbolic_orbital_demo.cpp](../../../examples/symbolic/symbolic_orbital_demo.cpp) |
| Jacobian computation | Already exists | Via `janus::jacobian()` |

### Deferred to Future Phases

Advanced staging features build on this infrastructure:

| Feature | Rationale |
|:--------|:----------|
| Trim optimization | Requires NewtonSolver/IPOPT integration |
| Linearization export | MATLAB/NumPy/JSON formatting |
| Trajectory optimization | DirectCollocation/MultipleShooting |

> **Note:** Basic symbolic capabilities already exist and are demonstrated in `symbolic_orbital_demo.cpp`. Phase 4.0.7 focuses on configuration/lifecycle infrastructure, not advanced analysis features.

---

## Implementation Order

### Phase A: Configuration Infrastructure

1. **4.0.7a** Create `TrimConfig` struct
2. **4.0.7b** Create `LinearizationConfig` struct
3. **4.0.7c** Create `SymbolicsConfig` struct
4. **4.0.7d** Create `StageConfig` struct (combines trim + linear + symbolic)
5. **4.0.7e** Create `SchedulerConfig` struct with rate groups
6. **4.0.7f** Create `OutputConfig` struct
7. **4.0.7g** Create `SimulatorConfig` master struct (includes `reference_epoch_jd`)
8. **4.0.7h** Create `IntegratorConfig` struct
9. **4.0.7i** Create `LogConfig` struct

> **Note:** Config structs are NOT templated - they use `double` for numeric values.

### Phase A.5: YAML Configuration Infrastructure

> **See:** [phase4_0_7_yaml_config.md](phase4_0_7_yaml_config.md) for full details.

1. **4.0.7a5a** Create `SimulationLoader` class using Vulcan's YAML infrastructure
2. **4.0.7a5b** Implement single-file mode (all config inline)
3. **4.0.7a5c** Implement multi-file mode (`!include` directive support via Vulcan)
4. **4.0.7a5d** Implement environment variable expansion via `YamlEnv`
5. **4.0.7a5e** Create error types: `ConfigError`, `MissingKeyError`, `TypeMismatchError`, `UnknownTypeError`
6. **4.0.7a5f** Implement error messages with file path and hints
7. **4.0.7a5g** Add Janus type support (Vec3, Quaternion, Mat3) via Vulcan
8. **4.0.7a5h** Add unit tests for all parsing paths

### Phase B: Entity System

1. **4.0.7j** Create `EntityTemplate` struct with components + internal routes
2. **4.0.7k** Create `EntityInstance` struct with overrides
3. **4.0.7l** Create `SwarmConfig` struct with count (identical copies)
4. **4.0.7m** Create `EntitySystemConfig` with `ExpandAll()` method
5. **4.0.7n** Implement entity template YAML loading
6. **4.0.7o** Implement entity instantiation with override merging
7. **4.0.7p** Implement swarm expansion (identical copies, no expressions)
8. **4.0.7q** Implement internal route expansion (relative → absolute paths)

> **Deferred:** Per-instance swarm expressions (`${index}`, Jinja-style templating) are future work.

### Phase C: Scheduler Integration

1. **4.0.7r** Update `Scheduler` to use `SchedulerConfig`
2. **4.0.7s** Implement group-based multi-rate execution (group priority + member priority)
3. **4.0.7t** Implement frame divisor logic: group runs when `frame_count % divisor == 0`
4. **4.0.7u** Each component receives group's dt (not global dt)
5. **4.0.7v** Add scheduler validation (rate constraints, no component in multiple groups)
6. **4.0.7w** Logging of execution order

> **Deferred:** Automatic topology-based ordering is future TODO.

### Phase D: Internal Managers

> **Note:** Internal managers ARE templated (they work with Component<Scalar>).

1. **4.0.7x** Create `StateManager<Scalar>` class
2. **4.0.7y** Create `IntegrationManager<Scalar>` class

### Phase E: Simulator Core Refactor

> **See:** [phase4_0_7_simulator_api.md](phase4_0_7_simulator_api.md) for full API design.

1. **4.0.7z** Refactor `Simulator` public API (single non-templated class)
2. **4.0.7aa** Implement `FromConfig()` factory method with include resolution
3. **4.0.7ab** Move `Provision()` to private, wire routing/state/ICs/scheduler
4. **4.0.7ac** Implement new `Stage()` (wiring validation, symbolic graphs if enabled)
5. **4.0.7ad** Implement `Step()` with scheduler execution, integration, callbacks
6. **4.0.7ae** Implement Query interface: `Time()`, `Peek<T>()`, `GetDataDictionary()`
7. **4.0.7af** Implement Control interface: `Poke<T>()`, `Reset()`, `SetInputSource()`
8. **4.0.7ag** Implement Expert interface: `GetState()`, `GetDynamicsGraph()`, `AdaptiveStep()`

### Phase F: Trim Optimization *(Future)*

> **Deferred:** These features build on Phase 4.0.7 infrastructure but are not required for the core lifecycle.

1. **Future** Implement `RunTrimOptimization()` using Janus NewtonSolver
2. **Future** Implement IPOPT-based trim via `janus::Opti`
3. **Future** Add control bounds support
4. **Future** Add parametric sweep capability

### Phase G: Linearization *(Future)*

> **Deferred:** Linearization export requires trim to establish operating point.

1. **Future** Implement `ComputeLinearModel()` at trim point
2. **Future** Add linear model export (MATLAB, NumPy, JSON)

### Phase H: Advanced Symbolic Mode *(Future)*

> **Deferred:** Basic symbolic capabilities already exist in `symbolic_orbital_demo.cpp`. Advanced features are additive.

1. **Future** Implement symbolic Jacobian export to files
2. **Future** Implement CasADi function serialization
3. **Future** Add trajectory optimization integration (DirectCollocation/MultipleShooting)

### Phase I: Testing & Cleanup

1. **4.0.7ah** Update existing tests to new API
2. **4.0.7ai** Add integration test: YAML → FromConfig → Stage → Step
3. **4.0.7aj** Add entity template + swarm tests
4. **4.0.7ak** Add scheduler rate group tests
5. **4.0.7al** Add YAML configuration loader tests
6. **4.0.7am** Remove deprecated public methods
7. **4.0.7an** Update documentation
8. **4.0.7ao** Create example config files in `config/` directory

---

## Exit Criteria

### Configuration Structs

> **Note:** Config structs are NOT templated - they use `double` for numeric values.

- [ ] `TrimConfig` with optimization settings, bounds, tolerances
- [ ] `LinearizationConfig` with state/input/output selection, export options
- [ ] `SymbolicsConfig` with dynamics/Jacobian/export settings
- [ ] `StageConfig` combining trim + linearization + symbolics
- [ ] `SchedulerConfig` with rate groups, topology settings
- [ ] `IntegratorConfig` with integrator type and tolerances
- [ ] `LogConfig` with console/file logging settings
- [ ] `OutputConfig` for data dictionary, telemetry, timing reports
- [ ] `SimulatorConfig` master struct with all configuration

### YAML Configuration Infrastructure

- [ ] `SimulationLoader` class using Vulcan's YAML infrastructure
- [ ] Single-file mode: all config inline
- [ ] Multi-file mode: `!include` directive support via Vulcan
- [ ] Environment variable expansion via `YamlEnv`
- [ ] Error types: `ConfigError`, `MissingKeyError`, `TypeMismatchError`, `UnknownTypeError`
- [ ] Error messages include file path and hints
- [ ] Janus type support (Vec3, Quaternion, Mat3) via Vulcan
- [ ] Unit tests for all parsing paths

### Entity System

- [ ] **Single-file mode**: entire sim definable in one YAML (no external files required)
- [ ] **Multi-file mode**: entity templates loadable from external files (optional)
- [ ] `EntityTemplate` with components + internal routes
- [ ] `EntityInstance` with template reference + overrides
- [ ] `SwarmConfig` with count (identical copies, no expressions)
- [ ] `EntitySystemConfig` with `ExpandAll()` expansion
- [ ] Override merging (template defaults + instance overrides)
- [ ] Swarm expansion (identical copies)
- [ ] Internal route expansion (relative → absolute: `EOM.force` → `Leader.EOM.force`)
- [ ] Cross-entity route support

### Deferred Entity Features

- [ ] *(Future)* Swarm per-instance expressions (`${index}`, Jinja-style)
- [ ] *(Future)* Swarm-internal routes (neighbor connections)

### Scheduler Integration

- [ ] `Scheduler` updated to use `SchedulerConfig` (group-based model)
- [ ] Auto-derive simulation rate from fastest group rate across all entities
- [ ] Validate all group rates are integer divisors of simulation rate
- [ ] Group-based multi-rate execution (group priority + member priority)
- [ ] Frame divisor logic: group runs when `frame_count % divisor == 0`
- [ ] Each component receives group's dt (not global dt)
- [ ] Scheduler validation (rate constraints, no component in multiple groups)
- [ ] Logging of execution order

### Deferred Scheduler Features

- [ ] *(Future)* Automatic topology-based ordering

### Internal Managers

> **Note:** Internal managers ARE templated (they work with Component<Scalar>).

- [ ] `StateManager<Scalar>` encapsulating state vector management
- [ ] `IntegrationManager<Scalar>` encapsulating integrator logic

### Simulator Refactor

> **Note:** `Simulator` class is NOT templated. Symbolic mode used internally during `Stage()`.

- [ ] Single non-templated `Simulator` class
- [ ] Clean 4-operation core API: `FromConfig()`, `Stage()`, `Step()`, `~Simulator()`
- [ ] Query interface: `Time()`, `Peek<T>()`, `GetDataDictionary()`
- [ ] Control interface: `Poke<T>()`, `Reset()`, `SetInputSource()`
- [ ] Expert interface: `GetState()`, `GetDynamicsGraph()`, `AdaptiveStep()`

### Lifecycle Implementation

- [ ] `FromConfig()` resolves includes, calls `Provision()` internally
- [ ] `Provision()` handles: components, routing, scheduler, state binding, ICs, validation
- [ ] `Stage()` handles: wiring validation, symbolic graphs (if enabled)
  - *(Future)* trim optimization, linearization
- [ ] `Step()` handles: scheduler execution, integration, callbacks

### Trim Optimization *(Future Phase)*

> Basic `Stage()` lifecycle is in scope; full trim optimization is deferred.

- [ ] *(Future)* `RunTrimOptimization()` using Janus optimization
- [ ] *(Future)* Newton method for trim solver
- [ ] *(Future)* Control bounds enforcement
- [ ] *(Future)* Configurable zero_derivatives and control_signals

### Linearization *(Future Phase)*

> Linearization requires trim to establish operating point.

- [ ] *(Future)* `ComputeLinearModel()` at trim point
- [ ] *(Future)* State-space (A, B, C, D) extraction
- [ ] *(Future)* Export to MATLAB, NumPy, JSON formats

### Symbolic Mode

> **Already exists:** See [symbolic_orbital_demo.cpp](../../../examples/symbolic/symbolic_orbital_demo.cpp)

- [x] `TraceDynamics()` for dynamics graph extraction
- [x] `TraceStep()` for discrete step function
- [x] Jacobian computation via `janus::jacobian()`
- [x] Graph visualization (HTML, DOT, PDF)
- [x] Data dictionary export (YAML, JSON)
- [ ] *(Future)* CasADi function serialization
- [ ] *(Future)* Symbolic Jacobian export to files

### YAML Schema

> **See:** [phase4_0_7_yaml_config.md](phase4_0_7_yaml_config.md) for complete schema reference.

- [ ] Master binding file (`simulation.yaml`)
- [ ] Single-file mode: all config inline in one file
- [ ] Multi-file mode: entity templates loadable from external files
- [ ] Component configs with config_file references and overrides
- [ ] Cross-entity routes
- [ ] Environment variable expansion (`${VAR}`, `${VAR:default}`)

### Testing

**Core Tests (Phase 4.0.7):**
- [ ] Existing tests updated to new API
- [ ] Integration test: YAML → FromConfig → Stage → Step
- [ ] Entity template + swarm tests
- [ ] Scheduler rate group tests
- [ ] YAML configuration loader tests
- [ ] Single-file and multi-file mode tests

**Future Tests:**
- [ ] *(Future)* Trim optimization unit tests
- [ ] *(Future)* Linearization tests
- [ ] *(Future)* Advanced symbolic export tests

### Cleanup

- [ ] Old deprecated methods removed or made private
- [ ] Documentation updated
- [ ] Example configs in `config/` directory

---

## Usage Examples

### Minimal Single-File Simulation

```cpp
// One line to load everything
auto sim = Simulator::FromConfig("orbital_demo.yaml");

// Stage (validates wiring, runs trim/linearization if configured)
sim.Stage();

// Run
while (sim.Time() < 100.0) {
    sim.Step(0.01);
}

// Read telemetry
auto pos = sim.Peek<Vec3<double>>("Dynamics.position");
```

### Programmatic Setup

```cpp
SimulatorConfig config;
config.name = "Rocket Sim";
config.dt = 0.01;

// Components
config.components.push_back({.type = "PointMassGravity", .name = "Gravity"});
config.components.push_back({.type = "PointMass3DOF", .name = "Dynamics"});

// Routes
config.routes.push_back({.input_path = "Gravity.position", .output_path = "Dynamics.position"});
config.routes.push_back({.input_path = "Dynamics.force", .output_path = "Gravity.force"});

// Create and run
auto sim = Simulator::FromConfig(config);
sim.Stage();
sim.Step();
```

### Accessing Symbolic Results (After Stage)

```cpp
auto sim = Simulator::FromConfig("rocket.yaml");
sim.Stage();

// If staging.symbolics.enabled = true:
auto dynamics = sim.GetDynamicsGraph();  // f(t, x) -> x_dot
auto jacobian = sim.GetJacobian();       // df/dx

// If staging.linearization.enabled = true:
auto linear = sim.GetLinearModel();      // A, B, C, D matrices
```

---

## Development Notes

- **No backwards compatibility required** - this is active development
- Existing tests/examples will be rewritten to use new API
- Component interface (`Provision()`, `Step()`) remains the same
- Only the Simulator facade changes
