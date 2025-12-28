# Phase 4.0.7: Implementation Order & Exit Criteria

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Implementation Order

### Phase A: Configuration Infrastructure

1. **4.0.7a** Create `TrimConfig<Scalar>` struct
2. **4.0.7b** Create `LinearizationConfig` struct
3. **4.0.7c** Create `SymbolicsConfig` struct
4. **4.0.7d** Create `StageConfig<Scalar>` struct (combines trim + linear + symbolic)
5. **4.0.7e** Create `SchedulerConfig<Scalar>` struct with rate groups
6. **4.0.7f** Create `OutputConfig` struct
7. **4.0.7g** Create `SimulatorConfig<Scalar>` master struct with YAML loading (includes `reference_epoch_jd`)

### Phase B: Entity System

1. **4.0.7i** Create `EntityTemplate` struct with components + internal routes
2. **4.0.7j** Create `EntityInstance` struct with overrides
3. **4.0.7k** Create `SwarmConfig` struct with count + per-instance expressions
4. **4.0.7l** Create `EntitySystemConfig` with `ExpandAll()` method
5. **4.0.7m** Implement entity template YAML loading
6. **4.0.7n** Implement entity instantiation with override merging
7. **4.0.7o** Implement swarm expansion with expression evaluation
8. **4.0.7p** Implement internal route expansion (relative → absolute paths)

### Phase C: Scheduler Integration

1. **4.0.7q** Update `Scheduler` to use `SchedulerConfig`
2. **4.0.7r** Implement group-based multi-rate execution (group priority + member priority)
3. **4.0.7s** Implement group-based rate execution (frame divisors)
4. **4.0.7t** Add scheduler validation (rate constraints, missing components/groups)
5. ~~**4.0.7r-future** Implement topology-based automatic ordering~~ (Future TODO)

### Phase D: Internal Managers

1. **4.0.7u** Create `StateManager<Scalar>` class
2. **4.0.7v** Create `IntegrationManager<Scalar>` class

### Phase E: Simulator Core Refactor

1. **4.0.7w** Refactor `Simulator` public API (4 core operations)
2. **4.0.7x** Implement `FromConfig()` factory method with include resolution
3. **4.0.7y** Move `Provision()` to private, wire routing/state/ICs/scheduler
4. **4.0.7z** Implement new `Stage()` with trim/linear/symbolic

### Phase F: Trim Optimization

1. **4.0.7aa** Implement `RunTrimOptimization()` using Janus
2. **4.0.7ab** Add Newton method for trim solver
3. **4.0.7ac** Add control bounds support

### Phase G: Linearization

1. **4.0.7ad** Implement `ComputeLinearModel()` at trim point
2. **4.0.7ae** Add linear model export (MATLAB, NumPy, JSON)

### Phase H: Symbolic Mode

1. **4.0.7af** Implement `GenerateSymbolicGraphs()` for dynamics
2. **4.0.7ag** Implement symbolic Jacobian generation
3. **4.0.7ah** Implement CasADi function export

### Phase I: Testing & Cleanup

1. **4.0.7ai** Update existing tests to new API
2. **4.0.7aj** Add integration test: YAML → FromConfig → Stage → Step
3. **4.0.7ak** Add entity template + swarm tests
4. **4.0.7al** Add trim optimization tests
5. **4.0.7am** Add scheduler rate group tests
6. **4.0.7an** Remove deprecated public methods
7. **4.0.7ao** Update documentation

---

## Exit Criteria

### Configuration Structs

- [ ] `TrimConfig<Scalar>` with optimization settings, bounds, tolerances
- [ ] `LinearizationConfig` with state/input/output selection, export options
- [ ] `SymbolicsConfig` with dynamics/Jacobian/export settings
- [ ] `StageConfig<Scalar>` combining trim + linearization + symbolics
- [ ] `SchedulerConfig<Scalar>` with rate groups, topology settings
- [ ] ~~`EnvironmentConfig`~~ Removed - environment models are regular components
- [ ] `OutputConfig` for data dictionary, telemetry, timing reports
- [ ] `SimulatorConfig<Scalar>` master struct with all configuration
- [ ] `SimulatorConfig::FromFile()` with include resolution

### Entity System

- [ ] `EntityTemplate` with components + internal routes
- [ ] `EntityInstance` with template reference + overrides
- [ ] `SwarmConfig` with count + per-instance expressions
- [ ] `EntitySystemConfig` with `ExpandAll()` expansion
- [ ] Entity template YAML loading from external files
- [ ] Override merging (template defaults + instance overrides)
- [ ] Swarm expansion with expression evaluation (`${index}`)
- [ ] Internal route expansion (relative → absolute: `EOM.force` → `Leader.EOM.force`)
- [ ] Cross-entity route support

### Scheduler Integration

- [ ] `Scheduler` updated to use `SchedulerConfig` (group-based model)
- [ ] Group-based multi-rate execution (group priority + member priority)
- [ ] Frame divisor logic: group runs when `frame_count % divisor == 0`
- [ ] Each component receives group's dt (not global dt)
- [ ] Scheduler validation (rate constraints, no component in multiple groups)
- [ ] Logging of execution order
- [ ] ~~Automatic topology-based ordering~~ (Future TODO)

### Internal Managers

- [ ] `StateManager<Scalar>` encapsulating state vector management
- [ ] `IntegrationManager<Scalar>` encapsulating integrator logic

### Simulator Refactor

- [ ] Clean 4-operation core API: `FromConfig()`, `Stage()`, `Step()`, `~Simulator()`
- [ ] Query interface: `Time()`, `Peek<T>()`, `GetDataDictionary()`
- [ ] Control interface: `Poke<T>()`, `Reset()`, `SetInputSource()`
- [ ] Expert interface: `GetState()`, `GenerateGraph()`, `AdaptiveStep()`

### Lifecycle Implementation

- [ ] `FromConfig()` resolves includes, calls `Provision()` internally
- [ ] `Provision()` handles: components, routing, scheduler, state binding, ICs, validation
- [ ] `Stage()` handles: trim optimization, linearization, symbolic generation
- [ ] `Step()` handles: scheduler execution, integration, callbacks

### Trim Optimization

- [ ] `RunTrimOptimization()` using Janus optimization
- [ ] Newton method for trim solver
- [ ] Control bounds enforcement
- [ ] Configurable zero_derivatives and control_signals

### Linearization

- [ ] `ComputeLinearModel()` at trim point
- [ ] State-space (A, B, C, D) extraction
- [ ] Export to MATLAB, NumPy, JSON formats

### Symbolic Mode

- [ ] `GenerateSymbolicGraphs()` for dynamics
- [ ] Symbolic Jacobian generation
- [ ] CasADi function export

### YAML Schema

- [ ] Master binding file (`simulation.yaml`)
- [ ] Entity templates with components, routes, scheduler, staging
- [ ] Component configs with config_file references and overrides
- [ ] Cross-entity routes

### Testing

- [ ] Existing tests updated to new API
- [ ] Integration test: YAML → FromConfig → Stage → Step
- [ ] Entity template + swarm tests
- [ ] Trim optimization unit tests
- [ ] Scheduler rate group tests
- [ ] Linearization tests
- [ ] Symbolic generation tests

### Cleanup

- [ ] Old deprecated methods removed or made private
- [ ] Documentation updated
- [ ] Example configs in `config/` directory

---

## Migration Guide

### Before (Current API)

```cpp
// Create simulator
Simulator<double> sim;

// Add components manually
auto gravity = std::make_unique<PointMassGravity<double>>("Gravity", "Rocket");
sim.AddComponent(std::move(gravity));

// Configure integrator
sim.SetIntegrator(IntegratorType::RK4);

// Configure logging
sim.SetLogFile("sim.log");
sim.SetQuietMode(false);

// Initialize
sim.Provision();

// Load wiring
sim.LoadWiring("routes.yaml");

// Stage
sim.Stage();

// Run
while (sim.Time() < 100.0) {
    sim.Step(0.01);
}
```

### After (New API)

```cpp
// Create from config - FromConfig() calls Provision() internally
// This handles: component creation, signal wiring, state binding, ICs
auto sim = Simulator<double>::FromConfig("rocket.yaml", "routes.yaml");

// Stage - prepares the vehicle for launch
// This handles: trim optimization, symbolic generation (if configured)
sim.Stage();

// Run - numerical stepping
while (sim.Time() < 100.0) {
    sim.Step(0.01);
}

// Read telemetry
auto pos = sim.Peek<Vec3<double>>("Rocket.EOM.position");
```

### Programmatic Setup

```cpp
SimulatorConfig<double> config;
config.name = "Rocket Sim";
config.dt = 0.01;

// Components
config.components.push_back({.type = "PointMassGravity", .name = "Gravity", .entity = "Rocket"});

// Routes
config.routes.push_back({.input_path = "Rocket.EOM.gravity", .output_path = "Rocket.Gravity.accel"});

// Initial conditions
config.initial_conditions["Rocket.GNC.throttle"] = 0.8;

// Trim configuration
config.staging.trim.enabled = true;
config.staging.trim.zero_derivatives = {"Rocket.EOM.velocity_dot"};
config.staging.trim.control_signals = {"Rocket.GNC.throttle"};

// Integrator and logging
config.integrator = IntegratorConfig<double>::RK4Default();
config.logging.file_path = "sim.log";

// Create and run
auto sim = Simulator<double>::FromConfig(config);
sim.Stage();  // Runs trim optimization
sim.Step();
```

### With Trim and Symbolic Mode

```cpp
// Load config with trim enabled
auto sim = Simulator<double>::FromConfig("rocket_with_trim.yaml");

// Stage with custom override
StageConfig<double> stage_cfg;
stage_cfg.trim.enabled = true;
stage_cfg.trim.tolerance = 1e-8;  // Tighter tolerance
stage_cfg.generate_symbolics = true;
stage_cfg.symbolic_output_dir = "./casadi_functions";

sim.Stage(stage_cfg);  // Runs trim, generates CasADi functions

// Now in equilibrium state, ready to step
sim.Step(0.01);
```

---

## Compatibility Notes

- The refactored API is **not backward compatible**
- A compatibility shim could be provided but is not recommended
- Phase 4.0.9 (component refactoring) should use the new API
- Phase 4.0.12 (integration test) should test the new API exclusively
