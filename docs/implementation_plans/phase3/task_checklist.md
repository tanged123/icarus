# Phase 3 Task Checklist

**Status:** Not Started
**Prerequisites:** Phase 2 complete

---

## Pre-Implementation Validation

These tasks verify the codebase is ready before writing new code.

### 3.0.1 Janus Compliance Audit
- [ ] Create `scripts/janus_audit.sh`
- [ ] Run audit on `include/`, `src/`, `components/`
- [ ] Document violations found
- [ ] Fix all violations (if any)
- [ ] Add audit to CI as pre-merge check

### 3.0.2 Vulcan Compatibility Check
- [ ] Verify `vulcan::gravity::point_mass::acceleration<MX>` compiles
- [ ] Verify `vulcan::dynamics::point_mass_acceleration<MX>` compiles
- [ ] Check all Vulcan functions used by Icarus components
- [ ] Document any Vulcan issues to address upstream

### 3.0.3 Basic Symbolic Compilation Test
- [ ] Create `tests/symbolic/basic_compilation_test.cpp`
- [ ] Instantiate `Simulator<SymbolicScalar>`
- [ ] Verify compilation succeeds
- [ ] Document any template errors and fixes

---

## 3.1 Standardized Execution Framework

### 3.1.1 SimulationBuilder
- [ ] Create `include/icarus/sim/SimulationBuilder.hpp`
- [ ] Implement `AddComponent()` (move semantics)
- [ ] Implement `AddComponent<T>()` (in-place construction)
- [ ] Implement `Wire()` (component-level wiring)
- [ ] Implement `SetIntegrator()` (with default RK4)
- [ ] Implement `SetTimeStep()` / `SetEndTime()`
- [ ] Implement `EnableProfiling()` / `SetLogFile()` / `DisableLogging()`
- [ ] Implement `Build()` (returns Simulator with components)
- [ ] Implement `BuildAndInitialize()` (includes Provision + Stage)
- [ ] Stub config methods for Phase 5 forward-compatibility:
  - [ ] `LoadScenario()` - throws NotImplemented, documents API
  - [ ] `LoadServices()` - throws NotImplemented
  - [ ] `LoadTrimConfig()` - throws NotImplemented
  - [ ] `OverrideParam()` / `OverrideInitialCondition()` - throws NotImplemented
  - [ ] `SetConfigPaths()` - throws NotImplemented
- [ ] Create `tests/sim/simulation_builder_test.cpp`
- [ ] Test: basic construction
- [ ] Test: wiring applied correctly
- [ ] Test: integrator configuration
- [ ] Test: symbolic instantiation compiles

### 3.1.2 Simulator External Interface API
- [ ] Implement `Peek<T>(name)` / `Poke<T>(name, value)` template methods
- [ ] Implement `PeekBatch()` / `PokeBatch()` for efficient bulk access
- [ ] Implement `GetState()` / `SetState()` for checkpointing
- [ ] Implement `Initialize()` convenience method (Provision + Stage)
- [ ] Implement `Reset()` for re-running with same setup
- [ ] Implement `IsInitialized()` state check
- [ ] Implement `GetSignalNames()` / `GetInputNames()` / `GetOutputNames()` / `GetParameterNames()`
- [ ] Implement `RegisterInputSource()` / `UnregisterInputSource()` for external control
- [ ] Implement `RegisterOutputObserver()` / `UnregisterOutputObserver()` for telemetry
- [ ] Test: Peek/Poke round-trip
- [ ] Test: State get/set preserves simulation
- [ ] Test: Input source callback invoked on Step()
- [ ] Test: Output observer callback invoked on Step()

### 3.1.3 SimulationResults
- [ ] Create `include/icarus/sim/SimulationResults.hpp`
- [ ] Define `SimulationResults<Scalar>` struct
- [ ] Include timing, final signals, statistics, status

### 3.1.4 SimulationRunner (Optional)
- [ ] Create `include/icarus/sim/SimulationRunner.hpp`
- [ ] Implement constructor with Simulator reference
- [ ] Implement `SetProgressCallback()`
- [ ] Implement `SetStepCallback()`
- [ ] Implement `SetRecorder()` with interval
- [ ] Implement `EnableProgressBar()`
- [ ] Implement `Initialize()` (Provision + Stage)
- [ ] Implement `Run(dt, t_end)` main loop
- [ ] Implement `Shutdown()` with debrief
- [ ] Implement `GetResults()` / `GetHistory()`
- [ ] Create `tests/sim/simulation_runner_test.cpp`
- [ ] Test: lifecycle management
- [ ] Test: callbacks invoked correctly
- [ ] Test: results populated

---

## 3.2 Dual-Mode Validation

### 3.2.1 Symbolic Compilation Tests
- [ ] Create `tests/symbolic/symbolic_compilation_test.cpp`
- [ ] Test: `Simulator<SymbolicScalar>` instantiation
- [ ] Test: `PointMass3DOF<SymbolicScalar>` construction
- [ ] Test: `PointMassGravity<SymbolicScalar>` construction
- [ ] Test: `RK4Integrator<SymbolicScalar>` instantiation
- [ ] Test: Full lifecycle (Provision → Stage → Step)

### 3.2.2 Component Audit
- [ ] Audit `PointMass3DOF.hpp` for Janus compliance
- [ ] Audit `PointMassGravity.hpp` for Janus compliance
- [ ] Document any fixes required
- [ ] Apply fixes and verify symbolic compilation

### 3.2.3 CI Integration
- [ ] Add CMake option: `-DICARUS_ENABLE_SYMBOLIC_TESTS=ON`
- [ ] Create CI job for symbolic mode build
- [ ] Run symbolic tests in CI
- [ ] Ensure failure blocks merge

---

## 3.3 Graph Export

### 3.3.1 SymbolicTracer Core
- [ ] Create `include/icarus/symbolic/SymbolicTracer.hpp`
- [ ] Create `src/symbolic/SymbolicTracer.cpp`
- [ ] Implement `TracerConfig` struct
- [ ] Implement `SymbolicTracer` constructor
- [ ] Implement `Configure()`
- [ ] Implement `CreateSymbolicVariables()` (internal)
- [ ] Implement `ScatterSymbolicState()` (internal)
- [ ] Implement `GatherSymbolicDerivatives()` (internal)
- [ ] Implement `TraceDynamics()` main method
- [ ] Implement `GetStateNames()` / `GetStateSize()`
- [ ] Implement `GetControlNames()` / `GetControlSize()`

### 3.3.2 Extended Tracing
- [ ] Implement `TraceOutputs()` for observable signals
- [ ] Implement `TraceStep()` for discrete-time function
- [ ] Implement `DetectControlSignals()` heuristic

### 3.3.3 Convenience Functions
- [ ] Implement `ExtractDynamics()` free function
- [ ] Implement `ExtractStateJacobian()` free function

### 3.3.4 Simulator Integration
- [ ] Add `GenerateGraph()` to `Simulator<Scalar>`
- [ ] Use SFINAE to restrict to `SymbolicScalar`
- [ ] Expose internal vectors via friend class

### 3.3.5 Tracer Tests
- [ ] Create `tests/symbolic/symbolic_tracer_test.cpp`
- [ ] Test: dynamics function dimensions
- [ ] Test: Jacobian extraction
- [ ] Test: numeric evaluation matches expected
- [ ] Test: control signal detection
- [ ] Test: output tracing

---

## 3.4 Symbolic Test Suite

### 3.4.1 Numeric/Symbolic Comparison
- [ ] Create `tests/symbolic/numeric_symbolic_comparison_test.cpp`
- [ ] Test: Point mass free fall comparison
- [ ] Test: Circular orbit comparison
- [ ] Test: Multi-component wiring comparison
- [ ] Test: Multi-step accumulation comparison
- [ ] Define tolerance guidelines (< 1e-10 relative error)

### 3.4.2 Derivative Verification
- [ ] Create `tests/symbolic/derivative_verification_test.cpp`
- [ ] Test: Free fall Jacobian (analytical reference)
- [ ] Test: Gravity Jacobian structure (sparse pattern)
- [ ] Test: Finite difference comparison for complex cases

### 3.4.3 Integration Tests
- [ ] Test: CasADi integrator with extracted dynamics
- [ ] Test: Opti formulation compiles (no solve)
- [ ] Test: Code generation produces valid C

---

## 3.5 Logger Symbolic Support

### 3.5.1 Scalar Formatting
- [ ] Create `include/icarus/util/ScalarFormat.hpp`
- [ ] Implement `FormatScalar<Scalar>()` template
- [ ] Specialize for `double` (numeric formatting)
- [ ] Specialize for `SymbolicScalar` (expression string)

### 3.5.2 Logger Updates
- [ ] Add `SetSymbolicMode()` to MissionLogger
- [ ] Suppress verbose output in symbolic mode
- [ ] Update progress display for MX values
- [ ] Test: symbolic sim runs without logger crash

---

## 3.6 Example Migration

### 3.6.1 Update Existing Examples
- [ ] Refactor `examples/intro/orbital_demo.cpp` to use SimulationBuilder
- [ ] Refactor `examples/intro/formation_flight_demo.cpp` to use SimulationBuilder
- [ ] Verify both examples still run correctly
- [ ] Measure code reduction (lines saved)

### 3.6.2 New Symbolic Example
- [ ] Create `examples/symbolic/symbolic_orbital_demo.cpp`
- [ ] Demonstrate identical builder pattern for `<double>` and `<MX>`
- [ ] Show dynamics extraction
- [ ] Show Jacobian computation
- [ ] Add to `CMakeLists.txt`
- [ ] Add to `scripts/run_examples.sh`

### 3.6.3 Documentation
- [ ] Update `docs/guides/` with symbolic mode guide
- [ ] Add API reference for SimulationBuilder
- [ ] Add API reference for SymbolicTracer
- [ ] Update architecture docs if needed

---

## Exit Criteria Verification

Final checklist before marking Phase 3 complete:

- [ ] `Simulator<SymbolicScalar>` compiles with all existing components
- [ ] `GenerateGraph()` returns valid `janus::Function`
- [ ] Numeric/symbolic outputs match within tolerance (< 1e-10)
- [ ] Jacobian extraction works for point mass dynamics
- [ ] Zero Janus violations (audit passes)
- [ ] Examples migrated to SimulationBuilder
- [ ] CI includes symbolic mode build and test
- [ ] All tests pass

---

## Execution Order

Recommended sequence to minimize rework:

```
Week 1: Validation & Foundation
├── 3.0.1 Janus Audit
├── 3.0.2 Vulcan Check
├── 3.0.3 Basic Compilation Test
└── 3.2.2 Component Audit + fixes

Week 2: Builder Infrastructure
├── 3.1.1 SimulationBuilder
├── 3.1.2 SimulationResults
└── 3.1.3 SimulationRunner

Week 3: Graph Export
├── 3.3.1 SymbolicTracer Core
├── 3.3.2 Extended Tracing
├── 3.3.3 Convenience Functions
├── 3.3.4 Simulator Integration
└── 3.3.5 Tracer Tests

Week 4: Testing & Examples
├── 3.4.1 Numeric/Symbolic Comparison
├── 3.4.2 Derivative Verification
├── 3.5.1 Scalar Formatting
├── 3.5.2 Logger Updates
├── 3.6.1 Update Examples
├── 3.6.2 New Symbolic Example
└── 3.6.3 Documentation
```

---

## Dependencies Between Tasks

```
3.0.1 ─┬─► 3.2.2 ─► 3.2.1
       │
3.0.2 ─┘

3.1.1 ─► 3.1.3 ─► 3.6.1
       │
       └─► 3.6.2

3.2.1 ─► 3.3.1 ─► 3.3.4 ─► 3.4.1
                        │
                        └─► 3.4.2

3.5.1 ─► 3.5.2
```

Key dependencies:
1. Janus audit must pass before component fixes
2. Builder must exist before runner
3. Symbolic compilation must work before tracer
4. Tracer must work before comparison tests
