# Phase 4.0.7: Simulator API Refactor

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Critical Design Decision: Single Simulator Class

**The user-facing `Simulator` class is NOT templated on Scalar.**

```cpp
// User sees ONE class
class Simulator {
    // Stage() uses symbolic mode internally for trim, linearization, graph gen
    // Step() uses numeric mode for deterministic frame-by-frame execution
};

// NOT this:
// Simulator<double> num_sim;
// Simulator<SymbolicScalar> sym_sim;  // ❌ No duplication
```

**Rationale:**
- Users shouldn't need to understand symbolic vs numeric mode
- Symbolic capabilities are implementation details used during `Stage()`
- Numeric execution happens in `Step()`
- Components are still templated internally, but the Simulator facade hides this

**Implementation Note:** Internally, the Simulator may maintain both numeric and symbolic component instances, or use type erasure. The user doesn't care.

---

## Problem Statement

The current `Simulator<Scalar>` class has grown into a "god object" with 50+ public methods spanning:

- Component lifecycle management
- Signal/backplane operations
- State vector management
- Integrator configuration and execution
- Logging configuration
- External input/output callbacks
- Configuration loading
- Wiring configuration
- Symbolic graph generation

This violates the Single Responsibility Principle and makes the external API difficult to understand and use correctly.

---

## Current Public API (Problematic)

```cpp
// Current: Everything is public, no clear structure
class Simulator {
public:
    // Lifecycle (OK)
    void AddComponent(...);
    void Provision();
    void Stage();
    void Step(Scalar dt);

    // But also...
    Scalar Time() const;
    Phase GetPhase() const;
    SignalRegistry<Scalar>& GetRegistry();
    Backplane<Scalar>& GetBackplane();
    Scalar GetSignal(const std::string&) const;
    void SetSignal(const std::string&, const Scalar&);
    template<typename T> T Peek(const std::string&) const;
    template<typename T> void Poke(const std::string&, const T&);
    std::map<...> PeekBatch(...) const;
    void PokeBatch(...);
    void Initialize();
    void Reset();
    bool IsInitialized() const;
    void SetTime(Scalar);
    std::vector<std::string> GetSignalNames() const;
    std::vector<std::string> GetInputNames() const;
    std::vector<std::string> GetOutputNames() const;
    void RegisterInputSource(...);
    void UnregisterInputSource(...);
    void RegisterOutputObserver(...);
    void UnregisterOutputObserver(...);
    std::size_t NumInputSources() const;
    std::size_t NumOutputObservers() const;
    std::size_t NumComponents() const;
    const std::vector<std::string>& GetComponentOutputs(...) const;
    const std::vector<std::string>& GetComponentInputs(...) const;
    void SetWiring(...);
    template<typename T> void Wire(...);
    void ValidateWiring() const;
    std::vector<std::string> GetUnwiredInputs() const;
    void LoadWiring(const std::string&);
    void LoadWiring(const WiringConfig&);
    const WiringConfig& GetWiringConfig() const;
    void GenerateDataDictionary(...) const;
    DataDictionary GetDataDictionary() const;
    std::size_t GetTotalStateSize() const;
    JanusVector<Scalar> GetState() const;
    void SetState(const JanusVector<Scalar>&);
    const JanusVector<Scalar>& ComputeDerivatives(Scalar);
    const JanusVector<Scalar>& GetDerivatives() const;
    const std::vector<StateSlice<Scalar>>& GetStateLayout() const;
    void SetNominalDt(Scalar);
    Scalar GetNominalDt() const;
    void SetIntegrator(const IntegratorConfig<Scalar>&);
    void SetIntegrator(IntegratorType);
    void SetIntegrator(const std::string&);
    void SetIntegrator(std::unique_ptr<Integrator<Scalar>>);
    Integrator<Scalar>* GetIntegrator() const;
    const IntegratorConfig<Scalar>& GetIntegratorConfig() const;
    IntegratorType GetIntegratorType() const;
    AdaptiveStepResult<Scalar> AdaptiveStep(Scalar);
    MissionLogger& GetLogger();
    const MissionLogger& GetLogger() const;
    void SetQuietMode(bool);
    void SetLogFile(const std::string&);
    void SetProfilingEnabled(bool);
    void ApplyLogConfig(const LogConfig&);
    janus::Function GenerateGraph();
    janus::Function GenerateJacobian();
};
```

---

## Design Principle: Clean External Interface

The external user should only need to understand 4 operations:

1. **Init** - Load configuration, create components
2. **Stage** - Wire signals, apply ICs, prepare for simulation
3. **Step** - Advance simulation by dt
4. **Deinit** - Cleanup (RAII via destructor)

Everything else is either:

- **Configuration** (provided upfront via config files)
- **Query/Introspection** (accessed via a separate interface)
- **Advanced/Expert** (hidden behind accessor methods)

---

## New Class Structure

```
┌─────────────────────────────────────────────────────────────┐
│                    SimulatorConfig                          │
│  - component configs, routes, integrator, logging, etc.     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      Simulator<Scalar>                       │
│  ┌─────────────────────────────────────────────────────────┐│
│  │              PUBLIC (External API - 4 ops)              ││
│  │  - FromConfig(path) / FromConfig(SimulatorConfig)       ││
│  │  - Stage()                                              ││
│  │  - Step(dt)                                             ││
│  │  - ~Simulator()                                         ││
│  └─────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────┐│
│  │              QUERY (Read-only access)                   ││
│  │  - Time() const                                         ││
│  │  - GetPhase() const                                     ││
│  │  - Peek<T>(name) const                                  ││
│  │  - GetDataDictionary() const                            ││
│  │  - IsInitialized() const                                ││
│  └─────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────┐│
│  │              CONTROL (Runtime modification)             ││
│  │  - Poke<T>(name, value)                                 ││
│  │  - Reset()                                              ││
│  │  - SetInputSource(name, callback)                       ││
│  │  - SetOutputObserver(name, callback)                    ││
│  └─────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────┐│
│  │              EXPERT (Advanced users only)               ││
│  │  - GetBackplane() → for signal introspection            ││
│  │  - GetState() / SetState() → for optimization           ││
│  │  - GenerateGraph() → symbolic mode                      ││
│  │  - AdaptiveStep() → advanced integration                ││
│  └─────────────────────────────────────────────────────────┘│
│                                                             │
│  ┌─────────────────────────────────────────────────────────┐│
│  │              PRIVATE (Internal subsystems)              ││
│  │  - ComponentManager                                     ││
│  │  - SignalManager (backplane + registry)                 ││
│  │  - StateManager (X, X_dot, layout)                      ││
│  │  - IntegrationManager (integrator, config)              ││
│  │  - LogManager (logger, config)                          ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

---

## Refactored Simulator Class

```cpp
namespace icarus {

/**
 * @brief Top-level simulation coordinator
 *
 * NOT templated on Scalar - user sees one class.
 * Symbolic mode is used internally during Stage() for:
 *   - Trim optimization
 *   - Linearization
 *   - Symbolic graph generation
 * Numeric mode is used during Step() for deterministic execution.
 *
 * Provides a clean 4-operation external interface:
 * - FromConfig() / constructor - Initialize simulation
 * - Stage() - Prepare for execution (trim, linearization, validation)
 * - Step() - Advance simulation (numeric)
 * - ~Simulator() - Cleanup (RAII)
 *
 * Additional functionality is grouped into:
 * - Query methods (read-only introspection)
 * - Control methods (runtime modification)
 * - Expert methods (advanced users)
 */
class Simulator {
public:
    // =========================================================================
    // CORE LIFECYCLE (The 4 Operations)
    // =========================================================================

    /**
     * @brief Create simulator from configuration file
     *
     * This is the primary entry point. Loads component configs and routes
     * from YAML, creates components, provisions them.
     *
     * Single-file mode: Everything can be inline in one YAML.
     * Multi-file mode: Use includes for organization (optional).
     *
     * @param config_path Path to simulation YAML configuration
     * @return Configured Simulator ready for Stage()
     */
    [[nodiscard]] static Simulator FromConfig(const std::string& config_path);

    /**
     * @brief Create simulator from configuration struct
     */
    [[nodiscard]] static Simulator FromConfig(const SimulatorConfig& config);

    /**
     * @brief Default constructor for programmatic setup
     *
     * Use AddComponent() and Configure() before Stage().
     */
    Simulator() = default;

    /**
     * @brief Stage the simulation
     *
     * "Staging" prepares the vehicle for launch:
     * - Validates all signal wiring
     * - Runs trim optimization (if configured) - uses symbolic mode internally
     * - Computes linearization (if configured) - uses symbolic mode internally
     * - Generates symbolic dynamics graph (if configured)
     *
     * Must be called after FromConfig() (which calls Provision internally).
     * Must be called before Step().
     *
     * @param config Optional staging config override
     */
    void Stage();
    void Stage(const StageConfig& config);

    /**
     * @brief Execute one simulation step (numeric mode)
     *
     * Advances time by dt using the configured integrator.
     * This is deterministic frame-by-frame numeric execution.
     *
     * @param dt Timestep (uses nominal dt if not specified)
     */
    void Step(double dt);
    void Step();  // Uses nominal dt

    /**
     * @brief Destructor - cleanup handled via RAII
     */
    ~Simulator() = default;

    // Move-only (no copy)
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    Simulator(Simulator&&) = default;
    Simulator& operator=(Simulator&&) = default;

    // =========================================================================
    // QUERY INTERFACE (Read-only)
    // =========================================================================

    [[nodiscard]] double Time() const;
    [[nodiscard]] Phase GetPhase() const;
    [[nodiscard]] bool IsInitialized() const;

    template <typename T>
    [[nodiscard]] T Peek(const std::string& name) const;

    [[nodiscard]] DataDictionary GetDataDictionary() const;

    // =========================================================================
    // CONTROL INTERFACE (Runtime modification)
    // =========================================================================

    template <typename T>
    void Poke(const std::string& name, const T& value);

    void Reset();

    void SetInputSource(const std::string& signal_name,
                        std::function<double(const std::string&)> callback);
    void SetOutputObserver(const std::string& signal_name,
                           std::function<void(const std::string&, double)> callback);
    void ClearInputSource(const std::string& signal_name);
    void ClearOutputObserver(const std::string& signal_name);

    // =========================================================================
    // EXPERT INTERFACE (Advanced users)
    // =========================================================================

    [[nodiscard]] Eigen::VectorXd GetState() const;
    void SetState(const Eigen::VectorXd& X);
    Eigen::VectorXd ComputeDerivatives(double t);
    AdaptiveStepResult AdaptiveStep(double dt_request);

    /**
     * @brief Get symbolic dynamics graph
     *
     * Available after Stage() if symbolics.enabled = true.
     * Returns the dynamics function f(t, x) -> x_dot as a CasADi function.
     */
    [[nodiscard]] janus::Function GetDynamicsGraph() const;

    /**
     * @brief Get symbolic Jacobian
     *
     * Available after Stage() if symbolics.generate_jacobian = true.
     */
    [[nodiscard]] janus::Function GetJacobian() const;

    /**
     * @brief Get linearized state-space model
     *
     * Available after Stage() if linearization.enabled = true.
     */
    [[nodiscard]] LinearModel GetLinearModel() const;

    // =========================================================================
    // PROGRAMMATIC SETUP (Alternative to FromConfig)
    // =========================================================================

    void AddComponent(const ComponentConfig& config);
    void Configure(const SimulatorConfig& config);
    void AddRoutes(const std::vector<signal::SignalRoute>& routes);

private:
    // Internal state
    SimulatorConfig config_;

    // Numeric components (for Step execution)
    std::vector<std::unique_ptr<Component<double>>> numeric_components_;
    SignalRegistry<double> numeric_registry_;
    Backplane<double> numeric_backplane_{numeric_registry_};

    // Symbolic components (for Stage analysis) - lazily created
    std::vector<std::unique_ptr<Component<SymbolicScalar>>> symbolic_components_;
    SignalRegistry<SymbolicScalar> symbolic_registry_;
    Backplane<SymbolicScalar> symbolic_backplane_{symbolic_registry_};
    bool symbolic_initialized_ = false;

    signal::SignalRouter router_;
    StateManager state_manager_;
    IntegrationManager integration_manager_;
    MissionLogger logger_;

    double time_ = 0.0;
    Phase phase_ = Phase::Uninitialized;

    // Results from Stage() - available after staging
    std::optional<janus::Function> dynamics_graph_;
    std::optional<janus::Function> jacobian_;
    std::optional<LinearModel> linear_model_;

    // External callbacks
    std::unordered_map<std::string, std::function<double(const std::string&)>> input_sources_;
    std::unordered_map<std::string, std::function<void(const std::string&, double)>> output_observers_;

    // Internal methods - Provision phase
    void ProvisionNumericComponents();
    void ProvisionSymbolicComponents();  // Called lazily during Stage if needed
    void ApplyRouting();
    void AllocateAndBindState();
    void ApplyInitialConditions();
    void ValidateWiring();

    // Internal methods - Stage phase (uses symbolic components internally)
    void RunTrimOptimization();      // Uses symbolic mode
    void ComputeLinearization();     // Uses symbolic mode
    void GenerateSymbolicGraphs();   // Uses symbolic mode

    // Internal methods - Step phase (uses numeric components)
    void InvokeInputSources();
    void InvokeOutputObservers();
};

} // namespace icarus
```

---

## Error Handling Strategy

Configuration and runtime errors must be **actionable**. Users should know:
1. **What** went wrong
2. **Where** in the config (file + line if applicable)
3. **How** to fix it (suggestions)

### Configuration Errors

```cpp
// Example error format
throw ConfigError(
    "Unknown component type 'PointMasGravity'",
    config_path,
    line_number,
    "Did you mean 'PointMassGravity'?"
);

// Output:
// Error: Unknown component type 'PointMasGravity'
//   at: simulation.yaml:15
//   hint: Did you mean 'PointMassGravity'?
```

### Signal Wiring Errors

```cpp
// Example: unwired input
throw WiringError(
    "Input 'Dynamics.gravity_accel' is not wired",
    "Available outputs matching 'gravity': ['Gravity.accel', 'Gravity.force']"
);

// Output:
// Error: Input 'Dynamics.gravity_accel' is not wired
//   hint: Available outputs matching 'gravity': ['Gravity.accel', 'Gravity.force']
```

### Runtime Errors

```cpp
// Example: component failure
throw ComponentError(
    "Atmosphere model returned NaN for density",
    "Component: Leader.Aero",
    "Time: 42.5s",
    "Position: [6.871e6, 0, 1e9]"  // Clearly invalid altitude
);
```

### Validation Levels

| Phase | Validation | Error Type |
|:------|:-----------|:-----------|
| `FromConfig()` | YAML syntax, component types, required fields | `ConfigError` |
| `Provision()` | Signal port existence, route validity | `WiringError` |
| `Stage()` | All inputs wired, trim convergence | `StageError` |
| `Step()` | NaN detection, bounds checking | `RuntimeError` |

### Integration with Vulcan

Vulcan's YAML loader provides basic syntax error handling. Icarus builds on top:
- Type checking (expected scalar, got array)
- Schema validation (required fields)
- Cross-reference validation (component X references non-existent Y)

---

## Lifecycle

```
┌──────────────────────────────────────────────────────────────┐
│  UNINITIALIZED                                               │
│    ↓                                                         │
│  FromConfig(path) or AddComponent() + Configure()            │
│    ↓                                                         │
│  CREATED (components exist but not wired)                    │
│    ↓                                                         │
│  Provision()  ← Called automatically by FromConfig()         │
│    ├── Components declare ports (inputs/outputs)             │
│    ├── Apply signal routing from config                      │
│    ├── Allocate global state vectors                         │
│    ├── Bind state pointers to components                     │
│    ├── Apply initial conditions from config                  │
│    └── Validate all inputs are wired                         │
│    ↓                                                         │
│  PROVISIONED (wired, state bound, ICs applied)               │
│    ↓                                                         │
│  Stage()  ← User calls this                                  │
│    ├── Generate symbolic graphs (if symbolic mode)           │
│    ├── Run trim optimization (find equilibrium)              │
│    ├── Apply trim results to state                           │
│    └── Prepare "staged" initial state                        │
│    ↓                                                         │
│  STAGED (ready to run - like a rocket on the pad)            │
│    ↓                                                         │
│  Step(dt) ... Step(dt) ... Step(dt)                          │
│    ↓                                                         │
│  RUNNING                                                     │
│    ↓                                                         │
│  Reset() → back to PROVISIONED (re-apply ICs, re-Stage)      │
│    ↓                                                         │
│  ~Simulator() → cleanup                                      │
└──────────────────────────────────────────────────────────────┘
```

**Key Insight:** "Stage" means "staging" the vehicle - like staging a rocket on the launch pad. This involves:

1. Finding the trim state (equilibrium initial conditions)
2. Generating symbolic graphs for analysis/optimization
3. Preparing the simulation for execution

This is separate from "Provision" which is the mechanical wiring/binding phase.

---

## Internal Manager Classes

### StateManager

```cpp
namespace icarus {

/**
 * @brief Manages global state vectors and component bindings
 */
template <typename Scalar>
class StateManager {
public:
    void AllocateState(const std::vector<std::unique_ptr<Component<Scalar>>>& components);
    void BindComponents(std::vector<std::unique_ptr<Component<Scalar>>>& components);
    void ZeroDerivatives();

    [[nodiscard]] std::size_t TotalSize() const;
    [[nodiscard]] JanusVector<Scalar> GetState() const;
    void SetState(const JanusVector<Scalar>& X);
    [[nodiscard]] const JanusVector<Scalar>& GetDerivatives() const;
    [[nodiscard]] const std::vector<StateSlice<Scalar>>& GetLayout() const;

private:
    JanusVector<Scalar> X_global_;
    JanusVector<Scalar> X_dot_global_;
    std::vector<StateSlice<Scalar>> layout_;
};

} // namespace icarus
```

### IntegrationManager

```cpp
namespace icarus {

/**
 * @brief Manages integrator lifecycle and stepping
 */
template <typename Scalar>
class IntegrationManager {
public:
    void Configure(const IntegratorConfig<Scalar>& config);

    JanusVector<Scalar> Step(
        std::function<JanusVector<Scalar>(Scalar, const JanusVector<Scalar>&)> deriv_func,
        const JanusVector<Scalar>& X,
        Scalar t,
        Scalar dt);

    AdaptiveStepResult<Scalar> AdaptiveStep(
        std::function<JanusVector<Scalar>(Scalar, const JanusVector<Scalar>&)> deriv_func,
        const JanusVector<Scalar>& X,
        Scalar t,
        Scalar dt_request);

    [[nodiscard]] IntegratorType Type() const;
    [[nodiscard]] const IntegratorConfig<Scalar>& Config() const;

private:
    std::unique_ptr<Integrator<Scalar>> integrator_;
    IntegratorConfig<Scalar> config_;
};

} // namespace icarus
```

---

## Removed/Hidden Methods

| Old Method | New Location/Approach |
|:-----------|:---------------------|
| `Provision()` | Private, called by `FromConfig()` |
| `Initialize()` | Removed, use `FromConfig()` + `Stage()` |
| `GetRegistry()` | Private, use `GetBackplane()` if needed |
| `GetSignal()/SetSignal()` | Use `Peek<Scalar>()/Poke<Scalar>()` |
| `PeekBatch()/PokeBatch()` | Removed (loop over Peek/Poke) |
| `SetTime()` | Private, time is managed internally |
| `GetSignalNames()` | Use `GetDataDictionary()` |
| `GetInputNames()` | Use `GetDataDictionary()` |
| `GetOutputNames()` | Use `GetDataDictionary()` |
| `NumInputSources()` etc. | Removed (internal detail) |
| `GetComponentOutputs()` | Use `GetDataDictionary()` |
| `GetComponentInputs()` | Use `GetDataDictionary()` |
| `SetWiring()` | Config-driven via routes file |
| `Wire<T>()` | Config-driven via routes file |
| `ValidateWiring()` | Called automatically in Stage() |
| `GetUnwiredInputs()` | Error thrown in Stage() if unwired |
| `LoadWiring()` | Config-driven via FromConfig() |
| `GetWiringConfig()` | Config-driven |
| `GenerateDataDictionary()` | Use `GetDataDictionary().ToYAML()` |
| `GetTotalStateSize()` | Use `GetState().size()` |
| `GetDerivatives()` | Expert: use ComputeDerivatives() |
| `GetStateLayout()` | Removed (internal detail) |
| `SetNominalDt()` | Configure via SimulatorConfig |
| `GetNominalDt()` | Access via config if needed |
| `SetIntegrator()` | Configure via SimulatorConfig |
| `GetIntegrator()` | Removed (internal detail) |
| `GetIntegratorConfig()` | Access via config if needed |
| `GetIntegratorType()` | Access via config if needed |
| `GetLogger()` | Private, logging is config-driven |
| `SetQuietMode()` | Configure via SimulatorConfig.logging |
| `SetLogFile()` | Configure via SimulatorConfig.logging |
| `SetProfilingEnabled()` | Configure via SimulatorConfig.logging |
| `ApplyLogConfig()` | Configure via SimulatorConfig.logging |
| `RegisterInputSource()` | `SetInputSource()` |
| `RegisterOutputObserver()` | `SetOutputObserver()` |
| `UnregisterInputSource()` | `ClearInputSource()` |
| `UnregisterOutputObserver()` | `ClearOutputObserver()` |

---

## Exit Criteria

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

### Internal Managers

- [ ] `StateManager<Scalar>` encapsulating state vector management
- [ ] `IntegrationManager<Scalar>` encapsulating integrator logic
