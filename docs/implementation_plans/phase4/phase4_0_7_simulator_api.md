# Phase 4.0.7: Simulator API Refactor

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

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
 * Provides a clean 4-operation external interface:
 * - FromConfig() / constructor - Initialize simulation
 * - Stage() - Prepare for execution (wire signals, apply ICs)
 * - Step() - Advance simulation
 * - ~Simulator() - Cleanup (RAII)
 *
 * Additional functionality is grouped into:
 * - Query methods (read-only introspection)
 * - Control methods (runtime modification)
 * - Expert methods (advanced users)
 */
template <typename Scalar>
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
     * @param config_path Path to simulation YAML configuration
     * @param routes_path Optional path to signal routes YAML
     * @return Configured Simulator ready for Stage()
     */
    [[nodiscard]] static Simulator FromConfig(const std::string& config_path,
                                               const std::string& routes_path = "");

    /**
     * @brief Create simulator from configuration struct
     */
    [[nodiscard]] static Simulator FromConfig(const SimulatorConfig<Scalar>& config);

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
     * - Runs trim optimization to find equilibrium state (if configured)
     * - Generates symbolic dynamics graph (if symbolic mode)
     * - Generates symbolic Jacobian (if configured)
     * - Exports symbolic functions (if output dir specified)
     *
     * Must be called after FromConfig() (which calls Provision internally).
     * Must be called before Step().
     *
     * @param config Optional staging config override
     */
    void Stage();
    void Stage(const StageConfig<Scalar>& config);

    /**
     * @brief Execute one simulation step
     *
     * Advances time by dt using the configured integrator.
     *
     * @param dt Timestep (uses nominal dt if not specified)
     */
    void Step(Scalar dt);
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

    [[nodiscard]] Scalar Time() const;
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
                        std::function<Scalar(const std::string&)> callback);
    void SetOutputObserver(const std::string& signal_name,
                           std::function<void(const std::string&, const Scalar&)> callback);
    void ClearInputSource(const std::string& signal_name);
    void ClearOutputObserver(const std::string& signal_name);

    // =========================================================================
    // EXPERT INTERFACE (Advanced users)
    // =========================================================================

    [[nodiscard]] const Backplane<Scalar>& GetBackplane() const;
    [[nodiscard]] JanusVector<Scalar> GetState() const;
    void SetState(const JanusVector<Scalar>& X);
    const JanusVector<Scalar>& ComputeDerivatives(Scalar t);
    AdaptiveStepResult<Scalar> AdaptiveStep(Scalar dt_request);

    template <typename S = Scalar,
              typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
    [[nodiscard]] janus::Function GenerateGraph();

    template <typename S = Scalar,
              typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
    [[nodiscard]] janus::Function GenerateJacobian();

    // =========================================================================
    // PROGRAMMATIC SETUP (Alternative to FromConfig)
    // =========================================================================

    void AddComponent(std::unique_ptr<Component<Scalar>> component);
    void AddComponent(std::unique_ptr<Component<Scalar>> component,
                      const ComponentConfig& config);
    void Configure(const SimulatorConfig<Scalar>& config);
    void AddRoutes(const std::vector<signal::SignalRoute>& routes);

private:
    // Internal state
    SimulatorConfig<Scalar> config_;
    std::vector<std::unique_ptr<Component<Scalar>>> components_;
    std::unordered_map<Component<Scalar>*, ComponentConfig> component_configs_;

    SignalRegistry<Scalar> registry_;
    Backplane<Scalar> backplane_{registry_};
    signal::SignalRouter<Scalar> router_;

    StateManager<Scalar> state_manager_;
    IntegrationManager<Scalar> integration_manager_;
    MissionLogger logger_;

    Scalar time_{};
    Phase phase_ = Phase::Uninitialized;

    // External callbacks
    std::unordered_map<std::string, std::function<Scalar(const std::string&)>> input_sources_;
    std::unordered_map<std::string, std::function<void(const std::string&, const Scalar&)>> output_observers_;

    // Internal methods - Provision phase
    void ProvisionComponents();
    void ApplyRouting();
    void AllocateAndBindState();
    void ApplyInitialConditions();
    void ValidateWiring();

    // Internal methods - Stage phase
    void RunTrimOptimization();
    void GenerateSymbolicGraphs();
    void ExportSymbolicFunctions();

    // Internal methods - Step phase
    void InvokeInputSources();
    void InvokeOutputObservers();
};

} // namespace icarus
```

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
