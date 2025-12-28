# Phase 4.0.7: Simulator Refactor Plan

**Status:** Proposed
**Prerequisite:** Phase 4.0.1-4.0.6 complete

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

### Current Public API (Problematic)

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

## Proposed Architecture

### Design Principle: Clean External Interface

The external user should only need to understand 4 operations:

1. **Init** - Load configuration, create components
2. **Stage** - Wire signals, apply ICs, prepare for simulation
3. **Step** - Advance simulation by dt
4. **Deinit** - Cleanup (RAII via destructor)

Everything else is either:
- **Configuration** (provided upfront via config files)
- **Query/Introspection** (accessed via a separate interface)
- **Advanced/Expert** (hidden behind accessor methods)

### New Class Structure

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

## Implementation Plan

### Step 1: Create SimulatorConfig

**File:** `include/icarus/sim/SimulatorConfig.hpp`

Consolidate all configuration into a single struct:

```cpp
namespace icarus {

/**
 * @brief Complete simulation configuration
 *
 * Loaded from YAML or constructed programmatically.
 * Passed to Simulator::FromConfig() for initialization.
 */
template <typename Scalar>
struct SimulatorConfig {
    // =========================================================================
    // Simulation Identity
    // =========================================================================
    std::string name = "Simulation";

    // =========================================================================
    // Time Configuration
    // =========================================================================
    Scalar dt = Scalar{0.01};      // Nominal timestep
    Scalar t_end = Scalar{100.0};  // End time (optional, for batch runs)

    // =========================================================================
    // Component Configuration
    // =========================================================================
    std::vector<ComponentConfig> components;

    // =========================================================================
    // Signal Routing
    // =========================================================================
    std::vector<signal::SignalRoute> routes;

    // =========================================================================
    // Integrator Configuration
    // =========================================================================
    IntegratorConfig<Scalar> integrator = IntegratorConfig<Scalar>::RK4Default();

    // =========================================================================
    // Logging Configuration
    // =========================================================================
    LogConfig logging;

    // =========================================================================
    // Factory Methods
    // =========================================================================

    /**
     * @brief Load configuration from YAML file
     */
    static SimulatorConfig FromFile(const std::string& config_path);

    /**
     * @brief Load configuration from YAML file with separate routes file
     */
    static SimulatorConfig FromFiles(const std::string& config_path,
                                      const std::string& routes_path);
};

} // namespace icarus
```

---

### Step 2: Create Internal Manager Classes

These encapsulate related functionality and keep Simulator focused on orchestration.

#### 2.1 StateManager

**File:** `include/icarus/sim/StateManager.hpp`

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

#### 2.2 IntegrationManager

**File:** `include/icarus/sim/IntegrationManager.hpp`

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

### Step 3: Refactor Simulator

**File:** `include/icarus/sim/Simulator.hpp` (refactored)

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
     * - Applies signal routing
     * - Allocates and binds state vectors
     * - Applies initial conditions
     * - Validates all inputs are wired
     *
     * Must be called before Step().
     */
    void Stage();

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

    /**
     * @brief Get current simulation time
     */
    [[nodiscard]] Scalar Time() const;

    /**
     * @brief Get simulation phase
     */
    [[nodiscard]] Phase GetPhase() const;

    /**
     * @brief Check if simulation is ready for stepping
     */
    [[nodiscard]] bool IsInitialized() const;

    /**
     * @brief Read a signal value
     */
    template <typename T>
    [[nodiscard]] T Peek(const std::string& name) const;

    /**
     * @brief Get data dictionary (full introspection)
     */
    [[nodiscard]] DataDictionary GetDataDictionary() const;

    // =========================================================================
    // CONTROL INTERFACE (Runtime modification)
    // =========================================================================

    /**
     * @brief Write a signal value
     */
    template <typename T>
    void Poke(const std::string& name, const T& value);

    /**
     * @brief Reset simulation to initial state
     */
    void Reset();

    /**
     * @brief Register external input source
     */
    void SetInputSource(const std::string& signal_name,
                        std::function<Scalar(const std::string&)> callback);

    /**
     * @brief Register output observer
     */
    void SetOutputObserver(const std::string& signal_name,
                           std::function<void(const std::string&, const Scalar&)> callback);

    /**
     * @brief Remove external input source
     */
    void ClearInputSource(const std::string& signal_name);

    /**
     * @brief Remove output observer
     */
    void ClearOutputObserver(const std::string& signal_name);

    // =========================================================================
    // EXPERT INTERFACE (Advanced users)
    // =========================================================================

    /**
     * @brief Access signal backplane (for advanced introspection)
     */
    [[nodiscard]] const Backplane<Scalar>& GetBackplane() const;

    /**
     * @brief Get/set state vector (for optimization, warmstart)
     */
    [[nodiscard]] JanusVector<Scalar> GetState() const;
    void SetState(const JanusVector<Scalar>& X);

    /**
     * @brief Compute derivatives without stepping
     */
    const JanusVector<Scalar>& ComputeDerivatives(Scalar t);

    /**
     * @brief Execute adaptive step (requires adaptive integrator)
     */
    AdaptiveStepResult<Scalar> AdaptiveStep(Scalar dt_request);

    /**
     * @brief Generate symbolic dynamics graph (symbolic mode only)
     */
    template <typename S = Scalar,
              typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
    [[nodiscard]] janus::Function GenerateGraph();

    /**
     * @brief Generate symbolic Jacobian (symbolic mode only)
     */
    template <typename S = Scalar,
              typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
    [[nodiscard]] janus::Function GenerateJacobian();

    // =========================================================================
    // PROGRAMMATIC SETUP (Alternative to FromConfig)
    // =========================================================================

    /**
     * @brief Add a component (before Stage)
     */
    void AddComponent(std::unique_ptr<Component<Scalar>> component);

    /**
     * @brief Add a component with its config (before Stage)
     */
    void AddComponent(std::unique_ptr<Component<Scalar>> component,
                      const ComponentConfig& config);

    /**
     * @brief Configure simulation settings (before Stage)
     */
    void Configure(const SimulatorConfig<Scalar>& config);

    /**
     * @brief Add signal routes (before Stage)
     */
    void AddRoutes(const std::vector<signal::SignalRoute>& routes);

private:
    // =========================================================================
    // Internal State
    // =========================================================================

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

    // =========================================================================
    // Internal Methods
    // =========================================================================

    void ProvisionComponents();
    void ApplyRouting();
    void AllocateAndBindState();
    void StageComponents();
    void ValidateWiring();
    void InvokeInputSources();
    void InvokeOutputObservers();
};

} // namespace icarus
```

---

### Step 4: Update Lifecycle

The new lifecycle is cleaner:

```
┌──────────────────────────────────────────────────────────────┐
│  UNINITIALIZED                                               │
│    ↓                                                         │
│  FromConfig(path) or AddComponent() + Configure()            │
│    ↓                                                         │
│  PROVISIONED (components created, ports declared)            │
│    ↓                                                         │
│  Stage()                                                     │
│    ├── Apply routing                                         │
│    ├── Allocate state                                        │
│    ├── Bind state to components                              │
│    ├── Apply initial conditions                              │
│    └── Validate wiring                                       │
│    ↓                                                         │
│  STAGED (ready to run)                                       │
│    ↓                                                         │
│  Step(dt) ... Step(dt) ... Step(dt)                          │
│    ↓                                                         │
│  RUNNING                                                     │
│    ↓                                                         │
│  Reset() → back to STAGED                                    │
│    ↓                                                         │
│  ~Simulator() → cleanup                                      │
└──────────────────────────────────────────────────────────────┘
```

---

## Removed/Hidden Methods

The following methods are removed from the public API:

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
// Create from config (one line!)
auto sim = Simulator<double>::FromConfig("rocket.yaml", "routes.yaml");

// Stage (applies ICs, wiring)
sim.Stage();

// Run
while (sim.Time() < 100.0) {
    sim.Step(0.01);
}
```

Or for programmatic setup:

```cpp
SimulatorConfig<double> config;
config.dt = 0.01;
config.integrator = IntegratorConfig<double>::RK4Default();
config.logging.file_path = "sim.log";

// Add components
config.components.push_back({.type = "PointMassGravity", .name = "Gravity", .entity = "Rocket"});

// Add routes
config.routes.push_back({.input_path = "...", .output_path = "...", .gain = 1.0});

auto sim = Simulator<double>::FromConfig(config);
sim.Stage();
sim.Step();
```

---

## Implementation Order

1. **4.0.7a** Create `SimulatorConfig` struct
2. **4.0.7b** Create `StateManager` internal class
3. **4.0.7c** Create `IntegrationManager` internal class
4. **4.0.7d** Refactor `Simulator` public API
5. **4.0.7e** Implement `FromConfig()` factory
6. **4.0.7f** Update lifecycle (Provision becomes private)
7. **4.0.7g** Update all examples and tests
8. **4.0.7h** Remove deprecated methods

---

## Exit Criteria

- [ ] `SimulatorConfig<Scalar>` struct with `FromFile()`, `FromFiles()`
- [ ] `StateManager<Scalar>` encapsulating state vector management
- [ ] `IntegrationManager<Scalar>` encapsulating integrator logic
- [ ] `Simulator<Scalar>` with clean 4-operation core API
- [ ] `Simulator::FromConfig()` factory method
- [ ] Existing tests updated to new API
- [ ] New integration test: YAML → FromConfig → Stage → Step
- [ ] Old deprecated methods removed or made private
- [ ] Documentation updated

---

## Compatibility Notes

- The refactored API is **not backward compatible**
- A compatibility shim could be provided but is not recommended
- Phase 4.0.9 (component refactoring) should use the new API
- Phase 4.0.12 (integration test) should test the new API exclusively
