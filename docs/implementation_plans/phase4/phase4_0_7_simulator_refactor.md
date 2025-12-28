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
/**
 * @brief Trim optimization configuration
 *
 * Defines how the simulator finds equilibrium/trim conditions during Stage().
 */
template <typename Scalar>
struct TrimConfig {
    bool enabled = false;                    // Whether to run trim optimization

    // Trim targets: which derivatives should be zero?
    std::vector<std::string> zero_derivatives;  // e.g., ["velocity", "angular_velocity"]

    // Trim controls: which signals can be adjusted?
    std::vector<std::string> control_signals;   // e.g., ["throttle", "elevator"]

    // Optimization settings
    Scalar tolerance = Scalar{1e-6};
    int max_iterations = 100;
    std::string method = "newton";           // "newton", "gradient", "simplex"

    // Initial guesses for controls (optional)
    std::unordered_map<std::string, Scalar> initial_guesses;
};

/**
 * @brief Staging configuration
 *
 * Configures the Stage() phase: trim optimization and symbolic generation.
 */
template <typename Scalar>
struct StageConfig {
    // =========================================================================
    // Trim Configuration
    // =========================================================================
    TrimConfig<Scalar> trim;

    // =========================================================================
    // Symbolic Mode Configuration
    // =========================================================================
    bool generate_symbolics = false;         // Generate symbolic dynamics graph
    bool generate_jacobian = false;          // Generate symbolic Jacobian
    std::string symbolic_output_dir = "";    // Where to export symbolic functions

    // =========================================================================
    // Validation
    // =========================================================================
    bool validate_wiring = true;             // Throw if unwired inputs exist
    bool warn_on_unwired = true;             // Log warning for unwired inputs
};

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
    Scalar t_start = Scalar{0.0};  // Start time

    // =========================================================================
    // Component Configuration
    // =========================================================================
    std::vector<ComponentConfig> components;

    // =========================================================================
    // Signal Routing
    // =========================================================================
    std::vector<signal::SignalRoute> routes;

    // =========================================================================
    // Initial Conditions (Config-driven ICs)
    // =========================================================================
    // Applied during Provision(), before trim optimization
    std::unordered_map<std::string, Scalar> initial_conditions;  // signal -> value

    // =========================================================================
    // Staging Configuration
    // =========================================================================
    StageConfig<Scalar> staging;

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
     * "Staging" prepares the vehicle for launch:
     * - Runs trim optimization to find equilibrium state (if configured)
     * - Generates symbolic dynamics graph (if symbolic mode)
     * - Generates symbolic Jacobian (if configured)
     * - Exports symbolic functions (if output dir specified)
     *
     * Must be called after FromConfig() (which calls Provision internally).
     * Must be called before Step().
     *
     * @param config Optional staging config override (uses SimulatorConfig.staging if not provided)
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
    // Internal Methods - Provision Phase
    // =========================================================================

    void ProvisionComponents();       // Call Provision() on all components
    void ApplyRouting();              // Wire signals via SignalRouter
    void AllocateAndBindState();      // Allocate X_global_, bind to components
    void ApplyInitialConditions();    // Apply config.initial_conditions
    void ValidateWiring();            // Check all inputs are wired

    // =========================================================================
    // Internal Methods - Stage Phase
    // =========================================================================

    void RunTrimOptimization();       // Find equilibrium state
    void GenerateSymbolicGraphs();    // Generate dynamics/Jacobian
    void ExportSymbolicFunctions();   // Write to symbolic_output_dir

    // =========================================================================
    // Internal Methods - Step Phase
    // =========================================================================

    void InvokeInputSources();        // Call external input callbacks
    void InvokeOutputObservers();     // Call external output callbacks
};

} // namespace icarus
```

---

### Step 4: Update Lifecycle

The new lifecycle has clear semantic separation:

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

### Step 5: YAML Schema for Staging

The staging configuration is fully YAML-driven:

```yaml
# rocket_sim.yaml - Complete simulation configuration

simulation:
  name: "Rocket 6DOF Test"
  dt: 0.01
  t_start: 0.0
  t_end: 100.0

# Components and their configs
components:
  - type: PointMassGravity
    name: Gravity
    entity: Rocket
    scalars:
      mu: 3.986004418e14

  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    vectors:
      initial_position: [0, 0, 6.471e6]
      initial_velocity: [7500, 0, 0]
      initial_attitude: [1, 0, 0, 0]

# Signal routing (can also be in separate file)
routes:
  - input: Rocket.EOM.gravity_accel
    output: Rocket.Gravity.accel
    gain: 1.0

# Initial conditions (applied during Provision, before trim)
initial_conditions:
  Rocket.GNC.throttle: 0.8
  Rocket.GNC.pitch_cmd: 0.0

# Staging configuration
staging:
  # Trim optimization settings
  trim:
    enabled: true
    method: newton           # newton, gradient, simplex
    tolerance: 1.0e-6
    max_iterations: 100

    # Which derivatives should be zero at trim?
    zero_derivatives:
      - Rocket.EOM.velocity_dot      # Steady velocity
      - Rocket.EOM.angular_rate_dot  # No angular acceleration

    # Which signals can be adjusted to achieve trim?
    control_signals:
      - Rocket.GNC.throttle          # Adjust thrust
      - Rocket.GNC.pitch_cmd         # Adjust pitch

    # Initial guesses for controls
    initial_guesses:
      Rocket.GNC.throttle: 0.7
      Rocket.GNC.pitch_cmd: 0.1

  # Symbolic generation (optional)
  generate_symbolics: false
  generate_jacobian: false
  symbolic_output_dir: "./symbolic_output"

  # Validation
  validate_wiring: true
  warn_on_unwired: true

# Integrator settings
integrator:
  type: RK4                  # Euler, RK4, RK45
  # For adaptive integrators:
  # abs_tol: 1.0e-6
  # rel_tol: 1.0e-3

# Logging settings
logging:
  console_level: Info
  file_enabled: true
  file_path: "sim.log"
  file_level: Debug
  progress_enabled: true
  profiling_enabled: false
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

Or for programmatic setup:

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

## Implementation Order

### Phase A: Configuration Infrastructure
1. **4.0.7a** Create `TrimConfig<Scalar>` struct
2. **4.0.7b** Create `StageConfig<Scalar>` struct
3. **4.0.7c** Create `SimulatorConfig<Scalar>` struct with YAML loading

### Phase B: Internal Managers
4. **4.0.7d** Create `StateManager<Scalar>` class
5. **4.0.7e** Create `IntegrationManager<Scalar>` class

### Phase C: Simulator Core Refactor
6. **4.0.7f** Refactor `Simulator` public API (4 core operations)
7. **4.0.7g** Implement `FromConfig()` factory method
8. **4.0.7h** Move `Provision()` to private, wire routing/state/ICs
9. **4.0.7i** Implement new `Stage()` with trim/symbolic placeholders

### Phase D: Trim Optimization
10. **4.0.7j** Implement `RunTrimOptimization()` using Janus
11. **4.0.7k** Add Newton method for trim solver

### Phase E: Symbolic Mode
12. **4.0.7l** Implement `GenerateSymbolicGraphs()`
13. **4.0.7m** Implement `ExportSymbolicFunctions()`

### Phase F: Testing & Cleanup
14. **4.0.7n** Update existing tests to new API
15. **4.0.7o** Add integration test: YAML → FromConfig → Stage → Step
16. **4.0.7p** Add trim optimization tests
17. **4.0.7q** Remove deprecated public methods
18. **4.0.7r** Update documentation

---

## Exit Criteria

### Configuration Structs
- [ ] `TrimConfig<Scalar>` with optimization settings
- [ ] `StageConfig<Scalar>` with trim + symbolic settings
- [ ] `SimulatorConfig<Scalar>` with all configuration consolidated
- [ ] `SimulatorConfig::FromFile()` / `FromFiles()` YAML loading

### Internal Managers
- [ ] `StateManager<Scalar>` encapsulating state vector management
- [ ] `IntegrationManager<Scalar>` encapsulating integrator logic

### Simulator Refactor
- [ ] Clean 4-operation core API: `FromConfig()`, `Stage()`, `Step()`, `~Simulator()`
- [ ] Query interface: `Time()`, `Peek<T>()`, `GetDataDictionary()`
- [ ] Control interface: `Poke<T>()`, `Reset()`, `SetInputSource()`
- [ ] Expert interface: `GetState()`, `GenerateGraph()`, `AdaptiveStep()`

### Lifecycle Implementation
- [ ] `FromConfig()` calls `Provision()` internally
- [ ] `Provision()` handles: components, routing, state binding, ICs, validation
- [ ] `Stage()` handles: trim optimization, symbolic generation
- [ ] `Step()` handles: integration, callbacks

### Trim Optimization
- [ ] `RunTrimOptimization()` using Janus optimization
- [ ] Newton method for trim
- [ ] Configurable zero_derivatives and control_signals

### Symbolic Mode
- [ ] `GenerateSymbolicGraphs()` for dynamics and Jacobian
- [ ] `ExportSymbolicFunctions()` to output directory

### Testing
- [ ] Existing tests updated to new API
- [ ] New integration test: YAML → FromConfig → Stage → Step
- [ ] Trim optimization unit test
- [ ] Symbolic generation test

### Cleanup
- [ ] Old deprecated methods removed or made private
- [ ] Documentation updated

---

## Compatibility Notes

- The refactored API is **not backward compatible**
- A compatibility shim could be provided but is not recommended
- Phase 4.0.9 (component refactoring) should use the new API
- Phase 4.0.12 (integration test) should test the new API exclusively
