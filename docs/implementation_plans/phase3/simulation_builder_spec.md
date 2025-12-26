# SimulationBuilder & External Interface Technical Specification

**Component:** Standardized Setup & Control Framework
**Phase:** 3.1
**Status:** Proposed

---

## Motivation

Current examples (`orbital_demo.cpp`, `formation_flight_demo.cpp`) duplicate significant boilerplate:

```cpp
// Current pattern (repeated in every example)
Simulator<double> sim;
sim.SetProfilingEnabled(true);
sim.SetLogFile("example.log");
auto &logger = sim.GetLogger();
logger.LogStartup();
logger.BeginPhase(SimPhase::Provision);
// ... add components ...
sim.Provision();
logger.LogStateAllocation(sim.GetTotalStateSize());
logger.LogManifest(sim.GetDataDictionary());
logger.EndPhase();
logger.BeginPhase(SimPhase::Stage);
// ... wire components ...
sim.Stage();
logger.EndPhase();
logger.BeginPhase(SimPhase::Run);
while (sim.Time() < t_end) {
    // ... progress display ...
    sim.Step(dt);
}
logger.EndPhase(sim.Time());
// ... shutdown ...
```

This pattern is:
1. **Error-prone**: Easy to miss lifecycle calls
2. **Inconsistent**: Each example implements differently
3. **Not portable**: Doesn't work cleanly with symbolic mode
4. **Verbose**: Obscures the actual simulation configuration

---

## Design Goals

1. **Fluent API**: Chain configuration calls for readable setup
2. **Mode-agnostic**: Identical code for `<double>` and `<SymbolicScalar>`
3. **Automatic lifecycle**: Builder handles Provision/Stage sequencing
4. **Optional logging**: Clean output in symbolic mode
5. **User retains control**: External code can step, peek, poke signals
6. **Binding-friendly**: API suitable for Python/MATLAB/C FFI

---

## Key Design Principle: Don't Hide the Simulator

**CRITICAL**: The builder is a *setup convenience*, not an execution wrapper.

External bindings (Python, MATLAB, C) require:
- **Step-by-step control**: External loop drives simulation
- **Signal peek/poke**: Read/write signals between steps
- **State access**: Get/set full state vector
- **Control injection**: Feed external control signals
- **Observer output**: Export signals to external observers

The `SimulationRunner` is OPTIONAL—only for "run to completion" convenience.
For external bindings, use `Simulator` directly after builder setup.

---

## SimulationBuilder

### Interface

```cpp
#pragma once

#include <icarus/sim/Simulator.hpp>
#include <icarus/sim/Integrator.hpp>
#include <icarus/core/Component.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Fluent builder for Simulator configuration
 *
 * Usage:
 *   auto sim = SimulationBuilder<double>()
 *       .AddComponent(std::make_unique<Gravity>("Gravity"))
 *       .AddComponent(std::make_unique<Vehicle>(mass, "Vehicle"))
 *       .Wire("Gravity", {{"position.x", "Vehicle.position.x"}})
 *       .Wire("Vehicle", {{"force.x", "Gravity.force.x"}})
 *       .SetIntegrator<RK4Integrator>()
 *       .EnableProfiling()
 *       .Build();
 */
template <typename Scalar>
class SimulationBuilder {
public:
    SimulationBuilder() = default;

    // =========================================================================
    // Component Configuration
    // =========================================================================

    /**
     * @brief Add a component to the simulation
     * @param component Owned component instance
     * @return Reference to builder for chaining
     *
     * Components are added in order; execution order follows addition order.
     */
    SimulationBuilder& AddComponent(std::unique_ptr<Component<Scalar>> component);

    /**
     * @brief Convenience: construct component in-place
     * @tparam ComponentT Component type to construct
     * @tparam Args Constructor argument types
     * @param args Constructor arguments forwarded to ComponentT
     */
    template <typename ComponentT, typename... Args>
    SimulationBuilder& AddComponent(Args&&... args);

    // =========================================================================
    // Wiring Configuration
    // =========================================================================

    /**
     * @brief Wire inputs for a target component
     * @param target_component Component name (with optional entity prefix)
     * @param wiring Map of input_name -> source_signal_name
     * @return Reference to builder for chaining
     *
     * Example:
     *   .Wire("Gravity", {
     *       {"position.x", "Vehicle.position.x"},
     *       {"mass", "Vehicle.mass"}
     *   })
     */
    SimulationBuilder& Wire(const std::string& target_component,
                            const std::map<std::string, std::string>& wiring);

    /**
     * @brief Wire a single input
     * @param target_input Full path: "Component.input_name"
     * @param source_output Full path: "Component.output_name"
     */
    SimulationBuilder& Wire(const std::string& target_input,
                            const std::string& source_output);

    // =========================================================================
    // Integrator Configuration
    // =========================================================================

    /**
     * @brief Set the integrator instance
     * @param integrator Owned integrator instance
     */
    SimulationBuilder& SetIntegrator(std::unique_ptr<Integrator<Scalar>> integrator);

    /**
     * @brief Convenience: construct integrator in-place
     * @tparam IntegratorT Integrator type (e.g., RK4Integrator)
     * @tparam Args Constructor argument types
     */
    template <typename IntegratorT, typename... Args>
    SimulationBuilder& SetIntegrator(Args&&... args);

    // =========================================================================
    // Simulation Parameters
    // =========================================================================

    /**
     * @brief Set the default time step
     * @param dt Time step in seconds
     */
    SimulationBuilder& SetTimeStep(Scalar dt);

    /**
     * @brief Set the simulation end time
     * @param t_end End time in seconds
     */
    SimulationBuilder& SetEndTime(Scalar t_end);

    // =========================================================================
    // Logging & Profiling
    // =========================================================================

    /**
     * @brief Enable component execution profiling
     */
    SimulationBuilder& EnableProfiling(bool enable = true);

    /**
     * @brief Set log file path
     * @param path Path to log file (empty disables file logging)
     */
    SimulationBuilder& SetLogFile(const std::string& path);

    /**
     * @brief Disable all logging (useful for symbolic mode)
     */
    SimulationBuilder& DisableLogging();

    // =========================================================================
    // Build
    // =========================================================================

    /**
     * @brief Build the configured simulator
     * @return Fully constructed Simulator (components added, NOT provisioned)
     *
     * After Build():
     * - All components added to simulator
     * - Wiring configuration stored (applied at Stage)
     * - Integrator set
     *
     * Caller must still call Provision() and Stage(), or use SimulationRunner.
     */
    Simulator<Scalar> Build();

    /**
     * @brief Build and initialize (Provision + Stage)
     * @return Ready-to-run Simulator
     */
    Simulator<Scalar> BuildAndInitialize();

private:
    std::vector<std::unique_ptr<Component<Scalar>>> components_;
    std::vector<std::pair<std::string, std::map<std::string, std::string>>> wiring_;
    std::unique_ptr<Integrator<Scalar>> integrator_;

    Scalar dt_ = Scalar{0.01};
    Scalar t_end_ = Scalar{1.0};
    bool profiling_enabled_ = false;
    std::string log_file_;
    bool logging_disabled_ = false;
};

} // namespace icarus
```

### Configuration File Loading (Phase 5 Forward-Compatible)

The builder supports both programmatic and file-based configuration.
See [13_configuration.md](../../architecture/13_configuration.md) for the 6-layer config architecture.

```cpp
template <typename Scalar>
class SimulationBuilder {
public:
    // =========================================================================
    // Configuration Loading (references 13_configuration.md layers)
    // =========================================================================

    /**
     * @brief Load scenario configuration (Layer B)
     * @param path Path to scenario YAML/JSON
     *
     * Scenario files reference:
     * - Entity definitions (Layer A')
     * - Component parameters (Layer A)
     * - Wiring (Layer C, embedded or separate)
     * - Scheduler config (Layer D, embedded or separate)
     *
     * Example:
     *   auto sim = SimulationBuilder<double>()
     *       .LoadScenario("scenarios/x15_mission.yaml")
     *       .BuildAndInitialize();
     */
    SimulationBuilder& LoadScenario(const std::string& path);

    /**
     * @brief Load services configuration (Layer F)
     * @param path Path to services YAML/JSON
     *
     * Configures: logging, recording, telemetry, debug mode.
     */
    SimulationBuilder& LoadServices(const std::string& path);

    /**
     * @brief Load trim/optimization configuration (Layer E)
     * @param path Path to trim config YAML/JSON
     *
     * Configures: free variables, targets, solver settings.
     * Applied during Stage phase.
     */
    SimulationBuilder& LoadTrimConfig(const std::string& path);

    /**
     * @brief Set configuration search paths
     * @param paths Directories to search for referenced configs
     *
     * Entity definitions (Layer A') and component configs (Layer A)
     * are resolved relative to these paths.
     */
    SimulationBuilder& SetConfigPaths(const std::vector<std::string>& paths);

    // =========================================================================
    // Hybrid: Programmatic Overrides After Config Load
    // =========================================================================

    /**
     * @brief Override a parameter after loading config
     * @param signal_path Full path: "Entity.Component.param"
     * @param value New value
     *
     * Programmatic overrides take precedence over file config.
     * Useful for sweeps, sensitivity analysis, etc.
     */
    SimulationBuilder& OverrideParam(const std::string& signal_path, Scalar value);

    /**
     * @brief Override initial condition after loading config
     */
    SimulationBuilder& OverrideInitialCondition(const std::string& state_path, Scalar value);

    /**
     * @brief Add additional wiring beyond what's in config
     */
    SimulationBuilder& AddWiring(const std::string& source, const std::string& target);

    // ... existing programmatic API ...
};
```

**Design Philosophy:**

1. **Config-first, code-second**: Load base scenario from YAML, override programmatically
2. **Layered resolution**: Later layers override earlier (A → A' → B → programmatic)
3. **No magic**: All wiring is explicit, traceable via config provenance
4. **Hybrid workflows**: Monte Carlo sweeps load base config, override parameters in loop

**Usage Patterns:**

```cpp
// Pattern 1: Pure config-driven
auto sim = SimulationBuilder<double>()
    .LoadScenario("scenarios/x15_mission.yaml")
    .LoadServices("services/default.yaml")
    .BuildAndInitialize();

// Pattern 2: Config with programmatic overrides
auto sim = SimulationBuilder<double>()
    .LoadScenario("scenarios/x15_mission.yaml")
    .OverrideParam("X15.MainEngine.max_thrust", 80000.0)  // Sweep value
    .OverrideInitialCondition("X15.EOM.altitude", 20000.0)
    .BuildAndInitialize();

// Pattern 3: Pure programmatic (current examples)
auto sim = SimulationBuilder<double>()
    .AddComponent<PointMassGravity>("Gravity")
    .AddComponent<PointMass3DOF>(mass, "Vehicle")
    .Wire("Gravity", {{"position.x", "Vehicle.position.x"}})
    .BuildAndInitialize();

// Pattern 4: Hybrid - add components to config-loaded sim
auto sim = SimulationBuilder<double>()
    .LoadScenario("scenarios/base.yaml")
    .AddComponent<CustomController>("Controller")  // Add to existing
    .AddWiring("Controller.output", "Vehicle.control_input")
    .BuildAndInitialize();
```

### Implementation Notes

1. **Component Ownership**: Builder owns components until `Build()`, then transfers to Simulator
2. **Wiring Deferred**: Wiring stored as config, applied in `BuildAndInitialize()` after Provision
3. **Integrator Default**: If not specified, uses `RK4Integrator<Scalar>`
4. **Build vs BuildAndInitialize**:
   - `Build()`: Returns Simulator with components, caller handles lifecycle
   - `BuildAndInitialize()`: Calls Provision + applies wiring + Stage
5. **Config Loading Order**: Scenario → Services → Trim → Programmatic overrides

---

## Simulator External Interface API

**This is the core API for external bindings (Python, MATLAB, C).**

The Simulator class exposes methods for step-by-step control and signal access.
These are the primary interfaces for external code integration.

### Signal Access (Peek/Poke)

```cpp
template <typename Scalar>
class Simulator {
public:
    // =========================================================================
    // Signal Peek (Read)
    // =========================================================================

    /**
     * @brief Read a signal value by name
     * @tparam T Expected signal type (double, Vec3, etc.)
     * @param name Full signal path: "Entity.Component.signal"
     * @return Current signal value
     * @throws SignalNotFoundError if signal doesn't exist
     * @throws TypeMismatchError if T doesn't match signal type
     */
    template <typename T>
    T Peek(const std::string& name) const;

    /**
     * @brief Read signal as Scalar (most common case)
     */
    Scalar GetSignal(const std::string& name) const;

    /**
     * @brief Read multiple signals at once (batch read)
     * @param names Vector of signal names
     * @return Map of name -> value
     */
    std::map<std::string, Scalar> PeekBatch(const std::vector<std::string>& names) const;

    // =========================================================================
    // Signal Poke (Write)
    // =========================================================================

    /**
     * @brief Write a signal value by name
     * @tparam T Signal type
     * @param name Full signal path
     * @param value New value to set
     *
     * Use for:
     * - Injecting control inputs from external controller
     * - Overriding sensor values for testing
     * - Setting initial conditions
     */
    template <typename T>
    void Poke(const std::string& name, const T& value);

    /**
     * @brief Write signal as Scalar
     */
    void SetSignal(const std::string& name, Scalar value);

    /**
     * @brief Write multiple signals at once (batch write)
     */
    void PokeBatch(const std::map<std::string, Scalar>& values);

    // =========================================================================
    // State Vector Access
    // =========================================================================

    /**
     * @brief Get full state vector (for checkpointing, warmstart)
     */
    JanusVector<Scalar> GetState() const;

    /**
     * @brief Set full state vector
     * @param state New state vector (must match GetTotalStateSize())
     */
    void SetState(const JanusVector<Scalar>& state);

    /**
     * @brief Get state derivative vector (read-only, for analysis)
     */
    const JanusVector<Scalar>& GetStateDerivatives() const;

    /**
     * @brief Get total state size
     */
    size_t GetTotalStateSize() const;

    // =========================================================================
    // Step Control
    // =========================================================================

    /**
     * @brief Advance simulation by one time step
     * @param dt Time step size
     *
     * This is the primary integration point for external control loops.
     * External code calls Step() when ready, not on a fixed schedule.
     */
    void Step(Scalar dt);

    /**
     * @brief Get current simulation time
     */
    Scalar Time() const;

    /**
     * @brief Set simulation time (for warmstart/reset)
     */
    void SetTime(Scalar t);

    // =========================================================================
    // Lifecycle Control
    // =========================================================================

    /**
     * @brief Initialize simulation (Provision + Stage)
     * Convenience method combining both lifecycle phases.
     */
    void Initialize();

    /**
     * @brief Reset simulation to initial state
     * Keeps components, resets time to 0, restores initial conditions.
     */
    void Reset();

    /**
     * @brief Check if simulation is initialized
     */
    bool IsInitialized() const;
};
```

### External Input/Output Callbacks

For real-time or hardware-in-the-loop scenarios:

```cpp
template <typename Scalar>
class Simulator {
public:
    // =========================================================================
    // External Input Sources
    // =========================================================================

    /**
     * @brief Register callback to provide input signal values
     * @param signal_name Input signal to drive
     * @param callback Called at each Step() to get current value
     *
     * Example (joystick input):
     *   sim.RegisterInputSource("Vehicle.Control.throttle",
     *       []() { return joystick.GetThrottle(); });
     */
    void RegisterInputSource(const std::string& signal_name,
                             std::function<Scalar()> callback);

    /**
     * @brief Remove input source callback
     */
    void UnregisterInputSource(const std::string& signal_name);

    // =========================================================================
    // External Output Observers
    // =========================================================================

    /**
     * @brief Register callback to observe output signal values
     * @param signal_name Output signal to observe
     * @param callback Called after each Step() with current value
     *
     * Example (telemetry):
     *   sim.RegisterOutputObserver("Vehicle.position.x",
     *       [&telemetry](double x) { telemetry.Send("pos_x", x); });
     */
    void RegisterOutputObserver(const std::string& signal_name,
                                std::function<void(Scalar)> callback);

    /**
     * @brief Remove output observer callback
     */
    void UnregisterOutputObserver(const std::string& signal_name);

    // =========================================================================
    // Bulk Signal Lists (for binding discovery)
    // =========================================================================

    /**
     * @brief Get all signal names
     */
    std::vector<std::string> GetSignalNames() const;

    /**
     * @brief Get all input signal names (can be driven externally)
     */
    std::vector<std::string> GetInputNames() const;

    /**
     * @brief Get all output signal names (can be observed)
     */
    std::vector<std::string> GetOutputNames() const;

    /**
     * @brief Get all parameter names (can be tuned)
     */
    std::vector<std::string> GetParameterNames() const;
};
```

### Python Binding Example

```python
import icarus

# Build simulation
sim = icarus.SimulationBuilder() \
    .add_component(icarus.PointMassGravity("Gravity")) \
    .add_component(icarus.PointMass3DOF(mass=450000, name="Vehicle")) \
    .wire("Gravity", {"position.x": "Vehicle.position.x", ...}) \
    .wire("Vehicle", {"force.x": "Gravity.force.x", ...}) \
    .build_and_initialize()

# External control loop
dt = 0.1
while sim.time() < 1000.0:
    # Read outputs
    altitude = sim.peek("Vehicle.position.z") - R_earth
    velocity = sim.peek("Vehicle.velocity.z")

    # Compute control (external logic)
    throttle = my_controller.compute(altitude, velocity)

    # Inject control input
    sim.poke("Vehicle.Control.throttle", throttle)

    # Advance simulation
    sim.step(dt)

    # Log for analysis
    print(f"t={sim.time():.1f}, alt={altitude/1000:.1f} km")

# Get final state
final_state = sim.get_state()
```

### MATLAB Binding Example

```matlab
% Build simulation
sim = icarus.SimulationBuilder() ...
    .addComponent(icarus.PointMassGravity('Gravity')) ...
    .addComponent(icarus.PointMass3DOF('mass', 450000, 'name', 'Vehicle')) ...
    .wire('Gravity', struct('position_x', 'Vehicle.position.x')) ...
    .buildAndInitialize();

% External control loop
dt = 0.1;
while sim.time() < 1000.0
    % Read outputs
    altitude = sim.peek('Vehicle.position.z') - R_earth;

    % Your MATLAB controller
    throttle = myMatlabController(altitude);

    % Inject control
    sim.poke('Vehicle.Control.throttle', throttle);

    % Step
    sim.step(dt);
end
```

### C API (FFI-Compatible)

```c
// Phase 7 will provide this, but design for it now
typedef void* IcarusSim;

IcarusSim icarus_create();
int icarus_add_component(IcarusSim sim, const char* type, const char* name);
int icarus_wire(IcarusSim sim, const char* target, const char* input, const char* source);
int icarus_initialize(IcarusSim sim);

double icarus_peek(IcarusSim sim, const char* signal);
int icarus_poke(IcarusSim sim, const char* signal, double value);
int icarus_step(IcarusSim sim, double dt);
double icarus_time(IcarusSim sim);

void icarus_destroy(IcarusSim sim);
```

---

## SimulationRunner (Optional Convenience)

**Use Case**: "Run to completion" scenarios where you don't need step-by-step control.

For external bindings or control loops, use the Simulator External Interface API directly.
SimulationRunner is useful for:
- Batch simulations
- Monte Carlo runs
- Quick demos/examples
- Cases where external code doesn't need to inject controls

### Interface

```cpp
#pragma once

#include <icarus/sim/Simulator.hpp>
#include <icarus/sim/SimulationResults.hpp>

#include <functional>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Optional convenience wrapper for "run to completion" scenarios
 *
 * NOTE: For external bindings (Python, MATLAB, C), use Simulator directly.
 * SimulationRunner is for cases where you want to:
 * - Run a batch simulation without step-by-step control
 * - Get automatic progress display and timing
 * - Collect results in a structured format
 *
 * Usage:
 *   SimulationRunner runner(sim);
 *   runner.SetProgressCallback([](double t, double t_end) {
 *       std::cout << "Progress: " << (t/t_end*100) << "%\n";
 *   });
 *   runner.Run(dt, t_end);
 *   auto results = runner.GetResults();
 */
template <typename Scalar>
class SimulationRunner {
public:
    using ProgressCallback = std::function<void(Scalar t, Scalar t_end)>;
    using StepCallback = std::function<void(Scalar t)>;
    using SignalRecorder = std::function<void(Scalar t, const Simulator<Scalar>&)>;

    /**
     * @brief Construct runner for a simulator
     * @param sim Reference to simulator (must outlive runner)
     */
    explicit SimulationRunner(Simulator<Scalar>& sim);

    // =========================================================================
    // Configuration
    // =========================================================================

    /**
     * @brief Set callback for progress updates
     * @param callback Called after each step with (t, t_end)
     */
    SimulationRunner& SetProgressCallback(ProgressCallback callback);

    /**
     * @brief Set callback for custom per-step logic
     * @param callback Called after each step with current time
     */
    SimulationRunner& SetStepCallback(StepCallback callback);

    /**
     * @brief Set callback for recording signals
     * @param callback Called after each step to record signal values
     * @param interval Record every N steps (default: 1)
     */
    SimulationRunner& SetRecorder(SignalRecorder callback, size_t interval = 1);

    /**
     * @brief Enable default console progress bar
     */
    SimulationRunner& EnableProgressBar(bool enable = true);

    // =========================================================================
    // Execution
    // =========================================================================

    /**
     * @brief Initialize simulation (Provision + Stage)
     *
     * Call before Run() if simulator was built with Build() not BuildAndInitialize().
     */
    void Initialize();

    /**
     * @brief Run simulation loop
     * @param dt Time step
     * @param t_end End time
     */
    void Run(Scalar dt, Scalar t_end);

    /**
     * @brief Run simulation with default dt and t_end from builder
     */
    void Run();

    /**
     * @brief Perform shutdown and generate debrief
     */
    void Shutdown();

    // =========================================================================
    // Results
    // =========================================================================

    /**
     * @brief Get simulation results
     * @return Results structure with final state, timing, etc.
     */
    SimulationResults<Scalar> GetResults() const;

    /**
     * @brief Get recorded signal history
     * @return Vector of (time, signal_values) pairs
     */
    const std::vector<std::pair<Scalar, std::map<std::string, Scalar>>>& GetHistory() const;

private:
    Simulator<Scalar>& sim_;
    ProgressCallback progress_callback_;
    StepCallback step_callback_;
    SignalRecorder recorder_;
    size_t record_interval_ = 1;
    bool show_progress_bar_ = false;
    bool initialized_ = false;

    std::vector<std::pair<Scalar, std::map<std::string, Scalar>>> history_;
    SimulationResults<Scalar> results_;
};

} // namespace icarus
```

### SimulationResults Structure

```cpp
#pragma once

#include <icarus/core/Types.hpp>

#include <chrono>
#include <map>
#include <string>

namespace icarus {

/**
 * @brief Results from a simulation run
 */
template <typename Scalar>
struct SimulationResults {
    // Timing
    Scalar sim_time_final;           ///< Final simulation time
    double wall_time_seconds;        ///< Wall clock duration
    double realtime_factor;          ///< sim_time / wall_time

    // State
    std::map<std::string, Scalar> final_signals;  ///< Signal values at end

    // Statistics
    size_t total_steps;              ///< Number of Step() calls
    size_t integrator_evaluations;   ///< Derivative evaluations (for adaptive)

    // Status
    bool completed;                  ///< True if ran to t_end
    std::string termination_reason;  ///< "completed", "error", "stopped"
};

} // namespace icarus
```

---

## Usage Examples

### Basic Numeric Simulation

```cpp
#include <icarus/sim/SimulationBuilder.hpp>
#include <icarus/sim/SimulationRunner.hpp>
#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

int main() {
    auto sim = SimulationBuilder<double>()
        .AddComponent<PointMassGravity>("Gravity")
        .AddComponent<PointMass3DOF>(450000.0, "Vehicle")
        .Wire("Gravity", {{"position.x", "Vehicle.position.x"},
                          {"position.y", "Vehicle.position.y"},
                          {"position.z", "Vehicle.position.z"},
                          {"mass", "Vehicle.mass"}})
        .Wire("Vehicle", {{"force.x", "Gravity.force.x"},
                          {"force.y", "Gravity.force.y"},
                          {"force.z", "Gravity.force.z"}})
        .SetIntegrator<RK4Integrator>()
        .EnableProfiling()
        .BuildAndInitialize();

    SimulationRunner runner(sim);
    runner.EnableProgressBar()
          .Run(10.0, 1500.0);  // dt=10s, t_end=1500s

    auto results = runner.GetResults();
    std::cout << "Completed in " << results.wall_time_seconds << "s\n";
    std::cout << "Realtime factor: " << results.realtime_factor << "x\n";
}
```

### Symbolic Mode

```cpp
#include <icarus/sim/SimulationBuilder.hpp>
#include <icarus/symbolic/SymbolicTracer.hpp>

int main() {
    using Scalar = icarus::SymbolicScalar;

    // Identical setup, different template parameter
    auto sim = SimulationBuilder<Scalar>()
        .AddComponent<PointMassGravity<Scalar>>("Gravity")
        .AddComponent<PointMass3DOF<Scalar>>(Scalar{450000.0}, "Vehicle")
        .Wire("Gravity", {{"position.x", "Vehicle.position.x"}, /* ... */})
        .Wire("Vehicle", {{"force.x", "Gravity.force.x"}, /* ... */})
        .DisableLogging()  // No console output for symbolic
        .BuildAndInitialize();

    // Extract dynamics as CasADi function
    auto dynamics = sim.GenerateGraph();
    std::cout << "Dynamics: " << dynamics << "\n";

    // Get Jacobian for optimization
    auto jacobian = dynamics.jacobian("x", "xdot");
    std::cout << "Jacobian: " << jacobian << "\n";
}
```

### Formation Flight with Entity Factory

```cpp
// Helper to create entity components
template <typename Scalar>
void AddSatellite(SimulationBuilder<Scalar>& builder,
                  const std::string& entity,
                  Scalar mass,
                  const Vec3<Scalar>& pos,
                  const Vec3<Scalar>& vel) {
    builder
        .AddComponent<PointMassGravity<Scalar>>("Gravity", entity)
        .AddComponent<PointMass3DOF<Scalar>>(mass, "Dynamics", entity);

    // Set initial conditions
    // (This requires builder to support init config - future enhancement)

    // Wire within entity
    std::string g = entity + ".Gravity";
    std::string d = entity + ".Dynamics";
    builder
        .Wire(g, {{"position.x", d + ".position.x"}, /* ... */})
        .Wire(d, {{"force.x", g + ".force.x"}, /* ... */});
}

int main() {
    SimulationBuilder<double> builder;

    AddSatellite(builder, "Leader", 500.0, {r0, 0, 0}, {0, v_circ, 0});
    AddSatellite(builder, "Follower", 500.0, {r0, -100, 0}, {0, v_circ, 0});

    auto sim = builder.EnableProfiling().BuildAndInitialize();
    SimulationRunner(sim).Run(10.0, 600.0);
}
```

---

## Implementation Priority

1. **SimulationBuilder** (core): Enables fluent configuration
2. **SimulationRunner** (convenience): Manages lifecycle
3. **SimulationResults** (utility): Structured output

Builder is essential for Phase 3 symbolic validation; Runner is quality-of-life improvement.

---

## Testing Strategy

```cpp
// tests/sim/simulation_builder_test.cpp

TEST(SimulationBuilder, BasicConstruction) {
    auto sim = SimulationBuilder<double>()
        .AddComponent(std::make_unique<DummyComponent>("Test"))
        .Build();

    EXPECT_EQ(sim.GetComponentCount(), 1);
}

TEST(SimulationBuilder, WiringApplied) {
    auto sim = SimulationBuilder<double>()
        .AddComponent(std::make_unique<Source>("Source"))
        .AddComponent(std::make_unique<Sink>("Sink"))
        .Wire("Sink", {{"input", "Source.output"}})
        .BuildAndInitialize();

    // Verify wiring resolved correctly
    EXPECT_NO_THROW(sim.GetSignal("Sink.input"));
}

TEST(SimulationBuilder, SymbolicInstantiation) {
    using Scalar = SymbolicScalar;

    // This must compile
    auto sim = SimulationBuilder<Scalar>()
        .AddComponent(std::make_unique<PointMassGravity<Scalar>>("G"))
        .Build();
}
```
