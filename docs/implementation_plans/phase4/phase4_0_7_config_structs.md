# Phase 4.0.7: Configuration Structs

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Overview

This document defines all C++ configuration structs used in the Phase 4.0.7 refactor.

---

## SimulatorConfig (Master)

```cpp
namespace icarus {

/**
 * @brief Complete simulation configuration
 *
 * Loaded from simulation.yaml master binding file.
 * All sub-configs can be inline or referenced via includes.
 */
template <typename Scalar>
struct SimulatorConfig {
    // =========================================================================
    // Identity
    // =========================================================================
    std::string name = "Simulation";
    std::string version = icarus::Version();
    std::string description;

    // =========================================================================
    // Time
    // =========================================================================
    Scalar t_start = Scalar{0};
    Scalar t_end = Scalar{100};
    Scalar dt = Scalar{0.01};  // Note: may be auto-derived from scheduler

    // Reference epoch for time-varying models (atmosphere, gravity, etc.)
    // Components that need absolute time can read this
    double reference_epoch_jd = 2451545.0;  // J2000.0 default

    // =========================================================================
    // Components (flattened from entity expansion)
    // =========================================================================
    std::vector<ComponentConfig> components;

    // =========================================================================
    // Signal Routing (flattened from entity expansion)
    // =========================================================================
    std::vector<signal::SignalRoute> routes;

    // =========================================================================
    // Scheduler (unified from all entities)
    // =========================================================================
    SchedulerConfig<Scalar> scheduler;

    // =========================================================================
    // Staging (Trim + Linearization + Symbolics)
    // =========================================================================
    StageConfig<Scalar> staging;

    // =========================================================================
    // Integrator
    // =========================================================================
    IntegratorConfig<Scalar> integrator = IntegratorConfig<Scalar>::RK4Default();

    // =========================================================================
    // Logging
    // =========================================================================
    LogConfig logging;

    // =========================================================================
    // Output
    // =========================================================================
    OutputConfig output;

    // =========================================================================
    // Factory Methods
    // =========================================================================

    /**
     * @brief Load from master binding file
     *
     * Resolves all includes and merges configuration.
     */
    static SimulatorConfig FromFile(const std::string& path);

    /**
     * @brief Load from master + explicit routes file
     */
    static SimulatorConfig FromFiles(const std::string& master_path,
                                      const std::string& routes_path);

    /**
     * @brief Validate configuration consistency
     */
    std::vector<std::string> Validate() const;
};

} // namespace icarus
```

---

## TrimConfig

```cpp
namespace icarus {

/**
 * @brief Trim optimization configuration
 *
 * Defines how the simulator finds equilibrium/trim conditions during Stage().
 */
template <typename Scalar>
struct TrimConfig {
    bool enabled = false;                    // Whether to run trim optimization

    // Trim targets: which derivatives should be zero?
    std::vector<std::string> zero_derivatives;  // e.g., ["velocity_dot", "angular_rate_dot"]

    // Trim controls: which signals can be adjusted?
    std::vector<std::string> control_signals;   // e.g., ["throttle", "elevator"]

    // Optimization settings
    Scalar tolerance = Scalar{1e-6};
    int max_iterations = 100;
    std::string method = "newton";           // "newton", "gradient", "simplex"

    // Initial guesses for controls (optional)
    std::unordered_map<std::string, Scalar> initial_guesses;

    // Control bounds (optional)
    std::unordered_map<std::string, std::pair<Scalar, Scalar>> control_bounds;
};

} // namespace icarus
```

---

## LinearizationConfig

```cpp
namespace icarus {

/**
 * @brief Linearization configuration
 */
struct LinearizationConfig {
    bool enabled = false;

    // State variables (for A matrix rows/cols)
    std::vector<std::string> states;

    // Input variables (for B matrix cols)
    std::vector<std::string> inputs;

    // Output variables (for C matrix rows)
    std::vector<std::string> outputs;

    // Export options
    bool export_matlab = false;
    bool export_numpy = false;
    bool export_json = false;
    std::string output_dir;
};

} // namespace icarus
```

---

## SymbolicsConfig

```cpp
namespace icarus {

/**
 * @brief Symbolic generation configuration
 */
struct SymbolicsConfig {
    bool enabled = false;
    bool generate_dynamics = true;    // Generate f(x, u) symbolic graph
    bool generate_jacobian = false;   // Generate df/dx, df/du
    std::string output_dir;           // Where to export symbolic functions
};

} // namespace icarus
```

---

## StageConfig

```cpp
namespace icarus {

/**
 * @brief Staging configuration
 *
 * Configures the Stage() phase: trim optimization, linearization, symbolic generation.
 */
template <typename Scalar>
struct StageConfig {
    // =========================================================================
    // Trim Configuration
    // =========================================================================
    TrimConfig<Scalar> trim;

    // =========================================================================
    // Linearization Configuration
    // =========================================================================
    LinearizationConfig linearization;

    // =========================================================================
    // Symbolic Mode Configuration
    // =========================================================================
    SymbolicsConfig symbolics;

    // =========================================================================
    // Validation
    // =========================================================================
    bool validate_wiring = true;             // Throw if unwired inputs exist
    bool warn_on_unwired = true;             // Log warning for unwired inputs
};

} // namespace icarus
```

---

## SchedulerConfig

See [phase4_0_7_scheduler.md](phase4_0_7_scheduler.md) for full details.

```cpp
namespace icarus {

enum class SchedulingMode {
    Automatic,  // Topological sort from signal dependencies (Future TODO)
    Explicit    // User-defined execution order
};

/**
 * @brief A component member within a scheduler group
 */
struct GroupMember {
    std::string component;     // Component name
    int priority = 0;          // Execution order within group (lower = runs first)
};

/**
 * @brief A scheduler group with its own rate and priority
 */
struct SchedulerGroupConfig {
    std::string name;
    double rate_hz = 400.0;    // Must be integer divisor of simulation rate
    int priority = 0;          // Execution order relative to other groups
    std::vector<GroupMember> members;
};

struct TopologyConfig {
    SchedulingMode mode = SchedulingMode::Explicit;
    enum class CycleHandling { Error, Warn, BreakAtDelay };
    CycleHandling cycle_detection = CycleHandling::Error;
    bool log_order = true;
};

/**
 * @brief Per-entity scheduler configuration (group-based)
 *
 * Every component belongs to exactly one group.
 * Simulation dt is auto-derived from the fastest group across all entities.
 */
template <typename Scalar>
struct SchedulerConfig {
    std::vector<SchedulerGroupConfig> groups;
    TopologyConfig topology;

    // Computed at runtime: frame divisor for each group
    std::unordered_map<std::string, int> group_frame_divisors;

    static SchedulerConfig FromFile(const std::string& yaml_path);
    static SchedulerConfig Default();
    std::vector<std::string> Validate() const;
    double MaxRate() const;
};

} // namespace icarus
```

---

## IntegratorConfig

```cpp
namespace icarus {

enum class IntegratorType {
    Euler,
    RK4,
    RK45,
    DormandPrince,
    // ... others
};

template <typename Scalar>
struct IntegratorConfig {
    IntegratorType type = IntegratorType::RK4;

    // For adaptive integrators
    Scalar abs_tol = Scalar{1e-6};
    Scalar rel_tol = Scalar{1e-3};
    Scalar dt_min = Scalar{1e-10};
    Scalar dt_max = Scalar{1.0};

    // Factory methods
    static IntegratorConfig RK4Default() {
        return IntegratorConfig{.type = IntegratorType::RK4};
    }

    static IntegratorConfig RK45Default() {
        return IntegratorConfig{
            .type = IntegratorType::RK45,
            .abs_tol = Scalar{1e-6},
            .rel_tol = Scalar{1e-3}
        };
    }
};

} // namespace icarus
```

---

## LogConfig

```cpp
namespace icarus {

enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Off
};

struct LogConfig {
    // Console logging
    LogLevel console_level = LogLevel::Info;

    // File logging
    bool file_enabled = false;
    std::string file_path;
    LogLevel file_level = LogLevel::Debug;

    // Progress bar
    bool progress_enabled = true;

    // Profiling
    bool profiling_enabled = false;

    // Telemetry recording
    bool telemetry_enabled = false;
    std::string telemetry_path;
    std::vector<std::string> telemetry_signals;  // Which signals to record
};

} // namespace icarus
```

---

## OutputConfig

```cpp
namespace icarus {

/**
 * @brief Output/export configuration
 */
struct OutputConfig {
    std::string directory = "./output";

    // Data dictionary
    bool data_dictionary = true;
    std::string data_dictionary_format = "yaml";  // "yaml", "json", "csv"

    // Telemetry
    bool telemetry = true;
    std::string telemetry_format = "hdf5";  // "hdf5", "csv", "binary"

    // Timing reports
    bool timing_report = false;
};

} // namespace icarus
```

---

## Entity System Configs

See [phase4_0_7_entity_system.md](phase4_0_7_entity_system.md) for full details.

```cpp
namespace icarus {

/**
 * @brief Entity template loaded from YAML
 */
struct EntityTemplate {
    std::string name;
    std::string description;

    std::vector<ComponentConfig> components;
    std::vector<SignalRoute> routes;
    SchedulerConfig scheduler;
    StageConfig staging;

    static EntityTemplate FromFile(const std::string& yaml_path);
};

/**
 * @brief Entity instance (template + name + overrides)
 */
struct EntityInstance {
    std::string template_path;
    std::string name;
    std::unordered_map<std::string, ComponentConfig> overrides;
};

/**
 * @brief Swarm configuration for bulk entity spawning
 */
struct SwarmConfig {
    std::string template_path;
    std::string name_prefix;
    int count = 1;
    std::unordered_map<std::string, ComponentConfig> per_instance;
    std::vector<SignalRoute> swarm_routes;
};

/**
 * @brief Entity system configuration
 */
struct EntitySystemConfig {
    std::vector<EntityInstance> entities;
    std::vector<SwarmConfig> swarms;
    std::vector<SignalRoute> cross_entity_routes;
    std::vector<std::string> entity_order;
    bool auto_order = true;

    std::tuple<std::vector<ComponentConfig>,
               std::vector<SignalRoute>,
               SchedulerConfig> ExpandAll() const;
};

} // namespace icarus
```

---

## ComponentConfig

Already defined in Phase 4.0.1, included here for reference:

```cpp
namespace icarus {

/**
 * @brief Component configuration from YAML
 */
struct ComponentConfig {
    std::string type;           // Component type name (for factory)
    std::string name;           // Instance name
    std::string entity;         // Entity this component belongs to

    // Optional external config file
    std::string config_file;

    // Inline parameter overrides
    std::unordered_map<std::string, double> scalars;
    std::unordered_map<std::string, int> integers;
    std::unordered_map<std::string, std::string> strings;
    std::unordered_map<std::string, std::vector<double>> vectors;

    /**
     * @brief Get fully qualified name (entity.name)
     */
    std::string FullName() const {
        return entity.empty() ? name : entity + "." + name;
    }

    /**
     * @brief Merge another config into this one (other takes precedence)
     */
    void Merge(const ComponentConfig& other);

    static ComponentConfig FromFile(const std::string& yaml_path);
};

} // namespace icarus
```

---

## SignalRoute

Already defined in Phase 4.0.6, included here for reference:

```cpp
namespace icarus::signal {

/**
 * @brief Signal route definition
 */
struct SignalRoute {
    std::string input_path;     // Destination: Entity.Component.input
    std::string output_path;    // Source: Entity.Component.output

    // Optional transformations
    double gain = 1.0;
    double offset = 0.0;
    double delay = 0.0;         // Transport delay in seconds

    /**
     * @brief Validate route paths
     */
    std::vector<std::string> Validate() const;
};

} // namespace icarus::signal
```

---

## Exit Criteria

- [ ] `TrimConfig<Scalar>` with optimization settings, bounds, tolerances
- [ ] `LinearizationConfig` with state/input/output selection, export options
- [ ] `SymbolicsConfig` with dynamics/Jacobian/export settings
- [ ] `StageConfig<Scalar>` combining trim + linearization + symbolics
- [ ] `SchedulerConfig<Scalar>` with rate groups, topology settings
- [ ] `OutputConfig` for data dictionary, telemetry, timing reports
- [ ] `SimulatorConfig<Scalar>` master struct with reference_epoch_jd for time-varying models
- [ ] `SimulatorConfig::FromFile()` with include resolution

**Note:** Environment models (atmosphere, gravity, winds) are handled as regular components, not via a global config. Each entity template defines which environment components it needs.
