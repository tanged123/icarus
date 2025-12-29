#pragma once

/**
 * @file SimulatorConfig.hpp
 * @brief Simulator and subsystem configuration structs
 *
 * Part of Phase 4.0.7: Configuration Infrastructure.
 * These structs are NOT templated - they use double for numeric values.
 *
 * The Simulator class uses these configs during construction and lifecycle.
 * Components access their configuration via ComponentConfig, not these structs.
 */

#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/io/LogConfig.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/sim/IntegratorTypes.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace icarus {

// =============================================================================
// Forward Declarations
// =============================================================================

struct TrimConfig;
struct LinearizationConfig;
struct SymbolicsConfig;
struct StageConfig;
struct SchedulerConfig;
struct OutputConfig;
struct SimulatorConfig;

// =============================================================================
// TrimConfig
// =============================================================================

/**
 * @brief Trim optimization configuration
 *
 * Defines how the simulator finds equilibrium/trim conditions during Stage().
 * Uses symbolic mode internally (janus::NewtonSolver or janus::Opti/IPOPT).
 */
struct TrimConfig {
    bool enabled = false; ///< Whether to run trim optimization

    /// Trim targets: which derivatives should be zero?
    std::vector<std::string> zero_derivatives; // e.g., ["velocity_dot", "angular_rate_dot"]

    /// Trim controls: which signals can be adjusted?
    std::vector<std::string> control_signals; // e.g., ["throttle", "elevator"]

    // Optimization settings
    double tolerance = 1e-6;
    int max_iterations = 100;
    std::string method = "newton"; ///< "newton" or "ipopt"

    /// Initial guesses for controls (optional)
    std::unordered_map<std::string, double> initial_guesses;

    /// Control bounds (optional): key -> (min, max)
    std::unordered_map<std::string, std::pair<double, double>> control_bounds;

    /// Create default disabled config
    [[nodiscard]] static TrimConfig Default() { return TrimConfig{}; }

    /// Validate configuration
    [[nodiscard]] std::vector<std::string> Validate() const {
        std::vector<std::string> errors;
        if (enabled) {
            if (zero_derivatives.empty()) {
                errors.push_back("Trim enabled but no zero_derivatives specified");
            }
            if (control_signals.empty()) {
                errors.push_back("Trim enabled but no control_signals specified");
            }
            if (method != "newton" && method != "ipopt") {
                errors.push_back("Unknown trim method: " + method);
            }
        }
        return errors;
    }
};

// =============================================================================
// LinearizationConfig
// =============================================================================

/**
 * @brief Linearization configuration
 *
 * Configures state-space extraction (A, B, C, D matrices).
 */
struct LinearizationConfig {
    bool enabled = false;

    /// State variables (for A matrix rows/cols)
    std::vector<std::string> states;

    /// Input variables (for B matrix cols)
    std::vector<std::string> inputs;

    /// Output variables (for C matrix rows)
    std::vector<std::string> outputs;

    // Export options
    bool export_matlab = false;
    bool export_numpy = false;
    bool export_json = false;
    std::string output_dir;

    /// Create default disabled config
    [[nodiscard]] static LinearizationConfig Default() { return LinearizationConfig{}; }

    /// Validate configuration
    [[nodiscard]] std::vector<std::string> Validate() const {
        std::vector<std::string> errors;
        if (enabled && states.empty()) {
            errors.push_back("Linearization enabled but no states specified");
        }
        return errors;
    }
};

// =============================================================================
// SymbolicsConfig
// =============================================================================

/**
 * @brief Symbolic generation configuration
 *
 * Controls symbolic graph generation during Stage().
 */
struct SymbolicsConfig {
    bool enabled = false;
    bool generate_dynamics = true;  ///< Generate f(x, u) symbolic graph
    bool generate_jacobian = false; ///< Generate df/dx, df/du
    std::string output_dir;         ///< Where to export symbolic functions

    /// Create default disabled config
    [[nodiscard]] static SymbolicsConfig Default() { return SymbolicsConfig{}; }
};

// =============================================================================
// StageConfig
// =============================================================================

/**
 * @brief Staging configuration
 *
 * Configures the Stage() phase: wiring validation, trim, linearization,
 * symbolic generation. Symbolic mode is used internally - user doesn't
 * need to manage it.
 */
struct StageConfig {
    TrimConfig trim;
    LinearizationConfig linearization;
    SymbolicsConfig symbolics;

    // Validation settings
    bool validate_wiring = true; ///< Throw if unwired inputs exist
    bool warn_on_unwired = true; ///< Log warning for unwired inputs

    /// Create default config
    [[nodiscard]] static StageConfig Default() { return StageConfig{}; }

    /// Validate all sub-configs
    [[nodiscard]] std::vector<std::string> Validate() const {
        std::vector<std::string> errors;
        auto trim_errors = trim.Validate();
        errors.insert(errors.end(), trim_errors.begin(), trim_errors.end());
        auto lin_errors = linearization.Validate();
        errors.insert(errors.end(), lin_errors.begin(), lin_errors.end());
        return errors;
    }
};

// =============================================================================
// SchedulerConfig
// =============================================================================

/**
 * @brief Scheduling mode enumeration
 */
enum class SchedulingMode {
    Automatic, ///< Topological sort from signal dependencies (Future TODO)
    Explicit   ///< User-defined execution order
};

/**
 * @brief A component member within a scheduler group
 */
struct GroupMember {
    std::string component; ///< Component name
    int priority = 0;      ///< Execution order within group (lower = runs first)

    GroupMember() = default;
    GroupMember(std::string comp, int prio = 0) : component(std::move(comp)), priority(prio) {}
};

/**
 * @brief A scheduler group with its own rate and priority
 */
struct SchedulerGroupConfig {
    std::string name;
    double rate_hz = 400.0; ///< Must be integer divisor of simulation rate
    int priority = 0;       ///< Execution order relative to other groups
    std::vector<GroupMember> members;

    SchedulerGroupConfig() = default;
    SchedulerGroupConfig(std::string n, double rate, int prio = 0)
        : name(std::move(n)), rate_hz(rate), priority(prio) {}
};

/**
 * @brief Cycle detection and handling configuration
 */
struct TopologyConfig {
    SchedulingMode mode = SchedulingMode::Explicit;

    enum class CycleHandling { Error, Warn, BreakAtDelay };
    CycleHandling cycle_detection = CycleHandling::Error;
    bool log_order = true; ///< Log execution order during Stage()
};

/**
 * @brief Scheduler configuration with rate groups
 *
 * Every component belongs to exactly one group.
 * Simulation dt is auto-derived from the fastest group across all entities.
 */
struct SchedulerConfig {
    std::vector<SchedulerGroupConfig> groups;
    TopologyConfig topology;

    /// Computed at runtime: frame divisor for each group
    std::unordered_map<std::string, int> group_frame_divisors;

    /// Create default config with a single 400Hz group
    [[nodiscard]] static SchedulerConfig Default() {
        SchedulerConfig cfg;
        cfg.groups.emplace_back("default", 400.0, 0);
        return cfg;
    }

    /// Load from YAML file (stub - returns default)
    [[nodiscard]] static SchedulerConfig FromFile(const std::string & /*yaml_path*/) {
        // TODO: Implement YAML loading
        return Default();
    }

    /// Get maximum rate across all groups
    [[nodiscard]] double MaxRate() const {
        double max_rate = 0.0;
        for (const auto &group : groups) {
            if (group.rate_hz > max_rate) {
                max_rate = group.rate_hz;
            }
        }
        return max_rate > 0.0 ? max_rate : 400.0; // Default fallback
    }

    /// Validate configuration
    [[nodiscard]] std::vector<std::string> Validate() const {
        std::vector<std::string> errors;
        if (groups.empty()) {
            errors.push_back("No scheduler groups defined");
        }
        for (const auto &group : groups) {
            if (group.rate_hz <= 0.0) {
                errors.push_back("Group '" + group.name +
                                 "' has invalid rate: " + std::to_string(group.rate_hz));
            }
        }
        return errors;
    }
};

// =============================================================================
// OutputConfig
// =============================================================================

/**
 * @brief Output/export configuration
 */
struct OutputConfig {
    std::string directory = "./output";

    // Data dictionary
    bool data_dictionary = true;
    std::string data_dictionary_format = "yaml"; ///< "yaml", "json", "csv"

    // Telemetry
    bool telemetry = true;
    std::string telemetry_format = "hdf5"; ///< "hdf5", "csv", "binary"

    // Timing reports
    bool timing_report = false;

    /// Create default config
    [[nodiscard]] static OutputConfig Default() { return OutputConfig{}; }
};

// =============================================================================
// SimulatorConfig
// =============================================================================

/**
 * @brief Complete simulation configuration
 *
 * Loaded from simulation.yaml master binding file.
 * All sub-configs can be inline or referenced via includes.
 *
 * NOT templated - uses double for numeric values.
 */
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
    double t_start = 0.0;
    double t_end = 100.0;
    double dt = 0.01; ///< Note: may be auto-derived from scheduler

    /// Reference epoch for time-varying models (atmosphere, gravity, etc.)
    /// Components that need absolute time can read this.
    double reference_epoch_jd = 2451545.0; ///< J2000.0 default

    // =========================================================================
    // Components (flattened from entity expansion)
    // =========================================================================
    std::vector<ComponentConfig> components;

    // =========================================================================
    // Signal Routing (flattened from entity expansion)
    // =========================================================================
    std::vector<signal::SignalRoute> routes;

    // =========================================================================
    // Scheduler
    // =========================================================================
    SchedulerConfig scheduler = SchedulerConfig::Default();

    // =========================================================================
    // Staging (Trim + Linearization + Symbolics)
    // =========================================================================
    StageConfig staging;

    // =========================================================================
    // Integrator
    // =========================================================================
    IntegratorConfig<double> integrator = IntegratorConfig<double>::RK4Default();

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

    /// Create default config
    [[nodiscard]] static SimulatorConfig Default() { return SimulatorConfig{}; }

    /// Load from master binding file (stub - returns default)
    [[nodiscard]] static SimulatorConfig FromFile(const std::string & /*path*/) {
        // TODO: Implement YAML loading with include resolution
        return Default();
    }

    /// Load from master + explicit routes file (stub - returns default)
    [[nodiscard]] static SimulatorConfig FromFiles(const std::string & /*master_path*/,
                                                   const std::string & /*routes_path*/) {
        // TODO: Implement YAML loading
        return Default();
    }

    /// Validate configuration consistency
    [[nodiscard]] std::vector<std::string> Validate() const {
        std::vector<std::string> errors;

        // Time validation
        if (t_end <= t_start) {
            errors.push_back("t_end must be greater than t_start");
        }
        if (dt <= 0.0) {
            errors.push_back("dt must be positive");
        }

        // Sub-config validation
        auto sched_errors = scheduler.Validate();
        errors.insert(errors.end(), sched_errors.begin(), sched_errors.end());

        auto stage_errors = staging.Validate();
        errors.insert(errors.end(), stage_errors.begin(), stage_errors.end());

        return errors;
    }
};

} // namespace icarus
