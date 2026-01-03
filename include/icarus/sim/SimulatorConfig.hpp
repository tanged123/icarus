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
#include <icarus/core/CoreTypes.hpp>
#include <icarus/io/log/LogService.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/sim/PhaseManager.hpp>
#include <icarus/sim/integrators/IntegratorTypes.hpp>

#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace icarus {

// =============================================================================
// Forward Declarations
// =============================================================================

struct EpochConfig;
struct TrimConfig;
struct LinearizationConfig;
struct SymbolicsConfig;
struct StageConfig;
struct SchedulerConfig;
struct OutputConfig;
struct SimulatorConfig;

// =============================================================================
// EpochConfig
// =============================================================================

/**
 * @brief Time epoch configuration for absolute time support
 *
 * Configures the simulation's reference epoch for time-varying models.
 * When configured, Vulcan's Epoch class provides conversions between
 * time scales (UTC, TAI, TT, GPS) and calendar representations.
 *
 * If not configured (empty reference), the simulation runs in MET-only mode
 * with an arbitrary J2000.0 reference epoch.
 */
struct EpochConfig {
    /// Time system: "MET" (default), "UTC", "TAI", "GPS"
    std::string system = "MET";

    /// Reference time as ISO 8601 string (for UTC system)
    /// Example: "2024-12-22T10:30:00Z"
    std::string reference;

    /// Julian Date (for TAI/TT systems when reference is empty)
    double jd = 0.0;

    /// GPS week number (for GPS system)
    int gps_week = 0;

    /// GPS seconds of week (for GPS system)
    double gps_seconds = 0.0;

    /// Check if epoch is configured (non-MET-only mode)
    [[nodiscard]] bool IsConfigured() const {
        return !reference.empty() || jd > 0.0 || gps_week > 0;
    }

    /// Create default MET-only config
    [[nodiscard]] static EpochConfig Default() { return EpochConfig{}; }
};

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

    /// Phase gating: if non-empty, group only executes when current phase is in this set
    std::set<int32_t> active_phases;

    SchedulerGroupConfig() = default;
    SchedulerGroupConfig(std::string n, double rate, int prio = 0)
        : name(std::move(n)), rate_hz(rate), priority(prio) {}

    /// Check if group should execute in given phase (empty = all phases)
    [[nodiscard]] bool IsActiveInPhase(int32_t current_phase) const {
        return active_phases.empty() || active_phases.contains(current_phase);
    }
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

    /// Load from YAML file
    /// @note Use SimulationLoader::Load() for full config loading
    [[nodiscard]] static SchedulerConfig FromFile(const std::string & /*yaml_path*/) {
        throw NotImplementedError("SchedulerConfig::FromFile - use SimulationLoader::Load()");
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

    /// Validate configuration (local validation, before global rate is known)
    [[nodiscard]] std::vector<std::string> Validate() const {
        std::vector<std::string> errors;

        if (groups.empty()) {
            errors.push_back("No scheduler groups defined");
        }

        std::unordered_set<std::string> seen_components;
        std::unordered_set<std::string> seen_group_names;

        for (const auto &group : groups) {
            // Check for duplicate group names
            if (seen_group_names.count(group.name)) {
                errors.push_back("Duplicate group name: " + group.name);
            }
            seen_group_names.insert(group.name);

            // Check rate is positive
            if (group.rate_hz <= 0.0) {
                errors.push_back("Group '" + group.name +
                                 "' has invalid rate: " + std::to_string(group.rate_hz));
            }

            // Check for duplicate component assignments
            for (const auto &member : group.members) {
                if (seen_components.count(member.component)) {
                    errors.push_back("Component '" + member.component +
                                     "' assigned to multiple groups");
                }
                seen_components.insert(member.component);
            }
        }

        return errors;
    }

    /// Validate rates against global simulation rate (called after rate derivation)
    [[nodiscard]] std::vector<std::string> ValidateGlobalTiming(double sim_rate_hz) const {
        std::vector<std::string> errors;

        for (const auto &group : groups) {
            // Group rate cannot exceed simulation rate
            if (group.rate_hz > sim_rate_hz) {
                errors.push_back("Group '" + group.name + "' rate " +
                                 std::to_string(group.rate_hz) + " Hz exceeds simulation rate " +
                                 std::to_string(sim_rate_hz) + " Hz");
            }

            // Group rate must be integer divisor of simulation rate
            double ratio = sim_rate_hz / group.rate_hz;
            if (std::abs(ratio - std::round(ratio)) > 1e-9) {
                errors.push_back("Group '" + group.name + "' rate " +
                                 std::to_string(group.rate_hz) +
                                 " Hz is not an integer divisor of simulation rate " +
                                 std::to_string(sim_rate_hz) + " Hz");
            }
        }

        return errors;
    }

    /// Compute frame divisors for all groups given simulation rate
    void ComputeFrameDivisors(double sim_rate_hz) {
        group_frame_divisors.clear();
        for (const auto &group : groups) {
            int divisor = static_cast<int>(std::round(sim_rate_hz / group.rate_hz));
            group_frame_divisors[group.name] = divisor;
        }
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
// EntityTemplate
// =============================================================================

/**
 * @brief Entity template loaded from YAML
 *
 * Self-contained unit defining components, internal routes, scheduler, and staging.
 * Can be instantiated multiple times with different names and overrides.
 */
struct EntityTemplate {
    std::string name;        ///< Template name (e.g., "Rocket")
    std::string description; ///< Optional description

    /// Components within this entity
    std::vector<ComponentConfig> components;

    /// Internal routes (relative names, expanded with entity prefix)
    std::vector<signal::SignalRoute> routes;

    /// Internal scheduler configuration
    SchedulerConfig scheduler;

    /// Internal staging configuration
    StageConfig staging;

    /// Load template from YAML file (implemented in SimulationLoader.hpp)
    static EntityTemplate FromFile(const std::string &yaml_path);
};

// =============================================================================
// EntityInstance
// =============================================================================

/**
 * @brief Entity instance (template + name + overrides)
 *
 * References a template and provides instance-specific configuration.
 */
struct EntityInstance {
    /// Template (either inline or loaded from file via !include)
    EntityTemplate entity_template;

    /// Instance name (becomes entity prefix in signals)
    std::string name;

    /// Component name -> config overrides
    std::unordered_map<std::string, ComponentConfig> overrides;
};

// =============================================================================
// SwarmConfig
// =============================================================================

/**
 * @brief Swarm configuration for bulk entity spawning
 *
 * Creates count copies of the template with names: prefix_000, prefix_001, ...
 * All copies are identical (per-instance expressions are future work).
 */
struct SwarmConfig {
    /// Entity template (inline or loaded from file)
    EntityTemplate entity_template;

    /// Name prefix (instances will be prefix_000, prefix_001, etc.)
    std::string name_prefix;

    /// Number of instances to create
    int count = 1;

    // Future: per-instance overrides with Jinja-style templating
    // Future: swarm-internal routes (neighbor connections)
};

// =============================================================================
// EntitySystemConfig
// =============================================================================

/**
 * @brief Entity system configuration
 *
 * Holds entities, swarms, cross-entity routes, and entity execution order.
 * Provides ExpandAll() to flatten everything to components and routes.
 */
struct EntitySystemConfig {
    std::vector<EntityInstance> entities;
    std::vector<SwarmConfig> swarms;
    std::vector<signal::SignalRoute> cross_entity_routes;

    /// Entity execution order (empty = auto-order based on dependencies)
    std::vector<std::string> entity_order;
    bool auto_order = true;

    /**
     * @brief Expand all entities and swarms to flat component list
     *
     * Expands all entity instances and swarms, prefixing signal paths.
     * @return Tuple of (components, routes, merged scheduler)
     */
    std::tuple<std::vector<ComponentConfig>, std::vector<signal::SignalRoute>, SchedulerConfig>
    ExpandAll() const;
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
    std::string source_file; ///< Path to source YAML file (set by loader)

    // =========================================================================
    // Time
    // =========================================================================
    double t_start = 0.0;
    double t_end = 100.0;
    double dt = 0.01; ///< Note: may be auto-derived from scheduler

    /// Epoch configuration for absolute time support
    /// When configured, enables time scale conversions (UTC, TAI, GPS, etc.)
    EpochConfig epoch;

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
    // Phase Management
    // =========================================================================
    PhaseConfig phases;

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

    /// Load from master binding file
    /// @note Implementation in SimulationLoader.hpp to avoid circular include
    [[nodiscard]] static SimulatorConfig FromFile(const std::string &path);

    /// Load from master + explicit routes file
    /// @note Use SimulationLoader::Load() which handles routes in the main config
    [[nodiscard]] static SimulatorConfig FromFiles(const std::string & /*master_path*/,
                                                   const std::string & /*routes_path*/) {
        throw NotImplementedError("SimulatorConfig::FromFiles - use SimulationLoader::Load()");
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
