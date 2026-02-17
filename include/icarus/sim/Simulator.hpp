#pragma once

/**
 * @file Simulator.hpp
 * @brief Top-level simulation coordinator
 *
 * Part of Phase 4.0.7: Simulator API Refactor.
 *
 * The Simulator is NOT templated - users see one class.
 * Internally uses double for numeric execution.
 * Symbolic mode (casadi::MX) is used during Stage() for analysis.
 */

#include <functional>
#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/ComponentFactory.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/ErrorLogging.hpp>
#include <icarus/io/MissionLogger.hpp>
#include <icarus/io/SimulationLoader.hpp>
#include <icarus/io/data/DataDictionary.hpp>
#include <icarus/io/data/IntrospectionGraph.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/sim/IntegrationManager.hpp>
#include <icarus/sim/PhaseManager.hpp>
#include <icarus/sim/Scheduler.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/sim/StateManager.hpp>
#include <icarus/sim/TopologyAnalyzer.hpp>
#include <icarus/staging/StagingTypes.hpp>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Janus includes for symbolic graph generation
#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>

// Vulcan time infrastructure
#include <vulcan/time/Epoch.hpp>

// Recording
#include <icarus/io/HDF5Recorder.hpp>

// Forward declarations for staging
namespace icarus::staging {
class TrimSolver;
class Linearizer;
} // namespace icarus::staging

namespace icarus {

/**
 * @brief Top-level simulation coordinator
 *
 * NOT templated on Scalar - user sees one class.
 * Provides clean 4-operation external interface:
 * - FromConfig() - Initialize from YAML
 * - Stage() - Prepare for execution (validation, optional trim/linearization)
 * - Step() - Advance simulation
 * - ~Simulator() - Cleanup (RAII)
 */
class Simulator {
  public:
    // =========================================================================
    // CORE LIFECYCLE (The 4 Operations)
    // =========================================================================

    /**
     * @brief Create simulator from configuration file
     *
     * Loads component configs and routes from YAML, creates components,
     * provisions them. Ready for Stage() after this.
     *
     * @param config_path Path to simulation YAML configuration
     * @return Configured Simulator ready for Stage()
     */
    [[nodiscard]] static std::unique_ptr<Simulator> FromConfig(const std::string &config_path);

    /**
     * @brief Create simulator from configuration struct
     */
    [[nodiscard]] static std::unique_ptr<Simulator> FromConfig(const SimulatorConfig &config);

    /**
     * @brief Default constructor for programmatic setup
     */
    Simulator();

    /// Destructor
    ~Simulator();

    // Non-copyable, non-movable (Backplane holds reference to registry_)
    Simulator(const Simulator &) = delete;
    Simulator &operator=(const Simulator &) = delete;
    Simulator(Simulator &&) = delete;
    Simulator &operator=(Simulator &&) = delete;

    /**
     * @brief Stage the simulation
     *
     * Prepares the vehicle for launch:
     * - Calls Stage() on all components (input wiring)
     * - Validates all signal wiring
     * - Generates symbolic graphs (if configured)
     * - Runs trim optimization (if configured)
     */
    void Stage();
    void Stage(const StageConfig &config);

    /**
     * @brief Execute one simulation step
     *
     * Uses scheduler for multi-rate execution.
     * Advances time by dt using configured integrator.
     *
     * @param dt Timestep (uses nominal dt if not specified)
     */
    void Step(double dt);
    void Step(); // Uses config dt

    // =========================================================================
    // QUERY INTERFACE (Read-only)
    // =========================================================================

    /// Get simulation name (from config)
    [[nodiscard]] const std::string &Name() const { return config_.name; }

    /// Get configured timestep
    [[nodiscard]] double Dt() const { return config_.dt; }

    /// Get configured end time
    [[nodiscard]] double EndTime() const { return config_.t_end; }

    /// Get current simulation time (MET - derived from epoch)
    [[nodiscard]] double Time() const { return epoch_ - epoch_start_; }

    /// Get current epoch (single source of truth for time)
    [[nodiscard]] const vulcan::time::NumericEpoch &Epoch() const { return epoch_; }

    /// Get Julian Date in UTC scale
    [[nodiscard]] double JD_UTC() const { return epoch_.jd_utc(); }

    /// Get Julian Date in TAI scale
    [[nodiscard]] double JD_TAI() const { return epoch_.jd_tai(); }

    /// Get Julian Date in TT scale
    [[nodiscard]] double JD_TT() const { return epoch_.jd_tt(); }

    /// Get Julian Date in GPS scale
    [[nodiscard]] double JD_GPS() const { return epoch_.jd_gps(); }

    /// Get GPS week number
    [[nodiscard]] int GPSWeek() const { return epoch_.gps_week(); }

    /// Get GPS seconds of week
    [[nodiscard]] double GPSSecondsOfWeek() const { return epoch_.gps_seconds_of_week(); }

    /// Get current time as ISO 8601 string
    [[nodiscard]] std::string ISO8601() const { return epoch_.to_iso_string(); }

    /// Get current simulation lifecycle state
    [[nodiscard]] Lifecycle GetLifecycle() const { return lifecycle_; }

    /// Get current flight phase value (from PhaseManager)
    [[nodiscard]] int32_t GetFlightPhase() const { return phase_manager_.CurrentPhase(); }

    /// Get current flight phase name (from PhaseManager)
    [[nodiscard]] std::string GetFlightPhaseName() const {
        return phase_manager_.CurrentPhaseName();
    }

    /// Check if simulation is initialized (Staged or later)
    [[nodiscard]] bool IsInitialized() const { return lifecycle_ >= Lifecycle::Staged; }

    /**
     * @brief Read a signal value by name
     * @param name Full signal path: "Entity.Component.signal"
     */
    /**
     * @brief Read a scalar signal value by name
     */
    [[nodiscard]] double Peek(const std::string &name) const { return registry_.GetByName(name); }

    /// Get data dictionary for the simulation
    [[nodiscard]] DataDictionary GetDataDictionary() const;

    /// Get introspection graph (data dictionary + topology edges)
    [[nodiscard]] IntrospectionGraph GetIntrospectionGraph() const;

    /// Get signal registry (for recording, introspection)
    [[nodiscard]] const SignalRegistry<double> &Registry() const { return registry_; }

    // =========================================================================
    // CONTROL INTERFACE (Runtime modification)
    // =========================================================================

    /**
     * @brief Write a signal value by name
     */
    /**
     * @brief Write a scalar signal value by name
     */
    void Poke(const std::string &name, double value) { registry_.SetByName(name, value); }

    /**
     * @brief Get the mission logger
     */
    [[nodiscard]] MissionLogger &GetLogger() { return logger_; }
    [[nodiscard]] const MissionLogger &GetLogger() const { return logger_; }

    /**
     * @brief Reset simulation to initial state
     * Re-applies ICs, resets time to 0.
     */
    void Reset();

    /**
     * @brief Set simulation time (MET)
     *
     * Used by warmstart to restore time from recording.
     * Sets epoch_ = epoch_start_ + met.
     *
     * @param met Mission Elapsed Time to set
     */
    void SetTime(double met);

    /// Input source callback: (signal_name) -> value
    using InputSourceCallback = std::function<double(const std::string &)>;

    /// Output observer callback: (signal_name, value) -> void
    using OutputObserverCallback = std::function<void(const std::string &, double)>;

    /// Register an external input source for a signal
    void SetInputSource(const std::string &signal_name, InputSourceCallback callback);

    /// Register an output observer for a signal
    void SetOutputObserver(const std::string &signal_name, OutputObserverCallback callback);

    /// Remove an input source
    void ClearInputSource(const std::string &signal_name);

    /// Remove an output observer
    void ClearOutputObserver(const std::string &signal_name);

    // =========================================================================
    // EXPERT INTERFACE (Advanced users)
    // =========================================================================

    /// Get current state vector
    [[nodiscard]] Eigen::VectorXd GetState() const;

    /// Set state vector
    void SetState(const Eigen::VectorXd &X);

    /// Compute derivatives for current state at time t
    Eigen::VectorXd ComputeDerivatives(double t);

    /// Execute adaptive step (requires RK45 integrator)
    AdaptiveStepResult<double> AdaptiveStep(double dt_request);

    /**
     * @brief Get symbolic dynamics graph
     * Available after Stage() if symbolics.enabled = true.
     */
    [[nodiscard]] std::optional<janus::Function> GetDynamicsGraph() const;

    /**
     * @brief Get symbolic Jacobian
     * Available after Stage() if symbolics.generate_jacobian = true.
     */
    [[nodiscard]] std::optional<janus::Function> GetJacobian() const;

    /**
     * @brief Get trim result
     * Available after Stage() if trim.enabled = true.
     */
    [[nodiscard]] const std::optional<staging::TrimResult> &GetTrimResult() const {
        return trim_result_;
    }

    /**
     * @brief Get linear model
     * Available after Stage() if linearization.enabled = true.
     */
    [[nodiscard]] const std::optional<staging::LinearModel> &GetLinearModel() const {
        return linear_model_;
    }

    /**
     * @brief Get simulator configuration
     */
    [[nodiscard]] const SimulatorConfig &GetConfig() const { return config_; }

    // =========================================================================
    // PROGRAMMATIC SETUP (Alternative to FromConfig)
    // =========================================================================

    /// Add a component to the simulation
    void AddComponent(std::unique_ptr<Component<double>> component);

    /// Add a component from config (requires ComponentFactory)
    void AddComponent(const ComponentConfig &config);

    /// Configure simulator with full config struct
    void Configure(const SimulatorConfig &config);

    /// Add signal routes
    void AddRoutes(const std::vector<signal::SignalRoute> &routes);

    /// Get number of components
    [[nodiscard]] std::size_t NumComponents() const { return components_.size(); }

    /// Get backplane for signal introspection (expert)
    [[nodiscard]] Backplane<double> &GetBackplane() { return backplane_; }
    [[nodiscard]] const Backplane<double> &GetBackplane() const { return backplane_; }

  private:
    // =========================================================================
    // PRIVATE METHODS
    // =========================================================================

    /// Provision all components (called by FromConfig)
    void Provision();

    /// Apply signal routing from config
    void ApplyRouting();

    /// Allocate state vectors and bind to components
    void AllocateAndBindState();

    /// Apply initial conditions from config
    void ApplyInitialConditions();

    /// Validate all inputs are wired
    void ValidateWiring();

    /// Invoke input source callbacks before Step
    void InvokeInputSources();

    /// Invoke output observer callbacks after Step
    void InvokeOutputObservers();

    /// Initialize epoch from configuration
    void InitializeEpoch();

    // =========================================================================
    // PRIVATE DATA
    // =========================================================================

    SimulatorConfig config_;

    // Components
    std::vector<std::unique_ptr<Component<double>>> components_;

    // Signal system
    SignalRegistry<double> registry_;
    Backplane<double> backplane_;
    signal::SignalRouter<double> router_;

    // Managers
    StateManager<double> state_manager_;
    IntegrationManager<double> integration_manager_;
    Scheduler scheduler_;
    PhaseManager<double> phase_manager_;

    // Logging
    MissionLogger logger_;

    // Time and lifecycle
    vulcan::time::NumericEpoch epoch_;       ///< Current epoch (THE time - single source of truth)
    vulcan::time::NumericEpoch epoch_start_; ///< Reference epoch at t=0 (for MET derivation)
    Lifecycle lifecycle_ = Lifecycle::Uninitialized;
    int frame_count_ = 0;

    // Symbolic results (optional, after Stage)
    std::optional<janus::Function> dynamics_graph_;
    std::optional<janus::Function> jacobian_;

    // Recording (optional, when enabled in config)
    std::unique_ptr<HDF5Recorder> recorder_;

    // Staging results (optional, after Stage)
    std::optional<staging::TrimResult> trim_result_;
    std::optional<staging::LinearModel> linear_model_;

    // External callbacks
    std::unordered_map<std::string, InputSourceCallback> input_sources_;
    std::unordered_map<std::string, OutputObserverCallback> output_observers_;

    // Dependency tracking
    std::unordered_map<Component<double> *, std::vector<std::string>> outputs_;
    std::unordered_map<Component<double> *, std::vector<std::string>> inputs_;

    // =========================================================================
    // FRAME CONTEXT (Scheduler filtering helper)
    // =========================================================================

    /**
     * @brief Context for component execution in a single frame
     *
     * Encapsulates scheduler logic for determining which components
     * should run and which are active for derivative gating.
     */
    struct FrameContext {
        std::unordered_set<std::string> active_components;
        std::unordered_set<std::string> all_scheduled_components;

        /// Check if a component should run this frame
        [[nodiscard]] bool ShouldRun(const std::string &name) const {
            // If component isn't registered with scheduler, run every frame
            if (!all_scheduled_components.contains(name)) {
                return true;
            }
            // Otherwise, only run if its group is active this frame
            return active_components.contains(name);
        }
    };

    /// Build frame context from scheduler for current frame
    [[nodiscard]] FrameContext BuildFrameContext() const;

    /// Execute component method with error handling and timing
    void ExecComponent(Component<double> &comp, void (Component<double>::*method)(double, double),
                       double t, double dt);
};

} // namespace icarus

// =============================================================================
// INLINE IMPLEMENTATIONS
// =============================================================================

// Include staging solvers for Stage() implementation
// (Must be outside namespace icarus to avoid double-nesting)
#include <icarus/staging/Linearizer.hpp>
#include <icarus/staging/SymbolicStager.hpp>
#include <icarus/staging/TrimSolver.hpp>

namespace icarus {

inline Simulator::~Simulator() {
    // Close recorder and log summary
    if (recorder_) {
        size_t frames = recorder_->FrameCount();
        recorder_->Close();
        logger_.Log(LogLevel::Info, "[REC] Recording complete: " + std::to_string(frames) +
                                        " frames written to " + config_.recording.path);
        recorder_.reset();
    }

    // If we ever ran, log the debrief on destruction (RAII)
    if (lifecycle_ == Lifecycle::Running) {
        logger_.BeginLifecycle(LifecycleStrings::Shutdown);
        auto wall_time = std::chrono::duration<double>(logger_.WallElapsed()).count();
        logger_.LogDebrief(Time(), wall_time);
    }
}

inline Simulator::Simulator() : backplane_(registry_) { integration_manager_.ConfigureDefault(); }

inline std::unique_ptr<Simulator> Simulator::FromConfig(const std::string &config_path) {
    // Load YAML configuration
    SimulatorConfig config = io::SimulationLoader::Load(config_path);
    auto sim = FromConfig(config);
    if (sim) {
        sim->logger_.LogConfigFile(config_path);
    }
    return sim;
}

inline std::unique_ptr<Simulator> Simulator::FromConfig(const SimulatorConfig &config) {
    auto sim = std::make_unique<Simulator>();
    sim->Configure(config);

    // Log startup banner (Icarus engine version)
    sim->logger_.LogStartup();

    // Log simulation configuration (from YAML)
    sim->logger_.LogSimulationConfig(config.name, config.version, config.description);

    // Log time and integrator settings
    sim->logger_.LogTimeConfig(config.t_start, config.t_end, config.dt);
    sim->logger_.LogIntegrator(config.integrator.type);

    // Log phase configuration
    sim->logger_.LogPhaseConfig(config.phases);

    // Begin Provision phase
    sim->logger_.BeginLifecycle(LifecycleStrings::Provision);

    // Create components from factory
    auto &factory = ComponentFactory<double>::Instance();
    std::size_t comp_count = config.components.size();
    for (std::size_t i = 0; i < comp_count; ++i) {
        const auto &comp_cfg = config.components[i];
        auto component = factory.Create(comp_cfg);

        // Log component addition
        bool is_last = (i == comp_count - 1);
        sim->logger_.LogComponentAdd(comp_cfg.name, comp_cfg.type, "YAML", is_last);

        sim->AddComponent(std::move(component));
    }

    // Add routes from config
    sim->AddRoutes(config.routes);

    // Provision all components
    sim->Provision();

    // Log state allocation
    sim->logger_.LogStateAllocation(sim->state_manager_.TotalSize());

    // Generate and log manifest (signal dictionary)
    auto dict = sim->GetDataDictionary();
    sim->logger_.LogManifest(dict);

    sim->logger_.EndLifecycle();

    return sim;
}

inline void Simulator::InitializeEpoch() {
    // Parse epoch configuration - always initialize epoch (single source of truth)
    if (config_.epoch.system == "UTC" && !config_.epoch.reference.empty()) {
        // Robust ISO 8601 parser with timezone offset and fractional seconds support
        // Format: YYYY-MM-DDTHH:MM:SS[.fractional][Z|+HH:MM|-HH:MM|+HHMM|-HHMM]
        static const std::regex iso8601_regex(
            R"(^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})(\.\d+)?(Z|([+-])(\d{2}):?(\d{2}))?$)");
        // Groups: 1=year, 2=month, 3=day, 4=hour, 5=minute, 6=whole_seconds
        //         7=fractional_seconds (optional, includes leading '.')
        //         8=timezone block (optional)
        //         9=tz_sign (+ or -), 10=tz_hours, 11=tz_minutes

        std::smatch match;
        if (!std::regex_match(config_.epoch.reference, match, iso8601_regex)) {
            throw ConfigError("Invalid epoch reference format: " + config_.epoch.reference +
                              " (expected ISO 8601: YYYY-MM-DDTHH:MM:SS[.sss][Z|±HH:MM])");
        }

        // Extract date/time components
        int year = std::stoi(match[1].str());
        int month = std::stoi(match[2].str());
        int day = std::stoi(match[3].str());
        int hour = std::stoi(match[4].str());
        int min = std::stoi(match[5].str());
        double sec = std::stod(match[6].str());

        // Add fractional seconds if present
        if (match[7].matched) {
            sec += std::stod(match[7].str());
        }

        // Parse timezone offset and convert to UTC
        if (match[8].matched && match[8].str() != "Z") {
            // Has explicit offset (not Z)
            int tz_sign = (match[9].str() == "-") ? -1 : 1;
            int tz_hours = std::stoi(match[10].str());
            int tz_minutes = std::stoi(match[11].str());
            int offset_minutes = tz_sign * (tz_hours * 60 + tz_minutes);

            // Convert local time to UTC by subtracting the offset
            // (positive offset = ahead of UTC, so subtract to get UTC)
            int total_minutes = hour * 60 + min - offset_minutes;

            // Handle day rollover
            while (total_minutes < 0) {
                total_minutes += 24 * 60;
                day -= 1;
            }
            while (total_minutes >= 24 * 60) {
                total_minutes -= 24 * 60;
                day += 1;
            }

            hour = total_minutes / 60;
            min = total_minutes % 60;

            // Handle month/year rollover (simplified - use days in month)
            auto days_in_month = [](int y, int m) -> int {
                static constexpr int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
                if (m == 2) {
                    bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
                    return leap ? 29 : 28;
                }
                return days[m - 1];
            };

            while (day < 1) {
                month -= 1;
                if (month < 1) {
                    month = 12;
                    year -= 1;
                }
                day += days_in_month(year, month);
            }
            while (day > days_in_month(year, month)) {
                day -= days_in_month(year, month);
                month += 1;
                if (month > 12) {
                    month = 1;
                    year += 1;
                }
            }
        }
        // else: Z suffix or no suffix means UTC - no conversion needed

        epoch_start_ = vulcan::time::NumericEpoch::from_utc(year, month, day, hour, min, sec);
    } else if (config_.epoch.system == "TAI" && config_.epoch.jd > 0.0) {
        epoch_start_ = vulcan::time::NumericEpoch::from_jd_tai(config_.epoch.jd);
    } else if (config_.epoch.system == "GPS" && config_.epoch.gps_configured) {
        epoch_start_ = vulcan::time::NumericEpoch::from_gps_week(config_.epoch.gps_week,
                                                                 config_.epoch.gps_seconds);
    } else {
        // MET-only mode or default: use J2000.0 as arbitrary reference
        epoch_start_ = vulcan::time::NumericEpoch();
    }

    // Current epoch starts at epoch_start_ (MET = 0)
    epoch_ = epoch_start_;
}

inline void Simulator::Configure(const SimulatorConfig &config) {
    config_ = config;

    // Configure logger from YAML settings
    if (!config_.logging.file_path.empty() && config_.logging.file_enabled) {
        logger_.SetLogFile(config_.logging.file_path);
    }
    logger_.SetConsoleLevel(config_.logging.console_level);
    logger_.SetFileLevel(config_.logging.file_level);
    logger_.SetProgressEnabled(config_.logging.progress_enabled);
    logger_.SetProfilingEnabled(config_.logging.profiling_enabled);
    logger_.SetVersion(config_.version);

    // Configure integrator
    integration_manager_.Configure(config_.integrator.type);

    // Configure scheduler
    double sim_rate = config_.scheduler.MaxRate();
    auto timing_errors = config_.scheduler.ValidateGlobalTiming(sim_rate);
    if (!timing_errors.empty()) {
        throw ConfigError("Scheduler timing validation failed: " + timing_errors[0]);
    }
    scheduler_.Configure(config_.scheduler, sim_rate);

    // Configure phase manager (if phases defined)
    if (!config_.phases.definitions.empty()) {
        phase_manager_.Configure(config_.phases);
    }

    // Initialize epoch from configuration
    InitializeEpoch();
}

inline void Simulator::AddComponent(std::unique_ptr<Component<double>> component) {
    components_.push_back(std::move(component));
}

inline void Simulator::AddComponent(const ComponentConfig &config) {
    auto &factory = ComponentFactory<double>::Instance();
    auto component = factory.Create(config);
    components_.push_back(std::move(component));
}

inline void Simulator::AddRoutes(const std::vector<signal::SignalRoute> &routes) {
    for (const auto &route : routes) {
        router_.AddRoute(route);
    }
}

inline void Simulator::Provision() {
    if (lifecycle_ != Lifecycle::Uninitialized) {
        throw LifecycleError(LifecyclePhase::Provision, "can only be called once");
    }

    // Provision each component
    for (auto &comp : components_) {
        backplane_.set_context(comp->Entity(), comp->Name());
        backplane_.clear_tracking();

        try {
            comp->Provision(backplane_);
        } catch (const Error &e) {
            backplane_.clear_context();
            LogError(e, 0.0, comp->FullName());
            throw;
        }
        comp->MarkProvisioned();

        outputs_[comp.get()] = backplane_.registered_outputs();
        backplane_.clear_context();
    }

    // Apply routing
    ApplyRouting();

    // Allocate and bind state
    AllocateAndBindState();

    lifecycle_ = Lifecycle::Provisioned;
}

inline void Simulator::ApplyRouting() { router_.ApplyRoutes(backplane_); }

inline void Simulator::AllocateAndBindState() {
    // Phase 6: Discover states from registry instead of scanning components
    state_manager_.DiscoverStates(registry_);
}

inline void Simulator::ApplyInitialConditions() {
    // ICs are typically applied in component Stage() methods
    // Config-driven ICs could be applied here in the future
}

inline void Simulator::ValidateWiring() {
    auto unwired = registry_.get_unwired_inputs();
    if (!unwired.empty()) {
        std::string msg;
        for (const auto &name : unwired) {
            msg += name + ", ";
        }
        // Remove trailing ", "
        if (msg.size() >= 2) {
            msg.resize(msg.size() - 2);
        }

        if (config_.staging.validate_wiring) {
            throw WiringError("Unwired inputs: " + msg);
        }
        // Warn only - unwired inputs default to 0, can be poked externally
        logger_.Log(LogLevel::Warning, "[WIRE] Unwired inputs (defaulting to 0): " + msg);
    }
}

inline void Simulator::Stage() {
    // Auto-provision if components were added programmatically
    if (lifecycle_ == Lifecycle::Uninitialized && !components_.empty()) {
        Provision();
    }

    if (lifecycle_ != Lifecycle::Provisioned) {
        throw LifecycleError(LifecyclePhase::Stage, "requires components to be added first");
    }

    // Begin Stage phase logging
    logger_.BeginLifecycle(LifecycleStrings::Stage);

    // =========================================================================
    // Automatic Topology Ordering (if enabled)
    // =========================================================================
    if (config_.scheduler.topology.mode == SchedulingMode::Automatic) {
        logger_.Log(LogLevel::Debug, "[TOPO] Computing execution order from signal dependencies");

        // Build dependency graph and compute topological order
        auto result = TopologyAnalyzer::ComputeExecutionOrder(
            config_, config_.scheduler.topology.cycle_detection);

        if (result.has_cycles) {
            // Log cycle warnings
            for (const auto &cycle : result.cycles) {
                logger_.Log(LogLevel::Warning, "[TOPO] " + cycle.ToString());
            }
        }

        // Apply topology order to scheduler
        TopologyAnalyzer::ApplyTopologyOrder(config_.scheduler, result);

        // Reconfigure scheduler with updated order
        scheduler_.Configure(config_.scheduler, config_.scheduler.MaxRate());

        logger_.Log(
            LogLevel::Info, "[TOPO] Computed execution order: " + [&]() {
                std::string s;
                for (size_t i = 0; i < result.execution_order.size(); ++i) {
                    s += result.execution_order[i];
                    if (i < result.execution_order.size() - 1)
                        s += " -> ";
                }
                return s;
            }());
    }

    // Log wiring info (Debug level)
    for (const auto &route : router_.GetRoutes()) {
        std::string from = route.output_path;
        std::string to = route.input_path;
        logger_.LogWiring(from, to);
    }

    // Log scheduler execution order (Debug level)
    scheduler_.LogExecutionOrder(&logger_);

    // Set epoch reference on backplane for component binding
    backplane_.set_epoch(&epoch_);

    // Stage each component (config loading, input wiring)
    for (auto &comp : components_) {
        backplane_.set_context(comp->Entity(), comp->Name());
        backplane_.clear_tracking();

        // Bind epoch to component for time-dependent calculations
        backplane_.bind_epoch_to(*comp);

        try {
            comp->Stage(backplane_);
        } catch (const Error &e) {
            backplane_.clear_context();
            LogError(e, 0.0, comp->FullName());
            throw;
        }
        comp->MarkStaged();

        inputs_[comp.get()] = backplane_.resolved_inputs();
        backplane_.clear_context();
    }

    // Validate wiring
    ValidateWiring();

    // =========================================================================
    // Trim Optimization (if enabled)
    // =========================================================================
    if (config_.staging.trim.enabled) {
        // Build targets list for logging
        auto solver =
            staging::CreateTrimSolver(config_.staging.trim, config_.staging.symbolics.enabled);
        auto result = solver->Solve(*this, config_.staging.trim);

        if (config_.staging.trim.tolerance > 0 && !result.converged) {
            // Only throw if strict tolerance was specified
            throw StageError("Trim optimization failed: " + result.message);
        }

        trim_result_ = std::move(result);
    }

    // =========================================================================
    // Linearization (if enabled)
    // =========================================================================
    if (config_.staging.linearization.enabled) {
        std::ostringstream oss;
        oss << "[LIN] Computing linear model (" << config_.staging.linearization.states.size()
            << " states, " << config_.staging.linearization.inputs.size() << " inputs, "
            << config_.staging.linearization.outputs.size() << " outputs)";
        logger_.Log(LogLevel::Debug, oss.str());

        auto linearizer = staging::CreateLinearizer(config_.staging.symbolics.enabled);
        auto model = linearizer->Compute(*this, config_.staging.linearization);

        // Log full linear model analysis
        logger_.LogLinearModel(model);

        // Export if configured
        const auto &lin = config_.staging.linearization;
        if (!lin.output_dir.empty()) {
            if (lin.export_matlab) {
                model.ExportMatlab(lin.output_dir + "/linear_model.m");
                logger_.Log(LogLevel::Debug,
                            "[LIN] Exported MATLAB to " + lin.output_dir + "/linear_model.m");
            }
            if (lin.export_numpy) {
                model.ExportNumPy(lin.output_dir + "/linear_model.py");
                logger_.Log(LogLevel::Debug,
                            "[LIN] Exported NumPy to " + lin.output_dir + "/linear_model.py");
            }
            if (lin.export_json) {
                model.ExportJSON(lin.output_dir + "/linear_model.json");
                logger_.Log(LogLevel::Debug,
                            "[LIN] Exported JSON to " + lin.output_dir + "/linear_model.json");
            }
        }

        linear_model_ = std::move(model);
    }

    // =========================================================================
    // Symbolic Graph Generation (if enabled)
    // =========================================================================
    if (config_.staging.symbolics.enabled) {
        try {
            staging::SymbolicSimulatorCore sym_sim(config_);
            staging::SymbolicStager stager(sym_sim);

            staging::SymbolicStagerConfig stager_config;
            stager_config.generate_dynamics = true;
            stager_config.generate_jacobian = config_.staging.symbolics.generate_jacobian;
            stager_config.include_time = true;

            auto sym_dynamics = stager.GenerateDynamics(stager_config);

            dynamics_graph_ = sym_dynamics.dynamics;
            if (config_.staging.symbolics.generate_jacobian) {
                jacobian_ = sym_dynamics.jacobian_x;
            }

            logger_.Log(LogLevel::Info, "[SYM] Symbolic graphs generated: " +
                                            std::to_string(sym_sim.GetStateSize()) + " states");
        } catch (const Error &e) {
            logger_.Log(LogLevel::Warning,
                        "[SYM] Symbolic generation failed: " + std::string(e.what()));
            // Continue - symbolic is optional
        } catch (const std::exception &e) {
            logger_.Log(LogLevel::Warning,
                        "[SYM] Symbolic generation failed: " + std::string(e.what()));
            // Continue - symbolic is optional
        }
    }

    logger_.EndLifecycle();

    // Initialize recording if enabled
    if (config_.recording.IsActive()) {
        recorder_ = std::make_unique<HDF5Recorder>(registry_, config_.recording);
        recorder_->Open("");
        logger_.Log(LogLevel::Info, "[REC] Recording " +
                                        std::to_string(recorder_->RecordedSignals().size()) +
                                        " signals to " + config_.recording.path);
    }

    lifecycle_ = Lifecycle::Staged;

    // Reset to t=0 unless warmstart mode restored time from recording
    if (!config_.staging.trim.IsWarmstart()) {
        epoch_ = epoch_start_;
    }
}

inline void Simulator::Stage(const StageConfig &config) {
    config_.staging = config;
    Stage();
}

inline void Simulator::Step(double dt) {
    if (lifecycle_ != Lifecycle::Staged && lifecycle_ != Lifecycle::Running) {
        throw LifecycleError(LifecyclePhase::Step, "requires prior Stage()");
    }

    // Auto-log RUN phase on first step
    if (lifecycle_ == Lifecycle::Staged) {
        lifecycle_ = Lifecycle::Running;
        logger_.BeginLifecycle(LifecycleStrings::Run);
        logger_.LogRunStart(Time(), config_.t_end, config_.dt);
    }

    // Invoke input sources
    InvokeInputSources();

    // Build frame context (scheduler filtering)
    auto ctx = BuildFrameContext();

    // Cache current time for this step
    double t = Time();

    // If no state, just call components directly
    if (state_manager_.TotalSize() == 0) {
        for (auto &comp : components_) {
            if (ctx.ShouldRun(comp->FullName())) {
                ExecComponent(*comp, &Component<double>::PreStep, t, dt);
            }
        }
        for (auto &comp : components_) {
            if (ctx.ShouldRun(comp->FullName())) {
                ExecComponent(*comp, &Component<double>::Step, t, dt);
            }
        }
        for (auto &comp : components_) {
            if (ctx.ShouldRun(comp->FullName())) {
                ExecComponent(*comp, &Component<double>::PostStep, t, dt);
            }
        }
    } else {
        // Create derivative function for integrator
        auto deriv_func = [this, &ctx, dt](double t,
                                           const JanusVector<double> &x) -> JanusVector<double> {
            state_manager_.SetState(x);
            state_manager_.ZeroDerivatives();

            for (auto &comp : components_) {
                if (ctx.ShouldRun(comp->FullName())) {
                    ExecComponent(*comp, &Component<double>::PreStep, t, dt);
                    ExecComponent(*comp, &Component<double>::Step, t, dt);
                    ExecComponent(*comp, &Component<double>::PostStep, t, dt);
                }
            }

            return state_manager_.GetDerivatives(ctx.active_components);
        };

        // Integrate
        try {
            JanusVector<double> X = state_manager_.GetState();
            JanusVector<double> X_new = integration_manager_.Step(deriv_func, X, t, dt);
            state_manager_.SetState(X_new);
        } catch (const Error &e) {
            LogError(e, t, "Integrator");
            throw;
        }
    }

    // Compute time from frame count to avoid floating-point drift
    // (accumulating dt compounds rounding errors; multiplying has only one)
    frame_count_++;
    epoch_ = epoch_start_ + frame_count_ * dt;

    // Evaluate phase transitions
    phase_manager_.EvaluateTransitions(registry_);
    if (phase_manager_.PhaseChangedThisStep()) {
        logger_.Log(LogLevel::Info,
                    "[PHASE] Transition: " +
                        phase_manager_.GetConfig().GetPhaseName(phase_manager_.PreviousPhase()) +
                        " → " + phase_manager_.CurrentPhaseName() +
                        " at t=" + std::to_string(Time()) + "s");
    }

    // Periodic progress logging (e.g., every 100 frames)
    if (logger_.IsProgressEnabled() && (frame_count_ % 100 == 0)) {
        logger_.LogRunProgress(Time(), config_.t_end);
    }

    // Invoke output observers
    InvokeOutputObservers();

    // Record frame if enabled
    if (recorder_) {
        recorder_->Record(Time());
    }
}

inline void Simulator::Step() { Step(config_.dt); }

inline void Simulator::Reset() {
    if (lifecycle_ < Lifecycle::Staged) {
        throw LifecycleError(LifecyclePhase::Reset, "requires prior Stage()");
    }

    epoch_ = epoch_start_; // Reset to t=0 (MET derived via Time())
    frame_count_ = 0;

    // Zero state
    if (state_manager_.TotalSize() > 0) {
        JanusVector<double> zero =
            JanusVector<double>::Zero(static_cast<Eigen::Index>(state_manager_.TotalSize()));
        state_manager_.SetState(zero);
    }

    // Re-run Stage on components
    for (auto &comp : components_) {
        backplane_.set_context(comp->Entity(), comp->Name());
        try {
            comp->Stage(backplane_);
        } catch (const Error &e) {
            backplane_.clear_context();
            LogError(e, 0.0, comp->FullName());
            throw;
        }
        backplane_.clear_context();
    }

    lifecycle_ = Lifecycle::Staged;
}

inline void Simulator::SetTime(double met) {
    epoch_ = epoch_start_ + met;
    // Estimate frame count from MET and dt
    frame_count_ = static_cast<int>(met / config_.dt);
}

inline void Simulator::SetInputSource(const std::string &signal_name,
                                      InputSourceCallback callback) {
    input_sources_[signal_name] = std::move(callback);
}

inline void Simulator::SetOutputObserver(const std::string &signal_name,
                                         OutputObserverCallback callback) {
    output_observers_[signal_name] = std::move(callback);
}

inline void Simulator::ClearInputSource(const std::string &signal_name) {
    input_sources_.erase(signal_name);
}

inline void Simulator::ClearOutputObserver(const std::string &signal_name) {
    output_observers_.erase(signal_name);
}

inline Eigen::VectorXd Simulator::GetState() const { return state_manager_.GetState(); }

inline void Simulator::SetState(const Eigen::VectorXd &X) { state_manager_.SetState(X); }

inline Eigen::VectorXd Simulator::ComputeDerivatives(double t) {
    state_manager_.ZeroDerivatives();

    double dt = config_.dt;
    auto ctx = BuildFrameContext();

    for (auto &comp : components_) {
        if (ctx.ShouldRun(comp->FullName())) {
            ExecComponent(*comp, &Component<double>::PreStep, t, dt);
        }
    }
    for (auto &comp : components_) {
        if (ctx.ShouldRun(comp->FullName())) {
            ExecComponent(*comp, &Component<double>::Step, t, dt);
        }
    }
    for (auto &comp : components_) {
        if (ctx.ShouldRun(comp->FullName())) {
            ExecComponent(*comp, &Component<double>::PostStep, t, dt);
        }
    }

    return state_manager_.GetDerivatives(ctx.active_components);
}

inline AdaptiveStepResult<double> Simulator::AdaptiveStep(double dt_request) {
    if (lifecycle_ != Lifecycle::Staged && lifecycle_ != Lifecycle::Running) {
        throw LifecycleError(LifecyclePhase::Step, "AdaptiveStep() requires prior Stage()");
    }

    // Auto-log RUN phase on first step
    if (lifecycle_ == Lifecycle::Staged) {
        lifecycle_ = Lifecycle::Running;
        logger_.BeginLifecycle(LifecycleStrings::Run);
        logger_.LogRunStart(Time(), config_.t_end, config_.dt);
    }

    // Invoke input sources
    InvokeInputSources();

    // Build frame context (scheduler filtering)
    auto ctx = BuildFrameContext();

    // Cache current time
    double t = Time();

    // Derivative function with scheduler filtering
    auto deriv_func = [this, &ctx,
                       dt_request](double t, const JanusVector<double> &x) -> JanusVector<double> {
        state_manager_.SetState(x);
        state_manager_.ZeroDerivatives();

        for (auto &comp : components_) {
            if (ctx.ShouldRun(comp->FullName())) {
                ExecComponent(*comp, &Component<double>::PreStep, t, dt_request);
                ExecComponent(*comp, &Component<double>::Step, t, dt_request);
                ExecComponent(*comp, &Component<double>::PostStep, t, dt_request);
            }
        }

        return state_manager_.GetDerivatives(ctx.active_components);
    };

    JanusVector<double> X = state_manager_.GetState();
    AdaptiveStepResult<double> result;
    try {
        result = integration_manager_.AdaptiveStep(deriv_func, X, t, dt_request);
    } catch (const Error &e) {
        LogError(e, t, "Integrator");
        throw;
    }

    if (result.accepted) {
        state_manager_.SetState(result.state);
        epoch_ += result.dt_actual; // Single source of truth
        frame_count_++;

        // Invoke output observers
        InvokeOutputObservers();
    }

    return result;
}

inline std::optional<janus::Function> Simulator::GetDynamicsGraph() const {
    return dynamics_graph_;
}

inline std::optional<janus::Function> Simulator::GetJacobian() const { return jacobian_; }

inline DataDictionary Simulator::GetDataDictionary() const {
    DataDictionary dict;

    for (const auto &comp : components_) {
        DataDictionary::ComponentEntry entry;
        entry.name = comp->FullName();
        entry.type = comp->TypeName();
        entry.outputs = registry_.get_outputs_for_component(comp->FullName());
        entry.inputs = registry_.get_inputs_for_component(comp->FullName());
        entry.parameters = registry_.get_params_for_component(comp->FullName());
        entry.config = registry_.get_config_for_component(comp->FullName());
        dict.components.push_back(entry);
    }

    dict.ComputeStats();
    return dict;
}

inline IntrospectionGraph Simulator::GetIntrospectionGraph() const {
    IntrospectionGraph graph;
    graph.dictionary = GetDataDictionary();

    // 1. Explicit route edges (from SignalRouter)
    for (const auto &route : router_.GetRoutes()) {
        graph.edges.push_back({route.output_path, route.input_path, EdgeKind::Route});
    }

    // 2. Resolve-based edges (implicit source bindings)
    // The inputs_ map is populated during Stage() from Backplane::resolved_inputs().
    // Any component that calls bp.resolve() to read another component's output
    // gets its dependency captured here automatically.
    for (const auto &comp : components_) {
        auto it = inputs_.find(comp.get());
        if (it != inputs_.end()) {
            for (const auto &resolved_signal : it->second) {
                graph.edges.push_back({resolved_signal, comp->FullName(), EdgeKind::Resolve});
            }
        }
    }

    return graph;
}

inline void Simulator::InvokeInputSources() {
    for (const auto &[name, callback] : input_sources_) {
        double value = callback(name);
        registry_.SetByName(name, value);
    }
}

inline void Simulator::InvokeOutputObservers() {
    for (const auto &[name, callback] : output_observers_) {
        double value = registry_.GetByName(name);
        callback(name, value);
    }
}

inline Simulator::FrameContext Simulator::BuildFrameContext() const {
    FrameContext ctx;

    // Get active groups for this frame AND current flight phase
    int32_t current_phase = phase_manager_.CurrentPhase();
    auto active_groups = scheduler_.GetGroupsForFrame(frame_count_, current_phase);

    // Build set of active component names for O(1) lookup
    for (const auto &group_name : active_groups) {
        for (const auto &comp_name : scheduler_.GetMembersForGroup(group_name)) {
            ctx.active_components.insert(comp_name);
        }
    }

    // Build set of all scheduled component names (to detect unscheduled components)
    for (const auto &group : scheduler_.GetGroups()) {
        for (const auto &member : group.members) {
            ctx.all_scheduled_components.insert(member.component);
        }
    }

    return ctx;
}

inline void Simulator::ExecComponent(Component<double> &comp,
                                     void (Component<double>::*method)(double, double), double t,
                                     double dt) {
    try {
        logger_.BeginComponentTiming(comp.Name());
        (comp.*method)(t, dt);
        logger_.EndComponentTiming();
    } catch (const Error &e) {
        logger_.EndComponentTiming();
        LogError(e, t, comp.FullName());
        throw;
    }
}

} // namespace icarus
