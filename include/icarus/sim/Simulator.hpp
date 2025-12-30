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
#include <icarus/core/ErrorLogging.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/io/DataDictionary.hpp>
#include <icarus/io/MissionLogger.hpp>
#include <icarus/io/SimulationLoader.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/sim/IntegrationManager.hpp>
#include <icarus/sim/Scheduler.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/sim/StateManager.hpp>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Janus includes for symbolic graph generation
#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>

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

    /// Get current simulation time
    [[nodiscard]] double Time() const { return time_; }

    /// Get current simulation phase
    [[nodiscard]] Phase GetPhase() const { return phase_; }

    /// Check if simulation is initialized (Staged or later)
    [[nodiscard]] bool IsInitialized() const { return phase_ >= Phase::Staged; }

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
     * @brief Reset simulation to initial state
     * Re-applies ICs, resets time to 0.
     */
    void Reset();

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

    // Logging
    MissionLogger logger_;

    // Time and phase
    double time_ = 0.0;
    Phase phase_ = Phase::Uninitialized;
    int frame_count_ = 0;

    // Symbolic results (optional, after Stage)
    std::optional<janus::Function> dynamics_graph_;
    std::optional<janus::Function> jacobian_;

    // External callbacks
    std::unordered_map<std::string, InputSourceCallback> input_sources_;
    std::unordered_map<std::string, OutputObserverCallback> output_observers_;

    // Dependency tracking
    std::unordered_map<Component<double> *, std::vector<std::string>> outputs_;
    std::unordered_map<Component<double> *, std::vector<std::string>> inputs_;
};

// =============================================================================
// INLINE IMPLEMENTATIONS
// =============================================================================

inline Simulator::~Simulator() {
    // If we ever ran, log the debrief on destruction (RAII)
    if (phase_ == Phase::Running) {
        logger_.BeginPhase(SimPhase::Shutdown);
        auto wall_time = std::chrono::duration<double>(logger_.WallElapsed()).count();
        logger_.LogDebrief(time_, wall_time);
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

    // Begin Provision phase
    sim->logger_.BeginPhase(SimPhase::Provision);

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

    sim->logger_.EndPhase();

    return sim;
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
}

inline void Simulator::AddComponent(std::unique_ptr<Component<double>> component) {
    components_.push_back(std::move(component));
}

inline void Simulator::AddComponent(const ComponentConfig & /*config*/) {
    // TODO: Implement ComponentFactory
    throw std::runtime_error("ComponentFactory not yet implemented. Use AddComponent(unique_ptr) "
                             "for programmatic setup.");
}

inline void Simulator::AddRoutes(const std::vector<signal::SignalRoute> &routes) {
    for (const auto &route : routes) {
        router_.AddRoute(route);
    }
}

inline void Simulator::Provision() {
    if (phase_ != Phase::Uninitialized) {
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

    phase_ = Phase::Provisioned;
}

inline void Simulator::ApplyRouting() { router_.ApplyRoutes(backplane_); }

inline void Simulator::AllocateAndBindState() {
    state_manager_.AllocateState(components_);
    state_manager_.BindComponents(components_);
}

inline void Simulator::ApplyInitialConditions() {
    // ICs are typically applied in component Stage() methods
    // Config-driven ICs could be applied here in the future
}

inline void Simulator::ValidateWiring() {
    auto unwired = registry_.get_unwired_inputs();
    if (!unwired.empty()) {
        std::string msg = "Unwired inputs: ";
        for (const auto &name : unwired) {
            msg += name + ", ";
        }
        throw WiringError(msg);
    }
}

inline void Simulator::Stage() {
    // Auto-provision if components were added programmatically
    if (phase_ == Phase::Uninitialized && !components_.empty()) {
        Provision();
    }

    if (phase_ != Phase::Provisioned) {
        throw LifecycleError(LifecyclePhase::Stage, "requires components to be added first");
    }

    // Begin Stage phase logging
    logger_.BeginPhase(SimPhase::Stage);

    // Log wiring info (Debug level)
    for (const auto &route : router_.GetRoutes()) {
        std::string from = route.output_path;
        std::string to = route.input_path;
        logger_.LogWiring(from, to);
    }

    // Log scheduler execution order (Debug level)
    scheduler_.LogExecutionOrder(&logger_);

    // Stage each component (config loading, input wiring)
    for (auto &comp : components_) {
        backplane_.set_context(comp->Entity(), comp->Name());
        backplane_.clear_tracking();

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

    // Generate symbolic graphs if enabled
    if (config_.staging.symbolics.enabled) {
        // TODO: Implement symbolic graph generation
        // This requires creating symbolic components and tracing
    }

    // Run trim if enabled
    if (config_.staging.trim.enabled) {
        // TODO: Implement trim optimization
    }

    logger_.EndPhase();

    phase_ = Phase::Staged;
    time_ = 0.0;
}

inline void Simulator::Stage(const StageConfig &config) {
    config_.staging = config;
    Stage();
}

inline void Simulator::Step(double dt) {
    if (phase_ != Phase::Staged && phase_ != Phase::Running) {
        throw LifecycleError(LifecyclePhase::Step, "requires prior Stage()");
    }

    // Auto-log RUN phase on first step
    if (phase_ == Phase::Staged) {
        phase_ = Phase::Running;
        logger_.BeginPhase(SimPhase::Run);
        logger_.LogRunStart(time_, config_.t_end, config_.dt);
    }

    // Invoke input sources
    InvokeInputSources();

    // Get active groups for this frame
    auto active_groups = scheduler_.GetGroupsForFrame(frame_count_);

    // Helper to execute component method with error handling
    auto exec_component = [this](auto &comp, auto method, double t, double step_dt) {
        try {
            logger_.BeginComponentTiming(comp->Name());
            (comp.get()->*method)(t, step_dt);
            logger_.EndComponentTiming();
        } catch (const Error &e) {
            logger_.EndComponentTiming();
            LogError(e, t, comp->FullName());
            throw;
        }
    };

    // If no state, just call components directly
    if (state_manager_.TotalSize() == 0) {
        for (auto &comp : components_) {
            exec_component(comp, &Component<double>::PreStep, time_, dt);
        }
        for (auto &comp : components_) {
            exec_component(comp, &Component<double>::Step, time_, dt);
        }
        for (auto &comp : components_) {
            exec_component(comp, &Component<double>::PostStep, time_, dt);
        }
    } else {
        // Create derivative function for integrator
        auto deriv_func =
            [this, &exec_component](double t, const JanusVector<double> &x) -> JanusVector<double> {
            state_manager_.SetState(x);
            state_manager_.ZeroDerivatives();

            for (auto &comp : components_) {
                exec_component(comp, &Component<double>::PreStep, t, config_.dt);
                exec_component(comp, &Component<double>::Step, t, config_.dt);
                exec_component(comp, &Component<double>::PostStep, t, config_.dt);
            }

            return state_manager_.GetDerivatives();
        };

        // Integrate
        try {
            JanusVector<double> X = state_manager_.GetState();
            JanusVector<double> X_new = integration_manager_.Step(deriv_func, X, time_, dt);
            state_manager_.SetState(X_new);
        } catch (const Error &e) {
            LogError(e, time_, "Integrator");
            throw;
        }
    }

    time_ += dt;
    frame_count_++;

    // Periodic progress logging (e.g., every 100 frames)
    if (logger_.IsProgressEnabled() && (frame_count_ % 100 == 0)) {
        logger_.LogRunProgress(time_, config_.t_end);
    };

    // Invoke output observers
    InvokeOutputObservers();
}

inline void Simulator::Step() { Step(config_.dt); }

inline void Simulator::Reset() {
    if (phase_ < Phase::Staged) {
        throw LifecycleError(LifecyclePhase::Reset, "requires prior Stage()");
    }

    time_ = 0.0;
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

    phase_ = Phase::Staged;
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

    // Helper to execute with error handling
    auto exec = [this, t](auto &comp, auto method, double step_dt) {
        try {
            (comp.get()->*method)(t, step_dt);
        } catch (const Error &e) {
            LogError(e, t, comp->FullName());
            throw;
        }
    };

    for (auto &comp : components_) {
        exec(comp, &Component<double>::PreStep, dt);
    }
    for (auto &comp : components_) {
        exec(comp, &Component<double>::Step, dt);
    }
    for (auto &comp : components_) {
        exec(comp, &Component<double>::PostStep, dt);
    }

    return state_manager_.GetDerivatives();
}

inline AdaptiveStepResult<double> Simulator::AdaptiveStep(double dt_request) {
    if (phase_ != Phase::Staged && phase_ != Phase::Running) {
        throw LifecycleError(LifecyclePhase::Step, "AdaptiveStep() requires prior Stage()");
    }
    phase_ = Phase::Running;

    auto deriv_func = [this](double t, const JanusVector<double> &x) -> JanusVector<double> {
        state_manager_.SetState(x);
        return ComputeDerivatives(t);
    };

    JanusVector<double> X = state_manager_.GetState();
    AdaptiveStepResult<double> result;
    try {
        result = integration_manager_.AdaptiveStep(deriv_func, X, time_, dt_request);
    } catch (const Error &e) {
        LogError(e, time_, "Integrator");
        throw;
    }

    if (result.accepted) {
        state_manager_.SetState(result.state);
        time_ += result.dt_actual;
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

} // namespace icarus
