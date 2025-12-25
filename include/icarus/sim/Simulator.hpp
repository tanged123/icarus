#pragma once

/**
 * @file Simulator.hpp
 * @brief Top-level simulation coordinator
 *
 * Part of Phase 1.4: Component Base.
 * Extended in Phase 2.2 with integrator support.
 * Owns components and orchestrates the Provision/Stage/Step lifecycle.
 */

#include <functional>
#include <icarus/core/Component.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/io/DataDictionary.hpp>
#include <icarus/io/LogConfig.hpp>
#include <icarus/io/MissionLogger.hpp>
#include <icarus/io/WiringConfig.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/sim/IntegratorFactory.hpp>
#include <icarus/sim/IntegratorTypes.hpp>
#include <icarus/sim/RK45Integrator.hpp>
#include <icarus/sim/RK4Integrator.hpp>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Janus includes for GenerateGraph() (only used when Scalar = SymbolicScalar)
#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>
#include <janus/math/AutoDiff.hpp>

namespace icarus {

/**
 * @brief Metadata for a component's state slice
 *
 * Tracks a component's position within the global state vectors.
 */
template <typename Scalar> struct StateSlice {
    Component<Scalar> *owner; ///< Component that owns this slice
    std::size_t offset;       ///< Starting index in X_global_
    std::size_t size;         ///< Number of state variables
};

/**
 * @brief Top-level simulation coordinator
 *
 * The Simulator owns all components and the signal backplane,
 * coordinating the Provision/Stage/Step lifecycle.
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class Simulator {
  public:
    Simulator() : backplane_(registry_) {}
    ~Simulator() = default;

    // Non-copyable, movable
    Simulator(const Simulator &) = delete;
    Simulator &operator=(const Simulator &) = delete;
    Simulator(Simulator &&) = default;
    Simulator &operator=(Simulator &&) = default;

    /**
     * @brief Add a component to the simulation
     *
     * Must be called before Provision().
     */
    void AddComponent(std::unique_ptr<Component<Scalar>> component) {
        components_.push_back(std::move(component));
    }

    /**
     * @brief Provision all components
     *
     * Calls Provision() on each component in order.
     */
    void Provision() {
        if (phase_ != Phase::Uninitialized) {
            throw LifecycleError("Provision() can only be called once");
        }
        for (auto &comp : components_) {
            backplane_.set_context(comp->Entity(), comp->Name());
            backplane_.clear_tracking();

            ComponentConfig config;
            config.name = comp->Name();
            config.entity = comp->Entity();
            configs_[comp.get()] = config;

            comp->Provision(backplane_, config);
            comp->MarkProvisioned();

            // Record outputs for dependency graph
            outputs_[comp.get()] = backplane_.registered_outputs();

            backplane_.clear_context();
        }
        phase_ = Phase::Provisioned;
    }

    /**
     * @brief Stage all components
     *
     * Allocates global state vectors and binds component state pointers,
     * then calls Stage() on each component for input wiring.
     */
    void Stage() {
        if (phase_ != Phase::Provisioned) {
            throw LifecycleError("Stage() requires prior Provision()");
        }

        // =====================================================================
        // Phase 2.1: Allocate global state vectors
        // =====================================================================
        std::size_t total_state_size = GetTotalStateSize();
        X_global_.resize(total_state_size);
        X_dot_global_.resize(total_state_size);
        state_layout_.clear();

        // Initialize to zero
        for (std::size_t i = 0; i < total_state_size; ++i) {
            X_global_[i] = Scalar{0};
            X_dot_global_[i] = Scalar{0};
        }

        // Bind each component to its slice
        std::size_t offset = 0;
        for (auto &comp : components_) {
            std::size_t state_size = comp->StateSize();

            if (state_size > 0) {
                comp->BindState(X_global_.data() + offset, X_dot_global_.data() + offset,
                                state_size);
                state_layout_.push_back({comp.get(), offset, state_size});
                offset += state_size;
            }
        }

        // Distribute wiring configuration to components
        const auto &all_wiring = wiring_config_.GetAllWirings();
        for (auto &comp : components_) {
            std::string prefix = comp->FullName() + ".";
            auto &comp_wiring = configs_[comp.get()].wiring;
            comp_wiring.clear();

            // Find all wirings for this component
            for (const auto &[input, source] : all_wiring) {
                if (input.find(prefix) == 0) {
                    comp_wiring[input] = source;
                }
            }
        }

        // =====================================================================
        // Existing Stage logic (signal wiring)
        // =====================================================================
        for (auto &comp : components_) {
            backplane_.set_context(comp->Entity(), comp->Name());
            backplane_.clear_tracking();

            comp->Stage(backplane_, configs_[comp.get()]);
            comp->MarkStaged();

            // Record inputs for dependency graph
            inputs_[comp.get()] = backplane_.resolved_inputs();

            backplane_.clear_context();
        }
        phase_ = Phase::Staged;
        time_ = Scalar{0};
    }

    /**
     * @brief Execute one time step using configured integrator
     *
     * Uses the integrator to advance state, which internally calls
     * ComputeDerivatives() at multiple points (for multi-stage methods).
     */
    void Step(Scalar dt) {
        if (phase_ != Phase::Staged && phase_ != Phase::Running) {
            throw LifecycleError("Step() requires prior Stage()");
        }
        phase_ = Phase::Running;

        // Invoke input sources before stepping
        InvokeInputSources();

        // If no stateful components, just call component Step() directly
        if (GetTotalStateSize() == 0) {
            for (auto &comp : components_) {
                comp->PreStep(time_, dt);
            }
            for (auto &comp : components_) {
                comp->Step(time_, dt);
            }
            for (auto &comp : components_) {
                comp->PostStep(time_, dt);
            }
            time_ = time_ + dt;

            // Invoke output observers after stepping
            InvokeOutputObservers();
            return;
        }

        // Create derivative function for integrator
        auto deriv_func = [this](Scalar t, const JanusVector<Scalar> &x) -> JanusVector<Scalar> {
            this->SetState(x);
            return this->ComputeDerivatives(t);
        };

        // Get current state and integrate
        JanusVector<Scalar> X = GetState();
        JanusVector<Scalar> X_new = integrator_->Step(deriv_func, X, time_, dt);

        // Update state and time
        SetState(X_new);
        time_ = time_ + dt;

        // Invoke output observers after stepping
        InvokeOutputObservers();
    }

    /**
     * @brief Get current simulation time
     */
    [[nodiscard]] Scalar Time() const { return time_; }

    /**
     * @brief Get current simulation phase
     */
    [[nodiscard]] Phase GetPhase() const { return phase_; }

    /**
     * @brief Get signal registry (for external access)
     */
    [[nodiscard]] SignalRegistry<Scalar> &GetRegistry() { return registry_; }
    [[nodiscard]] const SignalRegistry<Scalar> &GetRegistry() const { return registry_; }

    /**
     * @brief Get backplane (for external access)
     */
    [[nodiscard]] Backplane<Scalar> &GetBackplane() { return backplane_; }
    [[nodiscard]] const Backplane<Scalar> &GetBackplane() const { return backplane_; }

    /**
     * @brief Get signal value by name
     */
    [[nodiscard]] Scalar GetSignal(const std::string &name) const {
        return registry_.GetByName(name);
    }

    /**
     * @brief Set signal value by name
     */
    void SetSignal(const std::string &name, const Scalar &value) {
        registry_.SetByName(name, value);
    }

    // =========================================================================
    // External Interface API (Phase 3)
    // =========================================================================

    /**
     * @brief Read a signal value by name (typed)
     * @tparam T Expected signal type
     * @param name Full signal path: "Entity.Component.signal"
     * @return Current signal value
     */
    template <typename T> [[nodiscard]] T Peek(const std::string &name) const {
        return registry_.template GetByName<T>(name);
    }

    /**
     * @brief Write a signal value by name (typed)
     * @tparam T Signal type
     * @param name Full signal path
     * @param value New value to set
     */
    template <typename T> void Poke(const std::string &name, const T &value) {
        registry_.template SetByName<T>(name, value);
    }

    /**
     * @brief Read multiple signals at once (batch read)
     * @param names Vector of signal names
     * @return Map of name -> value
     */
    [[nodiscard]] std::map<std::string, Scalar>
    PeekBatch(const std::vector<std::string> &names) const {
        std::map<std::string, Scalar> result;
        for (const auto &name : names) {
            result[name] = GetSignal(name);
        }
        return result;
    }

    /**
     * @brief Write multiple signals at once (batch write)
     */
    void PokeBatch(const std::map<std::string, Scalar> &values) {
        for (const auto &[name, value] : values) {
            SetSignal(name, value);
        }
    }

    /**
     * @brief Initialize simulation (Provision + Stage)
     * Convenience method combining both lifecycle phases.
     */
    void Initialize() {
        if (phase_ == Phase::Uninitialized) {
            Provision();
        }
        if (phase_ == Phase::Provisioned) {
            Stage();
        }
    }

    /**
     * @brief Reset simulation to initial state
     * Keeps components, resets time to 0, restores initial conditions.
     */
    void Reset() {
        if (phase_ < Phase::Staged) {
            throw LifecycleError("Reset() requires prior Stage()");
        }
        time_ = Scalar{0};
        // Zero state vectors
        for (std::size_t i = 0; i < X_global_.size(); ++i) {
            X_global_[i] = Scalar{0};
            X_dot_global_[i] = Scalar{0};
        }
        phase_ = Phase::Staged;
    }

    /**
     * @brief Check if simulation is initialized (Staged or later)
     */
    [[nodiscard]] bool IsInitialized() const { return phase_ >= Phase::Staged; }

    /**
     * @brief Set simulation time (for warmstart/reset)
     */
    void SetTime(Scalar t) { time_ = t; }

    /**
     * @brief Get all signal names
     */
    [[nodiscard]] std::vector<std::string> GetSignalNames() const {
        return registry_.get_all_signal_names();
    }

    /**
     * @brief Get all input signal names
     */
    [[nodiscard]] std::vector<std::string> GetInputNames() const {
        return registry_.get_all_input_names();
    }

    /**
     * @brief Get all output signal names
     */
    [[nodiscard]] std::vector<std::string> GetOutputNames() const {
        return registry_.get_all_output_names();
    }

    // =========================================================================
    // External Control & Telemetry Callbacks (Phase 3.1.2)
    // =========================================================================

    /// Callback type for input sources: (signal_name) -> value
    using InputSourceCallback = std::function<Scalar(const std::string &)>;

    /// Callback type for output observers: (signal_name, value) -> void
    using OutputObserverCallback = std::function<void(const std::string &, const Scalar &)>;

    /**
     * @brief Register an external input source for a signal
     *
     * The callback will be invoked before each Step() to provide the input value.
     * Useful for pilot inputs, external controllers, hardware-in-the-loop.
     *
     * @param signal_name Name of the signal to control
     * @param callback Function returning the input value
     */
    void RegisterInputSource(const std::string &signal_name, InputSourceCallback callback) {
        input_sources_[signal_name] = std::move(callback);
    }

    /**
     * @brief Unregister an external input source
     */
    void UnregisterInputSource(const std::string &signal_name) {
        input_sources_.erase(signal_name);
    }

    /**
     * @brief Register an output observer for a signal
     *
     * The callback will be invoked after each Step() with the current value.
     * Useful for telemetry, visualization, external logging.
     *
     * @param signal_name Name of the signal to observe
     * @param callback Function receiving the signal value
     */
    void RegisterOutputObserver(const std::string &signal_name, OutputObserverCallback callback) {
        output_observers_[signal_name] = std::move(callback);
    }

    /**
     * @brief Unregister an output observer
     */
    void UnregisterOutputObserver(const std::string &signal_name) {
        output_observers_.erase(signal_name);
    }

    /**
     * @brief Get number of registered input sources
     */
    [[nodiscard]] std::size_t NumInputSources() const { return input_sources_.size(); }

    /**
     * @brief Get number of registered output observers
     */
    [[nodiscard]] std::size_t NumOutputObservers() const { return output_observers_.size(); }

    /**
     * @brief Get number of components
     */
    [[nodiscard]] std::size_t NumComponents() const { return components_.size(); }

    /**
     * @brief Get outputs registered by a component
     */
    [[nodiscard]] const std::vector<std::string> &
    GetComponentOutputs(Component<Scalar> *comp) const {
        static const std::vector<std::string> empty;
        auto it = outputs_.find(comp);
        return (it != outputs_.end()) ? it->second : empty;
    }

    /**
     * @brief Get inputs resolved by a component
     */
    [[nodiscard]] const std::vector<std::string> &
    GetComponentInputs(Component<Scalar> *comp) const {
        static const std::vector<std::string> empty;
        auto it = inputs_.find(comp);
        return (it != inputs_.end()) ? it->second : empty;
    }

    // =========================================================================
    // Phase 2.4: Wiring API
    // =========================================================================

    /**
     * @brief Configure wiring for a component or entity
     *
     * Helper to populate wiring config from code.
     *
     * @param prefix Component or Entity name prefix (e.g. "Gravity" or "Leader")
     * @param wiring_map Map of local input name -> full source name
     */
    void SetWiring(const std::string &prefix,
                   const std::map<std::string, std::string> &wiring_map) {
        for (const auto &[local, source] : wiring_map) {
            std::string full_input = prefix.empty() ? local : (prefix + "." + local);
            wiring_config_.AddWiring(full_input, source);
        }
    }

    /**
     * @brief Wire an input to a source signal
     *
     * Must be called after Provision() and before Stage().
     *
     * @tparam T The value type
     * @param input_name Full name of the input port
     * @param source_name Full name of the source signal
     */
    template <typename T> void Wire(const std::string &input_name, const std::string &source_name) {
        if (phase_ < Phase::Provisioned) {
            throw LifecycleError("Wire() requires prior Provision()");
        }
        registry_.template wire_input<T>(input_name, source_name);
    }

    /**
     * @brief Validate all inputs are wired
     *
     * @throws WiringError if any inputs are unwired
     */
    void ValidateWiring() const { registry_.validate_wiring(); }

    /**
     * @brief Get list of unwired inputs
     */
    [[nodiscard]] std::vector<std::string> GetUnwiredInputs() const {
        return registry_.get_unwired_inputs();
    }

    /**
     * @brief Load wiring configuration from YAML file
     *
     * Must be called after Provision() and before Stage().
     *
     * @param path Path to YAML wiring configuration file
     */
    void LoadWiring(const std::string &path) {
        if (phase_ < Phase::Provisioned) {
            throw LifecycleError("LoadWiring() requires prior Provision()");
        }
        wiring_config_ = WiringConfig::FromFile(path);
    }

    /**
     * @brief Load wiring configuration from WiringConfig object
     *
     * Must be called after Provision() and before Stage().
     *
     * @param config WiringConfig with input-to-source mappings
     */
    void LoadWiring(const WiringConfig &config) {
        if (phase_ < Phase::Provisioned) {
            throw LifecycleError("LoadWiring() requires prior Provision()");
        }
        wiring_config_ = config;
    }

    /**
     * @brief Get the loaded wiring configuration
     */
    [[nodiscard]] const WiringConfig &GetWiringConfig() const { return wiring_config_; }

    /**
     * @brief Generate data dictionary to file
     *
     * @param path Output file path (.yaml or .json based on extension)
     */
    void GenerateDataDictionary(const std::string &path) const {
        DataDictionary dict = GetDataDictionary();
        if (path.ends_with(".json")) {
            dict.ToJSON(path);
        } else {
            dict.ToYAML(path);
        }
    }

    /**
     * @brief Get data dictionary for the simulation
     *
     * @return DataDictionary with all component interfaces
     */
    [[nodiscard]] DataDictionary GetDataDictionary() const {
        DataDictionary dict;

        for (const auto &comp : components_) {
            DataDictionary::ComponentEntry entry;
            entry.name = comp->FullName();
            entry.type = comp->TypeName();

            // Query registry for this component's signals
            entry.outputs = registry_.get_outputs_for_component(comp->FullName());
            entry.inputs = registry_.get_inputs_for_component(comp->FullName());
            entry.parameters = registry_.get_params_for_component(comp->FullName());
            entry.config = registry_.get_config_for_component(comp->FullName());

            dict.components.push_back(entry);
        }

        dict.ComputeStats();
        return dict;
    }

    // =========================================================================
    // State Management (Phase 2.1)
    // =========================================================================

    /**
     * @brief Get total state vector size
     *
     * Sum of StateSize() across all components.
     */
    [[nodiscard]] std::size_t GetTotalStateSize() const {
        std::size_t total = 0;
        for (const auto &comp : components_) {
            total += comp->StateSize();
        }
        return total;
    }

    /**
     * @brief Get current state vector
     *
     * Returns a copy of X_global_ (for integrator use).
     */
    [[nodiscard]] JanusVector<Scalar> GetState() const { return X_global_; }

    /**
     * @brief Set state vector
     *
     * Copies values into X_global_. Called by integrator after advancing.
     */
    void SetState(const JanusVector<Scalar> &X) {
        if (X.size() != X_global_.size()) {
            throw StateError("State size mismatch: expected " + std::to_string(X_global_.size()) +
                             ", got " + std::to_string(X.size()));
        }
        X_global_ = X;
    }

    /**
     * @brief Compute derivatives for current state
     *
     * Called by integrator. Components read from X_global_ (via pointers)
     * and write to X_dot_global_ (via pointers).
     *
     * @param t Current time
     * @return Reference to derivative vector X_dot_global_
     */
    const JanusVector<Scalar> &ComputeDerivatives(Scalar t) {
        // Zero derivatives (components accumulate into them)
        for (std::size_t i = 0; i < X_dot_global_.size(); ++i) {
            X_dot_global_[i] = Scalar{0};
        }

        // All components compute derivatives
        // (pointers already bound, scatter/gather automatic)
        Scalar dt = dt_nominal_;
        for (auto &comp : components_) {
            comp->PreStep(t, dt);
        }
        for (auto &comp : components_) {
            comp->Step(t, dt);
        }
        for (auto &comp : components_) {
            comp->PostStep(t, dt);
        }

        return X_dot_global_;
    }

    /**
     * @brief Get derivative vector (after ComputeDerivatives)
     */
    [[nodiscard]] const JanusVector<Scalar> &GetDerivatives() const { return X_dot_global_; }

    /**
     * @brief Get state layout metadata
     *
     * Useful for debugging and introspection.
     */
    [[nodiscard]] const std::vector<StateSlice<Scalar>> &GetStateLayout() const {
        return state_layout_;
    }

    /**
     * @brief Set nominal timestep for derivative computation
     */
    void SetNominalDt(Scalar dt) { dt_nominal_ = dt; }

    /**
     * @brief Get nominal timestep
     */
    [[nodiscard]] Scalar GetNominalDt() const { return dt_nominal_; }

    // =========================================================================
    // Integrator Interface (Phase 2.2)
    // =========================================================================

    /**
     * @brief Set integrator using configuration
     */
    void SetIntegrator(const IntegratorConfig<Scalar> &config) {
        integrator_ = IntegratorFactory<Scalar>::Create(config);
        integrator_config_ = config;
    }

    /**
     * @brief Set integrator by type enum
     */
    void SetIntegrator(IntegratorType type) {
        SetIntegrator(IntegratorConfig<Scalar>::ForMethod(type));
    }

    /**
     * @brief Set integrator by type name string
     */
    void SetIntegrator(const std::string &type_name) {
        SetIntegrator(IntegratorConfig<Scalar>::ForMethod(parse_integrator_type(type_name)));
    }

    /**
     * @brief Set integrator directly
     *
     * If the integrator is adaptive, syncs tolerance config from the integrator.
     */
    void SetIntegrator(std::unique_ptr<Integrator<Scalar>> integrator) {
        integrator_ = std::move(integrator);
        integrator_config_.type = integrator_->Type();

        // Sync adaptive integrator config if applicable
        if (auto *adaptive = dynamic_cast<AdaptiveIntegrator<Scalar> *>(integrator_.get())) {
            integrator_config_.abs_tol = adaptive->GetAbsTol();
            integrator_config_.rel_tol = adaptive->GetRelTol();
        }
    }

    /**
     * @brief Get current integrator
     */
    [[nodiscard]] Integrator<Scalar> *GetIntegrator() const { return integrator_.get(); }

    /**
     * @brief Get current integrator configuration
     */
    [[nodiscard]] const IntegratorConfig<Scalar> &GetIntegratorConfig() const {
        return integrator_config_;
    }

    /**
     * @brief Get current integrator type
     */
    [[nodiscard]] IntegratorType GetIntegratorType() const { return integrator_config_.type; }

    /**
     * @brief Execute adaptive step (for RK45)
     *
     * Returns actual step taken and error estimate.
     * Requires adaptive integrator.
     */
    AdaptiveStepResult<Scalar> AdaptiveStep(Scalar dt_request) {
        if (phase_ != Phase::Staged && phase_ != Phase::Running) {
            throw LifecycleError("AdaptiveStep() requires prior Stage()");
        }
        phase_ = Phase::Running;

        auto *adaptive = dynamic_cast<AdaptiveIntegrator<Scalar> *>(integrator_.get());
        if (!adaptive) {
            throw IntegrationError("AdaptiveStep() requires an AdaptiveIntegrator");
        }

        auto deriv_func = [this](Scalar t, const JanusVector<Scalar> &x) -> JanusVector<Scalar> {
            this->SetState(x);
            return this->ComputeDerivatives(t);
        };

        JanusVector<Scalar> X = GetState();
        auto result = adaptive->AdaptiveStep(deriv_func, X, time_, dt_request);

        if (result.accepted) {
            SetState(result.state);
            time_ = time_ + result.dt_actual;
        }

        return result;
    }

    // =========================================================================
    // Logging Interface (Phase 2.5)
    // =========================================================================

    /**
     * @brief Get the mission logger
     */
    [[nodiscard]] MissionLogger &GetLogger() { return logger_; }
    [[nodiscard]] const MissionLogger &GetLogger() const { return logger_; }

    /**
     * @brief Enable/disable quiet mode (suppress all but errors)
     */
    void SetQuietMode(bool quiet) {
        if (quiet) {
            logger_.SetConsoleLevel(LogLevel::Error);
            logger_.SetProgressEnabled(false);
        } else {
            logger_.SetConsoleLevel(LogLevel::Info);
            logger_.SetProgressEnabled(true);
        }
    }

    /**
     * @brief Set log file path
     *
     * When set, all logs are written to the specified file.
     */
    void SetLogFile(const std::string &path) { logger_.SetLogFile(path); }

    /**
     * @brief Enable/disable profiling
     */
    void SetProfilingEnabled(bool enabled) { logger_.SetProfilingEnabled(enabled); }

    /**
     * @brief Apply logging configuration
     */
    void ApplyLogConfig(const LogConfig &config) {
        if (config.file_enabled && !config.file_path.empty()) {
            logger_.SetLogFile(config.file_path);
            logger_.SetFileLevel(config.file_level);
        }
        logger_.SetConsoleLevel(config.console_level);
        logger_.SetProgressEnabled(config.progress_enabled);
        logger_.SetProfilingEnabled(config.profiling_enabled);
    }

    // =========================================================================
    // Graph Generation (Phase 3.3.4) - Symbolic Mode Only
    // =========================================================================

    /**
     * @brief Generate dynamics graph as janus::Function (symbolic mode only)
     *
     * Extracts the computational graph xdot = f(t, x) from the simulation.
     * Only available when Scalar = SymbolicScalar.
     *
     * @tparam S SFINAE helper (defaults to Scalar)
     * @return janus::Function representing the dynamics
     * @throws LifecycleError if simulator is not staged
     *
     * Usage:
     *   Simulator<SymbolicScalar> sim;
     *   sim.Initialize();
     *   auto dynamics = sim.GenerateGraph();
     *   auto xdot = dynamics(t, x);  // Evaluate numerically
     */
    template <typename S = Scalar, typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
    [[nodiscard]] janus::Function GenerateGraph() {
        if (!IsInitialized()) {
            throw LifecycleError("GenerateGraph() requires a Staged simulator");
        }

        std::size_t n_states = GetTotalStateSize();

        // Create symbolic time
        auto t_sym = janus::sym("t");

        // Use sym_vec_pair to get both:
        // - x_vec (SymbolicVector) for templated simulator code
        // - x_mx (raw MX) for janus::Function inputs
        auto [x_vec, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states));

        // Set state using SymbolicVector (works with templates)
        SetState(x_vec);
        SetTime(t_sym);

        // Compute derivatives
        ComputeDerivatives(t_sym);

        // Gather derivatives
        const auto &X_dot = GetDerivatives();
        std::vector<SymbolicScalar> xdot_elements;
        xdot_elements.reserve(n_states);
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(n_states); ++i) {
            xdot_elements.push_back(X_dot(i));
        }
        SymbolicScalar xdot_mx = SymbolicScalar::vertcat(xdot_elements);

        // Build function with raw MX as input
        std::vector<janus::SymbolicArg> inputs = {t_sym, x_mx};
        std::vector<janus::SymbolicArg> outputs = {xdot_mx};

        return janus::Function("dynamics", inputs, outputs);
    }

    /**
     * @brief Generate state Jacobian as janus::Function (symbolic mode only)
     *
     * Computes the Jacobian df/dx where xdot = f(t, x).
     * Only available when Scalar = SymbolicScalar.
     *
     * @return janus::Function computing the Jacobian matrix
     */
    template <typename S = Scalar, typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
    [[nodiscard]] janus::Function GenerateJacobian() {
        std::size_t n_states = GetTotalStateSize();

        // Create symbolic variables for the Jacobian function
        auto t_sym = janus::sym("t");
        auto [x_vec, x_mx] = janus::sym_vec_pair("x", static_cast<int>(n_states));

        // Get dynamics and evaluate
        auto dynamics = GenerateGraph();
        auto xdot_result = dynamics.eval(t_sym, x_mx);

        // Compute Jacobian
        auto J_sym = janus::jacobian({janus::as_mx(xdot_result)}, {x_mx});

        return janus::Function("jacobian", {t_sym, x_mx}, {J_sym});
    }

  private:
    std::vector<std::unique_ptr<Component<Scalar>>> components_;
    SignalRegistry<Scalar> registry_;
    Backplane<Scalar> backplane_;
    Scalar time_{};
    Phase phase_ = Phase::Uninitialized;

    // Configuration and dependency tracking
    std::unordered_map<Component<Scalar> *, ComponentConfig> configs_;
    std::unordered_map<Component<Scalar> *, std::vector<std::string>> outputs_;
    std::unordered_map<Component<Scalar> *, std::vector<std::string>> inputs_;

    // State management (Phase 2.1)
    JanusVector<Scalar> X_global_;                 ///< Global state vector
    JanusVector<Scalar> X_dot_global_;             ///< Global derivative vector
    std::vector<StateSlice<Scalar>> state_layout_; ///< Per-component metadata
    Scalar dt_nominal_{0.01};                      ///< Nominal timestep

    // Integrator (Phase 2.2) - defaults to RK4
    std::unique_ptr<Integrator<Scalar>> integrator_ = std::make_unique<RK4Integrator<Scalar>>();
    IntegratorConfig<Scalar> integrator_config_ = IntegratorConfig<Scalar>::RK4Default();

    // Wiring configuration (Phase 2.4)
    WiringConfig wiring_config_;

    // Logging (Phase 2.5)
    MissionLogger logger_;

    // External callbacks (Phase 3.1.2)
    std::unordered_map<std::string, InputSourceCallback> input_sources_;
    std::unordered_map<std::string, OutputObserverCallback> output_observers_;

    /**
     * @brief Invoke all registered input sources before Step()
     */
    void InvokeInputSources() {
        for (const auto &[signal_name, callback] : input_sources_) {
            try {
                Scalar value = callback(signal_name);
                SetSignal(signal_name, value);
            } catch (const std::exception &e) {
                // Log but don't fail - external sources may be unreliable
                logger_.Log(LogLevel::Warning,
                            "InputSource callback failed for " + signal_name + ": " + e.what());
            }
        }
    }

    /**
     * @brief Invoke all registered output observers after Step()
     */
    void InvokeOutputObservers() {
        for (const auto &[signal_name, callback] : output_observers_) {
            try {
                Scalar value = GetSignal(signal_name);
                callback(signal_name, value);
            } catch (const std::exception &e) {
                // Log but don't fail - external observers may be unreliable
                logger_.Log(LogLevel::Warning,
                            "OutputObserver callback failed for " + signal_name + ": " + e.what());
            }
        }
    }
};

} // namespace icarus
