#pragma once

/**
 * @file Simulator.hpp
 * @brief Top-level simulation coordinator
 *
 * Part of Phase 1.4: Component Base.
 * Owns components and orchestrates the Provision/Stage/Step lifecycle.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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
     * @brief Execute one time step
     *
     * Calls Step() on each component in order.
     */
    void Step(Scalar dt) {
        phase_ = Phase::Running;
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
};

} // namespace icarus
