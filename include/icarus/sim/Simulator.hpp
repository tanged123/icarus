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
     * Calls Stage() on each component in order.
     */
    void Stage() {
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
};

} // namespace icarus
