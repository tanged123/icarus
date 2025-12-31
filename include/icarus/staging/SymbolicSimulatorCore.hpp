#pragma once

/**
 * @file SymbolicSimulatorCore.hpp
 * @brief Lightweight symbolic simulator for graph extraction
 *
 * Part of Phase 4: Staging Implementation (Phase C.2)
 *
 * Creates symbolic components from the same configs as the numeric sim.
 * Used internally during Stage() for trim/linearization/graph export.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/ComponentFactory.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/sim/StateManager.hpp>

#include <janus/core/JanusTypes.hpp>

#include <memory>
#include <string>
#include <vector>

namespace icarus::staging {

/**
 * @brief Lightweight symbolic simulator for graph extraction
 *
 * Creates symbolic components from the same configs as the numeric sim.
 * Used internally during Stage() for:
 *   - Symbolic trim (exact Jacobians)
 *   - Symbolic linearization (exact A, B, C, D matrices)
 *   - Dynamics graph export (janus::Function)
 *
 * Unlike the main Simulator, this class:
 *   - Only supports SymbolicScalar (casadi::MX)
 *   - Does not integrate time (no Step method)
 *   - Provides direct access to symbolic derivatives
 */
class SymbolicSimulatorCore {
  public:
    using Scalar = janus::SymbolicScalar;

    /**
     * @brief Create from simulator config
     *
     * Creates symbolic versions of all components using the ComponentFactory.
     * Provisions and stages them with symbolic backplane.
     *
     * @param config SimulatorConfig used by numeric simulator
     * @throws ConfigError if symbolic components can't be created
     */
    explicit SymbolicSimulatorCore(const SimulatorConfig &config)
        : config_(config), backplane_(registry_) {
        CreateComponents();
        ProvisionComponents();
        ApplyRouting();
        AllocateState();
        StageComponents();
    }

    // =========================================================================
    // State Access
    // =========================================================================

    /**
     * @brief Set state vector (symbolic)
     *
     * Copies the symbolic state into the internal state vector and
     * updates all component state bindings.
     */
    void SetState(const JanusVector<Scalar> &x) {
        if (x.size() != state_manager_.GetState().size()) {
            throw std::invalid_argument("SymbolicSimulatorCore::SetState: size mismatch");
        }
        state_manager_.SetState(x);
    }

    /**
     * @brief Set time (symbolic)
     */
    void SetTime(Scalar t) { time_ = t; }

    /**
     * @brief Compute derivatives symbolically
     *
     * Calls PreStep/Step/PostStep on all components to trace the
     * symbolic computational graph.
     *
     * @return Symbolic derivative vector xdot
     */
    JanusVector<Scalar> ComputeDerivatives() {
        state_manager_.ZeroDerivatives();

        Scalar dt = Scalar(config_.dt);

        for (auto &comp : components_) {
            comp->PreStep(time_, dt);
        }
        for (auto &comp : components_) {
            comp->Step(time_, dt);
        }
        for (auto &comp : components_) {
            comp->PostStep(time_, dt);
        }

        return state_manager_.GetDerivatives();
    }

    /**
     * @brief Get total state size
     */
    [[nodiscard]] std::size_t GetStateSize() const { return state_manager_.TotalSize(); }

    /**
     * @brief Get current state vector (copy)
     */
    [[nodiscard]] JanusVector<Scalar> GetState() const { return state_manager_.GetState(); }

    /**
     * @brief Get derivative vector (after ComputeDerivatives)
     */
    [[nodiscard]] const JanusVector<Scalar> &GetDerivatives() const {
        return state_manager_.GetDerivatives();
    }

    // =========================================================================
    // Signal Access
    // =========================================================================

    /**
     * @brief Read signal value (symbolic)
     */
    [[nodiscard]] Scalar GetSignal(const std::string &name) const {
        return registry_.GetByName(name);
    }

    /**
     * @brief Write signal value (symbolic)
     */
    void SetSignal(const std::string &name, Scalar value) { registry_.SetByName(name, value); }

    /**
     * @brief Check if signal exists
     */
    [[nodiscard]] bool HasSignal(const std::string &name) const {
        return registry_.HasSignal(name);
    }

    /**
     * @brief Get all registered signal names
     */
    [[nodiscard]] std::vector<std::string> GetSignalNames() const {
        return registry_.get_all_signal_names();
    }

    /**
     * @brief Get state layout (component slices)
     */
    [[nodiscard]] const std::vector<StateSlice<Scalar>> &GetStateLayout() const {
        return state_manager_.GetLayout();
    }

    /**
     * @brief Get component by name
     */
    [[nodiscard]] Component<Scalar> *GetComponent(const std::string &name) const {
        for (const auto &comp : components_) {
            if (comp->FullName() == name) {
                return comp.get();
            }
        }
        return nullptr;
    }

    /**
     * @brief Get number of components
     */
    [[nodiscard]] std::size_t NumComponents() const { return components_.size(); }

  private:
    // =========================================================================
    // Private Methods
    // =========================================================================

    void CreateComponents() {
        auto &factory = ComponentFactory<Scalar>::Instance();

        for (const auto &comp_cfg : config_.components) {
            if (!factory.HasType(comp_cfg.type)) {
                throw ConfigError("SymbolicSimulatorCore: No symbolic registration for component "
                                  "type '" +
                                  comp_cfg.type +
                                  "'. "
                                  "Ensure component uses ICARUS_REGISTER_COMPONENT macro.");
            }
            auto component = factory.Create(comp_cfg);
            components_.push_back(std::move(component));
        }
    }

    void ProvisionComponents() {
        for (auto &comp : components_) {
            backplane_.set_context(comp->Entity(), comp->Name());
            backplane_.clear_tracking();
            comp->Provision(backplane_);
            // Component marks itself as provisioned internally
            backplane_.clear_context();
        }
    }

    void ApplyRouting() {
        for (const auto &route : config_.routes) {
            router_.AddRoute(route);
        }
        router_.ApplyRoutes(backplane_);
    }

    void AllocateState() {
        state_manager_.AllocateState(components_);
        state_manager_.BindComponents(components_);
    }

    void StageComponents() {
        for (auto &comp : components_) {
            backplane_.set_context(comp->Entity(), comp->Name());
            backplane_.clear_tracking();
            comp->Stage(backplane_);
            // Component marks itself as staged internally
            backplane_.clear_context();
        }
    }

    // =========================================================================
    // Private Data
    // =========================================================================

    SimulatorConfig config_;

    // Components
    std::vector<std::unique_ptr<Component<Scalar>>> components_;

    // Signal system
    SignalRegistry<Scalar> registry_;
    Backplane<Scalar> backplane_;
    signal::SignalRouter<Scalar> router_;

    // State management
    StateManager<Scalar> state_manager_;

    // Time
    Scalar time_{0.0};
};

} // namespace icarus::staging
