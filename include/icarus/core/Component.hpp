#pragma once

/**
 * @file Component.hpp
 * @brief Base class for all simulation components
 *
 * Part of Phase 1.4: Component Base.
 * Components implement the Provision/Stage/Step lifecycle.
 */

#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <string>
#include <vector>

namespace icarus {

// Forward declarations
template <typename Scalar> class Backplane;

/**
 * @brief Signal declaration for introspection
 */
struct SignalDecl {
    std::string name;        ///< Local signal name
    std::string unit;        ///< Physical unit
    std::string description; ///< Human-readable description
    bool is_input = false;   ///< True if input, false if output
};

/**
 * @brief Base class for all simulation components
 *
 * Components are the fundamental unit of execution in Icarus. They own
 * state and implement the Provision/Stage/Step lifecycle.
 *
 * **Lifecycle:**
 * 1. Provision (once) - Register outputs, allocate memory, load params
 * 2. Stage (per run) - Wire inputs, apply ICs
 * 3. Step (per dt) - Compute dynamics (hot path!)
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class Component {
  public:
    virtual ~Component() = default;

    // =========================================================================
    // Core Lifecycle (Required)
    // =========================================================================

    /**
     * @brief Provision phase - called once at application launch
     *
     * Heavy lifting: allocate memory, register signals.
     *
     * @param bp Backplane for registering outputs
     */
    virtual void Provision(Backplane<Scalar> &bp) = 0;

    /**
     * @brief Stage phase - called at start of each run/episode
     *
     * Load parameters from config (via GetConfig()), apply initial conditions,
     * prepare for t=0. This is where default values and config loading happens.
     *
     * @param bp Backplane for resolving inputs
     */
    virtual void Stage(Backplane<Scalar> &bp) = 0;

    /**
     * @brief Step phase - called every time step (hot path!)
     *
     * Read inputs, compute derivatives, write outputs.
     * NO allocation, NO string lookups.
     *
     * @param t Current simulation time
     * @param dt Time step size
     */
    virtual void Step(Scalar t, Scalar dt) = 0;

    // =========================================================================
    // Extended Lifecycle Hooks (Optional)
    // =========================================================================

    /**
     * @brief Called before any component Steps (for pre-processing)
     */
    virtual void PreStep(Scalar /*t*/, Scalar /*dt*/) {}

    /**
     * @brief Called after all component Steps (for post-processing)
     */
    virtual void PostStep(Scalar /*t*/, Scalar /*dt*/) {}

    /**
     * @brief Called when entering a new flight phase
     */
    virtual void OnPhaseEnter(Phase /*phase*/) {}

    /**
     * @brief Called when exiting a flight phase
     */
    virtual void OnPhaseExit(Phase /*phase*/) {}

    /**
     * @brief Called when simulation encounters an error
     */
    virtual void OnError(const SimulationError & /*error*/) {}

    /**
     * @brief Called during shutdown (cleanup, flush buffers)
     */
    virtual void Shutdown() {}

    // =========================================================================
    // Identity & Introspection
    // =========================================================================

    /**
     * @brief Component instance name (e.g., "MainEngine")
     */
    [[nodiscard]] virtual std::string Name() const = 0;

    /**
     * @brief Entity namespace (e.g., "X15")
     *
     * Return empty string for singleton components (e.g., Environment).
     */
    [[nodiscard]] virtual std::string Entity() const { return ""; }

    /**
     * @brief Component type name for data dictionary (e.g., "JetEngine")
     */
    [[nodiscard]] virtual std::string TypeName() const { return Name(); }

    /**
     * @brief Full qualified name: entity.component (or just component)
     */
    [[nodiscard]] std::string FullName() const { return MakeFullPath(Entity(), Name()); }

    /**
     * @brief Declared inputs (for documentation/dependency graph)
     *
     * Override to declare expected inputs for tooling.
     */
    [[nodiscard]] virtual std::vector<SignalDecl> DeclareInputs() const { return {}; }

    /**
     * @brief Declared outputs (for documentation/dependency graph)
     *
     * Override to declare outputs for tooling.
     */
    [[nodiscard]] virtual std::vector<SignalDecl> DeclareOutputs() const { return {}; }

    /**
     * @brief Get list of output signal names
     */
    [[nodiscard]] std::vector<std::string> GetOutputNames() const {
        std::vector<std::string> names;
        for (const auto &decl : DeclareOutputs()) {
            names.push_back(decl.name);
        }
        return names;
    }

    /**
     * @brief Get list of input signal names
     */
    [[nodiscard]] std::vector<std::string> GetInputNames() const {
        std::vector<std::string> names;
        for (const auto &decl : DeclareInputs()) {
            names.push_back(decl.name);
        }
        return names;
    }

    /**
     * @brief Number of state variables owned by this component
     */
    [[nodiscard]] virtual std::size_t StateSize() const { return 0; }

    /**
     * @brief Check if component has integrated state
     */
    [[nodiscard]] bool HasState() const { return StateSize() > 0; }

    // =========================================================================
    // State Management (Phase 2.1)
    // =========================================================================

    /**
     * @brief Bind component to slices of global state vectors
     *
     * Called by Simulator during Stage(). Component stores pointers
     * into X_global_ and X_dot_global_ for efficient access during Step().
     *
     * Default implementation does nothing (component has no state).
     * Override if StateSize() > 0.
     *
     * @param state_ptr Pointer to X_global_[offset] for this component
     * @param state_dot_ptr Pointer to X_dot_global_[offset] for this component
     * @param state_size Number of state variables (matches StateSize())
     */
    virtual void BindState(Scalar * /*state_ptr*/, Scalar * /*state_dot_ptr*/,
                           std::size_t /*state_size*/) {
        // Default: no state binding (component has no integrated state)
    }

    // =========================================================================
    // Lifecycle State
    // =========================================================================

    /**
     * @brief Check if Provision has been called
     */
    [[nodiscard]] bool IsProvisioned() const { return provisioned_; }

    /**
     * @brief Check if Stage has been called
     */
    [[nodiscard]] bool IsStaged() const { return staged_; }

    // =========================================================================
    // Configuration Access
    // =========================================================================

    /**
     * @brief Set component configuration (called by factory after construction)
     */
    void SetConfig(ComponentConfig config) { config_ = std::move(config); }

    /**
     * @brief Get component configuration
     *
     * Use in Stage() to read parameters, initial conditions, etc.
     */
    [[nodiscard]] const ComponentConfig &GetConfig() const { return config_; }

  protected:
    // Called by Simulator to track lifecycle state
    void MarkProvisioned() { provisioned_ = true; }
    void MarkStaged() { staged_ = true; }
    void ResetStaged() { staged_ = false; }

    // Allow Simulator to call Mark* methods
    template <typename S> friend class Simulator;
    friend class Simulator; // Non-templated Simulator (Phase 4.0.7)

  private:
    ComponentConfig config_;
    bool provisioned_ = false;
    bool staged_ = false;
};

} // namespace icarus
