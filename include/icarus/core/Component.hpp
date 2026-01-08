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
#include <vulcan/time/Epoch.hpp>

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

    /**
     * @brief Get simulation epoch (read-only)
     *
     * Provides access to the simulator's authoritative time.
     * Available after Stage(). Use for time-dependent calculations
     * (e.g., lunar ephemeris via jd_tt()).
     *
     * @return Pointer to current epoch, or nullptr if not yet staged
     */
    [[nodiscard]] const vulcan::time::Epoch<Scalar> *GetEpoch() const { return epoch_; }

  protected:
    // =========================================================================
    // Configuration Helpers
    // =========================================================================
    // These simplify reading config values in Stage() by:
    // - Eliminating redundant Has/Get patterns
    // - Automatically handling double→Scalar conversion for vectors
    // - Providing a clean one-liner API

    /**
     * @brief Read a scalar config parameter with default
     *
     * Usage: mass_ = read_param("mass", 100.0);
     */
    template <typename T> T read_param(const std::string &key, const T &default_val) const {
        return config_.template Get<T>(key, default_val);
    }

    /**
     * @brief Read a required scalar config parameter (throws if missing)
     *
     * Usage: mass_ = require_param<double>("mass");
     */
    template <typename T> T require_param(const std::string &key) const {
        return config_.template Require<T>(key);
    }

    /**
     * @brief Read a Vec3 config parameter with automatic double→Scalar conversion
     *
     * Usage: position_ = read_param_vec3("initial_position", Vec3<Scalar>::Zero());
     */
    Vec3<Scalar> read_param_vec3(const std::string &key, const Vec3<Scalar> &default_val) const {
        if (!config_.template Has<Vec3<double>>(key)) {
            return default_val;
        }
        auto v = config_.template Get<Vec3<double>>(key, Vec3<double>::Zero());
        return Vec3<Scalar>{static_cast<Scalar>(v(0)), static_cast<Scalar>(v(1)),
                            static_cast<Scalar>(v(2))};
    }

    /**
     * @brief Read a required Vec3 config parameter (throws if missing)
     *
     * Usage: position_ = require_param_vec3("initial_position");
     */
    Vec3<Scalar> require_param_vec3(const std::string &key) const {
        auto v = config_.template Require<Vec3<double>>(key);
        return Vec3<Scalar>{static_cast<Scalar>(v(0)), static_cast<Scalar>(v(1)),
                            static_cast<Scalar>(v(2))};
    }

    /**
     * @brief Read a Vec4 config parameter with automatic double→Scalar conversion
     *
     * Usage: attitude_ = read_param_vec4("initial_attitude", Vec4<Scalar>{1, 0, 0, 0});
     */
    Vec4<Scalar> read_param_vec4(const std::string &key, const Vec4<Scalar> &default_val) const {
        if (!config_.template Has<Vec4<double>>(key)) {
            return default_val;
        }
        auto v = config_.template Get<Vec4<double>>(key, Vec4<double>::Zero());
        return Vec4<Scalar>{static_cast<Scalar>(v(0)), static_cast<Scalar>(v(1)),
                            static_cast<Scalar>(v(2)), static_cast<Scalar>(v(3))};
    }

    /**
     * @brief Read a required Vec4 config parameter (throws if missing)
     *
     * Usage: attitude_ = require_param_vec4("initial_attitude");
     */
    Vec4<Scalar> require_param_vec4(const std::string &key) const {
        auto v = config_.template Require<Vec4<double>>(key);
        return Vec4<Scalar>{static_cast<Scalar>(v(0)), static_cast<Scalar>(v(1)),
                            static_cast<Scalar>(v(2)), static_cast<Scalar>(v(3))};
    }

    // =========================================================================
    // Lifecycle State Management
    // =========================================================================

    // Called by Simulator to track lifecycle state
    void MarkProvisioned() { provisioned_ = true; }
    void MarkStaged() { staged_ = true; }
    void ResetStaged() { staged_ = false; }

    // Called by Backplane during Stage() to bind epoch reference
    void BindEpoch(const vulcan::time::Epoch<Scalar> *epoch) { epoch_ = epoch; }

    // Allow Simulator to call Mark* methods
    template <typename S> friend class Simulator;
    friend class Simulator; // Non-templated Simulator (Phase 4.0.7)
    template <typename S> friend class Backplane;

  private:
    ComponentConfig config_;
    const vulcan::time::Epoch<Scalar> *epoch_ = nullptr;
    bool provisioned_ = false;
    bool staged_ = false;
};

} // namespace icarus
