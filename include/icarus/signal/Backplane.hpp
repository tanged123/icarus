#pragma once

/**
 * @file Backplane.hpp
 * @brief Component-facing facade for signal registration and resolution
 *
 * Part of Phase 1.4: Component Base.
 * Wraps SignalRegistry with component context awareness.
 */

#include <icarus/core/Config.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/VecHandle.hpp>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Component-facing facade for signal registration and resolution
 *
 * Backplane adds:
 * - Automatic full name generation (entity.component.signal)
 * - Context tracking for dependency discovery
 * - Cleaner API for component authors
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class Backplane {
  public:
    explicit Backplane(SignalRegistry<Scalar> &registry) : registry_(registry) {}

    // =========================================================================
    // Context Management
    // =========================================================================

    /**
     * @brief Set current component context
     *
     * Called by Simulator before invoking component lifecycle methods.
     */
    void set_context(const std::string &entity, const std::string &component) {
        entity_ = entity;
        component_ = component;
        registry_.set_current_component(full_prefix());
    }

    /**
     * @brief Clear the current context
     */
    void clear_context() {
        entity_.clear();
        component_.clear();
        registry_.clear_current_component();
    }

    /**
     * @brief Get full prefix: entity.component (or just component if no entity)
     */
    [[nodiscard]] std::string full_prefix() const {
        if (entity_.empty())
            return component_;
        return entity_ + "." + component_;
    }

    // =========================================================================
    // Output Registration (Provision phase)
    // =========================================================================

    /**
     * @brief Register a scalar output signal
     *
     * @param local_name Signal name (without entity/component prefix)
     * @param data_ptr Pointer to component-owned storage
     * @param unit Physical unit
     * @param description Human-readable description
     */
    template <typename T>
    void register_output(const std::string &local_name, T *data_ptr, const std::string &unit = "",
                         const std::string &description = "") {
        ICARUS_ASSERT(!component_.empty(), "Context must be set before registration");
        std::string full_name = make_full_name(local_name);
        registry_.template register_output<T>(full_name, data_ptr, unit, description);
        registered_outputs_.push_back(full_name);
    }

    /**
     * @brief Register a Vec3 output signal (expands to .x/.y/.z)
     */
    template <typename S>
    void register_output_vec3(const std::string &local_name, Vec3<S> *data_ptr,
                              const std::string &unit = "", const std::string &description = "") {
        std::string full_name = make_full_name(local_name);
        registry_.template register_output_vec3<S>(full_name, data_ptr, unit, description);
        registered_outputs_.push_back(full_name + ".x");
        registered_outputs_.push_back(full_name + ".y");
        registered_outputs_.push_back(full_name + ".z");
    }

    /**
     * @brief Register a quaternion output signal (expands to .w/.x/.y/.z)
     */
    template <typename S>
    void register_output_quat(const std::string &local_name, Vec4<S> *data_ptr,
                              const std::string &unit = "", const std::string &description = "") {
        std::string full_name = make_full_name(local_name);
        registry_.template register_output_quat<S>(full_name, data_ptr, unit, description);
        registered_outputs_.push_back(full_name + ".w");
        registered_outputs_.push_back(full_name + ".x");
        registered_outputs_.push_back(full_name + ".y");
        registered_outputs_.push_back(full_name + ".z");
    }

    // =========================================================================
    // Input Resolution (Stage phase)
    // =========================================================================

    /**
     * @brief Resolve a signal by full path
     *
     * Tracks the resolution for dependency discovery.
     *
     * @param full_name Full signal path (e.g., "Environment.Atm.density")
     * @return SignalHandle<T> for zero-overhead access
     */
    template <typename T> [[nodiscard]] SignalHandle<T> resolve(const std::string &full_name) {
        resolved_inputs_.push_back(full_name);
        return registry_.template resolve<T>(full_name);
    }

    /**
     * @brief Resolve a Vec3 signal
     */
    template <typename S> [[nodiscard]] Vec3Handle<S> resolve_vec3(const std::string &full_name) {
        resolved_inputs_.push_back(full_name + ".x");
        resolved_inputs_.push_back(full_name + ".y");
        resolved_inputs_.push_back(full_name + ".z");
        return registry_.template resolve_vec3<S>(full_name);
    }

    /**
     * @brief Resolve a quaternion signal
     */
    template <typename S> [[nodiscard]] QuatHandle<S> resolve_quat(const std::string &full_name) {
        resolved_inputs_.push_back(full_name + ".w");
        resolved_inputs_.push_back(full_name + ".x");
        resolved_inputs_.push_back(full_name + ".y");
        resolved_inputs_.push_back(full_name + ".z");
        return registry_.template resolve_quat<S>(full_name);
    }

    /**
     * @brief Check if a signal exists
     */
    [[nodiscard]] bool has_signal(const std::string &full_name) const {
        return registry_.HasSignal(full_name);
    }

    // =========================================================================
    // Phase 2.4: Input Registration
    // =========================================================================

    /**
     * @brief Register an input port
     *
     * @tparam T The value type
     * @param local_name Local signal name
     * @param handle Pointer to InputHandle owned by component
     * @param units Physical units
     * @param description Human-readable description
     */
    template <typename T>
    void register_input(const std::string &local_name, InputHandle<T> *handle,
                        const std::string &units = "", const std::string &description = "") {
        ICARUS_ASSERT(!component_.empty(), "Context must be set before registration");
        std::string full_name = make_full_name(local_name);
        handle->set_full_name(full_name);
        registry_.template register_input<T>(full_name, handle, units, description);
        registered_inputs_.push_back(full_name);
    }

    // =========================================================================
    // Phase 2.4: Parameter Registration
    // =========================================================================

    /**
     * @brief Register a Scalar parameter (optimizable)
     */
    void register_param(const std::string &local_name, Scalar *storage, Scalar initial_value,
                        const std::string &units = "", const std::string &description = "") {
        ICARUS_ASSERT(!component_.empty(), "Context must be set before registration");
        std::string full_name = make_full_name(local_name);
        registry_.register_param(full_name, storage, initial_value, units, description);
        registered_params_.push_back(full_name);
    }

    // =========================================================================
    // Phase 2.4: Config Registration
    // =========================================================================

    /**
     * @brief Register an int config value
     */
    void register_config(const std::string &local_name, int *storage, int initial_value,
                         const std::string &description = "") {
        ICARUS_ASSERT(!component_.empty(), "Context must be set before registration");
        std::string full_name = make_full_name(local_name);
        registry_.register_config(full_name, storage, initial_value, description);
        registered_config_.push_back(full_name);
    }

    /**
     * @brief Register a bool config value
     */
    void register_config(const std::string &local_name, bool *storage, bool initial_value,
                         const std::string &description = "") {
        ICARUS_ASSERT(!component_.empty(), "Context must be set before registration");
        std::string full_name = make_full_name(local_name);
        registry_.register_config(full_name, storage, initial_value, description);
        registered_config_.push_back(full_name);
    }

    // =========================================================================
    // Phase 2.4: Wiring
    // =========================================================================

    /**
     * @brief Wire an input to a source signal
     */
    template <typename T>
    void wire_input(const std::string &input_name, const std::string &source_name) {
        registry_.template wire_input<T>(input_name, source_name);
    }

    // =========================================================================
    // Dependency Tracking
    // =========================================================================

    /**
     * @brief Get outputs registered by current component
     */
    [[nodiscard]] const std::vector<std::string> &registered_outputs() const {
        return registered_outputs_;
    }

    /**
     * @brief Get inputs registered by current component (Phase 2.4)
     */
    [[nodiscard]] const std::vector<std::string> &registered_inputs() const {
        return registered_inputs_;
    }

    /**
     * @brief Get legacy resolved inputs by current component
     */
    [[nodiscard]] const std::vector<std::string> &resolved_inputs() const {
        return resolved_inputs_;
    }

    /**
     * @brief Get parameters registered by current component (Phase 2.4)
     */
    [[nodiscard]] const std::vector<std::string> &registered_params() const {
        return registered_params_;
    }

    /**
     * @brief Get config registered by current component (Phase 2.4)
     */
    [[nodiscard]] const std::vector<std::string> &registered_config() const {
        return registered_config_;
    }

    /**
     * @brief Clear tracking for next component
     */
    void clear_tracking() {
        registered_outputs_.clear();
        registered_inputs_.clear();
        resolved_inputs_.clear();
        registered_params_.clear();
        registered_config_.clear();
    }

    // =========================================================================
    // Direct Registry Access (for Simulator)
    // =========================================================================

    [[nodiscard]] SignalRegistry<Scalar> &registry() { return registry_; }
    [[nodiscard]] const SignalRegistry<Scalar> &registry() const { return registry_; }

  private:
    [[nodiscard]] std::string make_full_name(const std::string &local_name) const {
        std::string prefix = full_prefix();
        if (prefix.empty())
            return local_name;
        return prefix + "." + local_name;
    }

    SignalRegistry<Scalar> &registry_;
    std::string entity_;
    std::string component_;
    std::vector<std::string> registered_outputs_;
    std::vector<std::string> registered_inputs_; // Phase 2.4
    std::vector<std::string> resolved_inputs_;
    std::vector<std::string> registered_params_; // Phase 2.4
    std::vector<std::string> registered_config_; // Phase 2.4
};

} // namespace icarus
