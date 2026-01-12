#pragma once

/**
 * @file InputHandle.hpp
 * @brief Input signal port handle for components
 *
 * Part of Phase 2.4: Component Interface System.
 * InputHandles are registered at Provision and wired at Stage.
 */

#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <string>
#include <type_traits>

namespace icarus {

// Forward declarations
template <typename Scalar> class Backplane;
template <typename Scalar> class SignalRegistry;

/**
 * @brief Handle to an input signal port
 *
 * Registered at Provision, wired to a source at Stage.
 * Provides type-safe access to the wired signal value.
 *
 * Inputs have their own storage buffer that:
 * - Is initialized to zero (default)
 * - Can be poked for external injection (e.g., from Hermes)
 * - Is overridden when wired (wired source takes precedence)
 *
 * @tparam T The value type (Scalar, Vec3<Scalar>, etc.)
 */
template <typename T> class InputHandle {
  public:
    InputHandle() : default_value_{} {}

    /**
     * @brief Get the current value (from wired source or default buffer)
     *
     * Priority:
     * 1. If wired: returns source value (with gain applied)
     * 2. If unwired: returns default_value_ (can be set via set())
     *
     * @return Current value
     */
    [[nodiscard]] T get() const {
        if (!source_) {
            // Unwired: return the pokeable default value
            return default_value_;
        }
        // Wired: read from source (with gain for scalars)
        if constexpr (std::is_arithmetic_v<T>) {
            return static_cast<T>(*source_ * gain_);
        } else {
            return *source_;
        }
    }

    /**
     * @brief Set the default value (for external injection when unwired)
     *
     * This value is used when the input is not wired.
     * Allows external systems (Hermes, tests) to inject values.
     *
     * @param value The value to set
     */
    void set(const T &value) { default_value_ = value; }

    /**
     * @brief Dereference operator (same as get())
     */
    [[nodiscard]] T operator*() const { return get(); }

    /**
     * @brief Get pointer to the default value buffer (for registry)
     *
     * Used by SignalRegistry to enable poke via sim.set("signal", value)
     */
    [[nodiscard]] T *data_ptr() { return &default_value_; }

    /**
     * @brief Check if this input has been wired
     */
    [[nodiscard]] bool is_wired() const { return source_ != nullptr; }

    /**
     * @brief Get the port name (local name)
     */
    [[nodiscard]] const std::string &name() const { return name_; }

    /**
     * @brief Get the full qualified name (entity.component.signal)
     */
    [[nodiscard]] const std::string &full_name() const { return full_name_; }

    /**
     * @brief Get the wired source signal name (empty if unwired)
     */
    [[nodiscard]] const std::string &wired_to() const { return wired_to_; }

    /**
     * @brief Get units
     */
    [[nodiscard]] const std::string &units() const { return units_; }

    /**
     * @brief Get description
     */
    [[nodiscard]] const std::string &description() const { return description_; }

  private:
    // Backplane needs access to wire inputs
    template <typename S> friend class Backplane;
    template <typename S> friend class SignalRegistry;

    // Internal setters for Backplane/Registry
    void set_name(const std::string &name) { name_ = name; }
    void set_full_name(const std::string &full_name) { full_name_ = full_name; }
    void set_units(const std::string &units) { units_ = units; }
    void set_description(const std::string &desc) { description_ = desc; }

    void wire(const T *source, const std::string &source_name) {
        source_ = source;
        wired_to_ = source_name;
        gain_ = 1.0; // Default gain
    }

    /**
     * @brief Wire with a gain factor (Phase 4.0)
     *
     * The gain is applied when reading the value via get().
     */
    void wire_with_gain(const T *source, const std::string &source_name, double gain) {
        source_ = source;
        wired_to_ = source_name;
        gain_ = gain;
    }

    std::string name_;          // Local name (e.g., "throttle")
    std::string full_name_;     // Full name (e.g., "X15.Engine.throttle")
    std::string wired_to_;      // Source signal name
    const T *source_ = nullptr; // Pointer to source value
    double gain_ = 1.0;         // Scale factor (Phase 4.0)
    T default_value_{};         // Default/poke buffer (used when unwired)

    // Metadata
    std::string units_;
    std::string description_;
};

// =============================================================================
// Convenience Aliases
// =============================================================================

/**
 * @brief Scalar input handle
 */
template <typename Scalar> using ScalarInput = InputHandle<Scalar>;

/**
 * @brief Vec3 input handle
 */
template <typename Scalar> using Vec3Input = InputHandle<Vec3<Scalar>>;

/**
 * @brief Vec4/Quaternion input handle
 */
template <typename Scalar> using QuatInput = InputHandle<Vec4<Scalar>>;

} // namespace icarus
