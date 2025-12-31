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
 * @tparam T The value type (Scalar, Vec3<Scalar>, etc.)
 */
template <typename T> class InputHandle {
  public:
    InputHandle() = default;

    /**
     * @brief Get the current value from the wired source (with gain applied)
     *
     * If a gain was set via wire_with_gain(), the source value is scaled.
     * For Scalar types: returns source * gain
     * For vector types: gain is not currently applied (use gain=1.0)
     *
     * @throws UnwiredInputError if not wired
     * @return Value by value (scaled if gain != 1.0)
     */
    [[nodiscard]] T get() const {
        if (!source_) {
            throw UnwiredInputError(name_);
        }
        // Apply gain for scalar types; for vectors, gain should be 1.0
        // (wire_input_with_gain only supports Scalar)
        if constexpr (std::is_arithmetic_v<T>) {
            return static_cast<T>(*source_ * gain_);
        } else {
            // For vector types, return as-is (gain should be 1.0)
            return *source_;
        }
    }

    /**
     * @brief Dereference operator for convenience (without gain)
     *
     * Note: For direct source access without gain, use operator*.
     * For scaled value, use get().
     *
     * @throws UnwiredInputError if not wired
     */
    [[nodiscard]] const T &operator*() const {
        if (!source_) {
            throw UnwiredInputError(name_);
        }
        return *source_;
    }

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
