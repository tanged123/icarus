#pragma once

/**
 * @file InputHandle.hpp
 * @brief Input signal port handle for components
 *
 * Part of Phase 2.4: Component Interface System.
 * InputHandles are registered at Provision and wired at Stage.
 */

#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>
#include <string>

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
     * @brief Get the current value from the wired source
     * @throws UnwiredInputError if not wired
     */
    [[nodiscard]] const T &get() const {
        if (!source_) {
            throw UnwiredInputError(name_);
        }
        return *source_;
    }

    /**
     * @brief Dereference operator for convenience
     * @throws UnwiredInputError if not wired
     */
    [[nodiscard]] const T &operator*() const { return get(); }

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
    }

    std::string name_;          // Local name (e.g., "throttle")
    std::string full_name_;     // Full name (e.g., "X15.Engine.throttle")
    std::string wired_to_;      // Source signal name
    const T *source_ = nullptr; // Pointer to source value

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
