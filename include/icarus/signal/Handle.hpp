#pragma once

/**
 * @file Handle.hpp
 * @brief Type-safe SignalHandle for zero-overhead hot path access
 *
 * Part of Phase 1.3: Signal Backplane.
 * Provides the "bind once, read forever" pattern for signal access.
 */

#include <icarus/signal/Signal.hpp>

#include <icarus/core/Config.hpp>

#include <cassert>
#include <string>

namespace icarus {

/**
 * @brief Type-safe handle for accessing a signal value
 *
 * SignalHandle provides:
 * - Zero-overhead dereferencing (hot path): operator*, operator->
 * - Raw pointer access for integration with Eigen loops
 * - Metadata access (cold path): name(), unit(), lifecycle()
 * - Validity checking: operator bool()
 *
 * Usage pattern:
 * @code
 *   // During Stage phase: bind once
 *   SignalHandle<double> thrust = registry.resolve<double>("thrust");
 *
 *   // During Step phase: read forever (zero overhead)
 *   double current_thrust = *thrust;
 *   thrust->some_method();  // if T is a class
 * @endcode
 *
 * @tparam T The signal value type (must have a TypeTraits specialization)
 */
template <typename T> class SignalHandle {
  public:
    /**
     * @brief Construct a SignalHandle with pointer and metadata
     * @param ptr Pointer to the signal storage
     * @param meta Pointer to the signal descriptor (metadata)
     */
    SignalHandle(T *ptr, const SignalDescriptor *meta) : ptr_(ptr), meta_(meta) {}

    /**
     * @brief Default constructor (creates invalid handle)
     */
    SignalHandle() : ptr_(nullptr), meta_(nullptr) {}

    // =========================================================================
    // Zero-overhead access (hot path)
    // =========================================================================

    /**
     * @brief Dereference to get/set the signal value
     */
    T &operator*() {
        ICARUS_ASSERT_PTR(ptr_, "SignalHandle::operator*");
        return *ptr_;
    }
    const T &operator*() const {
        ICARUS_ASSERT_PTR(ptr_, "SignalHandle::operator* const");
        return *ptr_;
    }

    /**
     * @brief Arrow operator for member access (if T is a class)
     */
    T *operator->() {
        ICARUS_ASSERT_PTR(ptr_, "SignalHandle::operator->");
        return ptr_;
    }
    const T *operator->() const {
        ICARUS_ASSERT_PTR(ptr_, "SignalHandle::operator-> const");
        return ptr_;
    }

    /**
     * @brief Get raw pointer for integration with Eigen loops
     */
    T *ptr() { return ptr_; }
    const T *ptr() const { return ptr_; }

    // =========================================================================
    // Introspection (cold path)
    // =========================================================================

    /**
     * @brief Get the signal name
     * @pre Handle must be valid (check with valid() or operator bool())
     */
    [[nodiscard]] const std::string &name() const {
        assert(meta_ != nullptr && "Cannot access name() on invalid handle");
        return meta_->name;
    }

    /**
     * @brief Get the signal unit
     * @pre Handle must be valid
     */
    [[nodiscard]] const std::string &unit() const {
        assert(meta_ != nullptr && "Cannot access unit() on invalid handle");
        return meta_->unit;
    }

    /**
     * @brief Get the signal lifecycle
     * @pre Handle must be valid
     */
    [[nodiscard]] SignalLifecycle lifecycle() const {
        assert(meta_ != nullptr && "Cannot access lifecycle() on invalid handle");
        return meta_->lifecycle;
    }

    /**
     * @brief Get the full signal descriptor
     * @pre Handle must be valid
     */
    [[nodiscard]] const SignalDescriptor *descriptor() const {
        assert(meta_ != nullptr && "Cannot access descriptor() on invalid handle");
        return meta_;
    }

    // =========================================================================
    // Validity check
    // =========================================================================

    /**
     * @brief Check if handle is valid (bound to a signal)
     */
    explicit operator bool() const { return ptr_ != nullptr; }

    /**
     * @brief Check if handle is valid
     */
    [[nodiscard]] bool valid() const { return ptr_ != nullptr; }

  private:
    T *ptr_;                       ///< Pointer to signal storage
    const SignalDescriptor *meta_; ///< Pointer to signal metadata
};

} // namespace icarus
