#pragma once

/**
 * @file Config.hpp
 * @brief Build configuration and debug macros for Icarus
 *
 * Provides compile-time configuration for debug vs release builds:
 * - ICARUS_DEBUG: Defined in Debug builds via CMake
 * - ICARUS_ASSERT: Runtime assertions (throws in debug, no-op in release)
 * - ICARUS_DEBUG_ONLY: Code that only executes in debug builds
 */

namespace icarus {

// =============================================================================
// Build Mode Detection
// =============================================================================

/// Check if we're in debug mode at compile time
#ifdef ICARUS_DEBUG
constexpr bool kDebugMode = true;
#else
constexpr bool kDebugMode = false;
#endif

} // namespace icarus

// =============================================================================
// Debug Assertion Macros
// =============================================================================

#ifdef ICARUS_DEBUG

/**
 * @brief Assert a condition in debug builds, throw if false
 * @param cond Condition to check
 * @param msg Error message if condition fails
 *
 * In release builds, this macro compiles to nothing.
 */
#define ICARUS_ASSERT(cond, msg)                                                                   \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            throw std::runtime_error(std::string("ICARUS_ASSERT failed: ") + (msg));               \
        }                                                                                          \
    } while (0)

/**
 * @brief Execute code only in debug builds
 * @param code Code block to execute
 */
#define ICARUS_DEBUG_ONLY(code) code

/**
 * @brief Assert a pointer is non-null in debug builds
 * @param ptr Pointer to check
 * @param context Description of where the check is happening
 */
#define ICARUS_ASSERT_PTR(ptr, context) ICARUS_ASSERT((ptr) != nullptr, "Null pointer in " context)

/**
 * @brief Assert a component lifecycle requirement in debug builds
 * @param phase Current phase
 * @param required Required phase for this operation
 * @param operation Name of the operation being attempted
 */
#define ICARUS_ASSERT_LIFECYCLE(phase, required, operation)                                        \
    ICARUS_ASSERT((phase) >= (required), std::string(operation) +                                  \
                                             " requires lifecycle phase >= " +                     \
                                             std::to_string(static_cast<int>(required)))

#else // Release builds

// All debug macros compile to nothing
#define ICARUS_ASSERT(cond, msg) ((void)0)
#define ICARUS_DEBUG_ONLY(code) ((void)0)
#define ICARUS_ASSERT_PTR(ptr, context) ((void)0)
#define ICARUS_ASSERT_LIFECYCLE(phase, required, operation) ((void)0)

#endif // ICARUS_DEBUG
