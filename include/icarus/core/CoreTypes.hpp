#pragma once

/**
 * @file CoreTypes.hpp
 * @brief Core type definitions, concepts, and configuration for Icarus
 *
 * Consolidates: Types.hpp, Concepts.hpp, Config.hpp
 *
 * Re-exports Metis types for dual-backend (numeric/symbolic) compatibility.
 * All Icarus components should use these types for Scalar, vectors, and matrices.
 */

#include <concepts>
#include <cstdint>
#include <string>

// Re-export Metis types and concepts
#include <metis/core/MetisConcepts.hpp>
#include <metis/core/MetisTypes.hpp>

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

// =============================================================================
// Metis Type Re-exports
// =============================================================================

// Matrix and vector types (templated on Scalar)
using metis::MetisMatrix;
using metis::MetisVector;

// Fixed-size types
using metis::Mat2;
using metis::Mat3;
using metis::Mat4;
using metis::Vec2;
using metis::Vec3;
using metis::Vec4;

// Concrete backend types
using metis::NumericMatrix;
using metis::NumericScalar;
using metis::NumericVector;
using metis::SymbolicMatrix;
using metis::SymbolicScalar;
using metis::SymbolicVector;

// Sparse types (numeric only)
using metis::SparseMatrix;
using metis::SparseTriplet;

// Symbolic variable creation
using metis::as_mx;
using metis::as_vector;
using metis::sym;
using metis::sym_vec;
using metis::sym_vector;
using metis::to_eigen;
using metis::to_mx;

// =============================================================================
// Scalar Concepts
// =============================================================================

/**
 * @brief Re-export MetisScalar concept
 *
 * Satisfied by floating point types (double, float) or CasADi symbolic (MX).
 * All Icarus components must be templated on types satisfying this concept.
 */
using metis::MetisScalar;

/**
 * @brief Alias for Icarus-specific documentation
 *
 * Identical to MetisScalar, provided for clarity in Icarus context.
 */
template <typename T>
concept IcarusScalar = MetisScalar<T>;

// =============================================================================
// Simulation Lifecycle
// =============================================================================

/**
 * @brief Simulation lifecycle phases
 */
enum class Lifecycle : uint8_t {
    Uninitialized, ///< Before Provision
    Provisioned,   ///< After Provision, before Stage
    Staged,        ///< After Stage, ready to run
    Running,       ///< During Step loop
    Paused,        ///< Temporarily halted
    Completed,     ///< Simulation finished
    Error          ///< Error state
};

// =============================================================================
// Configuration Structures
// =============================================================================

// ComponentConfig with typed accessors is now in its own header
// (Phase 4.0: Configuration Infrastructure)

/**
 * @brief Run configuration for Stage phase
 *
 * Passed to components during Stage to set up initial conditions.
 */
struct RunConfig {
    double t_start = 0.0;     ///< Simulation start time
    double t_end = 10.0;      ///< Simulation end time
    double dt = 0.001;        ///< Time step
    bool equilibrium = false; ///< If true, run trim solver at Stage
    // Additional fields for ICs, Monte Carlo, etc.
};

// =============================================================================
// Version Information
// =============================================================================

// Single source of truth for version numbers
#define ICARUS_VERSION_MAJOR 0
#define ICARUS_VERSION_MINOR 6
#define ICARUS_VERSION_PATCH 0

// Stringify helper
#define ICARUS_STRINGIFY(x) #x
#define ICARUS_VERSION_STR(major, minor, patch)                                                    \
    ICARUS_STRINGIFY(major) "." ICARUS_STRINGIFY(minor) "." ICARUS_STRINGIFY(patch)

/// Major version number
constexpr int VersionMajor() { return ICARUS_VERSION_MAJOR; }

/// Minor version number
constexpr int VersionMinor() { return ICARUS_VERSION_MINOR; }

/// Patch version number
constexpr int VersionPatch() { return ICARUS_VERSION_PATCH; }

/// Version string (derived from components)
constexpr const char *Version() {
    return ICARUS_VERSION_STR(ICARUS_VERSION_MAJOR, ICARUS_VERSION_MINOR, ICARUS_VERSION_PATCH);
}

// =============================================================================
// Naming Utilities
// =============================================================================

/**
 * @brief Build a full path from entity and name
 *
 * Returns "entity.name" if entity is non-empty, otherwise just "name".
 * Used consistently across Component, ComponentConfig, and logging.
 */
inline std::string MakeFullPath(const std::string &entity, const std::string &name) {
    if (entity.empty())
        return name;
    return entity + "." + name;
}

// =============================================================================
// Component Concepts
// =============================================================================

// Forward declarations
class ComponentConfig;
template <typename Scalar> class Backplane;

/**
 * @brief Concept for types that can serve as Icarus components
 *
 * A valid component must implement the core lifecycle methods:
 * - Provision(): Heavy setup, register signals
 * - Stage(): Wire inputs, apply ICs
 * - Step(): Hot path, compute derivatives
 */
template <typename T, typename Scalar>
concept ComponentType =
    requires(T &c, Backplane<Scalar> &bp, const ComponentConfig &cfg, Scalar t, Scalar dt) {
        // Component must have a name
        { c.Name() } -> std::convertible_to<std::string>;

        // Core lifecycle methods
        { c.Provision(bp, cfg) } -> std::same_as<void>;
        { c.Stage(bp, cfg) } -> std::same_as<void>;
        { c.Step(t, dt) } -> std::same_as<void>;
    };

/**
 * @brief Concept for components with optional extended hooks
 */
template <typename T, typename Scalar>
concept ExtendedComponent = ComponentType<T, Scalar> && requires(T &c, Lifecycle lifecycle) {
    { c.OnPhaseEnter(lifecycle) } -> std::same_as<void>;
    { c.OnPhaseExit(lifecycle) } -> std::same_as<void>;
};

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
#define ICARUS_ASSERT_LIFECYCLE(lifecycle, required, operation)                                    \
    ICARUS_ASSERT((lifecycle) >= (required), std::string(operation) +                              \
                                                 " requires lifecycle state >= " +                 \
                                                 std::to_string(static_cast<int>(required)))

#else // Release builds

// All debug macros compile to nothing
#define ICARUS_ASSERT(cond, msg) ((void)0)
#define ICARUS_DEBUG_ONLY(code) ((void)0)
#define ICARUS_ASSERT_PTR(ptr, context) ((void)0)
#define ICARUS_ASSERT_LIFECYCLE(lifecycle, required, operation) ((void)0)

#endif // ICARUS_DEBUG
