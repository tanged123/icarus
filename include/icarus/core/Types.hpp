#pragma once

/**
 * @file Types.hpp
 * @brief Core type definitions for Icarus simulation framework
 *
 * Re-exports Janus types for dual-backend (numeric/symbolic) compatibility.
 * All Icarus components should use these types for Scalar, vectors, and matrices.
 */

#include <cstdint>
#include <string>

// Re-export Janus types for dual-backend support
#include <janus/core/JanusTypes.hpp>

namespace icarus {

// =============================================================================
// Janus Type Re-exports
// =============================================================================

// Matrix and vector types (templated on Scalar)
using janus::JanusMatrix;
using janus::JanusVector;

// Fixed-size types
using janus::Mat2;
using janus::Mat3;
using janus::Mat4;
using janus::Vec2;
using janus::Vec3;
using janus::Vec4;

// Concrete backend types
using janus::NumericMatrix;
using janus::NumericScalar;
using janus::NumericVector;
using janus::SymbolicMatrix;
using janus::SymbolicScalar;
using janus::SymbolicVector;

// Sparse types (numeric only)
using janus::SparseMatrix;
using janus::SparseTriplet;

// Symbolic variable creation
using janus::as_mx;
using janus::as_vector;
using janus::sym;
using janus::sym_vec;
using janus::sym_vector;
using janus::to_eigen;
using janus::to_mx;

// =============================================================================
// Simulation Lifecycle
// =============================================================================

/**
 * @brief Simulation lifecycle phases
 */
enum class Phase : uint8_t {
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
#define ICARUS_VERSION_MINOR 3
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

} // namespace icarus
