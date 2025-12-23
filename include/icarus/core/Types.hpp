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

/**
 * @brief Component configuration structure
 *
 * Passed to components during Provision phase.
 */
struct ComponentConfig {
    std::string name;        ///< Component instance name
    std::string entity;      ///< Entity namespace (optional)
    std::string config_path; ///< Path to config file (optional)
    // Additional config fields will be added as needed
};

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

/// Major version number
constexpr int VersionMajor() { return 0; }

/// Minor version number
constexpr int VersionMinor() { return 1; }

/// Patch version number
constexpr int VersionPatch() { return 0; }

/// Version string (for display)
constexpr const char *Version() { return "0.1.0"; }

/// Version string (alias)
constexpr const char *VersionString() { return "0.1.0"; }

} // namespace icarus
