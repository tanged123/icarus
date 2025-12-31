#pragma once

/**
 * @file StagingTypes.hpp
 * @brief Core types for staging subsystem (trim, linearization, symbolic)
 *
 * Part of Phase 4: Staging Implementation.
 *
 * Note: LinearModel is in its own header due to size.
 * TrimResult and SymbolicDynamics are kept here as they're small and widely used.
 */

#include <icarus/staging/LinearModel.hpp>

#include <janus/core/Function.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace icarus::staging {

// =============================================================================
// TrimResult
// =============================================================================

/**
 * @brief Result of trim optimization
 */
struct TrimResult {
    bool converged = false;
    int iterations = 0;
    double residual_norm = 0.0;
    std::string message;

    /// Final control values (control_name -> value)
    std::unordered_map<std::string, double> controls;

    /// Final derivative residuals (derivative_name -> value)
    std::unordered_map<std::string, double> residuals;
};

// =============================================================================
// SymbolicDynamics
// =============================================================================

/**
 * @brief Symbolic dynamics representation
 *
 * Holds janus::Function objects for dynamics and Jacobians.
 * Produced by SymbolicStager during Stage().
 */
struct SymbolicDynamics {
    std::optional<janus::Function> dynamics;   ///< f(t, x) -> xdot
    std::optional<janus::Function> jacobian_x; ///< df/dx
    std::optional<janus::Function> jacobian_u; ///< df/du (if controls specified)

    std::vector<std::string> state_names;
    std::vector<std::string> control_names;
};

} // namespace icarus::staging
