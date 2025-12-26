#pragma once

/**
 * @file SimulationResults.hpp
 * @brief Results from a simulation run
 *
 * Part of Phase 3: Symbolic Mode.
 * Captures simulation outcomes for analysis.
 */

#include <icarus/core/Types.hpp>

#include <chrono>
#include <map>
#include <string>

namespace icarus {

/**
 * @brief Results from a simulation run
 *
 * Used by SimulationRunner to capture simulation outcomes.
 */
template <typename Scalar> struct SimulationResults {
    // =========================================================================
    // Timing
    // =========================================================================

    Scalar sim_time_final{};        ///< Final simulation time
    double wall_time_seconds = 0.0; ///< Wall clock duration
    double realtime_factor = 0.0;   ///< sim_time / wall_time

    // =========================================================================
    // State
    // =========================================================================

    std::map<std::string, Scalar> final_signals; ///< Signal values at end

    // =========================================================================
    // Statistics
    // =========================================================================

    std::size_t total_steps = 0;            ///< Number of Step() calls
    std::size_t integrator_evaluations = 0; ///< Derivative evaluations

    // =========================================================================
    // Status
    // =========================================================================

    bool completed = false;         ///< True if ran to t_end
    std::string termination_reason; ///< "completed", "error", "stopped"
};

} // namespace icarus
