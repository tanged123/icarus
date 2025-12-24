#pragma once

/**
 * @file IntegratorTypes.hpp
 * @brief Integrator type enumeration and configuration
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <algorithm>
#include <cctype>
#include <icarus/core/Types.hpp>
#include <stdexcept>
#include <string>

namespace icarus {

// =============================================================================
// Integrator Type Enumeration
// =============================================================================

/**
 * @brief Available integrator methods
 *
 * Maps directly to Janus step functions.
 */
enum class IntegratorType {
    Euler, ///< Forward Euler (1st order, 1 eval) - janus::euler_step
    RK2,   ///< Heun's method (2nd order, 2 evals) - janus::rk2_step
    RK4,   ///< Classic RK4 (4th order, 4 evals) - janus::rk4_step
    RK45   ///< Dormand-Prince adaptive (5th order, 7 evals) - janus::rk45_step
};

/**
 * @brief Convert integrator type to string
 */
[[nodiscard]] inline std::string to_string(IntegratorType type) {
    switch (type) {
    case IntegratorType::Euler:
        return "Euler";
    case IntegratorType::RK2:
        return "RK2";
    case IntegratorType::RK4:
        return "RK4";
    case IntegratorType::RK45:
        return "RK45";
    }
    return "Unknown";
}

/**
 * @brief Parse integrator type from string (case-insensitive)
 *
 * @throws std::invalid_argument if type name is not recognized
 */
[[nodiscard]] inline IntegratorType parse_integrator_type(const std::string &name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (lower == "euler")
        return IntegratorType::Euler;
    if (lower == "rk2")
        return IntegratorType::RK2;
    if (lower == "rk4")
        return IntegratorType::RK4;
    if (lower == "rk45")
        return IntegratorType::RK45;
    throw std::invalid_argument("Unknown integrator type: " + name);
}

// =============================================================================
// Integrator Configuration
// =============================================================================

/**
 * @brief Configuration for integrator creation
 *
 * Supports both fixed-step and adaptive integrators.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> struct IntegratorConfig {
    IntegratorType type = IntegratorType::RK4; ///< Method to use

    // Adaptive stepping parameters (RK45 only)
    Scalar abs_tol = Scalar{1e-6};      ///< Absolute tolerance
    Scalar rel_tol = Scalar{1e-6};      ///< Relative tolerance
    Scalar min_dt = Scalar{1e-10};      ///< Minimum step size
    Scalar max_dt = Scalar{1.0};        ///< Maximum step size
    Scalar safety_factor = Scalar{0.9}; ///< Step size safety factor

    /**
     * @brief Create default RK4 config
     */
    static IntegratorConfig RK4Default() {
        IntegratorConfig cfg;
        cfg.type = IntegratorType::RK4;
        return cfg;
    }

    /**
     * @brief Create adaptive RK45 config with tolerances
     */
    static IntegratorConfig RK45Adaptive(Scalar abs_tol_val = Scalar{1e-6},
                                         Scalar rel_tol_val = Scalar{1e-6}) {
        IntegratorConfig cfg;
        cfg.type = IntegratorType::RK45;
        cfg.abs_tol = abs_tol_val;
        cfg.rel_tol = rel_tol_val;
        return cfg;
    }

    /**
     * @brief Create config for specified method
     */
    static IntegratorConfig ForMethod(IntegratorType method) {
        IntegratorConfig cfg;
        cfg.type = method;
        return cfg;
    }
};

} // namespace icarus
