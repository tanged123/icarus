#pragma once

/**
 * @file Integrator.hpp
 * @brief Abstract interface for numerical integrators
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <functional>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/sim/integrators/IntegratorTypes.hpp>
#include <string>

namespace icarus {

// =============================================================================
// Adaptive Step Result
// =============================================================================

/**
 * @brief Result from adaptive step integrators
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> struct AdaptiveStepResult {
    JanusVector<Scalar> state; ///< New state at t + dt_actual
    Scalar dt_actual;          ///< Actual step taken (may differ from requested)
    Scalar error_estimate;     ///< Local truncation error estimate
    bool accepted;             ///< Whether step was accepted
};

// =============================================================================
// Abstract Integrator Interface
// =============================================================================

/**
 * @brief Abstract interface for numerical integrators
 *
 * Integrators advance state using derivative information from the Simulator.
 * All implementations are templated on Scalar for symbolic mode support.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class Integrator {
  public:
    virtual ~Integrator() = default;

    /**
     * @brief Derivative function signature
     *
     * Maps (t, X) → dX/dt
     */
    using DerivativeFunc =
        std::function<JanusVector<Scalar>(Scalar t, const JanusVector<Scalar> &x)>;

    /**
     * @brief Advance state by one step
     *
     * @param f Derivative function
     * @param x Current state vector
     * @param t Current time
     * @param dt Requested time step
     * @return New state at t + dt
     */
    virtual JanusVector<Scalar> Step(const DerivativeFunc &f, const JanusVector<Scalar> &x,
                                     Scalar t, Scalar dt) = 0;

    /**
     * @brief Get integrator name for logging
     */
    [[nodiscard]] virtual std::string Name() const = 0;

    /**
     * @brief Get integrator order (for error analysis)
     */
    [[nodiscard]] virtual int Order() const = 0;

    /**
     * @brief Get integrator type
     */
    [[nodiscard]] virtual IntegratorType Type() const = 0;

    /**
     * @brief Check if integrator supports adaptive stepping
     */
    [[nodiscard]] virtual bool IsAdaptive() const { return false; }
};

// =============================================================================
// Adaptive Integrator Interface
// =============================================================================

/**
 * @brief Interface for adaptive step integrators
 *
 * Extends base Integrator with error-controlled stepping.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class AdaptiveIntegrator : public Integrator<Scalar> {
  public:
    using typename Integrator<Scalar>::DerivativeFunc;

    /**
     * @brief Advance with error control
     *
     * May take a smaller step than requested to meet tolerance.
     *
     * @param f Derivative function
     * @param x Current state vector
     * @param t Current time
     * @param dt Requested time step
     * @return Result with actual step, error, and acceptance status
     */
    virtual AdaptiveStepResult<Scalar>
    AdaptiveStep(const DerivativeFunc &f, const JanusVector<Scalar> &x, Scalar t, Scalar dt) = 0;

    [[nodiscard]] bool IsAdaptive() const override { return true; }

    /**
     * @brief Set absolute tolerance
     */
    virtual void SetAbsTol(Scalar tol) = 0;

    /**
     * @brief Set relative tolerance
     */
    virtual void SetRelTol(Scalar tol) = 0;

    /**
     * @brief Get current absolute tolerance
     */
    [[nodiscard]] virtual Scalar GetAbsTol() const = 0;

    /**
     * @brief Get current relative tolerance
     */
    [[nodiscard]] virtual Scalar GetRelTol() const = 0;
};

} // namespace icarus
