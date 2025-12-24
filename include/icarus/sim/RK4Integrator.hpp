#pragma once

/**
 * @file RK4Integrator.hpp
 * @brief Fixed-step integrators: Euler, RK2, RK4
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <icarus/sim/Integrator.hpp>
#include <janus/math/IntegratorStep.hpp>

namespace icarus {

// =============================================================================
// Forward Euler Integrator (1st Order)
// =============================================================================

/**
 * @brief Forward Euler integrator (1st order)
 *
 * Simplest integrator, mainly for testing and comparison.
 * Wraps janus::euler_step() for dual-mode compatibility.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class EulerIntegrator : public Integrator<Scalar> {
  public:
    using typename Integrator<Scalar>::DerivativeFunc;

    JanusVector<Scalar> Step(const DerivativeFunc &f, const JanusVector<Scalar> &x, Scalar t,
                             Scalar dt) override {
        return janus::euler_step(f, x, t, dt);
    }

    [[nodiscard]] std::string Name() const override { return "Euler"; }
    [[nodiscard]] int Order() const override { return 1; }
    [[nodiscard]] IntegratorType Type() const override { return IntegratorType::Euler; }
};

// =============================================================================
// Heun's Method / RK2 Integrator (2nd Order)
// =============================================================================

/**
 * @brief Heun's method / RK2 integrator (2nd order)
 *
 * Two-stage method, useful for comparison studies.
 * Wraps janus::rk2_step() for dual-mode compatibility.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class RK2Integrator : public Integrator<Scalar> {
  public:
    using typename Integrator<Scalar>::DerivativeFunc;

    JanusVector<Scalar> Step(const DerivativeFunc &f, const JanusVector<Scalar> &x, Scalar t,
                             Scalar dt) override {
        return janus::rk2_step(f, x, t, dt);
    }

    [[nodiscard]] std::string Name() const override { return "RK2"; }
    [[nodiscard]] int Order() const override { return 2; }
    [[nodiscard]] IntegratorType Type() const override { return IntegratorType::RK2; }
};

// =============================================================================
// Classic RK4 Integrator (4th Order)
// =============================================================================

/**
 * @brief Classic 4th-order Runge-Kutta integrator
 *
 * Fixed-step method with 4 function evaluations per step.
 * Wraps janus::rk4_step() for dual-mode compatibility.
 *
 * **Butcher Tableau:**
 * ```
 * k₁ = f(t, x)
 * k₂ = f(t + dt/2, x + dt/2·k₁)
 * k₃ = f(t + dt/2, x + dt/2·k₂)
 * k₄ = f(t + dt, x + dt·k₃)
 * x_{n+1} = x + (dt/6)(k₁ + 2k₂ + 2k₃ + k₄)
 * ```
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class RK4Integrator : public Integrator<Scalar> {
  public:
    using typename Integrator<Scalar>::DerivativeFunc;

    JanusVector<Scalar> Step(const DerivativeFunc &f, const JanusVector<Scalar> &x, Scalar t,
                             Scalar dt) override {
        return janus::rk4_step(f, x, t, dt);
    }

    [[nodiscard]] std::string Name() const override { return "RK4"; }
    [[nodiscard]] int Order() const override { return 4; }
    [[nodiscard]] IntegratorType Type() const override { return IntegratorType::RK4; }
};

} // namespace icarus
