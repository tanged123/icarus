#pragma once

/**
 * @file RK45Integrator.hpp
 * @brief Adaptive Dormand-Prince RK45 integrator
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <icarus/core/Error.hpp>
#include <icarus/sim/integrators/Integrator.hpp>
#include <janus/math/Arithmetic.hpp>
#include <janus/math/IntegratorStep.hpp>
#include <janus/math/Logic.hpp>
#include <type_traits>

namespace icarus {

/**
 * @brief Adaptive Dormand-Prince RK4(5) integrator
 *
 * Uses embedded 4th and 5th order solutions to estimate local
 * truncation error. Adjusts step size to maintain error within tolerance.
 *
 * **Features:**
 * - 7 function evaluations per step (shares stages)
 * - Automatic step size control
 * - Configurable absolute and relative tolerances
 * - Step statistics tracking
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class RK45Integrator : public AdaptiveIntegrator<Scalar> {
  public:
    using typename Integrator<Scalar>::DerivativeFunc;

    /**
     * @brief Construct with default tolerances
     *
     * @param abs_tol Absolute tolerance (default 1e-6)
     * @param rel_tol Relative tolerance (default 1e-6)
     */
    explicit RK45Integrator(Scalar abs_tol = Scalar{1e-6}, Scalar rel_tol = Scalar{1e-6})
        : abs_tol_(abs_tol), rel_tol_(rel_tol) {}

    /**
     * @brief Fixed-step interface (uses requested dt directly)
     *
     * For compatibility with base Integrator interface.
     * No step adaptation; returns 5th-order solution.
     */
    JanusVector<Scalar> Step(const DerivativeFunc &f, const JanusVector<Scalar> &x, Scalar t,
                             Scalar dt) override {
        auto result = janus::rk45_step(f, x, t, dt);
        return result.y5;
    }

    /**
     * @brief Adaptive step with automatic error control
     *
     * Performs RK45 steps with automatic step size reduction when error
     * exceeds tolerance. Retries with progressively smaller dt until
     * the step is accepted or dt falls below min_dt.
     *
     * @param f Derivative function
     * @param x Current state
     * @param t Current time
     * @param dt Requested step size (may be reduced internally)
     * @return Result with final state, actual dt used, error, and acceptance flag
     * @throws StepSizeTooSmallError if dt reduces below min_dt without success
     */
    AdaptiveStepResult<Scalar> AdaptiveStep(const DerivativeFunc &f, const JanusVector<Scalar> &x,
                                            Scalar t, Scalar dt) override {
        Scalar current_dt = dt;

        // Retry loop with step size reduction
        constexpr int max_attempts = 20; // Prevent infinite loops
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            // Compute RK45 step
            auto result = janus::rk45_step(f, x, t, current_dt);

            // Compute error tolerance
            Scalar tol = ComputeTolerance(x, result.y5);

            // Check if step is accepted
            bool accepted = LessThanOrEqual(result.error, tol);

            if (accepted) {
                stats_.accepted_steps++;
                return AdaptiveStepResult<Scalar>{result.y5, current_dt, result.error, true};
            }

            // Step rejected - reduce dt and retry
            stats_.rejected_steps++;

            // Compute new step size
            Scalar dt_new = SuggestDt(current_dt, result.error, tol);

            // Check for minimum step size (only in numeric mode)
            if constexpr (std::is_same_v<Scalar, double>) {
                if (dt_new <= min_dt_) {
                    throw StepSizeTooSmallError(static_cast<double>(min_dt_));
                }
            }

            current_dt = dt_new;
        }

        // Max attempts reached - return last result as rejected
        auto result = janus::rk45_step(f, x, t, current_dt);
        return AdaptiveStepResult<Scalar>{result.y5, current_dt, result.error, false};
    }

    // Tolerance setters/getters
    void SetAbsTol(Scalar tol) override { abs_tol_ = tol; }
    void SetRelTol(Scalar tol) override { rel_tol_ = tol; }
    [[nodiscard]] Scalar GetAbsTol() const override { return abs_tol_; }
    [[nodiscard]] Scalar GetRelTol() const override { return rel_tol_; }

    [[nodiscard]] std::string Name() const override { return "RK45"; }
    [[nodiscard]] int Order() const override { return 5; }
    [[nodiscard]] IntegratorType Type() const override { return IntegratorType::RK45; }

    // Step size control parameters
    void SetMinDt(Scalar min_dt) { min_dt_ = min_dt; }
    void SetMaxDt(Scalar max_dt) { max_dt_ = max_dt; }
    void SetSafetyFactor(Scalar factor) { safety_ = factor; }

    [[nodiscard]] Scalar GetMinDt() const { return min_dt_; }
    [[nodiscard]] Scalar GetMaxDt() const { return max_dt_; }

    /**
     * @brief Suggest next step size based on error
     *
     * @param dt Current step size
     * @param error Current error estimate
     * @param tol Error tolerance
     * @return Suggested step size for next step
     */
    [[nodiscard]] Scalar SuggestDt(Scalar dt, Scalar error, Scalar tol) const {
        // Optimal step size formula: dt_new = dt * safety * (tol/error)^(1/5)
        // Use 5th root for RK45 (5th order method)
        Scalar ratio = tol / (error + Scalar{1e-15}); // Avoid division by zero
        Scalar factor = safety_ * janus::pow(ratio, Scalar{0.2});

        // Limit growth/shrinkage
        factor = janus::max(Scalar{0.1}, janus::min(Scalar{5.0}, factor));

        Scalar dt_new = dt * factor;

        // Clamp to min/max
        return janus::max(min_dt_, janus::min(max_dt_, dt_new));
    }

    /**
     * @brief Statistics for adaptive stepping
     */
    struct Statistics {
        std::size_t accepted_steps = 0;
        std::size_t rejected_steps = 0;

        void Reset() {
            accepted_steps = 0;
            rejected_steps = 0;
        }

        [[nodiscard]] double AcceptanceRate() const {
            auto total = accepted_steps + rejected_steps;
            return total > 0 ? static_cast<double>(accepted_steps) / static_cast<double>(total)
                             : 1.0;
        }
    };

    [[nodiscard]] const Statistics &GetStatistics() const { return stats_; }
    void ResetStatistics() { stats_.Reset(); }

  private:
    Scalar abs_tol_;
    Scalar rel_tol_;
    Scalar min_dt_{1e-10};
    Scalar max_dt_{1.0};
    Scalar safety_{0.9};
    Statistics stats_;

    /**
     * @brief Compute error tolerance for given states
     *
     * tolerance = abs_tol + rel_tol * max(|x|, |x_new|)
     */
    [[nodiscard]] Scalar ComputeTolerance(const JanusVector<Scalar> &x,
                                          const JanusVector<Scalar> &x_new) const {
        Scalar max_norm = Scalar{0};
        for (Eigen::Index i = 0; i < x.size(); ++i) {
            Scalar xi_abs = janus::abs(x[i]);
            Scalar xi_new_abs = janus::abs(x_new[i]);
            max_norm = janus::max(max_norm, janus::max(xi_abs, xi_new_abs));
        }
        return abs_tol_ + rel_tol_ * max_norm;
    }

    /**
     * @brief Compare Scalar values (symbolic-safe)
     */
    [[nodiscard]] bool LessThanOrEqual(Scalar a, Scalar b) const {
        // For numeric mode, direct comparison
        // For symbolic mode, this is only used in statistics (not traced)
        if constexpr (std::is_same_v<Scalar, double>) {
            return a <= b;
        } else {
            // For symbolic, evaluate numerically if possible
            // This branch only used in adaptive control, not traced
            return true; // Accept in symbolic mode
        }
    }
};

} // namespace icarus
