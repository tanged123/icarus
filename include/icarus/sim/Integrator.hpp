#pragma once

#include <functional>
#include <vector>

namespace icarus {

/**
 * @brief Abstract interface for numerical integrators.
 *
 * Integrators advance the state vector by one time step.
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class Integrator {
  public:
    virtual ~Integrator() = default;

    /**
     * @brief Derivative function type.
     *
     * Takes time and state vector, returns derivative vector.
     */
    using DerivativeFunc =
        std::function<std::vector<Scalar>(Scalar t, const std::vector<Scalar> &x)>;

    /**
     * @brief Advance state by one time step.
     *
     * @param t Current time
     * @param x Current state vector
     * @param dt Time step
     * @param f Derivative function
     * @return New state vector
     */
    virtual std::vector<Scalar> Step(Scalar t, const std::vector<Scalar> &x, Scalar dt,
                                     const DerivativeFunc &f) = 0;
};

} // namespace icarus
