#pragma once

/**
 * @file IntegratorFactory.hpp
 * @brief Factory for creating integrators from configuration
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <icarus/sim/Integrator.hpp>
#include <icarus/sim/IntegratorTypes.hpp>
#include <icarus/sim/RK45Integrator.hpp>
#include <icarus/sim/RK4Integrator.hpp>
#include <memory>
#include <stdexcept>

namespace icarus {

/**
 * @brief Factory for creating integrators from configuration
 *
 * Enables runtime selection of integration method without
 * hardcoding specific integrator types.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class IntegratorFactory {
  public:
    /**
     * @brief Create integrator from configuration
     *
     * @param config Integrator configuration
     * @return Unique pointer to configured integrator
     */
    static std::unique_ptr<Integrator<Scalar>> Create(const IntegratorConfig<Scalar> &config) {
        switch (config.type) {
        case IntegratorType::Euler:
            return std::make_unique<EulerIntegrator<Scalar>>();

        case IntegratorType::RK2:
            return std::make_unique<RK2Integrator<Scalar>>();

        case IntegratorType::RK4:
            return std::make_unique<RK4Integrator<Scalar>>();

        case IntegratorType::RK45: {
            auto rk45 = std::make_unique<RK45Integrator<Scalar>>(config.abs_tol, config.rel_tol);
            rk45->SetMinDt(config.min_dt);
            rk45->SetMaxDt(config.max_dt);
            rk45->SetSafetyFactor(config.safety_factor);
            return rk45;
        }
        }
        throw std::invalid_argument("Unknown integrator type");
    }

    /**
     * @brief Create integrator from type name string
     *
     * Convenience for configuration file parsing.
     */
    static std::unique_ptr<Integrator<Scalar>> Create(const std::string &type_name) {
        return Create(IntegratorConfig<Scalar>::ForMethod(parse_integrator_type(type_name)));
    }

    /**
     * @brief Create default integrator (RK4)
     */
    static std::unique_ptr<Integrator<Scalar>> CreateDefault() {
        return Create(IntegratorConfig<Scalar>::RK4Default());
    }
};

} // namespace icarus
