#pragma once

/**
 * @file IntegrationManager.hpp
 * @brief Manages integrator lifecycle and stepping
 *
 * Part of Phase 4.0.7: Internal Managers
 *
 * IntegrationManager handles:
 * - Integrator creation from configuration
 * - Fixed-step and adaptive integration
 * - Integration configuration storage
 *
 * This class IS templated on Scalar for dual-backend support.
 */

#include <icarus/core/CoreTypes.hpp>
#include <icarus/sim/integrators/Integrator.hpp>
#include <icarus/sim/integrators/IntegratorFactory.hpp>
#include <icarus/sim/integrators/IntegratorTypes.hpp>

#include <functional>
#include <memory>
#include <stdexcept>

namespace icarus {

// =============================================================================
// IntegrationManager
// =============================================================================

/**
 * @brief Manages integrator lifecycle and stepping
 *
 * Centralizes integrator management that was previously spread
 * across Simulator methods.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class IntegrationManager {
  public:
    /// Derivative function signature: (t, X) -> X_dot
    using DerivativeFunc = std::function<JanusVector<Scalar>(Scalar, const JanusVector<Scalar> &)>;

    IntegrationManager() = default;

    /**
     * @brief Configure from IntegratorConfig
     *
     * Creates the appropriate integrator based on configuration.
     *
     * @param config Integrator configuration
     */
    void Configure(const IntegratorConfig<Scalar> &config) {
        config_ = config;
        integrator_ = IntegratorFactory<Scalar>::Create(config);
    }

    /**
     * @brief Configure from integrator type enum
     *
     * @param type Integrator type
     */
    void Configure(IntegratorType type) {
        config_ = IntegratorConfig<Scalar>::ForMethod(type);
        integrator_ = IntegratorFactory<Scalar>::Create(config_);
    }

    /**
     * @brief Configure from type name string
     *
     * @param type_name Integrator type name (e.g., "RK4", "RK45")
     */
    void Configure(const std::string &type_name) {
        IntegratorType type = parse_integrator_type(type_name);
        Configure(type);
    }

    /**
     * @brief Configure with default integrator (RK4)
     */
    void ConfigureDefault() {
        config_ = IntegratorConfig<Scalar>::RK4Default();
        integrator_ = IntegratorFactory<Scalar>::Create(config_);
    }

    /**
     * @brief Perform fixed-step integration
     *
     * @param deriv_func Derivative function (t, X) -> X_dot
     * @param X Current state vector
     * @param t Current time
     * @param dt Time step
     * @return New state at t + dt
     */
    [[nodiscard]] JanusVector<Scalar> Step(const DerivativeFunc &deriv_func,
                                           const JanusVector<Scalar> &X, Scalar t, Scalar dt) {
        if (!integrator_) {
            throw std::runtime_error(
                "IntegrationManager: integrator not configured. Call Configure() first.");
        }
        return integrator_->Step(deriv_func, X, t, dt);
    }

    /**
     * @brief Perform adaptive-step integration
     *
     * Only available for adaptive integrators (e.g., RK45).
     *
     * @param deriv_func Derivative function (t, X) -> X_dot
     * @param X Current state vector
     * @param t Current time
     * @param dt_request Requested time step
     * @return AdaptiveStepResult with actual step taken
     */
    [[nodiscard]] AdaptiveStepResult<Scalar> AdaptiveStep(const DerivativeFunc &deriv_func,
                                                          const JanusVector<Scalar> &X, Scalar t,
                                                          Scalar dt_request) {
        if (!integrator_) {
            throw std::runtime_error(
                "IntegrationManager: integrator not configured. Call Configure() first.");
        }
        if (!integrator_->IsAdaptive()) {
            throw std::runtime_error(
                "IntegrationManager: AdaptiveStep() requires an adaptive integrator. "
                "Current integrator: " +
                integrator_->Name());
        }

        auto *adaptive = dynamic_cast<AdaptiveIntegrator<Scalar> *>(integrator_.get());
        if (!adaptive) {
            throw std::runtime_error("IntegrationManager: failed to cast to AdaptiveIntegrator");
        }
        return adaptive->AdaptiveStep(deriv_func, X, t, dt_request);
    }

    /**
     * @brief Get current integrator type
     */
    [[nodiscard]] IntegratorType Type() const {
        if (integrator_) {
            return integrator_->Type();
        }
        return config_.type;
    }

    /**
     * @brief Get current configuration
     */
    [[nodiscard]] const IntegratorConfig<Scalar> &Config() const { return config_; }

    /**
     * @brief Get integrator name
     */
    [[nodiscard]] std::string Name() const {
        if (integrator_) {
            return integrator_->Name();
        }
        return to_string(config_.type);
    }

    /**
     * @brief Check if integrator is configured
     */
    [[nodiscard]] bool IsConfigured() const { return integrator_ != nullptr; }

    /**
     * @brief Check if current integrator supports adaptive stepping
     */
    [[nodiscard]] bool IsAdaptive() const { return integrator_ && integrator_->IsAdaptive(); }

    /**
     * @brief Get direct access to integrator (expert use)
     */
    [[nodiscard]] Integrator<Scalar> *GetIntegrator() { return integrator_.get(); }

    /**
     * @brief Get direct const access to integrator (expert use)
     */
    [[nodiscard]] const Integrator<Scalar> *GetIntegrator() const { return integrator_.get(); }

  private:
    std::unique_ptr<Integrator<Scalar>> integrator_;
    IntegratorConfig<Scalar> config_;
};

} // namespace icarus
