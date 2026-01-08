#pragma once

/**
 * @file RocketEngine.hpp
 * @brief Rocket engine thrust model component
 *
 * Computes thrust force from throttle command using Vulcan propulsion utilities.
 * Outputs force vector compatible with ForceAggregator.
 *
 * Physics:
 *   Ve = Isp * g0           (exhaust velocity)
 *   F = throttle * F_max    (commanded thrust)
 *   mdot = F / Ve           (mass flow rate)
 *   force = F * direction   (force vector in body frame)
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>

#include <vulcan/core/Constants.hpp>
#include <vulcan/propulsion/Rocket.hpp>

namespace icarus {
namespace components {

/**
 * @brief Rocket engine component with throttle control
 *
 * Models a liquid rocket engine with:
 * - Throttle command input [0, 1]
 * - Configurable max thrust and specific impulse
 * - Outputs force vector for ForceAggregator
 * - Outputs mass flow rate for propellant depletion
 *
 * Configuration:
 * - max_thrust: Maximum vacuum thrust [N]
 * - isp_vacuum: Vacuum specific impulse [s]
 * - thrust_direction: Unit vector in body frame
 * - nozzle_position: Application point in body frame [m]
 *
 * Inputs:
 * - throttle_cmd: Throttle command [0, 1]
 *
 * Outputs:
 * - thrust: Thrust magnitude [N]
 * - mass_flow_rate: Propellant consumption [kg/s]
 * - force.x, .y, .z: Force vector [N]
 * - application_point.x, .y, .z: Nozzle position [m]
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class RocketEngine : public Component<Scalar> {
  public:
    explicit RocketEngine(std::string name = "Engine", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "RocketEngine"; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    void Provision(Backplane<Scalar> &bp) override {
        // === Outputs ===
        // Thrust magnitude for telemetry
        bp.template register_output<Scalar>("thrust", &thrust_, "N", "Thrust magnitude");

        // Mass flow rate for fuel depletion
        bp.template register_output<Scalar>("mass_flow_rate", &mass_flow_rate_, "kg/s",
                                            "Propellant mass flow rate");

        // Force vector (ForceAggregator compatible)
        bp.template register_output_vec3<Scalar>("force", &force_, "N",
                                                 "Thrust force in body frame");

        // Application point (ForceAggregator compatible)
        bp.template register_output_vec3<Scalar>("application_point", &nozzle_position_, "m",
                                                 "Nozzle position in body frame");

        // === Inputs ===
        // Throttle command
        bp.template register_input<Scalar>("throttle_cmd", &throttle_cmd_, "-",
                                           "Throttle command [0, 1]");
    }

    void Stage(Backplane<Scalar> &) override {
        const auto &config = this->GetConfig();

        // Read engine parameters
        if (config.template Has<double>("max_thrust")) {
            max_thrust_ = config.template Get<double>("max_thrust", 50000.0);
        }
        if (config.template Has<double>("isp_vacuum")) {
            isp_vacuum_ = config.template Get<double>("isp_vacuum", 300.0);
        }

        // Read thrust direction
        if (config.template Has<Vec3<double>>("thrust_direction")) {
            auto dir = config.template Get<Vec3<double>>("thrust_direction", Vec3<double>{0, 0, 1});
            // Normalize direction
            double norm = dir.norm();
            if (norm > 1e-10) {
                dir /= norm;
            }
            thrust_direction_ =
                Vec3<Scalar>{static_cast<Scalar>(dir(0)), static_cast<Scalar>(dir(1)),
                             static_cast<Scalar>(dir(2))};
        }

        // Read nozzle position
        if (config.template Has<Vec3<double>>("nozzle_position")) {
            auto pos = config.template Get<Vec3<double>>("nozzle_position", Vec3<double>::Zero());
            nozzle_position_ =
                Vec3<Scalar>{static_cast<Scalar>(pos(0)), static_cast<Scalar>(pos(1)),
                             static_cast<Scalar>(pos(2))};
        }

        // Pre-compute exhaust velocity
        exhaust_velocity_ = vulcan::propulsion::rocket::exhaust_velocity(
            Scalar(isp_vacuum_), vulcan::constants::physics::g0);
    }

    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Get throttle command, clamp to [0, 1]
        Scalar throttle = throttle_cmd_.get();
        throttle = janus::clamp(throttle, Scalar(0), Scalar(1));

        // Compute thrust magnitude
        thrust_ = throttle * Scalar(max_thrust_);

        // Compute mass flow rate: mdot = F / Ve
        mass_flow_rate_ = vulcan::propulsion::rocket::mass_flow_rate(thrust_, exhaust_velocity_);

        // Compute force vector in body frame
        force_ = thrust_direction_ * thrust_;
    }

    // =========================================================================
    // Configuration (programmatic)
    // =========================================================================

    void SetMaxThrust(double F) { max_thrust_ = F; }
    void SetIspVacuum(double isp) { isp_vacuum_ = isp; }
    void SetThrustDirection(const Vec3<Scalar> &dir) { thrust_direction_ = dir.normalized(); }
    void SetNozzlePosition(const Vec3<Scalar> &pos) { nozzle_position_ = pos; }

    // Accessors
    [[nodiscard]] Scalar GetThrust() const { return thrust_; }
    [[nodiscard]] Scalar GetMassFlowRate() const { return mass_flow_rate_; }
    [[nodiscard]] Vec3<Scalar> GetForce() const { return force_; }
    [[nodiscard]] double GetMaxThrust() const { return max_thrust_; }
    [[nodiscard]] double GetIspVacuum() const { return isp_vacuum_; }

  private:
    std::string name_;
    std::string entity_;

    // Engine parameters
    double max_thrust_{50000.0};      ///< Maximum thrust [N]
    double isp_vacuum_{300.0};        ///< Vacuum Isp [s]
    Scalar exhaust_velocity_{2942.0}; ///< Ve = Isp * g0 [m/s]

    // Geometry
    Vec3<Scalar> thrust_direction_{Scalar(0), Scalar(0), Scalar(1)}; ///< Thrust direction (body)
    Vec3<Scalar> nozzle_position_ = Vec3<Scalar>::Zero();            ///< Nozzle location (body)

    // Input
    InputHandle<Scalar> throttle_cmd_;

    // Outputs
    Scalar thrust_{0};
    Scalar mass_flow_rate_{0};
    Vec3<Scalar> force_ = Vec3<Scalar>::Zero();
};

} // namespace components
} // namespace icarus
