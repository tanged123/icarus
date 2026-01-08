#pragma once

/**
 * @file FuelTank.hpp
 * @brief Propellant tank with mass depletion
 *
 * Tracks propellant mass as an integrable state, providing time-varying
 * mass properties to MassAggregator.
 *
 * Physics:
 *   d(fuel_mass)/dt = -mass_flow_rate
 *   total_mass = dry_mass + fuel_mass
 *
 * Inertia model uses cylindrical approximation for the tank.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>

namespace icarus {
namespace components {

/**
 * @brief Propellant tank with integrable fuel mass
 *
 * Models a fuel tank with depleting propellant:
 * - Fuel mass decreases based on engine mass flow rate
 * - Outputs mass properties for MassAggregator
 * - Clamps fuel mass at zero (no negative fuel)
 *
 * Configuration:
 * - initial_fuel_mass: Starting propellant mass [kg]
 * - dry_mass: Empty tank mass [kg]
 * - tank_cg: CG position in body frame [m]
 * - tank_radius: For inertia computation [m] (optional)
 * - tank_length: For inertia computation [m] (optional)
 *
 * Inputs:
 * - mass_flow_rate: Consumption rate from engine [kg/s]
 *
 * Outputs (MassAggregator compatible):
 * - mass: Current total mass [kg]
 * - fuel_mass: Current propellant mass [kg]
 * - cg.x, .y, .z: CG position [m]
 * - inertia.xx, .yy, .zz, .xy, .xz, .yz: Inertia tensor [kg·m²]
 *
 * State:
 * - fuel_mass: Integrable propellant mass [kg]
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class FuelTank : public Component<Scalar> {
  public:
    explicit FuelTank(std::string name = "FuelTank", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "FuelTank"; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    void Provision(Backplane<Scalar> &bp) override {
        // === State (integrable fuel mass) ===
        bp.template register_state<Scalar>("fuel_mass", &fuel_mass_, &fuel_mass_dot_, "kg",
                                           "Propellant mass");

        // === Outputs (MassAggregator compatible) ===
        // Total mass (dry + fuel)
        bp.template register_output<Scalar>("mass", &total_mass_, "kg", "Total tank mass");

        // CG position
        bp.template register_output_vec3<Scalar>("cg", &cg_, "m", "CG position in body frame");

        // Inertia tensor (6 unique components)
        bp.template register_output<Scalar>("inertia.xx", &inertia_xx_, "kg*m^2",
                                            "Moment of inertia Ixx");
        bp.template register_output<Scalar>("inertia.yy", &inertia_yy_, "kg*m^2",
                                            "Moment of inertia Iyy");
        bp.template register_output<Scalar>("inertia.zz", &inertia_zz_, "kg*m^2",
                                            "Moment of inertia Izz");
        bp.template register_output<Scalar>("inertia.xy", &inertia_xy_, "kg*m^2",
                                            "Product of inertia Ixy");
        bp.template register_output<Scalar>("inertia.xz", &inertia_xz_, "kg*m^2",
                                            "Product of inertia Ixz");
        bp.template register_output<Scalar>("inertia.yz", &inertia_yz_, "kg*m^2",
                                            "Product of inertia Iyz");

        // === Inputs ===
        // Mass flow rate from engine
        bp.template register_input<Scalar>("mass_flow_rate", &mass_flow_rate_, "kg/s",
                                           "Propellant consumption rate");
    }

    void Stage(Backplane<Scalar> &) override {
        const auto &config = this->GetConfig();

        // Read tank parameters
        if (config.template Has<double>("initial_fuel_mass")) {
            initial_fuel_mass_ = config.template Get<double>("initial_fuel_mass", 1000.0);
            fuel_mass_ = Scalar(initial_fuel_mass_);
        }
        if (config.template Has<double>("dry_mass")) {
            dry_mass_ = config.template Get<double>("dry_mass", 50.0);
        }
        if (config.template Has<double>("tank_radius")) {
            tank_radius_ = config.template Get<double>("tank_radius", 0.5);
        }
        if (config.template Has<double>("tank_length")) {
            tank_length_ = config.template Get<double>("tank_length", 2.0);
        }

        // Read CG position
        if (config.template Has<Vec3<double>>("tank_cg")) {
            auto pos = config.template Get<Vec3<double>>("tank_cg", Vec3<double>::Zero());
            cg_ = Vec3<Scalar>{static_cast<Scalar>(pos(0)), static_cast<Scalar>(pos(1)),
                               static_cast<Scalar>(pos(2))};
        }

        // Initialize outputs
        UpdateMassProperties();
    }

    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Get mass flow rate from engine
        Scalar mdot = mass_flow_rate_.get();

        // Fuel depletes: d(fuel_mass)/dt = -mdot
        // But only if there's fuel remaining
        fuel_mass_dot_ = janus::where(fuel_mass_ > Scalar(0), -mdot, Scalar(0));

        // Update total mass and inertia
        UpdateMassProperties();
    }

    // =========================================================================
    // Configuration (programmatic)
    // =========================================================================

    void SetInitialFuelMass(double m) {
        initial_fuel_mass_ = m;
        fuel_mass_ = Scalar(m);
    }
    void SetDryMass(double m) { dry_mass_ = m; }
    void SetTankCG(const Vec3<Scalar> &pos) { cg_ = pos; }
    void SetTankGeometry(double radius, double length) {
        tank_radius_ = radius;
        tank_length_ = length;
    }

    // Accessors
    [[nodiscard]] Scalar GetFuelMass() const { return fuel_mass_; }
    [[nodiscard]] Scalar GetTotalMass() const { return total_mass_; }
    [[nodiscard]] double GetInitialFuelMass() const { return initial_fuel_mass_; }
    [[nodiscard]] double GetDryMass() const { return dry_mass_; }

  private:
    /**
     * @brief Update mass properties based on current fuel mass
     *
     * Computes total mass and inertia tensor.
     * Uses cylindrical approximation for inertia:
     *   I_radial = (m/12) * (3*r² + L²)
     *   I_axial = (m/2) * r²
     */
    void UpdateMassProperties() {
        // Total mass
        total_mass_ = Scalar(dry_mass_) + fuel_mass_;

        // Cylindrical inertia approximation
        // Assuming tank axis is along Z in body frame
        double r2 = tank_radius_ * tank_radius_;
        double L2 = tank_length_ * tank_length_;

        // Radial (X, Y axes): I = (m/12) * (3*r² + L²)
        Scalar I_radial = total_mass_ * Scalar((3.0 * r2 + L2) / 12.0);

        // Axial (Z axis): I = (m/2) * r²
        Scalar I_axial = total_mass_ * Scalar(r2 / 2.0);

        inertia_xx_ = I_radial;
        inertia_yy_ = I_radial;
        inertia_zz_ = I_axial;

        // Off-diagonal terms zero for cylinder aligned with principal axes
        inertia_xy_ = Scalar(0);
        inertia_xz_ = Scalar(0);
        inertia_yz_ = Scalar(0);
    }

    std::string name_;
    std::string entity_;

    // Tank parameters
    double initial_fuel_mass_{1000.0}; ///< Initial propellant [kg]
    double dry_mass_{50.0};            ///< Empty tank mass [kg]
    double tank_radius_{0.5};          ///< Tank radius [m]
    double tank_length_{2.0};          ///< Tank length [m]

    // State
    Scalar fuel_mass_{1000.0}; ///< Current fuel mass [kg]
    Scalar fuel_mass_dot_{0};  ///< Fuel mass derivative [kg/s]

    // CG position (constant for now)
    Vec3<Scalar> cg_ = Vec3<Scalar>::Zero();

    // Input
    InputHandle<Scalar> mass_flow_rate_;

    // Outputs
    Scalar total_mass_{1050.0};
    Scalar inertia_xx_{1.0};
    Scalar inertia_yy_{1.0};
    Scalar inertia_zz_{1.0};
    Scalar inertia_xy_{0};
    Scalar inertia_xz_{0};
    Scalar inertia_yz_{0};
};

} // namespace components
} // namespace icarus
