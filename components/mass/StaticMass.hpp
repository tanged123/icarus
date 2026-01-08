#pragma once

/**
 * @file StaticMass.hpp
 * @brief Static mass source component for aggregation testing
 *
 * Publishes constant mass properties (mass, CG, inertia) to the backplane.
 * Used as a mass source for MassAggregator.
 *
 * Part of Phase 4: Aggregation & 6DOF
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>

#include <vulcan/mass/MassProperties.hpp>

namespace icarus {
namespace components {

/**
 * @brief Static mass source component
 *
 * Publishes constant mass properties to the backplane for aggregation.
 * All values are set via configuration and remain constant during simulation.
 *
 * Signals published:
 * - mass [kg]
 * - cg.x, cg.y, cg.z [m] - CG position in body frame
 * - inertia.xx, .yy, .zz [kg·m²] - Principal moments
 * - inertia.xy, .xz, .yz [kg·m²] - Products of inertia
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class StaticMass : public Component<Scalar> {
  public:
    explicit StaticMass(std::string name = "StaticMass", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "StaticMass"; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    void Provision(Backplane<Scalar> &bp) override {
        // Mass scalar
        bp.template register_output<Scalar>("mass", &mass_, "kg", "Total mass");

        // CG position
        bp.template register_output_vec3<Scalar>("cg", &cg_, "m", "CG position in body frame");

        // Inertia tensor (6 unique components for symmetric matrix)
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

        // Parameters for configuration
        bp.register_param("mass", &mass_, mass_, "kg", "Total mass");
    }

    void Stage(Backplane<Scalar> &) override {
        // Read mass properties using config helpers
        // Pass current values as defaults to preserve programmatic configuration
        mass_ = static_cast<Scalar>(
            this->template read_param<double>("mass", static_cast<double>(mass_)));
        cg_ = this->read_param_vec3("cg", cg_);

        // Read diagonal inertia
        inertia_xx_ = static_cast<Scalar>(
            this->template read_param<double>("inertia_xx", static_cast<double>(inertia_xx_)));
        inertia_yy_ = static_cast<Scalar>(
            this->template read_param<double>("inertia_yy", static_cast<double>(inertia_yy_)));
        inertia_zz_ = static_cast<Scalar>(
            this->template read_param<double>("inertia_zz", static_cast<double>(inertia_zz_)));

        // Read products of inertia
        inertia_xy_ = static_cast<Scalar>(
            this->template read_param<double>("inertia_xy", static_cast<double>(inertia_xy_)));
        inertia_xz_ = static_cast<Scalar>(
            this->template read_param<double>("inertia_xz", static_cast<double>(inertia_xz_)));
        inertia_yz_ = static_cast<Scalar>(
            this->template read_param<double>("inertia_yz", static_cast<double>(inertia_yz_)));
    }

    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;
        // Static mass - nothing to compute each step
        // Values are already published via output pointers
    }

    // =========================================================================
    // Configuration (programmatic)
    // =========================================================================

    void SetMass(Scalar m) { mass_ = m; }
    void SetCG(const Vec3<Scalar> &cg) { cg_ = cg; }

    void SetDiagonalInertia(Scalar Ixx, Scalar Iyy, Scalar Izz) {
        inertia_xx_ = Ixx;
        inertia_yy_ = Iyy;
        inertia_zz_ = Izz;
        inertia_xy_ = Scalar(0);
        inertia_xz_ = Scalar(0);
        inertia_yz_ = Scalar(0);
    }

    void SetInertia(Scalar Ixx, Scalar Iyy, Scalar Izz, Scalar Ixy, Scalar Ixz, Scalar Iyz) {
        inertia_xx_ = Ixx;
        inertia_yy_ = Iyy;
        inertia_zz_ = Izz;
        inertia_xy_ = Ixy;
        inertia_xz_ = Ixz;
        inertia_yz_ = Iyz;
    }

    /**
     * @brief Set from Vulcan MassProperties
     */
    void SetFromMassProperties(const vulcan::mass::MassProperties<Scalar> &props) {
        mass_ = props.mass;
        cg_ = props.cg;
        inertia_xx_ = props.inertia(0, 0);
        inertia_yy_ = props.inertia(1, 1);
        inertia_zz_ = props.inertia(2, 2);
        // Note: Vulcan uses standard convention where off-diagonals are negative
        // Ixy in tensor is -∫xy dm, so we store the tensor value directly
        inertia_xy_ = props.inertia(0, 1);
        inertia_xz_ = props.inertia(0, 2);
        inertia_yz_ = props.inertia(1, 2);
    }

    /**
     * @brief Get as Vulcan MassProperties
     */
    [[nodiscard]] vulcan::mass::MassProperties<Scalar> GetMassProperties() const {
        Mat3<Scalar> I;
        I(0, 0) = inertia_xx_;
        I(1, 1) = inertia_yy_;
        I(2, 2) = inertia_zz_;
        I(0, 1) = I(1, 0) = inertia_xy_;
        I(0, 2) = I(2, 0) = inertia_xz_;
        I(1, 2) = I(2, 1) = inertia_yz_;

        return vulcan::mass::MassProperties<Scalar>{.mass = mass_, .cg = cg_, .inertia = I};
    }

  private:
    std::string name_;
    std::string entity_;

    // Mass properties
    Scalar mass_{1.0};
    Vec3<Scalar> cg_ = Vec3<Scalar>::Zero();

    // Inertia tensor components (symmetric, 6 unique values)
    Scalar inertia_xx_{1.0};
    Scalar inertia_yy_{1.0};
    Scalar inertia_zz_{1.0};
    Scalar inertia_xy_{0.0};
    Scalar inertia_xz_{0.0};
    Scalar inertia_yz_{0.0};
};

} // namespace components
} // namespace icarus
