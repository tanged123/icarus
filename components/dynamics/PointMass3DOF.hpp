#pragma once

/**
 * @file PointMass3DOF.hpp
 * @brief 3-DOF point mass dynamics component (inertial frame)
 *
 * Wraps Vulcan's point_mass_acceleration() for translational dynamics.
 * Suitable for ECI orbital dynamics or local-frame validation.
 *
 * Frame: Inertial (ECI) or local Cartesian (for validation)
 * NOT suitable for ECEF without adding Coriolis/centrifugal terms.
 *
 * Part of Phase 2.3: First Real Component
 * Updated for Phase 2.4: Uses register_input for explicit input declaration
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/dynamics/PointMass.hpp>

namespace icarus {
namespace components {

/**
 * @brief 3-DOF point mass translational dynamics (inertial frame)
 *
 * Phase 6: Uses unified signal model - states ARE outputs.
 *
 * Frame Assumption: This component assumes an INERTIAL reference frame.
 * - For orbital dynamics: use ECI coordinates
 * - For validation tests: use local Cartesian (Z-up)
 *
 * Uses Vulcan for physics:
 *   - vulcan::dynamics::point_mass_acceleration(force, mass) → a = F/m
 *
 * Equations of motion:
 *   dr/dt = v           (kinematics)
 *   dv/dt = a = F/m     (Newton's 2nd law via Vulcan)
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class PointMass3DOF : public Component<Scalar> {
  public:
    /**
     * @brief Construct with name and entity
     */
    explicit PointMass3DOF(std::string name = "PointMass3DOF", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "PointMass3DOF"; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register states, inputs, and parameters
     *
     * Phase 6: States are registered with register_state_vec3() which creates
     * both the state value outputs AND the derivative signals. No separate
     * output registration needed - states ARE outputs.
     */
    void Provision(Backplane<Scalar> &bp) override {
        // === States (Phase 6: unified signal model) ===
        // Position state with derivative (creates position.x/y/z and position_dot.x/y/z)
        bp.template register_state_vec3<Scalar>("position", &position_, &position_dot_, "m",
                                                "Position");

        // Velocity state with derivative (creates velocity.x/y/z and velocity_dot.x/y/z)
        bp.template register_state_vec3<Scalar>("velocity", &velocity_, &velocity_dot_, "m/s",
                                                "Velocity");

        // === Inputs ===
        bp.template register_input<Scalar>("force.x", &force_x_, "N", "Applied force X");
        bp.template register_input<Scalar>("force.y", &force_y_, "N", "Applied force Y");
        bp.template register_input<Scalar>("force.z", &force_z_, "N", "Applied force Z");

        // === Parameters ===
        bp.register_param("mass", &mass_, mass_, "kg", "Point mass");
        bp.template register_output<Scalar>("mass", &mass_, "kg", "Point mass");
    }

    /**
     * @brief Stage phase - load config and apply initial conditions
     */
    void Stage(Backplane<Scalar> &) override {
        const auto &config = this->GetConfig();

        // Load mass from config if set
        if (config.template Has<double>("mass")) {
            mass_ = static_cast<Scalar>(config.template Get<double>("mass", 1.0));
        }

        // Load initial position if set
        if (config.template Has<Vec3<double>>("initial_position")) {
            auto pos = config.template Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
            position_ = Vec3<Scalar>{static_cast<Scalar>(pos(0)), static_cast<Scalar>(pos(1)),
                                     static_cast<Scalar>(pos(2))};
        }

        // Load initial velocity if set
        if (config.template Has<Vec3<double>>("initial_velocity")) {
            auto vel = config.template Get<Vec3<double>>("initial_velocity", Vec3<double>::Zero());
            velocity_ = Vec3<Scalar>{static_cast<Scalar>(vel(0)), static_cast<Scalar>(vel(1)),
                                     static_cast<Scalar>(vel(2))};
        }

        // Zero derivatives
        position_dot_ = Vec3<Scalar>::Zero();
        velocity_dot_ = Vec3<Scalar>::Zero();
    }

    /**
     * @brief Compute derivatives using Vulcan dynamics
     *
     * Phase 6: No PreStep/PublishOutputs needed - states ARE outputs.
     * The integrator updates position_/velocity_ directly.
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read force input
        Vec3<Scalar> force{force_x_.get(), force_y_.get(), force_z_.get()};

        // Compute acceleration using Vulcan (a = F/m)
        Vec3<Scalar> accel = vulcan::dynamics::point_mass_acceleration(force, mass_);

        // Write derivatives directly to member variables
        // dr/dt = v (kinematics)
        position_dot_ = velocity_;

        // dv/dt = a = F/m
        velocity_dot_ = accel;
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    void SetMass(Scalar mass) { mass_ = mass; }
    [[nodiscard]] Scalar GetMass() const { return mass_; }

    void SetInitialPosition(const Vec3<Scalar> &pos) { position_ = pos; }
    void SetInitialPosition(Scalar x, Scalar y, Scalar z) { position_ = Vec3<Scalar>{x, y, z}; }

    void SetInitialVelocity(const Vec3<Scalar> &vel) { velocity_ = vel; }
    void SetInitialVelocity(Scalar vx, Scalar vy, Scalar vz) {
        velocity_ = Vec3<Scalar>{vx, vy, vz};
    }

    // Accessors
    [[nodiscard]] Vec3<Scalar> GetPosition() const { return position_; }
    [[nodiscard]] Vec3<Scalar> GetVelocity() const { return velocity_; }

  private:
    // Identity
    std::string name_;
    std::string entity_;

    // === Phase 6: State variables (ARE the outputs) ===
    Vec3<Scalar> position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> position_dot_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_dot_ = Vec3<Scalar>::Zero();

    // === Inputs ===
    InputHandle<Scalar> force_x_;
    InputHandle<Scalar> force_y_;
    InputHandle<Scalar> force_z_;

    // === Parameter ===
    Scalar mass_{1.0};
};

} // namespace components
} // namespace icarus
