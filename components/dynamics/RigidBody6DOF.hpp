#pragma once

/**
 * @file RigidBody6DOF.hpp
 * @brief 6-DOF rigid body dynamics component (quaternion formulation)
 *
 * Implements full 6DOF rigid body equations of motion using Vulcan's
 * compute_6dof_derivatives(). Uses quaternion attitude representation
 * to avoid gimbal lock.
 *
 * State vector (13 variables):
 * - Position: [rx, ry, rz] in reference frame (ECI/ECEF)
 * - Velocity: [vx, vy, vz] in body frame
 * - Quaternion: [qw, qx, qy, qz] body-to-reference
 * - Angular velocity: [wx, wy, wz] in body frame
 *
 * Part of Phase 4.4: RigidBody6DOF component
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>

#include <janus/math/Quaternion.hpp>
#include <vulcan/dynamics/RigidBody.hpp>
#include <vulcan/dynamics/RigidBodyTypes.hpp>
#include <vulcan/mass/MassProperties.hpp>

namespace icarus {
namespace components {

/**
 * @brief 6-DOF rigid body dynamics (quaternion formulation)
 *
 * Phase 6: Uses unified signal model - states ARE outputs.
 *
 * Integrates the full 6DOF equations of motion:
 * - Translational dynamics in body frame (with transport term)
 * - Rotational dynamics (Euler's equations)
 * - Quaternion kinematics
 *
 * Uses Vulcan for physics:
 *   - vulcan::dynamics::compute_6dof_derivatives()
 *
 * Equations of motion:
 *   dr/dt = R_body_to_ref * v_body           (position kinematics)
 *   dv/dt = F/m - omega x v                  (translational dynamics)
 *   dq/dt = 0.5 * q * (0, omega)             (quaternion kinematics)
 *   domega/dt = I^-1 * (M - omega x (I*omega)) (Euler's equations)
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class RigidBody6DOF : public Component<Scalar> {
  public:
    /**
     * @brief Construct with name and entity
     */
    explicit RigidBody6DOF(std::string name = "RigidBody6DOF", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "RigidBody6DOF"; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register states, inputs, and parameters
     *
     * Phase 6: States are registered with register_state_vec3/quat() which
     * creates both the state value outputs AND derivative signals.
     */
    void Provision(Backplane<Scalar> &bp) override {
        // === States (Phase 6: unified signal model) ===
        // Position state (creates position.x/y/z and position_dot.x/y/z)
        bp.template register_state_vec3<Scalar>("position", &position_, &position_dot_, "m",
                                                "Position");

        // Velocity body state (creates velocity_body.x/y/z and velocity_body_dot.x/y/z)
        bp.template register_state_vec3<Scalar>(
            "velocity_body", &velocity_body_, &velocity_body_dot_, "m/s", "Velocity in body frame");

        // Attitude quaternion state (creates attitude.w/x/y/z and attitude_dot.w/x/y/z)
        bp.template register_state_quat<Scalar>("attitude", &attitude_, &attitude_dot_, "",
                                                "Attitude quaternion");

        // Angular velocity body state (creates omega_body.x/y/z and omega_body_dot.x/y/z)
        bp.template register_state_vec3<Scalar>("omega_body", &omega_body_, &omega_body_dot_,
                                                "rad/s", "Angular velocity in body frame");

        // === Additional Outputs (derived quantities) ===
        bp.template register_output_vec3<Scalar>("velocity_ref", &velocity_ref_, "m/s",
                                                 "Velocity in reference frame");

        // === Inputs ===
        bp.template register_input<Scalar>("total_force.x", &force_x_, "N", "Total force X");
        bp.template register_input<Scalar>("total_force.y", &force_y_, "N", "Total force Y");
        bp.template register_input<Scalar>("total_force.z", &force_z_, "N", "Total force Z");

        bp.template register_input<Scalar>("total_moment.x", &moment_x_, "N*m", "Total moment X");
        bp.template register_input<Scalar>("total_moment.y", &moment_y_, "N*m", "Total moment Y");
        bp.template register_input<Scalar>("total_moment.z", &moment_z_, "N*m", "Total moment Z");

        bp.template register_input<Scalar>("total_mass", &mass_, "kg", "Total mass");
        bp.template register_input<Scalar>("inertia.xx", &inertia_xx_, "kg*m^2", "Inertia Ixx");
        bp.template register_input<Scalar>("inertia.yy", &inertia_yy_, "kg*m^2", "Inertia Iyy");
        bp.template register_input<Scalar>("inertia.zz", &inertia_zz_, "kg*m^2", "Inertia Izz");
        bp.template register_input<Scalar>("inertia.xy", &inertia_xy_, "kg*m^2", "Inertia Ixy");
        bp.template register_input<Scalar>("inertia.xz", &inertia_xz_, "kg*m^2", "Inertia Ixz");
        bp.template register_input<Scalar>("inertia.yz", &inertia_yz_, "kg*m^2", "Inertia Iyz");
    }

    /**
     * @brief Stage phase - load config and apply initial conditions
     */
    void Stage(Backplane<Scalar> &) override {
        // Load initial conditions using config helpers
        // Pass current values as defaults to preserve programmatic configuration
        position_ = this->read_param_vec3("initial_position", position_);
        velocity_body_ = this->read_param_vec3("initial_velocity_body", velocity_body_);
        attitude_ = this->read_param_vec4("initial_attitude", attitude_);
        omega_body_ = this->read_param_vec3("initial_omega_body", omega_body_);

        // Compute derived output
        janus::Quaternion<Scalar> q{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};
        velocity_ref_ = q.rotate(velocity_body_);

        // Zero all derivatives
        position_dot_ = Vec3<Scalar>::Zero();
        velocity_body_dot_ = Vec3<Scalar>::Zero();
        attitude_dot_ = Vec4<Scalar>::Zero();
        omega_body_dot_ = Vec3<Scalar>::Zero();
    }

    /**
     * @brief Compute derivatives using Vulcan 6DOF dynamics
     *
     * Phase 6: No PreStep/PublishOutputs needed - states ARE outputs.
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Build quaternion from state
        janus::Quaternion<Scalar> quat{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

        // Update derived output
        velocity_ref_ = quat.rotate(velocity_body_);

        // Build force and moment vectors from inputs
        Vec3<Scalar> force{force_x_.get(), force_y_.get(), force_z_.get()};
        Vec3<Scalar> moment{moment_x_.get(), moment_y_.get(), moment_z_.get()};

        // Build inertia tensor from inputs
        Mat3<Scalar> inertia;
        inertia(0, 0) = inertia_xx_.get();
        inertia(1, 1) = inertia_yy_.get();
        inertia(2, 2) = inertia_zz_.get();
        inertia(0, 1) = inertia(1, 0) = inertia_xy_.get();
        inertia(0, 2) = inertia(2, 0) = inertia_xz_.get();
        inertia(1, 2) = inertia(2, 1) = inertia_yz_.get();

        // Build mass properties
        vulcan::mass::MassProperties<Scalar> mass_props;
        mass_props.mass = mass_.get();
        mass_props.cg = Vec3<Scalar>::Zero(); // Forces/moments already about CG
        mass_props.inertia = inertia;

        // Build Vulcan state struct
        vulcan::dynamics::RigidBodyState<Scalar> rb_state;
        rb_state.position = position_;
        rb_state.velocity_body = velocity_body_;
        rb_state.attitude = quat;
        rb_state.omega_body = omega_body_;

        // Compute derivatives using Vulcan
        auto derivs =
            vulcan::dynamics::compute_6dof_derivatives(rb_state, force, moment, mass_props);

        // Write derivatives directly to member variables
        position_dot_ = derivs.position_dot;
        velocity_body_dot_ = derivs.velocity_dot;
        attitude_dot_ = Vec4<Scalar>{derivs.attitude_dot.w, derivs.attitude_dot.x,
                                     derivs.attitude_dot.y, derivs.attitude_dot.z};
        omega_body_dot_ = derivs.omega_dot;
    }

    // =========================================================================
    // Programmatic Configuration
    // =========================================================================

    void SetInitialPosition(const Vec3<Scalar> &pos) { position_ = pos; }
    void SetInitialPosition(Scalar x, Scalar y, Scalar z) { position_ = Vec3<Scalar>{x, y, z}; }

    void SetInitialVelocityBody(const Vec3<Scalar> &vel) { velocity_body_ = vel; }
    void SetInitialVelocityBody(Scalar vx, Scalar vy, Scalar vz) {
        velocity_body_ = Vec3<Scalar>{vx, vy, vz};
    }

    void SetInitialAttitude(const janus::Quaternion<Scalar> &q) {
        attitude_ = Vec4<Scalar>{q.w, q.x, q.y, q.z};
    }
    void SetInitialAttitude(Scalar w, Scalar x, Scalar y, Scalar z) {
        attitude_ = Vec4<Scalar>{w, x, y, z};
    }

    void SetInitialOmegaBody(const Vec3<Scalar> &omega) { omega_body_ = omega; }
    void SetInitialOmegaBody(Scalar wx, Scalar wy, Scalar wz) {
        omega_body_ = Vec3<Scalar>{wx, wy, wz};
    }

    // =========================================================================
    // Accessors
    // =========================================================================

    [[nodiscard]] Vec3<Scalar> GetPosition() const { return position_; }
    [[nodiscard]] Vec3<Scalar> GetVelocityBody() const { return velocity_body_; }
    [[nodiscard]] Vec3<Scalar> GetVelocityRef() const { return velocity_ref_; }
    [[nodiscard]] janus::Quaternion<Scalar> GetAttitude() const {
        return janus::Quaternion<Scalar>{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};
    }
    [[nodiscard]] Vec3<Scalar> GetOmegaBody() const { return omega_body_; }

  private:
    // Identity
    std::string name_;
    std::string entity_;

    // === Phase 6: State variables (ARE the outputs) ===
    Vec3<Scalar> position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> position_dot_ = Vec3<Scalar>::Zero();

    Vec3<Scalar> velocity_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_body_dot_ = Vec3<Scalar>::Zero();

    Vec4<Scalar> attitude_ =
        (Vec4<Scalar>() << Scalar(1), Scalar(0), Scalar(0), Scalar(0)).finished();
    Vec4<Scalar> attitude_dot_ = Vec4<Scalar>::Zero();

    Vec3<Scalar> omega_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> omega_body_dot_ = Vec3<Scalar>::Zero();

    // === Derived output ===
    Vec3<Scalar> velocity_ref_ = Vec3<Scalar>::Zero();

    // === Input Handles ===
    InputHandle<Scalar> force_x_;
    InputHandle<Scalar> force_y_;
    InputHandle<Scalar> force_z_;
    InputHandle<Scalar> moment_x_;
    InputHandle<Scalar> moment_y_;
    InputHandle<Scalar> moment_z_;

    InputHandle<Scalar> mass_;
    InputHandle<Scalar> inertia_xx_, inertia_yy_, inertia_zz_;
    InputHandle<Scalar> inertia_xy_, inertia_xz_, inertia_yz_;
};

} // namespace components
} // namespace icarus
