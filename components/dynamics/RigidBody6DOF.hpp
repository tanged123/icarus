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
    // State vector layout
    static constexpr std::size_t kStateSize = 13;
    static constexpr std::size_t kPosOffset = 0;    // [0-2]: position (3)
    static constexpr std::size_t kVelOffset = 3;    // [3-5]: velocity body (3)
    static constexpr std::size_t kQuatOffset = 6;   // [6-9]: quaternion (4)
    static constexpr std::size_t kOmegaOffset = 10; // [10-12]: omega body (3)

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

    [[nodiscard]] std::size_t StateSize() const override { return kStateSize; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register outputs, inputs, and parameters
     */
    void Provision(Backplane<Scalar> &bp) override {
        // === Outputs ===
        // Position (in reference frame: ECI or ECEF)
        bp.template register_output_vec3<Scalar>("position", &position_, "m", "Position");

        // Velocity (in body frame)
        bp.template register_output_vec3<Scalar>("velocity_body", &velocity_body_, "m/s",
                                                 "Velocity in body frame");

        // Velocity (in reference frame, computed for convenience)
        bp.template register_output_vec3<Scalar>("velocity_ref", &velocity_ref_, "m/s",
                                                 "Velocity in reference frame");

        // Attitude quaternion [w, x, y, z]
        bp.template register_output<Scalar>("attitude.w", &quat_w_, "", "Quaternion W (scalar)");
        bp.template register_output<Scalar>("attitude.x", &quat_x_, "", "Quaternion X");
        bp.template register_output<Scalar>("attitude.y", &quat_y_, "", "Quaternion Y");
        bp.template register_output<Scalar>("attitude.z", &quat_z_, "", "Quaternion Z");

        // Angular velocity (body frame)
        bp.template register_output_vec3<Scalar>("omega_body", &omega_body_, "rad/s",
                                                 "Angular velocity in body frame");

        // === Inputs ===
        // Total force from ForceAggregator (body frame)
        bp.template register_input<Scalar>("total_force.x", &force_x_, "N", "Total force X");
        bp.template register_input<Scalar>("total_force.y", &force_y_, "N", "Total force Y");
        bp.template register_input<Scalar>("total_force.z", &force_z_, "N", "Total force Z");

        // Total moment from ForceAggregator (body frame, about CG)
        bp.template register_input<Scalar>("total_moment.x", &moment_x_, "N*m", "Total moment X");
        bp.template register_input<Scalar>("total_moment.y", &moment_y_, "N*m", "Total moment Y");
        bp.template register_input<Scalar>("total_moment.z", &moment_z_, "N*m", "Total moment Z");

        // Mass properties from MassAggregator
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
     *
     * Reads from config (only if explicitly set):
     * - vectors["initial_position"]: Initial position [x, y, z]
     * - vectors["initial_velocity_body"]: Initial body-frame velocity [vx, vy, vz]
     * - vectors["initial_attitude"]: Initial quaternion [w, x, y, z]
     * - vectors["initial_omega_body"]: Initial angular velocity [wx, wy, wz]
     */
    void Stage(Backplane<Scalar> &) override {
        const auto &config = this->GetConfig();

        // Load initial position
        if (config.template Has<Vec3<double>>("initial_position")) {
            auto pos = config.template Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
            ic_position_ = Vec3<Scalar>{static_cast<Scalar>(pos(0)), static_cast<Scalar>(pos(1)),
                                        static_cast<Scalar>(pos(2))};
        }

        // Load initial velocity (body frame)
        if (config.template Has<Vec3<double>>("initial_velocity_body")) {
            auto vel =
                config.template Get<Vec3<double>>("initial_velocity_body", Vec3<double>::Zero());
            ic_velocity_body_ =
                Vec3<Scalar>{static_cast<Scalar>(vel(0)), static_cast<Scalar>(vel(1)),
                             static_cast<Scalar>(vel(2))};
        }

        // Load initial attitude quaternion [w, x, y, z]
        if (config.template Has<Vec4<double>>("initial_attitude")) {
            Vec4<double> default_quat;
            default_quat << 1.0, 0.0, 0.0, 0.0; // Identity quaternion
            auto quat = config.template Get<Vec4<double>>("initial_attitude", default_quat);
            ic_quat_ = janus::Quaternion<Scalar>{
                static_cast<Scalar>(quat(0)), static_cast<Scalar>(quat(1)),
                static_cast<Scalar>(quat(2)), static_cast<Scalar>(quat(3))};
        }

        // Load initial angular velocity (body frame)
        if (config.template Has<Vec3<double>>("initial_omega_body")) {
            auto omega =
                config.template Get<Vec3<double>>("initial_omega_body", Vec3<double>::Zero());
            ic_omega_body_ =
                Vec3<Scalar>{static_cast<Scalar>(omega(0)), static_cast<Scalar>(omega(1)),
                             static_cast<Scalar>(omega(2))};
        }

        // Apply ICs to state vector if already bound (for Reset() support)
        if (state_ != nullptr) {
            ApplyInitialConditions();
        }
    }

    /**
     * @brief Bind to global state vector slices
     */
    void BindState(Scalar *state, Scalar *state_dot, std::size_t size) override {
        if (size != kStateSize) {
            throw StateSizeMismatchError(kStateSize, size);
        }

        state_ = state;
        state_dot_ = state_dot;

        // Apply initial conditions to state
        ApplyInitialConditions();
    }

    /**
     * @brief Publish state outputs BEFORE other components' Step() runs
     */
    void PreStep(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Publish current state to outputs
        PublishOutputs();
    }

    /**
     * @brief Compute derivatives using Vulcan 6DOF dynamics
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read current state
        Vec3<Scalar> pos{state_[kPosOffset + 0], state_[kPosOffset + 1], state_[kPosOffset + 2]};
        Vec3<Scalar> vel_body{state_[kVelOffset + 0], state_[kVelOffset + 1],
                              state_[kVelOffset + 2]};
        janus::Quaternion<Scalar> quat{state_[kQuatOffset + 0], state_[kQuatOffset + 1],
                                       state_[kQuatOffset + 2], state_[kQuatOffset + 3]};
        Vec3<Scalar> omega{state_[kOmegaOffset + 0], state_[kOmegaOffset + 1],
                           state_[kOmegaOffset + 2]};

        // Update outputs for other components
        position_ = pos;
        velocity_body_ = vel_body;
        velocity_ref_ = quat.rotate(vel_body);
        quat_w_ = quat.w;
        quat_x_ = quat.x;
        quat_y_ = quat.y;
        quat_z_ = quat.z;
        omega_body_ = omega;

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
        rb_state.position = pos;
        rb_state.velocity_body = vel_body;
        rb_state.attitude = quat;
        rb_state.omega_body = omega;

        // Compute derivatives using Vulcan
        auto derivs =
            vulcan::dynamics::compute_6dof_derivatives(rb_state, force, moment, mass_props);

        // Write derivatives to state_dot
        state_dot_[kPosOffset + 0] = derivs.position_dot(0);
        state_dot_[kPosOffset + 1] = derivs.position_dot(1);
        state_dot_[kPosOffset + 2] = derivs.position_dot(2);

        state_dot_[kVelOffset + 0] = derivs.velocity_dot(0);
        state_dot_[kVelOffset + 1] = derivs.velocity_dot(1);
        state_dot_[kVelOffset + 2] = derivs.velocity_dot(2);

        state_dot_[kQuatOffset + 0] = derivs.attitude_dot.w;
        state_dot_[kQuatOffset + 1] = derivs.attitude_dot.x;
        state_dot_[kQuatOffset + 2] = derivs.attitude_dot.y;
        state_dot_[kQuatOffset + 3] = derivs.attitude_dot.z;

        state_dot_[kOmegaOffset + 0] = derivs.omega_dot(0);
        state_dot_[kOmegaOffset + 1] = derivs.omega_dot(1);
        state_dot_[kOmegaOffset + 2] = derivs.omega_dot(2);
    }

    // =========================================================================
    // Programmatic Configuration
    // =========================================================================

    void SetInitialPosition(const Vec3<Scalar> &pos) { ic_position_ = pos; }
    void SetInitialPosition(Scalar x, Scalar y, Scalar z) { ic_position_ = Vec3<Scalar>{x, y, z}; }

    void SetInitialVelocityBody(const Vec3<Scalar> &vel) { ic_velocity_body_ = vel; }
    void SetInitialVelocityBody(Scalar vx, Scalar vy, Scalar vz) {
        ic_velocity_body_ = Vec3<Scalar>{vx, vy, vz};
    }

    void SetInitialAttitude(const janus::Quaternion<Scalar> &q) { ic_quat_ = q; }
    void SetInitialAttitude(Scalar w, Scalar x, Scalar y, Scalar z) {
        ic_quat_ = janus::Quaternion<Scalar>{w, x, y, z};
    }

    void SetInitialOmegaBody(const Vec3<Scalar> &omega) { ic_omega_body_ = omega; }
    void SetInitialOmegaBody(Scalar wx, Scalar wy, Scalar wz) {
        ic_omega_body_ = Vec3<Scalar>{wx, wy, wz};
    }

    // =========================================================================
    // Accessors
    // =========================================================================

    [[nodiscard]] Vec3<Scalar> GetPosition() const { return position_; }
    [[nodiscard]] Vec3<Scalar> GetVelocityBody() const { return velocity_body_; }
    [[nodiscard]] Vec3<Scalar> GetVelocityRef() const { return velocity_ref_; }
    [[nodiscard]] janus::Quaternion<Scalar> GetAttitude() const {
        return janus::Quaternion<Scalar>{quat_w_, quat_x_, quat_y_, quat_z_};
    }
    [[nodiscard]] Vec3<Scalar> GetOmegaBody() const { return omega_body_; }

  private:
    // Identity
    std::string name_;
    std::string entity_;

    // Initial conditions
    Vec3<Scalar> ic_position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> ic_velocity_body_ = Vec3<Scalar>::Zero();
    janus::Quaternion<Scalar> ic_quat_{Scalar(1), Scalar(0), Scalar(0), Scalar(0)}; // Identity
    Vec3<Scalar> ic_omega_body_ = Vec3<Scalar>::Zero();

    // State pointers (bound in BindState)
    Scalar *state_ = nullptr;
    Scalar *state_dot_ = nullptr;

    // === Input Handles ===
    // Force and moment from ForceAggregator
    InputHandle<Scalar> force_x_;
    InputHandle<Scalar> force_y_;
    InputHandle<Scalar> force_z_;
    InputHandle<Scalar> moment_x_;
    InputHandle<Scalar> moment_y_;
    InputHandle<Scalar> moment_z_;

    // Mass properties from MassAggregator
    InputHandle<Scalar> mass_;
    InputHandle<Scalar> inertia_xx_, inertia_yy_, inertia_zz_;
    InputHandle<Scalar> inertia_xy_, inertia_xz_, inertia_yz_;

    // Output values
    Vec3<Scalar> position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_ref_ = Vec3<Scalar>::Zero();
    Scalar quat_w_{1}, quat_x_{0}, quat_y_{0}, quat_z_{0};
    Vec3<Scalar> omega_body_ = Vec3<Scalar>::Zero();

    /**
     * @brief Apply initial conditions to state vector
     */
    void ApplyInitialConditions() {
        // Position
        state_[kPosOffset + 0] = ic_position_(0);
        state_[kPosOffset + 1] = ic_position_(1);
        state_[kPosOffset + 2] = ic_position_(2);

        // Velocity (body frame)
        state_[kVelOffset + 0] = ic_velocity_body_(0);
        state_[kVelOffset + 1] = ic_velocity_body_(1);
        state_[kVelOffset + 2] = ic_velocity_body_(2);

        // Quaternion
        state_[kQuatOffset + 0] = ic_quat_.w;
        state_[kQuatOffset + 1] = ic_quat_.x;
        state_[kQuatOffset + 2] = ic_quat_.y;
        state_[kQuatOffset + 3] = ic_quat_.z;

        // Angular velocity (body frame)
        state_[kOmegaOffset + 0] = ic_omega_body_(0);
        state_[kOmegaOffset + 1] = ic_omega_body_(1);
        state_[kOmegaOffset + 2] = ic_omega_body_(2);

        // Update output signals
        PublishOutputs();
    }

    /**
     * @brief Update output signals from state
     */
    void PublishOutputs() {
        position_ =
            Vec3<Scalar>{state_[kPosOffset + 0], state_[kPosOffset + 1], state_[kPosOffset + 2]};
        velocity_body_ =
            Vec3<Scalar>{state_[kVelOffset + 0], state_[kVelOffset + 1], state_[kVelOffset + 2]};
        janus::Quaternion<Scalar> quat{state_[kQuatOffset + 0], state_[kQuatOffset + 1],
                                       state_[kQuatOffset + 2], state_[kQuatOffset + 3]};
        velocity_ref_ = quat.rotate(velocity_body_);
        quat_w_ = quat.w;
        quat_x_ = quat.x;
        quat_y_ = quat.y;
        quat_z_ = quat.z;
        omega_body_ = Vec3<Scalar>{state_[kOmegaOffset + 0], state_[kOmegaOffset + 1],
                                   state_[kOmegaOffset + 2]};
    }
};

} // namespace components
} // namespace icarus
