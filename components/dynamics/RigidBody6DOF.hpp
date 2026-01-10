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
#include <vulcan/coordinates/BodyFrames.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/core/Constants.hpp>
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

        // === Coordinate Frame Outputs ===
        // Position in LLA (geodetic)
        bp.template register_output<Scalar>("position_lla.lat", &position_lla_(0), "rad",
                                            "Geodetic latitude");
        bp.template register_output<Scalar>("position_lla.lon", &position_lla_(1), "rad",
                                            "Longitude");
        bp.template register_output<Scalar>("position_lla.alt", &position_lla_(2), "m",
                                            "Altitude above ellipsoid");

        // Velocity in NED
        bp.template register_output<Scalar>("velocity_ned.n", &velocity_ned_(0), "m/s",
                                            "North velocity");
        bp.template register_output<Scalar>("velocity_ned.e", &velocity_ned_(1), "m/s",
                                            "East velocity");
        bp.template register_output<Scalar>("velocity_ned.d", &velocity_ned_(2), "m/s",
                                            "Down velocity");

        // Attitude as Euler angles (ZYX: yaw-pitch-roll)
        bp.template register_output<Scalar>("euler_zyx.yaw", &euler_zyx_(0), "rad",
                                            "Yaw angle (heading from North)");
        bp.template register_output<Scalar>("euler_zyx.pitch", &euler_zyx_(1), "rad",
                                            "Pitch angle");
        bp.template register_output<Scalar>("euler_zyx.roll", &euler_zyx_(2), "rad", "Roll angle");

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
     *
     * Position init controlled by position_init_mode: "lla" or "ecef" (default)
     * Attitude init controlled by attitude_init_mode: "euler_zyx" or "quaternion" (default)
     */
    void Stage(Backplane<Scalar> &) override {
        const auto &config = this->GetConfig();

        // === Position Initialization ===
        // Track position as double for coordinate transforms (needed for Euler init)
        Vec3<double> r_ecef_double = Vec3<double>::Zero();

        auto pos_mode = this->template read_param<std::string>("position_init_mode", "ecef");
        if (pos_mode == "lla") {
            // LLA format: [lat_deg, lon_deg, alt_m]
            auto lla_vec = config.template Get<Vec3<double>>("initial_lla", Vec3<double>::Zero());

            vulcan::LLA<double> lla;
            lla.lat = lla_vec(0) * vulcan::constants::angle::deg2rad;
            lla.lon = lla_vec(1) * vulcan::constants::angle::deg2rad;
            lla.alt = lla_vec(2);

            r_ecef_double = vulcan::lla_to_ecef(lla);
            position_ = Vec3<Scalar>{static_cast<Scalar>(r_ecef_double(0)),
                                     static_cast<Scalar>(r_ecef_double(1)),
                                     static_cast<Scalar>(r_ecef_double(2))};
        } else {
            r_ecef_double =
                config.template Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
            position_ = this->read_param_vec3("initial_position", position_);
        }

        // Load velocity and omega (always body frame)
        velocity_body_ = this->read_param_vec3("initial_velocity_body", velocity_body_);
        omega_body_ = this->read_param_vec3("initial_omega_body", omega_body_);

        // === Attitude Initialization ===
        auto att_mode = this->template read_param<std::string>("attitude_init_mode", "quaternion");
        if (att_mode == "euler_zyx") {
            // Euler format: [yaw_deg, pitch_deg, roll_deg] (ZYX sequence)
            auto euler_deg =
                config.template Get<Vec3<double>>("initial_euler_zyx", Vec3<double>::Zero());

            double yaw = euler_deg(0) * vulcan::constants::angle::deg2rad;
            double pitch = euler_deg(1) * vulcan::constants::angle::deg2rad;
            double roll = euler_deg(2) * vulcan::constants::angle::deg2rad;

            // Compute NED frame at position (using double for coordinate transforms)
            auto ned = vulcan::local_ned_at(r_ecef_double);

            // Build body frame from Euler angles relative to NED
            auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

            // Extract quaternion (body-to-ECEF)
            Mat3<double> dcm;
            dcm.col(0) = body.x_axis;
            dcm.col(1) = body.y_axis;
            dcm.col(2) = body.z_axis;
            auto q_body_to_ecef = janus::Quaternion<double>::from_rotation_matrix(dcm);

            attitude_ = Vec4<Scalar>{
                static_cast<Scalar>(q_body_to_ecef.w), static_cast<Scalar>(q_body_to_ecef.x),
                static_cast<Scalar>(q_body_to_ecef.y), static_cast<Scalar>(q_body_to_ecef.z)};
        } else {
            attitude_ = this->read_param_vec4("initial_attitude", attitude_);
        }

        // Compute derived outputs
        janus::Quaternion<Scalar> q{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};
        velocity_ref_ = q.rotate(velocity_body_);
        ComputeCoordinateFrameOutputs();

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

        // Update derived outputs
        velocity_ref_ = quat.rotate(velocity_body_);
        ComputeCoordinateFrameOutputs();

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

    /**
     * @brief Set initial position from LLA coordinates
     * @param lat_deg Geodetic latitude [degrees]
     * @param lon_deg Longitude [degrees]
     * @param alt_m Altitude above ellipsoid [meters]
     */
    void SetInitialPositionLLA(double lat_deg, double lon_deg, double alt_m) {
        vulcan::LLA<double> lla;
        lla.lat = lat_deg * vulcan::constants::angle::deg2rad;
        lla.lon = lon_deg * vulcan::constants::angle::deg2rad;
        lla.alt = alt_m;
        Vec3<double> r_ecef = vulcan::lla_to_ecef(lla);
        position_ = Vec3<Scalar>{static_cast<Scalar>(r_ecef(0)), static_cast<Scalar>(r_ecef(1)),
                                 static_cast<Scalar>(r_ecef(2))};
    }

    /**
     * @brief Set initial attitude from Euler angles relative to local NED
     * @param yaw_deg Yaw angle from North [degrees], positive clockwise
     * @param pitch_deg Pitch angle [degrees], positive nose up
     * @param roll_deg Roll angle [degrees], positive right wing down
     * @note Position must be set before calling this method
     */
    void SetInitialAttitudeEuler(double yaw_deg, double pitch_deg, double roll_deg) {
        double yaw = yaw_deg * vulcan::constants::angle::deg2rad;
        double pitch = pitch_deg * vulcan::constants::angle::deg2rad;
        double roll = roll_deg * vulcan::constants::angle::deg2rad;

        // Get current position in ECEF
        Vec3<double> r_ecef{static_cast<double>(position_(0)), static_cast<double>(position_(1)),
                            static_cast<double>(position_(2))};

        // Compute NED frame at this position
        auto ned = vulcan::local_ned_at(r_ecef);

        // Build body frame from Euler angles relative to NED
        auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

        // Extract quaternion (body-to-ECEF)
        Mat3<double> dcm;
        dcm.col(0) = body.x_axis;
        dcm.col(1) = body.y_axis;
        dcm.col(2) = body.z_axis;
        auto q_body_to_ecef = janus::Quaternion<double>::from_rotation_matrix(dcm);

        attitude_ = Vec4<Scalar>{
            static_cast<Scalar>(q_body_to_ecef.w), static_cast<Scalar>(q_body_to_ecef.x),
            static_cast<Scalar>(q_body_to_ecef.y), static_cast<Scalar>(q_body_to_ecef.z)};
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

    // Coordinate frame accessors
    [[nodiscard]] Vec3<Scalar> GetPositionLLA() const { return position_lla_; }
    [[nodiscard]] Vec3<Scalar> GetVelocityNED() const { return velocity_ned_; }
    [[nodiscard]] Vec3<Scalar> GetEulerZYX() const { return euler_zyx_; }

  private:
    // =========================================================================
    // Private Methods
    // =========================================================================

    /**
     * @brief Compute coordinate frame outputs (LLA, NED velocity, Euler angles)
     *
     * Called during Stage() and Step() to update derived coordinate frame outputs.
     */
    void ComputeCoordinateFrameOutputs() {
        // Convert position to LLA
        vulcan::LLA<Scalar> lla = vulcan::ecef_to_lla(position_);
        position_lla_(0) = lla.lat;
        position_lla_(1) = lla.lon;
        position_lla_(2) = lla.alt;

        // Get NED frame at current position
        auto ned = vulcan::local_ned_at(position_);

        // Transform velocity from ECEF to NED
        velocity_ned_ = ned.from_ecef(velocity_ref_);

        // Compute Euler angles from attitude quaternion
        // Build body frame from quaternion (body-to-ECEF)
        janus::Quaternion<Scalar> quat{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

        // Body axes in ECEF (quaternion rotates body coords to ECEF)
        Vec3<Scalar> x_body_ecef = quat.rotate(Vec3<Scalar>::UnitX());
        Vec3<Scalar> y_body_ecef = quat.rotate(Vec3<Scalar>::UnitY());
        Vec3<Scalar> z_body_ecef = quat.rotate(Vec3<Scalar>::UnitZ());

        // Build body frame for euler_from_body
        vulcan::CoordinateFrame<Scalar> body_frame(x_body_ecef, y_body_ecef, z_body_ecef,
                                                   Vec3<Scalar>::Zero());

        // Extract Euler angles (returns [yaw, pitch, roll])
        // NOTE: This can have singularities at pitch = +/-90 degrees (gimbal lock)
        // The Euler outputs may be unreliable near these angles, but this should
        // not affect the dynamics which use quaternions directly.
        euler_zyx_ = vulcan::euler_from_body(body_frame, ned);
    }

    // =========================================================================
    // Member Variables
    // =========================================================================

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

    // === Derived outputs ===
    Vec3<Scalar> velocity_ref_ = Vec3<Scalar>::Zero();

    // Coordinate frame outputs
    Vec3<Scalar> position_lla_ = Vec3<Scalar>::Zero(); ///< [lat, lon, alt] in [rad, rad, m]
    Vec3<Scalar> velocity_ned_ = Vec3<Scalar>::Zero(); ///< [vn, ve, vd] in m/s
    Vec3<Scalar> euler_zyx_ = Vec3<Scalar>::Zero();    ///< [yaw, pitch, roll] in rad

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
