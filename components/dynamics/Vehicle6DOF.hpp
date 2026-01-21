#pragma once

/**
 * @file Vehicle6DOF.hpp
 * @brief Unified 6DOF vehicle dynamics with mass/force aggregation
 *
 * Part of Phase 9B: Vehicle6DOF Unified Model Implementation.
 * Combines MassAggregator + ForceAggregator + RigidBody6DOF into a single
 * component, eliminating internal signal routing.
 *
 * See docs/implementation_plans/phase9/phase9_vehicle6dof.md for design details.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentFactory.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Handle.hpp>

#include <janus/math/Quaternion.hpp>
#include <vulcan/coordinates/BodyFrames.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/dynamics/RigidBody.hpp>
#include <vulcan/dynamics/RigidBodyTypes.hpp>
#include <vulcan/mass/MassProperties.hpp>

#include <string>
#include <vector>

namespace icarus {
namespace components {

/**
 * @brief Mass source handle bundle for aggregation
 *
 * Holds SignalHandles to read mass properties from a source component.
 */
template <typename Scalar> struct MassSourceBundle {
    std::string name;
    SignalHandle<Scalar> mass;
    SignalHandle<Scalar> cg_x, cg_y, cg_z;
    SignalHandle<Scalar> inertia_xx, inertia_yy, inertia_zz;
    SignalHandle<Scalar> inertia_xy, inertia_xz, inertia_yz;

    /**
     * @brief Read handles into MassProperties
     */
    vulcan::mass::MassProperties<Scalar> Get() const {
        Mat3<Scalar> I;
        I(0, 0) = *inertia_xx;
        I(1, 1) = *inertia_yy;
        I(2, 2) = *inertia_zz;
        I(0, 1) = I(1, 0) = *inertia_xy;
        I(0, 2) = I(2, 0) = *inertia_xz;
        I(1, 2) = I(2, 1) = *inertia_yz;

        return vulcan::mass::MassProperties<Scalar>{
            .mass = *mass, .cg = Vec3<Scalar>{*cg_x, *cg_y, *cg_z}, .inertia = I};
    }
};

/**
 * @brief Force source handle bundle for aggregation
 *
 * Holds SignalHandles to read force/moment from a source component.
 */
template <typename Scalar> struct ForceSourceBundle {
    std::string name;
    SignalHandle<Scalar> force_x, force_y, force_z;
    SignalHandle<Scalar> moment_x, moment_y, moment_z;
    SignalHandle<Scalar> app_point_x, app_point_y, app_point_z;
    bool has_moment{false};
    bool has_app_point{false};
    bool is_ecef{false}; ///< True if force is in ECEF frame (needs transform)

    Vec3<Scalar> GetForce() const { return Vec3<Scalar>{*force_x, *force_y, *force_z}; }

    Vec3<Scalar> GetMoment() const {
        if (!has_moment) {
            return Vec3<Scalar>::Zero();
        }
        return Vec3<Scalar>{*moment_x, *moment_y, *moment_z};
    }

    Vec3<Scalar> GetApplicationPoint() const {
        if (!has_app_point) {
            return Vec3<Scalar>::Zero();
        }
        return Vec3<Scalar>{*app_point_x, *app_point_y, *app_point_z};
    }
};

/**
 * @brief Unified 6DOF vehicle dynamics with mass/force aggregation
 *
 * Vehicle6DOF combines:
 * - Mass property aggregation (from MassAggregator)
 * - Force/moment aggregation (from ForceAggregator)
 * - 13-state quaternion rigid body dynamics (from RigidBody6DOF)
 *
 * ## Configuration
 *
 * ```yaml
 * - type: Vehicle6DOF
 *   name: Vehicle
 *   entity: Rocket
 *
 *   # Mass sources (component names in same entity)
 *   sources: [Structure, FuelTank]
 *
 *   # Force sources by frame
 *   body_sources: [Engine]           # Forces in body frame
 *   ecef_sources: [Gravity]          # Forces in ECEF frame
 *
 *   # Initial conditions
 *   vectors:
 *     initial_lla: [lat_deg, lon_deg, alt_m]
 *     initial_euler_zyx: [yaw_deg, pitch_deg, roll_deg]
 *     initial_velocity_body: [vx, vy, vz]
 *     initial_omega_body: [wx, wy, wz]
 *
 *   # Reference frame
 *   strings:
 *     reference_frame: "ecef"       # "eci" or "ecef"
 *
 *   # Default mass (if no sources specified)
 *   scalars:
 *     default_mass: 1.0
 * ```
 *
 * ## Outputs
 *
 * **States:**
 * - `position.x/y/z` [m] — Position in reference frame
 * - `velocity_body.x/y/z` [m/s] — Velocity in body frame
 * - `attitude.w/x/y/z` — Quaternion (body-to-reference)
 * - `omega_body.x/y/z` [rad/s] — Angular velocity in body frame
 *
 * **Derived:**
 * - `velocity_ref.x/y/z` [m/s] — Velocity in reference frame
 * - `position_lla.lat/lon/alt` — Geodetic coordinates
 * - `euler_zyx.yaw/pitch/roll` [rad] — Euler angles
 *
 * **Aggregated:**
 * - `total_mass` [kg]
 * - `cg.x/y/z` [m]
 * - `inertia.xx/yy/zz/xy/xz/yz` [kg*m^2]
 * - `total_force.x/y/z` [N]
 * - `total_moment.x/y/z` [N*m]
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class Vehicle6DOF : public Component<Scalar> {
  public:
    explicit Vehicle6DOF(std::string name = "Vehicle6DOF", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "Vehicle6DOF"; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register states and outputs
     *
     * States: 13-state quaternion 6DOF (position, velocity_body, attitude, omega_body)
     * Outputs: aggregated mass/force, derived quantities
     */
    void Provision(Backplane<Scalar> &bp) override {
        // === States (13 total) ===
        bp.template register_state_vec3<Scalar>("position", &position_, &position_dot_, "m",
                                                "Position in reference frame");

        bp.template register_state_vec3<Scalar>(
            "velocity_body", &velocity_body_, &velocity_body_dot_, "m/s", "Velocity in body frame");

        bp.template register_state_quat<Scalar>("attitude", &attitude_, &attitude_dot_, "",
                                                "Body-to-reference quaternion");

        bp.template register_state_vec3<Scalar>("omega_body", &omega_body_, &omega_body_dot_,
                                                "rad/s", "Angular velocity in body frame");

        // === Aggregated Mass Outputs ===
        bp.template register_output<Scalar>("total_mass", &total_mass_, "kg", "Total mass");
        bp.template register_output_vec3<Scalar>("cg", &cg_, "m", "Combined CG position");
        bp.template register_output<Scalar>("inertia.xx", &inertia_xx_, "kg*m^2", "Inertia Ixx");
        bp.template register_output<Scalar>("inertia.yy", &inertia_yy_, "kg*m^2", "Inertia Iyy");
        bp.template register_output<Scalar>("inertia.zz", &inertia_zz_, "kg*m^2", "Inertia Izz");
        bp.template register_output<Scalar>("inertia.xy", &inertia_xy_, "kg*m^2", "Inertia Ixy");
        bp.template register_output<Scalar>("inertia.xz", &inertia_xz_, "kg*m^2", "Inertia Ixz");
        bp.template register_output<Scalar>("inertia.yz", &inertia_yz_, "kg*m^2", "Inertia Iyz");

        // === Aggregated Force Outputs ===
        bp.template register_output_vec3<Scalar>("total_force", &total_force_, "N",
                                                 "Total force in body frame");
        bp.template register_output_vec3<Scalar>("total_moment", &total_moment_, "N*m",
                                                 "Total moment about CG in body frame");

        // === Derived Outputs ===
        bp.template register_output_vec3<Scalar>("velocity_ref", &velocity_ref_, "m/s",
                                                 "Velocity in reference frame");

        bp.template register_output<Scalar>("position_lla.lat", &position_lla_(0), "rad",
                                            "Geodetic latitude");
        bp.template register_output<Scalar>("position_lla.lon", &position_lla_(1), "rad",
                                            "Longitude");
        bp.template register_output<Scalar>("position_lla.alt", &position_lla_(2), "m",
                                            "Altitude above ellipsoid");

        bp.template register_output<Scalar>("velocity_ned.n", &velocity_ned_(0), "m/s",
                                            "North velocity");
        bp.template register_output<Scalar>("velocity_ned.e", &velocity_ned_(1), "m/s",
                                            "East velocity");
        bp.template register_output<Scalar>("velocity_ned.d", &velocity_ned_(2), "m/s",
                                            "Down velocity");

        bp.template register_output<Scalar>("euler_zyx.yaw", &euler_zyx_(0), "rad", "Yaw angle");
        bp.template register_output<Scalar>("euler_zyx.pitch", &euler_zyx_(1), "rad",
                                            "Pitch angle");
        bp.template register_output<Scalar>("euler_zyx.roll", &euler_zyx_(2), "rad", "Roll angle");
    }

    /**
     * @brief Resolve source handles and apply initial conditions
     */
    void Stage(Backplane<Scalar> &bp) override {
        const auto &config = this->GetConfig();

        // Read reference frame setting
        reference_frame_ = this->template read_param<std::string>("reference_frame", "eci");

        // Read default mass for when no sources are specified
        // Only override if explicitly configured
        if (config.template Has<double>("default_mass")) {
            default_mass_ = Scalar(this->template read_param<double>("default_mass", 1.0));
        }
        auto default_inertia_vec =
            config.template Get<Vec3<double>>("default_inertia", Vec3<double>{1.0, 1.0, 1.0});
        default_inertia_diag_ = Vec3<Scalar>{static_cast<Scalar>(default_inertia_vec(0)),
                                             static_cast<Scalar>(default_inertia_vec(1)),
                                             static_cast<Scalar>(default_inertia_vec(2))};

        // === Resolve Mass Source Handles ===
        // Uses 'sources' field from config, or programmatic mass_source_names_
        auto mass_names = !config.sources.empty() ? config.sources : mass_source_names_;
        if (!mass_names.empty()) {
            ResolveMassSources(bp, mass_names);
        }

        // === Resolve Force Source Handles ===
        // Uses config lists, or programmatic lists
        auto body_names = !config.body_sources.empty() ? config.body_sources : body_source_names_;
        auto ecef_names = !config.ecef_sources.empty() ? config.ecef_sources : ecef_source_names_;

        if (!body_names.empty()) {
            ResolveForceSources(bp, body_names, /*is_ecef=*/false);
        }
        if (!ecef_names.empty()) {
            ResolveForceSources(bp, ecef_names, /*is_ecef=*/true);
        }

        // === Apply Initial Conditions ===
        ApplyInitialConditions(config);

        // Compute initial derived outputs
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
     * @brief Aggregate mass/forces and compute 6DOF dynamics
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // === Mass Aggregation ===
        AggregateMass();

        // === Force Aggregation ===
        AggregateForces();

        // === 6DOF Dynamics ===
        ComputeDynamics();

        // === Derived Outputs ===
        ComputeDerivedOutputs();
    }

    // =========================================================================
    // Programmatic Configuration
    // =========================================================================

    void AddMassSource(const std::string &source_name) {
        mass_source_names_.push_back(source_name);
    }
    void AddBodySource(const std::string &source_name) {
        body_source_names_.push_back(source_name);
    }
    void AddEcefSource(const std::string &source_name) {
        ecef_source_names_.push_back(source_name);
    }

    void SetInitialPosition(const Vec3<Scalar> &pos) { position_ = pos; }
    void SetInitialVelocityBody(const Vec3<Scalar> &vel) { velocity_body_ = vel; }
    void SetInitialAttitude(const janus::Quaternion<Scalar> &q) {
        attitude_ = Vec4<Scalar>{q.w, q.x, q.y, q.z};
    }
    void SetInitialOmegaBody(const Vec3<Scalar> &omega) { omega_body_ = omega; }

    void SetInitialPositionLLA(double lat_deg, double lon_deg, double alt_m) {
        vulcan::LLA<double> lla;
        lla.lat = lat_deg * vulcan::constants::angle::deg2rad;
        lla.lon = lon_deg * vulcan::constants::angle::deg2rad;
        lla.alt = alt_m;
        Vec3<double> r_ecef = vulcan::lla_to_ecef(lla);
        position_ = Vec3<Scalar>{static_cast<Scalar>(r_ecef(0)), static_cast<Scalar>(r_ecef(1)),
                                 static_cast<Scalar>(r_ecef(2))};
    }

    void SetInitialAttitudeEuler(Scalar yaw_deg, Scalar pitch_deg, Scalar roll_deg) {
        Scalar yaw = yaw_deg * Scalar(vulcan::constants::angle::deg2rad);
        Scalar pitch = pitch_deg * Scalar(vulcan::constants::angle::deg2rad);
        Scalar roll = roll_deg * Scalar(vulcan::constants::angle::deg2rad);

        // Use position_ directly (already Vec3<Scalar>)
        auto ned = vulcan::local_ned_at(position_);
        auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

        // Extract quaternion from body frame relative to NED, then convert to ECEF
        // body_from_euler returns body frame in ECEF, so we build DCM from its axes
        Mat3<Scalar> dcm;
        dcm.col(0) = body.x_axis;
        dcm.col(1) = body.y_axis;
        dcm.col(2) = body.z_axis;
        auto q_body_to_ecef = janus::Quaternion<Scalar>::from_rotation_matrix(dcm);

        attitude_ =
            Vec4<Scalar>{q_body_to_ecef.w, q_body_to_ecef.x, q_body_to_ecef.y, q_body_to_ecef.z};
    }

    void SetDefaultMass(Scalar mass) { default_mass_ = mass; }
    void SetDefaultInertia(const Vec3<Scalar> &diag) { default_inertia_diag_ = diag; }

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

    [[nodiscard]] Scalar GetTotalMass() const { return total_mass_; }
    [[nodiscard]] Vec3<Scalar> GetCG() const { return cg_; }
    [[nodiscard]] Vec3<Scalar> GetTotalForce() const { return total_force_; }
    [[nodiscard]] Vec3<Scalar> GetTotalMoment() const { return total_moment_; }

    [[nodiscard]] Vec3<Scalar> GetPositionLLA() const { return position_lla_; }
    [[nodiscard]] Vec3<Scalar> GetVelocityNED() const { return velocity_ned_; }
    [[nodiscard]] Vec3<Scalar> GetEulerZYX() const { return euler_zyx_; }

  private:
    // =========================================================================
    // Handle Resolution
    // =========================================================================

    void ResolveMassSources(Backplane<Scalar> &bp, const std::vector<std::string> &names) {
        std::string prefix = entity_.empty() ? "" : entity_ + ".";

        for (const auto &name : names) {
            MassSourceBundle<Scalar> src;
            src.name = name;
            std::string base = prefix + name + ".";

            src.mass = bp.template resolve<Scalar>(base + "mass");
            src.cg_x = bp.template resolve<Scalar>(base + "cg.x");
            src.cg_y = bp.template resolve<Scalar>(base + "cg.y");
            src.cg_z = bp.template resolve<Scalar>(base + "cg.z");
            src.inertia_xx = bp.template resolve<Scalar>(base + "inertia.xx");
            src.inertia_yy = bp.template resolve<Scalar>(base + "inertia.yy");
            src.inertia_zz = bp.template resolve<Scalar>(base + "inertia.zz");
            src.inertia_xy = bp.template resolve<Scalar>(base + "inertia.xy");
            src.inertia_xz = bp.template resolve<Scalar>(base + "inertia.xz");
            src.inertia_yz = bp.template resolve<Scalar>(base + "inertia.yz");

            mass_sources_.push_back(std::move(src));
        }
    }

    void ResolveForceSources(Backplane<Scalar> &bp, const std::vector<std::string> &names,
                             bool is_ecef) {
        std::string prefix = entity_.empty() ? "" : entity_ + ".";

        for (const auto &name : names) {
            ForceSourceBundle<Scalar> src;
            src.name = name;
            src.is_ecef = is_ecef;
            std::string base = prefix + name + ".";

            // Force vector (required)
            src.force_x = bp.template resolve<Scalar>(base + "force.x");
            src.force_y = bp.template resolve<Scalar>(base + "force.y");
            src.force_z = bp.template resolve<Scalar>(base + "force.z");

            // Moment (optional)
            if (bp.has_signal(base + "moment.x")) {
                src.moment_x = bp.template resolve<Scalar>(base + "moment.x");
                src.moment_y = bp.template resolve<Scalar>(base + "moment.y");
                src.moment_z = bp.template resolve<Scalar>(base + "moment.z");
                src.has_moment = true;
            }

            // Application point (optional)
            if (bp.has_signal(base + "application_point.x")) {
                src.app_point_x = bp.template resolve<Scalar>(base + "application_point.x");
                src.app_point_y = bp.template resolve<Scalar>(base + "application_point.y");
                src.app_point_z = bp.template resolve<Scalar>(base + "application_point.z");
                src.has_app_point = true;
            }

            force_sources_.push_back(std::move(src));
        }
    }

    // =========================================================================
    // Initial Conditions
    // =========================================================================

    void ApplyInitialConditions(const ComponentConfig &config) {
        // Track position as double for coordinate transforms
        Vec3<double> r_ecef_double = Vec3<double>::Zero();

        // Position initialization
        auto pos_mode = this->template read_param<std::string>("position_init_mode", "ecef");
        if (pos_mode == "lla") {
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
            // Read position from config/programmatic value
            position_ = this->read_param_vec3("initial_position", position_);
            // Convert position_ to double for coordinate transforms (local_ned_at, etc.)
            r_ecef_double =
                Vec3<double>{static_cast<double>(position_(0)), static_cast<double>(position_(1)),
                             static_cast<double>(position_(2))};
        }

        // Velocity and omega
        velocity_body_ = this->read_param_vec3("initial_velocity_body", velocity_body_);
        omega_body_ = this->read_param_vec3("initial_omega_body", omega_body_);

        // Attitude initialization
        auto att_mode = this->template read_param<std::string>("attitude_init_mode", "quaternion");
        if (att_mode == "euler_zyx") {
            auto euler_deg =
                config.template Get<Vec3<double>>("initial_euler_zyx", Vec3<double>::Zero());

            double yaw = euler_deg(0) * vulcan::constants::angle::deg2rad;
            double pitch = euler_deg(1) * vulcan::constants::angle::deg2rad;
            double roll = euler_deg(2) * vulcan::constants::angle::deg2rad;

            auto ned = vulcan::local_ned_at(r_ecef_double);
            auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

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
    }

    // =========================================================================
    // Aggregation Methods
    // =========================================================================

    void AggregateMass() {
        if (mass_sources_.empty()) {
            // Use defaults
            total_mass_ = default_mass_;
            cg_ = Vec3<Scalar>::Zero();
            inertia_xx_ = default_inertia_diag_(0);
            inertia_yy_ = default_inertia_diag_(1);
            inertia_zz_ = default_inertia_diag_(2);
            inertia_xy_ = inertia_xz_ = inertia_yz_ = Scalar(0);
            return;
        }

        // Aggregate using Vulcan's MassProperties
        vulcan::mass::MassProperties<Scalar> total = mass_sources_[0].Get();
        for (size_t i = 1; i < mass_sources_.size(); ++i) {
            total = total + mass_sources_[i].Get();
        }

        total_mass_ = total.mass;
        cg_ = total.cg;
        inertia_xx_ = total.inertia(0, 0);
        inertia_yy_ = total.inertia(1, 1);
        inertia_zz_ = total.inertia(2, 2);
        inertia_xy_ = total.inertia(0, 1);
        inertia_xz_ = total.inertia(0, 2);
        inertia_yz_ = total.inertia(1, 2);
    }

    void AggregateForces() {
        Vec3<Scalar> sum_force = Vec3<Scalar>::Zero();
        Vec3<Scalar> sum_moment = Vec3<Scalar>::Zero();

        // Get attitude for ECEF→body transformation
        janus::Quaternion<Scalar> q_body_to_ref{attitude_(0), attitude_(1), attitude_(2),
                                                attitude_(3)};

        for (const auto &src : force_sources_) {
            Vec3<Scalar> F = src.GetForce();
            Vec3<Scalar> M = src.GetMoment();

            // Transform ECEF forces to body frame
            if (src.is_ecef) {
                // ECEF sources require ECEF reference frame - the attitude quaternion
                // is body-to-ref, so we can only directly transform ECEF→body when
                // ref == ECEF. Supporting ECI would require ECEF→ECI rotation at
                // current epoch, which adds complexity.
                if (reference_frame_ != "ecef") {
                    throw ConfigError(
                        "Vehicle6DOF: ECEF force source '" + src.name +
                        "' requires reference_frame=\"ecef\", but reference_frame=\"" +
                        reference_frame_ + "\". Use body-frame forces for ECI dynamics.");
                }
                F = q_body_to_ref.conjugate().rotate(F);
                // Note: moments from ECEF sources are assumed to already be in body frame
                // or zero (most ECEF forces like gravity don't produce moments directly)
            }

            // Sum forces
            sum_force += F;

            // Add direct moment from source
            sum_moment += M;

            // Transfer moment to CG only if source has an application point
            // Forces without an application point act at the CG (no moment transfer)
            if (src.has_app_point) {
                Vec3<Scalar> app_point = src.GetApplicationPoint();
                Vec3<Scalar> r = app_point - cg_;
                sum_moment += r.cross(F);
            }
        }

        total_force_ = sum_force;
        total_moment_ = sum_moment;
    }

    // =========================================================================
    // Dynamics Methods
    // =========================================================================

    void ComputeDynamics() {
        // Build quaternion from state
        janus::Quaternion<Scalar> quat{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

        // Build inertia tensor
        Mat3<Scalar> inertia;
        inertia(0, 0) = inertia_xx_;
        inertia(1, 1) = inertia_yy_;
        inertia(2, 2) = inertia_zz_;
        inertia(0, 1) = inertia(1, 0) = inertia_xy_;
        inertia(0, 2) = inertia(2, 0) = inertia_xz_;
        inertia(1, 2) = inertia(2, 1) = inertia_yz_;

        // Build mass properties
        vulcan::mass::MassProperties<Scalar> mass_props;
        mass_props.mass = total_mass_;
        mass_props.cg = Vec3<Scalar>::Zero(); // Forces already about CG
        mass_props.inertia = inertia;

        // Build Vulcan state struct
        vulcan::dynamics::RigidBodyState<Scalar> rb_state;
        rb_state.position = position_;
        rb_state.velocity_body = velocity_body_;
        rb_state.attitude = quat;
        rb_state.omega_body = omega_body_;

        if (reference_frame_ == "ecef") {
            // ECEF mode: Include Coriolis and centrifugal terms
            Vec3<Scalar> omega_earth{Scalar(0), Scalar(0), Scalar(vulcan::constants::earth::omega)};

            Vec3<Scalar> velocity_ecef = quat.rotate(velocity_body_);
            Vec3<Scalar> force_ecef = quat.rotate(total_force_);

            Vec3<Scalar> accel_ecef = vulcan::dynamics::translational_dynamics_ecef(
                position_, velocity_ecef, force_ecef, mass_props.mass, omega_earth);

            velocity_body_dot_ = quat.conjugate().rotate(accel_ecef);
            position_dot_ = velocity_ecef;

            auto q_dot = vulcan::quaternion_rate_from_omega(quat, omega_body_);
            attitude_dot_ = Vec4<Scalar>{q_dot.w, q_dot.x, q_dot.y, q_dot.z};
            omega_body_dot_ = vulcan::dynamics::rotational_dynamics(omega_body_, total_moment_,
                                                                    mass_props.inertia);
        } else {
            // ECI mode (default): Standard inertial equations
            auto derivs = vulcan::dynamics::compute_6dof_derivatives(rb_state, total_force_,
                                                                     total_moment_, mass_props);

            position_dot_ = derivs.position_dot;
            velocity_body_dot_ = derivs.velocity_dot;
            attitude_dot_ = Vec4<Scalar>{derivs.attitude_dot.w, derivs.attitude_dot.x,
                                         derivs.attitude_dot.y, derivs.attitude_dot.z};
            omega_body_dot_ = derivs.omega_dot;
        }
    }

    void ComputeDerivedOutputs() {
        janus::Quaternion<Scalar> quat{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};
        velocity_ref_ = quat.rotate(velocity_body_);
        ComputeCoordinateFrameOutputs();
    }

    void ComputeCoordinateFrameOutputs() {
        // LLA/NED/Euler outputs require ECEF position and velocity.
        // In ECI mode, position_ and velocity_ref_ are in ECI frame, so these
        // outputs are not valid without an ECI→ECEF transform (which requires
        // simulation time for GMST calculation).
        if (reference_frame_ != "ecef") {
            // Set outputs to zero - they are not meaningful in ECI mode.
            // Users should not rely on position_lla, velocity_ned, or euler_zyx
            // when reference_frame="eci". Use position and velocity_ref directly.
            position_lla_ = Vec3<Scalar>::Zero();
            velocity_ned_ = Vec3<Scalar>::Zero();
            euler_zyx_ = Vec3<Scalar>::Zero();
            return;
        }

        // ECEF mode: position_ and velocity_ref_ are in ECEF frame

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
        janus::Quaternion<Scalar> quat{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

        Vec3<Scalar> x_body_ecef = quat.rotate(Vec3<Scalar>::UnitX());
        Vec3<Scalar> y_body_ecef = quat.rotate(Vec3<Scalar>::UnitY());
        Vec3<Scalar> z_body_ecef = quat.rotate(Vec3<Scalar>::UnitZ());

        vulcan::CoordinateFrame<Scalar> body_frame(x_body_ecef, y_body_ecef, z_body_ecef,
                                                   Vec3<Scalar>::Zero());

        euler_zyx_ = vulcan::euler_from_body(body_frame, ned);
    }

    // =========================================================================
    // Member Variables
    // =========================================================================

    // Identity
    std::string name_;
    std::string entity_;

    // Configuration
    std::string reference_frame_ = "eci";
    Scalar default_mass_ = Scalar(1);
    Vec3<Scalar> default_inertia_diag_ = Vec3<Scalar>{Scalar(1), Scalar(1), Scalar(1)};

    // Programmatic source names (used if config lists are empty)
    std::vector<std::string> mass_source_names_;
    std::vector<std::string> body_source_names_;
    std::vector<std::string> ecef_source_names_;

    // Resolved source handles
    std::vector<MassSourceBundle<Scalar>> mass_sources_;
    std::vector<ForceSourceBundle<Scalar>> force_sources_;

    // === 13-State Variables ===
    Vec3<Scalar> position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> position_dot_ = Vec3<Scalar>::Zero();

    Vec3<Scalar> velocity_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_body_dot_ = Vec3<Scalar>::Zero();

    Vec4<Scalar> attitude_ =
        (Vec4<Scalar>() << Scalar(1), Scalar(0), Scalar(0), Scalar(0)).finished();
    Vec4<Scalar> attitude_dot_ = Vec4<Scalar>::Zero();

    Vec3<Scalar> omega_body_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> omega_body_dot_ = Vec3<Scalar>::Zero();

    // === Aggregated Mass Properties ===
    Scalar total_mass_{1};
    Vec3<Scalar> cg_ = Vec3<Scalar>::Zero();
    Scalar inertia_xx_{1}, inertia_yy_{1}, inertia_zz_{1};
    Scalar inertia_xy_{0}, inertia_xz_{0}, inertia_yz_{0};

    // === Aggregated Forces ===
    Vec3<Scalar> total_force_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> total_moment_ = Vec3<Scalar>::Zero();

    // === Derived Outputs ===
    Vec3<Scalar> velocity_ref_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> position_lla_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_ned_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> euler_zyx_ = Vec3<Scalar>::Zero();
};

// Register with factory
ICARUS_REGISTER_COMPONENT_AS(Vehicle6DOF, "Vehicle6DOF")

} // namespace components
} // namespace icarus
