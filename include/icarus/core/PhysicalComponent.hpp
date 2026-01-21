#pragma once

/**
 * @file PhysicalComponent.hpp
 * @brief Intermediate class for components with physical body attachment
 *
 * Part of Phase 9A: PhysicalComponent Class Implementation.
 * Components that have a physical presence on a vehicle (mass sources,
 * force sources, sensors, actuators) should inherit from this class.
 */

#include "icarus/core/Component.hpp"
#include "janus/math/Quaternion.hpp"
#include <cmath>

namespace icarus {

/**
 * @brief Intermediate class for components with physical body attachment.
 *
 * Components that have a physical presence on the vehicle (mass sources,
 * force sources, sensors, actuators) should inherit from this class.
 *
 * Provides:
 * - Body position (mounting location in vehicle body frame)
 * - Body orientation (rotation from body frame to component local frame)
 * - Config parsing helper for both quaternion and Euler angle formats
 *
 * Non-physical components (environment models, schedulers, etc.) should
 * inherit directly from Component<Scalar>.
 *
 * ## Configuration Format
 *
 * Subclasses call ReadAttachmentFromConfig() in Stage() to parse:
 *
 * ```yaml
 * body_position: [x, y, z]                    # meters, in body frame
 *
 * # Option 1: Quaternion (w, x, y, z)
 * body_orientation: [1.0, 0.0, 0.0, 0.0]     # identity rotation
 *
 * # Option 2: Euler angles (ZYX order, degrees)
 * body_orientation_euler_zyx: [yaw, pitch, roll]
 * ```
 *
 * ## Orientation Convention
 *
 * `body_orientation` represents the rotation FROM body frame TO component
 * local frame (body-to-component).
 *
 * **CAD Interpretation**: The Euler angles describe "how I rotated this
 * component starting from alignment with body axes."
 *
 * **Frame Transformation**:
 * - To transform component outputs TO body frame:
 *   `F_body = body_orientation.conjugate().rotate(F_local)`
 * - To transform body vectors TO component frame (e.g., sensors):
 *   `v_local = body_orientation.rotate(v_body)`
 *
 * ## Example
 *
 * A rocket engine with thrust along its local +X axis, mounted at body
 * position [0, 0, 10] pointing along body -Z:
 *
 * ```yaml
 * - type: RocketEngine
 *   name: MainEngine
 *   body_position: [0.0, 0.0, 10.0]           # nozzle at z=10m
 *   body_orientation_euler_zyx: [0.0, -90.0, 0.0]  # pitch -90 deg -> +X -> -Z
 * ```
 *
 * The engine outputs `force_local = [thrust, 0, 0]`. Vehicle6DOF transforms:
 * `F_body = R.conjugate().rotate([thrust, 0, 0]) = [0, 0, -thrust]`
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class PhysicalComponent : public Component<Scalar> {
  protected:
    /// Mounting position in vehicle body frame [m]
    Vec3<Scalar> body_position_ = Vec3<Scalar>::Zero();

    /// Rotation from body frame to component local frame (identity by default)
    janus::Quaternion<Scalar> body_orientation_;

    /// True after ReadAttachmentFromConfig() is called
    bool has_body_attachment_ = false;

    /**
     * @brief Read body attachment from component config.
     *
     * Call this in Stage() to parse body_position and body_orientation
     * from the component's YAML configuration.
     *
     * Supports two orientation formats:
     * - `body_orientation: [w, x, y, z]` - quaternion
     * - `body_orientation_euler_zyx: [yaw, pitch, roll]` - Euler angles (degrees)
     *
     * If neither orientation is specified, defaults to identity (no rotation).
     * If body_position is not specified, defaults to origin [0, 0, 0].
     *
     * **Convention Note:**
     * The YAML config uses `body_orientation_euler_zyx: [yaw, pitch, roll]` (degrees,
     * aerospace order), but Janus `from_euler()` expects `(roll, pitch, yaw)` (radians).
     * This method handles the conversion.
     *
     * @pre GetConfig() returns valid ComponentConfig
     * @post has_body_attachment_ is true
     */
    void ReadAttachmentFromConfig() {
        const auto &config = this->GetConfig();

        // Read position (default to origin)
        body_position_ = this->read_param_vec3("body_position", Vec3<Scalar>::Zero());

        // Check for Euler angles first (more user-friendly format)
        if (config.template Has<std::vector<double>>("body_orientation_euler_zyx")) {
            auto euler_deg = config.template Get<std::vector<double>>("body_orientation_euler_zyx",
                                                                      {0.0, 0.0, 0.0});

            if (euler_deg.size() != 3) {
                throw ConfigError(
                    "body_orientation_euler_zyx must have 3 elements [yaw, pitch, roll]");
            }

            // Convert degrees to radians
            constexpr double deg2rad = M_PI / 180.0;
            double yaw_rad = euler_deg[0] * deg2rad;
            double pitch_rad = euler_deg[1] * deg2rad;
            double roll_rad = euler_deg[2] * deg2rad;

            // Build quaternion from ZYX Euler angles
            // Note: Janus from_euler() takes (roll, pitch, yaw) order for ZYX sequence
            body_orientation_ = janus::Quaternion<Scalar>::from_euler(
                static_cast<Scalar>(roll_rad), static_cast<Scalar>(pitch_rad),
                static_cast<Scalar>(yaw_rad));
        } else if (config.template Has<std::vector<double>>("body_orientation")) {
            // Quaternion format: [w, x, y, z]
            auto q =
                config.template Get<std::vector<double>>("body_orientation", {1.0, 0.0, 0.0, 0.0});

            if (q.size() != 4) {
                throw ConfigError("body_orientation must have 4 elements [w, x, y, z]");
            }

            body_orientation_ = janus::Quaternion<Scalar>{
                static_cast<Scalar>(q[0]), // w
                static_cast<Scalar>(q[1]), // x
                static_cast<Scalar>(q[2]), // y
                static_cast<Scalar>(q[3])  // z
            };

            // Normalize to ensure unit quaternion
            body_orientation_ = body_orientation_.normalized();
        }
        // else: keep default identity orientation

        has_body_attachment_ = true;
    }

    /**
     * @brief Set body attachment programmatically (for testing or dynamic mounting).
     *
     * @param position Position in vehicle body frame [m]
     * @param orientation Rotation from body to component frame
     */
    void SetBodyAttachment(const Vec3<Scalar> &position,
                           const janus::Quaternion<Scalar> &orientation) {
        body_position_ = position;
        body_orientation_ = orientation;
        has_body_attachment_ = true;
    }

    /**
     * @brief Set body position only (orientation defaults to identity).
     * @param position Position in vehicle body frame [m]
     */
    void SetBodyPosition(const Vec3<Scalar> &position) {
        body_position_ = position;
        has_body_attachment_ = true;
    }

  public:
    // ===== Override Base Class Virtual Methods =====

    [[nodiscard]] bool HasBodyAttachment() const override { return has_body_attachment_; }

    [[nodiscard]] Vec3<Scalar> GetBodyPosition() const override { return body_position_; }

    [[nodiscard]] janus::Quaternion<Scalar> GetBodyOrientation() const override {
        return body_orientation_;
    }

    // ===== Additional Accessors =====

    /**
     * @brief Get rotation from component frame to body frame.
     *
     * This is the inverse of GetBodyOrientation() and is commonly needed
     * to transform component outputs (forces, moments) to body frame.
     *
     * @return Quaternion that transforms component vectors to body frame
     */
    [[nodiscard]] janus::Quaternion<Scalar> GetComponentToBodyRotation() const {
        return body_orientation_.conjugate();
    }

    /**
     * @brief Transform a vector from component local frame to body frame.
     * @param v_local Vector in component local frame
     * @return Vector in body frame
     */
    [[nodiscard]] Vec3<Scalar> TransformToBodyFrame(const Vec3<Scalar> &v_local) const {
        return body_orientation_.conjugate().rotate(v_local);
    }

    /**
     * @brief Transform a vector from body frame to component local frame.
     * @param v_body Vector in body frame
     * @return Vector in component local frame
     */
    [[nodiscard]] Vec3<Scalar> TransformToLocalFrame(const Vec3<Scalar> &v_body) const {
        return body_orientation_.rotate(v_body);
    }
};

} // namespace icarus
