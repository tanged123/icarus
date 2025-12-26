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
#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/dynamics/PointMass.hpp>

namespace icarus {
namespace components {

/**
 * @brief 3-DOF point mass translational dynamics (inertial frame)
 *
 * State vector: [rx, ry, rz, vx, vy, vz] (6 variables)
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
    static constexpr std::size_t kStateSize = 6;
    static constexpr std::size_t kPosOffset = 0;
    static constexpr std::size_t kVelOffset = 3;

    /**
     * @brief Construct with mass and optional name/entity
     */
    explicit PointMass3DOF(Scalar mass = Scalar{1.0}, std::string name = "PointMass3DOF",
                           std::string entity = "")
        : mass_(mass), name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "PointMass3DOF"; }

    [[nodiscard]] std::size_t StateSize() const override { return kStateSize; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register outputs, inputs, and parameters
     */
    void Provision(Backplane<Scalar> &bp, const ComponentConfig &) override {
        // === Outputs ===
        // Position (in inertial frame: ECI for orbital, local for validation)
        bp.template register_output_vec3<Scalar>("position", &position_, "m", "Position");

        // Velocity (in same inertial frame as position)
        bp.template register_output_vec3<Scalar>("velocity", &velocity_, "m/s", "Velocity");

        // === Inputs (Phase 2.4) ===
        // Force input (from gravity and other force sources)
        bp.template register_input<Scalar>("force.x", &force_x_, "N", "Applied force X");
        bp.template register_input<Scalar>("force.y", &force_y_, "N", "Applied force Y");
        bp.template register_input<Scalar>("force.z", &force_z_, "N", "Applied force Z");

        // === Parameters (Phase 2.4) ===
        // Mass is a tunable parameter AND published as output for other components to read
        bp.register_param("mass", &mass_, mass_, "kg", "Point mass");
        bp.template register_output<Scalar>("mass", &mass_, "kg", "Point mass");
    }

    /**
     * @brief Stage phase - resolve dependencies
     *
     * Note: Wiring is defined externally (simulator config) and applied here.
     */
    void Stage(Backplane<Scalar> &bp, const ComponentConfig &cfg) override {
        // Apply wiring from configuration
        for (const auto &[input, source] : cfg.wiring) {
            bp.template wire_input<Scalar>(input, source);
        }
    }

    /**
     * @brief Bind to global state vector slices
     */
    void BindState(Scalar *state, Scalar *state_dot, std::size_t size) override {
        if (size != kStateSize) {
            throw StateSizeMismatchError(kStateSize, size);
        }

        // Store pointers to state slices
        state_pos_ = state + kPosOffset;
        state_dot_pos_ = state_dot + kPosOffset;
        state_vel_ = state + kVelOffset;
        state_dot_vel_ = state_dot + kVelOffset;

        // Apply initial conditions to state
        state_pos_[0] = ic_position_(0);
        state_pos_[1] = ic_position_(1);
        state_pos_[2] = ic_position_(2);
        state_vel_[0] = ic_velocity_(0);
        state_vel_[1] = ic_velocity_(1);
        state_vel_[2] = ic_velocity_(2);

        // CRITICAL: Also initialize output signals so other components
        // can read correct values during first derivative evaluation
        position_ = ic_position_;
        velocity_ = ic_velocity_;
    }

    /**
     * @brief Publish state outputs BEFORE other components' Step() runs
     *
     * This is critical for correct signal propagation: Gravity needs to read
     * the current position to compute force, but Dynamics reads force in its
     * Step(). By publishing position in PreStep, we ensure the data flow:
     *   PreStep: Dynamics publishes position
     *   Step: Gravity reads position, computes force, publishes force
     *   Step: Dynamics reads force, computes acceleration
     */
    void PreStep(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Publish current state to outputs BEFORE other components read them
        Vec3<Scalar> pos{state_pos_[0], state_pos_[1], state_pos_[2]};
        Vec3<Scalar> vel{state_vel_[0], state_vel_[1], state_vel_[2]};
        position_ = pos;
        velocity_ = vel;
    }

    /**
     * @brief Compute derivatives using Vulcan dynamics
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read current state into Vec3
        Vec3<Scalar> pos{state_pos_[0], state_pos_[1], state_pos_[2]};
        Vec3<Scalar> vel{state_vel_[0], state_vel_[1], state_vel_[2]};

        // CRITICAL: Update outputs FIRST so other components (e.g., Gravity)
        // can read the current trial position during RK4 intermediate stages
        position_ = pos;
        velocity_ = vel;

        // Read force input from registered InputHandles
        Vec3<Scalar> force{force_x_.get(), force_y_.get(), force_z_.get()};

        // Compute acceleration using Vulcan (inertial frame: a = F/m)
        Vec3<Scalar> accel = vulcan::dynamics::point_mass_acceleration(force, mass_);

        // Write derivatives
        // dr/dt = v (kinematics)
        state_dot_pos_[0] = vel(0);
        state_dot_pos_[1] = vel(1);
        state_dot_pos_[2] = vel(2);

        // dv/dt = a = F/m (Vulcan computes this)
        state_dot_vel_[0] = accel(0);
        state_dot_vel_[1] = accel(1);
        state_dot_vel_[2] = accel(2);
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    void SetMass(Scalar mass) { mass_ = mass; }
    [[nodiscard]] Scalar GetMass() const { return mass_; }

    void SetInitialPosition(const Vec3<Scalar> &pos) { ic_position_ = pos; }
    void SetInitialPosition(Scalar x, Scalar y, Scalar z) { ic_position_ = Vec3<Scalar>{x, y, z}; }

    void SetInitialVelocity(const Vec3<Scalar> &vel) { ic_velocity_ = vel; }
    void SetInitialVelocity(Scalar vx, Scalar vy, Scalar vz) {
        ic_velocity_ = Vec3<Scalar>{vx, vy, vz};
    }

    // Accessors
    [[nodiscard]] Vec3<Scalar> GetPosition() const { return position_; }
    [[nodiscard]] Vec3<Scalar> GetVelocity() const { return velocity_; }

  private:
    // Identity
    std::string name_;
    std::string entity_;

    // Initial conditions
    Vec3<Scalar> ic_position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> ic_velocity_ = Vec3<Scalar>::Zero();

    // State pointers (bound in BindState)
    Scalar *state_pos_ = nullptr;
    Scalar *state_vel_ = nullptr;
    Scalar *state_dot_pos_ = nullptr;
    Scalar *state_dot_vel_ = nullptr;

    // === Phase 2.4: Input Handles ===
    InputHandle<Scalar> force_x_;
    InputHandle<Scalar> force_y_;
    InputHandle<Scalar> force_z_;

    // === Phase 2.4: Parameter ===
    Scalar mass_{1.0};

    // Output values
    Vec3<Scalar> position_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> velocity_ = Vec3<Scalar>::Zero();
};

} // namespace components
} // namespace icarus
