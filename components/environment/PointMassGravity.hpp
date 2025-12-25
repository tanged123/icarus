#pragma once

/**
 * @file PointMassGravity.hpp
 * @brief Gravity model component using Vulcan's point-mass gravity
 *
 * Wraps vulcan::gravity::point_mass::acceleration() for gravity computation.
 *
 * Frame Considerations:
 * - Constant model: Assumes local vertical frame (Z-up)
 * - PointMass/J2 models: Input/output vectors relative to Earth center
 *   (valid in both ECI and ECEF since gravity is a central force)
 *
 * Part of Phase 2.3: First Real Component
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/signal/Backplane.hpp>

#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/gravity/PointMass.hpp>

namespace icarus {
namespace components {

/**
 * @brief Gravity model component using Vulcan physics
 *
 * Uses Vulcan's gravity models:
 *   - vulcan::gravity::point_mass::acceleration(r) → g = -μ/r³ · r
 *
 * Outputs gravitational **force** (not acceleration) so it can be summed
 * with other forces before passing to the dynamics component.
 *
 * For Phase 2.3 validation, also supports simplified constant gravity.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class PointMassGravity : public Component<Scalar> {
  public:
    /**
     * @brief Gravity model fidelity
     */
    enum class Model {
        Constant,  ///< g = [0, 0, -g0] (for analytical validation)
        PointMass, ///< vulcan::gravity::point_mass::acceleration()
    };

    /**
     * @brief Construct with optional name and model
     */
    explicit PointMassGravity(std::string name = "Gravity", std::string entity = "",
                              Model model = Model::Constant)
        : name_(std::move(name)), entity_(std::move(entity)), model_(model) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "PointMassGravity"; }

    [[nodiscard]] std::size_t StateSize() const override { return 0; } // Stateless

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register outputs: force (N) and acceleration (m/s²)
     */
    void Provision(Backplane<Scalar> &bp, const ComponentConfig &) override {
        // Output force (not acceleration) - allows summing multiple force sources
        bp.template register_vec3<Scalar>("force", &force_, "N", "Gravity force");

        // Also output acceleration for convenience/logging
        bp.template register_vec3<Scalar>("acceleration", &accel_, "m/s^2", "Gravity acceleration");
    }

    /**
     * @brief Wire inputs: position and mass from dynamics component
     */
    void Stage(Backplane<Scalar> &bp, const ComponentConfig &cfg) override {
        // Resolve position input (from dynamics component)
        // Uses Vec3Handle pattern for vector inputs
        pos_handle_ = bp.template resolve_vec3<Scalar>("PointMass3DOF.position");

        // Resolve mass input (for force = mass * acceleration)
        mass_handle_ = bp.template resolve<Scalar>("PointMass3DOF.mass");

        // Config handling to be added later
        (void)cfg;
    }

    /**
     * @brief Compute gravitational force using Vulcan
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read position and mass
        Vec3<Scalar> pos = pos_handle_.get();
        Scalar m = *mass_handle_;

        // Compute acceleration based on model
        switch (model_) {
        case Model::Constant:
            // Local vertical frame: Z-up, constant gravity downward
            // Used for analytical validation: z(t) = z0 - ½gt²
            // NOTE: Only valid with local-frame dynamics, NOT for orbital sims
            accel_ = Vec3<Scalar>{Scalar{0}, Scalar{0}, -Scalar{vulcan::constants::physics::g0}};
            break;

        case Model::PointMass:
            // Central gravity toward Earth center: g = -μ/r³ · r
            // Works in ECI or ECEF (central force, same formula)
            // Position must be from Earth center [m]
            accel_ = vulcan::gravity::point_mass::acceleration(pos, mu_);
            break;
        }

        // Compute force: F = m * a
        force_ = accel_ * m;
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    void SetModel(Model model) { model_ = model; }
    [[nodiscard]] Model GetModel() const { return model_; }

    void SetGravitationalParameter(double mu) { mu_ = mu; }
    [[nodiscard]] double GetGravitationalParameter() const { return mu_; }

    // Accessors
    [[nodiscard]] Vec3<Scalar> GetAcceleration() const { return accel_; }
    [[nodiscard]] Vec3<Scalar> GetForce() const { return force_; }

  private:
    // Identity
    std::string name_;
    std::string entity_;

    // Configuration
    Model model_ = Model::Constant;
    double mu_ = vulcan::constants::earth::mu;

    // Input handles (resolved in Stage)
    Vec3Handle<Scalar> pos_handle_;
    SignalHandle<Scalar> mass_handle_;

    // Output values
    Vec3<Scalar> accel_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> force_ = Vec3<Scalar>::Zero();
};

} // namespace components
} // namespace icarus
