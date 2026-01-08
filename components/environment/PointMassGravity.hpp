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
 * Updated for Phase 2.4: Uses register_input, register_param, register_config
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>

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
    enum class Model : int {
        Constant = 0,  ///< g = [0, 0, -g0] (for analytical validation)
        PointMass = 1, ///< vulcan::gravity::point_mass::acceleration()
    };

    /**
     * @brief Construct with name and entity
     */
    explicit PointMassGravity(std::string name = "Gravity", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "PointMassGravity"; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief Register outputs, inputs, and config
     */
    void Provision(Backplane<Scalar> &bp) override {
        // === Outputs ===
        // Output force (not acceleration) - allows summing multiple force sources
        bp.template register_output_vec3<Scalar>("force", &force_, "N", "Gravity force");

        // Also output acceleration for convenience/logging
        bp.template register_output_vec3<Scalar>("acceleration", &accel_, "m/s^2",
                                                 "Gravity acceleration");

        // === Inputs (Phase 2.4) ===
        // Position input from dynamics component
        bp.template register_input<Scalar>("position.x", &pos_x_, "m", "Position X");
        bp.template register_input<Scalar>("position.y", &pos_y_, "m", "Position Y");
        bp.template register_input<Scalar>("position.z", &pos_z_, "m", "Position Z");

        // Mass input for force calculation
        bp.template register_input<Scalar>("mass", &mass_input_, "kg", "Vehicle mass");

        // === Config (Phase 2.4) ===
        // Gravity model selection (0=Constant, 1=PointMass)
        bp.register_config("model", &model_int_, model_int_,
                           "Gravity model (0=Constant, 1=PointMass)");
    }

    /**
     * @brief Stage phase - load config
     *
     * Reads from config (only if explicitly set):
     * - scalars["mu"]: Gravitational parameter
     * - scalars["model"]: Gravity model (0=Constant, 1=PointMass)
     *
     * For programmatic setup, use SetGravitationalParameter(), SetModel()
     * before calling Stage() - these values won't be overwritten if
     * not present in config.
     */
    void Stage(Backplane<Scalar> &) override {
        // Read config using helpers (preserves defaults if not in config)
        mu_ = this->template read_param<double>("mu", mu_);
        model_int_ = this->template read_param<int>("model", model_int_);
    }

    /**
     * @brief Compute gravitational force using Vulcan
     */
    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read position and mass from registered inputs
        Vec3<Scalar> pos{pos_x_.get(), pos_y_.get(), pos_z_.get()};
        Scalar m = mass_input_.get();

        // Compute acceleration based on model config
        Model model = static_cast<Model>(model_int_);
        switch (model) {
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

    void SetModel(Model model) { model_int_ = static_cast<int>(model); }
    [[nodiscard]] Model GetModel() const { return static_cast<Model>(model_int_); }

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
    int model_int_ = 0; ///< Model as int for register_config
    double mu_ = vulcan::constants::earth::mu;

    // === Phase 2.4: Input Handles ===
    InputHandle<Scalar> pos_x_;
    InputHandle<Scalar> pos_y_;
    InputHandle<Scalar> pos_z_;
    InputHandle<Scalar> mass_input_;

    // Output values
    Vec3<Scalar> accel_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> force_ = Vec3<Scalar>::Zero();
};

} // namespace components
} // namespace icarus
