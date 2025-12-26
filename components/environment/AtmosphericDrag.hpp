#pragma once

/**
 * @file AtmosphericDrag.hpp
 * @brief Aerodynamic drag force component using exponential atmosphere
 *
 * Computes drag force: F_drag = -0.5 * ρ * v² * Cd * A * v_hat
 * Uses Vulcan's exponential atmosphere for density calculation.
 * THIS FILE IS JUST A DEMO. WILL NEED TO BE REDONE TO PROPERLY JUST OUTPUT ATMOSPHERIC VARS
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/signal/InputHandle.hpp>

#include <vulcan/atmosphere/ExponentialAtmosphere.hpp>
#include <vulcan/core/Constants.hpp>

namespace icarus::components {

/**
 * @brief Atmospheric drag force component
 *
 * Computes aerodynamic drag using:
 * - Exponential atmosphere model for density
 * - Standard drag equation: F = 0.5 * ρ * v² * Cd * A
 *
 * Inputs (via wiring):
 *   - position.{x,y,z}: Position in ECI frame [m]
 *   - velocity.{x,y,z}: Velocity in ECI frame [m/s]
 *
 * Outputs:
 *   - force.{x,y,z}: Drag force in ECI frame [N]
 *   - altitude: Computed altitude [m]
 *   - density: Computed atmospheric density [kg/m³]
 *   - dynamic_pressure: 0.5 * ρ * v² [Pa]
 *
 * Parameters:
 *   - drag_coefficient: Cd (default 2.2 for sphere)
 *   - reference_area: A [m²] (default 1.0)
 */
template <typename Scalar> class AtmosphericDrag : public Component<Scalar> {
  public:
    explicit AtmosphericDrag(std::string name = "AtmosphericDrag", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "AtmosphericDrag"; }
    [[nodiscard]] std::size_t StateSize() const override { return 0; } // Stateless

    // =========================================================================
    // Lifecycle
    // =========================================================================

    void Provision(Backplane<Scalar> &bp, const ComponentConfig &) override {
        // === Outputs ===
        bp.template register_output_vec3<Scalar>("force", &force_, "N", "Drag force");
        bp.template register_output<Scalar>("altitude", &altitude_, "m", "Altitude above surface");
        bp.template register_output<Scalar>("density", &density_, "kg/m^3", "Atmospheric density");
        bp.template register_output<Scalar>("dynamic_pressure", &dynamic_pressure_, "Pa",
                                            "Dynamic pressure");

        // === Inputs ===
        bp.template register_input<Scalar>("position.x", &position_x_);
        bp.template register_input<Scalar>("position.y", &position_y_);
        bp.template register_input<Scalar>("position.z", &position_z_);
        bp.template register_input<Scalar>("velocity.x", &velocity_x_);
        bp.template register_input<Scalar>("velocity.y", &velocity_y_);
        bp.template register_input<Scalar>("velocity.z", &velocity_z_);

        // === Parameters ===
        bp.register_param("drag_coefficient", &Cd_, Cd_);
        bp.register_param("reference_area", &area_, area_);
    }

    void Stage(Backplane<Scalar> &bp, const ComponentConfig &cfg) override {
        // Apply wiring from configuration
        for (const auto &[input, source] : cfg.wiring) {
            bp.template wire_input<Scalar>(input, source);
        }
    }

    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Read position and velocity from inputs
        Scalar px = position_x_.get();
        Scalar py = position_y_.get();
        Scalar pz = position_z_.get();
        Scalar vx = velocity_x_.get();
        Scalar vy = velocity_y_.get();
        Scalar vz = velocity_z_.get();

        // Compute altitude: h = |r| - R_earth
        Scalar r = janus::sqrt(px * px + py * py + pz * pz);
        Scalar altitude = r - vulcan::constants::earth::R_eq;
        altitude_ = altitude;

        // Compute atmospheric density using exponential model
        Scalar rho = vulcan::exponential_atmosphere::density(altitude);
        density_ = rho;

        // Compute velocity magnitude squared
        Scalar v_sq = vx * vx + vy * vy + vz * vz;
        Scalar v_mag = janus::sqrt(v_sq);

        // Dynamic pressure: q = 0.5 * ρ * v²
        Scalar q = Scalar{0.5} * rho * v_sq;
        dynamic_pressure_ = q;

        // Drag force magnitude: D = Cd * A * q
        Scalar D_mag = Cd_ * area_ * q;

        // Drag force direction: opposite to velocity
        // F_drag = -D * v_hat = -D * v / |v|
        // Use if_else_zero pattern for safe division (0 when v_mag is tiny)
        constexpr double eps = 1e-10;
        Scalar scale = janus::where(v_mag > eps, D_mag / v_mag, Scalar{0});

        force_(0) = -scale * vx;
        force_(1) = -scale * vy;
        force_(2) = -scale * vz;
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    void SetDragCoefficient(Scalar Cd) { Cd_ = Cd; }
    void SetReferenceArea(Scalar area) { area_ = area; }

    [[nodiscard]] Scalar GetDragCoefficient() const { return Cd_; }
    [[nodiscard]] Scalar GetReferenceArea() const { return area_; }

  private:
    std::string name_;
    std::string entity_;

    // Inputs
    InputHandle<Scalar> position_x_;
    InputHandle<Scalar> position_y_;
    InputHandle<Scalar> position_z_;
    InputHandle<Scalar> velocity_x_;
    InputHandle<Scalar> velocity_y_;
    InputHandle<Scalar> velocity_z_;

    // Outputs
    Vec3<Scalar> force_{};
    Scalar altitude_{0};
    Scalar density_{0};
    Scalar dynamic_pressure_{0};

    // Parameters
    Scalar Cd_{2.2};   // Drag coefficient (typical for sphere)
    Scalar area_{1.0}; // Reference area [m²]
};

} // namespace icarus::components
