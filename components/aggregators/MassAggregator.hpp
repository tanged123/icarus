#pragma once

/**
 * @file MassAggregator.hpp
 * @brief Aggregates mass properties from multiple sources
 *
 * Combines mass, CG, and inertia from multiple mass source components
 * using Vulcan's MassProperties aggregation with parallel axis theorem.
 *
 * Part of Phase 4: Aggregation & 6DOF
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Handle.hpp>

#include <vulcan/mass/MassProperties.hpp>

#include <vector>

namespace icarus {
namespace components {

/**
 * @brief Mass source handle bundle
 *
 * Holds all SignalHandles needed to read a mass source's properties.
 * Uses resolve() pattern for dynamic source discovery.
 */
template <typename Scalar> struct MassSourceHandles {
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
 * @brief Aggregates mass properties from multiple sources
 *
 * Reads mass, CG, and inertia from configured source components,
 * aggregates using Vulcan's MassProperties::operator+ (handles
 * parallel axis theorem automatically).
 *
 * Configuration:
 * - sources: List of component names to aggregate from
 *
 * Outputs:
 * - total_mass [kg]
 * - cg.x, cg.y, cg.z [m]
 * - inertia.xx, .yy, .zz, .xy, .xz, .yz [kg·m²]
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class MassAggregator : public Component<Scalar> {
  public:
    explicit MassAggregator(std::string name = "MassAggregator", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "MassAggregator"; }

    [[nodiscard]] std::size_t StateSize() const override { return 0; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    void Provision(Backplane<Scalar> &bp) override {
        // Output: aggregated mass
        bp.template register_output<Scalar>("total_mass", &total_mass_, "kg", "Total mass");

        // Output: aggregated CG
        bp.template register_output_vec3<Scalar>("cg", &cg_, "m", "Combined CG position");

        // Output: aggregated inertia tensor
        bp.template register_output<Scalar>("inertia.xx", &inertia_xx_, "kg*m^2", "Combined Ixx");
        bp.template register_output<Scalar>("inertia.yy", &inertia_yy_, "kg*m^2", "Combined Iyy");
        bp.template register_output<Scalar>("inertia.zz", &inertia_zz_, "kg*m^2", "Combined Izz");
        bp.template register_output<Scalar>("inertia.xy", &inertia_xy_, "kg*m^2", "Combined Ixy");
        bp.template register_output<Scalar>("inertia.xz", &inertia_xz_, "kg*m^2", "Combined Ixz");
        bp.template register_output<Scalar>("inertia.yz", &inertia_yz_, "kg*m^2", "Combined Iyz");
    }

    void Stage(Backplane<Scalar> &bp) override {
        const auto &config = this->GetConfig();

        // Get list of source component names from config (if provided)
        // Only override programmatic sources if config has sources
        if (!config.sources.empty()) {
            source_names_ = config.sources;
        }

        // If no sources configured, we're done
        if (source_names_.empty()) {
            return;
        }

        // Build entity prefix for signal resolution
        std::string prefix = entity_.empty() ? "" : entity_ + ".";

        // Create handles for each source
        sources_.clear();
        sources_.reserve(source_names_.size());

        for (const auto &source_name : source_names_) {
            MassSourceHandles<Scalar> handles;
            handles.name = source_name;

            std::string base = prefix + source_name + ".";

            // Resolve handles to source outputs
            handles.mass = bp.template resolve<Scalar>(base + "mass");
            handles.cg_x = bp.template resolve<Scalar>(base + "cg.x");
            handles.cg_y = bp.template resolve<Scalar>(base + "cg.y");
            handles.cg_z = bp.template resolve<Scalar>(base + "cg.z");
            handles.inertia_xx = bp.template resolve<Scalar>(base + "inertia.xx");
            handles.inertia_yy = bp.template resolve<Scalar>(base + "inertia.yy");
            handles.inertia_zz = bp.template resolve<Scalar>(base + "inertia.zz");
            handles.inertia_xy = bp.template resolve<Scalar>(base + "inertia.xy");
            handles.inertia_xz = bp.template resolve<Scalar>(base + "inertia.xz");
            handles.inertia_yz = bp.template resolve<Scalar>(base + "inertia.yz");

            sources_.push_back(std::move(handles));
        }
    }

    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Handle empty case
        if (sources_.empty()) {
            total_mass_ = Scalar(0);
            cg_ = Vec3<Scalar>::Zero();
            inertia_xx_ = inertia_yy_ = inertia_zz_ = Scalar(0);
            inertia_xy_ = inertia_xz_ = inertia_yz_ = Scalar(0);
            return;
        }

        // Read first source
        vulcan::mass::MassProperties<Scalar> total = sources_[0].Get();

        // Aggregate remaining sources using Vulcan's operator+
        for (size_t i = 1; i < sources_.size(); ++i) {
            total = total + sources_[i].Get();
        }

        // Write outputs
        total_mass_ = total.mass;
        cg_ = total.cg;
        inertia_xx_ = total.inertia(0, 0);
        inertia_yy_ = total.inertia(1, 1);
        inertia_zz_ = total.inertia(2, 2);
        inertia_xy_ = total.inertia(0, 1);
        inertia_xz_ = total.inertia(0, 2);
        inertia_yz_ = total.inertia(1, 2);
    }

    // =========================================================================
    // Programmatic Configuration
    // =========================================================================

    /**
     * @brief Add a source component by name (programmatic setup)
     */
    void AddSource(const std::string &source_name) { source_names_.push_back(source_name); }

    /**
     * @brief Clear all sources
     */
    void ClearSources() {
        source_names_.clear();
        sources_.clear();
    }

    // =========================================================================
    // Accessors
    // =========================================================================

    [[nodiscard]] Scalar GetTotalMass() const { return total_mass_; }
    [[nodiscard]] Vec3<Scalar> GetCG() const { return cg_; }

    [[nodiscard]] vulcan::mass::MassProperties<Scalar> GetMassProperties() const {
        Mat3<Scalar> I;
        I(0, 0) = inertia_xx_;
        I(1, 1) = inertia_yy_;
        I(2, 2) = inertia_zz_;
        I(0, 1) = I(1, 0) = inertia_xy_;
        I(0, 2) = I(2, 0) = inertia_xz_;
        I(1, 2) = I(2, 1) = inertia_yz_;

        return vulcan::mass::MassProperties<Scalar>{.mass = total_mass_, .cg = cg_, .inertia = I};
    }

  private:
    std::string name_;
    std::string entity_;

    // Source configuration
    std::vector<std::string> source_names_;
    std::vector<MassSourceHandles<Scalar>> sources_;

    // Aggregated outputs
    Scalar total_mass_{0};
    Vec3<Scalar> cg_ = Vec3<Scalar>::Zero();
    Scalar inertia_xx_{0}, inertia_yy_{0}, inertia_zz_{0};
    Scalar inertia_xy_{0}, inertia_xz_{0}, inertia_yz_{0};
};

} // namespace components
} // namespace icarus
