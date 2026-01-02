#pragma once

/**
 * @file ForceAggregator.hpp
 * @brief Aggregates forces and moments from multiple sources
 *
 * Combines forces from multiple source components, handling:
 * - Force summation in body frame
 * - Moment transfer about CG for offset application points: M_cg = M_app + r × F
 *
 * Part of Phase 4: Aggregation & 6DOF
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/InputHandle.hpp>

#include <vector>

namespace icarus {
namespace components {

/**
 * @brief Force source handle bundle
 *
 * Holds SignalHandles needed to read a force source's outputs.
 * Forces are assumed to be in body frame; moment transfer is computed
 * if application_point differs from CG.
 */
template <typename Scalar> struct ForceSourceHandles {
    std::string name;

    // Required: force vector
    SignalHandle<Scalar> force_x, force_y, force_z;

    // Optional: moment about application point
    SignalHandle<Scalar> moment_x, moment_y, moment_z;

    // Optional: application point (body frame, relative to body origin)
    SignalHandle<Scalar> app_point_x, app_point_y, app_point_z;

    // Flags for optional signals
    bool has_moment{false};
    bool has_app_point{false};

    /**
     * @brief Get force vector from handles
     */
    Vec3<Scalar> GetForce() const { return Vec3<Scalar>{*force_x, *force_y, *force_z}; }

    /**
     * @brief Get moment vector (zero if not provided)
     */
    Vec3<Scalar> GetMoment() const {
        if (!has_moment) {
            return Vec3<Scalar>::Zero();
        }
        return Vec3<Scalar>{*moment_x, *moment_y, *moment_z};
    }

    /**
     * @brief Get application point (zero/CG if not provided)
     */
    Vec3<Scalar> GetApplicationPoint() const {
        if (!has_app_point) {
            return Vec3<Scalar>::Zero();
        }
        return Vec3<Scalar>{*app_point_x, *app_point_y, *app_point_z};
    }
};

/**
 * @brief Aggregates forces and moments from multiple sources
 *
 * Reads forces from configured source components and sums them.
 * Computes moment transfer for forces applied at points offset from CG:
 *   M_total = Σ(M_i + r_i × F_i)
 * where r_i is the vector from CG to application point.
 *
 * Configuration:
 * - sources: List of component names to aggregate from
 *
 * Inputs:
 * - cg.x, cg.y, cg.z [m] - CG position for moment transfer
 *
 * Outputs:
 * - total_force.x, .y, .z [N]
 * - total_moment.x, .y, .z [N·m]
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> class ForceAggregator : public Component<Scalar> {
  public:
    explicit ForceAggregator(std::string name = "ForceAggregator", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // =========================================================================
    // Component Identity
    // =========================================================================

    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "ForceAggregator"; }

    [[nodiscard]] std::size_t StateSize() const override { return 0; }

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    void Provision(Backplane<Scalar> &bp) override {
        // Output: total force
        bp.template register_output_vec3<Scalar>("total_force", &total_force_, "N",
                                                 "Total aggregated force");

        // Output: total moment about CG
        bp.template register_output_vec3<Scalar>("total_moment", &total_moment_, "N*m",
                                                 "Total moment about CG");

        // Input: CG position from MassAggregator (for moment transfer)
        bp.template register_input<Scalar>("cg.x", &cg_x_, "m", "CG X position");
        bp.template register_input<Scalar>("cg.y", &cg_y_, "m", "CG Y position");
        bp.template register_input<Scalar>("cg.z", &cg_z_, "m", "CG Z position");
    }

    void Stage(Backplane<Scalar> &bp) override {
        const auto &config = this->GetConfig();

        // Get list of source component names from config (if provided)
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
            ForceSourceHandles<Scalar> handles;
            handles.name = source_name;

            std::string base = prefix + source_name + ".";

            // Required: force vector
            handles.force_x = bp.template resolve<Scalar>(base + "force.x");
            handles.force_y = bp.template resolve<Scalar>(base + "force.y");
            handles.force_z = bp.template resolve<Scalar>(base + "force.z");

            // Optional: moment about application point
            if (bp.has_signal(base + "moment.x")) {
                handles.moment_x = bp.template resolve<Scalar>(base + "moment.x");
                handles.moment_y = bp.template resolve<Scalar>(base + "moment.y");
                handles.moment_z = bp.template resolve<Scalar>(base + "moment.z");
                handles.has_moment = true;
            }

            // Optional: application point
            if (bp.has_signal(base + "application_point.x")) {
                handles.app_point_x = bp.template resolve<Scalar>(base + "application_point.x");
                handles.app_point_y = bp.template resolve<Scalar>(base + "application_point.y");
                handles.app_point_z = bp.template resolve<Scalar>(base + "application_point.z");
                handles.has_app_point = true;
            }

            sources_.push_back(std::move(handles));
        }
    }

    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Handle empty case
        if (sources_.empty()) {
            total_force_ = Vec3<Scalar>::Zero();
            total_moment_ = Vec3<Scalar>::Zero();
            return;
        }

        // Get CG position for moment transfer
        Vec3<Scalar> cg{cg_x_.get(), cg_y_.get(), cg_z_.get()};

        // Initialize accumulators
        Vec3<Scalar> sum_force = Vec3<Scalar>::Zero();
        Vec3<Scalar> sum_moment = Vec3<Scalar>::Zero();

        // Aggregate all sources
        for (const auto &src : sources_) {
            Vec3<Scalar> F = src.GetForce();
            Vec3<Scalar> M = src.GetMoment();
            Vec3<Scalar> app_point = src.GetApplicationPoint();

            // Sum forces directly
            sum_force += F;

            // Transfer moment to CG: M_cg = M_app + r × F
            // where r = (application_point - cg)
            Vec3<Scalar> r = app_point - cg;
            Vec3<Scalar> moment_transfer = r.cross(F);
            sum_moment += M + moment_transfer;
        }

        // Write outputs
        total_force_ = sum_force;
        total_moment_ = sum_moment;
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

    [[nodiscard]] Vec3<Scalar> GetTotalForce() const { return total_force_; }
    [[nodiscard]] Vec3<Scalar> GetTotalMoment() const { return total_moment_; }

  private:
    std::string name_;
    std::string entity_;

    // Source configuration
    std::vector<std::string> source_names_;
    std::vector<ForceSourceHandles<Scalar>> sources_;

    // CG input for moment transfer (wired via SignalRouter)
    InputHandle<Scalar> cg_x_;
    InputHandle<Scalar> cg_y_;
    InputHandle<Scalar> cg_z_;

    // Aggregated outputs
    Vec3<Scalar> total_force_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> total_moment_ = Vec3<Scalar>::Zero();
};

} // namespace components
} // namespace icarus
