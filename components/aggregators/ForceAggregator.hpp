#pragma once

/**
 * @file ForceAggregator.hpp
 * @brief Aggregates forces and moments from multiple sources
 *
 * Combines forces from multiple source components, handling:
 * - Force summation in body frame
 * - Frame transformation (ECEF → body) for sources in different frames
 * - Moment transfer about CG for offset application points: M_cg = M_app + r × F
 *
 * Part of Phase 4: Aggregation & 6DOF
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/InputHandle.hpp>

#include <janus/math/Quaternion.hpp>

#include <vector>

namespace icarus {
namespace components {

/**
 * @brief Force source handle bundle
 *
 * Holds SignalHandles needed to read a force source's outputs.
 * Frame information is tracked separately by ForceAggregator.
 * Moment transfer is computed if application_point differs from CG.
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
 * Supports forces in different coordinate frames:
 * - body_sources: Forces already in body frame (direct summation)
 * - ecef_sources: Forces in ECEF frame (transformed to body using attitude)
 *
 * Computes moment transfer for forces applied at points offset from CG:
 *   M_total = Σ(M_i + r_i × F_i)
 * where r_i is the vector from CG to application point.
 *
 * Configuration:
 * - body_sources: List of component names outputting body-frame forces
 * - ecef_sources: List of component names outputting ECEF-frame forces
 * - sources: Legacy - treated as body_sources for backwards compatibility
 *
 * Inputs:
 * - cg.x, cg.y, cg.z [m] - CG position for moment transfer
 * - attitude.w, .x, .y, .z [-] - Body-to-ECEF quaternion (for ECEF→body transform)
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

        // Input: Attitude quaternion (body-to-ECEF) for ECEF→body transformation
        bp.template register_input<Scalar>("attitude.w", &att_w_, "-", "Attitude quat W");
        bp.template register_input<Scalar>("attitude.x", &att_x_, "-", "Attitude quat X");
        bp.template register_input<Scalar>("attitude.y", &att_y_, "-", "Attitude quat Y");
        bp.template register_input<Scalar>("attitude.z", &att_z_, "-", "Attitude quat Z");
    }

    void Stage(Backplane<Scalar> &bp) override {
        const auto &config = this->GetConfig();

        // Get source lists from config, falling back to programmatic setup
        // Priority: config > programmatic
        std::vector<std::string> body_names =
            !config.body_sources.empty() ? config.body_sources : body_source_names_;
        std::vector<std::string> ecef_names =
            !config.ecef_sources.empty() ? config.ecef_sources : ecef_source_names_;

        // Legacy support: "sources" → body_sources (if neither config nor programmatic set)
        if (!config.sources.empty() && body_names.empty()) {
            body_names = config.sources;
        }

        // If no sources configured, we're done
        if (body_names.empty() && ecef_names.empty()) {
            return;
        }

        // Build entity prefix for signal resolution
        std::string prefix = entity_.empty() ? "" : entity_ + ".";

        // Helper to resolve handles for a source
        auto resolve_source = [&](const std::string &source_name) {
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

            return handles;
        };

        // Resolve body-frame sources
        body_sources_.clear();
        body_sources_.reserve(body_names.size());
        for (const auto &name : body_names) {
            body_sources_.push_back(resolve_source(name));
        }

        // Resolve ECEF-frame sources
        ecef_sources_.clear();
        ecef_sources_.reserve(ecef_names.size());
        for (const auto &name : ecef_names) {
            ecef_sources_.push_back(resolve_source(name));
        }
    }

    void Step(Scalar t, Scalar dt) override {
        (void)t;
        (void)dt;

        // Handle empty case
        if (body_sources_.empty() && ecef_sources_.empty()) {
            total_force_ = Vec3<Scalar>::Zero();
            total_moment_ = Vec3<Scalar>::Zero();
            return;
        }

        // Get CG position for moment transfer
        Vec3<Scalar> cg{cg_x_.get(), cg_y_.get(), cg_z_.get()};

        // Get attitude quaternion (body-to-ECEF) for frame transformation
        janus::Quaternion<Scalar> q_body_to_ecef{att_w_.get(), att_x_.get(), att_y_.get(),
                                                 att_z_.get()};

        // Initialize accumulators
        Vec3<Scalar> sum_force = Vec3<Scalar>::Zero();
        Vec3<Scalar> sum_moment = Vec3<Scalar>::Zero();

        // Helper to accumulate force/moment from a source
        auto accumulate = [&](const ForceSourceHandles<Scalar> &src, Vec3<Scalar> F_body) {
            Vec3<Scalar> M = src.GetMoment();
            Vec3<Scalar> app_point = src.GetApplicationPoint();

            // Sum forces
            sum_force += F_body;

            // Transfer moment to CG: M_cg = M_app + r × F
            // where r = (application_point - cg)
            Vec3<Scalar> r = app_point - cg;
            Vec3<Scalar> moment_transfer = r.cross(F_body);
            sum_moment += M + moment_transfer;
        };

        // Aggregate body-frame sources (no transformation needed)
        for (const auto &src : body_sources_) {
            Vec3<Scalar> F_body = src.GetForce();
            accumulate(src, F_body);
        }

        // Aggregate ECEF-frame sources (transform to body)
        for (const auto &src : ecef_sources_) {
            Vec3<Scalar> F_ecef = src.GetForce();

            // Transform ECEF → body using quaternion conjugate
            // q.conjugate() rotates from ECEF to body
            Vec3<Scalar> F_body = q_body_to_ecef.conjugate().rotate(F_ecef);

            accumulate(src, F_body);
        }

        // Write outputs
        total_force_ = sum_force;
        total_moment_ = sum_moment;
    }

    // =========================================================================
    // Programmatic Configuration
    // =========================================================================

    /**
     * @brief Add a body-frame source component by name
     */
    void AddBodySource(const std::string &source_name) {
        body_source_names_.push_back(source_name);
    }

    /**
     * @brief Add an ECEF-frame source component by name
     */
    void AddEcefSource(const std::string &source_name) {
        ecef_source_names_.push_back(source_name);
    }

    /**
     * @brief Add a source (legacy - treated as body source)
     */
    void AddSource(const std::string &source_name) { AddBodySource(source_name); }

    /**
     * @brief Clear all sources
     */
    void ClearSources() {
        body_source_names_.clear();
        ecef_source_names_.clear();
        body_sources_.clear();
        ecef_sources_.clear();
    }

    // =========================================================================
    // Accessors
    // =========================================================================

    [[nodiscard]] Vec3<Scalar> GetTotalForce() const { return total_force_; }
    [[nodiscard]] Vec3<Scalar> GetTotalMoment() const { return total_moment_; }

  private:
    std::string name_;
    std::string entity_;

    // Source configuration (programmatic)
    std::vector<std::string> body_source_names_;
    std::vector<std::string> ecef_source_names_;

    // Resolved handles by frame
    std::vector<ForceSourceHandles<Scalar>> body_sources_;
    std::vector<ForceSourceHandles<Scalar>> ecef_sources_;

    // CG input for moment transfer
    InputHandle<Scalar> cg_x_;
    InputHandle<Scalar> cg_y_;
    InputHandle<Scalar> cg_z_;

    // Attitude input for ECEF→body transformation
    InputHandle<Scalar> att_w_;
    InputHandle<Scalar> att_x_;
    InputHandle<Scalar> att_y_;
    InputHandle<Scalar> att_z_;

    // Aggregated outputs
    Vec3<Scalar> total_force_ = Vec3<Scalar>::Zero();
    Vec3<Scalar> total_moment_ = Vec3<Scalar>::Zero();
};

} // namespace components
} // namespace icarus
