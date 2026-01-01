#pragma once

/**
 * @file AggregationTypes.hpp
 * @brief Types for force and mass aggregation in 6DOF simulations
 *
 * Provides frame enumerations and data structures for multi-source
 * force/moment aggregation.
 */

#include <icarus/core/CoreTypes.hpp>

namespace icarus {

// =============================================================================
// Reference Frames
// =============================================================================

/**
 * @brief Reference frame for force/moment quantities
 *
 * Forces can be expressed in different reference frames. The aggregator
 * must transform all forces to body frame before summing.
 */
enum class Frame : uint8_t {
    BODY,    ///< Body-fixed frame (X=forward, Y=right, Z=down)
    WIND,    ///< Velocity-aligned frame (X=airspeed direction)
    LOCAL,   ///< Component-local frame (e.g., nozzle axis)
    INERTIAL ///< Inertial frame (ECI or ECEF)
};

/**
 * @brief Convert Frame enum to string for logging
 */
inline const char *FrameToString(Frame frame) {
    switch (frame) {
    case Frame::BODY:
        return "BODY";
    case Frame::WIND:
        return "WIND";
    case Frame::LOCAL:
        return "LOCAL";
    case Frame::INERTIAL:
        return "INERTIAL";
    default:
        return "UNKNOWN";
    }
}

// =============================================================================
// Force Contribution Structure (Optional - for advanced use)
// =============================================================================

/**
 * @brief Force contribution from a single source
 *
 * Captures all information needed to transform a force/moment pair
 * to body frame and transfer moments about the CG.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar> struct ForceContribution {
    Vec3<Scalar> force;             ///< Force vector in source frame
    Vec3<Scalar> moment;            ///< Moment about application point
    Vec3<Scalar> application_point; ///< Point of force application (body frame)
    Frame frame;                    ///< Frame of force/moment vectors

    /**
     * @brief Create a body-frame force at CG (no moment transfer needed)
     */
    static ForceContribution<Scalar> AtCG(const Vec3<Scalar> &f) {
        return ForceContribution<Scalar>{.force = f,
                                         .moment = Vec3<Scalar>::Zero(),
                                         .application_point = Vec3<Scalar>::Zero(),
                                         .frame = Frame::BODY};
    }

    /**
     * @brief Create a body-frame force at a specified point
     */
    static ForceContribution<Scalar> AtPoint(const Vec3<Scalar> &f, const Vec3<Scalar> &point) {
        return ForceContribution<Scalar>{.force = f,
                                         .moment = Vec3<Scalar>::Zero(),
                                         .application_point = point,
                                         .frame = Frame::BODY};
    }
};

// =============================================================================
// Lifecycle Classification for Aggregation
// =============================================================================

/**
 * @brief Signal lifecycle for mass/force sources
 *
 * Used to distinguish static (constant) vs dynamic (time-varying) quantities.
 */
enum class SourceLifecycle : uint8_t {
    STATIC, ///< Constant during run (e.g., dry mass, structure)
    DYNAMIC ///< Time-varying (e.g., fuel mass, sloshing)
};

} // namespace icarus
