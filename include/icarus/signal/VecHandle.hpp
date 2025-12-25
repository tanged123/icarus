#pragma once

/**
 * @file VecHandle.hpp
 * @brief Vector and matrix handles for structured signal access
 *
 * Part of Phase 1.3: Signal Backplane.
 * Provides Vec3Handle and Mat3Handle for convenient access to vector/matrix signals.
 */

#include <icarus/core/Types.hpp>
#include <icarus/signal/Handle.hpp>

namespace icarus {

/**
 * @brief Handle for accessing a Vec3 signal as three scalar components
 *
 * Vec3Handle bundles three SignalHandles (x, y, z) and provides
 * convenience methods for reading/writing the entire vector.
 *
 * Usage:
 * @code
 *   // Registration (in component Provision)
 *   Vec3<double> position_;
 *   registry.register_output_vec3("position", &position_, "m");
 *
 *   // Resolution (in consumer Stage)
 *   Vec3Handle<double> pos = registry.resolve_vec3<double>("position");
 *
 *   // Access (in Step)
 *   Vec3<double> p = pos.get();     // Read all components
 *   pos.set(new_position);          // Write all components
 *   double x = *pos.x;              // Read single component
 * @endcode
 *
 * @tparam Scalar The scalar type (double or casadi::MX)
 */
template <typename Scalar> struct Vec3Handle {
    SignalHandle<Scalar> x; ///< X component handle
    SignalHandle<Scalar> y; ///< Y component handle
    SignalHandle<Scalar> z; ///< Z component handle

    /**
     * @brief Read all components as a Vec3
     */
    [[nodiscard]] Vec3<Scalar> get() const { return Vec3<Scalar>(*x, *y, *z); }

    /**
     * @brief Write all components from a Vec3
     */
    void set(const Vec3<Scalar> &v) {
        *x = v(0);
        *y = v(1);
        *z = v(2);
    }

    /**
     * @brief Check if all handles are valid
     */
    [[nodiscard]] bool valid() const { return x.valid() && y.valid() && z.valid(); }

    /**
     * @brief Check if handle is valid
     */
    explicit operator bool() const { return valid(); }
};

/**
 * @brief Handle for accessing a quaternion signal as four scalar components
 *
 * @tparam Scalar The scalar type (double or casadi::MX)
 */
template <typename Scalar> struct QuatHandle {
    SignalHandle<Scalar> w; ///< W (scalar) component handle
    SignalHandle<Scalar> x; ///< X component handle
    SignalHandle<Scalar> y; ///< Y component handle
    SignalHandle<Scalar> z; ///< Z component handle

    /**
     * @brief Read all components as a Vec4 (w, x, y, z order)
     */
    [[nodiscard]] Vec4<Scalar> get() const { return Vec4<Scalar>(*w, *x, *y, *z); }

    /**
     * @brief Write all components from a Vec4 (w, x, y, z order)
     */
    void set(const Vec4<Scalar> &q) {
        *w = q(0);
        *x = q(1);
        *y = q(2);
        *z = q(3);
    }

    /**
     * @brief Check if all handles are valid
     */
    [[nodiscard]] bool valid() const { return w.valid() && x.valid() && y.valid() && z.valid(); }

    /**
     * @brief Check if handle is valid
     */
    explicit operator bool() const { return valid(); }
};

/**
 * @brief Handle for accessing a Mat3 signal as nine scalar components
 *
 * Matrix elements are stored in column-major order to match Eigen.
 *
 * @tparam Scalar The scalar type (double or casadi::MX)
 */
template <typename Scalar> struct Mat3Handle {
    SignalHandle<Scalar> m00, m10, m20; ///< Column 0
    SignalHandle<Scalar> m01, m11, m21; ///< Column 1
    SignalHandle<Scalar> m02, m12, m22; ///< Column 2

    /**
     * @brief Read all components as a Mat3
     */
    [[nodiscard]] Mat3<Scalar> get() const {
        Mat3<Scalar> m;
        m(0, 0) = *m00;
        m(1, 0) = *m10;
        m(2, 0) = *m20;
        m(0, 1) = *m01;
        m(1, 1) = *m11;
        m(2, 1) = *m21;
        m(0, 2) = *m02;
        m(1, 2) = *m12;
        m(2, 2) = *m22;
        return m;
    }

    /**
     * @brief Write all components from a Mat3
     */
    void set(const Mat3<Scalar> &m) {
        *m00 = m(0, 0);
        *m10 = m(1, 0);
        *m20 = m(2, 0);
        *m01 = m(0, 1);
        *m11 = m(1, 1);
        *m21 = m(2, 1);
        *m02 = m(0, 2);
        *m12 = m(1, 2);
        *m22 = m(2, 2);
    }

    /**
     * @brief Check if all handles are valid
     */
    [[nodiscard]] bool valid() const {
        return m00.valid() && m10.valid() && m20.valid() && m01.valid() && m11.valid() &&
               m21.valid() && m02.valid() && m12.valid() && m22.valid();
    }

    /**
     * @brief Check if handle is valid
     */
    explicit operator bool() const { return valid(); }
};

} // namespace icarus
