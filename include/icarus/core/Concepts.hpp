#pragma once

#include <concepts>
#include <type_traits>

namespace icarus {

/**
 * @brief Concept for types that can be used as Scalar in Icarus.
 *
 * A valid Scalar type must support:
 * - Arithmetic operations (+, -, *, /)
 * - Construction from double
 * - Default construction
 *
 * This is satisfied by both `double` and `casadi::MX`.
 */
template <typename T>
concept JanusScalar = requires(T a, T b, double d) {
    // Arithmetic operations
    { a + b } -> std::convertible_to<T>;
    { a - b } -> std::convertible_to<T>;
    { a * b } -> std::convertible_to<T>;
    { a / b } -> std::convertible_to<T>;

    // Construction from double
    { T(d) } -> std::same_as<T>;

    // Default constructible
    { T() } -> std::same_as<T>;
};

/**
 * @brief Concept for component types.
 *
 * Components must implement the core lifecycle methods.
 */
template <typename T, typename Scalar>
concept ComponentType = requires(T &c) {
    { c.Name() } -> std::convertible_to<std::string>;
};

} // namespace icarus
