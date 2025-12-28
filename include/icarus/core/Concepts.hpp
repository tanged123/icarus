#pragma once

/**
 * @file Concepts.hpp
 * @brief C++20 concepts for Icarus type constraints
 *
 * Re-exports Janus concepts to ensure components use identical constraints.
 */

#include <concepts>
#include <string>

// Re-export Janus concepts
#include <janus/core/JanusConcepts.hpp>

// For Phase enum
#include <icarus/core/Types.hpp>

namespace icarus {

// =============================================================================
// Scalar Concepts
// =============================================================================

/**
 * @brief Re-export JanusScalar concept
 *
 * Satisfied by floating point types (double, float) or CasADi symbolic (MX).
 * All Icarus components must be templated on types satisfying this concept.
 */
using janus::JanusScalar;

/**
 * @brief Alias for Icarus-specific documentation
 *
 * Identical to JanusScalar, provided for clarity in Icarus context.
 */
template <typename T>
concept IcarusScalar = JanusScalar<T>;

// =============================================================================
// Component Concepts
// =============================================================================

// Forward declarations
template <typename Scalar> class Backplane;

/**
 * @brief Concept for types that can serve as Icarus components
 *
 * A valid component must implement the core lifecycle methods:
 * - Provision(): Heavy setup, register signals
 * - Stage(): Wire inputs, apply ICs
 * - Step(): Hot path, compute derivatives
 */
template <typename T, typename Scalar>
concept ComponentType =
    requires(T &c, Backplane<Scalar> &bp, const ComponentConfig &cfg, Scalar t, Scalar dt) {
        // Component must have a name
        { c.Name() } -> std::convertible_to<std::string>;

        // Core lifecycle methods
        { c.Provision(bp, cfg) } -> std::same_as<void>;
        { c.Stage(bp, cfg) } -> std::same_as<void>;
        { c.Step(t, dt) } -> std::same_as<void>;
    };

/**
 * @brief Concept for components with optional extended hooks
 */
template <typename T, typename Scalar>
concept ExtendedComponent = ComponentType<T, Scalar> && requires(T &c, Phase phase) {
    { c.OnPhaseEnter(phase) } -> std::same_as<void>;
    { c.OnPhaseExit(phase) } -> std::same_as<void>;
};

} // namespace icarus
