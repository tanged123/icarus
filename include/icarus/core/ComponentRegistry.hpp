#pragma once

/**
 * @file ComponentRegistry.hpp
 * @brief Registers all built-in components with the ComponentFactory
 *
 * This header must be included somewhere in the final executable to ensure
 * component types are registered before Simulator::FromConfig() is called.
 *
 * Usage:
 * - Include this header in your main.cpp or simulator compilation unit
 * - Or call RegisterBuiltinComponents() explicitly
 *
 * Part of Phase 4.0.7: Config-driven simulation
 */

#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/ComponentFactory.hpp>

// Built-in components
#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

namespace icarus {

/**
 * @brief Register all built-in components with the factory
 *
 * Call this before using Simulator::FromConfig() to ensure
 * all component types are available.
 *
 * @return Number of components registered
 */
inline std::size_t RegisterBuiltinComponents() {
    auto &factory = ComponentFactory<double>::Instance();

    // Skip if already registered
    if (factory.NumRegistered() > 0) {
        return factory.NumRegistered();
    }

    // Dynamics components
    factory.Register("PointMass3DOF", [](const ComponentConfig &cfg) {
        return std::make_unique<components::PointMass3DOF<double>>(cfg);
    });

    // Environment components
    factory.Register("PointMassGravity", [](const ComponentConfig &cfg) {
        return std::make_unique<components::PointMassGravity<double>>(cfg);
    });

    return factory.NumRegistered();
}

/**
 * @brief Static initializer to auto-register components
 *
 * This ensures components are registered when this header is included.
 */
namespace detail {
inline const std::size_t _builtin_components_registered = RegisterBuiltinComponents();
} // namespace detail

} // namespace icarus
