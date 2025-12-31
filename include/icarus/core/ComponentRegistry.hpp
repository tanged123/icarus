#pragma once

/**
 * @file ComponentRegistry.hpp
 * @brief Documentation for component registration
 *
 * Component registration is handled automatically in components/registration.cpp
 * using the ICARUS_REGISTER_COMPONENT macro, which registers components with
 * both janus::NumericScalar (double) and janus::SymbolicScalar (casadi::MX)
 * backends.
 *
 * To add a new component to the factory:
 * 1. Include the component header in components/registration.cpp
 * 2. Add: ICARUS_REGISTER_COMPONENT(YourComponentType)
 *
 * The macro expects components to have a two-argument constructor:
 *   ComponentType(std::string name, std::string entity)
 *
 * Configuration is set via SetConfig() after construction.
 *
 * Part of Phase 4.0.7: Config-driven simulation
 */

// No runtime registration needed - handled by static initialization
// in components/registration.cpp
