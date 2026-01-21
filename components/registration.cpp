/**
 * @file registration.cpp
 * @brief Component factory registration for built-in components
 *
 * This file registers all built-in components with the ComponentFactory,
 * enabling them to be instantiated from YAML configuration files.
 *
 * Components are registered at static initialization time when this
 * translation unit is linked into an executable.
 *
 * Uses ICARUS_REGISTER_COMPONENT_AS macro for dual-backend registration
 * (NumericScalar + SymbolicScalar) with minimal boilerplate.
 */

#include <icarus/core/ComponentFactory.hpp>

#include <aggregators/ForceAggregator.hpp>
#include <aggregators/MassAggregator.hpp>
#include <dynamics/PointMass3DOF.hpp>
#include <dynamics/RigidBody6DOF.hpp>
#include <dynamics/Vehicle6DOF.hpp>
#include <environment/AtmosphericDrag.hpp>
#include <environment/PointMassGravity.hpp>
#include <mass/StaticMass.hpp>
#include <propulsion/FuelTank.hpp>
#include <propulsion/RocketEngine.hpp>

// =============================================================================
// Component Registration (Dual-Backend: Numeric + Symbolic)
// =============================================================================

// Dynamics
ICARUS_REGISTER_COMPONENT_AS(icarus::components::PointMass3DOF, "PointMass3DOF")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::RigidBody6DOF, "RigidBody6DOF")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::Vehicle6DOF, "Vehicle6DOF")

// Environment
ICARUS_REGISTER_COMPONENT_AS(icarus::components::PointMassGravity, "PointMassGravity")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::AtmosphericDrag, "AtmosphericDrag")

// Mass
ICARUS_REGISTER_COMPONENT_AS(icarus::components::StaticMass, "StaticMass")

// Aggregators
ICARUS_REGISTER_COMPONENT_AS(icarus::components::MassAggregator, "MassAggregator")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::ForceAggregator, "ForceAggregator")

// Propulsion
ICARUS_REGISTER_COMPONENT_AS(icarus::components::RocketEngine, "RocketEngine")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::FuelTank, "FuelTank")
