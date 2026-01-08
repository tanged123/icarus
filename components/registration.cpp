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
 * Registers with both janus::NumericScalar (double) and janus::SymbolicScalar
 * (casadi::MX) backends for full dual-backend support.
 */

#include <icarus/core/ComponentFactory.hpp>

#include <aggregators/ForceAggregator.hpp>
#include <aggregators/MassAggregator.hpp>
#include <dynamics/PointMass3DOF.hpp>
#include <dynamics/RigidBody6DOF.hpp>
#include <environment/AtmosphericDrag.hpp>
#include <environment/PointMassGravity.hpp>
#include <mass/StaticMass.hpp>
#include <propulsion/FuelTank.hpp>
#include <propulsion/RocketEngine.hpp>

namespace {

// Static registration - runs at program startup
// Registers all components with both numeric and symbolic backends.

// === Numeric Backend (janus::NumericScalar = double) ===
const bool registered_numeric = []() {
    auto &factory = ::icarus::ComponentFactory<janus::NumericScalar>::Instance();

    // Dynamics components
    factory.Register("PointMass3DOF", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::PointMass3DOF<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Environment components
    factory.Register("PointMassGravity", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::PointMassGravity<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("AtmosphericDrag", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::AtmosphericDrag<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Mass components
    factory.Register("StaticMass", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::StaticMass<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Aggregator components
    factory.Register("MassAggregator", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::MassAggregator<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("ForceAggregator", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::ForceAggregator<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("RigidBody6DOF", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::RigidBody6DOF<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Propulsion components
    factory.Register("RocketEngine", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::RocketEngine<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("FuelTank", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::FuelTank<janus::NumericScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    return true;
}();

// === Symbolic Backend (janus::SymbolicScalar = casadi::MX) ===
const bool registered_symbolic = []() {
    auto &factory = ::icarus::ComponentFactory<janus::SymbolicScalar>::Instance();

    // Dynamics components
    factory.Register("PointMass3DOF", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::PointMass3DOF<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Environment components
    factory.Register("PointMassGravity", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::PointMassGravity<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("AtmosphericDrag", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::AtmosphericDrag<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Mass components
    factory.Register("StaticMass", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::StaticMass<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Aggregator components
    factory.Register("MassAggregator", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::MassAggregator<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("ForceAggregator", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::ForceAggregator<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("RigidBody6DOF", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::RigidBody6DOF<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Propulsion components
    factory.Register("RocketEngine", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::RocketEngine<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("FuelTank", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::FuelTank<janus::SymbolicScalar>>(
            config.name, config.entity);
        comp->SetConfig(config);
        return comp;
    });

    return true;
}();

} // anonymous namespace
