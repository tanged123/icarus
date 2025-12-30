/**
 * @file registration.cpp
 * @brief Component factory registration for built-in components
 *
 * This file registers all built-in components with the ComponentFactory,
 * enabling them to be instantiated from YAML configuration files.
 *
 * Components are registered at static initialization time when this
 * translation unit is linked into an executable.
 */

#include <icarus/core/ComponentFactory.hpp>

#include <dynamics/PointMass3DOF.hpp>
#include <environment/AtmosphericDrag.hpp>
#include <environment/PointMassGravity.hpp>

namespace {

// Static registration - runs at program startup
const bool registered = []() {
    auto &factory = ::icarus::ComponentFactory<double>::Instance();

    // Dynamics components
    factory.Register("PointMass3DOF", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::PointMass3DOF<double>>(config.name,
                                                                                  config.entity);
        comp->SetConfig(config);
        return comp;
    });

    // Environment components
    factory.Register("PointMassGravity", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::PointMassGravity<double>>(config.name,
                                                                                     config.entity);
        comp->SetConfig(config);
        return comp;
    });

    factory.Register("AtmosphericDrag", [](const ::icarus::ComponentConfig &config) {
        auto comp = std::make_unique<::icarus::components::AtmosphericDrag<double>>(config.name,
                                                                                    config.entity);
        comp->SetConfig(config);
        return comp;
    });

    return true;
}();

} // anonymous namespace
