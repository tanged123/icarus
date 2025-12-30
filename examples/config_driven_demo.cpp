/**
 * @file config_driven_demo.cpp
 * @brief Configuration-Driven Simulation Demo
 *
 * Demonstrates loading simulation configuration from YAML:
 * - Load config with SimulationLoader::Load()
 * - Inspect parsed configuration
 * - Set up Simulator using config values
 * - Add components programmatically (until ComponentFactory is ready)
 * - Run simulation and verify orbit
 *
 * Usage: ./config_driven_demo [config_path]
 *        Default config: config/orbital_demo.yaml
 */

#include <icarus/icarus.hpp>
#include <icarus/io/SimulationLoader.hpp>

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace icarus;
using namespace icarus::components;
namespace fs = std::filesystem;

void print_config_summary(const SimulatorConfig &config) {
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Configuration Loaded                                      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Simulation: " << config.name << "\n";
    std::cout << "Version:    " << config.version << "\n";
    std::cout << "Description:\n  " << config.description.substr(0, 60) << "...\n\n";

    std::cout << "Time Configuration:\n";
    std::cout << "  Start:    " << config.t_start << " s\n";
    std::cout << "  End:      " << config.t_end << " s\n";
    std::cout << "  Timestep: " << config.dt << " s\n\n";

    std::cout << "Integrator: " << to_string(config.integrator.type) << "\n\n";

    std::cout << "Components (" << config.components.size() << "):\n";
    for (const auto &comp : config.components) {
        std::cout << "  - " << comp.entity << "." << comp.name << " [" << comp.type << "]\n";
    }
    std::cout << "\n";

    std::cout << "Routes (" << config.routes.size() << "):\n";
    for (size_t i = 0; i < std::min<size_t>(3, config.routes.size()); ++i) {
        std::cout << "  " << config.routes[i].input_path << " ← " << config.routes[i].output_path
                  << "\n";
    }
    if (config.routes.size() > 3) {
        std::cout << "  ... and " << (config.routes.size() - 3) << " more\n";
    }
    std::cout << "\n";

    std::cout << "Scheduler Groups (" << config.scheduler.groups.size() << "):\n";
    for (const auto &group : config.scheduler.groups) {
        std::cout << "  - " << group.name << " @ " << group.rate_hz << " Hz\n";
    }
    std::cout << "\n";
}

int main(int argc, char *argv[]) {
    // Determine config path
    std::string config_path = "config/orbital_demo.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       Icarus Configuration-Driven Demo                     ║\n";
    std::cout << "║       Loading from: " << std::setw(38) << std::left << config_path.substr(0, 38)
              << "║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // =========================================================================
    // Step 1: Load YAML Configuration
    // =========================================================================

    if (!fs::exists(config_path)) {
        std::cerr << "❌ Error: Config file not found: " << config_path << "\n";
        std::cerr << "   Run from project root or specify path as argument.\n";
        return 1;
    }

    std::cout << "Loading configuration from: " << config_path << "\n\n";

    SimulatorConfig config;
    try {
        config = io::SimulationLoader::Load(config_path);
    } catch (const std::exception &e) {
        std::cerr << "❌ Error loading config: " << e.what() << "\n";
        return 1;
    }

    print_config_summary(config);

    // =========================================================================
    // Step 2: Create Simulator and Apply Config
    // =========================================================================

    std::cout << "Setting up simulator...\n";

    Simulator sim;
    sim.Configure(config); // Applies time, integrator, scheduler settings

    // =========================================================================
    // Step 3: Create Components (based on config)
    // =========================================================================

    // Until ComponentFactory is implemented, we create components manually
    // using the configuration values from the YAML

    // Find component configs
    const ComponentConfig *pm_config = nullptr;
    const ComponentConfig *grav_config = nullptr;

    for (const auto &comp : config.components) {
        if (comp.type == "PointMass3DOF") {
            pm_config = &comp;
        } else if (comp.type == "PointMassGravity") {
            grav_config = &comp;
        }
    }

    if (!pm_config || !grav_config) {
        std::cerr << "❌ Error: Required components not found in config\n";
        return 1;
    }

    // Create PointMass3DOF with config values
    double mass = pm_config->scalars.count("mass") ? pm_config->scalars.at("mass") : 1.0;
    auto pm = std::make_unique<PointMass3DOF<double>>(mass, pm_config->name);

    // Apply initial conditions from config
    if (pm_config->vectors.count("initial_position")) {
        auto &pos = pm_config->vectors.at("initial_position");
        pm->SetInitialPosition(pos[0], pos[1], pos[2]);
    }
    if (pm_config->vectors.count("initial_velocity")) {
        auto &vel = pm_config->vectors.at("initial_velocity");
        pm->SetInitialVelocity(vel[0], vel[1], vel[2]);
    }

    // Create PointMassGravity with config values
    auto grav = std::make_unique<PointMassGravity<double>>(grav_config->name);
    if (grav_config->scalars.count("mu")) {
        grav->SetGravitationalParameter(grav_config->scalars.at("mu"));
    }

    std::cout << "  Created: " << pm_config->name << " (mass=" << mass << " kg)\n";
    std::cout << "  Created: " << grav_config->name << " (μ=" << grav_config->scalars.at("mu")
              << " m³/s²)\n\n";

    // Add components to simulator
    sim.AddComponent(std::move(grav));
    sim.AddComponent(std::move(pm));

    // Add routes from config
    sim.AddRoutes(config.routes);

    // =========================================================================
    // Step 4: Stage and Run Simulation
    // =========================================================================

    std::cout << "Staging simulation...\n";
    sim.Stage();
    std::cout << "  Phase: Staged\n";
    std::cout << "  Components: " << sim.NumComponents() << "\n";
    std::cout << "  State size: " << sim.GetState().size() << "\n\n";

    // Run for a fraction of the configured time (demo purposes)
    double demo_duration = 180.0; // 3 minutes instead of full 90
    std::cout << "Running simulation for " << demo_duration << " s (demo mode)...\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(10) << "Time [s]" << std::setw(15) << "Altitude [km]" << std::setw(15)
              << "Speed [km/s]" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";

    const double dt = config.dt;
    const double R_earth = 6.371e6; // Earth radius [m]
    int step = 0;

    while (sim.Time() < demo_duration) {
        sim.Step(dt);

        // Print every 30 seconds
        if (++step % 30 == 0 || sim.Time() >= demo_duration - 0.5) {
            auto state = sim.GetState();
            double x = state[0], y = state[1], z = state[2];
            double vx = state[3], vy = state[4], vz = state[5];

            double r = std::sqrt(x * x + y * y + z * z);
            double altitude = (r - R_earth) / 1000.0;                       // km
            double speed = std::sqrt(vx * vx + vy * vy + vz * vz) / 1000.0; // km/s

            std::cout << std::fixed << std::setprecision(1);
            std::cout << std::setw(10) << sim.Time() << std::setw(15) << altitude << std::setw(15)
                      << speed << "\n";
        }
    }

    std::cout << "─────────────────────────────────────────────────────────────\n\n";

    // =========================================================================
    // Step 5: Verify Results
    // =========================================================================

    auto final_state = sim.GetState();
    double x = final_state[0], y = final_state[1], z = final_state[2];
    double vx = final_state[3], vy = final_state[4], vz = final_state[5];

    double r_final = std::sqrt(x * x + y * y + z * z);
    double r_initial = pm_config->vectors.at("initial_position")[0]; // Started on x-axis
    double altitude_drift = std::abs(r_final - r_initial);

    std::cout << "Results:\n";
    std::cout << "  Final position: (" << x / 1e6 << ", " << y / 1e6 << ", " << z / 1e6 << ") Mm\n";
    std::cout << "  Final velocity: (" << vx / 1e3 << ", " << vy / 1e3 << ", " << vz / 1e3
              << ") km/s\n";
    std::cout << "  Orbital radius drift: " << altitude_drift << " m\n\n";

    // For a circular orbit, radius should stay nearly constant
    bool passed = altitude_drift < 100.0; // Allow 100m drift over 3 minutes

    if (passed) {
        std::cout << "✅ PASSED: Orbit remains stable (drift < 100m)\n";
    } else {
        std::cout << "⚠️  WARNING: Larger than expected drift (consider reducing dt)\n";
    }

    return 0;
}
