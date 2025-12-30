/**
 * @file config_driven_demo.cpp
 * @brief Configuration-Driven Simulation Demo
 *
 * Demonstrates the clean Simulator API:
 * - Simulator::FromConfig("config.yaml") loads and configures
 * - Components auto-created from YAML via ComponentFactory
 * - Stage() and Step() lifecycle
 *
 * Usage: ./config_driven_demo [config_path]
 *        Default: config/orbital_demo.yaml
 */

#include <icarus/icarus.hpp>

// Include components we want to use and register them
#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace icarus;
namespace fs = std::filesystem;

// Register component types with the factory
namespace {
void RegisterComponents() {
    auto &factory = ComponentFactory<double>::Instance();
    if (factory.NumRegistered() > 0)
        return; // Already registered

    factory.Register("PointMass3DOF", [](const ComponentConfig &cfg) {
        return std::make_unique<components::PointMass3DOF<double>>(cfg);
    });

    factory.Register("PointMassGravity", [](const ComponentConfig &cfg) {
        return std::make_unique<components::PointMassGravity<double>>(cfg);
    });
}
} // namespace

int main(int argc, char *argv[]) {
    // Register components before loading config
    RegisterComponents();

    // Config path - default or command line
    std::string config_path = "config/orbital_demo.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       Icarus Configuration-Driven Demo                     ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    if (!fs::exists(config_path)) {
        std::cerr << "❌ Config not found: " << config_path << "\n";
        std::cerr << "   Run from project root or specify path as argument.\n";
        return 1;
    }

    // =========================================================================
    // THE CLEAN API: One line to load YAML and create fully-configured sim
    // =========================================================================

    std::cout << "Loading: " << config_path << "\n";

    std::unique_ptr<Simulator> sim;
    try {
        sim = Simulator::FromConfig(config_path); // <-- ONE LINE!
    } catch (const std::exception &e) {
        std::cerr << "❌ Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "  Name: " << sim->Name() << "\n";
    std::cout << "  Components: " << sim->NumComponents() << "\n";
    std::cout << "  Timestep: " << sim->Dt() << " s\n";
    std::cout << "  End time: " << sim->EndTime() << " s\n\n";

    // =========================================================================
    // Stage() - Prepare for execution
    // =========================================================================

    std::cout << "Staging simulation...\n";
    sim->Stage();
    std::cout << "  State size: " << sim->GetState().size() << "\n\n";

    // =========================================================================
    // Run simulation
    // =========================================================================

    double demo_duration = 180.0; // 3 minutes (subset of full 90-min orbit)
    std::cout << "Running for " << demo_duration << " s...\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(10) << "Time [s]" << std::setw(15) << "Altitude [km]" << std::setw(15)
              << "Speed [km/s]" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";

    const double R_earth = 6.371e6; // Earth radius [m]
    int step = 0;

    while (sim->Time() < demo_duration) {
        sim->Step(sim->Dt());

        // Print every 30 seconds
        if (++step % static_cast<int>(30.0 / sim->Dt()) == 0) {
            auto state = sim->GetState();
            double x = state[0], y = state[1], z = state[2];
            double vx = state[3], vy = state[4], vz = state[5];

            double r = std::sqrt(x * x + y * y + z * z);
            double altitude = (r - R_earth) / 1000.0;                       // km
            double speed = std::sqrt(vx * vx + vy * vy + vz * vz) / 1000.0; // km/s

            std::cout << std::fixed << std::setprecision(1);
            std::cout << std::setw(10) << sim->Time() << std::setw(15) << altitude << std::setw(15)
                      << speed << "\n";
        }
    }

    std::cout << "─────────────────────────────────────────────────────────────\n\n";

    // =========================================================================
    // Results
    // =========================================================================

    auto final_state = sim->GetState();
    double x = final_state[0], y = final_state[1], z = final_state[2];
    double r_final = std::sqrt(x * x + y * y + z * z);

    std::cout << "Final Results:\n";
    std::cout << "  Time: " << std::fixed << std::setprecision(1) << sim->Time() << " s\n";
    std::cout << "  Position: (" << x / 1e6 << ", " << y / 1e6 << ", " << z / 1e6 << ") Mm\n";
    std::cout << "  Altitude: " << (r_final - R_earth) / 1000.0 << " km\n\n";

    std::cout << "✅ Simulation complete!\n";
    return 0;
}
