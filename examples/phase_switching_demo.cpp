/**
 * @file phase_switching_demo.cpp
 * @brief Phase Switching Demonstration
 *
 * Loads phase_demo.yaml and runs a multi-phase orbital simulation.
 * Phase transitions and component gating are defined entirely in YAML.
 *
 * Usage: ./phase_switching_demo [config_path]
 *        Default: config/phase_demo.yaml
 */

#include <icarus/icarus.hpp>

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace icarus;
namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    // Config path - default or command line
    std::string config_path = "config/phase_demo.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       Icarus Phase Switching Demo                          ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    if (!fs::exists(config_path)) {
        std::cerr << "❌ Config not found: " << config_path << "\n";
        std::cerr << "   Run from project root or specify path as argument.\n";
        return 1;
    }

    // =========================================================================
    // Load from YAML - all configuration including phases is in the YAML file
    // =========================================================================

    std::cout << "Loading: " << config_path << "\n";

    std::unique_ptr<Simulator> sim;
    try {
        sim = Simulator::FromConfig(config_path);
    } catch (const std::exception &e) {
        std::cerr << "❌ Error: " << e.what() << "\n";
        return 1;
    }

    // Stage the simulation
    std::cout << "Staging simulation...\n";
    sim->Stage();
    std::cout << "  State size: " << sim->GetState().size() << "\n";
    std::cout << "  Initial phase: " << sim->GetFlightPhaseName() << "\n\n";

    // =========================================================================
    // Run simulation
    // =========================================================================

    const double R_EARTH = 6.371e6;

    std::cout << "Expected Phase Transitions:\n";
    std::cout << "  COAST_1 → BURN:   When position.y > 3000 m\n";
    std::cout << "  BURN → COAST_2:   When position.y > 15000 m\n\n";

    std::cout << std::setw(10) << "Time [s]" << std::setw(12) << "Phase" << std::setw(15)
              << "Altitude [km]" << std::setw(15) << "Speed [m/s]"
              << "\n";
    std::cout << std::string(52, '-') << "\n";

    int step = 0;
    std::string last_phase = sim->GetFlightPhaseName();

    while (sim->Time() < sim->EndTime()) {
        sim->Step();

        // Print every 5 seconds or if phase changed
        std::string current_phase = sim->GetFlightPhaseName();
        bool phase_changed = (current_phase != last_phase);

        if (phase_changed || (++step % static_cast<int>(5.0 / sim->Dt()) == 0)) {
            auto state = sim->GetState();
            double x = state[0], y = state[1], z = state[2];
            double vx = state[3], vy = state[4], vz = state[5];

            double r = std::sqrt(x * x + y * y + z * z);
            double altitude = (r - R_EARTH) / 1000.0;
            double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

            std::cout << std::fixed << std::setprecision(1);
            std::cout << std::setw(10) << sim->Time() << std::setw(12) << current_phase
                      << std::setw(15) << altitude << std::setw(15) << speed << "\n";

            last_phase = current_phase;
        }
    }

    std::cout << std::string(52, '-') << "\n\n";
    std::cout << "Final phase: " << sim->GetFlightPhaseName() << "\n";
    std::cout << "✅ Simulation complete!\n";
    return 0;
}
