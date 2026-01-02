/**
 * @file aggregation_demo.cpp
 * @brief Multi-Body Aggregation Demo
 *
 * Demonstrates aggregation of multiple mass and force sources:
 * - MassAggregator: Combines mass, CG, and inertia using parallel axis theorem
 * - ForceAggregator: Sums forces and computes moment transfer about CG
 *
 * Scenario: A satellite with main body + two solar panel wings + reaction wheel
 * Each has separate mass properties at offset locations.
 *
 * Usage: ./aggregation_demo [config_path]
 *        Default: config/aggregation_demo.yaml
 */

#include <icarus/icarus.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace icarus;
namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       Icarus Multi-Body Aggregation Demo                   ║\n";
    std::cout << "║       MassAggregator + ForceAggregator                     ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // Config path - default or command line
    std::string config_path = "config/aggregation_demo.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    if (!fs::exists(config_path)) {
        std::cerr << "❌ Config not found: " << config_path << "\n";
        std::cerr << "   Run from project root or specify path as argument.\n";
        return 1;
    }

    // =========================================================================
    // Load and stage simulation
    // =========================================================================

    std::cout << "Loading: " << config_path << "\n";

    std::unique_ptr<Simulator> sim;
    try {
        sim = Simulator::FromConfig(config_path);
    } catch (const std::exception &e) {
        std::cerr << "❌ Error: " << e.what() << "\n";
        return 1;
    }

    sim->Stage();

    std::cout << "  Components: " << sim->NumComponents() << "\n";
    std::cout << "  State size: " << sim->GetState().size() << "\n";
    std::cout << "  Timestep: " << sim->Dt() << " s\n";
    std::cout << "  Duration: " << sim->EndTime() << " s\n\n";

    // =========================================================================
    // Query aggregated mass properties via signal peeking
    // =========================================================================

    // Run one step so aggregators compute their values
    sim->Step(sim->Dt());

    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Aggregated Mass Properties (from MassAggregator)          ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // Peek at aggregated values (Peek returns double directly)
    double total_mass = sim->Peek("TotalMass.total_mass");
    double cg_x_agg = sim->Peek("TotalMass.cg.x");
    double cg_y_agg = sim->Peek("TotalMass.cg.y");
    double cg_z_agg = sim->Peek("TotalMass.cg.z");
    double Ixx = sim->Peek("TotalMass.inertia.xx");
    double Iyy = sim->Peek("TotalMass.inertia.yy");
    double Izz = sim->Peek("TotalMass.inertia.zz");

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Component masses:\n";
    std::cout << "    MainBody:      100.0 kg at CG [0, 0, 0]\n";
    std::cout << "    LeftPanel:      10.0 kg at CG [0, +2, 0]\n";
    std::cout << "    RightPanel:     10.0 kg at CG [0, -2, 0]\n";
    std::cout << "    ReactionWheel:   5.0 kg at CG [+0.5, 0, 0]\n\n";

    std::cout << "  Aggregated total:\n";
    std::cout << "    Total mass:    " << total_mass << " kg\n";
    std::cout << "    Combined CG:   [" << cg_x_agg << ", " << cg_y_agg << ", " << cg_z_agg
              << "] m\n";
    std::cout << "    Inertia (diag): [" << Ixx << ", " << Iyy << ", " << Izz << "] kg*m^2\n";
    std::cout << "\n    Note: Inertia includes parallel axis terms for offset masses!\n";

    // =========================================================================
    // Expected calculations
    // =========================================================================

    std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Manual Verification                                       ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // Expected total mass
    double expected_mass = 100.0 + 10.0 + 10.0 + 5.0;
    std::cout << "  Expected total mass: " << expected_mass << " kg\n";

    // Expected CG (weighted average)
    double cg_x_exp = (100.0 * 0.0 + 10.0 * 0.0 + 10.0 * 0.0 + 5.0 * 0.5) / expected_mass;
    double cg_y_exp = (100.0 * 0.0 + 10.0 * 2.0 + 10.0 * (-2.0) + 5.0 * 0.0) / expected_mass;
    double cg_z_exp = 0.0;
    std::cout << "  Expected CG: [" << cg_x_exp << ", " << cg_y_exp << ", " << cg_z_exp << "] m\n";

    // Inertia with parallel axis theorem is complex - just note that aggregator does it
    std::cout << "  Inertia: Computed via parallel axis theorem by Vulcan\n\n";

    // =========================================================================
    // Run simulation
    // =========================================================================

    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  Running Simulation                                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "─────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(10) << "Time [s]" << std::setw(18) << "Altitude [km]" << std::setw(18)
              << "|F_total| [N]" << std::setw(15) << "|ω| [rad/s]" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";

    const double R_earth = 6.371e6; // Earth radius [m]
    int step = 0;
    const int print_interval = static_cast<int>(1.0 / sim->Dt()); // Every 1 second

    while (sim->Time() < sim->EndTime()) {
        sim->Step(sim->Dt());

        if (++step % print_interval == 0) {
            auto state = sim->GetState();

            // Position
            double x = state[0], y = state[1], z = state[2];
            double r = std::sqrt(x * x + y * y + z * z);
            double altitude = (r - R_earth) / 1000.0;

            // Angular velocity magnitude
            double wx = state[10], wy = state[11], wz = state[12];
            double omega_mag = std::sqrt(wx * wx + wy * wy + wz * wz);

            // Peek at total force
            double fx = sim->Peek("TotalForces.total_force.x");
            double fy = sim->Peek("TotalForces.total_force.y");
            double fz = sim->Peek("TotalForces.total_force.z");
            double f_mag = std::sqrt(fx * fx + fy * fy + fz * fz);

            std::cout << std::fixed << std::setprecision(2) << std::setw(10) << sim->Time()
                      << std::setprecision(1) << std::setw(18) << altitude << std::setprecision(1)
                      << std::setw(18) << f_mag << std::setprecision(4) << std::setw(15)
                      << omega_mag << "\n";
        }
    }

    std::cout << "─────────────────────────────────────────────────────────────\n\n";

    // =========================================================================
    // Final state
    // =========================================================================

    auto final_state = sim->GetState();
    double x = final_state[0], y = final_state[1], z = final_state[2];
    double r_final = std::sqrt(x * x + y * y + z * z);

    std::cout << "Final Results:\n";
    std::cout << "  Time:     " << sim->Time() << " s\n";
    std::cout << "  Altitude: " << (r_final - R_earth) / 1000.0 << " km\n";
    std::cout << "\n✅ Aggregation demo complete!\n";

    return 0;
}
