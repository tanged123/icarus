/**
 * @file topology_demo.cpp
 * @brief Topology Analyzer Demo
 *
 * Demonstrates automatic execution order from signal dependencies.
 *
 * The config file has scheduler groups in WRONG order:
 *   Body (priority 1) -> Forces (priority 2) -> Mass (priority 3)
 *
 * But the routes define dependencies:
 *   Mass outputs -> Forces inputs (CG)
 *   Mass outputs -> Body inputs (mass/inertia)
 *   Forces outputs -> Body inputs (force/moment)
 *
 * With `topology.mode: Automatic`, the Simulator automatically computes
 * the correct order during Stage(): Mass -> Forces -> Body
 *
 * Usage: ./topology_demo [config_path]
 *        Default: config/topology_demo.yaml
 */

#include <icarus/icarus.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace icarus;
namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    std::cout << "======================================================\n";
    std::cout << "       Icarus Topology Analyzer Demo\n";
    std::cout << "       Automatic Execution Order from Dependencies\n";
    std::cout << "======================================================\n\n";

    // Config path - default or command line
    std::string config_path = "config/topology_demo.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    if (!fs::exists(config_path)) {
        std::cerr << "Error: Config not found: " << config_path << "\n";
        std::cerr << "   Run from project root or specify path as argument.\n";
        return 1;
    }

    std::cout << "Loading: " << config_path << "\n";
    std::cout << "  topology.mode: Automatic\n";
    std::cout << "  (execution order computed from signal dependencies)\n\n";

    // =========================================================================
    // Load and stage simulation
    // =========================================================================
    // The magic happens in Stage():
    // 1. TopologyAnalyzer builds dependency graph from routes
    // 2. Kahn's algorithm computes topological order
    // 3. Scheduler groups are reordered automatically
    // 4. "[TOPO] Computed execution order: ..." is logged

    std::unique_ptr<Simulator> sim;
    try {
        sim = Simulator::FromConfig(config_path);
    } catch (const std::exception &e) {
        std::cerr << "Error loading: " << e.what() << "\n";
        return 1;
    }

    std::cout << "------------------------------------------------------\n";
    std::cout << "Staging (topology analysis happens here)...\n";
    std::cout << "------------------------------------------------------\n";

    sim->Stage();

    std::cout << "\n";
    std::cout << "Simulation staged successfully!\n";
    std::cout << "  Components: " << sim->NumComponents() << "\n";
    std::cout << "  State size: " << sim->GetState().size() << "\n";
    std::cout << "  Timestep:   " << sim->Dt() << " s\n";
    std::cout << "  Duration:   " << sim->EndTime() << " s\n\n";

    // =========================================================================
    // Run simulation
    // =========================================================================

    std::cout << "------------------------------------------------------\n";
    std::cout << "Running simulation...\n";
    std::cout << "------------------------------------------------------\n";
    std::cout << std::setw(10) << "Time [s]" << std::setw(12) << "omega_x" << std::setw(12)
              << "omega_y" << std::setw(12) << "omega_z" << "\n";

    int step = 0;
    int print_interval = static_cast<int>(1.0 / sim->Dt());

    while (sim->Time() < sim->EndTime()) {
        sim->Step(sim->Dt());

        if (++step % print_interval == 0) {
            auto state = sim->GetState();
            std::cout << std::fixed << std::setprecision(2) << std::setw(10) << sim->Time()
                      << std::setprecision(4) << std::setw(12) << state[10] << std::setw(12)
                      << state[11] << std::setw(12) << state[12] << "\n";
        }
    }

    std::cout << "\n";
    std::cout << "======================================================\n";
    std::cout << "Demo complete!\n";
    std::cout << "\n";
    std::cout << "The TopologyAnalyzer automatically computed the correct\n";
    std::cout << "execution order from signal dependencies:\n";
    std::cout << "  Mass -> Forces -> Body\n";
    std::cout << "\n";
    std::cout << "Even though the YAML had them in wrong order:\n";
    std::cout << "  Body (priority 1) -> Forces (priority 2) -> Mass (priority 3)\n";
    std::cout << "======================================================\n";

    return 0;
}
