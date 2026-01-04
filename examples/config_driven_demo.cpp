/**
 * @file config_driven_demo.cpp
 * @brief Configuration-Driven Simulation Demo
 *
 * Demonstrates the clean Simulator API:
 * - Simulator::FromConfig("config.yaml") loads and configures
 * - Components auto-created from YAML via ComponentFactory
 * - Stage() and Step() lifecycle
 * - Symbolic dynamics graph generation (Phase C) - automatic via YAML config
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

// Components are auto-registered via icarus_components library (registration.cpp)

int main(int argc, char *argv[]) {

    // Config path - default or command line
    std::string config_path = "config/orbital_demo.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    // std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    // std::cout << "║       Icarus Configuration-Driven Demo                     ║\n";
    // std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

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

    // std::cout << "  Name: " << sim->Name() << "\n";
    // std::cout << "  Components: " << sim->NumComponents() << "\n";
    // std::cout << "  Timestep: " << sim->Dt() << " s\n";
    // std::cout << "  End time: " << sim->EndTime() << " s\n\n";

    // =========================================================================
    // Stage() - Prepare for execution
    // - Includes linearization if enabled in YAML
    // - Includes symbolic graph generation if enabled in YAML (Phase C)
    // =========================================================================

    std::cout << "Staging simulation...\n";
    sim->Stage();
    std::cout << "  State size: " << sim->GetState().size() << "\n\n";

    // =========================================================================
    // Run simulation
    // =========================================================================
    // Recording is automatic when enabled in YAML config

    double demo_duration = 5400.0; // 90 minutes (full orbit)

    while (sim->Time() < demo_duration) {
        sim->Step(sim->Dt());
    }

    // =========================================================================
    // Results
    // =========================================================================

    constexpr double R_earth = 6.371e6; // Earth radius [m]
    auto final_state = sim->GetState();
    double x = final_state[0], y = final_state[1], z = final_state[2];
    double r_final = std::sqrt(x * x + y * y + z * z);

    std::cout << "Final Results:\n";
    std::cout << "  Time: " << std::fixed << std::setprecision(1) << sim->Time() << " s\n";
    std::cout << "  Position: (" << x / 1e6 << ", " << y / 1e6 << ", " << z / 1e6 << ") Mm\n";
    std::cout << "  Altitude: " << (r_final - R_earth) / 1000.0 << " km\n\n";

    // =========================================================================
    // Symbolic Dynamics Report (Phase C)
    // =========================================================================
    // Symbolic generation happens automatically in Stage() if configured in YAML.
    // We just query the results here.

    std::cout << "Symbolic Mode (Phase C):\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";

    auto dynamics_graph = sim->GetDynamicsGraph();
    auto jacobian = sim->GetJacobian();

    if (dynamics_graph.has_value()) {
        const auto &f = dynamics_graph->casadi_function();
        std::cout << "  ✅ Dynamics function: " << f.name() << "\n";
        std::cout << "     Inputs:  " << f.n_in() << " (";
        for (casadi_int i = 0; i < f.n_in(); ++i) {
            if (i > 0)
                std::cout << ", ";
            std::cout << f.name_in(i) << "[" << f.nnz_in(i) << "]";
        }
        std::cout << ")\n";
        std::cout << "     Outputs: " << f.n_out() << " (";
        for (casadi_int i = 0; i < f.n_out(); ++i) {
            if (i > 0)
                std::cout << ", ";
            std::cout << f.name_out(i) << "[" << f.nnz_out(i) << "]";
        }
        std::cout << ")\n";

        // Export interactive HTML graph visualization
        fs::create_directories("output");
        try {
            janus::export_graph_deep(f, "output/dynamics_graph", janus::DeepGraphFormat::HTML,
                                     "Orbital Dynamics");
            std::cout << "  📊 Exported: output/dynamics_graph.html\n";
        } catch (const std::exception &e) {
            std::cout << "  ⚠️  Graph export failed: " << e.what() << "\n";
        }
    } else {
        std::cout << "  ⚠️  Dynamics graph not available (symbolics disabled in config)\n";
    }

    if (jacobian.has_value()) {
        const auto &J = jacobian->casadi_function();
        std::cout << "  ✅ State Jacobian: " << J.name() << "\n";

        // Evaluate at initial state to show dimensions
        Eigen::VectorXd x0(6);
        x0 << 6.871e6, 0.0, 0.0, 0.0, 7612.0, 0.0;
        auto J_eval = (*jacobian)(0.0, x0);
        if (!J_eval.empty() && J_eval[0].rows() > 0) {
            std::cout << "     Dimensions: " << J_eval[0].rows() << " × " << J_eval[0].cols()
                      << "\n";
        }
    } else {
        std::cout << "  ⚠️  Jacobian not available (symbolics.generate_jacobian disabled)\n";
    }

    std::cout << "─────────────────────────────────────────────────────────────\n\n";

    // std::cout << "✅ Simulation complete!\n";
    return 0;
}
