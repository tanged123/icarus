/**
 * @file point_mass_freefall.cpp
 * @brief Simple Point Mass Free-Fall Example
 *
 * Demonstrates the Phase 4.0.7 Simulator API:
 * - Programmatic component setup
 * - Signal routing with AddRoutes()
 * - Stage() and Step() lifecycle
 * - State inspection via Peek()
 *
 * Physics: A 1kg mass falls from 100m under constant gravity.
 * Validates against analytical solution: z(t) = z₀ - ½gt²
 */

#include <icarus/icarus.hpp>

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace icarus;
using namespace icarus::components;

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       Icarus Point Mass Free-Fall Demo                     ║\n";
    std::cout << "║       Phase 4.0.7 Simulator API                            ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // =========================================================================
    // Setup
    // =========================================================================

    Simulator sim;

    // Create components
    auto grav = std::make_unique<PointMassGravity<double>>("Gravity");
    auto pm = std::make_unique<PointMass3DOF<double>>(1.0, "PointMass");

    // Configure gravity model
    grav->SetModel(PointMassGravity<double>::Model::Constant);

    // Initial conditions: start at 100m, at rest
    const double z0 = 100.0;
    pm->SetInitialPosition(0.0, 0.0, z0);
    pm->SetInitialVelocity(0.0, 0.0, 0.0);

    // Add components (order determines execution priority)
    sim.AddComponent(std::move(grav));
    sim.AddComponent(std::move(pm));

    // Wire signals: Gravity reads position/mass from PointMass, PointMass reads force from Gravity
    std::vector<signal::SignalRoute> routes = {
        {"Gravity.position.x", "PointMass.position.x"},
        {"Gravity.position.y", "PointMass.position.y"},
        {"Gravity.position.z", "PointMass.position.z"},
        {"Gravity.mass", "PointMass.mass"},
        {"PointMass.force.x", "Gravity.force.x"},
        {"PointMass.force.y", "Gravity.force.y"},
        {"PointMass.force.z", "Gravity.force.z"},
    };
    sim.AddRoutes(routes);

    // Stage the simulation (provisions + wires components)
    sim.Stage();
    std::cout << "Simulation staged. Components: " << sim.NumComponents() << "\n";
    std::cout << "Initial state size: " << sim.GetState().size() << " (6 = x,y,z,vx,vy,vz)\n\n";

    // =========================================================================
    // Run Simulation
    // =========================================================================

    const double dt = 0.01;   // 100 Hz
    const double t_end = 4.0; // 4 seconds (hits ground at ~4.5s)
    const double g = vulcan::constants::physics::g0;

    std::cout << "Running simulation for " << t_end << " seconds at dt=" << dt << "\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(10) << "Time [s]" << std::setw(15) << "z [m]" << std::setw(15)
              << "z_exact [m]" << std::setw(15) << "Error [m]" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";

    int step = 0;
    while (sim.Time() < t_end) {
        sim.Step(dt);

        // Print every 50 steps (0.5s intervals)
        if (++step % 50 == 0) {
            double t = sim.Time();
            auto state = sim.GetState();
            double z = state[2]; // Position Z

            // Analytical: z(t) = z₀ - ½gt²
            double z_exact = z0 - 0.5 * g * t * t;
            double error = z - z_exact;

            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(10) << t << std::setw(15) << z << std::setw(15) << z_exact
                      << std::setw(15) << std::scientific << std::setprecision(2) << error << "\n";
        }
    }

    std::cout << "─────────────────────────────────────────────────────────────\n\n";

    // =========================================================================
    // Results
    // =========================================================================

    double t_final = sim.Time();
    auto final_state = sim.GetState();
    double z_final = final_state[2];
    double vz_final = final_state[5];

    double z_exact = z0 - 0.5 * g * t_final * t_final;
    double v_exact = -g * t_final;

    std::cout << "Final Results:\n";
    std::cout << "  Time:     " << std::fixed << std::setprecision(4) << t_final << " s\n";
    std::cout << "  Position: " << z_final << " m (exact: " << z_exact << " m)\n";
    std::cout << "  Velocity: " << vz_final << " m/s (exact: " << v_exact << " m/s)\n";
    std::cout << "  Position error: " << std::scientific << std::setprecision(2)
              << std::abs(z_final - z_exact) << " m\n";
    std::cout << "  Velocity error: " << std::abs(vz_final - v_exact) << " m/s\n\n";

    bool passed = std::abs(z_final - z_exact) < 1e-8 && std::abs(vz_final - v_exact) < 1e-8;

    if (passed) {
        std::cout << "✅ PASSED: RK4 integration matches analytical solution!\n";
    } else {
        std::cout << "❌ FAILED: Numerical error exceeds tolerance\n";
    }

    return passed ? 0 : 1;
}
