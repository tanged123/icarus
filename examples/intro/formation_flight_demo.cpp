/**
 * @file formation_flight_demo.cpp
 * @brief Demonstrates entity naming with multiple satellites
 *
 * This example shows how entity names allow multiple instances of the same
 * component type for different vehicles in formation flight simulation.
 *
 * Signals are prefixed with Entity.Component, e.g.:
 *   - Leader.Dynamics.position.x
 *   - Follower.Dynamics.position.x
 */

#include <icarus/icarus.hpp>

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <vulcan/core/Constants.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace icarus;
using namespace icarus::components;

int main() {
    std::cout << "=== Formation Flight Demo (Entity Names) ===\n\n";

    // =========================================================================
    // Orbital Parameters
    // =========================================================================
    double altitude = 400e3; // 400 km (ISS-like)
    double r0 = vulcan::constants::earth::R_eq + altitude;
    double mu = vulcan::constants::earth::mu;
    double v_circular = std::sqrt(mu / r0);

    // Formation separation: 100m in-track
    double separation = 100.0; // meters

    // =========================================================================
    // Create Simulator with Multi-Entity Components
    // =========================================================================
    Simulator<double> sim;

    // --- Leader Satellite ---
    // Entity = "Leader", provides unique signal namespace
    auto leader_pm = std::make_unique<PointMass3DOF<double>>(
        500.0,      // mass (kg)
        "Dynamics", // component name
        "Leader"    // entity name -> signals: Leader.Dynamics.*
    );
    leader_pm->SetInitialPosition(r0, 0.0, 0.0);
    leader_pm->SetInitialVelocity(0.0, v_circular, 0.0);

    auto leader_grav = std::make_unique<PointMassGravity<double>>(
        "Gravity", // component name
        "Leader",  // entity name -> signals: Leader.Gravity.*
        PointMassGravity<double>::Model::PointMass);

    // --- Follower Satellite ---
    // Entity = "Follower", provides separate signal namespace
    auto follower_pm = std::make_unique<PointMass3DOF<double>>(
        500.0,       // mass (kg)
        "Dynamics",  // same component name
        "Follower"); // different entity -> signals: Follower.Dynamics.*

    // Start 100m behind leader (in velocity direction)
    follower_pm->SetInitialPosition(r0, -separation, 0.0);
    follower_pm->SetInitialVelocity(0.0, v_circular, 0.0);

    auto follower_grav = std::make_unique<PointMassGravity<double>>(
        "Gravity", "Follower", PointMassGravity<double>::Model::PointMass);

    // =========================================================================
    // Add Components (Gravity before Dynamics for correct ordering!)
    // =========================================================================
    sim.AddComponent(std::move(leader_grav));
    sim.AddComponent(std::move(leader_pm));
    sim.AddComponent(std::move(follower_grav));
    sim.AddComponent(std::move(follower_pm));

    // Run lifecycle
    sim.Provision();

    // =========================================================================
    // Wire Components (after Provision, before Stage)
    // =========================================================================
    // Each entity has its own internal wiring
    auto wire_entity = [&sim](const std::string &entity) {
        std::string grav = entity + ".Gravity";
        std::string dyn = entity + ".Dynamics";

        // Gravity reads position and mass from Dynamics
        sim.SetWiring(grav, {{"position.x", dyn + ".position.x"},
                             {"position.y", dyn + ".position.y"},
                             {"position.z", dyn + ".position.z"},
                             {"mass", dyn + ".mass"}});

        // Dynamics reads force from Gravity
        sim.SetWiring(dyn, {{"force.x", grav + ".force.x"},
                            {"force.y", grav + ".force.y"},
                            {"force.z", grav + ".force.z"}});
    };

    wire_entity("Leader");
    wire_entity("Follower");

    sim.Stage();

    // Show component full names (Entity.Component naming)
    std::cout << "Satellites in formation:\n";
    std::cout << "  - Leader.Gravity   (PointMassGravity)\n";
    std::cout << "  - Leader.Dynamics  (PointMass3DOF)\n";
    std::cout << "  - Follower.Gravity (PointMassGravity)\n";
    std::cout << "  - Follower.Dynamics (PointMass3DOF)\n";
    std::cout << "\n";

    // =========================================================================
    // Simulate Formation Flight
    // =========================================================================
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Time[s]   Leader(X,Y) [km]     Follower(X,Y) [km]   Separation[m]\n";
    std::cout << "------   -----------------    ------------------   -------------\n";

    double dt = 10.0;        // 10 second time steps
    double t_end = 600.0;    // 10 minutes
    int print_interval = 12; // Print every 2 minutes

    int step = 0;
    while (sim.Time() < t_end) {
        sim.Step(dt);
        step++;

        if (step % print_interval == 0) {
            // Use GetSignal to read position values
            double lx = sim.GetSignal("Leader.Dynamics.position.x") / 1000.0;   // km
            double ly = sim.GetSignal("Leader.Dynamics.position.y") / 1000.0;   // km
            double fx = sim.GetSignal("Follower.Dynamics.position.x") / 1000.0; // km
            double fy = sim.GetSignal("Follower.Dynamics.position.y") / 1000.0; // km

            // Compute separation in meters
            double dx = (lx - fx) * 1000.0;
            double dy = (ly - fy) * 1000.0;
            double sep = std::sqrt(dx * dx + dy * dy);

            std::cout << std::setw(5) << sim.Time() << "   (" << std::setw(7) << lx << ", "
                      << std::setw(7) << ly << ")   (" << std::setw(7) << fx << ", " << std::setw(7)
                      << fy << ")   " << std::setw(8) << sep << "\n";
        }
    }

    std::cout << "\nFormation flight complete!\n";
    std::cout << "Both satellites maintain ~" << separation << "m separation in circular orbit.\n";

    // =========================================================================
    // Export Data Dictionary
    // =========================================================================
    std::cout << "\nExporting Data Dictionary...\n";
    sim.GenerateDataDictionary("formation_datadict.yaml");
    std::cout << "Saved: formation_datadict.yaml\n";

    return 0;
}
