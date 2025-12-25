/**
 * @file orbital_demo.cpp
 * @brief Introduction to Icarus: Simulating a Low Earth Orbit (LEO).
 *
 * This example demonstrates:
 * 1. Creating a Simulator instance.
 * 2. Adding the PointMass3DOF (dynamics) and PointMassGravity (environment) components.
 * 3. Initializing an ISS-like circular orbit.
 * 4. Running the simulation loop and inspecting signal values.
 */

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>
#include <icarus/sim/Simulator.hpp>
#include <vulcan/core/Constants.hpp>

#include <iomanip>
#include <iostream>

using namespace icarus;
using namespace icarus::components;

int main() {
    std::cout << "====================================================\n";
    std::cout << "       Icarus Intro: Low Earth Orbit Demo           \n";
    std::cout << "====================================================\n\n";

    // 1. Create a Simulator (using double for numeric mode)
    Simulator<double> sim;

    // 2. Setup Orbital Parameters (ISS-like)
    const double altitude = 400e3; // 400 km
    const double r0 = vulcan::constants::earth::R_eq + altitude;
    const double mu = vulcan::constants::earth::mu;
    const double v_circular = std::sqrt(mu / r0);

    // 3. Create and configure components
    // Component ordering: Gravity should be added before Dynamics
    // so that force is computed from trial position at each RK4 stage.
    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    gravity->SetModel(PointMassGravity<double>::Model::PointMass);

    auto vehicle = std::make_unique<PointMass3DOF<double>>(450000.0, "PointMass3DOF");
    vehicle->SetInitialPosition(r0, 0.0, 0.0);         // Start at +X
    vehicle->SetInitialVelocity(0.0, v_circular, 0.0); // Velocity in +Y

    // 4. Add to simulator
    sim.AddComponent(std::move(gravity));
    sim.AddComponent(std::move(vehicle));

    // 5. Initialize lifecycle
    std::cout << "Provisioning and Staging components...\n";
    sim.Provision();

    // Wire components together after Provision(), before Stage()
    // Gravity reads position and mass from vehicle
    sim.Wire<double>("Gravity.position.x", "PointMass3DOF.position.x");
    sim.Wire<double>("Gravity.position.y", "PointMass3DOF.position.y");
    sim.Wire<double>("Gravity.position.z", "PointMass3DOF.position.z");
    sim.Wire<double>("Gravity.mass", "PointMass3DOF.mass");

    // Vehicle reads force from gravity
    sim.Wire<double>("PointMass3DOF.force.x", "Gravity.force.x");
    sim.Wire<double>("PointMass3DOF.force.y", "Gravity.force.y");
    sim.Wire<double>("PointMass3DOF.force.z", "Gravity.force.z");

    sim.Stage();

    // 6. Run simulation loop
    std::cout << "\nStarting orbit simulation (1/4 orbit)...\n";
    std::cout << std::setw(10) << "Time [s]" << std::setw(15) << "Alt [km]" << std::setw(15)
              << "Vel [m/s]" << std::setw(15) << "Force [kN]" << "\n";
    std::cout << std::string(55, '-') << "\n";

    const double dt = 10.0;    // 10-second steps
    const double t_end = 1500; // 25 minutes

    while (sim.Time() <= t_end) {
        // Log current values from the backplane
        double x = sim.GetSignal("PointMass3DOF.position.x");
        double y = sim.GetSignal("PointMass3DOF.position.y");
        double z = sim.GetSignal("PointMass3DOF.position.z");
        double r = std::sqrt(x * x + y * y + z * z);

        double vx = sim.GetSignal("PointMass3DOF.velocity.x");
        double vy = sim.GetSignal("PointMass3DOF.velocity.y");
        double vz = sim.GetSignal("PointMass3DOF.velocity.z");
        double v = std::sqrt(vx * vx + vy * vy + vz * vz);

        double fx = sim.GetSignal("Gravity.force.x");
        double fy = sim.GetSignal("Gravity.force.y");
        double fz = sim.GetSignal("Gravity.force.z");
        double f = std::sqrt(fx * fx + fy * fy + fz * fz);

        std::cout << std::fixed << std::setprecision(1) << std::setw(10) << sim.Time()
                  << std::setw(15) << (r - vulcan::constants::earth::R_eq) / 1000.0 << std::setw(15)
                  << v << std::setw(15) << f / 1000.0 << "\n";

        sim.Step(dt);
    }

    std::cout << "\nSimulation Complete. Satellite moved from (+X) towards (+Y).\n";

    // 7. Export Data Dictionary
    std::cout << "\nExporting Data Dictionary...\n";
    sim.GenerateDataDictionary("orbital_demo_datadict.yaml");
    sim.GenerateDataDictionary("orbital_demo_datadict.json");
    std::cout << "Saved: orbital_demo_datadict.yaml, orbital_demo_datadict.json\n";

    return 0;
}
