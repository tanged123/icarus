/**
 * @file orbital_demo.cpp
 * @brief Introduction to Icarus: Simulating a Low Earth Orbit (LEO).
 *
 * This example demonstrates:
 * 1. Creating a Simulator instance with logging.
 * 2. Adding the PointMass3DOF (dynamics) and PointMassGravity (environment) components.
 * 3. Initializing an ISS-like circular orbit.
 * 4. Running the simulation loop with progress display.
 * 5. Displaying mission debrief with profiling data.
 */

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>
#include <icarus/sim/Simulator.hpp>
#include <vulcan/core/Constants.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace icarus;
using namespace icarus::components;

int main() {
    // 1. Create a Simulator (using double for numeric mode)
    Simulator<double> sim;

    // Enable profiling to track component timing
    sim.SetProfilingEnabled(true);

    // Set log file (logs will be saved here)
    sim.SetLogFile("orbital_demo.log");

    // Get the logger and display startup splash
    auto &logger = sim.GetLogger();
    logger.LogStartup();

    // 2. Setup Orbital Parameters (ISS-like)
    const double altitude = 400e3; // 400 km
    const double r0 = vulcan::constants::earth::R_eq + altitude;
    const double mu = vulcan::constants::earth::mu;
    const double v_circular = std::sqrt(mu / r0);

    // 3. Create and configure components
    logger.BeginPhase(SimPhase::Provision);
    logger.LogEntityLoad("orbital_demo");

    // Component ordering: Gravity should be added before Dynamics
    // so that force is computed from trial position at each RK4 stage.
    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    gravity->SetModel(PointMassGravity<double>::Model::PointMass);

    auto vehicle = std::make_unique<PointMass3DOF<double>>(450000.0, "PointMass3DOF");
    vehicle->SetInitialPosition(r0, 0.0, 0.0);         // Start at +X
    vehicle->SetInitialVelocity(0.0, v_circular, 0.0); // Velocity in +Y

    // 4. Add to simulator
    sim.AddComponent(std::move(gravity));
    logger.LogComponentAdd("Gravity", "PointMassGravity", "defaults", false);

    sim.AddComponent(std::move(vehicle));
    logger.LogComponentAdd("PointMass3DOF", "PointMass3DOF", "defaults", true);

    // 5. Initialize lifecycle
    sim.Provision();
    logger.LogStateAllocation(sim.GetTotalStateSize());

    // Display Flight Manifest
    logger.LogManifest(sim.GetDataDictionary());
    logger.EndPhase();

    // Wire components together after Provision(), before Stage()
    logger.BeginPhase(SimPhase::Stage);

    // Gravity reads position and mass from vehicle
    sim.SetWiring("Gravity", {{"position.x", "PointMass3DOF.position.x"},
                              {"position.y", "PointMass3DOF.position.y"},
                              {"position.z", "PointMass3DOF.position.z"},
                              {"mass", "PointMass3DOF.mass"}});

    logger.LogWiring("PointMass3DOF.position.x", "Gravity.position.x");
    logger.LogWiring("PointMass3DOF.position.y", "Gravity.position.y");
    logger.LogWiring("PointMass3DOF.position.z", "Gravity.position.z");
    logger.LogWiring("PointMass3DOF.mass", "Gravity.mass");

    // Vehicle reads force from gravity
    sim.SetWiring("PointMass3DOF", {{"force.x", "Gravity.force.x"},
                                    {"force.y", "Gravity.force.y"},
                                    {"force.z", "Gravity.force.z"}});

    logger.LogWiring("Gravity.force.x", "PointMass3DOF.force.x");
    logger.LogWiring("Gravity.force.y", "PointMass3DOF.force.y");
    logger.LogWiring("Gravity.force.z", "PointMass3DOF.force.z");

    sim.Stage();
    logger.EndPhase();

    // 6. Run simulation loop
    logger.BeginPhase(SimPhase::Run);

    const double dt = 10.0;    // 10-second steps
    const double t_end = 1500; // 25 minutes

    auto wall_start = std::chrono::high_resolution_clock::now();

    while (sim.Time() <= t_end) {
        // Profile each step
        logger.BeginComponentTiming("Step");

        // Get altitude for progress display
        double x = sim.GetSignal("PointMass3DOF.position.x");
        double y = sim.GetSignal("PointMass3DOF.position.y");
        double z = sim.GetSignal("PointMass3DOF.position.z");
        double r = std::sqrt(x * x + y * y + z * z);
        double alt_km = (r - vulcan::constants::earth::R_eq) / 1000.0;

        // Update progress bar
        logger.UpdateProgress(sim.Time(), t_end, {{"Alt", alt_km}});

        sim.Step(dt);
        logger.EndComponentTiming();
    }

    logger.ClearProgress();
    logger.EndPhase();

    auto wall_end = std::chrono::high_resolution_clock::now();
    double wall_time = std::chrono::duration<double>(wall_end - wall_start).count();

    // 7. Shutdown with Mission Debrief
    logger.BeginPhase(SimPhase::Shutdown);
    logger.LogEvent("Simulation Complete", sim.Time(), "Satellite moved from (+X) towards (+Y)");

    // Display mission debrief with profiling stats
    logger.LogDebrief(sim.Time(), wall_time);

    // 8. Export Data Dictionary
    sim.GenerateDataDictionary("orbital_demo_datadict.yaml");
    sim.GenerateDataDictionary("orbital_demo_datadict.json");
    logger.LogTimed(LogLevel::Info, sim.Time(),
                    "[IO] Saved: orbital_demo_datadict.yaml, orbital_demo_datadict.json");

    logger.EndPhase();

    return 0;
}
