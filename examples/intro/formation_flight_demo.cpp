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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace icarus;
using namespace icarus::components;

int main() {
    // =========================================================================
    // Create Simulator with Logging
    // =========================================================================
    Simulator<double> sim;

    // Enable profiling and log file
    sim.SetProfilingEnabled(true);
    sim.SetLogFile("formation_flight.log");

    auto &logger = sim.GetLogger();
    logger.LogStartup();

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
    // Create Multi-Entity Components
    // =========================================================================
    logger.BeginPhase(SimPhase::Provision);

    // --- Leader Satellite ---
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
    logger.LogEntityLoad("Leader");
    sim.AddComponent(std::move(leader_grav));
    logger.LogComponentAdd("Leader.Gravity", "PointMassGravity");

    sim.AddComponent(std::move(leader_pm));
    logger.LogComponentAdd("Leader.Dynamics", "PointMass3DOF");

    logger.LogEntityLoad("Follower");
    sim.AddComponent(std::move(follower_grav));
    logger.LogComponentAdd("Follower.Gravity", "PointMassGravity");

    sim.AddComponent(std::move(follower_pm));
    logger.LogComponentAdd("Follower.Dynamics", "PointMass3DOF");

    sim.Provision();
    logger.LogStateAllocation(sim.GetTotalStateSize());

    // Display Flight Manifest
    logger.LogManifest(sim.GetDataDictionary());
    logger.EndPhase();

    // =========================================================================
    // Wire Components (after Provision, before Stage)
    // =========================================================================
    logger.BeginPhase(SimPhase::Stage);

    auto wire_entity = [&sim, &logger](const std::string &entity) {
        std::string grav = entity + ".Gravity";
        std::string dyn = entity + ".Dynamics";

        // Gravity reads position and mass from Dynamics
        sim.SetWiring(grav, {{"position.x", dyn + ".position.x"},
                             {"position.y", dyn + ".position.y"},
                             {"position.z", dyn + ".position.z"},
                             {"mass", dyn + ".mass"}});

        logger.LogWiring(dyn + ".position.x", grav + ".position.x");
        logger.LogWiring(dyn + ".position.y", grav + ".position.y");
        logger.LogWiring(dyn + ".position.z", grav + ".position.z");
        logger.LogWiring(dyn + ".mass", grav + ".mass");

        // Dynamics reads force from Gravity
        sim.SetWiring(dyn, {{"force.x", grav + ".force.x"},
                            {"force.y", grav + ".force.y"},
                            {"force.z", grav + ".force.z"}});

        logger.LogWiring(grav + ".force.x", dyn + ".force.x");
        logger.LogWiring(grav + ".force.y", dyn + ".force.y");
        logger.LogWiring(grav + ".force.z", dyn + ".force.z");
    };

    wire_entity("Leader");
    wire_entity("Follower");

    sim.Stage();
    logger.EndPhase();

    // =========================================================================
    // Simulate Formation Flight
    // =========================================================================
    logger.BeginPhase(SimPhase::Run);

    double dt = 10.0;     // 10 second time steps
    double t_end = 600.0; // 10 minutes

    auto wall_start = std::chrono::high_resolution_clock::now();

    while (sim.Time() < t_end) {
        logger.BeginComponentTiming("Step");

        // Compute separation for progress display
        double lx = sim.GetSignal("Leader.Dynamics.position.x");
        double ly = sim.GetSignal("Leader.Dynamics.position.y");
        double fx = sim.GetSignal("Follower.Dynamics.position.x");
        double fy = sim.GetSignal("Follower.Dynamics.position.y");
        double dx = lx - fx;
        double dy = ly - fy;
        double sep = std::sqrt(dx * dx + dy * dy);

        logger.UpdateProgress(sim.Time(), t_end, {{"Sep", sep}});

        sim.Step(dt);
        logger.EndComponentTiming();
    }

    logger.ClearProgress();
    logger.EndPhase(sim.Time());

    auto wall_end = std::chrono::high_resolution_clock::now();
    double wall_time = std::chrono::duration<double>(wall_end - wall_start).count();

    // =========================================================================
    // Shutdown with Mission Debrief
    // =========================================================================
    logger.BeginPhase(SimPhase::Shutdown);
    logger.LogEvent("Formation Flight Complete", sim.Time(),
                    "Both satellites maintained ~100m separation");

    logger.LogDebrief(sim.Time(), wall_time);

    // Export Data Dictionary
    sim.GenerateDataDictionary("formation_datadict.yaml");
    logger.LogTimed(LogLevel::Info, sim.Time(), "[IO] Saved: formation_datadict.yaml");

    logger.EndPhase(sim.Time());

    return 0;
}
