/**
 * @file reentry_dynamics_demo.cpp
 * @brief Complex scenario demo: Atmospheric reentry with gravity + drag
 *
 * This demo creates a more complex deep graph showing:
 * - Gravitational acceleration (μ/r³ computation)
 * - Atmospheric density (exponential model)
 * - Aerodynamic drag (velocity-dependent force)
 */

#include <dynamics/PointMass3DOF.hpp>
#include <environment/AtmosphericDrag.hpp>
#include <environment/PointMassGravity.hpp>

#include <icarus/icarus.hpp>

#include <janus/core/JanusIO.hpp>

#include <vulcan/core/Constants.hpp>

#include <cmath>
#include <iostream>

using namespace icarus;
using namespace icarus::components;

int main() {
    // =========================================================================
    // Create Simulator with Logging
    // =========================================================================
    Simulator<double> sim;
    sim.SetProfilingEnabled(true);
    sim.SetLogFile("reentry_demo.log");

    auto &logger = sim.GetLogger();
    logger.LogStartup();

    // =========================================================================
    // Reentry Vehicle Parameters
    // =========================================================================
    const double mass = 1000.0;       // 1 ton vehicle
    const double Cd = 1.5;            // Drag coefficient
    const double area = 2.0;          // Reference area [m²]
    const double altitude_km = 120.0; // 120 km (entry interface)
    const double r0 = vulcan::constants::earth::R_eq + altitude_km * 1000.0;
    const double mu = vulcan::constants::earth::mu;
    const double v_circular = std::sqrt(mu / r0);
    const double v_entry = v_circular * 0.99;

    // =========================================================================
    // Part 1: Numeric Simulation Setup
    // =========================================================================
    logger.BeginPhase(SimPhase::Provision);
    logger.LogEntityLoad("reentry_demo");

    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    gravity->SetModel(PointMassGravity<double>::Model::PointMass);

    auto drag = std::make_unique<AtmosphericDrag<double>>("Drag");
    drag->SetDragCoefficient(Cd);
    drag->SetReferenceArea(area);

    auto vehicle = std::make_unique<PointMass3DOF<double>>(mass, "Dynamics");
    vehicle->SetInitialPosition(r0, 0.0, 0.0);
    vehicle->SetInitialVelocity(0.0, v_entry, 0.0);

    sim.AddComponent(std::move(gravity));
    logger.LogComponentAdd("Gravity", "PointMassGravity", "point_mass", false);

    sim.AddComponent(std::move(drag));
    logger.LogComponentAdd("Drag", "AtmosphericDrag", "Cd=" + std::to_string(Cd), false);

    sim.AddComponent(std::move(vehicle));
    logger.LogComponentAdd("Dynamics", "PointMass3DOF", "mass=" + std::to_string(mass), true);

    sim.Provision();
    logger.LogStateAllocation(sim.GetTotalStateSize());
    logger.LogManifest(sim.GetDataDictionary());
    logger.EndPhase();

    // =========================================================================
    // Wire Components
    // =========================================================================
    logger.BeginPhase(SimPhase::Stage);

    sim.SetWiring("Gravity", {{"position.x", "Dynamics.position.x"},
                              {"position.y", "Dynamics.position.y"},
                              {"position.z", "Dynamics.position.z"},
                              {"mass", "Dynamics.mass"}});

    sim.SetWiring("Drag", {{"position.x", "Dynamics.position.x"},
                           {"position.y", "Dynamics.position.y"},
                           {"position.z", "Dynamics.position.z"},
                           {"velocity.x", "Dynamics.velocity.x"},
                           {"velocity.y", "Dynamics.velocity.y"},
                           {"velocity.z", "Dynamics.velocity.z"}});

    sim.SetWiring("Dynamics", {{"force.x", "Gravity.force.x"},
                               {"force.y", "Gravity.force.y"},
                               {"force.z", "Gravity.force.z"}});

    sim.Stage();
    logger.EndPhase();

    // =========================================================================
    // Part 2: Symbolic Graph Extraction
    // =========================================================================
    logger.Log(LogLevel::Info, "");
    logger.Log(LogLevel::Info, "=== Symbolic Mode (Reentry Dynamics) ===");

    Simulator<SymbolicScalar> sym_sim;
    sym_sim.SetQuietMode(true);
    {
        auto sym_gravity = std::make_unique<PointMassGravity<SymbolicScalar>>("Gravity");
        sym_gravity->SetModel(PointMassGravity<SymbolicScalar>::Model::PointMass);

        auto sym_drag = std::make_unique<AtmosphericDrag<SymbolicScalar>>("Drag");
        sym_drag->SetDragCoefficient(SymbolicScalar(Cd));
        sym_drag->SetReferenceArea(SymbolicScalar(area));

        auto sym_vehicle =
            std::make_unique<PointMass3DOF<SymbolicScalar>>(SymbolicScalar(mass), "Dynamics");

        sym_sim.AddComponent(std::move(sym_gravity));
        sym_sim.AddComponent(std::move(sym_drag));
        sym_sim.AddComponent(std::move(sym_vehicle));
        sym_sim.Provision();

        sym_sim.SetWiring("Gravity", {{"position.x", "Dynamics.position.x"},
                                      {"position.y", "Dynamics.position.y"},
                                      {"position.z", "Dynamics.position.z"},
                                      {"mass", "Dynamics.mass"}});

        sym_sim.SetWiring("Drag", {{"position.x", "Dynamics.position.x"},
                                   {"position.y", "Dynamics.position.y"},
                                   {"position.z", "Dynamics.position.z"},
                                   {"velocity.x", "Dynamics.velocity.x"},
                                   {"velocity.y", "Dynamics.velocity.y"},
                                   {"velocity.z", "Dynamics.velocity.z"}});

        sym_sim.SetWiring("Dynamics", {{"force.x", "Drag.force.x"},
                                       {"force.y", "Drag.force.y"},
                                       {"force.z", "Drag.force.z"}});
        sym_sim.Stage();
    }

    // Extract dynamics
    symbolic::SymbolicTracer tracer(sym_sim);
    auto dynamics = tracer.TraceDynamics();

    logger.Log(LogLevel::Info, "[Graph] Extracted dynamics: " + dynamics.casadi_function().name());
    logger.Log(LogLevel::Info, "[Graph] State dimension: " + std::to_string(tracer.GetStateSize()));

    // Export shallow graph (MX-based)
    auto t_vis = janus::sym("t");
    auto [x_vis_vec, x_vis_mx] = janus::sym_vec_pair("x", 6);
    auto xdot_vis = dynamics.eval(t_vis, x_vis_mx);

    janus::export_graph_html(janus::as_mx(xdot_vis), "reentry_shallow", "ReentryDynamics_Shallow");
    logger.Log(LogLevel::Info, "[Graph] Exported: reentry_shallow.html");

    // Export DEEP graph (SX-based, all operations expanded)
    janus::export_graph_deep(dynamics.casadi_function(), "reentry_deep",
                             janus::DeepGraphFormat::HTML, "ReentryDynamics_Deep");
    janus::export_graph_deep(dynamics.casadi_function(), "reentry_deep",
                             janus::DeepGraphFormat::DOT, "ReentryDynamics_Deep");
    logger.Log(LogLevel::Info, "[Graph] Exported: reentry_deep.html (deep view)");

    // =========================================================================
    // Summary
    // =========================================================================
    logger.BeginPhase(SimPhase::Shutdown);
    logger.LogEvent("Reentry Demo Complete", 0.0);

    std::cout << "\n=== Reentry Dynamics Demo Summary ===" << std::endl;
    std::cout << "  Vehicle mass:     " << mass << " kg" << std::endl;
    std::cout << "  Drag coefficient: " << Cd << std::endl;
    std::cout << "  Reference area:   " << area << " m²" << std::endl;
    std::cout << "  Initial altitude: " << altitude_km << " km" << std::endl;
    std::cout << std::endl;
    std::cout << "Generated files:" << std::endl;
    std::cout << "  - reentry_shallow.html (MX graph)" << std::endl;
    std::cout << "  - reentry_deep.html    (SX graph, all operations expanded)" << std::endl;

    logger.EndPhase(0.0);

    return 0;
}
