/**
 * @file symbolic_orbital_demo.cpp
 * @brief Demonstrates symbolic mode and graph extraction
 *
 * This example shows Icarus's dual-mode capability:
 * 1. Run simulation numerically (double mode)
 * 2. Extract computational graph symbolically (SymbolicScalar mode)
 * 3. Compute Jacobian for sensitivity analysis
 * 4. Generate data dictionary for simulation metadata
 */

#include <dynamics/PointMass3DOF.hpp>
#include <environment/PointMassGravity.hpp>

#include <icarus/icarus.hpp>

#include <janus/core/JanusIO.hpp>

#include <vulcan/core/Constants.hpp>

#include <chrono>
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
    sim.SetLogFile("symbolic_demo.log");

    auto &logger = sim.GetLogger();
    logger.LogStartup();

    // =========================================================================
    // Common Parameters
    // =========================================================================
    const double altitude = 400e3; // 400 km (ISS-like)
    const double r0 = vulcan::constants::earth::R_eq + altitude;
    const double mu = vulcan::constants::earth::mu;
    const double v_circular = std::sqrt(mu / r0);
    const double mass = 450000.0;

    // =========================================================================
    // Part 1: Numeric Simulation
    // =========================================================================
    logger.BeginPhase(SimPhase::Provision);
    logger.LogEntityLoad("symbolic_demo");

    auto gravity = std::make_unique<PointMassGravity<double>>("Gravity");
    gravity->SetModel(PointMassGravity<double>::Model::PointMass);

    auto vehicle = std::make_unique<PointMass3DOF<double>>(mass, "Dynamics");
    vehicle->SetInitialPosition(r0, 0.0, 0.0);
    vehicle->SetInitialVelocity(0.0, v_circular, 0.0);

    sim.AddComponent(std::move(gravity));
    logger.LogComponentAdd("Gravity", "PointMassGravity", "defaults", false);

    sim.AddComponent(std::move(vehicle));
    logger.LogComponentAdd("Dynamics", "PointMass3DOF", "defaults", true);

    sim.Provision();
    logger.LogStateAllocation(sim.GetTotalStateSize());

    // Display Flight Manifest
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

    logger.LogWiring("Dynamics.position.x", "Gravity.position.x");
    logger.LogWiring("Dynamics.position.y", "Gravity.position.y");
    logger.LogWiring("Dynamics.position.z", "Gravity.position.z");
    logger.LogWiring("Dynamics.mass", "Gravity.mass");

    sim.SetWiring("Dynamics", {{"force.x", "Gravity.force.x"},
                               {"force.y", "Gravity.force.y"},
                               {"force.z", "Gravity.force.z"}});

    logger.LogWiring("Gravity.force.x", "Dynamics.force.x");
    logger.LogWiring("Gravity.force.y", "Dynamics.force.y");
    logger.LogWiring("Gravity.force.z", "Dynamics.force.z");

    sim.Stage();
    logger.EndPhase();

    // =========================================================================
    // Run Numeric Simulation
    // =========================================================================
    logger.BeginPhase(SimPhase::Run);

    const double dt = 10.0;
    const double t_end = 100.0;

    auto wall_start = std::chrono::high_resolution_clock::now();

    while (sim.Time() < t_end) {
        logger.BeginComponentTiming("Step");

        double x = sim.GetSignal("Dynamics.position.x");
        double y = sim.GetSignal("Dynamics.position.y");
        double z = sim.GetSignal("Dynamics.position.z");
        double r = std::sqrt(x * x + y * y + z * z);
        double alt_km = (r - vulcan::constants::earth::R_eq) / 1000.0;

        logger.UpdateProgress(sim.Time(), t_end, {{"Alt", alt_km}});

        sim.Step(dt);
        logger.EndComponentTiming();
    }

    logger.ClearProgress();
    logger.EndPhase(sim.Time());

    auto wall_end = std::chrono::high_resolution_clock::now();
    double wall_time = std::chrono::duration<double>(wall_end - wall_start).count();

    logger.Log(LogLevel::Info, "Numeric simulation complete: " +
                                   std::to_string(static_cast<int>(sim.Time())) + "s simulated");

    // =========================================================================
    // Part 2: Symbolic Graph Extraction
    // =========================================================================
    logger.Log(LogLevel::Info, "");
    logger.Log(LogLevel::Info, "=== Symbolic Mode ===");

    Simulator<SymbolicScalar> sym_sim;
    sym_sim.SetQuietMode(true);
    {
        auto sym_gravity = std::make_unique<PointMassGravity<SymbolicScalar>>("Gravity");
        sym_gravity->SetModel(PointMassGravity<SymbolicScalar>::Model::PointMass);

        auto sym_vehicle =
            std::make_unique<PointMass3DOF<SymbolicScalar>>(SymbolicScalar(mass), "Dynamics");

        sym_sim.AddComponent(std::move(sym_gravity));
        sym_sim.AddComponent(std::move(sym_vehicle));
        sym_sim.Provision();

        sym_sim.SetWiring("Gravity", {{"position.x", "Dynamics.position.x"},
                                      {"position.y", "Dynamics.position.y"},
                                      {"position.z", "Dynamics.position.z"},
                                      {"mass", "Dynamics.mass"}});
        sym_sim.SetWiring("Dynamics", {{"force.x", "Gravity.force.x"},
                                       {"force.y", "Gravity.force.y"},
                                       {"force.z", "Gravity.force.z"}});
        sym_sim.Stage();
    }

    // Extract dynamics function using SymbolicTracer: xdot = f(t, x)
    symbolic::SymbolicTracer tracer(sym_sim);
    auto dynamics = tracer.TraceDynamics();

    logger.Log(LogLevel::Info,
               "[Graph] Extracted dynamics function: " + dynamics.casadi_function().name());
    logger.Log(LogLevel::Info, "[Graph] State dimension: " + std::to_string(tracer.GetStateSize()));

    // Export computational graph visualization
    // Get the symbolic output (xdot) for visualization
    auto t_vis = janus::sym("t");
    auto [x_vis_vec, x_vis_mx] = janus::sym_vec_pair("x", 6);
    auto xdot_vis = dynamics.eval(t_vis, x_vis_mx);

    // Export to interactive HTML
    janus::export_graph_html(janus::as_mx(xdot_vis), "dynamics_graph", "OrbitalDynamics");
    logger.Log(LogLevel::Info, "[Graph] Exported: dynamics_graph.html (interactive visualization)");

    // Also export to DOT and try PDF rendering
    janus::export_graph_dot(janus::as_mx(xdot_vis), "dynamics_graph", "OrbitalDynamics");
    if (janus::render_graph("dynamics_graph.dot", "dynamics_graph.pdf")) {
        logger.Log(LogLevel::Info, "[Graph] Exported: dynamics_graph.pdf");
    } else {
        logger.Log(LogLevel::Warning, "[Graph] PDF rendering skipped (Graphviz not installed)");
    }

    // =========================================================================
    // Part 3: Evaluate Dynamics
    // =========================================================================
    janus::NumericVector x0(6);
    x0 << r0, 0.0, 0.0, 0.0, v_circular, 0.0;

    auto xdot = dynamics(0.0, x0);

    logger.Log(LogLevel::Info, "[Eval] Dynamics evaluated at initial condition");

    // =========================================================================
    // Part 4: Jacobian Computation
    // =========================================================================
    auto t_jac = janus::sym("t_jac");
    auto [x_jac_vec, x_jac_mx] = janus::sym_vec_pair("x_jac", 6);
    auto xdot_jac = dynamics.eval(t_jac, x_jac_mx);
    auto J_sym = janus::jacobian({janus::as_mx(xdot_jac)}, {x_jac_mx});
    auto jacobian_fn = janus::Function("jacobian", {t_jac, x_jac_mx}, {J_sym});

    auto J_result = jacobian_fn(0.0, x0);
    auto J = J_result[0];

    logger.Log(LogLevel::Info, "[Jacobian] Computed " + std::to_string(J.rows()) + "x" +
                                   std::to_string(J.cols()) + " state Jacobian");

    // =========================================================================
    // Part 5: TraceStep for Discrete Dynamics
    // =========================================================================
    Simulator<SymbolicScalar> step_sim;
    step_sim.SetQuietMode(true);
    {
        auto step_gravity = std::make_unique<PointMassGravity<SymbolicScalar>>("Gravity");
        step_gravity->SetModel(PointMassGravity<SymbolicScalar>::Model::PointMass);

        auto step_vehicle =
            std::make_unique<PointMass3DOF<SymbolicScalar>>(SymbolicScalar(mass), "Dynamics");

        step_sim.AddComponent(std::move(step_gravity));
        step_sim.AddComponent(std::move(step_vehicle));
        step_sim.Provision();

        step_sim.SetWiring("Gravity", {{"position.x", "Dynamics.position.x"},
                                       {"position.y", "Dynamics.position.y"},
                                       {"position.z", "Dynamics.position.z"},
                                       {"mass", "Dynamics.mass"}});
        step_sim.SetWiring("Dynamics", {{"force.x", "Gravity.force.x"},
                                        {"force.y", "Gravity.force.y"},
                                        {"force.z", "Gravity.force.z"}});
        step_sim.Stage();
    }

    auto step_fn = symbolic::SymbolicTracer(step_sim).TraceStep(10.0);

    auto x_next_result = step_fn(0.0, x0);
    auto x_next = x_next_result[0];

    logger.Log(LogLevel::Info, "[TraceStep] Discrete RK4 step function extracted (dt=10s)");

    // =========================================================================
    // Shutdown with Mission Debrief
    // =========================================================================
    logger.BeginPhase(SimPhase::Shutdown);
    logger.LogEvent("Symbolic Demo Complete", sim.Time(),
                    "Graph extraction, Jacobian, and TraceStep demonstrated");

    logger.LogDebrief(sim.Time(), wall_time);

    // Export Data Dictionary
    sim.GenerateDataDictionary("symbolic_demo_datadict.yaml");
    sim.GenerateDataDictionary("symbolic_demo_datadict.json");
    logger.LogTimed(LogLevel::Info, sim.Time(),
                    "[IO] Saved: symbolic_demo_datadict.yaml, symbolic_demo_datadict.json");

    logger.EndPhase(sim.Time());

    // =========================================================================
    // Console Summary (for quick visual verification)
    // =========================================================================
    std::cout << std::endl;
    std::cout << "=== Symbolic Demo Summary ===" << std::endl;
    std::cout << "  Numeric sim:   " << sim.Time() << "s simulated in " << std::fixed
              << std::setprecision(3) << wall_time << "s wall time" << std::endl;
    std::cout << "  State size:    " << sim.GetTotalStateSize() << std::endl;
    std::cout << "  Jacobian:      " << J.rows() << "x" << J.cols() << std::endl;
    std::cout << std::endl;
    std::cout << "  Jacobian diagnonal:" << std::endl;
    for (int i = 0; i < std::min(static_cast<int>(J.rows()), 3); ++i) {
        std::cout << "    J[" << i << "," << i << "] = " << J(i, i) << std::endl;
    }
    std::cout << std::endl;
    std::cout << "  Data dictionary exported to:" << std::endl;
    std::cout << "    - symbolic_demo_datadict.yaml" << std::endl;
    std::cout << "    - symbolic_demo_datadict.json" << std::endl;
    std::cout << std::endl;
    std::cout << "  Graph visualizations exported to:" << std::endl;
    std::cout << "    - dynamics_graph.html (interactive)" << std::endl;
    std::cout << "    - dynamics_graph.dot" << std::endl;
    std::cout << "    - dynamics_graph.pdf (if Graphviz installed)" << std::endl;

    return 0;
}
