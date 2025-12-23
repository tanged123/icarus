#include <icarus/icarus.hpp>
#include <iostream>

int main() {
    std::cout << "=== Icarus 6DOF Simulation Engine ===" << std::endl;
    std::cout << "Version: " << icarus::Version() << std::endl;
    std::cout << std::endl;

    // Create a simulator instance
    icarus::Simulator<double> sim;
    std::cout << "Simulator created" << std::endl;
    std::cout << "  Phase: Uninitialized" << std::endl;
    std::cout << "  Components: " << sim.NumComponents() << std::endl;
    std::cout << std::endl;

    // Provision (no components yet, but tests the lifecycle)
    sim.Provision();
    std::cout << "After Provision:" << std::endl;
    std::cout << "  Phase: Provisioned" << std::endl;
    std::cout << std::endl;

    // Stage
    sim.Stage();
    std::cout << "After Stage:" << std::endl;
    std::cout << "  Phase: Staged" << std::endl;
    std::cout << "  Time: " << sim.Time() << std::endl;
    std::cout << std::endl;

    // Run a few steps
    double dt = 0.01;
    for (int i = 0; i < 10; ++i) {
        sim.Step(dt);
    }
    std::cout << "After 10 steps (dt=" << dt << "):" << std::endl;
    std::cout << "  Time: " << sim.Time() << std::endl;
    std::cout << std::endl;

    std::cout << "=== Hello World Complete ===" << std::endl;
    return 0;
}
