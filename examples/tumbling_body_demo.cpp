/**
 * @file tumbling_body_demo.cpp
 * @brief Tumbling Rigid Body Demo
 *
 * Demonstrates RigidBody6DOF capabilities:
 * - Config-driven 6DOF simulation with quaternion attitude
 * - Angular momentum conservation (torque-free tumbling)
 * - Rotational energy conservation
 * - Quaternion normalization tracking
 *
 * Physics: An asymmetric rigid body tumbles freely without external torques.
 * Conservation laws verify the integration quality.
 *
 * Usage: ./tumbling_body_demo [config_path]
 *        Default: config/tumbling_body.yaml
 */

#include <icarus/icarus.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace icarus;
namespace fs = std::filesystem;

// =============================================================================
// Physics Helpers
// =============================================================================

/**
 * @brief Compute angular momentum in inertial frame
 *
 * H_inertial = R_body_to_inertial * (I * omega_body)
 */
Vec3<double> ComputeAngularMomentumInertial(const janus::Quaternion<double> &q,
                                            const Vec3<double> &omega_body, double Ix, double Iy,
                                            double Iz) {
    Mat3<double> I = Mat3<double>::Zero();
    I(0, 0) = Ix;
    I(1, 1) = Iy;
    I(2, 2) = Iz;
    Vec3<double> H_body = I * omega_body;
    return q.rotate(H_body);
}

/**
 * @brief Compute rotational kinetic energy
 *
 * T = 0.5 * omega^T * I * omega
 */
double ComputeRotationalEnergy(const Vec3<double> &omega_body, double Ix, double Iy, double Iz) {
    Mat3<double> I = Mat3<double>::Zero();
    I(0, 0) = Ix;
    I(1, 1) = Iy;
    I(2, 2) = Iz;
    Vec3<double> H_body = I * omega_body;
    return 0.5 * omega_body.dot(H_body);
}

int main(int argc, char *argv[]) {
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       Icarus Tumbling Rigid Body Demo                      ║\n";
    std::cout << "║       RigidBody6DOF with Quaternion Attitude               ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // Config path - default or command line
    std::string config_path = "config/tumbling_body.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    if (!fs::exists(config_path)) {
        std::cerr << "❌ Config not found: " << config_path << "\n";
        std::cerr << "   Run from project root or specify path as argument.\n";
        return 1;
    }

    // =========================================================================
    // Load and stage simulation
    // =========================================================================

    std::cout << "Loading: " << config_path << "\n";

    std::unique_ptr<Simulator> sim;
    try {
        sim = Simulator::FromConfig(config_path);
    } catch (const std::exception &e) {
        std::cerr << "❌ Error: " << e.what() << "\n";
        return 1;
    }

    sim->Stage();

    std::cout << "  Components: " << sim->NumComponents() << "\n";
    std::cout << "  State size: " << sim->GetState().size()
              << " (13 = pos[3] + vel[3] + quat[4] + omega[3])\n";
    std::cout << "  Timestep: " << sim->Dt() << " s\n";
    std::cout << "  Duration: " << sim->EndTime() << " s\n\n";

    // =========================================================================
    // Inertia values (must match tumbling_body.yaml)
    // =========================================================================

    const double Ix = 1.0; // kg*m^2
    const double Iy = 2.0; // kg*m^2
    const double Iz = 3.0; // kg*m^2

    // =========================================================================
    // Initial state capture
    // =========================================================================

    auto state0 = sim->GetState();
    janus::Quaternion<double> q0(state0[6], state0[7], state0[8], state0[9]);
    Vec3<double> omega0{state0[10], state0[11], state0[12]};

    Vec3<double> H0 = ComputeAngularMomentumInertial(q0, omega0, Ix, Iy, Iz);
    double H0_mag = H0.norm();
    double T0 = ComputeRotationalEnergy(omega0, Ix, Iy, Iz);

    std::cout << "Initial Conditions:\n";
    std::cout << "  Angular velocity (body): [" << omega0(0) << ", " << omega0(1) << ", "
              << omega0(2) << "] rad/s\n";
    std::cout << "  Angular momentum |H|:    " << std::fixed << std::setprecision(6) << H0_mag
              << " kg*m²/s\n";
    std::cout << "  Rotational energy T:     " << T0 << " J\n\n";

    // =========================================================================
    // Run simulation with periodic output
    // =========================================================================

    std::cout << "Running simulation...\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(10) << "Time [s]" << std::setw(15) << "|H| [kg*m²/s]" << std::setw(15)
              << "T [J]" << std::setw(15) << "|q|" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";

    int step = 0;
    const int print_interval = static_cast<int>(1.0 / sim->Dt()); // Every 1 second

    while (sim->Time() < sim->EndTime()) {
        sim->Step(sim->Dt());

        // Print every second
        if (++step % print_interval == 0) {
            auto state = sim->GetState();

            janus::Quaternion<double> q(state[6], state[7], state[8], state[9]);
            Vec3<double> omega{state[10], state[11], state[12]};

            Vec3<double> H = ComputeAngularMomentumInertial(q, omega, Ix, Iy, Iz);
            double T = ComputeRotationalEnergy(omega, Ix, Iy, Iz);
            double q_norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

            std::cout << std::fixed << std::setprecision(2) << std::setw(10) << sim->Time()
                      << std::setprecision(6) << std::setw(15) << H.norm() << std::setw(15) << T
                      << std::setw(15) << q_norm << "\n";
        }
    }

    std::cout << "─────────────────────────────────────────────────────────────\n\n";

    // =========================================================================
    // Conservation law verification
    // =========================================================================

    auto final_state = sim->GetState();
    janus::Quaternion<double> q_final(final_state[6], final_state[7], final_state[8],
                                      final_state[9]);
    Vec3<double> omega_final{final_state[10], final_state[11], final_state[12]};

    Vec3<double> H_final = ComputeAngularMomentumInertial(q_final, omega_final, Ix, Iy, Iz);
    double T_final = ComputeRotationalEnergy(omega_final, Ix, Iy, Iz);
    double q_norm_final = std::sqrt(q_final.w * q_final.w + q_final.x * q_final.x +
                                    q_final.y * q_final.y + q_final.z * q_final.z);

    double H_error = (H_final - H0).norm() / H0_mag * 100.0;
    double T_error = std::abs(T_final - T0) / T0 * 100.0;
    double q_error = std::abs(q_norm_final - 1.0);

    std::cout << "Conservation Law Verification:\n";
    std::cout << "─────────────────────────────────────────────────────────────\n";
    std::cout << "  Angular Momentum:\n";
    std::cout << "    Initial |H|:  " << std::fixed << std::setprecision(6) << H0_mag
              << " kg*m²/s\n";
    std::cout << "    Final |H|:    " << H_final.norm() << " kg*m²/s\n";
    std::cout << "    Relative err: " << std::scientific << std::setprecision(2) << H_error
              << "%\n\n";

    std::cout << "  Rotational Energy:\n";
    std::cout << "    Initial T:    " << std::fixed << std::setprecision(6) << T0 << " J\n";
    std::cout << "    Final T:      " << T_final << " J\n";
    std::cout << "    Relative err: " << std::scientific << std::setprecision(2) << T_error
              << "%\n\n";

    std::cout << "  Quaternion Normalization:\n";
    std::cout << "    |q| = " << std::fixed << std::setprecision(8) << q_norm_final << "\n";
    std::cout << "    Drift: " << std::scientific << std::setprecision(2) << q_error << "\n\n";

    // =========================================================================
    // Pass/Fail determination
    // =========================================================================

    bool momentum_ok = H_error < 0.01; // < 0.01% drift
    bool energy_ok = T_error < 0.01;   // < 0.01% drift
    bool quat_ok = q_error < 1e-3;     // Quaternion norm within 0.1%

    std::cout << "─────────────────────────────────────────────────────────────\n";
    std::cout << "Results:\n";
    std::cout << "  Angular momentum: " << (momentum_ok ? "✅ CONSERVED" : "❌ DRIFT") << "\n";
    std::cout << "  Rotational energy: " << (energy_ok ? "✅ CONSERVED" : "❌ DRIFT") << "\n";
    std::cout << "  Quaternion norm:  " << (quat_ok ? "✅ NORMALIZED" : "❌ DRIFT") << "\n";

    bool passed = momentum_ok && energy_ok && quat_ok;
    std::cout << "\n" << (passed ? "✅ Demo PASSED!" : "❌ Demo FAILED") << "\n";

    return passed ? 0 : 1;
}
