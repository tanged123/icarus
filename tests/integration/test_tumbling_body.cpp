/**
 * @file test_tumbling_body.cpp
 * @brief Integration test: Tumbling rigid body conservation laws
 *
 * Validates RigidBody6DOF by simulating a torque-free tumbling body
 * and verifying conservation of:
 * - Angular momentum (in inertial frame)
 * - Rotational kinetic energy
 * - Quaternion normalization
 *
 * Part of Phase 4.5: Integration test - tumbling rigid body
 */

#include <gtest/gtest.h>

#include <icarus/icarus.hpp>

#include <janus/math/Quaternion.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>

using namespace icarus;
namespace fs = std::filesystem;

// =============================================================================
// Test Fixture
// =============================================================================

class TumblingBodyTest : public ::testing::Test {
  protected:
    // Inertia values (must match tumbling_body.yaml)
    static constexpr double Ix = 1.0;
    static constexpr double Iy = 2.0;
    static constexpr double Iz = 3.0;

    // Initial omega (must match tumbling_body.yaml)
    Vec3<double> omega0{1.0, 0.5, 0.2};

    /**
     * @brief Compute angular momentum in inertial frame
     */
    Vec3<double> ComputeAngularMomentumInertial(const janus::Quaternion<double> &q,
                                                const Vec3<double> &omega_body) const {
        Mat3<double> I = Mat3<double>::Zero();
        I(0, 0) = Ix;
        I(1, 1) = Iy;
        I(2, 2) = Iz;
        Vec3<double> H_body = I * omega_body;
        return q.rotate(H_body);
    }

    /**
     * @brief Compute rotational kinetic energy
     */
    double ComputeRotationalEnergy(const Vec3<double> &omega_body) const {
        Mat3<double> I = Mat3<double>::Zero();
        I(0, 0) = Ix;
        I(1, 1) = Iy;
        I(2, 2) = Iz;
        Vec3<double> H_body = I * omega_body;
        return 0.5 * omega_body.dot(H_body);
    }

    /**
     * @brief Get config path (works from build directory)
     */
    std::string GetConfigPath() const {
        // Try relative to source root first
        if (fs::exists("config/tumbling_body.yaml")) {
            return "config/tumbling_body.yaml";
        }
        // Try from build/tests directory
        if (fs::exists("../../config/tumbling_body.yaml")) {
            return "../../config/tumbling_body.yaml";
        }
        // No fallback - throw descriptive error
        throw std::runtime_error("Could not find config/tumbling_body.yaml. "
                                 "Run tests from repository root or build directory.");
    }
};

// =============================================================================
// Test: Angular Momentum Conservation
// =============================================================================

TEST_F(TumblingBodyTest, AngularMomentumConserved) {
    auto sim = Simulator::FromConfig(GetConfigPath());
    sim->Stage();

    // Get initial state
    auto state0 = sim->GetState();
    ASSERT_EQ(state0.size(), 13u);

    janus::Quaternion<double> q0(state0[6], state0[7], state0[8], state0[9]);
    Vec3<double> omega0_state{state0[10], state0[11], state0[12]};
    Vec3<double> H0 = ComputeAngularMomentumInertial(q0, omega0_state);
    double H0_mag = H0.norm();

    std::cout << "\n=== Angular Momentum Conservation ===\n";
    std::cout << "Initial |H| = " << H0_mag << " kg*m^2/s\n";

    // Run simulation
    while (sim->Time() < sim->EndTime()) {
        sim->Step(sim->Dt());
    }

    // Check final
    auto state = sim->GetState();
    janus::Quaternion<double> q(state[6], state[7], state[8], state[9]);
    Vec3<double> omega{state[10], state[11], state[12]};
    Vec3<double> H = ComputeAngularMomentumInertial(q, omega);

    double relative_error = (H - H0).norm() / H0_mag;
    std::cout << "Final |H| = " << H.norm() << " kg*m^2/s\n";
    std::cout << "Relative error = " << relative_error * 100 << "%\n";

    EXPECT_LT(relative_error, 1e-4) << "Angular momentum not conserved";
}

// =============================================================================
// Test: Rotational Energy Conservation
// =============================================================================

TEST_F(TumblingBodyTest, RotationalEnergyConserved) {
    auto sim = Simulator::FromConfig(GetConfigPath());
    sim->Stage();

    auto state0 = sim->GetState();
    Vec3<double> omega0_state{state0[10], state0[11], state0[12]};
    double T0 = ComputeRotationalEnergy(omega0_state);

    std::cout << "\n=== Energy Conservation ===\n";
    std::cout << "Initial T = " << T0 << " J\n";

    while (sim->Time() < sim->EndTime()) {
        sim->Step(sim->Dt());
    }

    auto state = sim->GetState();
    Vec3<double> omega{state[10], state[11], state[12]};
    double T = ComputeRotationalEnergy(omega);

    double relative_error = std::abs(T - T0) / T0;
    std::cout << "Final T = " << T << " J\n";
    std::cout << "Relative error = " << relative_error * 100 << "%\n";

    EXPECT_LT(relative_error, 1e-4) << "Energy not conserved";
}

// =============================================================================
// Test: Quaternion Normalization
// =============================================================================

TEST_F(TumblingBodyTest, QuaternionNormalized) {
    auto sim = Simulator::FromConfig(GetConfigPath());
    sim->Stage();

    std::cout << "\n=== Quaternion Normalization ===\n";

    while (sim->Time() < sim->EndTime()) {
        sim->Step(sim->Dt());
    }

    auto state = sim->GetState();
    double qw = state[6], qx = state[7], qy = state[8], qz = state[9];
    double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);

    std::cout << "Final |q| = " << q_norm << "\n";

    EXPECT_NEAR(q_norm, 1.0, 1e-3) << "Quaternion normalization drifted";
}
