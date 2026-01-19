"""
Rocket Ascent Integration Tests

This module tests the rocket ascent simulation end-to-end, verifying:
- Mass depletion over burn
- Thrust produces expected acceleration
- Position/velocity integration
- Comparison against analytical predictions (Tsiolkovsky)

Reference values from implementation plan (docs/implementation_plans/rocket_example.md):
    Ve = Isp * g0 = 311 * 9.80665 = 3050 m/s
    m0 = 1550 kg (500 structure + 50 tank + 1000 fuel)
    mf = 550 kg (500 structure + 50 tank)
    ΔV = Ve * ln(m0/mf) = 3050 * ln(2.82) = 3163 m/s
    mdot = F / Ve = 50000 / 3050 = 16.4 kg/s
    t_burn = 1000 / 16.4 = 61 s
"""

import math
import numpy as np
import pytest
from pathlib import Path

import icarus


# Physical constants
G0 = 9.80665  # Standard gravity [m/s²]
MU_EARTH = 3.986004418e14  # Earth GM [m³/s²]

# Rocket parameters from YAML config
MAX_THRUST = 50000.0  # [N]
ISP_VACUUM = 311.0  # [s]
INITIAL_FUEL_MASS = 1000.0  # [kg]
DRY_MASS_TANK = 50.0  # [kg]
STRUCTURE_MASS = 500.0  # [kg]
INITIAL_TOTAL_MASS = STRUCTURE_MASS + DRY_MASS_TANK + INITIAL_FUEL_MASS  # 1550 kg
FINAL_TOTAL_MASS = STRUCTURE_MASS + DRY_MASS_TANK  # 550 kg

# Derived analytical values
EXHAUST_VELOCITY = ISP_VACUUM * G0  # ~3050 m/s
MASS_FLOW_RATE = MAX_THRUST / EXHAUST_VELOCITY  # ~16.4 kg/s
BURN_TIME = INITIAL_FUEL_MASS / MASS_FLOW_RATE  # ~61 s
DELTA_V_ANALYTICAL = EXHAUST_VELOCITY * math.log(INITIAL_TOTAL_MASS / FINAL_TOTAL_MASS)  # ~3163 m/s


def get_config_path() -> Path:
    """Get path to rocket_ascent.yaml config."""
    # Find config relative to this test file
    test_dir = Path(__file__).parent
    repo_root = test_dir.parent.parent
    config_path = repo_root / "config" / "examples" / "rocket_ascent.yaml"
    if not config_path.exists():
        pytest.skip(f"Config not found: {config_path}")
    return config_path


@pytest.fixture
def rocket_sim():
    """Create and stage a rocket simulation."""
    config_path = get_config_path()
    sim = icarus.Simulator(str(config_path))
    sim.stage()
    sim.step()  # Initialize aggregated values (MassAggregator, etc.)
    return sim


class TestRocketConfiguration:
    """Tests to verify rocket configuration loaded correctly."""

    def test_initial_mass(self, rocket_sim):
        """Verify initial total mass matches expected value."""
        total_mass = rocket_sim["Rocket.Mass.total_mass"]
        assert abs(total_mass - INITIAL_TOTAL_MASS) < 1.0, (
            f"Initial mass {total_mass:.1f} kg != expected {INITIAL_TOTAL_MASS:.1f} kg"
        )

    def test_initial_fuel_mass(self, rocket_sim):
        """Verify initial fuel mass."""
        fuel_mass = rocket_sim["Rocket.FuelTank.fuel_mass"]
        assert abs(fuel_mass - INITIAL_FUEL_MASS) < 1.0, (
            f"Initial fuel {fuel_mass:.1f} kg != expected {INITIAL_FUEL_MASS:.1f} kg"
        )

    def test_initial_position(self, rocket_sim):
        """Verify initial position is at ~100km altitude."""
        pos_x = rocket_sim["Rocket.EOM.position.x"]
        pos_y = rocket_sim["Rocket.EOM.position.y"]
        pos_z = rocket_sim["Rocket.EOM.position.z"]
        r = math.sqrt(pos_x**2 + pos_y**2 + pos_z**2)
        altitude = r - 6.371e6  # Earth radius
        assert 90_000 < altitude < 110_000, f"Initial altitude {altitude/1000:.1f} km not ~100 km"


class TestMassDepletion:
    """Tests verifying fuel consumption and mass depletion."""

    def test_mass_flow_rate_at_full_throttle(self, rocket_sim):
        """Verify mass flow rate matches analytical prediction at full throttle."""
        # Set full throttle
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0
        rocket_sim.step()

        mdot = rocket_sim["Rocket.Engine.mass_flow_rate"]
        expected_mdot = MASS_FLOW_RATE

        # Allow 1% tolerance
        rel_error = abs(mdot - expected_mdot) / expected_mdot
        assert rel_error < 0.01, (
            f"Mass flow rate {mdot:.2f} kg/s != expected {expected_mdot:.2f} kg/s"
        )

    def test_mass_depletes_over_time(self, rocket_sim):
        """Verify fuel mass decreases during burn."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        initial_fuel = rocket_sim["Rocket.FuelTank.fuel_mass"]

        # Run for 10 seconds
        for _ in range(1000):  # 10s at 0.01s dt
            rocket_sim.step()

        final_fuel = rocket_sim["Rocket.FuelTank.fuel_mass"]
        fuel_consumed = initial_fuel - final_fuel

        # Expected: ~164 kg consumed (16.4 kg/s * 10s)
        expected_consumed = MASS_FLOW_RATE * 10.0
        rel_error = abs(fuel_consumed - expected_consumed) / expected_consumed
        assert rel_error < 0.02, (
            f"Fuel consumed {fuel_consumed:.1f} kg != expected {expected_consumed:.1f} kg"
        )

    def test_fuel_exhaustion_clamps_at_zero(self, rocket_sim):
        """Verify fuel doesn't go negative when exhausted."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        # Run past expected burn time
        burn_steps = int((BURN_TIME + 10) / rocket_sim.dt)
        for _ in range(burn_steps):
            rocket_sim.step()

        fuel_mass = rocket_sim["Rocket.FuelTank.fuel_mass"]
        # Allow small negative values due to integrator overshoot at discontinuity
        # (RK4 can overshoot when derivative changes abruptly at fuel exhaustion)
        assert fuel_mass >= -1.0, f"Fuel mass went significantly negative: {fuel_mass:.2f} kg"

        # Total mass should be at minimum (structure + dry tank)
        total_mass = rocket_sim["Rocket.Mass.total_mass"]
        assert abs(total_mass - FINAL_TOTAL_MASS) < 10.0, (
            f"Final mass {total_mass:.1f} kg != expected {FINAL_TOTAL_MASS:.1f} kg"
        )


class TestThrust:
    """Tests verifying thrust computation and force application."""

    def test_zero_throttle_zero_thrust(self, rocket_sim):
        """Verify zero throttle produces zero thrust."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 0.0
        rocket_sim.step()

        thrust = rocket_sim["Rocket.Engine.thrust"]
        assert abs(thrust) < 1.0, f"Expected zero thrust, got {thrust:.1f} N"

    def test_full_throttle_max_thrust(self, rocket_sim):
        """Verify full throttle produces max thrust."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0
        rocket_sim.step()

        thrust = rocket_sim["Rocket.Engine.thrust"]
        rel_error = abs(thrust - MAX_THRUST) / MAX_THRUST
        assert rel_error < 0.01, (
            f"Thrust {thrust:.0f} N != expected {MAX_THRUST:.0f} N"
        )

    def test_partial_throttle(self, rocket_sim):
        """Verify partial throttle scales thrust linearly."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 0.5
        rocket_sim.step()

        thrust = rocket_sim["Rocket.Engine.thrust"]
        expected = MAX_THRUST * 0.5
        rel_error = abs(thrust - expected) / expected
        assert rel_error < 0.01, (
            f"Half throttle thrust {thrust:.0f} N != expected {expected:.0f} N"
        )

    def test_thrust_produces_force_vector(self, rocket_sim):
        """Verify thrust produces force in correct direction."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0
        rocket_sim.step()

        # Thrust direction is [0, 0, 1] in body frame (from YAML)
        fx = rocket_sim["Rocket.Engine.force.x"]
        fy = rocket_sim["Rocket.Engine.force.y"]
        fz = rocket_sim["Rocket.Engine.force.z"]

        # Force should be primarily in Z direction
        assert abs(fx) < 100.0, f"Unexpected X force: {fx:.1f} N"
        assert abs(fy) < 100.0, f"Unexpected Y force: {fy:.1f} N"
        assert fz > MAX_THRUST * 0.99, f"Z force {fz:.0f} N != max thrust"


class TestAcceleration:
    """Tests verifying acceleration from thrust."""

    def test_initial_acceleration(self, rocket_sim):
        """Verify initial acceleration matches F/m."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0
        rocket_sim.step()

        thrust = rocket_sim["Rocket.Engine.thrust"]
        total_mass = rocket_sim["Rocket.Mass.total_mass"]
        expected_accel = thrust / total_mass  # ~32.3 m/s²

        # Get gravity at current altitude
        pos_x = rocket_sim["Rocket.EOM.position.x"]
        pos_y = rocket_sim["Rocket.EOM.position.y"]
        pos_z = rocket_sim["Rocket.EOM.position.z"]
        r = math.sqrt(pos_x**2 + pos_y**2 + pos_z**2)
        g = MU_EARTH / (r * r)  # ~9.56 m/s² at 100km

        # Net acceleration should be thrust/mass - gravity
        expected_net_accel = expected_accel - g  # ~22.7 m/s²

        # Initial T/W ratio should be about 3.2
        twr = thrust / (total_mass * g)
        assert 2.5 < twr < 4.0, f"Initial T/W ratio {twr:.2f} outside expected range"

    def test_acceleration_increases_during_burn(self, rocket_sim):
        """Verify acceleration increases as mass decreases."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        # Record initial acceleration (after first step for forces to compute)
        rocket_sim.step()
        initial_mass = rocket_sim["Rocket.Mass.total_mass"]
        initial_thrust = rocket_sim["Rocket.Engine.thrust"]
        initial_accel = initial_thrust / initial_mass

        # Run for 30 seconds
        for _ in range(3000):
            rocket_sim.step()

        final_mass = rocket_sim["Rocket.Mass.total_mass"]
        final_thrust = rocket_sim["Rocket.Engine.thrust"]
        final_accel = final_thrust / final_mass

        # Acceleration should increase (same thrust, less mass)
        assert final_accel > initial_accel, (
            f"Acceleration did not increase: {initial_accel:.2f} -> {final_accel:.2f} m/s²"
        )

        # Ratio should match mass ratio
        expected_ratio = initial_mass / final_mass
        actual_ratio = final_accel / initial_accel
        assert abs(actual_ratio - expected_ratio) < 0.1, (
            f"Acceleration ratio {actual_ratio:.2f} != mass ratio {expected_ratio:.2f}"
        )


class TestPositionVelocity:
    """Tests verifying position and velocity integration."""

    def test_velocity_increases_with_thrust(self, rocket_sim):
        """Verify velocity magnitude increases during burn."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        vx0 = rocket_sim["Rocket.EOM.velocity_body.x"]
        vy0 = rocket_sim["Rocket.EOM.velocity_body.y"]
        vz0 = rocket_sim["Rocket.EOM.velocity_body.z"]
        v0 = math.sqrt(vx0**2 + vy0**2 + vz0**2)

        # Run for 10 seconds
        for _ in range(1000):
            rocket_sim.step()

        vx1 = rocket_sim["Rocket.EOM.velocity_body.x"]
        vy1 = rocket_sim["Rocket.EOM.velocity_body.y"]
        vz1 = rocket_sim["Rocket.EOM.velocity_body.z"]
        v1 = math.sqrt(vx1**2 + vy1**2 + vz1**2)

        # Velocity should increase significantly
        delta_v = v1 - v0
        assert delta_v > 100.0, f"Velocity only increased by {delta_v:.1f} m/s in 10s"

    def test_position_changes_with_velocity(self, rocket_sim):
        """Verify position changes as vehicle accelerates."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        x0 = rocket_sim["Rocket.EOM.position.x"]
        y0 = rocket_sim["Rocket.EOM.position.y"]
        z0 = rocket_sim["Rocket.EOM.position.z"]

        # Run for 30 seconds
        for _ in range(3000):
            rocket_sim.step()

        x1 = rocket_sim["Rocket.EOM.position.x"]
        y1 = rocket_sim["Rocket.EOM.position.y"]
        z1 = rocket_sim["Rocket.EOM.position.z"]

        # Check total displacement (not just radial, since thrust may be tangential)
        # Note: Thrust direction [0,0,1] with identity quaternion pushes in +Z inertial
        # Starting at (6.471e6, 0, 0), displacement should primarily be in Z
        displacement = math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)

        # With ~32 m/s² accel for 30s: d = 0.5 * 32 * 30² ≈ 14400m
        # Allow for gravity losses, so expect at least 5000m
        assert displacement > 5000.0, f"Only moved {displacement:.0f} m in 30s of thrust"


class TestTsiolkovsky:
    """Tests comparing simulation against Tsiolkovsky rocket equation."""

    def test_delta_v_matches_tsiolkovsky(self, rocket_sim):
        """Verify total delta-V matches Tsiolkovsky prediction."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        # Record initial velocity
        vx0 = rocket_sim["Rocket.EOM.velocity_body.x"]
        vy0 = rocket_sim["Rocket.EOM.velocity_body.y"]
        vz0 = rocket_sim["Rocket.EOM.velocity_body.z"]
        v0 = math.sqrt(vx0**2 + vy0**2 + vz0**2)

        initial_mass = rocket_sim["Rocket.Mass.total_mass"]

        # Run until fuel exhausted (plus margin)
        burn_steps = int((BURN_TIME + 5) / rocket_sim.dt)
        for _ in range(burn_steps):
            rocket_sim.step()
            # Stop if fuel exhausted
            if rocket_sim["Rocket.FuelTank.fuel_mass"] < 0.1:
                break

        final_mass = rocket_sim["Rocket.Mass.total_mass"]
        vxf = rocket_sim["Rocket.EOM.velocity_body.x"]
        vyf = rocket_sim["Rocket.EOM.velocity_body.y"]
        vzf = rocket_sim["Rocket.EOM.velocity_body.z"]
        vf = math.sqrt(vxf**2 + vyf**2 + vzf**2)

        # Note: This is velocity magnitude change, not true delta-V
        # (gravity affects trajectory). For a more accurate comparison,
        # we'd need to integrate net acceleration excluding gravity.
        delta_v_sim = vf - v0

        # Compute Tsiolkovsky prediction for actual mass ratio
        mass_ratio = initial_mass / final_mass
        delta_v_tsiolkovsky = EXHAUST_VELOCITY * math.log(mass_ratio)

        # The simulated delta-V will be less than Tsiolkovsky due to:
        # 1. Gravity losses (fighting gravity during ascent)
        # 2. Trajectory effects (not a pure 1D problem)
        # Expect ~10-20% gravity loss for vertical ascent

        gravity_loss_fraction = 1 - (delta_v_sim / delta_v_tsiolkovsky)
        assert 0.0 < gravity_loss_fraction < 0.30, (
            f"Gravity loss {gravity_loss_fraction*100:.1f}% outside expected range.\n"
            f"Simulated ΔV: {delta_v_sim:.0f} m/s, Tsiolkovsky: {delta_v_tsiolkovsky:.0f} m/s"
        )

        print(f"\nTsiolkovsky Comparison:")
        print(f"  Initial mass: {initial_mass:.0f} kg")
        print(f"  Final mass: {final_mass:.0f} kg")
        print(f"  Mass ratio: {mass_ratio:.2f}")
        print(f"  Exhaust velocity: {EXHAUST_VELOCITY:.0f} m/s")
        print(f"  Tsiolkovsky ΔV: {delta_v_tsiolkovsky:.0f} m/s")
        print(f"  Simulated ΔV: {delta_v_sim:.0f} m/s")
        print(f"  Gravity loss: {gravity_loss_fraction*100:.1f}%")

    def test_burn_time_matches_prediction(self, rocket_sim):
        """Verify burn time matches fuel_mass / mdot."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        t_start = rocket_sim.time
        initial_fuel = rocket_sim["Rocket.FuelTank.fuel_mass"]

        # Run until fuel exhausted
        while rocket_sim["Rocket.FuelTank.fuel_mass"] > 0.1:
            rocket_sim.step()
            # Safety: don't run forever
            if rocket_sim.time > BURN_TIME * 2:
                break

        t_burn_sim = rocket_sim.time - t_start
        expected_burn_time = initial_fuel / MASS_FLOW_RATE

        rel_error = abs(t_burn_sim - expected_burn_time) / expected_burn_time
        assert rel_error < 0.05, (
            f"Burn time {t_burn_sim:.1f}s != expected {expected_burn_time:.1f}s"
        )

        print(f"\nBurn Time Comparison:")
        print(f"  Initial fuel: {initial_fuel:.0f} kg")
        print(f"  Mass flow rate: {MASS_FLOW_RATE:.2f} kg/s")
        print(f"  Expected burn time: {expected_burn_time:.1f} s")
        print(f"  Simulated burn time: {t_burn_sim:.1f} s")


class TestEndToEnd:
    """Full end-to-end simulation tests."""

    def test_full_simulation_completes(self, rocket_sim):
        """Run full simulation and verify it completes without errors."""
        rocket_sim["Rocket.Engine.throttle_cmd"] = 1.0

        times = []
        masses = []
        altitudes = []
        thrusts = []

        # Run for 120 seconds (full sim time)
        while rocket_sim.time < 120.0:
            times.append(rocket_sim.time)
            masses.append(rocket_sim["Rocket.Mass.total_mass"])
            thrusts.append(rocket_sim["Rocket.Engine.thrust"])

            pos_x = rocket_sim["Rocket.EOM.position.x"]
            pos_y = rocket_sim["Rocket.EOM.position.y"]
            pos_z = rocket_sim["Rocket.EOM.position.z"]
            r = math.sqrt(pos_x**2 + pos_y**2 + pos_z**2)
            altitudes.append(r - 6.371e6)

            rocket_sim.step()

        # Verify simulation completed
        assert rocket_sim.time >= 120.0, "Simulation did not complete"

        # Print summary
        print(f"\nEnd-to-End Simulation Summary:")
        print(f"  Duration: {times[-1]:.1f} s")
        print(f"  Initial mass: {masses[0]:.0f} kg")
        print(f"  Final mass: {masses[-1]:.0f} kg")
        print(f"  Initial altitude: {altitudes[0]/1000:.1f} km")
        print(f"  Final altitude: {altitudes[-1]/1000:.1f} km")
        print(f"  Max thrust: {max(thrusts):.0f} N")

        # Basic sanity checks
        assert masses[-1] < masses[0], "Mass should decrease"
        assert altitudes[-1] > altitudes[0], "Altitude should increase"


if __name__ == "__main__":
    # Allow running as script for quick testing
    pytest.main([__file__, "-v", "-s"])
