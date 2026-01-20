#!/usr/bin/env python3
"""
Rocket Ascent Simulation Example

Demonstrates a single-stage rocket ascent with:
- Mass depletion (FuelTank component)
- Throttleable engine (RocketEngine component)
- Full 6DOF rigid body dynamics (Vehicle6DOF unified component)
- Point mass gravity model

The Vehicle6DOF component combines MassAggregator + ForceAggregator + RigidBody6DOF
into a single unified component, dramatically simplifying signal routing.

The rocket starts at 100km altitude on the equator, pointing radially up (90 deg pitch),
and fires its engine at full throttle. The simulation tracks altitude, velocity,
fuel consumption, and verifies the physics are working correctly.

Usage:
    ./scripts/dev.sh python3 examples/python/rocket_ascent.py

Requirements:
    Build with --python flag: ./scripts/build.sh --python
"""

import sys
import math

# Add Python bindings to path
sys.path.insert(0, 'build/interfaces/python')

try:
    import _icarus as icarus
except ImportError:
    print("Error: Icarus Python bindings not found.")
    print("Build with: ./scripts/build.sh --python")
    sys.exit(1)


def print_header(title: str) -> None:
    """Print a formatted section header."""
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print('=' * 60)


def print_subheader(title: str) -> None:
    """Print a formatted subsection header."""
    print(f"\n--- {title} ---")


def main():
    print_header("Icarus Rocket Ascent Demo (Vehicle6DOF)")

    # =========================================================================
    # Load Configuration
    # =========================================================================

    config_path = "config/examples/rocket_ascent.yaml"
    print(f"Loading config: {config_path}")

    try:
        sim = icarus.Simulator(config_path)
    except Exception as e:
        print(f"Error loading config: {e}")
        print("Make sure to run from the project root directory.")
        sys.exit(1)

    # Stage the simulation (initializes all components)
    sim.stage()

    # Command full throttle before first step
    sim["Rocket.Engine.throttle_cmd"] = 1.0

    # Run one step to initialize all component outputs
    sim.step()

    # =========================================================================
    # Initial State (after first step to populate outputs)
    # =========================================================================

    print_subheader("Initial State")

    # Position (from Vehicle6DOF)
    px = sim['Rocket.Vehicle.position.x']
    py = sim['Rocket.Vehicle.position.y']
    pz = sim['Rocket.Vehicle.position.z']
    alt = sim['Rocket.Vehicle.position_lla.alt']
    lat = sim['Rocket.Vehicle.position_lla.lat']
    lon = sim['Rocket.Vehicle.position_lla.lon']
    print(f"Position ECEF: ({px/1e6:.3f}, {py/1e6:.3f}, {pz/1e6:.3f}) Mm")
    print(f"Position LLA:  lat={math.degrees(lat):.1f} deg, lon={math.degrees(lon):.1f} deg, alt={alt/1000:.1f} km")

    # Attitude (from Vehicle6DOF)
    euler_yaw = sim['Rocket.Vehicle.euler_zyx.yaw']
    euler_pitch = sim['Rocket.Vehicle.euler_zyx.pitch']
    euler_roll = sim['Rocket.Vehicle.euler_zyx.roll']
    print(f"Attitude:      yaw={math.degrees(euler_yaw):.1f} deg, pitch={math.degrees(euler_pitch):.1f} deg, roll={math.degrees(euler_roll):.1f} deg")

    # Mass properties (from Vehicle6DOF - aggregated internally)
    total_mass = sim['Rocket.Vehicle.total_mass']
    fuel_mass = sim['Rocket.FuelTank.fuel_mass']
    cgx = sim['Rocket.Vehicle.cg.x']
    print(f"Total mass:    {total_mass:.1f} kg (fuel: {fuel_mass:.1f} kg)")
    print(f"CG position:   {cgx:.3f} m (body X)")

    # =========================================================================
    # Forces and Moments
    # =========================================================================

    print_subheader("Forces at Ignition")

    # Check forces (from Vehicle6DOF - aggregated internally)
    thrust = sim['Rocket.Engine.thrust']
    fx = sim['Rocket.Vehicle.total_force.x']
    gx = sim['Rocket.Gravity.force.x']
    print(f"Engine thrust: {thrust/1000:.1f} kN")
    print(f"Gravity force: {gx/1000:.1f} kN (toward Earth center)")
    print(f"Net force:     {fx/1000:.1f} kN (body X = radially up)")

    # Check for spurious moments (should be zero for symmetric rocket)
    mx = sim['Rocket.Vehicle.total_moment.x']
    my = sim['Rocket.Vehicle.total_moment.y']
    mz = sim['Rocket.Vehicle.total_moment.z']
    moment_mag = math.sqrt(mx**2 + my**2 + mz**2)
    if moment_mag < 0.1:
        print(f"Moments:       ({mx:.1f}, {my:.1f}, {mz:.1f}) N*m [OK - symmetric]")
    else:
        print(f"WARNING: Non-zero moments: ({mx:.1f}, {my:.1f}, {mz:.1f}) N*m")

    # =========================================================================
    # Run Simulation
    # =========================================================================

    print_subheader("Ascent Trajectory (50 seconds)")

    dt = 0.01  # 100 Hz simulation rate
    duration = 50.0  # seconds
    steps = int(duration / dt)
    print_interval = int(5.0 / dt)  # Print every 5 seconds

    print(f"{'Time':>6s}  {'Alt (km)':>10s}  {'Vel (m/s)':>10s}  {'Mass (kg)':>10s}  {'Fuel (kg)':>10s}  {'Thrust (kN)':>12s}")
    print("-" * 70)

    for i in range(steps):
        sim.step()

        if (i + 1) % print_interval == 0:
            t = (i + 2) * dt  # +2 because we already did one step
            alt = sim['Rocket.Vehicle.position_lla.alt']
            vx = sim['Rocket.Vehicle.velocity_body.x']
            mass = sim['Rocket.Vehicle.total_mass']
            fuel = sim['Rocket.FuelTank.fuel_mass']
            thrust = sim['Rocket.Engine.thrust']

            print(f"{t:6.1f}  {alt/1000:10.2f}  {vx:10.1f}  {mass:10.0f}  {fuel:10.0f}  {thrust/1000:12.1f}")

    # =========================================================================
    # Final State
    # =========================================================================

    print_subheader("Final State")

    alt_final = sim['Rocket.Vehicle.position_lla.alt']
    vx_final = sim['Rocket.Vehicle.velocity_body.x']
    vy_final = sim['Rocket.Vehicle.velocity_body.y']
    vz_final = sim['Rocket.Vehicle.velocity_body.z']
    mass_final = sim['Rocket.Vehicle.total_mass']
    fuel_final = sim['Rocket.FuelTank.fuel_mass']

    fuel_consumed = fuel_mass - fuel_final
    alt_gained = alt_final - 0.0  # Started at 0km

    print(f"Altitude:      {alt_final/1000:.2f} km (+{alt_gained/1000:.2f} km)")
    print(f"Velocity:      {vx_final:.1f} m/s (radially up)")
    print(f"Lateral vel:   vy={vy_final:.3f} m/s, vz={vz_final:.3f} m/s (Coriolis effect)")
    print(f"Mass:          {mass_final:.0f} kg")
    print(f"Fuel consumed: {fuel_consumed:.1f} kg")

    # Verify attitude remained stable (no rotation)
    omega_x = sim['Rocket.Vehicle.omega_body.x']
    omega_y = sim['Rocket.Vehicle.omega_body.y']
    omega_z = sim['Rocket.Vehicle.omega_body.z']
    omega_mag = math.sqrt(omega_x**2 + omega_y**2 + omega_z**2)

    if omega_mag < 0.001:
        print(f"Angular rate:  {omega_mag:.6f} rad/s [OK - stable]")
    else:
        print(f"WARNING: Non-zero angular rate: {omega_mag:.4f} rad/s")

    # =========================================================================
    # Physics Sanity Check
    # =========================================================================

    print_subheader("Physics Verification")

    # Check basic physics relationships
    # Net force = Thrust - Gravity (both in body X direction)
    net_force = thrust - abs(gx)
    accel_initial = net_force / total_mass
    accel_final = net_force / mass_final  # Thrust and gravity roughly constant

    print(f"Initial acceleration: {accel_initial:.1f} m/s^2")
    print(f"Final acceleration:   {accel_final:.1f} m/s^2")
    print(f"Final velocity:       {vx_final:.1f} m/s")

    # Tsiolkovsky rocket equation check (for ideal case)
    # dv = Isp * g0 * ln(m0/mf), but we have constant thrust so it's simpler
    # For constant thrust: v = integral(F/m) dt, which with mass depletion is complex
    # Just verify altitude and velocity are increasing as expected
    checks_passed = 0
    checks_total = 3

    # Check 1: Altitude increased
    if alt_gained > 1000:  # Should gain at least 1 km
        print(f"Altitude gain:        PASS (+{alt_gained/1000:.2f} km)")
        checks_passed += 1
    else:
        print(f"Altitude gain:        FAIL (+{alt_gained/1000:.2f} km)")

    # Check 2: Velocity is reasonable (should be 1500-2000 m/s after 50s with mass depletion)
    if 1500 < vx_final < 2000:
        print(f"Velocity range:       PASS ({vx_final:.0f} m/s in expected range)")
        checks_passed += 1
    else:
        print(f"Velocity range:       FAIL ({vx_final:.0f} m/s outside 1500-2000 m/s)")

    # Check 3: Fuel consumption matches engine ISP
    # mdot = thrust / (Isp * g0) = 50000 / (311 * 9.81) ~ 16.4 kg/s
    expected_fuel_consumed = 16.4 * duration
    fuel_error = abs(fuel_consumed - expected_fuel_consumed) / expected_fuel_consumed * 100
    if fuel_error < 5:
        print(f"Fuel consumption:     PASS ({fuel_consumed:.1f} kg, expected ~{expected_fuel_consumed:.0f} kg)")
        checks_passed += 1
    else:
        print(f"Fuel consumption:     FAIL ({fuel_consumed:.1f} kg, expected ~{expected_fuel_consumed:.0f} kg)")

    print(f"\nResult: {checks_passed}/{checks_total} physics checks passed")

    print_header("Demo Complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
