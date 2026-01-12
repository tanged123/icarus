#!/usr/bin/env python3
import sys
sys.path.insert(0, 'build/interfaces/python')
import _icarus as icarus
import math

sim = icarus.Simulator("config/examples/rocket_ascent.yaml")
sim.stage()

print("=== After Stage (before Step) ===")
qw = sim['Rocket.EOM.attitude.w']
qx = sim['Rocket.EOM.attitude.x']
qy = sim['Rocket.EOM.attitude.y']
qz = sim['Rocket.EOM.attitude.z']
print(f"Initial Quat: ({qw:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f})")

# Check euler output
euler_yaw = sim['Rocket.EOM.euler_zyx.yaw']
euler_pitch = sim['Rocket.EOM.euler_zyx.pitch']
euler_roll = sim['Rocket.EOM.euler_zyx.roll']
import math
print(f"Euler (deg): yaw={math.degrees(euler_yaw):.1f}, pitch={math.degrees(euler_pitch):.1f}, roll={math.degrees(euler_roll):.1f}")

# Expected quaternion for 90° pitch from NED at equator/prime meridian:
# At (0°, 0°), NED X = North = -ECEF_Z, NED Z = Down = -ECEF_X
# With 90° pitch, body +X points up along -NED_Z = +ECEF_X (radially outward)
# Actually, let me think about this more carefully...
print(f"Expected: 90° pitch means nose points radially up")

px = sim['Rocket.EOM.position.x']
py = sim['Rocket.EOM.position.y']
pz = sim['Rocket.EOM.position.z']
print(f"Initial Pos ECEF: ({px:.1f}, {py:.1f}, {pz:.1f}) m")

# Set throttle and step
sim["Rocket.Engine.throttle_cmd"] = 1.0
sim.step()

print("=== Forces and Moments ===")
fx = sim['Rocket.Forces.total_force.x']
fy = sim['Rocket.Forces.total_force.y']
fz = sim['Rocket.Forces.total_force.z']
print(f"Total force (body): ({fx:.1f}, {fy:.1f}, {fz:.1f}) N")

mx = sim['Rocket.Forces.total_moment.x']
my = sim['Rocket.Forces.total_moment.y']
mz = sim['Rocket.Forces.total_moment.z']
print(f"Total moment (body): ({mx:.1f}, {my:.1f}, {mz:.1f}) N*m")

# Check gravity output
gx = sim['Rocket.Gravity.force.x']
gy = sim['Rocket.Gravity.force.y']
gz = sim['Rocket.Gravity.force.z']
print(f"Gravity force: ({gx:.1f}, {gy:.1f}, {gz:.1f}) N")

# Check engine output
ex = sim['Rocket.Engine.force.x']
ey = sim['Rocket.Engine.force.y']
ez = sim['Rocket.Engine.force.z']
print(f"Engine force: ({ex:.1f}, {ey:.1f}, {ez:.1f}) N")

# Check CG from mass aggregator
cgx = sim['Rocket.Mass.cg.x']
cgy = sim['Rocket.Mass.cg.y']
cgz = sim['Rocket.Mass.cg.z']
print(f"CG position: ({cgx:.3f}, {cgy:.3f}, {cgz:.3f}) m")

# Check engine application point
appx = sim['Rocket.Engine.application_point.x']
appy = sim['Rocket.Engine.application_point.y']
appz = sim['Rocket.Engine.application_point.z']
print(f"Engine app point: ({appx:.3f}, {appy:.3f}, {appz:.3f}) m")

# Check gravity acceleration (should point toward -X in ECEF for equator position)
gax = sim['Rocket.Gravity.acceleration.x']
gay = sim['Rocket.Gravity.acceleration.y']
gaz = sim['Rocket.Gravity.acceleration.z']
print(f"Gravity accel: ({gax:.3f}, {gay:.3f}, {gaz:.3f}) m/s^2")
print(f"Expected accel: (-9.8, 0, 0) for ECEF at equator")

# Check what position Gravity is reading
gpx = sim['Rocket.Gravity.position.x']
gpy = sim['Rocket.Gravity.position.y']
gpz = sim['Rocket.Gravity.position.z']
print(f"Gravity input pos: ({gpx:.1f}, {gpy:.1f}, {gpz:.1f}) m")

# Check mass
mass = sim['Rocket.Mass.total_mass']
print(f"Total mass: {mass:.1f} kg")

print("\n=== Initial Quaternion ===")
qw = sim['Rocket.EOM.attitude.w']
qx = sim['Rocket.EOM.attitude.x']
qy = sim['Rocket.EOM.attitude.y']
qz = sim['Rocket.EOM.attitude.z']
print(f"Quat: ({qw:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f})")

print("\n=== Stepping 1000 times (10 seconds simulation) ===")
for i in range(1000):
    sim.step()
    if (i + 1) % 100 == 0:  # Print every 100 steps (1 second)
        t = (i + 1) * 0.01  # dt = 0.01s
        alt = sim['Rocket.EOM.position_lla.alt']
        vx = sim['Rocket.EOM.velocity_body.x']
        fuel = sim['Rocket.FuelTank.fuel_mass']
        mass = sim['Rocket.Mass.total_mass']
        thrust = sim['Rocket.Engine.thrust']
        print(f"t={t:.1f}s: alt={alt:.0f}m, v={vx:.1f} m/s, mass={mass:.0f}kg, fuel={fuel:.0f}kg, thrust={thrust/1000:.1f}kN")
