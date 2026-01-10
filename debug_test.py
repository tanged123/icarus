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

print("\n=== Initial Quaternion ===")
qw = sim['Rocket.EOM.attitude.w']
qx = sim['Rocket.EOM.attitude.x']
qy = sim['Rocket.EOM.attitude.y']
qz = sim['Rocket.EOM.attitude.z']
print(f"Quat: ({qw:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f})")

print("\n=== Stepping 10 times ===")
for i in range(10):
    sim.step()
    qw = sim['Rocket.EOM.attitude.w']
    qx = sim['Rocket.EOM.attitude.x']
    qy = sim['Rocket.EOM.attitude.y']
    qz = sim['Rocket.EOM.attitude.z']
    ox = sim['Rocket.EOM.omega_body.x']
    oy = sim['Rocket.EOM.omega_body.y']
    oz = sim['Rocket.EOM.omega_body.z']
    print(f"Step {i+1}: quat=({qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f}), omega=({ox:.4f}, {oy:.4f}, {oz:.4f})")
