# Icarus Components

This directory contains the built-in component library for Icarus.

## Directory Structure

```
components/
├── eom/           # Equations of Motion (RigidBody3DOF, RigidBody6DOF)
├── environment/   # Atmosphere, Gravity, Wind
├── propulsion/    # RocketEngine, JetEngine, FuelTank
├── aerodynamics/  # AeroBody, AeroTable
├── structure/     # Mass properties
├── gnc/           # Autopilot, NavigationFilter
├── sensors/       # IMU, GPS
└── actuators/     # TVC, ControlSurface
```

## Adding Components

Components are added incrementally during Phase 2+ of implementation.
See `docs/implementation_plans/icarus_implementation_plan.md` for the roadmap.
