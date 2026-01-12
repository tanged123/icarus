# Python Examples

Python examples demonstrating the Icarus simulation engine.

## Prerequisites

Build Icarus with Python bindings enabled:

```bash
./scripts/build.sh --python
```

## Running Examples

Run examples from the project root directory:

```bash
./scripts/dev.sh python3 examples/python/rocket_ascent.py
```

## Examples

### rocket_ascent.py

Single-stage rocket vertical ascent simulation demonstrating:

- **6DOF Rigid Body Dynamics** - Full translational and rotational motion
- **Mass Depletion** - FuelTank component with propellant consumption
- **Rocket Engine** - Throttleable thrust with ISP-based mass flow
- **Point Mass Gravity** - Spherical Earth gravity model
- **Force/Mass Aggregation** - Multiple sources combined correctly

The rocket starts at 100km altitude on the equator, fires at full throttle,
and ascends while consuming fuel. The example verifies:

- Altitude and velocity increase correctly
- Fuel consumption matches engine ISP
- No spurious rotation (moments are zero for symmetric config)
