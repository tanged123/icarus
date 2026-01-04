# Python API Guide

**Related:** [C API Guide](c_api_guide.md) | [Component Authoring](component_authoring_guide.md)

---

The Icarus Python API provides a Pythonic interface to the simulation framework with full numpy integration. It's ideal for interactive analysis, Monte Carlo campaigns, and scripting.

## Installation

The Python bindings are built with the `--python` or `--all-interfaces` flag:

```bash
./scripts/build.sh --python
```

Then add the build directory to your Python path:

```bash
export PYTHONPATH=/path/to/icarus/build/python:$PYTHONPATH
```

Or in Python:

```python
import sys
sys.path.insert(0, "/path/to/icarus/build/python")
import icarus
```

## Quick Start

```python
import icarus

# Create simulator from YAML config
sim = icarus.Simulator("config/simulation.yaml")

# Stage (validate, wire, apply ICs)
sim.stage()

# Run simulation loop
while sim.time < sim.end_time:
    print(f"t={sim.time:.2f}, alt={sim['Vehicle.position.z']:.1f}")
    sim.step()
```

---

## API Reference

### Module Attributes

```python
import icarus

icarus.__version__    # "0.5.1"
icarus.version_info   # (0, 5, 1)
```

### Exception Classes

| Exception | Description |
|-----------|-------------|
| `icarus.IcarusError` | Base exception for all Icarus errors |
| `icarus.ConfigError` | Configuration loading failed |
| `icarus.StageError` | Staging failed (validation, wiring) |
| `icarus.SignalNotFoundError` | Signal name not found |
| `icarus.LifecycleError` | Invalid lifecycle state for operation |

```python
try:
    sim = icarus.Simulator("bad_config.yaml")
except icarus.ConfigError as e:
    print(f"Config error: {e}")

try:
    sim["NonExistent.Signal"]
except icarus.SignalNotFoundError:
    print("Signal not found")
```

### Lifecycle Enum

```python
from icarus import Lifecycle

Lifecycle.UNINITIALIZED  # 0 - Not yet configured
Lifecycle.PROVISIONED    # 1 - Components loaded, ready to stage
Lifecycle.STAGED         # 2 - Staged and ready to run
Lifecycle.RUNNING        # 3 - Simulation in progress
```

---

### Simulator Class

#### Constructor

```python
sim = icarus.Simulator(config_path: str)
```

Create simulator from YAML configuration file. Raises `ConfigError` on failure.

#### Lifecycle Methods

```python
sim.stage()        # Validate wiring, apply ICs, prepare for run
sim.step()         # Execute one step using configured dt
sim.step(0.005)    # Execute one step with explicit dt
sim.reset()        # Reset to initial conditions, time to 0
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `sim.lifecycle` | `Lifecycle` | Current lifecycle state |
| `sim.time` | `float` | Current simulation time (MET) in seconds |
| `sim.dt` | `float` | Configured timestep in seconds |
| `sim.end_time` | `float` | Configured end time in seconds |
| `sim.name` | `str` | Simulation name from config |
| `sim.flight_phase` | `int` | Current flight phase value |
| `sim.flight_phase_name` | `str` | Current flight phase name |

---

### Signal Access

Signals can be accessed by name using either method syntax or dictionary syntax:

```python
# Get signal value
altitude = sim.get("Vehicle.position.z")
altitude = sim["Vehicle.position.z"]  # Same thing

# Set signal value
sim.set("Vehicle.position.z", 1000.0)
sim["Vehicle.position.z"] = 1000.0    # Same thing
```

Signal names use dot notation: `"Component.signal.axis"`.

---

### State Vector (Numpy Integration)

The state vector provides high-performance bulk access for:
- Monte Carlo IC perturbation
- External integrators
- Checkpointing / warmstart
- High-frequency logging

> **Note:** States ARE signals in Icarus. Every state is also accessible via `sim["name"]`. The state vector is an optimization for bulk operations.

```python
import numpy as np

# Get state as numpy array
state = sim.state           # Returns np.ndarray
print(f"State size: {sim.state_size}")

# Modify and set back
state = sim.state.copy()    # Copy if modifying
state[0] += 1.0
sim.state = state

# Get state signal names (in state vector order)
names = sim.state_names     # List of signal names
print(f"State[0] is: {names[0]}")
```

---

### Introspection

```python
# List all signal names
signals = sim.signals       # List[str]
print(f"Total signals: {sim.signal_count}")

# Get data dictionary as Python dict
schema = sim.schema_json
print(f"Components: {len(schema['components'])}")
print(f"Total outputs: {schema['summary']['total_outputs']}")

# Export current values as dict
data = sim.to_dict()        # Dict[str, float]
print(f"Altitude: {data['Vehicle.position.z']}")
```

---

### Utility Methods

#### `run_until`

```python
sim.run_until(end_time: float, callback: Callable = None)
```

Run simulation until `end_time`. Optional callback is called after each step.

```python
# Simple run
sim.run_until(10.0)

# With callback
times = []
altitudes = []

def record(s):
    times.append(s.time)
    altitudes.append(s["Vehicle.position.z"])

sim.run_until(10.0, record)
```

#### `compute_derivatives` (Expert)

```python
xdot = sim.compute_derivatives(t: float)  # Returns np.ndarray
```

Compute state derivatives at time `t`. For external integrators.

---

## Usage Patterns

### Basic Simulation Loop

```python
import icarus

sim = icarus.Simulator("config.yaml")
sim.stage()

# Collect data during run
data = {"time": [], "altitude": [], "velocity": []}

while sim.time < sim.end_time:
    data["time"].append(sim.time)
    data["altitude"].append(sim["Vehicle.position.z"])
    data["velocity"].append(sim["Vehicle.velocity.z"])
    sim.step()

# Plot results
import matplotlib.pyplot as plt
plt.plot(data["time"], data["altitude"])
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.show()
```

### Monte Carlo Analysis

```python
import icarus
import numpy as np

def run_monte_carlo(config_path, num_runs=100, seed=42):
    rng = np.random.default_rng(seed)
    results = []

    for i in range(num_runs):
        sim = icarus.Simulator(config_path)
        sim.stage()

        # Perturb initial state
        state = sim.state.copy()
        state += rng.normal(0, 0.01, size=state.shape)
        sim.state = state

        # Run to completion
        sim.run_until(sim.end_time)

        # Collect result
        results.append({
            "final_altitude": sim["Vehicle.position.z"],
            "final_velocity": sim["Vehicle.velocity.z"],
        })

    return results

results = run_monte_carlo("config/ballistic.yaml", num_runs=1000)

# Analyze
import pandas as pd
df = pd.DataFrame(results)
print(df.describe())
```

### Parallel Monte Carlo with multiprocessing

```python
import icarus
import numpy as np
from multiprocessing import Pool

def run_single(args):
    config_path, seed = args
    rng = np.random.default_rng(seed)

    sim = icarus.Simulator(config_path)
    sim.stage()

    state = sim.state.copy()
    state += rng.normal(0, 0.01, size=state.shape)
    sim.state = state

    sim.run_until(sim.end_time)

    return sim["Vehicle.position.z"]

if __name__ == "__main__":
    config = "config/ballistic.yaml"
    seeds = range(1000)

    with Pool() as pool:
        results = pool.map(run_single, [(config, s) for s in seeds])

    print(f"Mean altitude: {np.mean(results):.2f}")
    print(f"Std deviation: {np.std(results):.2f}")
```

### Checkpointing / Warmstart

```python
import icarus
import numpy as np

sim = icarus.Simulator("config.yaml")
sim.stage()

# Run to checkpoint time
sim.run_until(10.0)

# Save checkpoint
checkpoint_state = sim.state.copy()
checkpoint_time = sim.time

print(f"Checkpoint at t={checkpoint_time}")

# Continue running
sim.run_until(20.0)
print(f"Final altitude: {sim['Vehicle.position.z']:.1f}")

# Restore and try different scenario
sim.reset()
sim.state = checkpoint_state
# (Note: time resets to 0; for full warmstart, use save/load files)

sim["Vehicle.force.z"] = 1000.0  # Add thrust
sim.run_until(20.0)
print(f"With thrust: {sim['Vehicle.position.z']:.1f}")
```

### Interactive Exploration (Jupyter)

```python
import icarus
import numpy as np
import matplotlib.pyplot as plt
from IPython.display import clear_output

sim = icarus.Simulator("config.yaml")
sim.stage()

# Interactive stepping
fig, ax = plt.subplots()

for _ in range(100):
    sim.step()

    # Live plot
    clear_output(wait=True)
    ax.clear()
    ax.set_title(f"t = {sim.time:.2f} s")
    ax.bar(["x", "y", "z"], [
        sim["Vehicle.position.x"],
        sim["Vehicle.position.y"],
        sim["Vehicle.position.z"]
    ])
    plt.pause(0.01)
```

### Sensitivity Analysis

```python
import icarus
import numpy as np

def run_with_param(config, param_name, param_value):
    sim = icarus.Simulator(config)
    sim.stage()
    sim[param_name] = param_value
    sim.run_until(sim.end_time)
    return sim["Vehicle.position.z"]

# Sweep parameter
masses = np.linspace(0.5, 2.0, 20)
altitudes = [run_with_param("config.yaml", "Vehicle.mass", m) for m in masses]

import matplotlib.pyplot as plt
plt.plot(masses, altitudes)
plt.xlabel("Mass (kg)")
plt.ylabel("Final Altitude (m)")
plt.title("Sensitivity to Mass")
plt.show()
```

---

## Integration with Scientific Python

### NumPy

```python
import numpy as np

# State vector is a numpy array
state = sim.state
print(f"Shape: {state.shape}, dtype: {state.dtype}")

# Apply transformations
state = np.clip(state, -100, 100)
sim.state = state
```

### Pandas

```python
import pandas as pd

# Export to DataFrame
data = []
sim.stage()
while sim.time < sim.end_time:
    row = {"time": sim.time}
    row.update(sim.to_dict())
    data.append(row)
    sim.step()

df = pd.DataFrame(data)
df.to_csv("results.csv")
```

### SciPy (External Integration)

```python
import icarus
import numpy as np
from scipy.integrate import solve_ivp

sim = icarus.Simulator("config.yaml")
sim.stage()

def dynamics(t, y):
    sim.state = y
    return sim.compute_derivatives(t)

# Use SciPy's RK45
sol = solve_ivp(
    dynamics,
    [0, sim.end_time],
    sim.state.copy(),
    method='RK45',
    max_step=sim.dt
)

print(f"Final state: {sol.y[:, -1]}")
```

---

## Error Handling

```python
import icarus

# Config errors
try:
    sim = icarus.Simulator("nonexistent.yaml")
except icarus.ConfigError as e:
    print(f"Failed to load config: {e}")

# Signal errors
sim = icarus.Simulator("config.yaml")
sim.stage()

try:
    value = sim["NonExistent.Signal"]
except icarus.SignalNotFoundError:
    print("Signal not found")

# Lifecycle errors
try:
    sim.step()  # Without stage()
except icarus.LifecycleError:
    print("Must stage before stepping")
```

---

## Best Practices

1. **Copy state before modifying**: `state = sim.state.copy()` to avoid unintended side effects

2. **Use run_until for simple runs**: More efficient than manual loops

3. **Prefer `sim["name"]` syntax**: More Pythonic than `sim.get("name")`

4. **Use multiprocessing for Monte Carlo**: Each process gets its own simulator

5. **Check lifecycle state**: `sim.lifecycle == Lifecycle.STAGED` before operations

---

## See Also

- [C API Guide](c_api_guide.md) - Lower-level C interface
- [Component Authoring Guide](component_authoring_guide.md) - Writing custom components
- [interfaces/python/icarus/__init__.py](../../interfaces/python/icarus/__init__.py) - Package documentation
