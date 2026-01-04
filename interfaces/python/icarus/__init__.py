"""Icarus 6DOF Simulation Framework Python Bindings.

This package provides Python bindings for the Icarus simulation framework,
enabling interactive simulation, analysis, and Monte Carlo campaigns.

Basic Usage::

    import icarus

    # Create simulator from YAML config
    sim = icarus.Simulator("config.yaml")
    sim.stage()

    # Run simulation
    while sim.time < sim.end_time:
        sim.step()
        print(f"t={sim.time:.2f}, alt={sim['Ball.position.z']:.1f}")

Signal Access::

    # Get signal value
    altitude = sim.get("Vehicle.position.z")
    altitude = sim["Vehicle.position.z"]  # Same thing

    # Set signal value
    sim.set("Vehicle.position.z", 1000.0)
    sim["Vehicle.position.z"] = 1000.0  # Same thing

State Vector (for Monte Carlo)::

    import numpy as np

    # Get state as numpy array
    state = sim.state

    # Perturb and set back
    state += np.random.normal(0, 0.01, size=state.shape)
    sim.state = state

Version Info::

    print(icarus.__version__)      # "0.5.1"
    print(icarus.version_info)     # (0, 5, 1)
"""

from ._icarus import (
    # Core classes
    Simulator,
    Lifecycle,
    # Version info
    __version__,
    version_info,
    # Exception classes
    IcarusError,
    ConfigError,
    StageError,
    SignalNotFoundError,
    LifecycleError,
)

__all__ = [
    # Core
    "Simulator",
    "Lifecycle",
    # Exceptions
    "IcarusError",
    "ConfigError",
    "StageError",
    "SignalNotFoundError",
    "LifecycleError",
    # Version
    "__version__",
    "version_info",
]
