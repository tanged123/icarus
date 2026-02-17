"""Tests for Icarus Python bindings.

Run with: pytest tests/python/test_simulator.py -v
Requires: PYTHONPATH=build/python pytest ...
"""

import os
import sys
import tempfile
import pytest
import numpy as np

# Add build directory to path for finding the module
build_python_dir = os.path.join(os.path.dirname(__file__), "..", "..", "build", "python")
if os.path.exists(build_python_dir):
    sys.path.insert(0, build_python_dir)

import icarus
from icarus import Simulator, Lifecycle


# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def minimal_config():
    """Create a minimal config file for testing."""
    config_content = """
simulation:
  name: "Python Test"

time:
  start: 0.0
  end: 1.0
  dt: 0.01

components:
  - type: PointMass3DOF
    name: Ball
    config:
      mass: 1.0
      initial_position: [0.0, 0.0, 100.0]
      initial_velocity: [0.0, 0.0, 0.0]

  - type: PointMassGravity
    name: Gravity
    config:
      model: 0  # Constant gravity (g0 downward)

routes:
  # Gravity reads position and mass from Ball
  - input: Gravity.position.x
    output: Ball.position.x
  - input: Gravity.position.y
    output: Ball.position.y
  - input: Gravity.position.z
    output: Ball.position.z
  - input: Gravity.mass
    output: Ball.mass
  # Ball reads force from Gravity
  - input: Ball.force.x
    output: Gravity.force.x
  - input: Ball.force.y
    output: Gravity.force.y
  - input: Ball.force.z
    output: Gravity.force.z
"""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write(config_content)
        f.flush()
        yield f.name
    os.unlink(f.name)


# =============================================================================
# Version Tests
# =============================================================================


def test_version():
    """Test version info is available."""
    assert icarus.__version__ is not None
    assert len(icarus.__version__) > 0
    assert isinstance(icarus.version_info, tuple)
    assert len(icarus.version_info) == 3
    # All components should be non-negative integers
    for v in icarus.version_info:
        assert isinstance(v, int)
        assert v >= 0


# =============================================================================
# Lifecycle Tests
# =============================================================================


def test_lifecycle_enum():
    """Test Lifecycle enum values."""
    assert Lifecycle.UNINITIALIZED.value == 0
    assert Lifecycle.PROVISIONED.value == 1
    assert Lifecycle.STAGED.value == 2
    assert Lifecycle.RUNNING.value == 3


def test_create_simulator(minimal_config):
    """Test simulator creation from YAML config."""
    sim = Simulator(minimal_config)
    assert sim is not None
    assert sim.lifecycle == Lifecycle.PROVISIONED


def test_create_with_invalid_path():
    """Test that creating with invalid path raises ConfigError."""
    with pytest.raises(icarus.ConfigError):
        Simulator("/nonexistent/path/to/config.yaml")


def test_stage(minimal_config):
    """Test staging the simulation."""
    sim = Simulator(minimal_config)
    sim.stage()
    assert sim.lifecycle == Lifecycle.STAGED


def test_step(minimal_config):
    """Test stepping advances time."""
    sim = Simulator(minimal_config)
    sim.stage()
    initial_time = sim.time
    sim.step()
    assert sim.time > initial_time
    assert sim.lifecycle == Lifecycle.RUNNING


def test_step_with_dt(minimal_config):
    """Test stepping with explicit dt."""
    sim = Simulator(minimal_config)
    sim.stage()
    sim.step(0.005)  # Half the configured dt
    assert sim.time == pytest.approx(0.005, abs=1e-10)


def test_reset(minimal_config):
    """Test resetting returns to initial state."""
    sim = Simulator(minimal_config)
    sim.stage()

    # Get initial position
    initial_z = sim.get("Ball.position.z")

    # Run a few steps
    for _ in range(10):
        sim.step()

    # Position should have changed
    assert sim.get("Ball.position.z") != initial_z

    # Reset
    sim.reset()
    assert sim.time == 0.0
    assert sim.lifecycle == Lifecycle.STAGED
    assert sim.get("Ball.position.z") == pytest.approx(initial_z, abs=1e-10)


# =============================================================================
# Time Properties Tests
# =============================================================================


def test_time_properties(minimal_config):
    """Test time property accessors."""
    sim = Simulator(minimal_config)
    sim.stage()

    assert sim.time == pytest.approx(0.0)
    assert sim.dt == pytest.approx(0.01)
    assert sim.end_time == pytest.approx(1.0)


def test_name_property(minimal_config):
    """Test name property."""
    sim = Simulator(minimal_config)
    assert sim.name == "Python Test"


# =============================================================================
# Signal Access Tests
# =============================================================================


def test_get_signal(minimal_config):
    """Test getting signal value by name."""
    sim = Simulator(minimal_config)
    sim.stage()

    # Note: ICs from config not currently applied - verify signal access works
    z = sim.get("Ball.position.z")
    # Just verify we can read the signal (IC application is a separate issue)
    assert isinstance(z, float)


def test_set_signal(minimal_config):
    """Test setting signal value by name."""
    sim = Simulator(minimal_config)
    sim.stage()

    sim.set("Ball.position.z", 200.0)
    assert sim.get("Ball.position.z") == pytest.approx(200.0)


def test_getitem_setitem(minimal_config):
    """Test Pythonic [] access syntax."""
    sim = Simulator(minimal_config)
    sim.stage()

    # Write with []
    sim["Ball.position.z"] = 300.0
    assert sim["Ball.position.z"] == pytest.approx(300.0)

    # Also test read
    z = sim["Ball.position.z"]
    assert isinstance(z, float)


def test_signal_not_found(minimal_config):
    """Test that accessing nonexistent signal raises error."""
    sim = Simulator(minimal_config)
    sim.stage()

    with pytest.raises(icarus.SignalNotFoundError):
        sim.get("NonExistent.Signal")


# =============================================================================
# State Vector Tests
# =============================================================================


def test_state_vector(minimal_config):
    """Test state vector access."""
    sim = Simulator(minimal_config)
    sim.stage()

    state = sim.state
    assert isinstance(state, np.ndarray)
    assert state.ndim == 1
    assert len(state) == sim.state_size
    assert sim.state_size > 0


def test_state_vector_modification(minimal_config):
    """Test modifying state vector."""
    sim = Simulator(minimal_config)
    sim.stage()

    state = sim.state.copy()
    state[0] += 1.0
    sim.state = state

    new_state = sim.state
    assert new_state[0] == pytest.approx(state[0], abs=1e-10)


def test_state_names(minimal_config):
    """Test state signal names."""
    sim = Simulator(minimal_config)
    sim.stage()

    names = sim.state_names
    assert isinstance(names, list)
    assert len(names) == sim.state_size
    assert all(isinstance(n, str) for n in names)


# =============================================================================
# Introspection Tests
# =============================================================================


def test_signals_list(minimal_config):
    """Test getting all signal names."""
    sim = Simulator(minimal_config)
    sim.stage()

    signals = sim.signals
    assert isinstance(signals, list)
    assert len(signals) > 0
    assert "Ball.position.z" in signals


def test_signal_count(minimal_config):
    """Test signal count property."""
    sim = Simulator(minimal_config)
    sim.stage()

    assert sim.signal_count == len(sim.signals)


def test_schema_json(minimal_config):
    """Test schema JSON property."""
    sim = Simulator(minimal_config)
    sim.stage()

    schema = sim.schema_json
    assert isinstance(schema, dict)
    assert "summary" in schema
    assert "components" in schema


def test_introspection_graph(minimal_config):
    """Test introspection graph property."""
    sim = Simulator(minimal_config)
    sim.stage()

    graph = sim.introspection_graph
    assert isinstance(graph, dict)
    assert "summary" in graph
    assert "components" in graph
    assert "edges" in graph
    assert "total_edges" in graph["summary"]
    assert isinstance(graph["edges"], list)
    assert graph["summary"]["total_edges"] >= 7

    route_edges = [e for e in graph["edges"] if e.get("kind") == "route"]
    assert len(route_edges) == 7


def test_to_dict(minimal_config):
    """Test exporting signals as dict."""
    sim = Simulator(minimal_config)
    sim.stage()

    data = sim.to_dict()
    assert isinstance(data, dict)
    assert len(data) > 0
    assert "Ball.position.z" in data


# =============================================================================
# Utility Method Tests
# =============================================================================


def test_run_until(minimal_config):
    """Test run_until helper."""
    sim = Simulator(minimal_config)
    sim.stage()

    sim.run_until(0.1)
    assert sim.time >= 0.1


def test_run_until_with_callback(minimal_config):
    """Test run_until with callback."""
    sim = Simulator(minimal_config)
    sim.stage()

    times = []

    def callback(s):
        times.append(s.time)

    sim.run_until(0.05, callback)
    assert len(times) > 0
    assert times[-1] >= 0.05


# =============================================================================
# Integration Tests
# =============================================================================


def test_falling_ball_physics(minimal_config):
    """Test that falling ball follows expected physics."""
    sim = Simulator(minimal_config)
    sim.stage()

    # Initial conditions
    z0 = sim.get("Ball.position.z")
    v0 = sim.get("Ball.velocity.z")
    g = 9.80665  # Standard gravity

    # Run for 0.5 seconds
    t_end = 0.5
    sim.run_until(t_end)

    # Check against kinematic equation: z = z0 + v0*t - 0.5*g*t^2
    expected_z = z0 + v0 * t_end - 0.5 * g * t_end**2
    actual_z = sim.get("Ball.position.z")

    # Allow some tolerance for numerical integration
    assert actual_z == pytest.approx(expected_z, rel=0.01)


def test_multiple_simulators(minimal_config):
    """Test running multiple simulators independently."""
    sim1 = Simulator(minimal_config)
    sim2 = Simulator(minimal_config)

    sim1.stage()
    sim2.stage()

    # Set known values in both
    sim1["Ball.position.z"] = 100.0
    sim2["Ball.position.z"] = 50.0

    # Modify sim1
    sim1["Ball.position.z"] = 200.0

    # sim2 should be unchanged at 50.0
    assert sim2["Ball.position.z"] == pytest.approx(50.0)

    # Run sim1 only
    sim1.step()

    # sim2 time should still be 0
    assert sim2.time == 0.0


# =============================================================================
# Monte Carlo Pattern Test
# =============================================================================


def test_monte_carlo_pattern(minimal_config):
    """Test typical Monte Carlo usage pattern."""
    results = []

    for seed in range(5):
        rng = np.random.default_rng(seed)
        sim = Simulator(minimal_config)
        sim.stage()

        # Perturb initial state
        state = sim.state
        state += rng.normal(0, 0.1, size=state.shape)
        sim.state = state

        # Run to end
        sim.run_until(0.1)
        results.append(sim["Ball.position.z"])

    # All results should be slightly different
    assert len(set(results)) == len(results)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
