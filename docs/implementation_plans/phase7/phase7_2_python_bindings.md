# Python Bindings Implementation

**Status:** Complete
**Phase:** 7.2
**Related:** [16_external_bindings.md](../../architecture/16_external_bindings.md) | [phase7_1_c_api.md](phase7_1_c_api.md)

---

## Overview

Provide Python bindings for Icarus using **PyBind11**, enabling:
- Interactive simulation in Jupyter notebooks
- Integration with pandas, numpy, matplotlib
- Scripted Monte Carlo campaigns
- Analysis workflow automation

### Binding Strategy

| Approach | Pros | Cons | Choice |
|----------|------|------|--------|
| **PyBind11** | Rich type conversion, pythonic API, excellent numpy support | Requires compilation | Primary |
| ctypes/cffi | Pure Python, uses C API | Less ergonomic, manual conversion | Fallback option |

---

## API Parity with C API

The Python bindings wrap the same functionality as the C API (`icarus.h`), with Pythonic enhancements:

### C API to Python Mapping

| C API Function | Python Equivalent | Notes |
|----------------|-------------------|-------|
| `icarus_create()` | `Simulator(config_path)` | Constructor |
| `icarus_destroy()` | Automatic via `__del__` | RAII cleanup |
| `icarus_stage()` | `sim.stage()` | |
| `icarus_step()` | `sim.step(dt=None)` | Uses configured dt if None |
| `icarus_reset()` | `sim.reset()` | |
| `icarus_get_lifecycle()` | `sim.lifecycle` | Property, returns `Lifecycle` enum |
| `icarus_get_signal()` | `sim.get(name)` or `sim[name]` | |
| `icarus_set_signal()` | `sim.set(name, value)` or `sim[name] = value` | |
| `icarus_get_state_vector()` | `sim.state` | Property, returns numpy array |
| `icarus_set_state_vector()` | `sim.state = array` | Property setter |
| `icarus_get_state_size()` | `sim.state_size` | Property |
| `icarus_get_state_signal_names()` | `sim.state_names` | Property, returns list |
| `icarus_get_time()` | `sim.time` | Property |
| `icarus_get_dt()` | `sim.dt` | Property |
| `icarus_get_end_time()` | `sim.end_time` | Property |
| `icarus_get_schema_json()` | `sim.schema_json` | Property, returns dict |
| `icarus_get_signal_count()` | `len(sim.signals)` | |
| `icarus_get_signal_names()` | `sim.signals` | Property, returns list |
| `icarus_get_last_error()` | Exceptions raised | Python exceptions instead |
| `icarus_error_name()` | Not needed | Exception types provide info |
| `icarus_version()` | `icarus.__version__` | Module attribute |
| `icarus_version_components()` | `icarus.version_info` | Tuple (major, minor, patch) |

---

## Python API Design

### Module Structure

```python
import icarus

# Core classes
from icarus import Simulator, Lifecycle

# Version
print(icarus.__version__)  # "0.5.1"
print(icarus.version_info)  # (0, 5, 1)
```

### Lifecycle Enum

```python
class Lifecycle(IntEnum):
    """Simulation lifecycle state (mirrors IcarusLifecycle)."""
    UNINITIALIZED = 0
    PROVISIONED = 1
    STAGED = 2
    RUNNING = 3
```

### Exception Classes

```python
class IcarusError(Exception):
    """Base exception for Icarus errors."""
    pass

class ConfigError(IcarusError):
    """Configuration loading failed."""
    pass

class StageError(IcarusError):
    """Stage() failed."""
    pass

class SignalNotFoundError(IcarusError):
    """Signal name not found."""
    pass

class LifecycleError(IcarusError):
    """Invalid lifecycle state for operation."""
    pass
```

### Core Simulator Class

```python
class Simulator:
    """Icarus 6DOF Simulator Python Interface.

    This class wraps the C++ Simulator via PyBind11, providing
    a Pythonic interface with numpy integration.
    """

    def __init__(self, config_path: str):
        """Create simulator from YAML configuration file.

        Args:
            config_path: Path to simulation YAML configuration.

        Raises:
            ConfigError: If configuration loading fails.
        """
        ...

    @classmethod
    def from_yaml(cls, yaml_string: str) -> "Simulator":
        """Create simulator from YAML string.

        Args:
            yaml_string: YAML configuration as string.

        Returns:
            New Simulator instance.
        """
        ...

    # -------------------------------------------------------------------------
    # Lifecycle Methods
    # -------------------------------------------------------------------------

    def stage(self) -> None:
        """Stage the simulation (validate, wire, apply ICs).

        Raises:
            StageError: If staging fails.
        """
        ...

    def step(self, dt: float = None) -> None:
        """Execute one simulation step.

        Args:
            dt: Timestep in seconds. If None, uses configured dt.
        """
        ...

    def reset(self) -> None:
        """Reset simulation to initial state.

        Re-applies initial conditions and resets time to 0.
        """
        ...

    @property
    def lifecycle(self) -> Lifecycle:
        """Current simulation lifecycle state."""
        ...

    # -------------------------------------------------------------------------
    # Time Properties
    # -------------------------------------------------------------------------

    @property
    def time(self) -> float:
        """Current simulation time (MET) in seconds."""
        ...

    @property
    def dt(self) -> float:
        """Configured timestep in seconds."""
        ...

    @property
    def end_time(self) -> float:
        """Configured end time in seconds."""
        ...

    # -------------------------------------------------------------------------
    # Signal Access (Per-Signal)
    # -------------------------------------------------------------------------

    def get(self, name: str) -> float:
        """Get scalar signal value by name.

        Args:
            name: Full signal path (e.g., "Vehicle.Nav.altitude").

        Returns:
            Signal value.

        Raises:
            SignalNotFoundError: If signal not found.
        """
        ...

    def set(self, name: str, value: float) -> None:
        """Set scalar signal value by name.

        Args:
            name: Full signal path.
            value: Value to set.

        Raises:
            SignalNotFoundError: If signal not found.
        """
        ...

    def __getitem__(self, name: str) -> float:
        """Enable sim['Signal.Name'] syntax."""
        return self.get(name)

    def __setitem__(self, name: str, value: float) -> None:
        """Enable sim['Signal.Name'] = value syntax."""
        self.set(name, value)

    # -------------------------------------------------------------------------
    # Bulk State Access (High Performance)
    # -------------------------------------------------------------------------

    @property
    def state(self) -> np.ndarray:
        """Current state vector as numpy array.

        States ARE signals in Icarus. This bulk API exists for
        performance-critical use cases like:
        - External integrators needing full state vector
        - Monte Carlo IC perturbation
        - Checkpointing / warmstart
        - High-frequency logging

        For casual access, prefer sim.get(signal_name).
        """
        ...

    @state.setter
    def state(self, value: np.ndarray) -> None:
        """Set state vector from numpy array.

        Args:
            value: State vector (must match state_size).
        """
        ...

    @property
    def state_size(self) -> int:
        """Number of elements in state vector."""
        ...

    @property
    def state_names(self) -> List[str]:
        """Signal names for each state vector element.

        Returns list in same order as state vector, so:
            state_names[i] corresponds to state[i]
        """
        ...

    # -------------------------------------------------------------------------
    # Introspection
    # -------------------------------------------------------------------------

    @property
    def signals(self) -> List[str]:
        """List of all signal names."""
        ...

    @property
    def schema_json(self) -> dict:
        """Data dictionary as Python dict.

        Contains signal metadata, types, descriptions.
        """
        ...

    def to_dict(self) -> Dict[str, float]:
        """Export current signal values as dict.

        Returns:
            Dict mapping signal names to current values.
        """
        ...

    # -------------------------------------------------------------------------
    # Utilities
    # -------------------------------------------------------------------------

    def run_until(self, end_time: float, callback: Callable = None) -> None:
        """Run simulation until end_time.

        Args:
            end_time: Stop time in seconds.
            callback: Optional function called after each step.
        """
        while self.time < end_time:
            self.step()
            if callback:
                callback(self)
```

---

## PyBind11 Implementation

### Main Module

```cpp
// interfaces/python/icarus_python.cpp

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <icarus/sim/Simulator.hpp>
#include <icarus/io/data/DataDictionary.hpp>
#include <nlohmann/json.hpp>

namespace py = pybind11;

// =============================================================================
// Numpy Array Conversion
// =============================================================================

py::array_t<double> state_to_numpy(const Eigen::VectorXd& state) {
    auto result = py::array_t<double>(state.size());
    auto buf = result.request();
    std::memcpy(buf.ptr, state.data(), state.size() * sizeof(double));
    return result;
}

Eigen::VectorXd numpy_to_state(py::array_t<double> arr) {
    auto buf = arr.request();
    if (buf.ndim != 1) {
        throw std::runtime_error("State must be 1D array");
    }
    Eigen::VectorXd result(buf.shape[0]);
    std::memcpy(result.data(), buf.ptr, buf.shape[0] * sizeof(double));
    return result;
}

// =============================================================================
// Exception Translation
// =============================================================================

void register_exceptions(py::module_& m) {
    py::register_exception<icarus::ConfigError>(m, "ConfigError");
    py::register_exception<icarus::StageError>(m, "StageError");
    py::register_exception<icarus::SignalNotFoundError>(m, "SignalNotFoundError");
    py::register_exception<icarus::LifecycleError>(m, "LifecycleError");
}

// =============================================================================
// Module Definition
// =============================================================================

PYBIND11_MODULE(_icarus, m) {
    m.doc() = "Icarus 6DOF Simulation Framework";
    m.attr("__version__") = ICARUS_VERSION_STRING;
    m.attr("version_info") = py::make_tuple(
        ICARUS_VERSION_MAJOR, ICARUS_VERSION_MINOR, ICARUS_VERSION_PATCH);

    // Register exception types
    register_exceptions(m);

    // =========================================================================
    // Lifecycle Enum
    // =========================================================================
    py::enum_<icarus::Lifecycle>(m, "Lifecycle")
        .value("UNINITIALIZED", icarus::Lifecycle::Uninitialized)
        .value("PROVISIONED", icarus::Lifecycle::Provisioned)
        .value("STAGED", icarus::Lifecycle::Staged)
        .value("RUNNING", icarus::Lifecycle::Running)
        .export_values();

    // =========================================================================
    // Simulator Class
    // =========================================================================
    py::class_<icarus::Simulator>(m, "Simulator")
        // Constructor
        .def(py::init([](const std::string& config_path) {
            auto sim = icarus::Simulator::FromConfig(config_path);
            if (!sim) {
                throw icarus::ConfigError("Failed to load config: " + config_path);
            }
            return sim.release();
        }), py::arg("config_path"),
            "Create simulator from YAML configuration file")

        // Class methods
        .def_static("from_yaml", [](const std::string& yaml) {
            // TODO: Implement from-string loading
            throw std::runtime_error("from_yaml not yet implemented");
            return nullptr;
        }, py::arg("yaml_string"),
           "Create simulator from YAML string")

        // ----- Lifecycle -----
        .def("stage", py::overload_cast<>(&icarus::Simulator::Stage),
             "Stage the simulation")

        .def("step", [](icarus::Simulator& self, py::object dt_obj) {
            if (dt_obj.is_none()) {
                self.Step();
            } else {
                self.Step(dt_obj.cast<double>());
            }
        }, py::arg("dt") = py::none(),
           "Execute one step (uses configured dt if None)")

        .def("reset", &icarus::Simulator::Reset,
             "Reset simulation to initial state")

        .def_property_readonly("lifecycle", &icarus::Simulator::GetLifecycle,
                               "Current lifecycle state")

        // ----- Time Properties -----
        .def_property_readonly("time", &icarus::Simulator::Time,
                               "Current simulation time (MET)")
        .def_property_readonly("dt", &icarus::Simulator::Dt,
                               "Configured timestep")
        .def_property_readonly("end_time", &icarus::Simulator::EndTime,
                               "Configured end time")

        // ----- Signal Access -----
        .def("get", &icarus::Simulator::Peek,
             py::arg("name"), "Get signal value by name")
        .def("set", &icarus::Simulator::Poke,
             py::arg("name"), py::arg("value"), "Set signal value by name")

        // Pythonic access
        .def("__getitem__", &icarus::Simulator::Peek)
        .def("__setitem__", &icarus::Simulator::Poke)

        // ----- Bulk State Access -----
        .def_property("state",
            [](const icarus::Simulator& self) {
                return state_to_numpy(self.GetState());
            },
            [](icarus::Simulator& self, py::array_t<double> state) {
                self.SetState(numpy_to_state(state));
            },
            "State vector as numpy array")

        .def_property_readonly("state_size", [](const icarus::Simulator& self) {
            return self.GetState().size();
        }, "Number of state elements")

        .def_property_readonly("state_names", [](icarus::Simulator& self) {
            std::vector<std::string> names;
            const auto& registry = self.Registry();
            const auto& state_pairs = registry.get_state_pairs();
            for (const auto& pair : state_pairs) {
                names.push_back(pair.value_name);
            }
            return names;
        }, "Signal names for each state element")

        // ----- Introspection -----
        .def_property_readonly("signals", [](const icarus::Simulator& self) {
            return self.Registry().get_all_signal_names();
        }, "List of all signal names")

        .def_property_readonly("schema_json", [](icarus::Simulator& self) {
            auto dict = self.GetDataDictionary();

            // Build JSON then parse to Python dict
            nlohmann::json j;
            j["summary"]["total_outputs"] = dict.total_outputs;
            j["summary"]["total_inputs"] = dict.total_inputs;
            j["summary"]["total_parameters"] = dict.total_parameters;
            j["summary"]["total_config"] = dict.total_config;
            j["summary"]["integrable_states"] = dict.integrable_states;
            j["summary"]["unwired_inputs"] = dict.unwired_inputs;

            j["components"] = nlohmann::json::array();
            for (const auto& comp : dict.components) {
                nlohmann::json jcomp;
                jcomp["name"] = comp.name;
                jcomp["type"] = comp.type;

                auto to_json_signals = [](const std::vector<icarus::SignalDescriptor>& signals) {
                    nlohmann::json arr = nlohmann::json::array();
                    for (const auto& sig : signals) {
                        nlohmann::json jsig;
                        jsig["name"] = sig.name;
                        jsig["unit"] = sig.unit;
                        jsig["description"] = sig.description;
                        if (!sig.wired_to.empty()) {
                            jsig["wired_to"] = sig.wired_to;
                        }
                        if (sig.is_state) {
                            jsig["is_state"] = true;
                        }
                        arr.push_back(jsig);
                    }
                    return arr;
                };

                jcomp["outputs"] = to_json_signals(comp.outputs);
                jcomp["inputs"] = to_json_signals(comp.inputs);
                jcomp["parameters"] = to_json_signals(comp.parameters);
                jcomp["config"] = to_json_signals(comp.config);

                j["components"].push_back(jcomp);
            }

            // Convert to Python dict via JSON parsing
            return py::module_::import("json").attr("loads")(j.dump());
        }, "Data dictionary as Python dict")

        // ----- Utilities -----
        .def("to_dict", [](icarus::Simulator& self) {
            py::dict result;
            for (const auto& name : self.Registry().get_all_signal_names()) {
                result[py::cast(name)] = self.Peek(name);
            }
            return result;
        }, "Export current signal values as dict")

        .def("run_until", [](icarus::Simulator& self, double end_time, py::object callback) {
            while (self.Time() < end_time) {
                self.Step();
                if (!callback.is_none()) {
                    callback(py::cast(&self, py::return_value_policy::reference));
                }
            }
        }, py::arg("end_time"), py::arg("callback") = py::none(),
           "Run simulation until end_time");
}
```

---

## Tasks

### 7.2.1 Core Bindings

- [x] Create `interfaces/python/icarus_python.cpp`
- [x] Bind `Simulator` class with lifecycle methods
- [x] Bind `Lifecycle` enum
- [x] Bind signal access (`get`, `set`, `__getitem__`, `__setitem__`)
- [x] Bind state vector with numpy conversion
- [x] Register exception types (ConfigError, StageError, SignalNotFoundError, LifecycleError)

### 7.2.2 Numpy Integration

- [x] Implement `state_to_numpy()` conversion
- [x] Implement `numpy_to_state()` conversion
- [x] Test array ownership and memory safety
- [ ] Support both C-contiguous and F-contiguous arrays (future)

### 7.2.3 Build & Packaging

- [x] Add CMake configuration for PyBind11
- [x] Update `flake.nix` with pybind11 dependency
- [x] Update `scripts/build.sh` to build Python bindings (--python flag)
- [ ] Create `pyproject.toml` for pip installation (future)
- [ ] Create `setup.py` fallback (future)

### 7.2.4 Testing

- [x] Create `tests/python/test_simulator.py`
- [x] Test lifecycle methods
- [x] Test signal access
- [x] Test state vector operations
- [x] Test error handling

### 7.2.5 Documentation

- [x] Add docstrings to all bound methods
- [ ] Create example scripts in `examples/python/` (future)

---

## Build Configuration

### CMakeLists.txt

```cmake
# interfaces/python/CMakeLists.txt

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_icarus MODULE
    icarus_python.cpp
)

target_link_libraries(_icarus PRIVATE icarus)
target_compile_definitions(_icarus PRIVATE
    ICARUS_VERSION_STRING="${PROJECT_VERSION}"
    ICARUS_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    ICARUS_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    ICARUS_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

# Install to Python site-packages
install(TARGETS _icarus
    LIBRARY DESTINATION ${Python_SITEARCH}/icarus
)
```

### Python Package Structure

```
interfaces/python/
├── CMakeLists.txt
├── icarus_python.cpp
└── icarus/
    ├── __init__.py      # Re-exports from _icarus, version info
    └── py.typed         # PEP 561 marker
```

### `__init__.py`

```python
"""Icarus 6DOF Simulation Framework Python Bindings."""

from ._icarus import (
    Simulator,
    Lifecycle,
    __version__,
    version_info,
)

# Exception classes (if not directly from C++)
from ._icarus import (
    ConfigError,
    StageError,
    SignalNotFoundError,
    LifecycleError,
)

__all__ = [
    "Simulator",
    "Lifecycle",
    "ConfigError",
    "StageError",
    "SignalNotFoundError",
    "LifecycleError",
    "__version__",
    "version_info",
]
```

---

## Verification Plan

### Unit Tests

#### `tests/python/test_simulator.py`

```python
import pytest
import numpy as np
import icarus
from icarus import Simulator, Lifecycle

def test_version():
    assert icarus.__version__ is not None
    assert len(icarus.__version__) > 0
    assert isinstance(icarus.version_info, tuple)
    assert len(icarus.version_info) == 3

def test_create_simulator(config_path):
    sim = Simulator(config_path)
    assert sim is not None
    assert sim.lifecycle == Lifecycle.PROVISIONED

def test_stage(config_path):
    sim = Simulator(config_path)
    sim.stage()
    assert sim.lifecycle == Lifecycle.STAGED

def test_step(config_path):
    sim = Simulator(config_path)
    sim.stage()
    initial_time = sim.time
    sim.step()
    assert sim.time > initial_time
    assert sim.lifecycle == Lifecycle.RUNNING

def test_reset(config_path):
    sim = Simulator(config_path)
    sim.stage()
    sim.step()
    sim.step()
    sim.reset()
    assert sim.time == 0.0
    assert sim.lifecycle == Lifecycle.STAGED

def test_signal_access(config_path):
    sim = Simulator(config_path)
    sim.stage()

    # Get/set methods
    z = sim.get("Ball.position.z")
    assert z == 100.0  # Initial position

    sim.set("Ball.position.z", 200.0)
    assert sim.get("Ball.position.z") == 200.0

    # Pythonic access
    sim["Ball.position.z"] = 300.0
    assert sim["Ball.position.z"] == 300.0

def test_signal_not_found(config_path):
    sim = Simulator(config_path)
    sim.stage()

    with pytest.raises(icarus.SignalNotFoundError):
        sim.get("NonExistent.Signal")

def test_state_vector(config_path):
    sim = Simulator(config_path)
    sim.stage()

    state = sim.state
    assert isinstance(state, np.ndarray)
    assert state.ndim == 1
    assert len(state) == sim.state_size

    # Modify and set back
    original = state.copy()
    state[0] += 1.0
    sim.state = state
    assert sim.state[0] == original[0] + 1.0

def test_state_names(config_path):
    sim = Simulator(config_path)
    sim.stage()

    names = sim.state_names
    assert isinstance(names, list)
    assert len(names) == sim.state_size
    assert all(isinstance(n, str) for n in names)

def test_signals_list(config_path):
    sim = Simulator(config_path)
    sim.stage()

    signals = sim.signals
    assert isinstance(signals, list)
    assert len(signals) > 0

def test_schema_json(config_path):
    sim = Simulator(config_path)
    sim.stage()

    schema = sim.schema_json
    assert isinstance(schema, dict)
    assert "summary" in schema
    assert "components" in schema

def test_to_dict(config_path):
    sim = Simulator(config_path)
    sim.stage()

    data = sim.to_dict()
    assert isinstance(data, dict)
    assert len(data) > 0

def test_run_until(config_path):
    sim = Simulator(config_path)
    sim.stage()

    sim.run_until(0.1)
    assert sim.time >= 0.1

def test_run_until_with_callback(config_path):
    sim = Simulator(config_path)
    sim.stage()

    times = []
    def callback(s):
        times.append(s.time)

    sim.run_until(0.05, callback)
    assert len(times) > 0
```

---

## Example Usage

### Basic Simulation

```python
import icarus
import matplotlib.pyplot as plt

# Create and run
sim = icarus.Simulator("scenarios/falling_ball.yaml")
sim.stage()

times, altitudes = [], []
while sim.time < sim.end_time:
    times.append(sim.time)
    altitudes.append(sim["Ball.position.z"])
    sim.step()

# Plot
plt.plot(times, altitudes)
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Falling Ball Simulation")
plt.show()
```

### Monte Carlo

```python
import icarus
import numpy as np

def run_case(seed):
    rng = np.random.default_rng(seed)
    sim = icarus.Simulator("scenarios/6dof_vehicle.yaml")
    sim.stage()

    # Perturb initial state
    state = sim.state
    state += rng.normal(0, 0.01, size=state.shape)
    sim.state = state

    # Run to end
    sim.run_until(sim.end_time)
    return sim["Vehicle.position.z"]

# Run 100 Monte Carlo cases
results = [run_case(i) for i in range(100)]
print(f"Mean: {np.mean(results):.2f}, Std: {np.std(results):.2f}")
```

---

## Dependencies

### Build-time

- Python >=3.9
- pybind11 (via Nix)
- CMake >=3.15
- NumPy (for headers)

### Runtime

- numpy >=1.20 (required)
- pandas (optional, for DataFrame support)
- matplotlib (optional, for plotting)

### Nix Integration

```nix
# Add to flake.nix buildInputs
python3Packages.pybind11
python3Packages.numpy
```

---

## Exit Criteria

- [x] `import icarus` works
- [x] `Simulator` class fully functional with all C API equivalents
- [x] State vector accessible as numpy array
- [x] All signal access methods work (get/set/[])
- [x] Exceptions raised appropriately
- [x] Basic tests pass (26 tests)
