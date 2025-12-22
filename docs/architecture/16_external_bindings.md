# External Bindings & Language Interface

**Related:** [13_configuration.md](13_configuration.md) | [03_signal_backplane.md](03_signal_backplane.md)

---

Icarus must be usable by diverse consumers:
- **Analysis Engineers:** Python (Jupyter, pandas, matplotlib)
- **GNC Engineers:** MATLAB / Simulink
- **Firmware Engineers:** C / Embedded C
- **Orchestration:** C++ applications, Hermes middleware

---

## 1. The C API Strategy

> [!IMPORTANT]
> **Design Decision:** The top-level Simulator interface is exposed via a **C API** for maximum compatibility.
>
> C is the universal FFI (Foreign Function Interface). Every language can call C.

---

## 2. API Surface

```c
// icarus.h - The universal C interface

// Lifecycle
IcarusHandle* icarus_create(const char* config_path);
void icarus_destroy(IcarusHandle* sim);

int icarus_provision(IcarusHandle* sim);
int icarus_stage(IcarusHandle* sim, const char* run_config_path);
int icarus_step(IcarusHandle* sim, double dt);

// State Access
double icarus_get_signal(IcarusHandle* sim, const char* signal_name);
int icarus_set_signal(IcarusHandle* sim, const char* signal_name, double value);

// Bulk Access (for performance)
int icarus_get_state_vector(IcarusHandle* sim, double* out_buffer, int* out_size);
int icarus_set_state_vector(IcarusHandle* sim, const double* buffer, int size);

// Introspection
const char* icarus_get_schema_json(IcarusHandle* sim);
int icarus_get_signal_count(IcarusHandle* sim);

// Error Handling
const char* icarus_get_last_error(IcarusHandle* sim);
```

---

## 3. Language Bindings

| Language | Binding Method | Notes |
|----------|---------------|-------|
| **Python** | `ctypes` or `cffi` wrapping C API, or PyBind11 for richer interface | Primary for analysis |
| **MATLAB** | `loadlibrary` / MEX calling C API | S-Function wrapper for Simulink |
| **C++** | Direct include of C++ headers, or C API | Hermes uses native C++ |
| **Julia** | `ccall` to C API | Growing in aerospace |
| **Rust** | `bindgen` for C API | Safety-critical applications |

---

## 4. Simulink Integration Pattern

For MATLAB/Simulink users, wrap the C API in an S-Function:

```matlab
% In Simulink S-Function
function [sys] = mdlOutputs(t, x, u, sim_handle)
    % Set inputs
    icarus_set_signal(sim_handle, 'Control.Throttle', u(1));
    icarus_set_signal(sim_handle, 'Control.Elevator', u(2));

    % Step
    icarus_step(sim_handle, dt);

    % Get outputs
    sys(1) = icarus_get_signal(sim_handle, 'Nav.Altitude');
    sys(2) = icarus_get_signal(sim_handle, 'Nav.Velocity');
end
```

---

## 5. Python Integration (PyBind11)

PyBind11 provides a richer, more Pythonic interface than raw ctypes:

```cpp
// icarus_python.cpp - PyBind11 bindings
#include <pybind11/pybind11.h>
#include "icarus/Simulator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(icarus, m) {
    py::class_<Simulator>(m, "Simulator")
        .def(py::init<const std::string&>())
        .def("provision", &Simulator::Provision)
        .def("stage", &Simulator::Stage)
        .def("step", &Simulator::Step)
        .def("get_signal", &Simulator::GetSignal)
        .def("set_signal", &Simulator::SetSignal)
        .def("get_schema", &Simulator::GetSchemaJson)
        .def_property_readonly("time", &Simulator::GetTime);
}
```

```python
# Python usage
import icarus

sim = icarus.Simulator("scenarios/x15_mission.yaml")
sim.provision()
sim.stage("trim/steady_cruise.yaml")

for _ in range(10000):
    sim.step(0.001)
    alt = sim.get_signal("Nav.Altitude")
    print(f"t={sim.time:.3f}s, Altitude: {alt:.1f}m")
```

---

## 6. Performance Considerations

| Interface | Overhead | Use Case |
|-----------|----------|----------|
| **C API (per-signal)** | ~100ns per call | Interactive, low-frequency |
| **C API (bulk vector)** | ~1μs for full state | Monte Carlo, batch |
| **Native C++** | Zero | Hermes, performance-critical |

> [!NOTE]
> For high-performance Monte Carlo (Hydra), use native C++ instantiation, not the C API. The C API is for external consumers, not internal batch execution.

---

## 7. ZMQ Remote Interface

For network-accessible simulation control:

```yaml
services:
  bindings:
    zmq:
      enabled: true
      port: 5555
      protocol: REQ-REP
```

```python
# Remote client
import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

socket.send_json({"cmd": "step", "dt": 0.001})
response = socket.recv_json()
print(response["signals"]["Nav.Altitude"])
```
