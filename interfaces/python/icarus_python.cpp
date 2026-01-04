/**
 * @file icarus_python.cpp
 * @brief Icarus Python bindings via PyBind11
 *
 * Part of Phase 7.2: Python Bindings Implementation
 */

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <icarus/core/Error.hpp>
#include <icarus/io/data/DataDictionary.hpp>
#include <icarus/sim/Simulator.hpp>
#include <nlohmann/json.hpp>

#include <cstring>

namespace py = pybind11;

// Version info from CMake
#ifndef ICARUS_VERSION_STRING
#define ICARUS_VERSION_STRING "0.5.1"
#endif
#ifndef ICARUS_VERSION_MAJOR
#define ICARUS_VERSION_MAJOR 0
#endif
#ifndef ICARUS_VERSION_MINOR
#define ICARUS_VERSION_MINOR 5
#endif
#ifndef ICARUS_VERSION_PATCH
#define ICARUS_VERSION_PATCH 1
#endif

// =============================================================================
// Numpy Array Conversion
// =============================================================================

/**
 * @brief Convert Eigen vector to numpy array
 */
py::array_t<double> state_to_numpy(const Eigen::VectorXd &state) {
    auto result = py::array_t<double>(state.size());
    auto buf = result.request();
    std::memcpy(buf.ptr, state.data(), static_cast<size_t>(state.size()) * sizeof(double));
    return result;
}

/**
 * @brief Convert numpy array to Eigen vector
 */
Eigen::VectorXd numpy_to_state(py::array_t<double> arr) {
    auto buf = arr.request();
    if (buf.ndim != 1) {
        throw std::runtime_error("State must be 1D array");
    }
    Eigen::VectorXd result(buf.shape[0]);
    std::memcpy(result.data(), buf.ptr, static_cast<size_t>(buf.shape[0]) * sizeof(double));
    return result;
}

// =============================================================================
// Exception Classes
// =============================================================================

// Python exception types (static to avoid multiple registrations)
static PyObject *ConfigErrorType = nullptr;
static PyObject *StageErrorType = nullptr;
static PyObject *SignalNotFoundErrorType = nullptr;
static PyObject *LifecycleErrorType = nullptr;

/**
 * @brief Register exception types for Python
 */
void register_exceptions(py::module_ &m) {
    // Create IcarusError base class
    auto icarus_error = py::register_exception<icarus::Error>(m, "IcarusError");

    // Create specific exception types
    ConfigErrorType = PyErr_NewException("icarus.ConfigError", icarus_error.ptr(), nullptr);
    StageErrorType = PyErr_NewException("icarus.StageError", icarus_error.ptr(), nullptr);
    SignalNotFoundErrorType =
        PyErr_NewException("icarus.SignalNotFoundError", icarus_error.ptr(), nullptr);
    LifecycleErrorType = PyErr_NewException("icarus.LifecycleError", icarus_error.ptr(), nullptr);

    m.attr("ConfigError") = py::handle(ConfigErrorType);
    m.attr("StageError") = py::handle(StageErrorType);
    m.attr("SignalNotFoundError") = py::handle(SignalNotFoundErrorType);
    m.attr("LifecycleError") = py::handle(LifecycleErrorType);

    // Register exception translators
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p)
                std::rethrow_exception(p);
        } catch (const icarus::ConfigError &e) {
            PyErr_SetString(ConfigErrorType, e.what());
        } catch (const icarus::StageError &e) {
            PyErr_SetString(StageErrorType, e.what());
        } catch (const icarus::SignalNotFoundError &e) {
            PyErr_SetString(SignalNotFoundErrorType, e.what());
        } catch (const icarus::LifecycleError &e) {
            PyErr_SetString(LifecycleErrorType, e.what());
        }
        // Other icarus::Error subclasses will be caught by the base handler
    });
}

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Build schema JSON dict from DataDictionary
 */
py::dict build_schema_dict(const icarus::DataDictionary &dict) {
    nlohmann::json j;

    // Summary
    j["summary"]["total_outputs"] = dict.total_outputs;
    j["summary"]["total_inputs"] = dict.total_inputs;
    j["summary"]["total_parameters"] = dict.total_parameters;
    j["summary"]["total_config"] = dict.total_config;
    j["summary"]["integrable_states"] = dict.integrable_states;
    j["summary"]["unwired_inputs"] = dict.unwired_inputs;

    // Components
    j["components"] = nlohmann::json::array();
    for (const auto &comp : dict.components) {
        nlohmann::json jcomp;
        jcomp["name"] = comp.name;
        jcomp["type"] = comp.type;

        auto to_json_signals = [](const std::vector<icarus::SignalDescriptor> &signals) {
            nlohmann::json arr = nlohmann::json::array();
            for (const auto &sig : signals) {
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
    return py::module_::import("json").attr("loads")(j.dump()).cast<py::dict>();
}

// =============================================================================
// Module Definition
// =============================================================================

PYBIND11_MODULE(_icarus, m) {
    m.doc() = "Icarus 6DOF Simulation Framework Python Bindings";
    m.attr("__version__") = ICARUS_VERSION_STRING;
    m.attr("version_info") =
        py::make_tuple(ICARUS_VERSION_MAJOR, ICARUS_VERSION_MINOR, ICARUS_VERSION_PATCH);

    // Register exception types
    register_exceptions(m);

    // =========================================================================
    // Lifecycle Enum
    // =========================================================================
    py::enum_<icarus::Lifecycle>(m, "Lifecycle", "Simulation lifecycle state")
        .value("UNINITIALIZED", icarus::Lifecycle::Uninitialized, "Not yet configured")
        .value("PROVISIONED", icarus::Lifecycle::Provisioned, "Components added, ready for Stage")
        .value("STAGED", icarus::Lifecycle::Staged, "Staged and ready to run")
        .value("RUNNING", icarus::Lifecycle::Running, "Simulation in progress")
        .export_values();

    // =========================================================================
    // Simulator Class
    // =========================================================================
    py::class_<icarus::Simulator>(m, "Simulator", R"doc(
Icarus 6DOF Simulator Python Interface.

This class wraps the C++ Simulator via PyBind11, providing
a Pythonic interface with numpy integration.

Example::

    import icarus

    sim = icarus.Simulator("config.yaml")
    sim.stage()

    while sim.time < sim.end_time:
        print(f"t={sim.time:.2f}, alt={sim['Ball.position.z']:.1f}")
        sim.step()
)doc")
        // -----------------------------------------------------------------
        // Constructor
        // -----------------------------------------------------------------
        .def(py::init([](const std::string &config_path) {
                 auto sim = icarus::Simulator::FromConfig(config_path);
                 if (!sim) {
                     throw icarus::ConfigError("Failed to load config: " + config_path);
                 }
                 return sim.release();
             }),
             py::arg("config_path"), "Create simulator from YAML configuration file")

        // Class methods
        .def_static(
            "from_yaml",
            [](const std::string &yaml) {
                // TODO: Implement from-string loading
                throw std::runtime_error("from_yaml not yet implemented");
                return nullptr;
            },
            py::arg("yaml_string"), "Create simulator from YAML string (not yet implemented)")

        // -----------------------------------------------------------------
        // Lifecycle Methods
        // -----------------------------------------------------------------
        .def("stage", py::overload_cast<>(&icarus::Simulator::Stage),
             "Stage the simulation (validate, wire, apply ICs)")

        .def(
            "step",
            [](icarus::Simulator &self, py::object dt_obj) {
                if (dt_obj.is_none()) {
                    self.Step();
                } else {
                    self.Step(dt_obj.cast<double>());
                }
            },
            py::arg("dt") = py::none(), "Execute one step (uses configured dt if None)")

        .def("reset", &icarus::Simulator::Reset, "Reset simulation to initial state")

        .def_property_readonly("lifecycle", &icarus::Simulator::GetLifecycle,
                               "Current lifecycle state")

        // -----------------------------------------------------------------
        // Time Properties
        // -----------------------------------------------------------------
        .def_property_readonly("time", &icarus::Simulator::Time,
                               "Current simulation time (MET) in seconds")
        .def_property_readonly("dt", &icarus::Simulator::Dt, "Configured timestep in seconds")
        .def_property_readonly("end_time", &icarus::Simulator::EndTime,
                               "Configured end time in seconds")
        .def_property_readonly("name", &icarus::Simulator::Name, "Simulation name from config")

        // -----------------------------------------------------------------
        // Signal Access
        // -----------------------------------------------------------------
        .def("get", &icarus::Simulator::Peek, py::arg("name"), "Get signal value by name")
        .def("set", &icarus::Simulator::Poke, py::arg("name"), py::arg("value"),
             "Set signal value by name")

        // Pythonic access via __getitem__/__setitem__
        .def("__getitem__", &icarus::Simulator::Peek, "Get signal value using sim['name'] syntax")
        .def("__setitem__", &icarus::Simulator::Poke,
             "Set signal value using sim['name'] = value syntax")

        // -----------------------------------------------------------------
        // Bulk State Access
        // -----------------------------------------------------------------
        .def_property(
            "state", [](const icarus::Simulator &self) { return state_to_numpy(self.GetState()); },
            [](icarus::Simulator &self, py::array_t<double> state) {
                self.SetState(numpy_to_state(state));
            },
            R"doc(
State vector as numpy array.

States ARE signals in Icarus. This bulk API exists for
performance-critical use cases like:
- External integrators needing full state vector
- Monte Carlo IC perturbation
- Checkpointing / warmstart
- High-frequency logging

For casual access, prefer sim.get(signal_name) or sim[signal_name].
)doc")

        .def_property_readonly(
            "state_size",
            [](const icarus::Simulator &self) {
                return static_cast<size_t>(self.GetState().size());
            },
            "Number of state elements")

        .def_property_readonly(
            "state_names",
            [](icarus::Simulator &self) {
                std::vector<std::string> names;
                const auto &registry = self.Registry();
                const auto &state_pairs = registry.get_state_pairs();
                for (const auto &pair : state_pairs) {
                    names.push_back(pair.value_name);
                }
                return names;
            },
            "Signal names for each state element (in state vector order)")

        // -----------------------------------------------------------------
        // Introspection
        // -----------------------------------------------------------------
        .def_property_readonly(
            "signals",
            [](const icarus::Simulator &self) { return self.Registry().get_all_signal_names(); },
            "List of all signal names")

        .def_property_readonly(
            "signal_count",
            [](const icarus::Simulator &self) {
                return self.Registry().get_all_signal_names().size();
            },
            "Total number of signals")

        .def_property_readonly(
            "schema_json",
            [](icarus::Simulator &self) {
                auto dict = self.GetDataDictionary();
                return build_schema_dict(dict);
            },
            "Data dictionary as Python dict")

        // -----------------------------------------------------------------
        // Utilities
        // -----------------------------------------------------------------
        .def(
            "to_dict",
            [](icarus::Simulator &self) {
                py::dict result;
                for (const auto &name : self.Registry().get_all_signal_names()) {
                    result[py::cast(name)] = self.Peek(name);
                }
                return result;
            },
            "Export current signal values as dict")

        .def(
            "run_until",
            [](icarus::Simulator &self, double end_time, py::object callback) {
                while (self.Time() < end_time) {
                    self.Step();
                    if (!callback.is_none()) {
                        callback(py::cast(&self, py::return_value_policy::reference));
                    }
                }
            },
            py::arg("end_time"), py::arg("callback") = py::none(),
            R"doc(
Run simulation until end_time.

Args:
    end_time: Stop time in seconds.
    callback: Optional function called after each step with simulator as argument.

Example::

    times = []
    def record(sim):
        times.append(sim.time)
    sim.run_until(10.0, record)
)doc")

        // -----------------------------------------------------------------
        // Expert Interface
        // -----------------------------------------------------------------
        .def(
            "compute_derivatives",
            [](icarus::Simulator &self, double t) {
                return state_to_numpy(self.ComputeDerivatives(t));
            },
            py::arg("t"), "Compute state derivatives at time t (expert)")

        .def_property_readonly("flight_phase", &icarus::Simulator::GetFlightPhase,
                               "Current flight phase value")
        .def_property_readonly("flight_phase_name", &icarus::Simulator::GetFlightPhaseName,
                               "Current flight phase name");
}
