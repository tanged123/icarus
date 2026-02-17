/**
 * @file icarus_c.cpp
 * @brief Icarus C API implementation
 *
 * Wraps the C++ Simulator class in a C-compatible interface.
 */

#include "icarus.h"

#include <icarus/core/Error.hpp>
#include <icarus/io/data/DataDictionary.hpp>
#include <icarus/sim/Simulator.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Version info from CMake
#ifndef ICARUS_VERSION_STRING
#define ICARUS_VERSION_STRING "0.6.0"
#endif
#ifndef ICARUS_VERSION_MAJOR
#define ICARUS_VERSION_MAJOR 0
#endif
#ifndef ICARUS_VERSION_MINOR
#define ICARUS_VERSION_MINOR 6
#endif
#ifndef ICARUS_VERSION_PATCH
#define ICARUS_VERSION_PATCH 0
#endif

// =============================================================================
// Internal Handle Structure
// =============================================================================

struct IcarusHandle {
    std::unique_ptr<icarus::Simulator> sim;
    std::string last_error;

    // Caches for string returns (valid until next API call)
    std::string schema_json_cache;
    std::vector<std::string> signal_names_cache;
    std::vector<const char *> signal_names_ptrs_cache;
    std::vector<std::string> state_names_cache;
    std::vector<const char *> state_names_ptrs_cache;

    void SetError(const std::string &msg) { last_error = msg; }

    void ClearError() { last_error.clear(); }
};

// Thread-local storage for creation errors (before handle exists)
static thread_local std::string g_creation_error;

// =============================================================================
// Helper Functions
// =============================================================================

static IcarusError TranslateException(IcarusHandle *handle, const std::exception &e) {
    if (handle) {
        handle->SetError(e.what());
    }

    // Try to identify specific error types
    if (dynamic_cast<const icarus::ConfigError *>(&e)) {
        return ICARUS_ERROR_CONFIG_LOAD;
    }
    if (dynamic_cast<const icarus::StageError *>(&e)) {
        return ICARUS_ERROR_STAGE_FAILED;
    }
    if (dynamic_cast<const icarus::SignalNotFoundError *>(&e)) {
        return ICARUS_ERROR_SIGNAL_NOT_FOUND;
    }
    if (dynamic_cast<const icarus::LifecycleError *>(&e)) {
        return ICARUS_ERROR_INVALID_STATE;
    }

    return ICARUS_ERROR_UNKNOWN;
}

// =============================================================================
// Lifecycle Functions
// =============================================================================

extern "C" {

ICARUS_API IcarusHandle *icarus_create(const char *config_path) {
    if (!config_path) {
        g_creation_error = "config_path is NULL";
        return nullptr;
    }

    auto handle = std::make_unique<IcarusHandle>();

    try {
        handle->sim = icarus::Simulator::FromConfig(config_path);
        if (!handle->sim) {
            g_creation_error = "Failed to create simulator from config";
            return nullptr;
        }
        g_creation_error.clear();
        return handle.release();
    } catch (const std::exception &e) {
        g_creation_error = e.what();
        return nullptr;
    }
}

ICARUS_API void icarus_destroy(IcarusHandle *sim) {
    delete sim; // Safe if null
}

ICARUS_API IcarusError icarus_stage(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        sim->sim->Stage();
        return ICARUS_OK;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

ICARUS_API IcarusError icarus_step(IcarusHandle *sim, double dt) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        if (dt <= 0.0) {
            sim->sim->Step();
        } else {
            sim->sim->Step(dt);
        }
        return ICARUS_OK;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

ICARUS_API IcarusError icarus_reset(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        sim->sim->Reset();
        return ICARUS_OK;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

ICARUS_API IcarusLifecycle icarus_get_lifecycle(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return ICARUS_LIFECYCLE_UNINITIALIZED;
    }

    switch (sim->sim->GetLifecycle()) {
    case icarus::Lifecycle::Uninitialized:
        return ICARUS_LIFECYCLE_UNINITIALIZED;
    case icarus::Lifecycle::Provisioned:
        return ICARUS_LIFECYCLE_PROVISIONED;
    case icarus::Lifecycle::Staged:
        return ICARUS_LIFECYCLE_STAGED;
    case icarus::Lifecycle::Running:
        return ICARUS_LIFECYCLE_RUNNING;
    default:
        return ICARUS_LIFECYCLE_UNINITIALIZED;
    }
}

// =============================================================================
// Signal Access
// =============================================================================

ICARUS_API IcarusError icarus_get_signal(IcarusHandle *sim, const char *signal_name,
                                         double *out_value) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }
    if (!signal_name || !out_value) {
        if (sim) {
            sim->SetError("signal_name or out_value is NULL");
        }
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        *out_value = sim->sim->Peek(signal_name);
        return ICARUS_OK;
    } catch (const icarus::SignalNotFoundError &e) {
        sim->SetError(std::string("Signal not found: ") + signal_name);
        return ICARUS_ERROR_SIGNAL_NOT_FOUND;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

ICARUS_API IcarusError icarus_set_signal(IcarusHandle *sim, const char *signal_name, double value) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }
    if (!signal_name) {
        sim->SetError("signal_name is NULL");
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        sim->sim->Poke(signal_name, value);
        return ICARUS_OK;
    } catch (const icarus::SignalNotFoundError &e) {
        sim->SetError(std::string("Signal not found: ") + signal_name);
        return ICARUS_ERROR_SIGNAL_NOT_FOUND;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

// =============================================================================
// Bulk State Access
// =============================================================================

ICARUS_API IcarusError icarus_get_state_vector(IcarusHandle *sim, double *out_buffer,
                                               size_t buffer_size, size_t *out_size) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }
    if (!out_buffer || !out_size) {
        sim->SetError("out_buffer or out_size is NULL");
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        Eigen::VectorXd state = sim->sim->GetState();
        size_t state_size = static_cast<size_t>(state.size());

        *out_size = state_size;

        if (buffer_size < state_size) {
            sim->SetError("Buffer too small: need " + std::to_string(state_size) + ", got " +
                          std::to_string(buffer_size));
            return ICARUS_ERROR_BUFFER_TOO_SMALL;
        }

        std::memcpy(out_buffer, state.data(), state_size * sizeof(double));
        return ICARUS_OK;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

ICARUS_API IcarusError icarus_set_state_vector(IcarusHandle *sim, const double *buffer,
                                               size_t size) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }
    if (!buffer) {
        sim->SetError("buffer is NULL");
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        Eigen::VectorXd state(static_cast<Eigen::Index>(size));
        std::memcpy(state.data(), buffer, size * sizeof(double));
        sim->sim->SetState(state);
        return ICARUS_OK;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

ICARUS_API size_t icarus_get_state_size(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return 0;
    }

    try {
        return static_cast<size_t>(sim->sim->GetState().size());
    } catch (...) {
        return 0;
    }
}

ICARUS_API IcarusError icarus_get_state_signal_names(IcarusHandle *sim, const char **out_names,
                                                     size_t max_count, size_t *out_count) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }
    if (!out_names || !out_count) {
        sim->SetError("out_names or out_count is NULL");
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        // Get state pairs from registry
        const auto &registry = sim->sim->Registry();
        const auto &state_pairs = registry.get_state_pairs();

        // Cache the names
        sim->state_names_cache.clear();
        sim->state_names_ptrs_cache.clear();

        for (const auto &pair : state_pairs) {
            sim->state_names_cache.push_back(pair.value_name);
        }

        // Build pointer array
        for (const auto &name : sim->state_names_cache) {
            sim->state_names_ptrs_cache.push_back(name.c_str());
        }

        // Copy to output
        size_t count = std::min(max_count, sim->state_names_ptrs_cache.size());
        for (size_t i = 0; i < count; ++i) {
            out_names[i] = sim->state_names_ptrs_cache[i];
        }
        *out_count = count;

        return ICARUS_OK;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

// =============================================================================
// Time Access
// =============================================================================

ICARUS_API double icarus_get_time(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return std::nan("");
    }
    return sim->sim->Time();
}

ICARUS_API double icarus_get_dt(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return std::nan("");
    }
    return sim->sim->Dt();
}

ICARUS_API double icarus_get_end_time(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return std::nan("");
    }
    return sim->sim->EndTime();
}

// =============================================================================
// Introspection
// =============================================================================

ICARUS_API const char *icarus_get_schema_json(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return nullptr;
    }

    sim->ClearError();

    try {
        auto dict = sim->sim->GetDataDictionary();

        // Build JSON manually (DataDictionary::ToJSON writes to file)
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

        sim->schema_json_cache = j.dump(2); // Pretty print with indent=2

        // Allocate a copy that the caller will free
        char *result = static_cast<char *>(std::malloc(sim->schema_json_cache.size() + 1));
        if (!result) {
            sim->SetError("Failed to allocate memory for JSON");
            return nullptr;
        }
        std::strcpy(result, sim->schema_json_cache.c_str());
        return result;
    } catch (const std::exception &e) {
        sim->SetError(e.what());
        return nullptr;
    }
}

ICARUS_API const char *icarus_get_introspection_graph_json(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return nullptr;
    }

    sim->ClearError();

    try {
        auto graph = sim->sim->GetIntrospectionGraph();
        auto j = graph.ToJSON();

        std::string json_str = j.dump(2);

        char *result = static_cast<char *>(std::malloc(json_str.size() + 1));
        if (!result) {
            sim->SetError("Failed to allocate memory for JSON");
            return nullptr;
        }
        std::strcpy(result, json_str.c_str());
        return result;
    } catch (const std::exception &e) {
        sim->SetError(e.what());
        return nullptr;
    }
}

ICARUS_API size_t icarus_get_signal_count(IcarusHandle *sim) {
    if (!sim || !sim->sim) {
        return 0;
    }

    try {
        return sim->sim->Registry().get_all_signal_names().size();
    } catch (...) {
        return 0;
    }
}

ICARUS_API IcarusError icarus_get_signal_names(IcarusHandle *sim, const char **out_names,
                                               size_t max_count, size_t *out_count) {
    if (!sim || !sim->sim) {
        return ICARUS_ERROR_NULL_HANDLE;
    }
    if (!out_names || !out_count) {
        sim->SetError("out_names or out_count is NULL");
        return ICARUS_ERROR_NULL_HANDLE;
    }

    sim->ClearError();

    try {
        // Cache the names
        sim->signal_names_cache = sim->sim->Registry().get_all_signal_names();
        sim->signal_names_ptrs_cache.clear();

        for (const auto &name : sim->signal_names_cache) {
            sim->signal_names_ptrs_cache.push_back(name.c_str());
        }

        // Copy to output
        size_t count = std::min(max_count, sim->signal_names_ptrs_cache.size());
        for (size_t i = 0; i < count; ++i) {
            out_names[i] = sim->signal_names_ptrs_cache[i];
        }
        *out_count = count;

        return ICARUS_OK;
    } catch (const std::exception &e) {
        return TranslateException(sim, e);
    }
}

// =============================================================================
// Error Handling
// =============================================================================

ICARUS_API const char *icarus_get_last_error(IcarusHandle *sim) {
    if (!sim) {
        return g_creation_error.c_str();
    }
    return sim->last_error.c_str();
}

ICARUS_API const char *icarus_error_name(IcarusError error) {
    switch (error) {
    case ICARUS_OK:
        return "ICARUS_OK";
    case ICARUS_ERROR_NULL_HANDLE:
        return "ICARUS_ERROR_NULL_HANDLE";
    case ICARUS_ERROR_CONFIG_LOAD:
        return "ICARUS_ERROR_CONFIG_LOAD";
    case ICARUS_ERROR_STAGE_FAILED:
        return "ICARUS_ERROR_STAGE_FAILED";
    case ICARUS_ERROR_STEP_FAILED:
        return "ICARUS_ERROR_STEP_FAILED";
    case ICARUS_ERROR_SIGNAL_NOT_FOUND:
        return "ICARUS_ERROR_SIGNAL_NOT_FOUND";
    case ICARUS_ERROR_TYPE_MISMATCH:
        return "ICARUS_ERROR_TYPE_MISMATCH";
    case ICARUS_ERROR_INVALID_STATE:
        return "ICARUS_ERROR_INVALID_STATE";
    case ICARUS_ERROR_BUFFER_TOO_SMALL:
        return "ICARUS_ERROR_BUFFER_TOO_SMALL";
    case ICARUS_ERROR_ALLOCATION:
        return "ICARUS_ERROR_ALLOCATION";
    case ICARUS_ERROR_UNKNOWN:
    default:
        return "ICARUS_ERROR_UNKNOWN";
    }
}

// =============================================================================
// Memory Management
// =============================================================================

ICARUS_API void icarus_free_string(const char *str) {
    std::free(const_cast<char *>(str)); // Safe if null
}

// =============================================================================
// Version
// =============================================================================

ICARUS_API const char *icarus_version(void) { return ICARUS_VERSION_STRING; }

ICARUS_API void icarus_version_components(int *major, int *minor, int *patch) {
    if (major) {
        *major = ICARUS_VERSION_MAJOR;
    }
    if (minor) {
        *minor = ICARUS_VERSION_MINOR;
    }
    if (patch) {
        *patch = ICARUS_VERSION_PATCH;
    }
}

} // extern "C"
