# C API Implementation

**Status:** Proposed
**Phase:** 7.1
**Related:** [16_external_bindings.md](../../architecture/16_external_bindings.md)

---

## Overview

Implement a **C API** for Icarus to enable FFI (Foreign Function Interface) bindings from any language. C is the universal FFI target - Python, MATLAB, Julia, Rust, and embedded systems can all call C functions.

### Design Principles

1. **Opaque Handle:** Users interact with `IcarusHandle*`, never see C++ internals
2. **Error Propagation:** All functions return error codes; last error retrievable via `icarus_get_last_error()`
3. **Memory Safety:** Clear ownership semantics; user never manages internal memory
4. **Performance:** Bulk access APIs for Monte Carlo and batch operations

### Design Notes: States vs Signals

> **Key Insight:** In Icarus (Phase 6+), states ARE signals. Every integrable state is a signal with an associated derivative that gets integrated each step.

This means there are two ways to access state values:

| Method | Use Case | Performance |
|--------|----------|-------------|
| `icarus_get_signal("Vehicle.Body.pos.x", &val)` | Casual access, specific values | ~100ns/call |
| `icarus_get_state_vector(sim, buf, n, &n)` | Bulk access, checkpointing, Monte Carlo | ~1μs total |

The bulk state API exists for performance-critical use cases:
- **External integrators** that need the full state vector
- **Monte Carlo** IC perturbation (modify many states at once)
- **Checkpointing / warmstart** (save/restore full state)
- **High-frequency logging** (avoid N string lookups per frame)

Use `icarus_get_state_signal_names()` to discover what each state vector element represents.

---

## API Surface

### Core Lifecycle

```c
// icarus.h - The universal C interface

#ifndef ICARUS_H
#define ICARUS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to Simulator
typedef struct IcarusHandle IcarusHandle;

// Error codes
typedef enum {
    ICARUS_OK = 0,
    ICARUS_ERROR_NULL_HANDLE = -1,
    ICARUS_ERROR_CONFIG_LOAD = -2,
    ICARUS_ERROR_STAGE_FAILED = -3,
    ICARUS_ERROR_STEP_FAILED = -4,
    ICARUS_ERROR_SIGNAL_NOT_FOUND = -5,
    ICARUS_ERROR_TYPE_MISMATCH = -6,
    ICARUS_ERROR_INVALID_STATE = -7,
    ICARUS_ERROR_ALLOCATION = -8,
    ICARUS_ERROR_UNKNOWN = -99
} IcarusError;

// =============================================================================
// Lifecycle
// =============================================================================

/**
 * @brief Create simulator from YAML configuration file
 * @param config_path Path to simulation YAML
 * @return Handle to simulator, or NULL on failure
 */
IcarusHandle* icarus_create(const char* config_path);

/**
 * @brief Create simulator from YAML string
 * @param yaml_content YAML configuration as string
 * @return Handle to simulator, or NULL on failure
 */
IcarusHandle* icarus_create_from_string(const char* yaml_content);

/**
 * @brief Destroy simulator and free all resources
 * @param sim Handle to destroy (safe to pass NULL)
 */
void icarus_destroy(IcarusHandle* sim);

/**
 * @brief Stage the simulation (validate, wire, optional trim)
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_stage(IcarusHandle* sim);

/**
 * @brief Stage with override configuration
 * @param stage_config_path Path to stage config YAML (trim, linearization, etc.)
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_stage_with_config(IcarusHandle* sim, const char* stage_config_path);

/**
 * @brief Execute one simulation step
 * @param dt Timestep in seconds
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_step(IcarusHandle* sim, double dt);

/**
 * @brief Reset simulation to initial state
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_reset(IcarusHandle* sim);

// =============================================================================
// Signal Access (Per-Signal)
// =============================================================================

/**
 * @brief Get scalar signal value by name
 * @param sim Simulator handle
 * @param signal_name Full signal path (e.g., "Vehicle.Nav.altitude")
 * @param out_value Output: signal value
 * @return ICARUS_OK on success, ICARUS_ERROR_SIGNAL_NOT_FOUND if not found
 */
IcarusError icarus_get_signal(IcarusHandle* sim, const char* signal_name, double* out_value);

/**
 * @brief Set scalar signal value by name
 * @param sim Simulator handle
 * @param signal_name Full signal path
 * @param value Value to set
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_set_signal(IcarusHandle* sim, const char* signal_name, double value);

/**
 * @brief Get vector signal values
 * @param sim Simulator handle
 * @param signal_name Signal name (must be vector type)
 * @param out_buffer Output buffer (caller allocated)
 * @param buffer_size Size of output buffer
 * @param out_written Number of elements written
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_get_signal_vector(IcarusHandle* sim, const char* signal_name,
                                      double* out_buffer, size_t buffer_size,
                                      size_t* out_written);

// =============================================================================
// Bulk State Access (High Performance)
// =============================================================================
//
// NOTE: States ARE signals in Icarus (Phase 6 unified signal model).
// Every integrable state is accessible both via icarus_get_signal() by name
// AND via this bulk state vector API.
//
// The bulk API exists for performance-critical use cases:
//   - External integrators needing the full state vector
//   - Monte Carlo IC perturbation (modify many states at once)
//   - Checkpointing / warmstart (save/restore full state)
//   - High-frequency logging (avoid N string lookups per frame)
//
// For casual access, prefer icarus_get_signal() with the state's signal name.

/**
 * @brief Get full state vector
 * @param sim Simulator handle
 * @param out_buffer Output buffer (caller allocated)
 * @param buffer_size Size of output buffer
 * @param out_size Actual state vector size
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_get_state_vector(IcarusHandle* sim, double* out_buffer,
                                     size_t buffer_size, size_t* out_size);

/**
 * @brief Set full state vector
 * @param sim Simulator handle
 * @param buffer State values
 * @param size Number of elements
 * @return ICARUS_OK on success, error code on failure
 */
IcarusError icarus_set_state_vector(IcarusHandle* sim, const double* buffer, size_t size);

/**
 * @brief Get state vector size
 * @param sim Simulator handle
 * @return Number of state elements, or 0 on error
 */
size_t icarus_get_state_size(IcarusHandle* sim);

/**
 * @brief Get state signal names (in state vector order)
 *
 * Returns the signal names corresponding to each element of the state vector.
 * Use this to understand what icarus_get_state_vector() returns.
 *
 * @param sim Simulator handle
 * @param out_names Array of string pointers (caller allocated)
 * @param max_count Maximum names to return
 * @param out_count Actual count written
 * @return ICARUS_OK on success
 *
 * Example:
 *   size_t n = icarus_get_state_size(sim);
 *   const char** names = malloc(n * sizeof(char*));
 *   icarus_get_state_signal_names(sim, names, n, &n);
 *   // names[0] might be "Vehicle.Body.position.x"
 *   // names[1] might be "Vehicle.Body.position.y"
 *   // etc.
 */
IcarusError icarus_get_state_signal_names(IcarusHandle* sim, const char** out_names,
                                           size_t max_count, size_t* out_count);

// =============================================================================
// Time Access
// =============================================================================

/**
 * @brief Get current simulation time (MET)
 * @param sim Simulator handle
 * @return Mission Elapsed Time in seconds, or NaN on error
 */
double icarus_get_time(IcarusHandle* sim);

/**
 * @brief Get configured timestep
 * @param sim Simulator handle
 * @return Timestep in seconds
 */
double icarus_get_dt(IcarusHandle* sim);

/**
 * @brief Get configured end time
 * @param sim Simulator handle
 * @return End time in seconds
 */
double icarus_get_end_time(IcarusHandle* sim);

// =============================================================================
// Introspection
// =============================================================================

/**
 * @brief Get data dictionary as JSON string
 * @param sim Simulator handle
 * @return JSON string (caller must free with icarus_free_string)
 */
const char* icarus_get_schema_json(IcarusHandle* sim);

/**
 * @brief Get number of signals
 * @param sim Simulator handle
 * @return Signal count
 */
size_t icarus_get_signal_count(IcarusHandle* sim);

/**
 * @brief Get signal names
 * @param sim Simulator handle
 * @param out_names Array of string pointers (caller allocated)
 * @param max_count Maximum number of names to return
 * @param out_count Actual number of names returned
 * @return ICARUS_OK on success
 */
IcarusError icarus_get_signal_names(IcarusHandle* sim, const char** out_names,
                                     size_t max_count, size_t* out_count);

// =============================================================================
// Error Handling
// =============================================================================

/**
 * @brief Get last error message
 * @param sim Simulator handle (can be NULL for creation errors)
 * @return Error message string (valid until next API call)
 */
const char* icarus_get_last_error(IcarusHandle* sim);

/**
 * @brief Get error code name as string
 * @param error Error code
 * @return Human-readable error name
 */
const char* icarus_error_name(IcarusError error);

// =============================================================================
// Memory Management
// =============================================================================

/**
 * @brief Free string allocated by Icarus
 * @param str String to free
 */
void icarus_free_string(const char* str);

// =============================================================================
// Version
// =============================================================================

/**
 * @brief Get Icarus version string
 * @return Version string (e.g., "1.0.0")
 */
const char* icarus_version(void);

#ifdef __cplusplus
}
#endif

#endif // ICARUS_H
```

---

## Implementation

### Internal Handle Structure

```cpp
// src/interfaces/icarus_c_api.cpp

#include "icarus/icarus.h"
#include <icarus/sim/Simulator.hpp>
#include <memory>
#include <string>

struct IcarusHandle {
    std::unique_ptr<icarus::Simulator> sim;
    std::string last_error;
    std::string schema_cache;  // Cached JSON schema
    std::vector<std::string> signal_names_cache;

    void SetError(const std::string& msg) {
        last_error = msg;
    }
};
```

### Lifecycle Implementation

```cpp
extern "C" {

IcarusHandle* icarus_create(const char* config_path) {
    auto handle = new(std::nothrow) IcarusHandle();
    if (!handle) return nullptr;

    try {
        handle->sim = icarus::Simulator::FromConfig(config_path);
        if (!handle->sim) {
            handle->SetError("Failed to create simulator from config");
            return nullptr;
        }
        return handle;
    } catch (const std::exception& e) {
        handle->SetError(e.what());
        delete handle;
        return nullptr;
    }
}

void icarus_destroy(IcarusHandle* sim) {
    delete sim;  // safe if null
}

IcarusError icarus_stage(IcarusHandle* sim) {
    if (!sim || !sim->sim) return ICARUS_ERROR_NULL_HANDLE;

    try {
        sim->sim->Stage();
        return ICARUS_OK;
    } catch (const icarus::StageError& e) {
        sim->SetError(e.what());
        return ICARUS_ERROR_STAGE_FAILED;
    } catch (const std::exception& e) {
        sim->SetError(e.what());
        return ICARUS_ERROR_UNKNOWN;
    }
}

IcarusError icarus_step(IcarusHandle* sim, double dt) {
    if (!sim || !sim->sim) return ICARUS_ERROR_NULL_HANDLE;

    try {
        sim->sim->Step(dt);
        return ICARUS_OK;
    } catch (const std::exception& e) {
        sim->SetError(e.what());
        return ICARUS_ERROR_STEP_FAILED;
    }
}

} // extern "C"
```

### Signal Access Implementation

```cpp
extern "C" {

IcarusError icarus_get_signal(IcarusHandle* sim, const char* signal_name, double* out_value) {
    if (!sim || !sim->sim) return ICARUS_ERROR_NULL_HANDLE;
    if (!signal_name || !out_value) return ICARUS_ERROR_NULL_HANDLE;

    try {
        *out_value = sim->sim->Peek(signal_name);
        return ICARUS_OK;
    } catch (const icarus::SignalNotFoundError&) {
        sim->SetError(std::string("Signal not found: ") + signal_name);
        return ICARUS_ERROR_SIGNAL_NOT_FOUND;
    } catch (const std::exception& e) {
        sim->SetError(e.what());
        return ICARUS_ERROR_UNKNOWN;
    }
}

IcarusError icarus_set_signal(IcarusHandle* sim, const char* signal_name, double value) {
    if (!sim || !sim->sim) return ICARUS_ERROR_NULL_HANDLE;
    if (!signal_name) return ICARUS_ERROR_NULL_HANDLE;

    try {
        sim->sim->Poke(signal_name, value);
        return ICARUS_OK;
    } catch (const icarus::SignalNotFoundError&) {
        sim->SetError(std::string("Signal not found: ") + signal_name);
        return ICARUS_ERROR_SIGNAL_NOT_FOUND;
    } catch (const std::exception& e) {
        sim->SetError(e.what());
        return ICARUS_ERROR_UNKNOWN;
    }
}

} // extern "C"
```

---

## Tasks

### 7.1.1 Header and Types

- [x] Design: Complete API surface (see above)
- [x] Create `interfaces/c_api/icarus.h` with all declarations
- [x] Create `IcarusError` enum with comprehensive error codes
- [x] Document all functions with doxygen comments

### 7.1.2 Core Implementation

- [x] Create `interfaces/c_api/icarus_c.cpp`
- [x] Implement `IcarusHandle` internal structure
- [x] Implement lifecycle: `create`, `destroy`, `stage`, `step`, `reset`
- [x] Implement signal access: `get_signal`, `set_signal`
- [x] Implement bulk state access: `get_state_vector`, `set_state_vector`, `get_state_signal_names`

### 7.1.3 Error Handling

- [x] Implement thread-local error storage (for thread safety)
- [x] Implement `icarus_get_last_error()`
- [x] Implement `icarus_error_name()`
- [x] Test error propagation for all failure modes

### 7.1.4 Introspection

- [x] Implement `icarus_get_schema_json()` with caching
- [x] Implement `icarus_get_signal_names()`
- [x] Implement `icarus_get_signal_count()`
- [x] Implement `icarus_get_state_signal_names()`

### 7.1.5 Build Integration

- [x] Add CMake target `icarus_c` (shared library)
- [x] Export C symbols correctly (`__declspec(dllexport)` on Windows)
- [x] Install header to `include/icarus/icarus.h`
- [x] Add pkg-config file for Unix systems
- [x] Update `scripts/build.sh` with `--interfaces` flag
- [x] Update `scripts/ci.sh` and `scripts/verify.sh` with `--interfaces` support

---

## Build Configuration

### CMakeLists.txt

```cmake
# interfaces/c/CMakeLists.txt

add_library(icarus_c SHARED
    icarus_c_api.cpp
)

target_link_libraries(icarus_c PRIVATE icarus_core)

# Ensure C linkage
set_target_properties(icarus_c PROPERTIES
    PUBLIC_HEADER icarus.h
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Install
install(TARGETS icarus_c
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include/icarus
)

# pkg-config
configure_file(icarus.pc.in icarus.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/icarus.pc
    DESTINATION lib/pkgconfig)
```

---

## Verification Plan

### Unit Tests

#### `tests/interfaces/test_c_api.cpp`

```cpp
TEST(CApi, CreateDestroy) {
    auto handle = icarus_create("test_config.yaml");
    ASSERT_NE(handle, nullptr);
    icarus_destroy(handle);
}

TEST(CApi, StageAndStep) {
    auto handle = icarus_create("test_config.yaml");
    ASSERT_EQ(icarus_stage(handle), ICARUS_OK);
    ASSERT_EQ(icarus_step(handle, 0.01), ICARUS_OK);
    icarus_destroy(handle);
}

TEST(CApi, GetSetSignal) {
    auto handle = icarus_create("test_config.yaml");
    icarus_stage(handle);

    double value = 0;
    ASSERT_EQ(icarus_get_signal(handle, "Vehicle.Nav.altitude", &value), ICARUS_OK);
    ASSERT_EQ(icarus_set_signal(handle, "Vehicle.Nav.altitude", 1000.0), ICARUS_OK);

    icarus_destroy(handle);
}

TEST(CApi, ErrorHandling) {
    auto handle = icarus_create("nonexistent.yaml");
    ASSERT_EQ(handle, nullptr);

    // Test null handle
    double val;
    ASSERT_EQ(icarus_get_signal(nullptr, "foo", &val), ICARUS_ERROR_NULL_HANDLE);

    // Test signal not found
    handle = icarus_create("test_config.yaml");
    icarus_stage(handle);
    ASSERT_EQ(icarus_get_signal(handle, "NonExistent.Signal", &val), ICARUS_ERROR_SIGNAL_NOT_FOUND);

    const char* err = icarus_get_last_error(handle);
    ASSERT_NE(err, nullptr);
    ASSERT_STRNE(err, "");

    icarus_destroy(handle);
}

TEST(CApi, BulkStateAccess) {
    auto handle = icarus_create("test_config.yaml");
    icarus_stage(handle);

    size_t state_size = icarus_get_state_size(handle);
    std::vector<double> state(state_size);
    size_t actual_size;

    ASSERT_EQ(icarus_get_state_vector(handle, state.data(), state_size, &actual_size), ICARUS_OK);
    ASSERT_EQ(actual_size, state_size);

    // Modify and set back
    state[0] += 1.0;
    ASSERT_EQ(icarus_set_state_vector(handle, state.data(), state_size), ICARUS_OK);

    icarus_destroy(handle);
}
```

### Integration Tests

#### From C (extern linkage test)

```c
// tests/interfaces/test_c_api_c.c
#include <icarus/icarus.h>
#include <assert.h>

int main() {
    IcarusHandle* sim = icarus_create("test_config.yaml");
    assert(sim != NULL);

    assert(icarus_stage(sim) == ICARUS_OK);

    for (int i = 0; i < 100; ++i) {
        assert(icarus_step(sim, 0.01) == ICARUS_OK);
    }

    double alt;
    assert(icarus_get_signal(sim, "Vehicle.Nav.altitude", &alt) == ICARUS_OK);

    icarus_destroy(sim);
    return 0;
}
```

---

## Performance Considerations

| Operation | Expected Overhead | Notes |
|-----------|------------------|-------|
| `icarus_create()` | ~100ms | Config parsing, component creation |
| `icarus_stage()` | ~10-1000ms | Wiring validation, optional trim |
| `icarus_step()` | ~100μs-10ms | Depends on complexity |
| `icarus_get_signal()` | ~100ns | String lookup (O(1) hash) |
| `icarus_get_state_vector()` | ~1μs | Bulk memcpy |

### Optimization Notes

1. **Signal Caching:** For repeated access, consider `icarus_get_signal_handle()` returning a numeric ID for O(1) access
2. **Batch Operations:** Future: `icarus_step_batch(n, dt)` for Monte Carlo
3. **ZeroMQ Integration:** Future: `icarus_serve(port)` for network access

---

## Thread Safety

The C API is **NOT** thread-safe by default:
- Single `IcarusHandle*` must not be accessed from multiple threads
- Different handles can be used from different threads
- Error messages are stored per-handle (no global state)

Future: Add `icarus_create_threadsafe()` with internal locking if needed.

---

## Dependencies

- None beyond core Icarus library
- Standard C headers only in public API
- C++17 internally for implementation
