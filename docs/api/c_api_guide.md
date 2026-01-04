# C API Guide

**Related:** [Python API Guide](python_api_guide.md) | [Component Authoring](component_authoring_guide.md)

---

The Icarus C API provides a stable FFI interface for integrating with any language that supports C bindings (Python, MATLAB, Julia, Rust, etc.). This guide covers usage patterns and best practices.

## Quick Start

```c
#include <icarus.h>
#include <stdio.h>

int main() {
    // Create simulator from YAML config
    IcarusHandle *sim = icarus_create("config/simulation.yaml");
    if (!sim) {
        printf("Error: %s\n", icarus_get_last_error(NULL));
        return 1;
    }

    // Stage (validate wiring, apply ICs)
    if (icarus_stage(sim) != ICARUS_OK) {
        printf("Stage failed: %s\n", icarus_get_last_error(sim));
        icarus_destroy(sim);
        return 1;
    }

    // Run simulation loop
    while (icarus_get_time(sim) < icarus_get_end_time(sim)) {
        double altitude;
        icarus_get_signal(sim, "Vehicle.position.z", &altitude);
        printf("t=%.2f, alt=%.1f\n", icarus_get_time(sim), altitude);

        icarus_step(sim, 0.0);  // 0 = use configured dt
    }

    icarus_destroy(sim);
    return 0;
}
```

## Building and Linking

### With pkg-config (Unix)

```bash
# Compile
gcc -o my_sim my_sim.c $(pkg-config --cflags --libs icarus)

# Or with CMake
find_package(PkgConfig REQUIRED)
pkg_check_modules(ICARUS REQUIRED icarus)
target_link_libraries(my_app ${ICARUS_LIBRARIES})
target_include_directories(my_app PRIVATE ${ICARUS_INCLUDE_DIRS})
```

### Direct Linking

```bash
gcc -o my_sim my_sim.c -I/path/to/icarus/include -L/path/to/icarus/lib -licarus_c
```

---

## API Reference

### Types

#### `IcarusHandle`

Opaque handle to a simulator instance. Created by `icarus_create()`, destroyed by `icarus_destroy()`.

#### `IcarusError`

| Code | Name | Description |
|------|------|-------------|
| 0 | `ICARUS_OK` | Success |
| -1 | `ICARUS_ERROR_NULL_HANDLE` | NULL handle or argument |
| -2 | `ICARUS_ERROR_CONFIG_LOAD` | Failed to load configuration |
| -3 | `ICARUS_ERROR_STAGE_FAILED` | Stage() failed |
| -4 | `ICARUS_ERROR_STEP_FAILED` | Step() failed |
| -5 | `ICARUS_ERROR_SIGNAL_NOT_FOUND` | Signal name not found |
| -6 | `ICARUS_ERROR_TYPE_MISMATCH` | Type mismatch |
| -7 | `ICARUS_ERROR_INVALID_STATE` | Invalid lifecycle state |
| -8 | `ICARUS_ERROR_BUFFER_TOO_SMALL` | Output buffer too small |
| -9 | `ICARUS_ERROR_ALLOCATION` | Memory allocation failed |
| -99 | `ICARUS_ERROR_UNKNOWN` | Unknown error |

#### `IcarusLifecycle`

| Value | Name | Description |
|-------|------|-------------|
| 0 | `ICARUS_LIFECYCLE_UNINITIALIZED` | Not yet configured |
| 1 | `ICARUS_LIFECYCLE_PROVISIONED` | Components loaded, ready to stage |
| 2 | `ICARUS_LIFECYCLE_STAGED` | Staged and ready to run |
| 3 | `ICARUS_LIFECYCLE_RUNNING` | Simulation in progress |

---

### Lifecycle Functions

#### `icarus_create`

```c
IcarusHandle *icarus_create(const char *config_path);
```

Create simulator from YAML configuration file. Returns NULL on failure; check `icarus_get_last_error(NULL)` for details.

#### `icarus_destroy`

```c
void icarus_destroy(IcarusHandle *sim);
```

Destroy simulator and free resources. Safe to call with NULL.

#### `icarus_stage`

```c
IcarusError icarus_stage(IcarusHandle *sim);
```

Prepare simulation for execution:
- Validates signal wiring
- Applies initial conditions
- Runs trim optimization (if configured)
- Generates symbolic graphs (if configured)

#### `icarus_step`

```c
IcarusError icarus_step(IcarusHandle *sim, double dt);
```

Execute one integration step. Pass `dt=0` to use configured timestep.

#### `icarus_reset`

```c
IcarusError icarus_reset(IcarusHandle *sim);
```

Reset to initial conditions, time to 0. Simulator transitions back to STAGED state.

#### `icarus_get_lifecycle`

```c
IcarusLifecycle icarus_get_lifecycle(IcarusHandle *sim);
```

Returns current lifecycle state. Returns `ICARUS_LIFECYCLE_UNINITIALIZED` for NULL handle.

---

### Signal Access

#### `icarus_get_signal`

```c
IcarusError icarus_get_signal(IcarusHandle *sim, const char *signal_name, double *out_value);
```

Get scalar signal value by name. Signal names use dot notation: `"Component.signal.axis"`.

```c
double altitude;
icarus_get_signal(sim, "Vehicle.position.z", &altitude);
```

#### `icarus_set_signal`

```c
IcarusError icarus_set_signal(IcarusHandle *sim, const char *signal_name, double value);
```

Set scalar signal value. Useful for injecting inputs or modifying states.

```c
icarus_set_signal(sim, "Vehicle.position.z", 1000.0);
```

---

### Bulk State Access

The bulk state API provides high-performance access to the full state vector. Use this for:
- External integrators needing full state
- Monte Carlo IC perturbation
- Checkpointing / warmstart
- High-frequency data logging

> **Note:** States ARE signals in Icarus. Every state is also accessible via `icarus_get_signal()` by name. The bulk API is an optimization.

#### `icarus_get_state_size`

```c
size_t icarus_get_state_size(IcarusHandle *sim);
```

Get number of state elements. Returns 0 for NULL handle.

#### `icarus_get_state_vector`

```c
IcarusError icarus_get_state_vector(IcarusHandle *sim, double *out_buffer,
                                     size_t buffer_size, size_t *out_size);
```

Copy state vector to caller-allocated buffer. Returns `ICARUS_ERROR_BUFFER_TOO_SMALL` if buffer is too small (with required size in `out_size`).

```c
size_t n = icarus_get_state_size(sim);
double *state = malloc(n * sizeof(double));
size_t actual;
icarus_get_state_vector(sim, state, n, &actual);
```

#### `icarus_set_state_vector`

```c
IcarusError icarus_set_state_vector(IcarusHandle *sim, const double *buffer, size_t size);
```

Replace state vector. Size must match `icarus_get_state_size()`.

#### `icarus_get_state_signal_names`

```c
IcarusError icarus_get_state_signal_names(IcarusHandle *sim, const char **out_names,
                                           size_t max_count, size_t *out_count);
```

Get signal names for each state element (in state vector order). Useful for understanding state vector layout.

```c
size_t n = icarus_get_state_size(sim);
const char **names = malloc(n * sizeof(char*));
size_t count;
icarus_get_state_signal_names(sim, names, n, &count);
// names[0] might be "Vehicle.position.x"
free(names);
```

---

### Time Access

#### `icarus_get_time`

```c
double icarus_get_time(IcarusHandle *sim);
```

Get current simulation time (Mission Elapsed Time) in seconds. Returns NaN for NULL handle.

#### `icarus_get_dt`

```c
double icarus_get_dt(IcarusHandle *sim);
```

Get configured timestep in seconds.

#### `icarus_get_end_time`

```c
double icarus_get_end_time(IcarusHandle *sim);
```

Get configured simulation end time in seconds.

---

### Introspection

#### `icarus_get_signal_count`

```c
size_t icarus_get_signal_count(IcarusHandle *sim);
```

Get total number of signals in the simulation.

#### `icarus_get_signal_names`

```c
IcarusError icarus_get_signal_names(IcarusHandle *sim, const char **out_names,
                                     size_t max_count, size_t *out_count);
```

Get all signal names. Returned strings are valid until next API call.

#### `icarus_get_schema_json`

```c
const char *icarus_get_schema_json(IcarusHandle *sim);
```

Get data dictionary as JSON string. **Caller must free with `icarus_free_string()`**.

```c
const char *json = icarus_get_schema_json(sim);
printf("Schema: %s\n", json);
icarus_free_string(json);  // Don't forget!
```

---

### Error Handling

#### `icarus_get_last_error`

```c
const char *icarus_get_last_error(IcarusHandle *sim);
```

Get error message from last failed operation. Pass NULL for creation errors.

#### `icarus_error_name`

```c
const char *icarus_error_name(IcarusError error);
```

Get human-readable name for error code.

```c
IcarusError err = icarus_step(sim, 0.01);
if (err != ICARUS_OK) {
    printf("Error %s: %s\n", icarus_error_name(err), icarus_get_last_error(sim));
}
```

---

### Memory Management

#### `icarus_free_string`

```c
void icarus_free_string(const char *str);
```

Free strings returned by `icarus_get_schema_json()`. Safe to call with NULL.

---

### Version Information

#### `icarus_version`

```c
const char *icarus_version(void);
```

Get version string (e.g., "0.5.1").

#### `icarus_version_components`

```c
void icarus_version_components(int *major, int *minor, int *patch);
```

Get version components.

```c
int major, minor, patch;
icarus_version_components(&major, &minor, &patch);
printf("Icarus v%d.%d.%d\n", major, minor, patch);
```

---

## Usage Patterns

### Monte Carlo Simulation

```c
#include <icarus.h>
#include <stdlib.h>
#include <time.h>

void run_monte_carlo(const char *config, int num_runs) {
    srand(time(NULL));

    for (int run = 0; run < num_runs; run++) {
        IcarusHandle *sim = icarus_create(config);
        icarus_stage(sim);

        // Perturb initial state
        size_t n = icarus_get_state_size(sim);
        double *state = malloc(n * sizeof(double));
        size_t actual;
        icarus_get_state_vector(sim, state, n, &actual);

        for (size_t i = 0; i < n; i++) {
            state[i] += ((double)rand()/RAND_MAX - 0.5) * 0.1;
        }
        icarus_set_state_vector(sim, state, n);
        free(state);

        // Run to completion
        while (icarus_get_time(sim) < icarus_get_end_time(sim)) {
            icarus_step(sim, 0.0);
        }

        // Collect results
        double final_alt;
        icarus_get_signal(sim, "Vehicle.position.z", &final_alt);
        printf("Run %d: final altitude = %.1f m\n", run, final_alt);

        icarus_destroy(sim);
    }
}
```

### Checkpointing

```c
// Save state
size_t n = icarus_get_state_size(sim);
double *checkpoint = malloc(n * sizeof(double));
icarus_get_state_vector(sim, checkpoint, n, &n);
double checkpoint_time = icarus_get_time(sim);

// ... later, restore
icarus_reset(sim);
icarus_set_state_vector(sim, checkpoint, n);
// Note: time is reset by icarus_reset(); you may need to track it separately
```

### External Integrator

```c
// Get current state
size_t n = icarus_get_state_size(sim);
double *x = malloc(n * sizeof(double));
double *xdot = malloc(n * sizeof(double));

icarus_get_state_vector(sim, x, n, &n);

// Use your own integrator
// (Note: derivative computation not exposed in C API yet)
// For now, use icarus_step() with custom dt

free(x);
free(xdot);
```

---

## Thread Safety

- Each `IcarusHandle` is independent and can be used from different threads
- Do not share a single `IcarusHandle` across threads without synchronization
- The global last-error for creation failures is thread-local

---

## See Also

- [Python API Guide](python_api_guide.md) - Higher-level Python interface
- [Component Authoring Guide](component_authoring_guide.md) - Writing custom components
- [icarus.h](../../interfaces/c_api/icarus.h) - Full header documentation
