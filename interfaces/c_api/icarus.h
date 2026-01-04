/**
 * @file icarus.h
 * @brief Icarus C API - Universal FFI interface
 *
 * This header provides the C API for Icarus, enabling bindings from
 * any language that supports C FFI (Python, MATLAB, Julia, Rust, etc.)
 *
 * Design Notes:
 * - All functions use opaque IcarusHandle* (users never see C++ internals)
 * - All functions return IcarusError codes (0 = success)
 * - Last error message retrievable via icarus_get_last_error()
 * - States ARE signals - bulk state API exists for performance optimization
 *
 * @see docs/implementation_plans/phase7/phase7_1_c_api.md
 */

#ifndef ICARUS_H
#define ICARUS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * Platform-specific export macros
 * ===========================================================================*/

#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef ICARUS_C_BUILDING_DLL
#define ICARUS_API __declspec(dllexport)
#else
#define ICARUS_API __declspec(dllimport)
#endif
#else
#define ICARUS_API __attribute__((visibility("default")))
#endif

/* =============================================================================
 * Types
 * ===========================================================================*/

/** @brief Opaque handle to Icarus Simulator */
typedef struct IcarusHandle IcarusHandle;

/** @brief Error codes returned by all API functions */
typedef enum IcarusError {
    ICARUS_OK = 0,                      /**< Success */
    ICARUS_ERROR_NULL_HANDLE = -1,      /**< NULL handle or argument */
    ICARUS_ERROR_CONFIG_LOAD = -2,      /**< Failed to load configuration */
    ICARUS_ERROR_STAGE_FAILED = -3,     /**< Stage() failed */
    ICARUS_ERROR_STEP_FAILED = -4,      /**< Step() failed */
    ICARUS_ERROR_SIGNAL_NOT_FOUND = -5, /**< Signal name not found */
    ICARUS_ERROR_TYPE_MISMATCH = -6,    /**< Type mismatch (e.g., vector vs scalar) */
    ICARUS_ERROR_INVALID_STATE = -7,    /**< Invalid lifecycle state */
    ICARUS_ERROR_BUFFER_TOO_SMALL = -8, /**< Output buffer too small */
    ICARUS_ERROR_ALLOCATION = -9,       /**< Memory allocation failed */
    ICARUS_ERROR_UNKNOWN = -99          /**< Unknown error */
} IcarusError;

/** @brief Simulation lifecycle state */
typedef enum IcarusLifecycle {
    ICARUS_LIFECYCLE_UNINITIALIZED = 0,
    ICARUS_LIFECYCLE_PROVISIONED = 1,
    ICARUS_LIFECYCLE_STAGED = 2,
    ICARUS_LIFECYCLE_RUNNING = 3
} IcarusLifecycle;

/* =============================================================================
 * Lifecycle Functions
 * ===========================================================================*/

/**
 * @brief Create simulator from YAML configuration file
 *
 * Loads the configuration, creates components, and provisions them.
 * After this call, the simulator is ready for icarus_stage().
 *
 * @param config_path Path to simulation YAML configuration
 * @return Handle to simulator, or NULL on failure (check icarus_get_last_error(NULL))
 */
ICARUS_API IcarusHandle *icarus_create(const char *config_path);

/**
 * @brief Destroy simulator and free all resources
 *
 * Safe to call with NULL handle (no-op).
 *
 * @param sim Handle to destroy
 */
ICARUS_API void icarus_destroy(IcarusHandle *sim);

/**
 * @brief Stage the simulation
 *
 * Prepares the simulation for execution:
 * - Validates signal wiring
 * - Applies initial conditions
 * - Runs trim optimization (if configured)
 * - Generates symbolic graphs (if configured)
 *
 * @param sim Simulator handle
 * @return ICARUS_OK on success, error code on failure
 */
ICARUS_API IcarusError icarus_stage(IcarusHandle *sim);

/**
 * @brief Execute one simulation step
 *
 * Advances the simulation by the specified timestep.
 * Uses configured integrator (RK4, RK45, etc.).
 *
 * @param sim Simulator handle
 * @param dt Timestep in seconds (use 0 for configured dt)
 * @return ICARUS_OK on success, error code on failure
 */
ICARUS_API IcarusError icarus_step(IcarusHandle *sim, double dt);

/**
 * @brief Reset simulation to initial state
 *
 * Re-applies initial conditions and resets time to 0.
 * Simulator must be staged before reset.
 *
 * @param sim Simulator handle
 * @return ICARUS_OK on success, error code on failure
 */
ICARUS_API IcarusError icarus_reset(IcarusHandle *sim);

/**
 * @brief Get current simulation lifecycle state
 *
 * @param sim Simulator handle
 * @return Current lifecycle state
 */
ICARUS_API IcarusLifecycle icarus_get_lifecycle(IcarusHandle *sim);

/* =============================================================================
 * Signal Access (Per-Signal)
 *
 * Use these for casual access to individual signals.
 * For bulk operations, see State Access section below.
 * ===========================================================================*/

/**
 * @brief Get scalar signal value by name
 *
 * @param sim Simulator handle
 * @param signal_name Full signal path (e.g., "Vehicle.Nav.altitude")
 * @param out_value Output: signal value
 * @return ICARUS_OK on success, ICARUS_ERROR_SIGNAL_NOT_FOUND if not found
 */
ICARUS_API IcarusError icarus_get_signal(IcarusHandle *sim, const char *signal_name,
                                         double *out_value);

/**
 * @brief Set scalar signal value by name
 *
 * @param sim Simulator handle
 * @param signal_name Full signal path
 * @param value Value to set
 * @return ICARUS_OK on success, error code on failure
 */
ICARUS_API IcarusError icarus_set_signal(IcarusHandle *sim, const char *signal_name, double value);

/* =============================================================================
 * Bulk State Access (High Performance)
 *
 * NOTE: States ARE signals in Icarus (Phase 6 unified signal model).
 * Every integrable state is accessible both via icarus_get_signal() by name
 * AND via this bulk state vector API.
 *
 * The bulk API exists for performance-critical use cases:
 *   - External integrators needing the full state vector
 *   - Monte Carlo IC perturbation (modify many states at once)
 *   - Checkpointing / warmstart (save/restore full state)
 *   - High-frequency logging (avoid N string lookups per frame)
 *
 * For casual access, prefer icarus_get_signal() with the state's signal name.
 * ===========================================================================*/

/**
 * @brief Get full state vector
 *
 * Copies the current state vector into the provided buffer.
 * Use icarus_get_state_size() to determine required buffer size.
 *
 * @param sim Simulator handle
 * @param out_buffer Output buffer (caller allocated)
 * @param buffer_size Size of output buffer (number of doubles)
 * @param out_size Actual state vector size written
 * @return ICARUS_OK on success, ICARUS_ERROR_BUFFER_TOO_SMALL if buffer too small
 */
ICARUS_API IcarusError icarus_get_state_vector(IcarusHandle *sim, double *out_buffer,
                                               size_t buffer_size, size_t *out_size);

/**
 * @brief Set full state vector
 *
 * Replaces the current state vector with the provided values.
 * Size must match icarus_get_state_size().
 *
 * @param sim Simulator handle
 * @param buffer State values
 * @param size Number of elements (must match state size)
 * @return ICARUS_OK on success, error code on failure
 */
ICARUS_API IcarusError icarus_set_state_vector(IcarusHandle *sim, const double *buffer,
                                               size_t size);

/**
 * @brief Get state vector size
 *
 * @param sim Simulator handle
 * @return Number of state elements, or 0 on error
 */
ICARUS_API size_t icarus_get_state_size(IcarusHandle *sim);

/**
 * @brief Get state signal names (in state vector order)
 *
 * Returns the signal names corresponding to each element of the state vector.
 * Use this to understand what icarus_get_state_vector() returns.
 *
 * The returned names are valid until the next API call on this handle.
 *
 * @param sim Simulator handle
 * @param out_names Array of string pointers (caller allocated)
 * @param max_count Maximum names to return
 * @param out_count Actual count written
 * @return ICARUS_OK on success
 *
 * Example:
 * @code
 *   size_t n = icarus_get_state_size(sim);
 *   const char** names = malloc(n * sizeof(char*));
 *   icarus_get_state_signal_names(sim, names, n, &n);
 *   // names[0] might be "Vehicle.Body.position.x"
 *   free(names);
 * @endcode
 */
ICARUS_API IcarusError icarus_get_state_signal_names(IcarusHandle *sim, const char **out_names,
                                                     size_t max_count, size_t *out_count);

/* =============================================================================
 * Time Access
 * ===========================================================================*/

/**
 * @brief Get current simulation time (MET)
 *
 * @param sim Simulator handle
 * @return Mission Elapsed Time in seconds, or NaN on error
 */
ICARUS_API double icarus_get_time(IcarusHandle *sim);

/**
 * @brief Get configured timestep
 *
 * @param sim Simulator handle
 * @return Timestep in seconds, or NaN on error
 */
ICARUS_API double icarus_get_dt(IcarusHandle *sim);

/**
 * @brief Get configured end time
 *
 * @param sim Simulator handle
 * @return End time in seconds, or NaN on error
 */
ICARUS_API double icarus_get_end_time(IcarusHandle *sim);

/* =============================================================================
 * Introspection
 * ===========================================================================*/

/**
 * @brief Get data dictionary as JSON string
 *
 * Returns a JSON representation of all signals, their types, and metadata.
 * Caller must free the returned string with icarus_free_string().
 *
 * @param sim Simulator handle
 * @return JSON string (caller must free), or NULL on error
 */
ICARUS_API const char *icarus_get_schema_json(IcarusHandle *sim);

/**
 * @brief Get number of signals
 *
 * @param sim Simulator handle
 * @return Total signal count, or 0 on error
 */
ICARUS_API size_t icarus_get_signal_count(IcarusHandle *sim);

/**
 * @brief Get all signal names
 *
 * The returned names are valid until the next API call on this handle.
 *
 * @param sim Simulator handle
 * @param out_names Array of string pointers (caller allocated)
 * @param max_count Maximum number of names to return
 * @param out_count Actual number of names returned
 * @return ICARUS_OK on success
 */
ICARUS_API IcarusError icarus_get_signal_names(IcarusHandle *sim, const char **out_names,
                                               size_t max_count, size_t *out_count);

/* =============================================================================
 * Error Handling
 * ===========================================================================*/

/**
 * @brief Get last error message
 *
 * Returns the error message from the last failed operation.
 * The returned string is valid until the next API call on this handle.
 *
 * @param sim Simulator handle (can be NULL for creation errors)
 * @return Error message string, or empty string if no error
 */
ICARUS_API const char *icarus_get_last_error(IcarusHandle *sim);

/**
 * @brief Get error code name as string
 *
 * @param error Error code
 * @return Human-readable error name (e.g., "ICARUS_OK")
 */
ICARUS_API const char *icarus_error_name(IcarusError error);

/* =============================================================================
 * Memory Management
 * ===========================================================================*/

/**
 * @brief Free string allocated by Icarus
 *
 * Use this to free strings returned by icarus_get_schema_json().
 *
 * @param str String to free (safe to pass NULL)
 */
ICARUS_API void icarus_free_string(const char *str);

/* =============================================================================
 * Version Information
 * ===========================================================================*/

/**
 * @brief Get Icarus version string
 *
 * @return Version string (e.g., "0.5.1")
 */
ICARUS_API const char *icarus_version(void);

/**
 * @brief Get Icarus version components
 *
 * @param major Output: major version
 * @param minor Output: minor version
 * @param patch Output: patch version
 */
ICARUS_API void icarus_version_components(int *major, int *minor, int *patch);

#ifdef __cplusplus
}
#endif

#endif /* ICARUS_H */
