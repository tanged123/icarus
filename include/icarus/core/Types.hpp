#pragma once

#include <cstdint>
#include <string>

namespace icarus {

/**
 * @brief Simulation lifecycle phases
 */
enum class Phase : uint8_t {
    Uninitialized, // Before Provision
    Provisioned,   // After Provision, before Stage
    Staged,        // After Stage, ready to run
    Running,       // During Step loop
    Paused,        // Temporarily halted
    Completed,     // Simulation finished
    Error          // Error state
};

/**
 * @brief Component configuration structure
 *
 * Passed to components during Provision phase.
 */
struct ComponentConfig {
    std::string name;        // Component instance name
    std::string entity;      // Entity namespace (optional)
    std::string config_path; // Path to config file (optional)
    // Additional config fields will be added as needed
};

/**
 * @brief Version information
 */
constexpr const char *Version() { return "0.1.0"; }

} // namespace icarus
