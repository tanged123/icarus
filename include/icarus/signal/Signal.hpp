#pragma once

#include <cstdint>
#include <limits>
#include <string>

namespace icarus {

/**
 * @brief Supported signal data types
 */
enum class SignalType : uint8_t {
    Float64, // double
    Int32,   // int32_t (also used for booleans: 0/1)
    Int64    // int64_t
};

/**
 * @brief Signal lifecycle classification
 */
enum class SignalLifecycle : uint8_t {
    Static, // Set at Provision/Stage, immutable during run (parameters)
    Dynamic // Updated every Step (state, outputs)
};

/**
 * @brief Descriptor for a signal on the backplane
 */
struct SignalDescriptor {
    std::string name;          // Full path: "Falcon9.Propulsion.Thrust"
    std::string unit;          // Physical unit: "N", "m/s", "rad"
    SignalType type;           // Data type
    SignalLifecycle lifecycle; // Static or Dynamic
    std::string description;   // Human-readable description

    // Optional metadata
    double min_value = -std::numeric_limits<double>::infinity();
    double max_value = std::numeric_limits<double>::infinity();
    bool is_state = false; // Requires integration
};

} // namespace icarus
