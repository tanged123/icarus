#pragma once

/**
 * @file Signal.hpp
 * @brief Signal types and descriptors for the Icarus Signal Backplane
 *
 * Part of Phase 1.3: Signal Backplane.
 *
 * Re-exports Vulcan's core signal types (SignalType, SignalLifecycle) for
 * consistency, and extends SignalDescriptor with Icarus-specific fields
 * for pointer binding, owner tracking, and simulation metadata.
 */

#include <vulcan/io/Signal.hpp>

#include <cstdint>
#include <limits>
#include <string>

namespace icarus {

// =============================================================================
// Re-export Vulcan Signal Types
// =============================================================================

/// Signal data type (re-exported from Vulcan for consistency)
using SignalType = vulcan::io::SignalType;

/// Signal lifecycle (re-exported from Vulcan for consistency)
using SignalLifecycle = vulcan::io::SignalLifecycle;

// =============================================================================
// TypeTraits - Compile-time type ID mapping
// =============================================================================

/**
 * @brief Primary template for TypeTraits (generates compile error if not specialized)
 * @tparam T The type to get traits for
 */
template <typename T> struct TypeTraits {
    static_assert(sizeof(T) == 0, "TypeTraits not specialized for this type. "
                                  "Supported types: double, int32_t, int64_t");
};

/**
 * @brief TypeTraits specialization for double
 */
template <> struct TypeTraits<double> {
    static constexpr SignalType type_id = SignalType::Double;
    static constexpr const char *name = "Double";
};

/**
 * @brief TypeTraits specialization for int32_t
 */
template <> struct TypeTraits<int32_t> {
    static constexpr SignalType type_id = SignalType::Int32;
    static constexpr const char *name = "Int32";
};

/**
 * @brief TypeTraits specialization for int64_t
 */
template <> struct TypeTraits<int64_t> {
    static constexpr SignalType type_id = SignalType::Int64;
    static constexpr const char *name = "Int64";
};

} // namespace icarus

// Include Icarus types for SymbolicScalar specialization
#include <icarus/core/Types.hpp>

namespace icarus {

/**
 * @brief TypeTraits specialization for SymbolicScalar (casadi::MX)
 *
 * Symbolic scalars map to SignalType::Double since they represent
 * floating-point values in the computational graph.
 */
template <> struct TypeTraits<SymbolicScalar> {
    static constexpr SignalType type_id = SignalType::Double;
    static constexpr const char *name = "SymbolicDouble";
};

// =============================================================================
// SignalDescriptor (extends Vulcan's with Icarus-specific fields)
// =============================================================================

/**
 * @brief Descriptor for a signal on the backplane
 *
 * Extends Vulcan's SignalDescriptor concept with Icarus-specific fields:
 * - `owner_component`: For collision detection during registration
 * - `data_ptr`: For pointer-bind pattern (zero-overhead hot path)
 * - `description`: Human-readable documentation
 * - `min_value/max_value`: Value bounds for validation
 * - `is_state`: Flag for signals requiring integration
 *
 * Compatible with Vulcan's telemetry system while adding simulation features.
 */
struct SignalDescriptor {
    // Core fields (compatible with Vulcan's SignalDescriptor)
    std::string name;          ///< Full path: "Falcon9.Propulsion.Thrust"
    SignalType type;           ///< Data type (Double, Int32, Int64)
    SignalLifecycle lifecycle; ///< Static or Dynamic
    std::string unit;          ///< Physical unit: "N", "m/s", "rad"
    std::string semantic;      ///< Optional semantic hint: "boolean", "enum"

    // Icarus extensions
    std::string description;     ///< Human-readable description
    std::string owner_component; ///< Component that registered this signal
    void *data_ptr = nullptr;    ///< Pointer to storage (for pointer-bind pattern)

    // Simulation metadata
    double min_value = -std::numeric_limits<double>::infinity();
    double max_value = std::numeric_limits<double>::infinity();
    bool is_state = false; ///< Requires integration

    /// Get padded size in bytes (always 8 for wire format alignment, matching Vulcan)
    /// Note: Actual data size may be smaller (e.g., int32_t is 4 bytes)
    [[nodiscard]] constexpr size_t size_bytes() const { return 8; }
};

} // namespace icarus
