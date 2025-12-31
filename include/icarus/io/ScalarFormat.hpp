#pragma once

/**
 * @file ScalarFormat.hpp
 * @brief Scalar type formatting utilities for dual-mode compatibility
 *
 * Part of Phase 3.5: Logger Symbolic Support
 * Provides templated formatting that works for both double and SymbolicScalar.
 */

#include <icarus/core/CoreTypes.hpp>

#include <iomanip>
#include <sstream>
#include <string>

namespace icarus::io {

/**
 * @brief Format a scalar value to string
 *
 * Specializations handle numeric vs symbolic types appropriately.
 */
template <typename Scalar> struct ScalarFormatter {
    static std::string format(const Scalar &value, int precision = 6) {
        std::ostringstream ss;
        ss << std::setprecision(precision) << value;
        return ss.str();
    }

    static std::string format_fixed(const Scalar &value, int precision = 6) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    }

    static std::string format_scientific(const Scalar &value, int precision = 6) {
        std::ostringstream ss;
        ss << std::scientific << std::setprecision(precision) << value;
        return ss.str();
    }
};

/**
 * @brief Specialization for double (numeric mode)
 */
template <> struct ScalarFormatter<double> {
    static std::string format(const double &value, int precision = 6) {
        std::ostringstream ss;
        ss << std::setprecision(precision) << value;
        return ss.str();
    }

    static std::string format_fixed(const double &value, int precision = 6) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(precision) << value;
        return ss.str();
    }

    static std::string format_scientific(const double &value, int precision = 6) {
        std::ostringstream ss;
        ss << std::scientific << std::setprecision(precision) << value;
        return ss.str();
    }

    /**
     * @brief Check if value is numeric (always true for double)
     */
    static constexpr bool is_numeric() { return true; }
};

/**
 * @brief Specialization for SymbolicScalar (symbolic mode)
 */
template <> struct ScalarFormatter<SymbolicScalar> {
    static std::string format(const SymbolicScalar &value, int /*precision*/ = 6) {
        std::ostringstream ss;
        ss << value;
        return ss.str();
    }

    static std::string format_fixed(const SymbolicScalar &value, int /*precision*/ = 6) {
        return format(value); // No special formatting for symbolic
    }

    static std::string format_scientific(const SymbolicScalar &value, int /*precision*/ = 6) {
        return format(value); // No special formatting for symbolic
    }

    /**
     * @brief Check if value is numeric (always false for symbolic)
     */
    static constexpr bool is_numeric() { return false; }

    /**
     * @brief Check if this MX is a constant (evaluates to a single numeric value)
     */
    static bool is_constant(const SymbolicScalar &value) { return value.is_constant(); }

    /**
     * @brief Get constant value (only valid if is_constant returns true)
     */
    static double get_constant(const SymbolicScalar &value) {
        return static_cast<double>(casadi::DM(value));
    }
};

// =============================================================================
// Convenience Functions
// =============================================================================

/**
 * @brief Format a scalar value (auto-dispatch)
 */
template <typename Scalar> inline std::string FormatScalar(const Scalar &value, int precision = 6) {
    return ScalarFormatter<Scalar>::format(value, precision);
}

/**
 * @brief Format a scalar with fixed notation
 */
template <typename Scalar>
inline std::string FormatScalarFixed(const Scalar &value, int precision = 6) {
    return ScalarFormatter<Scalar>::format_fixed(value, precision);
}

/**
 * @brief Format a scalar with scientific notation
 */
template <typename Scalar>
inline std::string FormatScalarScientific(const Scalar &value, int precision = 6) {
    return ScalarFormatter<Scalar>::format_scientific(value, precision);
}

/**
 * @brief Check if a scalar type is numeric
 */
template <typename Scalar> constexpr bool IsNumericScalar() {
    return ScalarFormatter<Scalar>::is_numeric();
}

/**
 * @brief Check if a value can be safely converted to double
 */
template <typename Scalar> bool CanEvaluateToDouble(const Scalar &value) {
    if constexpr (std::is_same_v<Scalar, double>) {
        (void)value; // Unused
        return true;
    } else if constexpr (std::is_same_v<Scalar, SymbolicScalar>) {
        return value.is_constant();
    } else {
        (void)value;
        return false;
    }
}

/**
 * @brief Safely get double value if possible
 */
template <typename Scalar> double EvaluateToDouble(const Scalar &value, double default_val = 0.0) {
    if constexpr (std::is_same_v<Scalar, double>) {
        return value;
    } else if constexpr (std::is_same_v<Scalar, SymbolicScalar>) {
        if (value.is_constant()) {
            return static_cast<double>(casadi::DM(value));
        }
        return default_val;
    } else {
        (void)value;
        return default_val;
    }
}

} // namespace icarus::io
