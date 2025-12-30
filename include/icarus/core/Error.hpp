#pragma once

/**
 * @file Error.hpp
 * @brief Consolidated error handling for Icarus
 *
 * Provides a flattened exception hierarchy with 8 error categories.
 * Each category has contextual information rather than many subclasses.
 */

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace icarus {

// =============================================================================
// Error Severity (for SimulationError / logging)
// =============================================================================

enum class Severity : uint8_t {
    INFO,    ///< Informational (logged, no action)
    WARNING, ///< Warning (may trigger graceful degradation)
    ERROR,   ///< Error (simulation may continue with fallback)
    FATAL    ///< Fatal (simulation must stop)
};

// =============================================================================
// Simulation Error (structured error for OnError hooks)
// =============================================================================

struct SimulationError {
    Severity severity;
    std::string message;
    std::string component;
    double time = 0.0;
};

// =============================================================================
// Base Exception
// =============================================================================

/**
 * @brief Base class for all Icarus exceptions
 *
 * All Icarus exceptions carry:
 * - A severity level (defaults to ERROR)
 * - A category string for logging context
 * - Conversion to SimulationError for ErrorHandler integration
 */
class Error : public std::runtime_error {
  public:
    explicit Error(const std::string &msg, Severity severity = Severity::ERROR,
                   std::string category = "general")
        : std::runtime_error("[icarus] " + msg), severity_(severity),
          category_(std::move(category)) {}

    [[nodiscard]] Severity severity() const { return severity_; }
    [[nodiscard]] const std::string &category() const { return category_; }

    /// Convert to SimulationError for ErrorHandler integration
    [[nodiscard]] SimulationError toSimulationError(double time = 0.0,
                                                    const std::string &component = "") const {
        return SimulationError{.severity = severity_,
                               .message = what(),
                               .component = component.empty() ? category_ : component,
                               .time = time};
    }

  protected:
    Severity severity_;
    std::string category_;
};

// =============================================================================
// Signal Errors
// =============================================================================

/**
 * @brief Signal-related error categories
 */
enum class SignalErrorKind {
    NotFound,     ///< Signal does not exist
    Duplicate,    ///< Signal already registered
    TypeMismatch, ///< Type mismatch during resolution
    Unwired,      ///< Input not wired to output
    NullPointer   ///< Null data pointer provided
};

/**
 * @brief Signal-related errors (registration, resolution, wiring)
 */
class SignalError : public Error {
  public:
    // Legacy single-string constructor for backward compatibility
    explicit SignalError(const std::string &msg)
        : Error("Signal: " + msg, Severity::ERROR, "signal"), kind_(SignalErrorKind::NotFound) {}

    SignalError(SignalErrorKind kind, const std::string &signal_name,
                const std::string &detail = "")
        : Error(FormatMessage(kind, signal_name, detail), KindToSeverity(kind), "signal"),
          kind_(kind), signal_name_(signal_name) {}

    [[nodiscard]] SignalErrorKind kind() const { return kind_; }
    [[nodiscard]] const std::string &signal_name() const { return signal_name_; }

    // Convenience factory methods
    static SignalError NotFound(const std::string &name) {
        return {SignalErrorKind::NotFound, name};
    }

    static SignalError Duplicate(const std::string &name, const std::string &existing_owner,
                                 const std::string &new_owner) {
        return {SignalErrorKind::Duplicate, name,
                "already owned by '" + existing_owner + "', conflict from '" + new_owner + "'"};
    }

    static SignalError TypeMismatch(const std::string &name, const std::string &expected,
                                    const std::string &actual) {
        return {SignalErrorKind::TypeMismatch, name, "expected " + expected + ", got " + actual};
    }

    static SignalError Unwired(const std::string &name) { return {SignalErrorKind::Unwired, name}; }

    static SignalError NullPointer(const std::string &name, const std::string &context = "") {
        return {SignalErrorKind::NullPointer, name, context};
    }

  private:
    static Severity KindToSeverity(SignalErrorKind kind) {
        switch (kind) {
        case SignalErrorKind::NotFound:
            return Severity::ERROR;
        case SignalErrorKind::Duplicate:
            return Severity::ERROR;
        case SignalErrorKind::TypeMismatch:
            return Severity::ERROR;
        case SignalErrorKind::Unwired:
            return Severity::WARNING; // May be intentional
        case SignalErrorKind::NullPointer:
            return Severity::FATAL; // Programming error
        }
        return Severity::ERROR;
    }

    static std::string FormatMessage(SignalErrorKind kind, const std::string &name,
                                     const std::string &detail) {
        std::string prefix;
        switch (kind) {
        case SignalErrorKind::NotFound:
            prefix = "Signal not found";
            break;
        case SignalErrorKind::Duplicate:
            prefix = "Duplicate signal";
            break;
        case SignalErrorKind::TypeMismatch:
            prefix = "Type mismatch";
            break;
        case SignalErrorKind::Unwired:
            prefix = "Unwired input";
            break;
        case SignalErrorKind::NullPointer:
            prefix = "Null pointer";
            break;
        }
        std::string msg = prefix + ": '" + name + "'";
        if (!detail.empty()) {
            msg += " (" + detail + ")";
        }
        return msg;
    }

    SignalErrorKind kind_;
    std::string signal_name_;
};

// =============================================================================
// Configuration Errors
// =============================================================================

/**
 * @brief Configuration/parsing errors with optional file context
 */
class ConfigError : public Error {
  public:
    explicit ConfigError(const std::string &msg)
        : Error("Config: " + msg, Severity::ERROR, "config") {}

    ConfigError(const std::string &component, const std::string &key)
        : Error("Config: " + component + " missing required key '" + key + "'", Severity::ERROR,
                "config") {}

    ConfigError(const std::string &message, const std::string &file, int line,
                const std::string &hint = "")
        : Error(FormatMessage(message, file, line, hint), Severity::ERROR, "config"), file_(file),
          line_(line), hint_(hint) {}

    [[nodiscard]] const std::string &file() const { return file_; }
    [[nodiscard]] int line() const { return line_; }
    [[nodiscard]] const std::string &hint() const { return hint_; }

  private:
    static std::string FormatMessage(const std::string &msg, const std::string &file, int line,
                                     const std::string &hint) {
        std::string result = "Config: " + msg;
        if (!file.empty()) {
            result += "\n  at: " + file;
            if (line >= 0) {
                result += ":" + std::to_string(line);
            }
        }
        if (!hint.empty()) {
            result += "\n  hint: " + hint;
        }
        return result;
    }

    std::string file_;
    int line_ = -1;
    std::string hint_;
};

// =============================================================================
// Lifecycle Errors
// =============================================================================

/**
 * @brief Simulation lifecycle phases
 */
enum class LifecyclePhase { Provision, Stage, Step, Reset, Other };

/**
 * @brief Lifecycle ordering/state errors
 */
class LifecycleError : public Error {
  public:
    explicit LifecycleError(const std::string &msg)
        : Error("Lifecycle: " + msg, Severity::ERROR, "lifecycle"), phase_(LifecyclePhase::Other) {}

    LifecycleError(LifecyclePhase phase, const std::string &msg)
        : Error("Lifecycle [" + PhaseName(phase) + "]: " + msg, Severity::ERROR, "lifecycle"),
          phase_(phase) {}

    [[nodiscard]] LifecyclePhase phase() const { return phase_; }

  private:
    static std::string PhaseName(LifecyclePhase phase) {
        switch (phase) {
        case LifecyclePhase::Provision:
            return "Provision";
        case LifecyclePhase::Stage:
            return "Stage";
        case LifecyclePhase::Step:
            return "Step";
        case LifecyclePhase::Reset:
            return "Reset";
        case LifecyclePhase::Other:
            return "Other";
        }
        return "Unknown";
    }

    LifecyclePhase phase_;
};

// =============================================================================
// Wiring/Routing Errors
// =============================================================================

/**
 * @brief Wiring and signal routing errors
 */
class WiringError : public Error {
  public:
    explicit WiringError(const std::string &msg)
        : Error("Wiring: " + msg, Severity::ERROR, "wiring") {}

    WiringError(const std::string &input, const std::string &output, const std::string &reason)
        : Error("Wiring: cannot connect '" + input + "' -> '" + output + "': " + reason,
                Severity::ERROR, "wiring"),
          input_(input), output_(output) {}

    [[nodiscard]] const std::string &input() const { return input_; }
    [[nodiscard]] const std::string &output() const { return output_; }

  private:
    std::string input_;
    std::string output_;
};

// =============================================================================
// Integration/ODE Errors
// =============================================================================

/**
 * @brief Integration/ODE solver errors with time context
 */
class IntegrationError : public Error {
  public:
    explicit IntegrationError(const std::string &msg)
        : Error("Integration: " + msg, Severity::ERROR, "integration") {}

    IntegrationError(double t, double dt, const std::string &reason)
        : Error("Integration failed at t=" + std::to_string(t) + ", dt=" + std::to_string(dt) +
                    ": " + reason,
                Severity::ERROR, "integration"),
          t_(t), dt_(dt) {}

    [[nodiscard]] std::optional<double> t() const { return t_; }
    [[nodiscard]] std::optional<double> dt() const { return dt_; }

    // Convenience for step size errors
    static IntegrationError StepTooSmall(double min_dt) {
        return IntegrationError("Step size below minimum: " + std::to_string(min_dt));
    }

  private:
    std::optional<double> t_;
    std::optional<double> dt_;
};

// =============================================================================
// State Errors
// =============================================================================

/**
 * @brief State vector management errors
 */
class StateError : public Error {
  public:
    explicit StateError(const std::string &msg)
        : Error("State: " + msg, Severity::ERROR, "state") {}

    StateError(std::size_t expected, std::size_t actual)
        : Error("State: size mismatch - expected " + std::to_string(expected) + ", got " +
                    std::to_string(actual),
                Severity::FATAL, "state"), // Size mismatch is a programming error
          expected_(expected), actual_(actual) {}

    [[nodiscard]] std::optional<std::size_t> expected() const { return expected_; }
    [[nodiscard]] std::optional<std::size_t> actual() const { return actual_; }

  private:
    std::optional<std::size_t> expected_;
    std::optional<std::size_t> actual_;
};

// =============================================================================
// I/O Errors
// =============================================================================

/**
 * @brief File and I/O operation errors
 */
class IOError : public Error {
  public:
    explicit IOError(const std::string &msg) : Error("IO: " + msg, Severity::ERROR, "io") {}

    IOError(const std::string &operation, const std::string &path, const std::string &reason)
        : Error("IO: " + operation + " '" + path + "': " + reason, Severity::ERROR, "io"),
          path_(path) {}

    [[nodiscard]] const std::string &path() const { return path_; }

  private:
    std::string path_;
};

// =============================================================================
// Not Implemented
// =============================================================================

/**
 * @brief Stub for unimplemented features
 */
class NotImplementedError : public Error {
  public:
    explicit NotImplementedError(const std::string &feature)
        : Error("Not implemented: " + feature, Severity::ERROR, "system") {}
};

// =============================================================================
// Legacy Types (for backward compatibility during transition)
// =============================================================================

// These wrapper classes allow existing code to compile while migrating.
// They inherit from the consolidated classes for catch compatibility.
// TODO: Remove after all sites are updated to use base types directly

// Lifecycle phase wrappers
class ProvisionError : public LifecycleError {
  public:
    explicit ProvisionError(const std::string &msg)
        : LifecycleError(LifecyclePhase::Provision, msg) {}
};

class StageError : public LifecycleError {
  public:
    explicit StageError(const std::string &msg) : LifecycleError(LifecyclePhase::Stage, msg) {}
};

class StepError : public LifecycleError {
  public:
    explicit StepError(const std::string &msg) : LifecycleError(LifecyclePhase::Step, msg) {}
};

// Wiring/routing alias
using RoutingError = WiringError;

// Config alias
using ConfigurationError = ConfigError;

// Integration alias
using IntegratorError = IntegrationError;

// Signal error subtypes - must be classes for EXPECT_THROW compatibility
class TypeMismatchError : public SignalError {
  public:
    TypeMismatchError(const std::string &name, const std::string &expected,
                      const std::string &actual)
        : SignalError(SignalErrorKind::TypeMismatch, name,
                      "expected " + expected + ", got " + actual) {}
};

class DuplicateSignalError : public SignalError {
  public:
    DuplicateSignalError(const std::string &name, const std::string &existing,
                         const std::string &new_owner)
        : SignalError(SignalErrorKind::Duplicate, name,
                      "already owned by '" + existing + "', conflict from '" + new_owner + "'") {}
};

class SignalNotFoundError : public SignalError {
  public:
    explicit SignalNotFoundError(const std::string &name)
        : SignalError(SignalErrorKind::NotFound, name) {}
};

class UnwiredInputError : public SignalError {
  public:
    explicit UnwiredInputError(const std::string &name)
        : SignalError(SignalErrorKind::Unwired, name) {}
};

// State error subtypes
class StateSizeMismatchError : public StateError {
  public:
    StateSizeMismatchError(std::size_t expected, std::size_t actual)
        : StateError(expected, actual) {}
};

// Integration error subtypes
class StepSizeTooSmallError : public IntegrationError {
  public:
    explicit StepSizeTooSmallError(double min_dt)
        : IntegrationError("Step size below minimum: " + std::to_string(min_dt)) {}
};

// =============================================================================
// Throw-and-Log Helpers
// =============================================================================

/**
 * @brief Helper to throw an error and optionally log it
 *
 * Usage with LogService integration (in files that include LogService.hpp):
 * @code
 * ICARUS_THROW(SignalError::NotFound("mySignal"), time, "MyComponent");
 * @endcode
 *
 * The error will be logged before being thrown if ICARUS_LOG_ERRORS is defined.
 */
template <typename E>
[[noreturn]] void ThrowError(E &&error, [[maybe_unused]] double time = 0.0,
                             [[maybe_unused]] const std::string &component = "") {
    // Note: Logging integration happens in ErrorHandler when catching,
    // or can be done explicitly before throwing via LogService.
    throw std::forward<E>(error);
}

} // namespace icarus

// =============================================================================
// Error Throwing Macros
// =============================================================================

// NOLINTBEGIN(cppcoreguidelines-macro-usage)

/**
 * @brief Throw an error (simple version, no logging)
 */
#define ICARUS_THROW(error) throw(error)

/**
 * @brief Throw an error with context (for future logging integration)
 * @param error The error to throw
 * @param time Simulation time
 * @param component Component name for logging context
 */
#define ICARUS_THROW_CTX(error, time, component) ::icarus::ThrowError((error), (time), (component))

// NOLINTEND(cppcoreguidelines-macro-usage)
