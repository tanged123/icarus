#pragma once

/**
 * @file Error.hpp
 * @brief Error handling and exception hierarchy for Icarus
 *
 * Provides structured error handling with lifecycle-aware exceptions.
 */

#include <cstdint>
#include <stdexcept>
#include <string>

namespace icarus {

// =============================================================================
// Error Severity
// =============================================================================

/**
 * @brief Severity levels for simulation errors
 */
enum class Severity : uint8_t {
    INFO,    ///< Informational (logged, no action)
    WARNING, ///< Warning (may trigger graceful degradation)
    ERROR,   ///< Error (simulation may continue with fallback)
    FATAL    ///< Fatal (simulation must stop)
};

// =============================================================================
// Simulation Error (for OnError hook)
// =============================================================================

/**
 * @brief Structured error information passed to component OnError hooks
 */
struct SimulationError {
    Severity severity;     ///< Error severity level
    std::string message;   ///< Human-readable error message
    std::string component; ///< Component that triggered the error
    double time = 0.0;     ///< Simulation time when error occurred
};

// =============================================================================
// Exception Hierarchy
// =============================================================================

/**
 * @brief Base class for all Icarus exceptions
 *
 * Derives from std::runtime_error for catch compatibility.
 */
class Error : public std::runtime_error {
  public:
    explicit Error(const std::string &msg) : std::runtime_error("[icarus] " + msg) {}
};

/**
 * @brief Error during component Provision phase
 *
 * Thrown when signal registration, memory allocation, or config parsing fails.
 */
class ProvisionError : public Error {
  public:
    explicit ProvisionError(const std::string &msg) : Error("Provision: " + msg) {}
};

/**
 * @brief Error during component Stage phase
 *
 * Thrown when input wiring, IC application, or trim solving fails.
 */
class StageError : public Error {
  public:
    explicit StageError(const std::string &msg) : Error("Stage: " + msg) {}
};

/**
 * @brief Error during component Step phase
 *
 * Thrown when dynamics computation fails (should be rare in Step).
 */
class StepError : public Error {
  public:
    explicit StepError(const std::string &msg) : Error("Step: " + msg) {}
};

/**
 * @brief Signal-related error
 *
 * Thrown for signal registration conflicts, missing signals, type mismatches.
 */
class SignalError : public Error {
  public:
    explicit SignalError(const std::string &msg) : Error("Signal: " + msg) {}
};

/**
 * @brief Type mismatch when resolving a signal
 *
 * Thrown when a signal is resolved with a different type than it was registered.
 */
class TypeMismatchError : public SignalError {
  public:
    TypeMismatchError(const std::string &signal_name, const std::string &expected_type,
                      const std::string &actual_type)
        : SignalError("Type mismatch for '" + signal_name + "': expected " + expected_type +
                      ", got " + actual_type) {}
};

/**
 * @brief Duplicate signal registration error
 *
 * Thrown when attempting to register a signal with a name that already exists.
 */
class DuplicateSignalError : public SignalError {
  public:
    DuplicateSignalError(const std::string &signal_name, const std::string &existing_owner,
                         const std::string &new_owner)
        : SignalError("Duplicate signal '" + signal_name + "': already registered by '" +
                      existing_owner + "', conflicting registration from '" + new_owner + "'") {}
};

/**
 * @brief Signal not found error
 *
 * Thrown when attempting to resolve a signal that does not exist.
 */
class SignalNotFoundError : public SignalError {
  public:
    explicit SignalNotFoundError(const std::string &signal_name)
        : SignalError("Signal not found: '" + signal_name + "'") {}
};

/**
 * @brief Configuration error
 *
 * Thrown when config parsing or validation fails.
 */
class ConfigError : public Error {
  public:
    explicit ConfigError(const std::string &msg) : Error("Config: " + msg) {}
};

/**
 * @brief Integration/ODE solver error
 *
 * Thrown when integrator fails to advance state.
 */
class IntegrationError : public Error {
  public:
    explicit IntegrationError(const std::string &msg) : Error("Integration: " + msg) {}
};

/**
 * @brief State management error
 *
 * Thrown for state vector allocation, scatter/gather, or bounds issues.
 */
class StateError : public Error {
  public:
    explicit StateError(const std::string &msg) : Error("State: " + msg) {}
};

/**
 * @brief Base class for integrator errors
 */
class IntegratorError : public Error {
  public:
    explicit IntegratorError(const std::string &msg) : Error("Integrator: " + msg) {}
};

/**
 * @brief Thrown when step size becomes too small
 */
class StepSizeTooSmallError : public IntegratorError {
  public:
    explicit StepSizeTooSmallError(double min_dt)
        : IntegratorError("Step size below minimum: " + std::to_string(min_dt)), min_dt_(min_dt) {}

    [[nodiscard]] double min_dt() const { return min_dt_; }

  private:
    double min_dt_;
};

/**
 * @brief Thrown when integration fails
 *
 * Provides time, step size, and reason for the failure.
 */
template <typename Scalar> class IntegrationFailedError : public IntegratorError {
  public:
    IntegrationFailedError(Scalar t, Scalar dt, const std::string &reason)
        : IntegratorError("Integration failed at t=" + std::to_string(static_cast<double>(t)) +
                          ", dt=" + std::to_string(static_cast<double>(dt)) + ": " + reason),
          t_(t), dt_(dt), reason_(reason) {}

    [[nodiscard]] Scalar t() const { return t_; }
    [[nodiscard]] Scalar dt() const { return dt_; }
    [[nodiscard]] const std::string &reason() const { return reason_; }

  private:
    Scalar t_;
    Scalar dt_;
    std::string reason_;
};

/**
 * @brief State size mismatch error
 *
 * Thrown when a component's state size doesn't match the expected size
 * during state binding.
 */
class StateSizeMismatchError : public StateError {
  public:
    StateSizeMismatchError(std::size_t expected, std::size_t actual)
        : StateError("Size mismatch: expected " + std::to_string(expected) + ", got " +
                     std::to_string(actual)),
          expected_(expected), actual_(actual) {}

    [[nodiscard]] std::size_t expected() const { return expected_; }
    [[nodiscard]] std::size_t actual() const { return actual_; }

  private:
    std::size_t expected_;
    std::size_t actual_;
};

/**
 * @brief Lifecycle ordering error
 *
 * Thrown when lifecycle methods are called in the wrong order.
 */
class LifecycleError : public Error {
  public:
    explicit LifecycleError(const std::string &msg) : Error("Lifecycle: " + msg) {}
};

/**
 * @brief Unwired input error
 *
 * Thrown when accessing an input that has not been wired to a source.
 */
class UnwiredInputError : public SignalError {
  public:
    explicit UnwiredInputError(const std::string &input_name)
        : SignalError("Input not wired: '" + input_name + "'"), input_name_(input_name) {}

    [[nodiscard]] const std::string &input_name() const { return input_name_; }

  private:
    std::string input_name_;
};

/**
 * @brief Wiring configuration error
 *
 * Thrown when wiring configuration is invalid or incomplete.
 */
class WiringError : public Error {
  public:
    explicit WiringError(const std::string &msg) : Error("Wiring: " + msg) {}
};

/**
 * @brief Input/Output error
 *
 * Thrown when file operations fail.
 */
class IOError : public Error {
  public:
    explicit IOError(const std::string &msg) : Error("IO: " + msg) {}
};

/**
 * @brief Feature not yet implemented
 *
 * Thrown when a feature is stubbed for future implementation.
 */
class NotImplementedError : public Error {
  public:
    explicit NotImplementedError(const std::string &msg) : Error("Not implemented: " + msg) {}
};

/**
 * @brief Configuration error
 *
 * Thrown when builder or configuration setup is invalid.
 */
class ConfigurationError : public Error {
  public:
    explicit ConfigurationError(const std::string &msg) : Error("Configuration: " + msg) {}
};

} // namespace icarus
