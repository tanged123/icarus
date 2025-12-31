#pragma once

/**
 * @file ErrorLogging.hpp
 * @brief Integration between Error types and LogService
 *
 * Include this header to get automatic logging when throwing errors.
 * This file bridges Error.hpp and LogService.hpp.
 */

#include <icarus/core/Error.hpp>
#include <icarus/io/log/LogService.hpp>

namespace icarus {

/**
 * @brief Convert error severity to log level
 */
inline LogLevel SeverityToLogLevel(Severity severity) {
    switch (severity) {
    case Severity::INFO:
        return LogLevel::Info;
    case Severity::WARNING:
        return LogLevel::Warning;
    case Severity::ERROR:
        return LogLevel::Error;
    case Severity::FATAL:
        return LogLevel::Fatal;
    }
    return LogLevel::Error;
}

/**
 * @brief Log an error to the global LogService
 */
inline void LogError(const Error &error, double time = 0.0, const std::string &component = "") {
    auto sim_error = error.toSimulationError(time, component);
    LogLevel level = SeverityToLogLevel(sim_error.severity);

    // Use component as context if provided
    if (!component.empty()) {
        LogContext ctx;
        ctx.component = component;
        GetLogService().Log(level, time, sim_error.message, ctx);
    } else {
        GetLogService().Log(level, time, sim_error.message);
    }
}

/**
 * @brief Throw an error after logging it
 *
 * Usage:
 * @code
 * ThrowAndLog(SignalError::NotFound("mySignal"), 0.0, "MyComponent");
 * @endcode
 */
template <typename E>
[[noreturn]] void ThrowAndLog(E &&error, double time = 0.0, const std::string &component = "") {
    LogError(error, time, component);
    throw std::forward<E>(error);
}

} // namespace icarus

// =============================================================================
// Logging-Enabled Throw Macros
// =============================================================================

// NOLINTBEGIN(cppcoreguidelines-macro-usage)

/**
 * @brief Throw an error after logging it to global LogService
 * @param error The error to throw
 */
#define ICARUS_THROW_LOG(error) ::icarus::ThrowAndLog((error), 0.0, "")

/**
 * @brief Throw an error with full context, logging before throw
 * @param error The error to throw
 * @param time Simulation time
 * @param component Component name for logging context
 */
#define ICARUS_THROW_LOG_CTX(error, time, component)                                               \
    ::icarus::ThrowAndLog((error), (time), (component))

// NOLINTEND(cppcoreguidelines-macro-usage)
