#pragma once

/**
 * @file ErrorHandler.hpp
 * @brief Error handling integration with logging
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 * Bridges logging, Error.hpp exceptions, and simulation lifecycle.
 */

#include <icarus/core/Error.hpp>
#include <icarus/io/LogService.hpp>

#include <cstdlib>
#include <functional>
#include <map>
#include <optional>

namespace icarus {

/**
 * @brief Error handling policy
 */
enum class ErrorPolicy {
    Continue, ///< Log and continue (for warnings)
    Pause,    ///< Pause simulation, allow resume
    Abort,    ///< Clean shutdown with debrief
    Crash     ///< Immediate termination (for fatal errors)
};

/**
 * @brief Error handler callback signature
 */
using ErrorCallback = std::function<ErrorPolicy(const SimulationError &)>;

/**
 * @brief Central error handler integrating logging and exceptions
 *
 * Bridges the gap between:
 * - LogService (buffered logging)
 * - Error.hpp exceptions
 * - Simulation lifecycle (pause/abort/crash)
 */
class ErrorHandler {
  public:
    explicit ErrorHandler(LogService &log_service) : log_service_(log_service) {
        // Set default policies
        policies_[Severity::INFO] = ErrorPolicy::Continue;
        policies_[Severity::WARNING] = ErrorPolicy::Continue;
        policies_[Severity::ERROR] = ErrorPolicy::Continue;
        policies_[Severity::FATAL] = ErrorPolicy::Crash;
    }

    // === Policy Configuration ===

    /// Set default policy for each severity level
    void SetPolicy(Severity severity, ErrorPolicy policy) { policies_[severity] = policy; }

    /// Set callback for error handling (can override default policy)
    void SetCallback(ErrorCallback callback) { callback_ = std::move(callback); }

    /// Set maximum errors before automatic abort
    void SetMaxErrors(std::size_t max_errors) { max_errors_ = max_errors; }

    // === Error Reporting ===

    /// Report an error (from exception or direct call)
    /// Returns the policy that should be applied
    ErrorPolicy Report(const SimulationError &error) {
        // Log the error
        LogLevel level = SeverityToLogLevel(error.severity);
        log_service_.Log(level, error.time, "[" + error.component + "] " + error.message);

        // Track counts
        ++error_counts_[error.severity];

        // Store fatal error
        if (error.severity == Severity::FATAL) {
            fatal_error_ = error;
        }

        // Get policy
        ErrorPolicy policy = policies_[error.severity];

        // Allow callback to override
        if (callback_) {
            policy = callback_(error);
        }

        // Check max errors
        if (GetTotalErrorCount() >= max_errors_) {
            policy = ErrorPolicy::Abort;
        }

        return policy;
    }

    /// Report an exception, extracting info automatically
    ErrorPolicy Report(const Error &exception, double sim_time = 0.0,
                       const std::string &component = "") {
        return Report(exception.toSimulationError(sim_time, component));
    }

    /// Report a fatal error and trigger crash
    [[noreturn]] void ReportFatal(const std::string &message, double sim_time = 0.0) {
        SimulationError error;
        error.severity = Severity::FATAL;
        error.message = message;
        error.component = "system";
        error.time = sim_time;
        Report(error);

        // Force flush logs before terminating
        log_service_.FlushAndClear();

        std::abort();
    }

    // === Query ===

    /// Get count of errors at each severity
    [[nodiscard]] std::size_t GetErrorCount(Severity severity) const {
        auto it = error_counts_.find(severity);
        return it != error_counts_.end() ? it->second : 0;
    }

    /// Get total error count (WARNING and above)
    [[nodiscard]] std::size_t GetTotalErrorCount() const {
        std::size_t total = 0;
        for (const auto &[severity, count] : error_counts_) {
            if (severity >= Severity::WARNING) {
                total += count;
            }
        }
        return total;
    }

    /// Check if simulation should abort
    [[nodiscard]] bool ShouldAbort() const {
        return fatal_error_.has_value() || GetTotalErrorCount() >= max_errors_;
    }

    /// Get the first fatal error (if any)
    [[nodiscard]] std::optional<SimulationError> GetFatalError() const { return fatal_error_; }

    // === Integration with Simulator ===

    /// Check buffered logs for errors, apply policies.
    /// Called by Simulator after Flush().
    /// @return true if simulation should abort (fatal error or max errors exceeded)
    [[nodiscard]] bool ProcessBufferedErrors() {
        // Errors are already counted in Log(), so just check thresholds
        return ShouldAbort();
    }

    /// Clear error counts (for new run)
    void Reset() {
        error_counts_.clear();
        fatal_error_.reset();
    }

  private:
    LogService &log_service_;
    ErrorCallback callback_;

    std::map<Severity, ErrorPolicy> policies_;
    std::map<Severity, std::size_t> error_counts_;
    std::optional<SimulationError> fatal_error_;
    std::size_t max_errors_ = 100; ///< Abort after this many errors

    [[nodiscard]] static LogLevel SeverityToLogLevel(Severity severity) {
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
};

/**
 * @brief RAII wrapper for try/catch with automatic error reporting
 */
class ErrorScope {
  public:
    ErrorScope(ErrorHandler &handler, double sim_time, std::string component)
        : handler_(handler), sim_time_(sim_time), component_(std::move(component)) {}

    ~ErrorScope() = default;

    /// Execute a callable, catching and reporting exceptions
    template <typename F> auto Execute(F &&func) -> decltype(func()) {
        try {
            return func();
        } catch (const Error &e) {
            handler_.Report(e, sim_time_, component_);
            throw; // Re-throw after reporting
        }
    }

  private:
    ErrorHandler &handler_;
    double sim_time_;
    std::string component_;
};

} // namespace icarus
