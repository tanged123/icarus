#pragma once

/**
 * @file ValidationResult.hpp
 * @brief Structured validation result for warmstart and configuration validation
 *
 * Phase 6.3: Warmstart - State Restoration
 */

#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Severity level for validation issues
 */
enum class ValidationSeverity {
    Error,   ///< Fatal - operation cannot proceed
    Warning, ///< Proceed with caution
    Info     ///< Informational note
};

/**
 * @brief Single validation issue
 */
struct ValidationIssue {
    ValidationSeverity severity;
    std::string message;
    std::string context; ///< Additional context (e.g., signal name)

    ValidationIssue(ValidationSeverity sev, std::string msg, std::string ctx = "")
        : severity(sev), message(std::move(msg)), context(std::move(ctx)) {}

    /// Check if this issue is an error
    [[nodiscard]] bool IsError() const { return severity == ValidationSeverity::Error; }
};

/**
 * @brief Structured validation result with errors and warnings
 *
 * Used for recording validation before warmstart and other validation checks.
 *
 * Example:
 * @code
 * ValidationResult result = sim->ValidateRecording("flight.h5");
 * if (!result.IsValid()) {
 *     for (const auto& error : result.GetErrors()) {
 *         std::cerr << "Error: " << error << std::endl;
 *     }
 *     throw WarmstartError("Recording incompatible");
 * }
 * for (const auto& warning : result.GetWarnings()) {
 *     std::cerr << "Warning: " << warning << std::endl;
 * }
 * @endcode
 */
class ValidationResult {
  public:
    ValidationResult() = default;

    /// Check if validation passed (no errors)
    [[nodiscard]] bool IsValid() const {
        for (const auto &issue : issues_) {
            if (issue.IsError()) {
                return false;
            }
        }
        return true;
    }

    /// Get all error messages
    [[nodiscard]] std::vector<std::string> GetErrors() const {
        std::vector<std::string> errors;
        for (const auto &issue : issues_) {
            if (issue.severity == ValidationSeverity::Error) {
                errors.push_back(FormatIssue(issue));
            }
        }
        return errors;
    }

    /// Get all warning messages
    [[nodiscard]] std::vector<std::string> GetWarnings() const {
        std::vector<std::string> warnings;
        for (const auto &issue : issues_) {
            if (issue.severity == ValidationSeverity::Warning) {
                warnings.push_back(FormatIssue(issue));
            }
        }
        return warnings;
    }

    /// Get all info messages
    [[nodiscard]] std::vector<std::string> GetInfos() const {
        std::vector<std::string> infos;
        for (const auto &issue : issues_) {
            if (issue.severity == ValidationSeverity::Info) {
                infos.push_back(FormatIssue(issue));
            }
        }
        return infos;
    }

    /// Get all issues
    [[nodiscard]] const std::vector<ValidationIssue> &GetIssues() const { return issues_; }

    /// Add an error
    void AddError(const std::string &message, const std::string &context = "") {
        issues_.emplace_back(ValidationSeverity::Error, message, context);
    }

    /// Add a warning
    void AddWarning(const std::string &message, const std::string &context = "") {
        issues_.emplace_back(ValidationSeverity::Warning, message, context);
    }

    /// Add an info message
    void AddInfo(const std::string &message, const std::string &context = "") {
        issues_.emplace_back(ValidationSeverity::Info, message, context);
    }

    /// Check if there are any issues at all
    [[nodiscard]] bool HasIssues() const { return !issues_.empty(); }

    /// Get error count
    [[nodiscard]] size_t ErrorCount() const {
        size_t count = 0;
        for (const auto &issue : issues_) {
            if (issue.IsError()) {
                ++count;
            }
        }
        return count;
    }

    /// Get warning count
    [[nodiscard]] size_t WarningCount() const {
        size_t count = 0;
        for (const auto &issue : issues_) {
            if (issue.severity == ValidationSeverity::Warning) {
                ++count;
            }
        }
        return count;
    }

  private:
    std::vector<ValidationIssue> issues_;

    [[nodiscard]] static std::string FormatIssue(const ValidationIssue &issue) {
        if (issue.context.empty()) {
            return issue.message;
        }
        return issue.message + " [" + issue.context + "]";
    }
};

} // namespace icarus
