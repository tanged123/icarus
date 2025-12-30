#pragma once

/**
 * @file LogEntry.hpp
 * @brief Log context and entry structures
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 * Captures full context for each log message: entity, component, timestamp, level.
 */

#include <icarus/core/Types.hpp>
#include <icarus/io/Console.hpp>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace icarus {

// =============================================================================
// LogContext
// =============================================================================

/**
 * @brief Immutable log context - set by component/entity during execution
 *
 * The LogContext is thread-local and automatically captures the current
 * entity and component name. Components don't need to pass context explicitly.
 */
struct LogContext {
    std::string entity;    ///< Entity name (e.g., "falcon9_1")
    std::string component; ///< Component name (e.g., "merlinEngine2")
    std::string type;      ///< Component type (e.g., "LiquidEngine")

    /// Full path: "entity.component" or just "component" if no entity
    [[nodiscard]] std::string FullPath() const { return MakeFullPath(entity, component); }

    /// Check if context is set
    [[nodiscard]] bool IsSet() const { return !component.empty(); }
};

// =============================================================================
// LogEntry
// =============================================================================

/**
 * @brief A single log entry with full context
 *
 * Immutable after creation. Collected into LogBuffer during hot loop.
 */
struct LogEntry {
    LogLevel level;      ///< Severity level
    double sim_time;     ///< Simulation time when logged
    std::string message; ///< Log message
    LogContext context;  ///< Entity/component context

    /// Wall clock time for ordering logs from concurrent sources
    std::chrono::steady_clock::time_point wall_time;

    /// Create a log entry with current context
    static LogEntry Create(LogLevel level, double sim_time, std::string_view message,
                           const LogContext &ctx) {
        LogEntry entry;
        entry.level = level;
        entry.sim_time = sim_time;
        entry.message = std::string(message);
        entry.context = ctx;
        entry.wall_time = std::chrono::steady_clock::now();
        return entry;
    }

    /// Format for output: "[sim_time] [LEVEL] [entity.component] message"
    [[nodiscard]] std::string Format(bool include_context = true) const {
        std::ostringstream oss;
        oss << "[" << std::fixed << std::setprecision(3) << sim_time << "] ";
        oss << "[" << GetLevelString(level) << "] ";

        if (include_context && context.IsSet()) {
            oss << "[" << context.FullPath() << "] ";
        }

        oss << message;
        return oss.str();
    }

    /// Format with colors (for terminal)
    [[nodiscard]] std::string FormatColored(const Console &console) const {
        std::ostringstream oss;

        // Time in dim
        oss << console.Colorize("[", AnsiColor::Dim);
        oss << std::fixed << std::setprecision(3) << sim_time;
        oss << console.Colorize("]", AnsiColor::Dim) << " ";

        // Level with appropriate color
        oss << console.Colorize("[" + std::string(GetLevelString(level)) + "]",
                                GetLevelColor(level))
            << " ";

        // Context in cyan
        if (context.IsSet()) {
            oss << console.Colorize("[" + context.FullPath() + "]", AnsiColor::Cyan) << " ";
        }

        oss << message;
        return oss.str();
    }

  private:
    [[nodiscard]] static const char *GetLevelString(LogLevel level) {
        switch (level) {
        case LogLevel::Trace:
            return "TRC";
        case LogLevel::Debug:
            return "DBG";
        case LogLevel::Info:
            return "INF";
        case LogLevel::Event:
            return "EVT";
        case LogLevel::Warning:
            return "WRN";
        case LogLevel::Error:
            return "ERR";
        case LogLevel::Fatal:
            return "FTL";
        }
        return "???";
    }

    [[nodiscard]] static const char *GetLevelColor(LogLevel level) {
        switch (level) {
        case LogLevel::Trace:
            return AnsiColor::Gray;
        case LogLevel::Debug:
            return AnsiColor::Cyan;
        case LogLevel::Info:
            return AnsiColor::White;
        case LogLevel::Event:
            return AnsiColor::Green;
        case LogLevel::Warning:
            return AnsiColor::Yellow;
        case LogLevel::Error:
            return AnsiColor::Red;
        case LogLevel::Fatal:
            return AnsiColor::BgRed;
        }
        return AnsiColor::White;
    }
};

// =============================================================================
// LogContextManager
// =============================================================================

/**
 * @brief Thread-local log context manager
 *
 * The Simulator sets this before calling component methods.
 * Components can query GetCurrentContext() implicitly.
 */
class LogContextManager {
  public:
    /// Set context for current thread (called by Simulator before Step)
    static void SetContext(const LogContext &ctx) { current_context_ = ctx; }

    /// Clear context (called by Simulator after Step)
    static void ClearContext() { current_context_ = LogContext{}; }

    /// Get current context (called internally by LogBuffer::Log)
    [[nodiscard]] static const LogContext &GetContext() { return current_context_; }

    /**
     * @brief RAII guard for automatic context management
     */
    class ScopedContext {
      public:
        ScopedContext(const std::string &entity, const std::string &component,
                      const std::string &type = "")
            : previous_(current_context_) {
            current_context_.entity = entity;
            current_context_.component = component;
            current_context_.type = type;
        }

        ~ScopedContext() { current_context_ = previous_; }

        // Non-copyable, non-movable
        ScopedContext(const ScopedContext &) = delete;
        ScopedContext &operator=(const ScopedContext &) = delete;
        ScopedContext(ScopedContext &&) = delete;
        ScopedContext &operator=(ScopedContext &&) = delete;

      private:
        LogContext previous_;
    };

  private:
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static inline thread_local LogContext current_context_;
};

} // namespace icarus
