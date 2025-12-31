#pragma once

/**
 * @file LogService.hpp
 * @brief Unified logging service for Icarus
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 * Provides both immediate and buffered logging modes.
 *
 * Consolidates: LogConfig, LogEntry, LogContextManager, LogService
 */

#include <icarus/core/CoreTypes.hpp>
#include <icarus/io/Console.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace icarus {

// =============================================================================
// LogConfig
// =============================================================================

/**
 * @brief Logging configuration
 */
struct LogConfig {
    // Console output
    LogLevel console_level = LogLevel::Info;
    bool progress_enabled = true;

    // File output
    bool file_enabled = false;
    std::string file_path;
    LogLevel file_level = LogLevel::Debug;
    bool file_rotate = false;
    std::size_t max_file_size_mb = 100;

    // Features
    bool profiling_enabled = false;
    bool quiet_mode = false; ///< Suppress all but errors

    // Telemetry recording
    bool telemetry_enabled = false;
    std::string telemetry_path;
    std::vector<std::string> telemetry_signals; ///< Which signals to record

    /// Create default config
    [[nodiscard]] static LogConfig Default() {
        LogConfig config;
        return config;
    }

    /// Create quiet config (errors only)
    [[nodiscard]] static LogConfig Quiet() {
        LogConfig config;
        config.console_level = LogLevel::Error;
        config.progress_enabled = false;
        config.quiet_mode = true;
        return config;
    }

    /// Create verbose config (all output)
    [[nodiscard]] static LogConfig Verbose() {
        LogConfig config;
        config.console_level = LogLevel::Trace;
        config.file_enabled = true;
        config.file_level = LogLevel::Trace;
        return config;
    }

    /// Create profiling config
    [[nodiscard]] static LogConfig WithProfiling() {
        LogConfig config;
        config.profiling_enabled = true;
        return config;
    }
};

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

// =============================================================================
// LogService
// =============================================================================

/**
 * @brief Unified logging service for Icarus
 *
 * ALL logging goes through this service. It operates in two modes:
 *
 * 1. **Immediate mode** (Provision/Stage/Shutdown):
 *    Logs are flushed to sinks immediately after each call.
 *    Use for lifecycle events where you want instant feedback.
 *
 * 2. **Buffered mode** (Run/Step):
 *    Logs are collected into a buffer, flushed at end of step.
 *    Use during hot loop to eliminate I/O latency.
 *
 * The Simulator controls the mode based on lifecycle phase.
 */
class LogService {
  public:
    /// Sink callback type: receives batch of entries to output
    using Sink = std::function<void(const std::vector<LogEntry> &)>;

    LogService() = default;

    // === Mode Control ===

    /// Enable immediate flush after each log (for lifecycle phases)
    void SetImmediateMode(bool immediate) { immediate_mode_ = immediate; }
    [[nodiscard]] bool IsImmediateMode() const { return immediate_mode_; }

    /**
     * @brief RAII guard for buffered mode
     *
     * Switches to buffered mode on construction, restores previous mode
     * and flushes on destruction.
     */
    class BufferedScope {
      public:
        explicit BufferedScope(LogService &service)
            : service_(service), previous_mode_(service.immediate_mode_) {
            service_.SetImmediateMode(false);
        }

        ~BufferedScope() {
            service_.FlushAndClear();
            service_.SetImmediateMode(previous_mode_);
        }

        // Non-copyable, non-movable
        BufferedScope(const BufferedScope &) = delete;
        BufferedScope &operator=(const BufferedScope &) = delete;
        BufferedScope(BufferedScope &&) = delete;
        BufferedScope &operator=(BufferedScope &&) = delete;

      private:
        LogService &service_;
        bool previous_mode_;
    };

    // === Configuration ===

    /// Set minimum level (below this = dropped)
    void SetMinLevel(LogLevel level) { min_level_ = level; }
    [[nodiscard]] LogLevel GetMinLevel() const { return min_level_; }

    /// Set buffer capacity (pre-allocate for performance)
    void Reserve(std::size_t capacity) { entries_.reserve(capacity); }

    /// Add an output sink
    void AddSink(Sink sink) { sinks_.emplace_back(std::move(sink), LogLevel::Trace); }

    /// Add a sink that only receives entries at or above a level
    void AddSink(Sink sink, LogLevel min_level) { sinks_.emplace_back(std::move(sink), min_level); }

    /// Clear all sinks
    void ClearSinks() { sinks_.clear(); }

    // === Logging API ===

    /// Log a message (uses current thread-local context)
    void Log(LogLevel level, double sim_time, std::string_view message) {
        Log(level, sim_time, message, LogContextManager::GetContext());
    }

    /// Log with explicit context (bypasses thread-local)
    void Log(LogLevel level, double sim_time, std::string_view message, const LogContext &ctx) {
        if (level < min_level_) {
            return;
        }

        auto entry = LogEntry::Create(level, sim_time, message, ctx);

        std::lock_guard<std::mutex> lock(mutex_);

        // Track error counts (inside lock for thread safety)
        if (level == LogLevel::Error) {
            ++error_count_;
        } else if (level == LogLevel::Fatal) {
            ++fatal_count_;
        }

        entries_.push_back(std::move(entry));

        if (immediate_mode_) {
            FlushEntry(entries_.back());
        }
    }

    // Convenience methods (use thread-local context)
    void Trace(double t, std::string_view msg) { Log(LogLevel::Trace, t, msg); }
    void Debug(double t, std::string_view msg) { Log(LogLevel::Debug, t, msg); }
    void Info(double t, std::string_view msg) { Log(LogLevel::Info, t, msg); }
    void Event(double t, std::string_view msg) { Log(LogLevel::Event, t, msg); }
    void Warning(double t, std::string_view msg) { Log(LogLevel::Warning, t, msg); }
    void Error(double t, std::string_view msg) { Log(LogLevel::Error, t, msg); }
    void Fatal(double t, std::string_view msg) { Log(LogLevel::Fatal, t, msg); }

    // === Flush Control ===

    /// Flush buffer to all sinks
    void Flush(bool sort_by_time = false) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (entries_.empty()) {
            return;
        }

        if (sort_by_time) {
            std::sort(entries_.begin(), entries_.end(), [](const LogEntry &a, const LogEntry &b) {
                return a.wall_time < b.wall_time;
            });
        }

        for (const auto &[sink, min_level] : sinks_) {
            // Filter entries by sink's minimum level
            std::vector<LogEntry> filtered;
            filtered.reserve(entries_.size());
            for (const auto &entry : entries_) {
                if (entry.level >= min_level) {
                    filtered.push_back(entry);
                }
            }
            if (!filtered.empty()) {
                sink(filtered);
            }
        }
    }

    /// Flush and clear buffer
    void FlushAndClear(bool sort_by_time = false) {
        Flush(sort_by_time);
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.clear();
    }

    /// Clear buffer without flushing (discard pending logs)
    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.clear();
    }

    // === Query API ===

    [[nodiscard]] std::size_t PendingCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return entries_.size();
    }

    [[nodiscard]] bool HasPending() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !entries_.empty();
    }

    [[nodiscard]] std::vector<LogEntry> GetPending() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return entries_; // Return copy for thread safety
    }

    /// Check if any errors were logged (in buffer or flushed)
    [[nodiscard]] bool HasErrors() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_count_ > 0;
    }

    [[nodiscard]] bool HasFatalErrors() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return fatal_count_ > 0;
    }

    [[nodiscard]] std::size_t ErrorCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_count_;
    }

    [[nodiscard]] std::size_t FatalCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return fatal_count_;
    }

    /// Reset error counts (call at start of new run)
    void ResetErrorCounts() {
        std::lock_guard<std::mutex> lock(mutex_);
        error_count_ = 0;
        fatal_count_ = 0;
    }

    // === Filtering (for query, not output) ===

    [[nodiscard]] std::vector<LogEntry> GetEntriesAtLevel(LogLevel level) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<LogEntry> result;
        for (const auto &entry : entries_) {
            if (entry.level == level) {
                result.push_back(entry);
            }
        }
        return result;
    }

    [[nodiscard]] std::vector<LogEntry> GetEntriesForEntity(std::string_view entity) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<LogEntry> result;
        for (const auto &entry : entries_) {
            if (entry.context.entity == entity) {
                result.push_back(entry);
            }
        }
        return result;
    }

    [[nodiscard]] std::vector<LogEntry> GetEntriesForComponent(std::string_view path) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<LogEntry> result;
        for (const auto &entry : entries_) {
            if (entry.context.FullPath() == path) {
                result.push_back(entry);
            }
        }
        return result;
    }

  private:
    std::vector<LogEntry> entries_;
    std::vector<std::pair<Sink, LogLevel>> sinks_; ///< sink + min level
    LogLevel min_level_ = LogLevel::Info;
    bool immediate_mode_ = true; ///< Default: immediate for safety

    std::size_t error_count_ = 0;
    std::size_t fatal_count_ = 0;

    mutable std::mutex mutex_;

    void FlushEntry(const LogEntry &entry) {
        for (const auto &[sink, min_level] : sinks_) {
            if (entry.level >= min_level) {
                sink({entry});
            }
        }
    }
};

/**
 * @brief Global log service singleton
 */
inline LogService &GetLogService() {
    static LogService instance;
    return instance;
}

} // namespace icarus

// =============================================================================
// Logging Macros
// =============================================================================

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define ICARUS_LOG_TRACE(sim_time, msg) ::icarus::GetLogService().Trace(sim_time, msg)

#define ICARUS_LOG_DEBUG(sim_time, msg) ::icarus::GetLogService().Debug(sim_time, msg)

#define ICARUS_LOG_INFO(sim_time, msg) ::icarus::GetLogService().Info(sim_time, msg)

#define ICARUS_LOG_EVENT(sim_time, msg) ::icarus::GetLogService().Event(sim_time, msg)

#define ICARUS_LOG_WARN(sim_time, msg) ::icarus::GetLogService().Warning(sim_time, msg)

#define ICARUS_LOG_ERROR(sim_time, msg) ::icarus::GetLogService().Error(sim_time, msg)

#define ICARUS_LOG_FATAL(sim_time, msg) ::icarus::GetLogService().Fatal(sim_time, msg)
// NOLINTEND(cppcoreguidelines-macro-usage)
