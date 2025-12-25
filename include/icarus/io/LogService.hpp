#pragma once

/**
 * @file LogService.hpp
 * @brief Unified logging service for Icarus
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 * Provides both immediate and buffered logging modes.
 */

#include <icarus/io/Console.hpp>
#include <icarus/io/LogEntry.hpp>

#include <algorithm>
#include <functional>
#include <mutex>
#include <vector>

namespace icarus {

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
