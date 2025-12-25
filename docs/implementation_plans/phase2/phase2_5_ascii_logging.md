# Phase 2.5: ASCII-Rich Logging and Data Dictionary Display

**Status:** Proposed
**Prerequisites:** Phase 2.4 (Component Interface System)
**Architecture Reference:** [15_services.md](../../architecture/15_services.md), [02_component_protocol.md](../../architecture/02_component_protocol.md)

---

## Overview

This phase implements a "Flight Recorder" style logging system with rich ASCII formatting for terminal and log file output. The system provides:

1. **Flight Manifest** — Formatted Data Dictionary displayed at end of Provision
2. **Mission Log** — Lifecycle logging with phases (Startup → Provision → Stage → Run → Shutdown)
3. **Mission Debrief** — Performance statistics and profiling at simulation end

The logging feels less like "debug dump" and more like a professional flight recorder.

---

## Goals

1. ASCII box-drawing tables for Data Dictionary display
2. Hierarchical tree visualization for component loading
3. Signal wiring graph visualization during Stage
4. ANSI color codes for severity levels (with fallback for non-terminal output)
5. Sparse event logging during Run (only events, not every step)
6. Performance profiling and real-time factor calculation
7. Configurable verbosity levels
8. Dual output: terminal (colorized) and file (plain ASCII)
9. **Buffered logging** during hot loop (collect → flush at step end)
10. **Automatic context capture** (entity.component, sim time, log level)
11. **Error integration** with `Error.hpp` for proper sim exit/crash handling

---

## Tasks

### 2.5.1 Console Abstraction Layer

**File:** `include/icarus/io/Console.hpp`

Provides terminal-aware output with ANSI escape code support and box-drawing characters.

```cpp
#pragma once

#include <cstdio>
#include <string>
#include <string_view>
#include <unistd.h>  // isatty

namespace icarus {

/**
 * @brief Log severity levels
 */
enum class LogLevel {
    Trace,    // Most verbose, internal debugging
    Debug,    // Debugging info
    Info,     // Normal operation
    Event,    // Simulation events (phase changes, etc.)
    Warning,  // Potential issues
    Error,    // Recoverable errors
    Fatal     // Unrecoverable errors
};

/**
 * @brief ANSI color codes
 */
struct AnsiColor {
    static constexpr const char* Reset   = "\033[0m";
    static constexpr const char* Bold    = "\033[1m";
    static constexpr const char* Dim     = "\033[2m";

    // Foreground colors
    static constexpr const char* Red     = "\033[31m";
    static constexpr const char* Green   = "\033[32m";
    static constexpr const char* Yellow  = "\033[33m";
    static constexpr const char* Blue    = "\033[34m";
    static constexpr const char* Magenta = "\033[35m";
    static constexpr const char* Cyan    = "\033[36m";
    static constexpr const char* White   = "\033[37m";
    static constexpr const char* Gray    = "\033[90m";

    // Background colors
    static constexpr const char* BgRed   = "\033[41m";
    static constexpr const char* BgGreen = "\033[42m";
};

/**
 * @brief Box-drawing characters (Unicode)
 */
struct BoxChars {
    // Single-line box drawing
    static constexpr const char* TopLeft     = "\u250C";  // ┌
    static constexpr const char* TopRight    = "\u2510";  // ┐
    static constexpr const char* BottomLeft  = "\u2514";  // └
    static constexpr const char* BottomRight = "\u2518";  // ┘
    static constexpr const char* Horizontal  = "\u2500";  // ─
    static constexpr const char* Vertical    = "\u2502";  // │
    static constexpr const char* TeeRight    = "\u251C";  // ├
    static constexpr const char* TeeLeft     = "\u2524";  // ┤
    static constexpr const char* TeeDown     = "\u252C";  // ┬
    static constexpr const char* TeeUp       = "\u2534";  // ┴
    static constexpr const char* Cross       = "\u253C";  // ┼

    // Tree characters
    static constexpr const char* TreeBranch  = "\u251C";  // ├
    static constexpr const char* TreeLast    = "\u2514";  // └
    static constexpr const char* TreePipe    = "\u2502";  // │

    // Heavy line (for headers)
    static constexpr const char* HeavyHoriz  = "\u2550";  // ═
};

/**
 * @brief Console output with color and formatting support
 */
class Console {
public:
    Console();

    /// Check if stdout is a terminal (supports ANSI codes)
    [[nodiscard]] bool IsTerminal() const { return is_tty_; }

    /// Enable/disable color output (auto-detected by default)
    void SetColorEnabled(bool enabled) { color_enabled_ = enabled; }
    [[nodiscard]] bool IsColorEnabled() const { return color_enabled_; }

    /// Set minimum log level for output
    void SetLogLevel(LogLevel level) { min_level_ = level; }
    [[nodiscard]] LogLevel GetLogLevel() const { return min_level_; }

    // === Logging Methods ===

    void Trace(std::string_view msg);
    void Debug(std::string_view msg);
    void Info(std::string_view msg);
    void Event(std::string_view msg);
    void Warning(std::string_view msg);
    void Error(std::string_view msg);
    void Fatal(std::string_view msg);

    /// Log with explicit level
    void Log(LogLevel level, std::string_view msg);

    /// Log with timestamp prefix
    void LogTimed(LogLevel level, double sim_time, std::string_view msg);

    // === Formatting Helpers ===

    /// Apply color if enabled
    [[nodiscard]] std::string Colorize(std::string_view text,
                                        const char* color) const;

    /// Create horizontal rule
    [[nodiscard]] std::string HorizontalRule(int width = 80,
                                              char c = '-') const;

    /// Pad string to width
    [[nodiscard]] static std::string PadRight(std::string_view text,
                                               std::size_t width);
    [[nodiscard]] static std::string PadLeft(std::string_view text,
                                              std::size_t width);

    /// Format number with comma separators
    [[nodiscard]] static std::string FormatNumber(double value,
                                                   int precision = 2);

private:
    bool is_tty_ = false;
    bool color_enabled_ = false;
    LogLevel min_level_ = LogLevel::Info;

    [[nodiscard]] const char* GetLevelColor(LogLevel level) const;
    [[nodiscard]] const char* GetLevelPrefix(LogLevel level) const;
};

} // namespace icarus
```

**Exit Criteria:**

- [ ] `IsTerminal()` correctly detects TTY
- [ ] ANSI colors work on Linux/macOS terminals
- [ ] Color codes stripped when not TTY
- [ ] All log levels functional with correct prefixes

---

### 2.5.2 Log Context and Entry

**File:** `include/icarus/io/LogEntry.hpp`

Captures full context for each log message: entity, component, timestamp, level.

```cpp
#pragma once

#include <icarus/io/Console.hpp>
#include <chrono>
#include <string>
#include <string_view>

namespace icarus {

/**
 * @brief Immutable log context - set by component/entity during execution
 *
 * The LogContext is thread-local and automatically captures the current
 * entity and component name. Components don't need to pass context explicitly.
 */
struct LogContext {
    std::string entity;      ///< Entity name (e.g., "falcon9_1")
    std::string component;   ///< Component name (e.g., "merlinEngine2")
    std::string type;        ///< Component type (e.g., "LiquidEngine")

    /// Full path: "entity.component" or just "component" if no entity
    [[nodiscard]] std::string FullPath() const {
        if (entity.empty()) return component;
        return entity + "." + component;
    }

    /// Check if context is set
    [[nodiscard]] bool IsSet() const { return !component.empty(); }
};

/**
 * @brief A single log entry with full context
 *
 * Immutable after creation. Collected into LogBuffer during hot loop.
 */
struct LogEntry {
    LogLevel level;              ///< Severity level
    double sim_time;             ///< Simulation time when logged
    std::string message;         ///< Log message
    LogContext context;          ///< Entity/component context

    // Wall clock time for ordering logs from concurrent sources
    std::chrono::steady_clock::time_point wall_time;

    /// Create a log entry with current context
    static LogEntry Create(LogLevel level, double sim_time,
                           std::string_view message,
                           const LogContext& ctx);

    /// Format for output: "[sim_time] [LEVEL] [entity.component] message"
    [[nodiscard]] std::string Format(bool include_context = true) const;

    /// Format with colors (for terminal)
    [[nodiscard]] std::string FormatColored(const Console& console) const;
};

/**
 * @brief Thread-local log context manager
 *
 * The Simulator sets this before calling component methods.
 * Components can query GetCurrentContext() implicitly.
 */
class LogContextManager {
public:
    /// Set context for current thread (called by Simulator before Step)
    static void SetContext(const LogContext& ctx);

    /// Clear context (called by Simulator after Step)
    static void ClearContext();

    /// Get current context (called internally by LogBuffer::Log)
    [[nodiscard]] static const LogContext& GetContext();

    /// RAII guard for automatic context management
    class ScopedContext {
    public:
        ScopedContext(const std::string& entity,
                      const std::string& component,
                      const std::string& type = "");
        ~ScopedContext();

        // Non-copyable
        ScopedContext(const ScopedContext&) = delete;
        ScopedContext& operator=(const ScopedContext&) = delete;

    private:
        LogContext previous_;
    };

private:
    static thread_local LogContext current_context_;
};

} // namespace icarus
```

**Exit Criteria:**

- [ ] `LogEntry` captures all required fields
- [ ] `LogContext` correctly formats `entity.component` paths
- [ ] Thread-local context management works
- [ ] `ScopedContext` RAII guard works correctly

---

### 2.5.3 Log Service (Unified Logging)

**File:** `include/icarus/io/LogService.hpp`

The **single logging service** for all Icarus output. Supports both immediate (lifecycle phases) and buffered (hot loop) modes.

```cpp
#pragma once

#include <icarus/io/Console.hpp>
#include <icarus/io/LogEntry.hpp>
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
    using Sink = std::function<void(const std::vector<LogEntry>&)>;

    LogService();

    // === Mode Control ===

    /// Enable immediate flush after each log (for lifecycle phases)
    void SetImmediateMode(bool immediate) { immediate_mode_ = immediate; }
    [[nodiscard]] bool IsImmediateMode() const { return immediate_mode_; }

    /// RAII guard for buffered mode
    class BufferedScope {
    public:
        explicit BufferedScope(LogService& service);
        ~BufferedScope();  // Flushes on destruction
    private:
        LogService& service_;
        bool previous_mode_;
    };

    // === Configuration ===

    /// Set minimum level (below this = dropped)
    void SetMinLevel(LogLevel level) { min_level_ = level; }
    [[nodiscard]] LogLevel GetMinLevel() const { return min_level_; }

    /// Set buffer capacity (pre-allocate for performance)
    void Reserve(std::size_t capacity);

    /// Add an output sink
    void AddSink(Sink sink);

    /// Add a sink that only receives entries at or above a level
    void AddSink(Sink sink, LogLevel min_level);

    /// Clear all sinks
    void ClearSinks();

    // === Logging API ===

    /// Log a message (uses current thread-local context)
    void Log(LogLevel level, double sim_time, std::string_view message);

    /// Log with explicit context (bypasses thread-local)
    void Log(LogLevel level, double sim_time, std::string_view message,
             const LogContext& ctx);

    // Convenience methods (use thread-local context)
    void Trace(double t, std::string_view msg);
    void Debug(double t, std::string_view msg);
    void Info(double t, std::string_view msg);
    void Event(double t, std::string_view msg);
    void Warning(double t, std::string_view msg);
    void Error(double t, std::string_view msg);
    void Fatal(double t, std::string_view msg);

    // === Flush Control ===

    /// Flush buffer to all sinks
    void Flush(bool sort_by_time = false);

    /// Flush and clear buffer
    void FlushAndClear(bool sort_by_time = false);

    /// Clear buffer without flushing (discard pending logs)
    void Clear();

    // === Query API ===

    [[nodiscard]] std::size_t PendingCount() const { return entries_.size(); }
    [[nodiscard]] bool HasPending() const { return !entries_.empty(); }
    [[nodiscard]] const std::vector<LogEntry>& GetPending() const { return entries_; }

    /// Check if any errors were logged (in buffer or flushed)
    [[nodiscard]] bool HasErrors() const;
    [[nodiscard]] bool HasFatalErrors() const;
    [[nodiscard]] std::size_t ErrorCount() const { return error_count_; }
    [[nodiscard]] std::size_t FatalCount() const { return fatal_count_; }

    /// Reset error counts (call at start of new run)
    void ResetErrorCounts();

    // === Filtering (for query, not output) ===

    [[nodiscard]] std::vector<LogEntry> GetEntriesAtLevel(LogLevel level) const;
    [[nodiscard]] std::vector<LogEntry> GetEntriesForEntity(std::string_view entity) const;
    [[nodiscard]] std::vector<LogEntry> GetEntriesForComponent(std::string_view path) const;

private:
    std::vector<LogEntry> entries_;
    std::vector<std::pair<Sink, LogLevel>> sinks_;  // sink + min level
    LogLevel min_level_ = LogLevel::Info;
    bool immediate_mode_ = true;  // Default: immediate for safety

    std::size_t error_count_ = 0;
    std::size_t fatal_count_ = 0;

    mutable std::mutex mutex_;

    void FlushEntry(const LogEntry& entry);
};

/**
 * @brief Global log service singleton
 */
LogService& GetLogService();

} // namespace icarus

// =============================================================================
// Logging Macros
// =============================================================================

#define ICARUS_TRACE(sim_time, msg) \
    ::icarus::GetLogService().Trace(sim_time, msg)

#define ICARUS_DEBUG(sim_time, msg) \
    ::icarus::GetLogService().Debug(sim_time, msg)

#define ICARUS_INFO(sim_time, msg) \
    ::icarus::GetLogService().Info(sim_time, msg)

#define ICARUS_EVENT(sim_time, msg) \
    ::icarus::GetLogService().Event(sim_time, msg)

#define ICARUS_WARN(sim_time, msg) \
    ::icarus::GetLogService().Warning(sim_time, msg)

#define ICARUS_ERROR(sim_time, msg) \
    ::icarus::GetLogService().Error(sim_time, msg)

#define ICARUS_FATAL(sim_time, msg) \
    ::icarus::GetLogService().Fatal(sim_time, msg)
```

**Simulator Integration:**

```cpp
template <typename Scalar>
class Simulator {
public:
    Simulator() {
        // Configure sinks at startup
        auto& log = GetLogService();
        log.AddSink(LogSinks::Console(console_));           // Terminal output
        log.AddSink(LogSinks::File("sim.log"));             // All logs to file
        log.AddSink(LogSinks::File("errors.log"), LogLevel::Error);  // Errors only
    }

    void Provision() {
        // Immediate mode: see output as it happens
        GetLogService().SetImmediateMode(true);

        ICARUS_INFO(0, "Beginning Provision phase");
        for (auto& comp : components_) {
            LogContextManager::ScopedContext ctx(comp->Entity(), comp->Name());
            ICARUS_INFO(0, "Registering component: " + comp->TypeName());
            comp->Provision(backplane_, configs_[comp.get()]);
        }
        // ... display Flight Manifest ...
    }

    void Run(Scalar t_max, Scalar dt) {
        ICARUS_EVENT(time_, "Simulation started");

        while (time_ < t_max) {
            // Buffered mode for hot loop
            LogService::BufferedScope buffered(GetLogService());

            for (auto& comp : components_) {
                LogContextManager::ScopedContext ctx(
                    comp->Entity(), comp->Name(), comp->TypeName());
                comp->Step(time_, dt);
            }

            // BufferedScope destructor flushes here
            // All logs from this step output together, sorted by entity

            time_ += dt;
        }

        ICARUS_EVENT(time_, "Simulation complete");
    }
};
```

**Usage in Components:**

```cpp
template <typename Scalar>
void LiquidEngine<Scalar>::Step(Scalar t, Scalar dt) {
    // Context already set by Simulator - logs capture "falcon9_1.merlinEngine2"
    // Mode already set to buffered - no I/O until step ends

    if (chamber_pressure_ > max_pressure_) {
        ICARUS_WARN(t, "Chamber pressure exceeds limit: " +
                    std::to_string(chamber_pressure_) + " Pa");
    }

    if (fuel_remaining_ <= 0) {
        ICARUS_EVENT(t, "Engine shutdown - fuel depleted");
    }

    // Normal operation - no logging needed
}
```

**Output Flow:**

```
┌─────────────────────────────────────────────────────────────────┐
│                        LogService                                │
│  ┌─────────────┐                                                │
│  │ Log Entry   │──┬── Immediate Mode ──► Flush to all sinks     │
│  │ + Context   │  │                                             │
│  │ + Time      │  └── Buffered Mode ───► Buffer until Flush()   │
│  └─────────────┘                              │                 │
│                                               ▼                 │
│                                    ┌──────────────────┐         │
│                                    │ Sort by entity   │         │
│                                    │ (optional)       │         │
│                                    └────────┬─────────┘         │
│                                             │                   │
└─────────────────────────────────────────────┼───────────────────┘
                                              ▼
            ┌─────────────────────────────────────────────────┐
            │                    Sinks                         │
            ├──────────────┬──────────────┬───────────────────┤
            │ Console      │ sim.log      │ errors.log        │
            │ (colored)    │ (all logs)   │ (ERROR+ only)     │
            └──────────────┴──────────────┴───────────────────┘
```

**Exit Criteria:**

- [ ] Single `LogService` handles all logging
- [ ] Immediate mode flushes each log instantly
- [ ] Buffered mode collects until `Flush()` or scope exit
- [ ] Multiple sinks with per-sink level filtering
- [ ] File sinks write without ANSI codes
- [ ] Thread-local context captured automatically
- [ ] `HasErrors()`/`HasFatalErrors()` track across modes

---

### 2.5.4 Error Integration

**File:** `include/icarus/io/ErrorHandler.hpp`

Integrates logging with `Error.hpp` exception hierarchy for proper simulation exit.

```cpp
#pragma once

#include <icarus/core/Error.hpp>
#include <icarus/io/LogBuffer.hpp>
#include <functional>
#include <optional>

namespace icarus {

/**
 * @brief Error handling policy
 */
enum class ErrorPolicy {
    Continue,      ///< Log and continue (for warnings)
    Pause,         ///< Pause simulation, allow resume
    Abort,         ///< Clean shutdown with debrief
    Crash          ///< Immediate termination (for fatal errors)
};

/**
 * @brief Error handler callback signature
 */
using ErrorCallback = std::function<ErrorPolicy(const SimulationError&)>;

/**
 * @brief Central error handler integrating logging and exceptions
 *
 * Bridges the gap between:
 * - LogBuffer (buffered logging)
 * - Error.hpp exceptions
 * - Simulation lifecycle (pause/abort/crash)
 */
class ErrorHandler {
public:
    ErrorHandler(LogService& log_service);

    // === Policy Configuration ===

    /// Set default policy for each severity level
    void SetPolicy(Severity severity, ErrorPolicy policy);

    /// Set callback for error handling (can override default policy)
    void SetCallback(ErrorCallback callback);

    /// Set maximum errors before automatic abort
    void SetMaxErrors(std::size_t max_errors);

    // === Error Reporting ===

    /// Report an error (from exception or direct call)
    /// Returns the policy that should be applied
    ErrorPolicy Report(const SimulationError& error);

    /// Report an exception, extracting info automatically
    ErrorPolicy Report(const Error& exception, double sim_time = 0.0);

    /// Report a fatal error and trigger crash
    [[noreturn]] void ReportFatal(const std::string& message,
                                   double sim_time = 0.0);

    // === Query ===

    /// Get count of errors at each severity
    [[nodiscard]] std::size_t GetErrorCount(Severity severity) const;

    /// Get total error count (WARNING and above)
    [[nodiscard]] std::size_t GetTotalErrorCount() const;

    /// Check if simulation should abort
    [[nodiscard]] bool ShouldAbort() const;

    /// Get the first fatal error (if any)
    [[nodiscard]] std::optional<SimulationError> GetFatalError() const;

    // === Integration with Simulator ===

    /// Check buffered logs for errors, apply policies
    /// Called by Simulator after Flush()
    void ProcessBufferedErrors();

    /// Clear error counts (for new run)
    void Reset();

private:
    LogService& log_service_;
    ErrorCallback callback_;

    std::map<Severity, ErrorPolicy> policies_;
    std::map<Severity, std::size_t> error_counts_;
    std::optional<SimulationError> fatal_error_;
    std::size_t max_errors_ = 100;  // Abort after this many errors
};

/**
 * @brief RAII wrapper for try/catch with automatic error reporting
 */
class ErrorScope {
public:
    ErrorScope(ErrorHandler& handler, double sim_time,
               const std::string& component);
    ~ErrorScope();

    /// Execute a callable, catching and reporting exceptions
    template <typename F>
    auto Execute(F&& func) -> decltype(func());

private:
    ErrorHandler& handler_;
    double sim_time_;
    std::string component_;
};

} // namespace icarus
```

**Simulator Integration:**

```cpp
template <typename Scalar>
class Simulator {
public:
    void Step(Scalar dt) {
        // Buffered mode for the step
        LogService::BufferedScope buffered(GetLogService());

        for (auto& comp : components_) {
            LogContextManager::ScopedContext ctx(
                comp->Entity(), comp->Name(), comp->TypeName());

            try {
                comp->Step(time_, dt);
            } catch (const Error& e) {
                auto policy = error_handler_.Report(e, time_);
                HandleErrorPolicy(policy, comp.get());
            }
        }

        // BufferedScope flushes here, then check for errors
        // (destructor runs, logs output)
    }

    void PostStep() {
        // After flush, process any errors that were logged
        error_handler_.ProcessBufferedErrors();

        if (error_handler_.ShouldAbort()) {
            TriggerShutdown(ExitStatus::Error);
        }
    }

private:
    void HandleErrorPolicy(ErrorPolicy policy, Component<Scalar>* comp) {
        switch (policy) {
            case ErrorPolicy::Continue:
                break;  // Already logged
            case ErrorPolicy::Pause:
                phase_ = Phase::Paused;
                break;
            case ErrorPolicy::Abort:
                TriggerShutdown(ExitStatus::Error);
                break;
            case ErrorPolicy::Crash:
                std::terminate();
                break;
        }
    }

    ErrorHandler error_handler_{GetLogService()};
};
```

**Default Policies:**

| Severity | Default Policy | Behavior |
|:---------|:---------------|:---------|
| INFO | Continue | Log only |
| WARNING | Continue | Log, increment counter |
| ERROR | Continue | Log, increment counter, check max |
| FATAL | Crash | Log, immediate termination |

**Exit Criteria:**

- [ ] Exceptions automatically logged with context
- [ ] Error policies configurable per severity
- [ ] Max error limit triggers abort
- [ ] Fatal errors cause immediate crash
- [ ] Error counts tracked and queryable
- [ ] Integration with Simulator lifecycle

---

### 2.5.5 Log Sinks

**File:** `include/icarus/io/LogSink.hpp`

Pre-built sinks for common output destinations.

```cpp
#pragma once

#include <icarus/io/Console.hpp>
#include <icarus/io/LogBuffer.hpp>
#include <fstream>
#include <memory>

namespace icarus {

/**
 * @brief Factory for common log sinks
 */
class LogSinks {
public:
    /// Console sink with colors (respects TTY detection)
    static LogBuffer::Sink Console(const class Console& console);

    /// Console sink with entity grouping
    /// Groups logs by entity, prints entity header before each group
    static LogBuffer::Sink ConsoleGrouped(const class Console& console);

    /// File sink (plain text, no colors)
    static LogBuffer::Sink File(const std::string& path);

    /// File sink with rotation
    static LogBuffer::Sink RotatingFile(const std::string& path,
                                         std::size_t max_size_bytes,
                                         std::size_t max_files);

    /// JSON Lines sink (for log aggregation)
    static LogBuffer::Sink JsonLines(const std::string& path);

    /// Null sink (for testing/benchmarking)
    static LogBuffer::Sink Null();

    /// Callback sink (custom handling)
    static LogBuffer::Sink Callback(
        std::function<void(const LogEntry&)> handler);
};

/**
 * @brief Entity-grouped console output example:
 *
 * ─── falcon9_1 ───────────────────────────────────────────────────────────────
 * [00.150] [INFO] [merlinEngine1] Ignition sequence started
 * [00.150] [INFO] [merlinEngine2] Ignition sequence started
 * [00.200] [WARN] [merlinEngine2] Chamber pressure 105% of nominal
 *
 * ─── falcon9_2 ───────────────────────────────────────────────────────────────
 * [00.150] [INFO] [merlinEngine1] Ignition sequence started
 */

} // namespace icarus
```

**Exit Criteria:**

- [ ] Console sink outputs colored entries
- [ ] Grouped sink organizes by entity
- [ ] File sink writes without colors
- [ ] Rotating file sink respects size limits
- [ ] JSON Lines sink outputs valid JSONL

---

### 2.5.6 ASCII Table Formatter

**File:** `include/icarus/io/AsciiTable.hpp`

Generates formatted ASCII tables with box-drawing characters.

```cpp
#pragma once

#include <icarus/io/Console.hpp>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief ASCII table generator with box-drawing characters
 */
class AsciiTable {
public:
    enum class Align { Left, Right, Center };

    struct Column {
        std::string header;
        std::size_t width = 0;   // 0 = auto-size
        Align align = Align::Left;
    };

    /// Add a column definition
    void AddColumn(const std::string& header, std::size_t width = 0,
                   Align align = Align::Left);

    /// Add a row of data
    void AddRow(const std::vector<std::string>& cells);

    /// Generate the formatted table string
    [[nodiscard]] std::string Render() const;

    /// Clear all rows (keep columns)
    void ClearRows();

    /// Clear everything
    void Clear();

private:
    std::vector<Column> columns_;
    std::vector<std::vector<std::string>> rows_;

    void AutoSizeColumns();
    [[nodiscard]] std::string RenderDivider(bool top, bool bottom) const;
    [[nodiscard]] std::string RenderRow(const std::vector<std::string>& cells) const;
    [[nodiscard]] static std::string AlignCell(const std::string& text,
                                                std::size_t width, Align align);
};

/**
 * @brief Example output:
 *
 * ┌────────────────────────┬─────────┬─────────────────────────────────────┐
 * │ SIGNAL NAME            │ UNIT    │ DESCRIPTION                         │
 * ├────────────────────────┼─────────┼─────────────────────────────────────┤
 * │ Gravity.force.x        │ N       │ Gravity force (x)                   │
 * │ Gravity.force.y        │ N       │ Gravity force (y)                   │
 * │ Gravity.force.z        │ N       │ Gravity force (z)                   │
 * └────────────────────────┴─────────┴─────────────────────────────────────┘
 */

} // namespace icarus
```

**Exit Criteria:**

- [ ] Table headers rendered correctly
- [ ] Column auto-sizing works
- [ ] All alignment modes work (left, right, center)
- [ ] Box-drawing characters render correctly in terminal

---

### 2.5.7 Flight Manifest Generator

**File:** `include/icarus/io/FlightManifest.hpp`

Generates the formatted ASCII Data Dictionary. **Writes to file** (can be thousands of signals), prints **summary only** to console.

```cpp
#pragma once

#include <icarus/io/AsciiTable.hpp>
#include <icarus/io/Console.hpp>
#include <icarus/io/DataDictionary.hpp>
#include <filesystem>
#include <fstream>
#include <string>

namespace icarus {

/**
 * @brief Generate Flight Manifest (ASCII Data Dictionary)
 *
 * Called at the end of Provision phase. Behavior:
 * - Full manifest → file (data_dictionary.txt)
 * - Summary only → console (counts, unwired inputs, warnings)
 *
 * The full manifest can be thousands of signals, so it's never
 * dumped to console by default.
 */
class FlightManifest {
public:
    explicit FlightManifest(const Console& console);

    // === Configuration ===

    /// Set output file path (default: "data_dictionary.txt")
    void SetOutputPath(const std::filesystem::path& path) { output_path_ = path; }

    /// Set simulation version string
    void SetVersion(const std::string& version) { version_ = version; }

    /// Set config source for a component
    void SetConfigSource(const std::string& component, const std::string& source);

    /// Enable/disable full console output (default: false, summary only)
    void SetFullConsoleOutput(bool enabled) { full_console_ = enabled; }

    // === Generation ===

    /// Generate manifest: write full to file, summary to console
    void Generate(const DataDictionary& dict);

    /// Generate full manifest string (for file or explicit request)
    [[nodiscard]] std::string GenerateFull(const DataDictionary& dict) const;

    /// Generate summary string (for console)
    [[nodiscard]] std::string GenerateSummary(const DataDictionary& dict) const;

    /// Write full manifest to file
    void WriteToFile(const DataDictionary& dict) const;

    /// Print summary to console (default behavior)
    void PrintSummary(const DataDictionary& dict) const;

    /// Print full manifest to console (opt-in, use sparingly)
    void PrintFull(const DataDictionary& dict) const;

private:
    const Console& console_;
    std::filesystem::path output_path_ = "data_dictionary.txt";
    std::string version_ = "0.1.0-dev";
    std::map<std::string, std::string> config_sources_;
    bool full_console_ = false;

    [[nodiscard]] std::string GenerateHeader() const;
    [[nodiscard]] std::string GenerateComponentSection(
        const DataDictionary::ComponentEntry& comp) const;
    [[nodiscard]] std::string GenerateSignalTable(
        const std::string& title,
        const std::vector<SignalDescriptor>& signals) const;
};

/**
 * @brief Console summary example (what users see):
 *
 * ─── [ FLIGHT MANIFEST ] ─────────────────────────────────────────────────────
 * [00.050] [SYS] Data Dictionary written to: data_dictionary.txt
 *
 *   Components:    12          Outputs:     847
 *   Inputs:       203          Parameters:   52
 *   States:        24          Config:       18
 *
 *   ⚠ Unwired inputs: 3
 *     - X15.Aero.mach
 *     - X15.Engine.altitude
 *     - X15.GNC.nav_state
 *
 * See data_dictionary.txt for full signal listing.
 * ─────────────────────────────────────────────────────────────────────────────
 */

/**
 * @brief Full file output example (data_dictionary.txt):
 *
 * ================================================================================
 *   ICARUS SIMULATION ENGINE | DATA DICTIONARY
 *   Generated: 2023-10-27 14:30:00 | Ver: 0.1.0-alpha
 * ================================================================================
 *
 * [ SYSTEM SUMMARY ]
 *   Total Components: 2        Total Outputs: 13
 *   Total Inputs:     0        Total Params:  0
 *   Integrable States: 6       Config Values: 2
 *
 * --------------------------------------------------------------------------------
 * COMPONENT: Gravity
 *   Type: PointMassGravity
 *   Source: config/environment/gravity_simple.yaml
 * --------------------------------------------------------------------------------
 *   [OUTPUTS]
 *   ┌────────────────────────┬─────────┬─────────────────────────────────────┐
 *   │ SIGNAL NAME            │ UNIT    │ DESCRIPTION                         │
 *   ├────────────────────────┼─────────┼─────────────────────────────────────┤
 *   │ Gravity.force.x        │ N       │ Gravity force (x)                   │
 *   ...
 */

} // namespace icarus
```

**Exit Criteria:**

- [ ] Full manifest written to `data_dictionary.txt`
- [ ] Console shows summary only (counts, unwired warnings)
- [ ] Unwired inputs listed in console summary
- [ ] Header with timestamp and version in file
- [ ] Each component section has type and config source
- [ ] Signal tables use box-drawing characters in file
- [ ] File scales to thousands of signals without issue

---

### 2.5.8 Mission Logger

**File:** `include/icarus/io/MissionLogger.hpp`

The main logging service that tracks lifecycle phases and events.

```cpp
#pragma once

#include <icarus/io/Console.hpp>
#include <icarus/io/FlightManifest.hpp>
#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Simulation lifecycle phases for logging
 */
enum class SimPhase {
    Init,
    Provision,
    Stage,
    Run,
    Shutdown
};

/**
 * @brief Component timing statistics
 */
struct ComponentStats {
    std::string name;
    std::size_t call_count = 0;
    double total_time_us = 0.0;
    double avg_time_us = 0.0;
    double max_time_us = 0.0;
    double percent_load = 0.0;
};

/**
 * @brief Mission Logger - Flight Recorder style logging
 *
 * Provides structured logging throughout the simulation lifecycle.
 * Outputs to both terminal (with colors) and log file (plain ASCII).
 */
class MissionLogger {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;

    MissionLogger();
    explicit MissionLogger(const std::string& log_file_path);

    // === Configuration ===

    /// Set console log level (terminal output)
    void SetConsoleLevel(LogLevel level);

    /// Set file log level (log file output)
    void SetFileLevel(LogLevel level);

    /// Set simulation version string
    void SetVersion(const std::string& version);

    /// Set build type (DEBUG/RELEASE)
    void SetBuildType(const std::string& build_type);

    /// Set scalar mode (DOUBLE/CASADI)
    void SetScalarMode(const std::string& mode);

    /// Enable/disable progress display during Run
    void SetProgressEnabled(bool enabled);

    /// Enable/disable profiling
    void SetProfilingEnabled(bool enabled);

    // === Lifecycle Logging ===

    /// Log simulation startup (splash screen)
    void LogStartup();

    /// Begin a lifecycle phase
    void BeginPhase(SimPhase phase);

    /// End current phase
    void EndPhase();

    /// Log the Flight Manifest (Data Dictionary)
    void LogManifest(const DataDictionary& dict);

    /// Log mission debrief (shutdown statistics)
    void LogDebrief(double sim_time, double wall_time);

    // === Provision Phase Logging ===

    /// Log entity loading
    void LogEntityLoad(const std::string& entity_name);

    /// Log component addition (tree format)
    void LogComponentAdd(const std::string& component_name,
                         const std::string& type,
                         const std::string& config_source = "defaults",
                         bool is_last = false);

    /// Log static asset loading
    void LogAssetLoad(const std::string& asset_name,
                      const std::string& description);

    /// Log state vector allocation
    void LogStateAllocation(std::size_t continuous, std::size_t discrete = 0);

    // === Stage Phase Logging ===

    /// Log signal wiring
    void LogWiring(const std::string& source, const std::string& target,
                   bool is_warning = false);

    /// Log wiring warning (multiple writers, etc.)
    void LogWiringWarning(const std::string& message);

    /// Log topological sort order
    void LogExecutionOrder(const std::vector<std::string>& component_order);

    /// Log trim solver progress
    void LogTrimStart(const std::string& mode,
                      const std::vector<std::pair<std::string, double>>& targets);
    void LogTrimIteration(int iteration, double residual);
    void LogTrimConverged(int iterations);
    void LogTrimFailed(const std::string& reason);

    // === Run Phase Logging ===

    /// Log phase transition event
    void LogPhaseEvent(const std::string& from_phase,
                       const std::string& to_phase, double sim_time);

    /// Log notable event (apogee, impact, etc.)
    void LogEvent(const std::string& event_name, double sim_time,
                  const std::string& details = "");

    /// Log warning during run
    void LogRunWarning(const std::string& source, const std::string& message,
                       double sim_time);

    /// Log error during run
    void LogRunError(const std::string& source, const std::string& message,
                     double sim_time);

    /// Update progress display (single-line, \r overwrite)
    void UpdateProgress(double sim_time, double t_max,
                        const std::map<std::string, double>& key_values = {});

    // === Profiling ===

    /// Begin timing a component
    void BeginComponentTiming(const std::string& component_name);

    /// End timing for current component
    void EndComponentTiming();

    /// Get profiling statistics
    [[nodiscard]] std::vector<ComponentStats> GetProfilingStats() const;

    // === Low-Level Access ===

    /// Get underlying console
    [[nodiscard]] Console& GetConsole() { return console_; }
    [[nodiscard]] const Console& GetConsole() const { return console_; }

    /// Log raw message
    void Log(LogLevel level, const std::string& message);
    void LogTimed(LogLevel level, double sim_time, const std::string& message);

private:
    Console console_;
    std::ofstream log_file_;
    LogLevel file_level_ = LogLevel::Debug;

    // Simulation metadata
    std::string version_ = "0.1.0-dev";
    std::string build_type_ = "RELEASE";
    std::string scalar_mode_ = "DOUBLE";

    // Timing
    TimePoint startup_time_;
    TimePoint phase_start_time_;
    SimPhase current_phase_ = SimPhase::Init;

    // Progress display
    bool progress_enabled_ = true;
    int progress_width_ = 50;

    // Profiling
    bool profiling_enabled_ = false;
    std::map<std::string, ComponentStats> component_stats_;
    std::string current_timed_component_;
    TimePoint component_start_time_;

    // Helpers
    [[nodiscard]] double GetWallClockSeconds() const;
    [[nodiscard]] std::string GetTimestamp() const;
    [[nodiscard]] std::string FormatPhaseHeader(SimPhase phase) const;
    void WriteToFile(const std::string& message);
};

} // namespace icarus
```

**Exit Criteria:**

- [ ] Splash screen renders correctly with ASCII art
- [ ] Each phase has distinct visual header
- [ ] Component tree uses proper tree characters
- [ ] Progress bar updates in-place on terminal
- [ ] Log file contains all messages (stripped of ANSI codes)

---

### 2.5.9 Splash Screen and Banner

**File:** `include/icarus/io/Banner.hpp`

ASCII art banner and version display.

```cpp
#pragma once

#include <string>

namespace icarus {

/**
 * @brief ASCII art banners and headers
 */
class Banner {
public:
    /// Get the main Icarus splash screen (ASCII art logo)
    [[nodiscard]] static std::string GetSplashScreen();

    /// Get a section header
    [[nodiscard]] static std::string GetPhaseHeader(const std::string& phase_name);

    /// Get the mission debrief header
    [[nodiscard]] static std::string GetDebriefHeader();

    /// Get horizontal rule
    [[nodiscard]] static std::string GetRule(int width = 80, char c = '=');

    /// Get double-line rule
    [[nodiscard]] static std::string GetDoubleRule(int width = 80);
};

/**
 * @brief Example splash screen:
 *
 * ################################################################################
 * #                                                                              #
 * #    ██╗ ██████╗ █████╗ ██████╗ ██╗   ██╗███████╗                              #
 * #    ██║██╔════╝██╔══██╗██╔══██╗██║   ██║██╔════╝                              #
 * #    ██║██║     ███████║██████╔╝██║   ██║███████╗                              #
 * #    ██║██║     ██╔══██║██╔══██╗██║   ██║╚════██║                              #
 * #    ██║╚██████╗██║  ██║██║  ██║╚██████╔╝███████║                              #
 * #    ╚═╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝                              #
 * #                                                                              #
 * #    6DOF FLIGHT DYNAMICS ENGINE | BUILD: RELEASE | MODE: CASADI               #
 * ################################################################################
 */

} // namespace icarus
```

**Exit Criteria:**

- [ ] ASCII art logo renders correctly
- [ ] Build type and mode displayed
- [ ] Width fits standard 80-column terminal

---

### 2.5.10 Mission Debrief Formatter

**File:** `include/icarus/io/MissionDebrief.hpp`

Generates the shutdown statistics and profiling report.

```cpp
#pragma once

#include <icarus/io/AsciiTable.hpp>
#include <icarus/io/Console.hpp>
#include <icarus/io/MissionLogger.hpp>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Exit status codes
 */
enum class ExitStatus {
    Success,
    EndConditionMet,
    UserAbort,
    Error,
    Divergence
};

/**
 * @brief Mission Debrief generator
 *
 * Generates the shutdown statistics and profiling summary.
 */
class MissionDebrief {
public:
    explicit MissionDebrief(const Console& console);

    /// Set exit status
    void SetExitStatus(ExitStatus status) { exit_status_ = status; }

    /// Set exit reason message
    void SetExitReason(const std::string& reason) { exit_reason_ = reason; }

    /// Set timing information
    void SetTiming(double sim_time, double wall_time);

    /// Set profiling data
    void SetProfilingData(const std::vector<ComponentStats>& stats);

    /// Generate the full debrief string
    [[nodiscard]] std::string Generate() const;

    /// Generate and print to console
    void Print() const;

private:
    const Console& console_;

    ExitStatus exit_status_ = ExitStatus::Success;
    std::string exit_reason_;
    double sim_time_ = 0.0;
    double wall_time_ = 0.0;
    double real_time_factor_ = 0.0;
    std::vector<ComponentStats> profiling_data_;

    [[nodiscard]] std::string StatusToString(ExitStatus status) const;
    [[nodiscard]] std::string GenerateProfilingTable() const;
    [[nodiscard]] std::string GenerateQuote() const;
};

/**
 * @brief Example output:
 *
 * ─── [ PHASE: SHUTDOWN ] ────────────────────────────────────────────────────────
 * [30.010] [IO ] Flushing data logs to: /logs/run_20231027_001.h5
 * [30.015] [SYS] Resources released.
 *
 * ================================================================================
 *   MISSION DEBRIEF
 * ================================================================================
 *   Exit Status:      SUCCESS
 *   Exit Reason:      End condition met (t >= t_max)
 *   Simulated Time:   30.00 s
 *   Wall Clock Time:  0.45 s
 *   Real-Time Factor: 66.6x (Faster than real-time)
 *
 *   [ PROFILE HOTSPOTS ]
 *   ┌──────────────────────┬─────────────┬──────────┐
 *   │ COMPONENT            │ AVG (us)    │ % LOAD   │
 *   ├──────────────────────┼─────────────┼──────────┤
 *   │ Aerodynamics         │ 120         │ 45.0%    │
 *   │ EquationsOfMotion    │ 40          │ 15.0%    │
 *   │ Integrator (RK4)     │ 20          │ 7.5%     │
 *   └──────────────────────┴─────────────┴──────────┘
 *
 *   "Icarus flew... and this time, he stuck the landing."
 * ================================================================================
 */

} // namespace icarus
```

**Exit Criteria:**

- [ ] Exit status correctly displayed with color
- [ ] Real-time factor calculated and displayed
- [ ] Profiling table sorted by load percentage
- [ ] Closing quote displayed

---

### 2.5.11 Simulator Integration

**File:** `include/icarus/sim/Simulator.hpp` (extend)

Integrate MissionLogger into the Simulator lifecycle.

```cpp
template <typename Scalar>
class Simulator {
public:
    // === Logging Configuration ===

    /// Get the mission logger
    [[nodiscard]] MissionLogger& GetLogger() { return logger_; }
    [[nodiscard]] const MissionLogger& GetLogger() const { return logger_; }

    /// Enable/disable console output during run
    void SetQuietMode(bool quiet);

    /// Set log file path
    void SetLogFile(const std::string& path);

    /// Enable/disable profiling
    void SetProfilingEnabled(bool enabled);

    // Existing methods updated to log:

    void Provision() {
        logger_.BeginPhase(SimPhase::Provision);

        // ... existing provision logic ...

        // Log component additions as tree
        for (auto& comp : components_) {
            bool is_last = (&comp == &components_.back());
            logger_.LogComponentAdd(comp->FullName(), comp->TypeName(),
                                     "defaults", is_last);
            // ... existing provision logic ...
        }

        // Log state allocation
        logger_.LogStateAllocation(GetTotalStateSize());

        // Generate and display Flight Manifest
        logger_.LogManifest(GetDataDictionary());

        logger_.EndPhase();
        // ...
    }

    void Stage() {
        logger_.BeginPhase(SimPhase::Stage);

        // ... existing stage logic ...

        // Log wiring as it happens
        for (const auto& wire : wiring_config_.GetAllWirings()) {
            logger_.LogWiring(wire.second, wire.first);
        }

        logger_.EndPhase();
        // ...
    }

    void Step(Scalar dt) {
        if (profiling_enabled_) {
            for (auto& comp : components_) {
                logger_.BeginComponentTiming(comp->FullName());
                comp->Step(time_, dt);
                logger_.EndComponentTiming();
            }
        } else {
            // ... normal step ...
        }
    }

    void Run(Scalar t_max, Scalar dt) {
        logger_.BeginPhase(SimPhase::Run);
        logger_.Log(LogLevel::Info, "Simulation started. dt=" +
                    std::to_string(dt) + "s");

        while (time_ < t_max) {
            Step(dt);
            logger_.UpdateProgress(time_, t_max);
        }

        logger_.EndPhase();
    }

    void Shutdown() {
        logger_.BeginPhase(SimPhase::Shutdown);
        logger_.LogDebrief(time_, GetWallClockTime());
        logger_.EndPhase();
    }

private:
    MissionLogger logger_;
    bool profiling_enabled_ = false;
    bool quiet_mode_ = false;
};
```

**Exit Criteria:**

- [ ] Provision logs component tree
- [ ] Provision displays Flight Manifest
- [ ] Stage logs wiring graph
- [ ] Run displays progress (when not quiet)
- [ ] Shutdown displays debrief with stats
- [ ] Profiling works when enabled

---

### 2.5.12 Logging Configuration

**File:** `include/icarus/io/LogConfig.hpp`

Configuration structure for logging settings.

```cpp
#pragma once

#include <icarus/io/Console.hpp>
#include <string>

namespace icarus {

/**
 * @brief Logging configuration
 */
struct LogConfig {
    // Console output
    LogLevel console_level = LogLevel::Info;
    bool color_enabled = true;    // Auto-detect if terminal
    bool progress_enabled = true;

    // File output
    bool file_enabled = false;
    std::string file_path;
    LogLevel file_level = LogLevel::Debug;
    bool file_rotate = false;
    std::size_t max_file_size_mb = 100;

    // Features
    bool profiling_enabled = false;
    bool quiet_mode = false;      // Suppress all but errors
    bool show_manifest = true;    // Show Flight Manifest at Provision

    // Progress display
    int progress_width = 50;
    std::string progress_format = "[RUN] Time: {time} | {custom}";

    /// Create default config
    static LogConfig Default();

    /// Create quiet config (errors only)
    static LogConfig Quiet();

    /// Create verbose config (all output)
    static LogConfig Verbose();

    /// Create profiling config
    static LogConfig WithProfiling();
};

} // namespace icarus
```

**YAML Configuration Format:**

```yaml
logging:
  console:
    level: INFO        # TRACE, DEBUG, INFO, EVENT, WARNING, ERROR
    color: auto        # true, false, auto (detect terminal)
    progress: true     # Show progress bar during Run

  file:
    enabled: true
    path: "logs/sim_{timestamp}.log"
    level: DEBUG
    rotate: true
    max_size_mb: 100

  features:
    profiling: false
    show_manifest: true

  progress:
    width: 50
    format: "[RUN] Time: {time:.2f}s | Alt: {altitude:.0f}m | Vel: {velocity:.1f}m/s"
```

**Exit Criteria:**

- [ ] YAML configuration parsing works
- [ ] All config options respected
- [ ] Auto-detection of terminal for colors

---

### 2.5.13 Implementation Files

**File:** `src/io/Console.cpp`
**File:** `src/io/AsciiTable.cpp`
**File:** `src/io/FlightManifest.cpp`
**File:** `src/io/MissionLogger.cpp`
**File:** `src/io/Banner.cpp`
**File:** `src/io/MissionDebrief.cpp`
**File:** `src/io/LogConfig.cpp`

Implementation of all logging classes.

**Exit Criteria:**

- [ ] All classes compile and link
- [ ] No memory leaks
- [ ] Thread-safe file writing (if applicable)

---

### 2.5.14 Unit Tests

**File:** `tests/unit/console_test.cpp`

```cpp
TEST(Console, DetectsTerminal) {
    Console console;
    // isatty should return consistent result
    bool is_tty = console.IsTerminal();
    EXPECT_EQ(is_tty, isatty(STDOUT_FILENO));
}

TEST(Console, ColorizeStripsWhenDisabled) {
    Console console;
    console.SetColorEnabled(false);

    auto result = console.Colorize("test", AnsiColor::Red);
    EXPECT_EQ(result, "test");  // No color codes
}

TEST(Console, ColorizeAddsWhenEnabled) {
    Console console;
    console.SetColorEnabled(true);

    auto result = console.Colorize("test", AnsiColor::Red);
    EXPECT_TRUE(result.find("\033[31m") != std::string::npos);
    EXPECT_TRUE(result.find("\033[0m") != std::string::npos);
}
```

**File:** `tests/unit/ascii_table_test.cpp`

```cpp
TEST(AsciiTable, RendersCorrectly) {
    AsciiTable table;
    table.AddColumn("NAME", 10);
    table.AddColumn("VALUE", 8);

    table.AddRow({"foo", "123"});
    table.AddRow({"bar", "456"});

    std::string result = table.Render();

    EXPECT_TRUE(result.find("┌") != std::string::npos);
    EXPECT_TRUE(result.find("foo") != std::string::npos);
    EXPECT_TRUE(result.find("└") != std::string::npos);
}

TEST(AsciiTable, AutoSizesColumns) {
    AsciiTable table;
    table.AddColumn("HEADER");  // Auto-size

    table.AddRow({"short"});
    table.AddRow({"this is a longer value"});

    std::string result = table.Render();

    // Should fit the longer value
    EXPECT_TRUE(result.find("this is a longer value") != std::string::npos);
}
```

**File:** `tests/unit/mission_logger_test.cpp`

```cpp
TEST(MissionLogger, LogsAllPhases) {
    std::stringstream capture;
    // Redirect cout to capture

    MissionLogger logger;
    logger.LogStartup();
    logger.BeginPhase(SimPhase::Provision);
    logger.LogComponentAdd("TestComponent", "TestType", "config.yaml", true);
    logger.EndPhase();

    // Verify output contains expected elements
    std::string output = capture.str();
    EXPECT_TRUE(output.find("ICARUS") != std::string::npos);
    EXPECT_TRUE(output.find("PROVISION") != std::string::npos);
    EXPECT_TRUE(output.find("TestComponent") != std::string::npos);
}

TEST(MissionLogger, ProfilesComponents) {
    MissionLogger logger;
    logger.SetProfilingEnabled(true);

    logger.BeginComponentTiming("Component1");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    logger.EndComponentTiming();

    auto stats = logger.GetProfilingStats();
    ASSERT_EQ(stats.size(), 1);
    EXPECT_EQ(stats[0].name, "Component1");
    EXPECT_GT(stats[0].avg_time_us, 9000);  // At least 9ms
}
```

**Exit Criteria:**

- [ ] Console color tests pass
- [ ] ASCII table rendering tests pass
- [ ] Logger phase tests pass
- [ ] Profiling timing tests pass

---

### 2.5.15 Integration Tests

**File:** `tests/integration/logging_integration_test.cpp`

```cpp
TEST(LoggingIntegration, FullSimulationRun) {
    Simulator<double> sim;
    sim.SetLogFile("/tmp/test_sim.log");
    sim.GetLogger().SetConsoleLevel(LogLevel::Info);

    auto gravity = std::make_unique<PointMassGravity<double>>();
    auto point_mass = std::make_unique<PointMass3DOF<double>>();
    point_mass->SetInitialPosition(0, 0, 100);  // 100m altitude

    sim.AddComponent(std::move(gravity));
    sim.AddComponent(std::move(point_mass));

    sim.Wire<double>("PointMass3DOF.force.x", "Gravity.force.x");
    sim.Wire<double>("PointMass3DOF.force.y", "Gravity.force.y");
    sim.Wire<double>("PointMass3DOF.force.z", "Gravity.force.z");

    // Run full lifecycle
    sim.Provision();  // Should display Flight Manifest
    sim.Stage();      // Should display wiring
    sim.Run(5.0, 0.01);  // Run for 5 seconds
    sim.Shutdown();   // Should display debrief

    // Verify log file exists and has content
    std::ifstream log_file("/tmp/test_sim.log");
    EXPECT_TRUE(log_file.good());

    std::stringstream buffer;
    buffer << log_file.rdbuf();
    std::string log_content = buffer.str();

    EXPECT_TRUE(log_content.find("PROVISION") != std::string::npos);
    EXPECT_TRUE(log_content.find("STAGE") != std::string::npos);
    EXPECT_TRUE(log_content.find("RUN") != std::string::npos);
    EXPECT_TRUE(log_content.find("SHUTDOWN") != std::string::npos);
    EXPECT_TRUE(log_content.find("DEBRIEF") != std::string::npos);
}
```

**Exit Criteria:**

- [ ] Full lifecycle produces expected log output
- [ ] Log file written correctly
- [ ] No crashes during logging

---

## File Summary

| File | Action | Description |
|:-----|:-------|:------------|
| `include/icarus/io/Console.hpp` | Create | Console abstraction with ANSI colors |
| `include/icarus/io/LogEntry.hpp` | Create | Log entry and context structs |
| `include/icarus/io/LogService.hpp` | Create | **Unified logging service** (immediate + buffered) |
| `include/icarus/io/LogSink.hpp` | Create | Pre-built output sinks (console, file, etc.) |
| `include/icarus/io/ErrorHandler.hpp` | Create | Error integration with `Error.hpp` |
| `include/icarus/io/AsciiTable.hpp` | Create | Box-drawing table generator |
| `include/icarus/io/FlightManifest.hpp` | Create | ASCII Data Dictionary formatter |
| `include/icarus/io/Banner.hpp` | Create | ASCII art banners |
| `include/icarus/io/MissionDebrief.hpp` | Create | Shutdown statistics formatter |
| `include/icarus/io/LogConfig.hpp` | Create | Logging configuration |
| `include/icarus/sim/Simulator.hpp` | Extend | Integrate LogService and ErrorHandler |
| `src/io/*.cpp` | Create | Implementation files |
| `tests/unit/*_test.cpp` | Create | Unit tests |
| `tests/integration/logging_integration_test.cpp` | Create | Integration tests |

---

## Dependencies

```
Console.hpp
     ↓
LogEntry.hpp (LogContext, LogEntry)
     ↓
LogService.hpp ←── LogSink.hpp (console, file, grouped, etc.)
     ↓
ErrorHandler.hpp ←── Error.hpp
     ↓
AsciiTable.hpp
     ↓
FlightManifest.hpp ←── DataDictionary.hpp
     ↓
Banner.hpp, MissionDebrief.hpp
     ↓
LogConfig.hpp
     ↓
Simulator.hpp integration
     ↓
Unit tests → Integration tests
```

---

## Exit Criteria (Phase 2.5)

**Buffered Logging:**
- [ ] `LogBuffer` collects entries during Step without I/O
- [ ] Logs flush at end of each step (or on demand)
- [ ] Multi-entity logs can be grouped/sorted by entity
- [ ] Thread-local context automatically captured

**Context and Error Integration:**
- [ ] `LogContext` captures entity.component paths (e.g., `falcon9_1.merlinEngine2`)
- [ ] All logs include sim time, level, and context
- [ ] `ErrorHandler` integrates with `Error.hpp` exceptions
- [ ] Error policies (Continue/Pause/Abort/Crash) work correctly
- [ ] Fatal errors trigger proper simulation crash

**Flight Recorder Output:**
- [ ] `MissionLogger` tracks all lifecycle phases
- [ ] Flight Manifest writes full dictionary to file, summary to console
- [ ] Unwired inputs highlighted in console summary
- [ ] Component loading shown as hierarchical tree
- [ ] Wiring displayed during Stage
- [ ] Progress bar updates during Run (when enabled)
- [ ] Mission Debrief displays stats at Shutdown
- [ ] Profiling data collected and displayed (when enabled)

**Output and Configuration:**
- [ ] ANSI colors work in terminal, stripped for files
- [ ] Log file written with all messages
- [ ] YAML configuration works for all options
- [ ] Console and file sinks work correctly
- [ ] Entity-grouped output works

**Testing:**
- [ ] All unit tests pass
- [ ] Integration test passes

---

## Design Decisions

### 1. Separate Console and File Output

Two output streams with different configurations:

| Aspect | Console | File |
|:-------|:--------|:-----|
| **Colors** | ANSI codes (if terminal) | Stripped |
| **Progress** | In-place updates (`\r`) | New lines |
| **Default Level** | INFO | DEBUG |
| **Format** | Compact | Timestamped |

This allows clean terminal output while maintaining detailed logs for debugging.

### 2. Box-Drawing Characters

Using Unicode box-drawing characters for tables and trees:

- **Pro:** Professional appearance, easy to read
- **Con:** Requires UTF-8 support
- **Fallback:** ASCII-only mode with `+`, `-`, `|` characters

### 3. Profiling as Opt-In

Profiling adds overhead, so it's disabled by default:

```cpp
sim.SetProfilingEnabled(true);  // Must explicitly enable
```

When enabled:
- Wraps every `comp->Step()` call with timing
- Accumulates statistics per component
- Reports in Mission Debrief

### 4. Progress Display

During Run, a single-line progress display updates in place:

```
[RUN] Time: 12.4s | Alt: 500m | Vel: 300m/s
```

Uses `\r` to overwrite the line. Only works on terminal (TTY).

### 5. Logging During Integrator Callbacks

The integrator calls `ComputeDerivatives()` multiple times per step (e.g., 4 times for RK4). Logging must be aware of this:

- Don't log every derivative evaluation
- Only log actual time advances
- Events logged based on post-step state

### 6. Flight Manifest Timing

The Flight Manifest is generated and displayed at the **end of Provision**, because:
- All components are registered
- State allocation known
- Before wiring (inputs may still be unwired)

The manifest can be regenerated at any time via `sim.GenerateDataDictionary()`.

### 7. Unified LogService with Mode Switching

All logging goes through `LogService`. The mode determines when logs are flushed:

```cpp
// Immediate mode (lifecycle phases): flush each log instantly
GetLogService().SetImmediateMode(true);
ICARUS_INFO(0, "Loading component...");  // → Console + file immediately

// Buffered mode (hot loop): collect, flush at step end
{
    LogService::BufferedScope buffered(GetLogService());
    // ... all Step() calls ...
    ICARUS_WARN(t, "Pressure high");  // → Buffer only
}  // ← Destructor flushes all buffered logs
```

**Why one service, not two?**
- Single configuration point (sinks, levels)
- Consistent formatting and context capture
- Error tracking works across modes
- File logging always works (mode just controls timing)

**Trade-off:** Logs may be lost if simulation crashes mid-step. For safety-critical logging, add a dedicated "crash sink" that bypasses buffering.

### 8. Automatic Context Capture

The Simulator sets thread-local context before calling component methods:

```cpp
for (auto& comp : components_) {
    LogContextManager::ScopedContext ctx(
        comp->Entity(),    // "falcon9_1"
        comp->Name(),      // "merlinEngine2"
        comp->TypeName()   // "LiquidEngine"
    );
    comp->Step(time_, dt);
}
```

**Result:** Components use `ICARUS_WARN(t, msg)` and the log automatically includes:
- Simulation time
- Entity path: `falcon9_1.merlinEngine2`
- Component type
- Log level

Components never need to pass context manually.

### 9. Error Integration with Error.hpp

The `ErrorHandler` bridges three systems:

| System | Role |
|:-------|:-----|
| `LogBuffer` | Collects error logs during Step |
| `Error.hpp` | Exception hierarchy for thrown errors |
| `ErrorPolicy` | Determines simulation response |

**Flow:**

1. Component throws `StepError` or logs `ICARUS_ERROR`
2. `ErrorHandler` catches/detects and applies policy
3. Policy determines: Continue, Pause, Abort, or Crash
4. Error counts tracked for max-error abort

**Why not just exceptions?**

- Some errors are recoverable (warnings)
- Some should pause, not crash
- Need to track error counts across time
- Buffered logging means errors detected post-step

### 10. Entity Grouping for Multi-Vehicle Sims

For simulations with multiple entities (e.g., two Falcon 9 vehicles), logs can be grouped by entity:

```
─── falcon9_1 ───────────────────────────────────────────────────────────────
[00.150] [INFO] [merlinEngine1] Ignition sequence started
[00.150] [INFO] [merlinEngine2] Ignition sequence started
[00.200] [WARN] [merlinEngine2] Chamber pressure 105% of nominal

─── falcon9_2 ───────────────────────────────────────────────────────────────
[00.150] [INFO] [merlinEngine1] Ignition sequence started
```

This is enabled via the `ConsoleGrouped` sink, which:
1. Collects all logs from the step
2. Sorts by entity, then by time
3. Prints entity headers between groups

---

## Example Output

### Startup and Provision

```
################################################################################
#                                                                              #
#    ██╗ ██████╗ █████╗ ██████╗ ██╗   ██╗███████╗                              #
#    ██║██╔════╝██╔══██╗██╔══██╗██║   ██║██╔════╝                              #
#    ██║██║     ███████║██████╔╝██║   ██║███████╗                              #
#    ██║██║     ██╔══██║██╔══██╗██║   ██║╚════██║                              #
#    ██║╚██████╗██║  ██║██║  ██║╚██████╔╝███████║                              #
#    ╚═╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝                              #
#                                                                              #
#    6DOF FLIGHT DYNAMICS ENGINE | BUILD: RELEASE | MODE: DOUBLE               #
################################################################################

[00.000] [SYS] Initializing Kernel...
[00.002] [MEM] Allocating Backplane

─── [ PHASE: PROVISION ] ───────────────────────────────────────────────────────
[00.010] [LOD] Loading Components:
         ├─ [CMP] Gravity          (PointMassGravity)
         └─ [CMP] PointMass3DOF    (PointMass3DOF)
[00.015] [MEM] State Vector Allocation:
         └─ Continuous States (x): 6

================================================================================
  ICARUS SIMULATION ENGINE | DATA DICTIONARY
  Generated: 2024-01-15 10:30:00 | Ver: 0.1.0-dev
================================================================================

[ SYSTEM SUMMARY ]
  Total Components: 2        Total Outputs: 7
  Total Inputs:     3        Total Params:  1
  Integrable States: 6       Config Values: 0

--------------------------------------------------------------------------------
COMPONENT: Gravity
  Type: PointMassGravity
--------------------------------------------------------------------------------
  [OUTPUTS]
  ┌────────────────────────┬─────────┬─────────────────────────────────────┐
  │ SIGNAL NAME            │ UNIT    │ DESCRIPTION                         │
  ├────────────────────────┼─────────┼─────────────────────────────────────┤
  │ force.x                │ N       │ Gravity force (x)                   │
  │ force.y                │ N       │ Gravity force (y)                   │
  │ force.z                │ N       │ Gravity force (z)                   │
  └────────────────────────┴─────────┴─────────────────────────────────────┘
```

### Stage (Wiring)

```
─── [ PHASE: STAGE ] ───────────────────────────────────────────────────────────
[00.020] [WIR] Resolving Signal Graph...
         ├─ Gravity.force.x      >> PointMass3DOF.force.x
         ├─ Gravity.force.y      >> PointMass3DOF.force.y
         └─ Gravity.force.z      >> PointMass3DOF.force.z
[00.022] [CHK] All inputs wired successfully.
```

### Run (Progress)

```
─── [ PHASE: RUN ] ─────────────────────────────────────────────────────────────
[00.025] [SIM] Simulation started. dt=0.01s, t_max=5.00s
[RUN] Time: 4.52s | Progress: [████████████████████░░░░░] 90% | Alt: -87.2m
```

### Shutdown (Debrief)

```
─── [ PHASE: SHUTDOWN ] ────────────────────────────────────────────────────────
[05.025] [SIM] End condition met: t >= t_max
[05.026] [IO ] Resources released.

================================================================================
  MISSION DEBRIEF
================================================================================
  Exit Status:      SUCCESS
  Exit Reason:      End condition met (t >= t_max)
  Simulated Time:   5.00 s
  Wall Clock Time:  0.03 s
  Real-Time Factor: 166.7x (Faster than real-time)

  [ PROFILE HOTSPOTS ]
  ┌──────────────────────┬─────────────┬──────────┐
  │ COMPONENT            │ AVG (μs)    │ % LOAD   │
  ├──────────────────────┼─────────────┼──────────┤
  │ PointMass3DOF        │ 1.2         │ 60.0%    │
  │ Gravity              │ 0.8         │ 40.0%    │
  └──────────────────────┴─────────────┴──────────┘

  "Icarus flew... and this time, he stuck the landing."
================================================================================
```

---

## Future Extensions (Out of Scope)

- Structured logging (JSON format for log aggregation)
- Remote logging (UDP/TCP sink)
- Log rotation and compression
- Custom log formats via format strings
- Interactive TUI mode (ncurses)
