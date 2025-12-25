#pragma once

/**
 * @file Console.hpp
 * @brief Console abstraction with ANSI color support
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 * Provides terminal-aware output with ANSI escape code support and box-drawing.
 */

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

#ifdef _WIN32
#include <io.h>
#define isatty _isatty
#define STDOUT_FILENO 1
#else
#include <unistd.h>
#endif

namespace icarus {

// =============================================================================
// LogLevel
// =============================================================================

/**
 * @brief Log severity levels
 */
enum class LogLevel {
    Trace,   ///< Most verbose, internal debugging
    Debug,   ///< Debugging info
    Info,    ///< Normal operation
    Event,   ///< Simulation events (phase changes, etc.)
    Warning, ///< Potential issues
    Error,   ///< Recoverable errors
    Fatal    ///< Unrecoverable errors
};

// =============================================================================
// AnsiColor
// =============================================================================

/**
 * @brief ANSI color codes
 */
struct AnsiColor {
    static constexpr const char *Reset = "\033[0m";
    static constexpr const char *Bold = "\033[1m";
    static constexpr const char *Dim = "\033[2m";

    // Foreground colors
    static constexpr const char *Red = "\033[31m";
    static constexpr const char *Green = "\033[32m";
    static constexpr const char *Yellow = "\033[33m";
    static constexpr const char *Blue = "\033[34m";
    static constexpr const char *Magenta = "\033[35m";
    static constexpr const char *Cyan = "\033[36m";
    static constexpr const char *White = "\033[37m";
    static constexpr const char *Gray = "\033[90m";

    // Background colors
    static constexpr const char *BgRed = "\033[41m";
    static constexpr const char *BgGreen = "\033[42m";
};

// =============================================================================
// BoxChars
// =============================================================================

/**
 * @brief Box-drawing characters (Unicode)
 */
struct BoxChars {
    // Single-line box drawing
    static constexpr const char *TopLeft = "\u250C";     // ┌
    static constexpr const char *TopRight = "\u2510";    // ┐
    static constexpr const char *BottomLeft = "\u2514";  // └
    static constexpr const char *BottomRight = "\u2518"; // ┘
    static constexpr const char *Horizontal = "\u2500";  // ─
    static constexpr const char *Vertical = "\u2502";    // │
    static constexpr const char *TeeRight = "\u251C";    // ├
    static constexpr const char *TeeLeft = "\u2524";     // ┤
    static constexpr const char *TeeDown = "\u252C";     // ┬
    static constexpr const char *TeeUp = "\u2534";       // ┴
    static constexpr const char *Cross = "\u253C";       // ┼

    // Tree characters
    static constexpr const char *TreeBranch = "\u251C"; // ├
    static constexpr const char *TreeLast = "\u2514";   // └
    static constexpr const char *TreePipe = "\u2502";   // │

    // Heavy line (for headers)
    static constexpr const char *HeavyHoriz = "\u2550"; // ═
};

// =============================================================================
// Console
// =============================================================================

/**
 * @brief Console output with color and formatting support
 *
 * Detects if stdout is a terminal and enables/disables ANSI colors accordingly.
 */
class Console {
  public:
    Console() : is_tty_(isatty(STDOUT_FILENO) != 0), color_enabled_(is_tty_) {}

    /// Check if stdout is a terminal (supports ANSI codes)
    [[nodiscard]] bool IsTerminal() const { return is_tty_; }

    /// Enable/disable color output (auto-detected by default)
    void SetColorEnabled(bool enabled) { color_enabled_ = enabled; }
    [[nodiscard]] bool IsColorEnabled() const { return color_enabled_; }

    /// Set minimum log level for output
    void SetLogLevel(LogLevel level) { min_level_ = level; }
    [[nodiscard]] LogLevel GetLogLevel() const { return min_level_; }

    // === Logging Methods ===

    void Trace(std::string_view msg) { Log(LogLevel::Trace, msg); }
    void Debug(std::string_view msg) { Log(LogLevel::Debug, msg); }
    void Info(std::string_view msg) { Log(LogLevel::Info, msg); }
    void Event(std::string_view msg) { Log(LogLevel::Event, msg); }
    void Warning(std::string_view msg) { Log(LogLevel::Warning, msg); }
    void Error(std::string_view msg) { Log(LogLevel::Error, msg); }
    void Fatal(std::string_view msg) { Log(LogLevel::Fatal, msg); }

    /// Log with explicit level
    void Log(LogLevel level, std::string_view msg) {
        if (level < min_level_) {
            return;
        }

        std::string output;
        if (color_enabled_) {
            output = std::string(GetLevelColor(level)) + std::string(GetLevelPrefix(level)) +
                     std::string(AnsiColor::Reset) + " " + std::string(msg);
        } else {
            output = std::string(GetLevelPrefix(level)) + " " + std::string(msg);
        }

        std::cout << output << "\n";
    }

    /// Log with timestamp prefix
    void LogTimed(LogLevel level, double sim_time, std::string_view msg) {
        if (level < min_level_) {
            return;
        }

        std::ostringstream oss;
        oss << "[" << std::fixed << std::setprecision(3) << sim_time << "] ";

        std::string output;
        if (color_enabled_) {
            output = oss.str() + std::string(GetLevelColor(level)) +
                     std::string(GetLevelPrefix(level)) + std::string(AnsiColor::Reset) + " " +
                     std::string(msg);
        } else {
            output = oss.str() + std::string(GetLevelPrefix(level)) + " " + std::string(msg);
        }

        std::cout << output << "\n";
    }

    // === Formatting Helpers ===

    /// Apply color if enabled
    [[nodiscard]] std::string Colorize(std::string_view text, const char *color) const {
        if (!color_enabled_) {
            return std::string(text);
        }
        return std::string(color) + std::string(text) + AnsiColor::Reset;
    }

    /// Create horizontal rule
    [[nodiscard]] std::string HorizontalRule(int width = 80, char c = '-') const {
        return std::string(static_cast<std::size_t>(width), c);
    }

    /// Create horizontal rule with box-drawing character
    [[nodiscard]] std::string BoxHorizontalRule(int width = 80) const {
        std::string result;
        result.reserve(static_cast<std::size_t>(width) * 3); // UTF-8 can be 3 bytes per char
        for (int i = 0; i < width; ++i) {
            result += BoxChars::Horizontal;
        }
        return result;
    }

    /// Pad string to width (right-aligned)
    [[nodiscard]] static std::string PadRight(std::string_view text, std::size_t width) {
        if (text.size() >= width) {
            return std::string(text);
        }
        return std::string(text) + std::string(width - text.size(), ' ');
    }

    /// Pad string to width (left-aligned)
    [[nodiscard]] static std::string PadLeft(std::string_view text, std::size_t width) {
        if (text.size() >= width) {
            return std::string(text);
        }
        return std::string(width - text.size(), ' ') + std::string(text);
    }

    /// Pad string to width (center-aligned)
    [[nodiscard]] static std::string PadCenter(std::string_view text, std::size_t width) {
        if (text.size() >= width) {
            return std::string(text);
        }
        std::size_t padding = width - text.size();
        std::size_t left_pad = padding / 2;
        std::size_t right_pad = padding - left_pad;
        return std::string(left_pad, ' ') + std::string(text) + std::string(right_pad, ' ');
    }

    /// Format number with comma separators
    [[nodiscard]] static std::string FormatNumber(double value, int precision = 2) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }

    /// Write raw string (no formatting)
    void Write(std::string_view text) const { std::cout << text; }

    /// Write raw string with newline
    void WriteLine(std::string_view text = "") const { std::cout << text << "\n"; }

    /// Flush output
    void Flush() const { std::cout.flush(); }

  private:
    bool is_tty_ = false;
    bool color_enabled_ = false;
    LogLevel min_level_ = LogLevel::Info;

    [[nodiscard]] const char *GetLevelColor(LogLevel level) const {
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

    [[nodiscard]] static const char *GetLevelPrefix(LogLevel level) {
        switch (level) {
        case LogLevel::Trace:
            return "[TRC]";
        case LogLevel::Debug:
            return "[DBG]";
        case LogLevel::Info:
            return "[INF]";
        case LogLevel::Event:
            return "[EVT]";
        case LogLevel::Warning:
            return "[WRN]";
        case LogLevel::Error:
            return "[ERR]";
        case LogLevel::Fatal:
            return "[FTL]";
        }
        return "[???]";
    }
};

} // namespace icarus
