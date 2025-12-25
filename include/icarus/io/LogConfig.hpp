#pragma once

/**
 * @file LogConfig.hpp
 * @brief Logging configuration structure
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 */

#include <icarus/io/Console.hpp>

#include <string>

namespace icarus {

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

} // namespace icarus
