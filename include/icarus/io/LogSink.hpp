#pragma once

/**
 * @file LogSink.hpp
 * @brief Pre-built log sinks for common output destinations
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 */

#include <icarus/io/Console.hpp>
#include <icarus/io/LogEntry.hpp>
#include <icarus/io/LogService.hpp>

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace icarus {

/**
 * @brief Factory for common log sinks
 */
class LogSinks {
  public:
    /// Console sink with colors (respects TTY detection)
    static LogService::Sink Console(const class Console &console) {
        return [&console](const std::vector<LogEntry> &entries) {
            for (const auto &entry : entries) {
                if (console.IsColorEnabled()) {
                    std::cout << entry.FormatColored(console) << "\n";
                } else {
                    std::cout << entry.Format() << "\n";
                }
            }
            std::cout.flush();
        };
    }

    /// Console sink with entity grouping
    /// Groups logs by entity, prints entity header before each group
    static LogService::Sink ConsoleGrouped(const class Console &console) {
        return [&console](const std::vector<LogEntry> &entries) {
            // Group by entity
            std::map<std::string, std::vector<const LogEntry *>> grouped;
            for (const auto &entry : entries) {
                grouped[entry.context.entity].push_back(&entry);
            }

            // Print each group
            for (const auto &[entity, group_entries] : grouped) {
                // Entity header
                std::string header = "─── " + (entity.empty() ? "(no entity)" : entity) + " ";
                std::size_t remaining = 80 - header.size();
                for (std::size_t i = 0; i < remaining; ++i) {
                    header += "─";
                }

                if (console.IsColorEnabled()) {
                    std::cout << console.Colorize(header, AnsiColor::Dim) << "\n";
                } else {
                    std::cout << header << "\n";
                }

                // Entries
                for (const auto *entry : group_entries) {
                    if (console.IsColorEnabled()) {
                        std::cout << entry->FormatColored(console) << "\n";
                    } else {
                        std::cout << entry->Format() << "\n";
                    }
                }
            }
            std::cout.flush();
        };
    }

    /// File sink (plain text, no colors)
    static LogService::Sink File(const std::string &path) {
        // Use shared_ptr to keep file open across calls
        auto file = std::make_shared<std::ofstream>(path, std::ios::app);
        return [file](const std::vector<LogEntry> &entries) {
            if (!file->is_open()) {
                return;
            }
            for (const auto &entry : entries) {
                *file << entry.Format() << "\n";
            }
            file->flush();
        };
    }

    /// File sink with rotation
    static LogService::Sink RotatingFile(const std::string &path, std::size_t max_size_bytes,
                                         std::size_t max_files) {
        struct State {
            std::string base_path;
            std::size_t max_size;
            std::size_t max_files;
            std::size_t current_size = 0;
            std::shared_ptr<std::ofstream> file;
        };

        auto state = std::make_shared<State>();
        state->base_path = path;
        state->max_size = max_size_bytes;
        state->max_files = max_files;
        state->file = std::make_shared<std::ofstream>(path, std::ios::app);

        return [state](const std::vector<LogEntry> &entries) {
            if (!state->file->is_open()) {
                return;
            }

            for (const auto &entry : entries) {
                std::string line = entry.Format() + "\n";
                *state->file << line;
                state->current_size += line.size();

                // Check for rotation
                if (state->current_size >= state->max_size) {
                    state->file->close();

                    // Rotate files
                    for (std::size_t i = state->max_files - 1; i > 0; --i) {
                        std::string old_name = state->base_path + "." + std::to_string(i);
                        std::string new_name = state->base_path + "." + std::to_string(i + 1);
                        std::rename(old_name.c_str(), new_name.c_str());
                    }
                    std::rename(state->base_path.c_str(), (state->base_path + ".1").c_str());

                    // Open new file
                    state->file = std::make_shared<std::ofstream>(state->base_path, std::ios::app);
                    state->current_size = 0;
                }
            }
            state->file->flush();
        };
    }

    /// JSON Lines sink (for log aggregation)
    static LogService::Sink JsonLines(const std::string &path) {
        auto file = std::make_shared<std::ofstream>(path, std::ios::app);
        return [file](const std::vector<LogEntry> &entries) {
            if (!file->is_open()) {
                return;
            }
            for (const auto &entry : entries) {
                // Simple JSON formatting (no external deps)
                *file << "{\"time\":" << entry.sim_time << ",\"level\":\""
                      << GetLevelString(entry.level) << "\",\"entity\":\""
                      << EscapeJson(entry.context.entity) << "\",\"component\":\""
                      << EscapeJson(entry.context.component) << "\",\"message\":\""
                      << EscapeJson(entry.message) << "\"}\n";
            }
            file->flush();
        };
    }

    /// Null sink (for testing/benchmarking)
    static LogService::Sink Null() {
        return [](const std::vector<LogEntry> & /*entries*/) {
            // Do nothing
        };
    }

    /// Callback sink (custom handling)
    static LogService::Sink Callback(std::function<void(const LogEntry &)> handler) {
        return [handler = std::move(handler)](const std::vector<LogEntry> &entries) {
            for (const auto &entry : entries) {
                handler(entry);
            }
        };
    }

  private:
    [[nodiscard]] static const char *GetLevelString(LogLevel level) {
        switch (level) {
        case LogLevel::Trace:
            return "TRACE";
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Event:
            return "EVENT";
        case LogLevel::Warning:
            return "WARNING";
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Fatal:
            return "FATAL";
        }
        return "UNKNOWN";
    }

    [[nodiscard]] static std::string EscapeJson(const std::string &s) {
        std::string result;
        result.reserve(s.size());
        for (char c : s) {
            switch (c) {
            case '"':
                result += "\\\"";
                break;
            case '\\':
                result += "\\\\";
                break;
            case '\n':
                result += "\\n";
                break;
            case '\r':
                result += "\\r";
                break;
            case '\t':
                result += "\\t";
                break;
            default:
                result += c;
                break;
            }
        }
        return result;
    }
};

} // namespace icarus
