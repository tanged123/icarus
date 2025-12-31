#pragma once

/**
 * @file FlightManifest.hpp
 * @brief ASCII Data Dictionary formatter
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 * Generates the Flight Manifest (Data Dictionary) in ASCII format.
 */

#include <icarus/io/data/DataDictionary.hpp>
#include <icarus/io/log/Console.hpp>
#include <icarus/io/report/AsciiTable.hpp>
#include <icarus/io/report/Banner.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

namespace icarus {

/**
 * @brief Generate Flight Manifest (ASCII Data Dictionary)
 *
 * Called at the end of Provision phase. Behavior:
 * - Full manifest -> file (signal_dictionary.dict)
 * - Summary only → console (counts, unwired inputs, warnings)
 *
 * The full manifest can be thousands of signals, so it's never
 * dumped to console by default.
 */
class FlightManifest {
  public:
    explicit FlightManifest(const Console &console) : console_(console) {}

    // === Configuration ===

    /// Set output file path (default: "signal_dictionary.dict")
    void SetOutputPath(const std::filesystem::path &path) { output_path_ = path; }

    /// Set simulation version string
    void SetVersion(const std::string &version) { version_ = version; }

    /// Set config source for a component
    void SetConfigSource(const std::string &component, const std::string &source) {
        config_sources_[component] = source;
    }

    /// Enable/disable full console output (default: false, summary only)
    void SetFullConsoleOutput(bool enabled) { full_console_ = enabled; }

    // === Generation ===

    /// Generate manifest: write full to file, summary to console
    void Generate(const DataDictionary &dict) {
        WriteToFile(dict);
        PrintSummary(dict);
    }

    /// Generate full manifest string (for file or explicit request)
    [[nodiscard]] std::string GenerateFull(const DataDictionary &dict) const {
        std::ostringstream oss;

        // Header
        oss << GenerateHeader();
        oss << "\n\n";

        // System summary
        oss << "[ SYSTEM SUMMARY ]\n";
        oss << "  Total Components: " << std::setw(4) << dict.components.size()
            << "        Total Outputs: " << dict.total_outputs << "\n";
        oss << "  Total Inputs:     " << std::setw(4) << dict.total_inputs
            << "        Total Params:  " << dict.total_parameters << "\n";
        oss << "  Integrable States: " << std::setw(3) << dict.integrable_states
            << "        Config Values: " << dict.total_config << "\n";
        oss << "\n";

        // Component sections
        for (const auto &comp : dict.components) {
            oss << GenerateComponentSection(comp);
            oss << "\n";
        }

        return oss.str();
    }

    /// Generate summary string (for console)
    [[nodiscard]] std::string GenerateSummary(const DataDictionary &dict) const {
        std::ostringstream oss;

        oss << Banner::GetSectionHeader("FLIGHT MANIFEST") << "\n";
        oss << "Signal Dictionary written to: " << output_path_.string() << "\n\n";

        oss << "  Components:    " << std::setw(4) << dict.components.size()
            << "          Outputs:     " << std::setw(4) << dict.total_outputs << "\n";
        oss << "  Inputs:        " << std::setw(4) << dict.total_inputs
            << "          Parameters:  " << std::setw(4) << dict.total_parameters << "\n";
        oss << "  States:        " << std::setw(4) << dict.integrable_states
            << "          Config:      " << std::setw(4) << dict.total_config << "\n";

        // Unwired inputs warning
        if (dict.unwired_inputs > 0) {
            oss << "\n  ⚠ Unwired inputs: " << dict.unwired_inputs << "\n";
            for (const auto &comp : dict.components) {
                for (const auto &input : comp.inputs) {
                    if (input.wired_to.empty()) {
                        oss << "    - " << comp.name << "." << input.name << "\n";
                    }
                }
            }
        }

        oss << "\nSee " << output_path_.string() << " for full signal listing.\n";
        oss << std::string(80, '-') << "\n";

        return oss.str();
    }

    /// Write full manifest to file
    void WriteToFile(const DataDictionary &dict) const {
        std::ofstream file(output_path_);
        if (!file.is_open()) {
            std::cerr << "[ERR] Failed to write manifest to: " << output_path_ << "\n";
            return;
        }
        file << GenerateFull(dict);
        file.close();
    }

    /// Print summary to console (default behavior)
    void PrintSummary(const DataDictionary &dict) const {
        std::string summary = GenerateSummary(dict);
        if (console_.IsColorEnabled()) {
            // Colorize warnings (⚠ Unwired inputs) in yellow
            const std::string warning_marker = "⚠ Unwired";
            std::size_t pos = summary.find(warning_marker);
            if (pos != std::string::npos) {
                // Find the end of the warning section (next blank line or end)
                std::size_t end_pos = summary.find("\n\nSee ", pos);
                if (end_pos == std::string::npos) {
                    end_pos = summary.length();
                }
                // Insert ANSI yellow before warning, reset after
                std::string colored = summary.substr(0, pos);
                colored += console_.Colorize(summary.substr(pos, end_pos - pos), AnsiColor::Yellow);
                colored += summary.substr(end_pos);
                std::cout << colored;
            } else {
                std::cout << summary;
            }
        } else {
            std::cout << summary;
        }
    }

    /// Print full manifest to console (opt-in, use sparingly)
    void PrintFull(const DataDictionary &dict) const { std::cout << GenerateFull(dict); }

  private:
    const Console &console_;
    std::filesystem::path output_path_ = "signal_dictionary.dict";
    std::string version_ = icarus::Version();
    std::map<std::string, std::string> config_sources_;
    bool full_console_ = false;

    [[nodiscard]] std::string GenerateHeader() const {
        std::ostringstream oss;

        // Get current time
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &time_t);
#else
        localtime_r(&time_t, &tm_buf);
#endif

        oss << Banner::GetRule(80) << "\n";
        oss << "  ICARUS SIMULATION ENGINE | DATA DICTIONARY\n";
        oss << "  Generated: " << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
        oss << " | Ver: " << version_ << "\n";
        oss << Banner::GetRule(80);

        return oss.str();
    }

    [[nodiscard]] std::string
    GenerateComponentSection(const DataDictionary::ComponentEntry &comp) const {
        std::ostringstream oss;

        oss << std::string(80, '-') << "\n";
        oss << "COMPONENT: " << comp.name << "\n";
        oss << "  Type: " << comp.type << "\n";

        auto it = config_sources_.find(comp.name);
        if (it != config_sources_.end()) {
            oss << "  Source: " << it->second << "\n";
        }

        oss << std::string(80, '-') << "\n";

        if (!comp.outputs.empty()) {
            oss << GenerateSignalTable("OUTPUTS", comp.outputs);
        }
        if (!comp.inputs.empty()) {
            oss << GenerateSignalTable("INPUTS", comp.inputs);
        }
        if (!comp.parameters.empty()) {
            oss << GenerateSignalTable("PARAMETERS", comp.parameters);
        }
        if (!comp.config.empty()) {
            oss << GenerateSignalTable("CONFIG", comp.config);
        }

        return oss.str();
    }

    [[nodiscard]] std::string
    GenerateSignalTable(const std::string &title,
                        const std::vector<SignalDescriptor> &signals) const {
        std::ostringstream oss;

        oss << "  [" << title << "]\n";

        AsciiTable table;
        table.AddColumn("SIGNAL NAME", 40);
        table.AddColumn("UNIT", 12);
        table.AddColumn("DESCRIPTION", 80);

        for (const auto &sig : signals) {
            table.AddRow({sig.name, sig.unit, sig.description});
        }

        // Indent the table
        std::istringstream iss(table.Render());
        std::string line;
        while (std::getline(iss, line)) {
            oss << "  " << line << "\n";
        }

        return oss.str();
    }
};

} // namespace icarus
