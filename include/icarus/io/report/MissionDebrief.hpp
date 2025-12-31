#pragma once

/**
 * @file MissionDebrief.hpp
 * @brief Shutdown statistics and profiling report
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 */

#include <icarus/io/log/Console.hpp>
#include <icarus/io/report/AsciiTable.hpp>
#include <icarus/io/report/Banner.hpp>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Exit status codes
 */
enum class ExitStatus {
    Success,         ///< Normal completion
    EndConditionMet, ///< End condition triggered
    UserAbort,       ///< User requested abort
    Error,           ///< Error during simulation
    Divergence       ///< Numerical divergence
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
 * @brief Mission Debrief generator
 *
 * Generates the shutdown statistics and profiling summary.
 */
class MissionDebrief {
  public:
    explicit MissionDebrief(const Console &console) : console_(console) {}

    /// Set exit status
    void SetExitStatus(ExitStatus status) { exit_status_ = status; }

    /// Set exit reason message
    void SetExitReason(const std::string &reason) { exit_reason_ = reason; }

    /// Set timing information
    void SetTiming(double sim_time, double wall_time) {
        sim_time_ = sim_time;
        wall_time_ = wall_time;
        if (wall_time > 0) {
            real_time_factor_ = sim_time / wall_time;
        }
    }

    /// Set profiling data
    void SetProfilingData(const std::vector<ComponentStats> &stats) { profiling_data_ = stats; }

    /// Generate the full debrief string
    [[nodiscard]] std::string Generate() const {
        std::ostringstream oss;

        oss << Banner::GetDebriefHeader() << "\n";

        // Exit status
        oss << "  Exit Status:      " << StatusToString(exit_status_) << "\n";
        if (!exit_reason_.empty()) {
            oss << "  Exit Reason:      " << exit_reason_ << "\n";
        }

        // Timing
        oss << "  Sim Time:         " << std::fixed << std::setprecision(2) << sim_time_ << " s\n";
        oss << "  Wall Time:        " << std::scientific << std::setprecision(2) << wall_time_
            << " s\n";

        // Real-time factor
        if (wall_time_ > 0) {
            oss << "  Real-Time Factor: " << std::fixed << std::setprecision(1) << real_time_factor_
                << "x ";
            if (real_time_factor_ > 1.0) {
                oss << "(Faster than real-time)\n";
            } else if (real_time_factor_ < 1.0) {
                oss << "(Slower than real-time)\n";
            } else {
                oss << "(Real-time)\n";
            }
        }

        // Profiling data
        if (!profiling_data_.empty()) {
            oss << "\n" << GenerateProfilingTable();
        }

        oss << Banner::GetRule(80) << "\n";

        return oss.str();
    }

    /// Generate and print to console
    void Print() const {
        std::string debrief = Generate();
        if (console_.IsColorEnabled()) {
            // Color the status based on success/failure
            if (exit_status_ == ExitStatus::Success ||
                exit_status_ == ExitStatus::EndConditionMet) {
                std::cout << console_.Colorize(debrief, AnsiColor::Green);
            } else {
                std::cout << console_.Colorize(debrief, AnsiColor::Red);
            }
        } else {
            std::cout << debrief;
        }
    }

  private:
    const Console &console_;

    ExitStatus exit_status_ = ExitStatus::Success;
    std::string exit_reason_;
    double sim_time_ = 0.0;
    double wall_time_ = 0.0;
    double real_time_factor_ = 0.0;
    std::vector<ComponentStats> profiling_data_;

    [[nodiscard]] static std::string StatusToString(ExitStatus status) {
        switch (status) {
        case ExitStatus::Success:
            return "SUCCESS";
        case ExitStatus::EndConditionMet:
            return "END CONDITION MET";
        case ExitStatus::UserAbort:
            return "USER ABORT";
        case ExitStatus::Error:
            return "ERROR";
        case ExitStatus::Divergence:
            return "DIVERGENCE";
        }
        return "UNKNOWN";
    }

    [[nodiscard]] std::string GenerateProfilingTable() const {
        std::ostringstream oss;

        oss << "  [ PROFILE HOTSPOTS ]\n";

        // Sort by percent load (descending)
        std::vector<ComponentStats> sorted = profiling_data_;
        std::sort(sorted.begin(), sorted.end(),
                  [](const ComponentStats &a, const ComponentStats &b) {
                      return a.percent_load > b.percent_load;
                  });

        AsciiTable table;
        table.AddColumn("COMPONENT", 20);
        table.AddColumn("AVG (μs)", 12, AsciiTable::Align::Right);
        table.AddColumn("% LOAD", 8, AsciiTable::Align::Right);

        for (const auto &stat : sorted) {
            std::ostringstream avg;
            avg << std::fixed << std::setprecision(1) << stat.avg_time_us;
            std::ostringstream load;
            load << std::fixed << std::setprecision(1) << stat.percent_load << "%";
            table.AddRow({stat.name, avg.str(), load.str()});
        }

        // Indent the table
        std::istringstream iss(table.Render());
        std::string line;
        while (std::getline(iss, line)) {
            oss << "  " << line << "\n";
        }

        return oss.str();
    }

    [[nodiscard]] std::string GenerateQuote() const {
        switch (exit_status_) {
        case ExitStatus::Success:
        case ExitStatus::EndConditionMet:
            return "  \"Icarus flew... and this time, he stuck the landing.\"";
        case ExitStatus::UserAbort:
            return "  \"Mission aborted. Sometimes discretion is the better part of valor.\"";
        case ExitStatus::Error:
            return "  \"Houston, we have a problem.\"";
        case ExitStatus::Divergence:
            return "  \"The numbers went to infinity... and beyond.\"";
        }
        return "";
    }
};

} // namespace icarus
