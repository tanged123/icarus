#pragma once

/**
 * @file MissionLogger.hpp
 * @brief Flight Recorder style logging service
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 * Provides structured logging throughout the simulation lifecycle.
 */

#include <icarus/core/Types.hpp>
#include <icarus/io/Banner.hpp>
#include <icarus/io/Console.hpp>
#include <icarus/io/DataDictionary.hpp>
#include <icarus/io/FlightManifest.hpp>
#include <icarus/io/MissionDebrief.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/staging/StagingTypes.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Simulation phase names for logging banners
 *
 * These are string constants used with MissionLogger::BeginPhase().
 * Separate from the Phase enum in Types.hpp which tracks simulation state.
 */
namespace SimPhase {
constexpr const char *Init = "INIT";
constexpr const char *Provision = "PROVISION";
constexpr const char *Stage = "STAGE";
constexpr const char *Run = "RUN";
constexpr const char *Shutdown = "SHUTDOWN";
} // namespace SimPhase

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

    MissionLogger() : startup_time_(Clock::now()) {}

    explicit MissionLogger(const std::string &log_file_path)
        : startup_time_(Clock::now()), log_file_(log_file_path), log_file_name_(log_file_path) {}

    // === Configuration ===

    /// Set console log level (terminal output)
    void SetConsoleLevel(LogLevel level) { console_.SetLogLevel(level); }

    /// Set file log level (log file output)
    void SetFileLevel(LogLevel level) { file_level_ = level; }

    /// Set simulation version string
    void SetVersion(const std::string &version) { version_ = version; }

    /// Set build type (DEBUG/RELEASE) - auto-detected by default
    void SetBuildType(const std::string &build_type) { build_type_ = build_type; }

    /// Enable/disable progress display during Run
    void SetProgressEnabled(bool enabled) { progress_enabled_ = enabled; }

    /// Enable/disable profiling
    void SetProfilingEnabled(bool enabled) { profiling_enabled_ = enabled; }

    /// Get current log file name
    [[nodiscard]] std::string GetLogFileName() const { return log_file_name_; }

    /// Get current console level
    [[nodiscard]] LogLevel GetConsoleLevel() const { return console_.GetLogLevel(); }

    /// Check if progress is enabled
    [[nodiscard]] bool IsProgressEnabled() const { return progress_enabled_; }

    /// Check if profiling is enabled
    [[nodiscard]] bool IsProfilingEnabled() const { return profiling_enabled_; }

    /// Get total wall time elapsed since startup
    [[nodiscard]] std::chrono::nanoseconds WallElapsed() const {
        return Clock::now() - startup_time_;
    }

    /// Set log file path (closes existing if open)
    void SetLogFile(const std::string &path) {
        if (log_file_.is_open()) {
            log_file_.close();
        }
        log_file_name_ = path;
        if (!path.empty()) {
            log_file_.open(path);
        }
    }

    // === Lifecycle Logging ===

    /// Log simulation startup (splash screen) - uses Icarus engine version
    void LogStartup() {
        std::string splash = Banner::GetSplashScreen(build_type_, icarus::Version());
        console_.WriteLine(splash);
        WriteToFile(splash);
    }

    /// Log simulation configuration info (from YAML)
    void LogSimulationConfig(const std::string &name, const std::string &version,
                             const std::string &description = "") {
        std::ostringstream oss;
        oss << "[SIM] Simulation: " << name << " (v" << version << ")";
        Log(LogLevel::Info, oss.str());

        if (!description.empty()) {
            // Log first line of description
            std::string first_line = description;
            auto pos = first_line.find('\n');
            if (pos != std::string::npos) {
                first_line = first_line.substr(0, pos);
            }
            if (!first_line.empty()) {
                Log(LogLevel::Debug, "         " + first_line);
            }
        }
    }

    /// Log configuration file path
    void LogConfigFile(const std::string &path) { Log(LogLevel::Info, "[CFG] Config: " + path); }

    /// Log time configuration
    void LogTimeConfig(double t_start, double t_end, double dt) {
        std::ostringstream oss;
        oss << "[CFG] Time: start=" << std::fixed << std::setprecision(1) << t_start
            << "s, end=" << t_end << "s, dt=" << std::setprecision(4) << dt << "s";
        Log(LogLevel::Info, oss.str());
    }

    /// Log integrator type
    void LogIntegrator(const std::string &type) {
        Log(LogLevel::Debug, "[CFG] Integrator: " + type);
    }

    /// Log integrator type (enum version)
    void LogIntegrator(IntegratorType type) { LogIntegrator(to_string(type)); }

    /// Begin a lifecycle phase
    void BeginPhase(const char *phase) {
        current_phase_ = phase;
        phase_start_time_ = Clock::now();

        std::string header = Banner::GetPhaseHeader(phase);
        console_.WriteLine(header);
        WriteToFile(header);
    }

    /// End current phase (pass current sim time for consistent timestamps)
    void EndPhase(double sim_time = 0.0) {
        auto elapsed = std::chrono::duration<double>(Clock::now() - phase_start_time_).count();
        std::ostringstream oss;
        oss << "[SYS] " << current_phase_ << " phase complete (" << FormatDuration(elapsed) << ")";
        LogTimed(LogLevel::Info, sim_time, oss.str());
    }

    /// Get the signal dictionary output path (derived from log file name)
    [[nodiscard]] std::string GetDictionaryPath() const {
        if (log_file_name_.empty()) {
            return "signal_dictionary.dict";
        }
        // Extract base name without extension
        std::string base = log_file_name_;
        auto dot_pos = base.rfind('.');
        if (dot_pos != std::string::npos) {
            base = base.substr(0, dot_pos);
        }
        return base + "_signal_dictionary.dict";
    }

    /// Log the Flight Manifest (Data Dictionary)
    void LogManifest(const DataDictionary &dict) {
        FlightManifest manifest(console_);
        manifest.SetVersion(icarus::Version());
        manifest.SetOutputPath(GetDictionaryPath());
        manifest.Generate(dict);

        // Also write summary to log file
        WriteToFile(manifest.GenerateSummary(dict));
    }

    /// Log simulation run start (entering RUN phase)
    void LogRunStart(double t_start, double t_end, double dt) {
        run_start_wall_time_ = Clock::now();
        std::ostringstream oss;
        oss << "[SYS] Starting simulation (t=" << std::fixed << std::setprecision(3) << t_start
            << " → " << std::setprecision(1) << t_end << " s, dt=" << std::setprecision(4) << dt
            << " s)";
        LogTimed(LogLevel::Info, t_start, oss.str());
    }

    /// Log periodic run progress
    void LogRunProgress(double sim_time, double t_end) {
        double progress = (t_end > 0) ? (sim_time / t_end) * 100.0 : 0.0;
        auto wall_elapsed =
            std::chrono::duration<double>(Clock::now() - run_start_wall_time_).count();
        double rtf = (wall_elapsed > 0) ? sim_time / wall_elapsed : 0.0;

        std::ostringstream oss;
        oss << "[RUN] Progress: " << std::fixed << std::setprecision(1) << progress << "% ("
            << std::setprecision(1) << sim_time << "/" << t_end
            << " s, RTF: " << std::setprecision(1) << rtf << "x)";
        LogTimed(LogLevel::Trace, sim_time, oss.str());
    }

    /// Log mission debrief (shutdown statistics)
    void LogDebrief(double sim_time, double wall_time) {
        MissionDebrief debrief(console_);
        debrief.SetExitStatus(ExitStatus::EndConditionMet);
        debrief.SetExitReason("Reached end of simulation time");
        debrief.SetTiming(sim_time, wall_time);

        if (profiling_enabled_) {
            debrief.SetProfilingData(GetProfilingStats());
        }

        std::string output = debrief.Generate();
        console_.Write(output);
        WriteToFile(output);
    }

    // === Provision Phase Logging ===

    /// Log entity loading
    void LogEntityLoad(const std::string &entity_name) {
        LogTimed(LogLevel::Info, 0.0, "[LOD] Loading Entity: " + entity_name);
    }

    /// Log component addition (tree format)
    void LogComponentAdd(const std::string &component_name, const std::string &type,
                         const std::string &config_source = "defaults", bool is_last = false) {
        std::string prefix = is_last ? "└─" : "├─";
        std::ostringstream oss;
        oss << "         " << prefix << " [CMP] " << component_name << " (" << type << ")";
        if (config_source != "defaults") {
            oss << " [" << config_source << "]";
        }
        console_.WriteLine(oss.str());
        WriteToFile(oss.str());
    }

    /// Log static asset loading
    void LogAssetLoad(const std::string &asset_name, const std::string &description) {
        LogTimed(LogLevel::Debug, 0.0, "[AST] " + asset_name + ": " + description);
    }

    /// Log state vector allocation
    void LogStateAllocation(std::size_t continuous, std::size_t discrete = 0) {
        std::ostringstream oss;
        oss << "[MEM] States: " << continuous << " continuous";
        if (discrete > 0) {
            oss << ", " << discrete << " discrete";
        }
        LogTimed(LogLevel::Info, 0.0, oss.str());
    }

    // === Stage Phase Logging ===

    /// Log signal wiring
    void LogWiring(const std::string &source, const std::string &target, bool is_warning = false) {
        std::string prefix = is_warning ? "[!WIR]" : "[WIR]";
        std::ostringstream oss;
        oss << "         ├─ " << source << " >> " << target;
        if (is_warning) {
            console_.Warning(oss.str());
        } else {
            Log(LogLevel::Debug, oss.str());
        }
    }

    /// Log wiring warning (multiple writers, etc.)
    void LogWiringWarning(const std::string &message) {
        LogTimed(LogLevel::Warning, 0.0, "[WIR] " + message);
    }

    /// Log topological sort order
    void LogExecutionOrder(const std::vector<std::string> &component_order) {
        Log(LogLevel::Debug, "[ORD] Execution Order:");
        for (std::size_t i = 0; i < component_order.size(); ++i) {
            std::ostringstream oss;
            oss << "         " << (i + 1) << ". " << component_order[i];
            Log(LogLevel::Debug, oss.str());
        }
    }

    /// Log scheduler execution order with groups and rates (Debug level)
    void LogSchedulerOrder(double sim_rate_hz, const std::vector<SchedulerGroupConfig> &groups,
                           const std::unordered_map<std::string, int> &divisors) {
        std::ostringstream header;
        header << "[SCH] Scheduler (sim rate: " << sim_rate_hz << " Hz)";
        Log(LogLevel::Debug, header.str());

        for (std::size_t i = 0; i < groups.size(); ++i) {
            const auto &group = groups[i];
            auto it = divisors.find(group.name);
            int divisor = (it != divisors.end()) ? it->second : 1;
            double group_dt = 1.0 / group.rate_hz;

            std::ostringstream oss;
            bool is_last = (i == groups.size() - 1);
            oss << "         " << (is_last ? "└─" : "├─") << " [" << group.name << "] "
                << group.rate_hz << " Hz (÷" << divisor << ", dt=" << std::fixed
                << std::setprecision(6) << group_dt << "s)";
            Log(LogLevel::Debug, oss.str());

            for (std::size_t j = 0; j < group.members.size(); ++j) {
                const auto &member = group.members[j];
                std::ostringstream mem_oss;
                bool mem_last = (j == group.members.size() - 1);
                mem_oss << "         " << (is_last ? "   " : "│  ") << (mem_last ? "└─" : "├─")
                        << " " << member.component;
                Log(LogLevel::Debug, mem_oss.str());
            }
        }
    }

    /// Log trim solver progress
    void LogTrimStart(const std::string &mode,
                      const std::vector<std::pair<std::string, double>> &targets) {
        Log(LogLevel::Info, "[TRM] Trim Solver: " + mode);
        for (const auto &[name, value] : targets) {
            std::ostringstream oss;
            oss << "         └─ Target: " << name << " = " << value;
            Log(LogLevel::Debug, oss.str());
        }
    }

    void LogTrimIteration(int iteration, double residual) {
        std::ostringstream oss;
        oss << "[TRM] Iteration " << iteration << ": residual = " << std::scientific << residual;
        Log(LogLevel::Debug, oss.str());
    }

    void LogTrimConverged(int iterations) {
        std::ostringstream oss;
        oss << "[TRM] Converged in " << iterations << " iterations";
        Log(LogLevel::Info, oss.str());
    }

    void LogTrimFailed(const std::string &reason) {
        Log(LogLevel::Error, "[TRM] Trim failed: " + reason);
    }

    // === Linearization Logging ===

    /// Log linear model summary (called after linearization)
    void LogLinearModel(const staging::LinearModel &model) {
        // Banner
        Log(LogLevel::Info, "[LIN] ═══════════════════════════════════════════════════════");
        Log(LogLevel::Info, "[LIN]   Linear Model at Operating Point");
        Log(LogLevel::Info, "[LIN] ═══════════════════════════════════════════════════════");

        // Dimensions (Info level)
        {
            std::ostringstream oss;
            oss << "[LIN] State-Space: A[" << model.A.rows() << "x" << model.A.cols() << "], B["
                << model.B.rows() << "x" << model.B.cols() << "], C[" << model.C.rows() << "x"
                << model.C.cols() << "], D[" << model.D.rows() << "x" << model.D.cols() << "]";
            Log(LogLevel::Info, oss.str());
        }

        // Operating point (Debug level)
        Log(LogLevel::Debug, "[LIN] Operating Point (x0):");
        for (std::size_t i = 0; i < model.state_names.size(); ++i) {
            std::ostringstream oss;
            oss << "         " << model.state_names[i] << " = " << std::scientific
                << std::setprecision(4) << model.x0(static_cast<Eigen::Index>(i));
            Log(LogLevel::Debug, oss.str());
        }

        // A matrix (Trace level - very verbose)
        Log(LogLevel::Trace, "[LIN] A Matrix (df/dx):");
        for (Eigen::Index i = 0; i < model.A.rows(); ++i) {
            std::ostringstream oss;
            oss << "         [";
            for (Eigen::Index j = 0; j < model.A.cols(); ++j) {
                oss << std::fixed << std::setw(12) << std::setprecision(6) << model.A(i, j);
                if (j < model.A.cols() - 1)
                    oss << ", ";
            }
            oss << "]";
            Log(LogLevel::Trace, oss.str());
        }

        // Eigenvalue analysis (Info level - important for stability)
        Log(LogLevel::Info, "[LIN] Eigenvalue Analysis:");
        auto eigenvalues = model.Eigenvalues();
        for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
            double real = eigenvalues(i).real();
            double imag = eigenvalues(i).imag();

            std::ostringstream oss;
            oss << "         λ" << (i + 1) << " = " << std::scientific << std::setprecision(4)
                << real;
            if (std::abs(imag) > 1e-10) {
                oss << (imag >= 0 ? " + " : " - ") << std::abs(imag) << "i";
                // Compute period from imaginary part
                double period = 2.0 * M_PI / std::abs(imag);
                oss << std::fixed << std::setprecision(1) << "  (period: " << period
                    << " s = " << period / 60.0 << " min)";
            }
            Log(LogLevel::Info, oss.str());
        }

        // Stability and controllability (Info level)
        {
            std::ostringstream oss;
            oss << "[LIN] Stability: " << (model.IsStable() ? "STABLE" : "UNSTABLE")
                << " (Lyapunov sense)";
            Log(LogLevel::Info, oss.str());
        }
        {
            std::ostringstream oss;
            oss << "[LIN] Controllability: " << model.ControllabilityRank() << " / "
                << model.A.rows()
                << (model.ControllabilityRank() == model.A.rows() ? " (fully controllable)" : "");
            Log(LogLevel::Info, oss.str());
        }
        {
            std::ostringstream oss;
            oss << "[LIN] Observability: " << model.ObservabilityRank() << " / " << model.A.rows()
                << (model.ObservabilityRank() == model.A.rows() ? " (fully observable)" : "");
            Log(LogLevel::Info, oss.str());
        }

        Log(LogLevel::Info, "[LIN] ═══════════════════════════════════════════════════════");
    }

    // === Run Phase Logging ===

    /// Log phase transition event
    void LogPhaseEvent(const std::string &from_phase, const std::string &to_phase,
                       double sim_time) {
        LogTimed(LogLevel::Event, sim_time,
                 "[PHS] Phase transition: " + from_phase + " -> " + to_phase);
    }

    /// Log notable event (apogee, impact, etc.)
    void LogEvent(const std::string &event_name, double sim_time, const std::string &details = "") {
        std::string msg = "[EVT] " + event_name;
        if (!details.empty()) {
            msg += ": " + details;
        }
        LogTimed(LogLevel::Event, sim_time, msg);
    }

    /// Log warning during run
    void LogRunWarning(const std::string &source, const std::string &message, double sim_time) {
        LogTimed(LogLevel::Warning, sim_time, "[" + source + "] " + message);
    }

    /// Log error during run
    void LogRunError(const std::string &source, const std::string &message, double sim_time) {
        LogTimed(LogLevel::Error, sim_time, "[" + source + "] " + message);
    }

    /// Update progress display (single-line, \r overwrite)
    void UpdateProgress(double sim_time, double t_max,
                        const std::map<std::string, double> &key_values = {}) {
        if (!progress_enabled_ || !console_.IsTerminal()) {
            return;
        }

        double progress = (t_max > 0) ? (sim_time / t_max) : 0.0;
        int filled = static_cast<int>(progress * static_cast<double>(progress_width_));

        std::ostringstream oss;
        oss << "\r[RUN] Time: " << std::fixed << std::setprecision(2) << sim_time << "s | [";

        for (int i = 0; i < progress_width_; ++i) {
            if (i < filled) {
                oss << "█";
            } else {
                oss << "░";
            }
        }

        oss << "] " << static_cast<int>(progress * 100) << "%";

        for (const auto &[key, value] : key_values) {
            oss << " | " << key << ": " << std::fixed << std::setprecision(1) << value;
        }

        std::cout << oss.str() << std::flush;
    }

    /// Clear progress line (call after Run completes)
    void ClearProgress() {
        if (progress_enabled_ && console_.IsTerminal()) {
            std::cout << "\r" << std::string(120, ' ') << "\r" << std::flush;
        }
    }

    // === Profiling ===

    /// Begin timing a component
    void BeginComponentTiming(const std::string &component_name) {
        if (!profiling_enabled_) {
            return;
        }
        current_timed_component_ = component_name;
        component_start_time_ = Clock::now();
    }

    /// End timing for current component
    void EndComponentTiming() {
        if (!profiling_enabled_ || current_timed_component_.empty()) {
            return;
        }

        auto elapsed =
            std::chrono::duration<double, std::micro>(Clock::now() - component_start_time_).count();

        auto &stats = component_stats_[current_timed_component_];
        stats.name = current_timed_component_;
        stats.call_count++;
        stats.total_time_us += elapsed;
        stats.max_time_us = std::max(stats.max_time_us, elapsed);
        stats.avg_time_us = stats.total_time_us / static_cast<double>(stats.call_count);

        current_timed_component_.clear();
    }

    /// Get profiling statistics
    [[nodiscard]] std::vector<ComponentStats> GetProfilingStats() const {
        std::vector<ComponentStats> result;
        result.reserve(component_stats_.size());

        // Calculate total time for percentage
        double total_time = 0.0;
        for (const auto &[name, stats] : component_stats_) {
            total_time += stats.total_time_us;
        }

        for (auto [name, stats] : component_stats_) {
            if (total_time > 0) {
                stats.percent_load = (stats.total_time_us / total_time) * 100.0;
            }
            result.push_back(stats);
        }

        return result;
    }

    // === Low-Level Access ===

    /// Get underlying console
    [[nodiscard]] Console &GetConsole() { return console_; }
    [[nodiscard]] const Console &GetConsole() const { return console_; }

    /// Log raw message
    void Log(LogLevel level, const std::string &message) {
        console_.Log(level, message);
        if (log_file_.is_open() && level >= file_level_) {
            log_file_ << LevelPrefix(level) << " " << StripAnsi(message) << "\n";
            log_file_.flush();
        }
    }

    void LogTimed(LogLevel level, double sim_time, const std::string &message) {
        std::string formatted = FormatTime(sim_time) + " " + message;
        console_.Log(level, formatted);
        if (log_file_.is_open() && level >= file_level_) {
            log_file_ << LevelPrefix(level) << " " << StripAnsi(formatted) << "\n";
            log_file_.flush();
        }
    }

  private:
    Console console_;
    std::ofstream log_file_;
    std::string log_file_name_;
    LogLevel file_level_ = LogLevel::Debug;

    // Simulation metadata
    std::string version_ = icarus::Version();
#ifdef NDEBUG
    std::string build_type_ = "RELEASE";
#else
    std::string build_type_ = "DEBUG";
#endif

    // Timing
    TimePoint startup_time_;
    TimePoint phase_start_time_;
    TimePoint run_start_wall_time_;
    std::string current_phase_ = SimPhase::Init;

    // Progress display
    bool progress_enabled_ = true;
    int progress_width_ = 40;

    // Profiling
    bool profiling_enabled_ = false;
    std::map<std::string, ComponentStats> component_stats_;
    std::string current_timed_component_;
    TimePoint component_start_time_;

    // Helpers
    [[nodiscard]] double GetWallClockSeconds() const {
        return std::chrono::duration<double>(Clock::now() - startup_time_).count();
    }

    /// Format time as fixed-width string [XXX.XXX] - uses scientific for large values
    [[nodiscard]] static std::string FormatTime(double t) {
        std::ostringstream oss;
        if (t >= 1000.0 || t <= -1000.0) {
            oss << "[" << std::scientific << std::setprecision(2) << t << "]";
        } else {
            oss << "[" << std::fixed << std::setprecision(6) << t << "]";
        }
        return oss.str();
    }

    /// Format duration with appropriate unit
    [[nodiscard]] static std::string FormatDuration(double seconds) {
        std::ostringstream oss;
        if (seconds < 0.001) {
            oss << std::fixed << std::setprecision(1) << (seconds * 1e6) << "μs";
        } else if (seconds < 1.0) {
            oss << std::fixed << std::setprecision(2) << (seconds * 1000) << "ms";
        } else {
            oss << std::fixed << std::setprecision(2) << seconds << "s";
        }
        return oss.str();
    }

    void WriteToFile(const std::string &message) {
        if (log_file_.is_open()) {
            log_file_ << StripAnsi(message) << "\n";
            log_file_.flush();
        }
    }

    [[nodiscard]] static std::string LevelPrefix(LogLevel level) {
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

    [[nodiscard]] static std::string StripAnsi(const std::string &s) {
        std::string result;
        result.reserve(s.size());
        bool in_escape = false;
        for (char c : s) {
            if (c == '\033') {
                in_escape = true;
            } else if (in_escape && c == 'm') {
                in_escape = false;
            } else if (!in_escape) {
                result += c;
            }
        }
        return result;
    }
};

} // namespace icarus
