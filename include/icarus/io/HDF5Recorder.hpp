#pragma once

/**
 * @file HDF5Recorder.hpp
 * @brief HDF5 recorder wrapping Vulcan's telemetry system
 *
 * Minimal Icarus-specific adapter that:
 * - Auto-builds TelemetrySchema from SignalRegistry
 * - Captures signal values each frame
 * - Delegates to Vulcan's HDF5Writer for file I/O
 */

#include <memory>
#include <regex>
#include <string>
#include <vector>

#include <icarus/io/data/Recorder.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <vulcan/io/CSVExport.hpp>
#include <vulcan/io/Frame.hpp>
#include <vulcan/io/HDF5Reader.hpp>
#include <vulcan/io/HDF5Writer.hpp>
#include <vulcan/io/TelemetrySchema.hpp>

namespace icarus {

// Forward declaration for backwards compatibility
struct HDF5RecorderConfig;

/**
 * @brief HDF5 recorder for Icarus simulations
 *
 * Wraps Vulcan's telemetry system with automatic schema generation
 * from the Icarus signal registry. Implements the Recorder interface.
 *
 * Supports multiple modes:
 * - "all"     : Record all signals (outputs, inputs, params, config)
 * - "outputs" : Record only output signals (default)
 * - "signals" : Record signals matching include patterns
 *
 * Example:
 * @code
 * HDF5Recorder recorder(registry, config.recording);
 * recorder.Open("");
 *
 * while (t < t_end) {
 *     sim.Step(dt);
 *     recorder.Record(sim.Time());
 * }
 *
 * recorder.Close();
 * @endcode
 */
class HDF5Recorder : public Recorder {
  public:
    /**
     * @brief Construct recorder from RecordingConfig
     * @param registry Reference to signal registry (must outlive Recorder)
     * @param config Recording configuration from SimulatorConfig
     */
    HDF5Recorder(const SignalRegistry<double> &registry, const RecordingConfig &config)
        : registry_(registry), config_(config) {}

    /**
     * @brief Open recording file and initialize writer
     * @param path Output file path (overrides config.path if non-empty)
     */
    void Open(const std::string &path) override {
        if (!path.empty()) {
            config_.path = path;
        }
        BuildSchema();
        writer_ = std::make_unique<vulcan::io::HDF5Writer>(config_.path, schema_);
        frame_ = std::make_unique<vulcan::io::Frame>(schema_);
        frame_counter_ = 0; // Reset for new recording
    }

    /**
     * @brief Record current signal values as a frame
     * @param time Current simulation time (MET)
     *
     * Respects decimation setting: only writes every N frames.
     */
    void Record(double time) override {
        if (!writer_) {
            throw std::runtime_error("Recorder not open");
        }

        // Apply decimation: only record every N frames
        ++frame_counter_;
        if (config_.decimation > 1 && (frame_counter_ % config_.decimation) != 1) {
            return; // Skip this frame
        }

        frame_->set_time(time);
        CaptureSignals();
        writer_->write_frame(*frame_);
    }

    /**
     * @brief Close recording file
     *
     * If export_csv is enabled in config, exports data to CSV after closing HDF5.
     */
    void Close() override {
        if (writer_) {
            writer_->close();
            writer_.reset();

            // Export to CSV if configured
            if (config_.export_csv) {
                ExportCSV();
            }
        }
    }

    /**
     * @brief Get number of frames written
     */
    [[nodiscard]] size_t FrameCount() const { return writer_ ? writer_->frame_count() : 0; }

    /**
     * @brief Flush buffered data to disk
     */
    void Flush() {
        if (writer_) {
            writer_->flush();
        }
    }

    /**
     * @brief Get the generated schema
     */
    [[nodiscard]] const vulcan::io::TelemetrySchema &Schema() const { return schema_; }

    /**
     * @brief Get list of recorded signal names
     */
    [[nodiscard]] const std::vector<std::string> &RecordedSignals() const {
        return recorded_signals_;
    }

  private:
    const SignalRegistry<double> &registry_;
    RecordingConfig config_;
    vulcan::io::TelemetrySchema schema_;
    std::unique_ptr<vulcan::io::HDF5Writer> writer_;
    std::unique_ptr<vulcan::io::Frame> frame_;
    std::vector<std::string> recorded_signals_;
    size_t frame_counter_ = 0; ///< Frame counter for decimation

    /**
     * @brief Build TelemetrySchema from registry signals based on mode
     */
    void BuildSchema() {
        schema_ = vulcan::io::TelemetrySchema();
        recorded_signals_.clear();

        // Compile include/exclude patterns for "signals" mode
        std::vector<std::regex> include_re, exclude_re;
        if (config_.mode == "signals" && !config_.include.empty()) {
            for (const auto &pat : config_.include) {
                include_re.emplace_back(pat);
            }
        }
        for (const auto &pat : config_.exclude) {
            exclude_re.emplace_back(pat);
        }

        // Iterate over all signals and filter based on mode
        for (const auto &desc : registry_.GetDescriptors()) {
            // Skip derivatives unless explicitly included
            if (!config_.include_derivatives && desc.name.find("_dot") != std::string::npos) {
                continue;
            }

            // Mode-based filtering
            bool should_include = false;

            if (config_.mode == "all") {
                // Include everything (outputs, inputs if enabled, params, config)
                if (desc.kind == SignalKind::Input) {
                    should_include = config_.include_inputs;
                } else {
                    should_include = true;
                }
            } else if (config_.mode == "outputs") {
                // Only outputs (default mode)
                should_include =
                    (desc.kind == SignalKind::Output || desc.kind == SignalKind::Parameter);
            } else if (config_.mode == "signals") {
                // Pattern-based selection
                if (include_re.empty()) {
                    should_include = true; // No patterns = include all
                } else {
                    for (const auto &re : include_re) {
                        if (std::regex_search(desc.name, re)) {
                            should_include = true;
                            break;
                        }
                    }
                }
            }

            if (!should_include) {
                continue;
            }

            // Apply exclude patterns
            bool excluded = false;
            for (const auto &re : exclude_re) {
                if (std::regex_search(desc.name, re)) {
                    excluded = true;
                    break;
                }
            }
            if (excluded) {
                continue;
            }

            // Add to schema based on type
            switch (desc.type) {
            case SignalType::Double:
                schema_.add_double(desc.name, desc.lifecycle, desc.unit);
                break;
            case SignalType::Int32:
                schema_.add_int32(desc.name, desc.lifecycle, desc.semantic);
                break;
            case SignalType::Int64:
                schema_.add_int64(desc.name, desc.lifecycle);
                break;
            }

            recorded_signals_.push_back(desc.name);
        }
    }

    /**
     * @brief Capture current signal values into frame
     */
    void CaptureSignals() {
        for (const auto &name : recorded_signals_) {
            const auto *desc = registry_.get_descriptor(name);
            if (!desc || !desc->data_ptr) {
                continue;
            }

            switch (desc->type) {
            case SignalType::Double:
                frame_->set(name, *static_cast<const double *>(desc->data_ptr));
                break;
            case SignalType::Int32:
                frame_->set(name, *static_cast<const int32_t *>(desc->data_ptr));
                break;
            case SignalType::Int64:
                frame_->set(name, *static_cast<const int64_t *>(desc->data_ptr));
                break;
            }
        }
    }

    /**
     * @brief Export HDF5 data to CSV format
     *
     * Uses Vulcan's CSVExport to convert the HDF5 file.
     * CSV path is derived from HDF5 path by replacing extension.
     */
    void ExportCSV() {
        // Derive CSV path from HDF5 path
        std::string csv_path = config_.path;
        auto dot_pos = csv_path.rfind('.');
        if (dot_pos != std::string::npos) {
            csv_path = csv_path.substr(0, dot_pos) + ".csv";
        } else {
            csv_path += ".csv";
        }

        // Use Vulcan's CSV export
        vulcan::io::CSVExportOptions options;
        options.signals = recorded_signals_;
        vulcan::io::export_to_csv(config_.path, csv_path, options);
    }
};

} // namespace icarus
