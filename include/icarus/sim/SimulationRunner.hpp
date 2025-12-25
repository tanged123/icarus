#pragma once

/**
 * @file SimulationRunner.hpp
 * @brief Optional convenience wrapper for "run to completion" scenarios
 *
 * Part of Phase 3: Symbolic Mode.
 * For external bindings (Python, MATLAB, C), use Simulator directly.
 * SimulationRunner is for batch simulations and quick demos.
 */

#include <icarus/sim/SimulationResults.hpp>
#include <icarus/sim/Simulator.hpp>

#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace icarus {

/**
 * @brief Optional convenience wrapper for "run to completion" scenarios
 *
 * NOTE: For external bindings (Python, MATLAB, C), use Simulator directly.
 * SimulationRunner is for cases where you want to:
 * - Run a batch simulation without step-by-step control
 * - Get automatic progress display and timing
 * - Collect results in a structured format
 *
 * Usage:
 *   SimulationRunner runner(sim);
 *   runner.SetProgressCallback([](double t, double t_end) {
 *       std::cout << "Progress: " << (t/t_end*100) << "%\n";
 *   });
 *   runner.Run(dt, t_end);
 *   auto results = runner.GetResults();
 */
template <typename Scalar> class SimulationRunner {
  public:
    using ProgressCallback = std::function<void(Scalar t, Scalar t_end)>;
    using StepCallback = std::function<void(Scalar t)>;
    using SignalRecorder = std::function<void(Scalar t, const Simulator<Scalar> &)>;

    /**
     * @brief Construct runner for a simulator
     * @param sim Reference to simulator (must outlive runner)
     */
    explicit SimulationRunner(Simulator<Scalar> &sim) : sim_(sim) {}

    // =========================================================================
    // Configuration
    // =========================================================================

    /**
     * @brief Set callback for progress updates
     * @param callback Called after each step with (t, t_end)
     */
    SimulationRunner &SetProgressCallback(ProgressCallback callback) {
        progress_callback_ = std::move(callback);
        return *this;
    }

    /**
     * @brief Set callback for custom per-step logic
     * @param callback Called after each step with current time
     */
    SimulationRunner &SetStepCallback(StepCallback callback) {
        step_callback_ = std::move(callback);
        return *this;
    }

    /**
     * @brief Set callback for recording signals
     * @param callback Called after each step to record signal values
     * @param interval Record every N steps (default: 1)
     */
    SimulationRunner &SetRecorder(SignalRecorder callback, std::size_t interval = 1) {
        recorder_ = std::move(callback);
        record_interval_ = interval;
        return *this;
    }

    /**
     * @brief Enable default console progress bar
     */
    SimulationRunner &EnableProgressBar(bool enable = true) {
        show_progress_bar_ = enable;
        return *this;
    }

    // =========================================================================
    // Execution
    // =========================================================================

    /**
     * @brief Initialize simulation (Provision + Stage)
     *
     * Call before Run() if simulator was built with Build() not BuildAndInitialize().
     */
    void Initialize() {
        if (!initialized_) {
            sim_.Provision();
            sim_.Stage();
            initialized_ = true;
        }
    }

    /**
     * @brief Run simulation loop
     * @param dt Time step
     * @param t_end End time
     */
    void Run(Scalar dt, Scalar t_end) {
        // Ensure initialized
        if (sim_.GetPhase() < Phase::Staged) {
            Initialize();
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        std::size_t step_count = 0;
        while (sim_.Time() < t_end) {
            sim_.Step(dt);
            step_count++;

            // Call callbacks
            if (progress_callback_) {
                progress_callback_(sim_.Time(), t_end);
            }
            if (step_callback_) {
                step_callback_(sim_.Time());
            }
            if (recorder_ && (step_count % record_interval_ == 0)) {
                recorder_(sim_.Time(), sim_);
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double wall_time = std::chrono::duration<double>(end_time - start_time).count();

        // Populate results
        results_.sim_time_final = sim_.Time();
        results_.wall_time_seconds = wall_time;
        if (wall_time > 0) {
            // Note: For symbolic mode, this conversion may not make sense
            if constexpr (std::is_same_v<Scalar, double>) {
                results_.realtime_factor = static_cast<double>(sim_.Time()) / wall_time;
            }
        }
        results_.total_steps = step_count;
        results_.completed = true;
        results_.termination_reason = "completed";
    }

    /**
     * @brief Perform shutdown and generate debrief
     */
    void Shutdown() {
        // Components can implement Shutdown() if needed
        // For now, this is a placeholder for debrief generation
    }

    // =========================================================================
    // Results
    // =========================================================================

    /**
     * @brief Get simulation results
     * @return Results structure with final state, timing, etc.
     */
    [[nodiscard]] const SimulationResults<Scalar> &GetResults() const { return results_; }

    /**
     * @brief Get recorded signal history
     * @return Vector of (time, signal_values) pairs
     */
    [[nodiscard]] const std::vector<std::pair<Scalar, std::map<std::string, Scalar>>> &
    GetHistory() const {
        return history_;
    }

  private:
    Simulator<Scalar> &sim_;
    ProgressCallback progress_callback_;
    StepCallback step_callback_;
    SignalRecorder recorder_;
    std::size_t record_interval_ = 1;
    bool show_progress_bar_ = false;
    bool initialized_ = false;

    std::vector<std::pair<Scalar, std::map<std::string, Scalar>>> history_;
    SimulationResults<Scalar> results_;
};

} // namespace icarus
