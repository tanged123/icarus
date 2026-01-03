# Warmstart Implementation

**Status:** In Progress
**Phase:** 6.3
**Related:** [20_recording.md](../../architecture/20_recording.md) | [10_entity_lifecycle.md](../../architecture/10_entity_lifecycle.md)

---

## Overview

Warmstart enables resuming simulation from a recorded mid-flight state. This is essential for:

- Debugging specific flight phases without re-running from start
- Monte Carlo variations from a common waypoint
- Hardware-in-the-loop restart after anomalies

### Key Insight: Warmstart is Just Another Trim Mode

Both **trim/equilibrium** and **warmstart** are forms of **state initialization** during Stage(). The existing TrimSolver infrastructure already handles this - we just need to add warmstart as another mode.

| Mode | Purpose | Method |
|------|---------|--------|
| `equilibrium` | Find steady-state via optimization | Newton solver (existing) |
| `warmstart` | Restore from recording | Load from HDF5 at specified MET |

---

## Design

### Extended TrimConfig

Add warmstart fields to existing TrimConfig:

```cpp
struct TrimConfig {
    bool enabled = false;

    /// Mode: "equilibrium" (default, existing behavior) or "warmstart"
    std::string mode = "equilibrium";

    // === Equilibrium mode settings (existing) ===
    std::vector<std::string> zero_derivatives;
    std::vector<std::string> control_signals;
    double tolerance = 1e-6;
    int max_iterations = 100;
    std::string method = "newton";
    std::unordered_map<std::string, double> initial_guesses;
    std::unordered_map<std::string, std::pair<double, double>> control_bounds;

    // === Warmstart mode settings (new) ===
    std::string recording_path;       ///< Path to recording file
    double resume_time = 0.0;         ///< MET to resume from
    bool validate_schema = true;      ///< Validate recording compatibility
};
```

### Workflow

```cpp
// Option 1: Normal startup (no trim)
auto sim = Simulator::FromConfig("mission.yaml");
sim->Stage();
sim->Step(dt);

// Option 2: Equilibrium trim (existing behavior)
auto sim = Simulator::FromConfig("mission.yaml");
// config has: staging.trim.enabled=true, mode="equilibrium"
sim->Stage();  // Solves for trim
sim->Step(dt);

// Option 3: Warmstart from recording
auto sim = Simulator::FromConfig("mission.yaml");
// config has: staging.trim.enabled=true, mode="warmstart"
//             staging.trim.recording_path="flight.h5", resume_time=120.0
sim->Stage();  // Restores state from recording
sim->Step(dt); // Continues from t=120s
```

### YAML Configuration

```yaml
staging:
  trim:
    enabled: true
    mode: warmstart  # "equilibrium" or "warmstart"

    # Warmstart settings
    recording_path: "recordings/flight_001.h5"
    resume_time: 120.0
    validate_schema: true

    # Equilibrium settings (ignored in warmstart mode)
    zero_derivatives: []
    control_signals: []
```

### WarmstartSolver

New TrimSolver implementation that loads state from recording:

```cpp
class WarmstartSolver : public TrimSolver {
public:
    TrimResult Solve(Simulator& sim, const TrimConfig& config) override {
        TrimResult result;

        // 1. Open recording
        vulcan::io::HDF5Reader reader(config.recording_path);

        // 2. Validate if requested
        if (config.validate_schema) {
            auto validation = ValidateRecording(sim, reader);
            if (!validation.IsValid()) {
                result.converged = false;
                result.message = "Recording validation failed: " +
                                 validation.GetErrors()[0];
                return result;
            }
        }

        // 3. Find frame index for resume_time
        auto times = reader.times();
        size_t frame_idx = FindFrameIndex(times, config.resume_time);

        // 4. Restore state signals
        RestoreState(sim, reader, frame_idx);

        // 5. Restore time
        sim.SetTime(config.resume_time);

        result.converged = true;
        result.message = "Warmstart from " + config.recording_path +
                        " at t=" + std::to_string(config.resume_time);
        return result;
    }

private:
    ValidationResult ValidateRecording(const Simulator& sim,
                                       const vulcan::io::HDF5Reader& reader);
    size_t FindFrameIndex(const std::vector<double>& times, double target_time);
    void RestoreState(Simulator& sim, vulcan::io::HDF5Reader& reader, size_t frame_idx);
};
```

### Updated Factory

```cpp
std::unique_ptr<TrimSolver> CreateTrimSolver(const TrimConfig& config,
                                              bool symbolic_enabled) {
    // Warmstart mode
    if (config.mode == "warmstart") {
        return std::make_unique<WarmstartSolver>();
    }

    // Equilibrium mode (existing logic)
    if (symbolic_enabled && config.method == "newton") {
        return std::make_unique<SymbolicTrim>();
    }
    FiniteDifferenceTrim::Options opts;
    opts.tolerance = config.tolerance;
    opts.max_iterations = config.max_iterations;
    return std::make_unique<FiniteDifferenceTrim>(opts);
}
```

---

## Proposed Changes

### [MODIFY] SimulatorConfig.hpp

- Add `mode` field to TrimConfig: `"equilibrium"` | `"warmstart"`
- Add warmstart fields: `recording_path`, `resume_time`, `validate_schema`

### [MODIFY] SimulationLoader.hpp

- Parse new TrimConfig fields from YAML

### [MODIFY] TrimSolver.hpp

- Add `WarmstartSolver` class implementing TrimSolver
- Update `CreateTrimSolver()` factory to handle warmstart mode

### [NEW] core/ValidationResult.hpp

- Structured validation result for schema checking

### [MODIFY] Simulator.hpp

- Add `SetTime(double met)` helper for warmstart time restoration
- Existing Stage() flow handles warmstart via TrimSolver - no changes needed

---

## State Restoration Process

When `mode = "warmstart"`, the WarmstartSolver:

1. **Open Recording**
   - Use vulcan::io::HDF5Reader

2. **Validate Recording** (if `validate_schema = true`)
   - Check all state signals exist in recording
   - Check types match
   - Warn on extra/missing non-state signals

3. **Find Frame**
   - Locate frame closest to `resume_time`
   - Optionally interpolate between frames

4. **Restore State Vector**
   - Read state signal values at target frame
   - Apply via `sim.Poke()` or `sim.SetState()`

5. **Restore Time**
   - Set `epoch_ = epoch_start_ + resume_time`

6. **Restore Phase** (if phase signals recorded)
   - Read phase signal values
   - Update PhaseManager state

---

## Verification Plan

### Automated Tests: `tests/staging/test_warmstart.cpp`

1. `Warmstart_BasicRestore` - State restored from recording
2. `Warmstart_TimeRestored` - Time()/Epoch() correct after warmstart
3. `Warmstart_ValidationPass` - Compatible recording passes
4. `Warmstart_ValidationFail` - Missing signals detected
5. `Warmstart_ContinueSimulation` - Simulation continues correctly
6. `Warmstart_Interpolation` - MET between frames works

### Integration Test

```cpp
// Record a trajectory
auto sim1 = Simulator::FromConfig("test.yaml");
sim1->Stage();
for (int i = 0; i < 100; ++i) sim1->Step(dt);
double pos_at_50 = sim1->Peek("position.x");  // saved at frame 50

// Warmstart from frame 50
TrimConfig warmstart_cfg;
warmstart_cfg.enabled = true;
warmstart_cfg.mode = "warmstart";
warmstart_cfg.recording_path = "test.h5";
warmstart_cfg.resume_time = 50 * dt;

auto sim2 = Simulator::FromConfig("test.yaml");
sim2->Stage();  // Uses warmstart_cfg from YAML

EXPECT_NEAR(sim2->Time(), 50 * dt, 1e-10);
EXPECT_NEAR(sim2->Peek("position.x"), pos_at_50, 1e-10);
```

---

## Open Questions

1. **Interpolation**: Linear interpolation sufficient for MET between frames?
2. **Phase restoration**: How to restore PhaseManager state correctly?
3. **Component internal state**: Do components need hooks for non-signal state?
