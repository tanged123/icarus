# Warmstart Implementation

**Status:** Proposed  
**Phase:** 6.3  
**Related:** [20_recording.md](../../architecture/20_recording.md) | [10_entity_lifecycle.md](../../architecture/10_entity_lifecycle.md)

---

## Overview

Warmstart enables resuming simulation from a recorded mid-flight state. This is essential for:

- Debugging specific flight phases without re-running from start
- Monte Carlo variations from a common waypoint
- Hardware-in-the-loop restart after anomalies

### Key Requirements

1. **Load state from recording**: Restore full simulation state at any recorded time
2. **Schema validation**: Verify recording compatibility before attempting load
3. **Continue simulation**: Seamlessly continue stepping from restored state
4. **Phase continuity**: Restore correct phase and component activation states
5. **Epoch restoration**: Reconstruct full time state including absolute epoch

---

## Design

### Warmstart Workflow

```cpp
// 1. Create simulator from config (Provision happens in FromConfig)
auto sim = Simulator::FromConfig("mission.yaml");

// 2. Validate recording compatibility
auto validation = sim->ValidateRecording("flight_001.icarec");
if (!validation.IsValid()) {
    for (const auto& error : validation.GetErrors()) {
        ICARUS_ERROR("{}", error);
    }
    throw WarmstartError("Recording incompatible");
}

// 3. Stage in warmstart mode (skips initial conditions)
StageConfig stage_cfg;
stage_cfg.mode = StageMode::WARMSTART;
sim->Stage(stage_cfg);

// 4. Load state from recording at specific MET
double t_resume = 120.0;  // Resume at MET=120s
sim->WarmstartFrom("flight_001.icarec", t_resume);
// - State vector restored
// - Phase signals restored
// - Epoch restored (epoch_start_ + MET)
// - Component internal state restored

// 5. Continue simulation normally
while (sim->Time() < t_end) {
    sim->Step(dt);
}

// Epoch is correctly positioned:
std::cout << "Resumed at: " << sim->ISO8601() << std::endl;
// Output: "2024-12-22T10:32:00Z" (epoch_start + 120s)
```

### State Restoration

The warmstart process restores:

1. **Epoch**: Reconstruct `epoch_` from recording metadata and MET
2. **State vector**: All integrable signals (position, velocity, attitude, etc.)
3. **Phase signals**: Current entity phases
4. **Component internals**: Auxiliary state (e.g., integrator accumulators, filter states)

```cpp
class Simulator {
public:
    ValidationResult ValidateRecording(const std::string& path);
    
    void WarmstartFrom(const std::string& recording_path, double met);
    
private:
    void RestoreEpoch(const RecordingReader& reader, double met);
    void RestoreStateVector(const RecordingReader& reader, double met);
    void RestorePhaseSignals(const RecordingReader& reader, double met);
    void RestoreComponentState(const RecordingReader& reader, double met);
};
```

### Epoch Restoration

The recording stores epoch metadata (see recording.md), enabling full epoch reconstruction:

```cpp
void Simulator::RestoreEpoch(const RecordingReader& reader, double met) {
    // Get epoch_start from recording metadata
    auto schema = reader.GetSchema();
    double epoch_jd_tai = schema.time.epoch_jd_tai;
    int delta_at = schema.time.delta_at;
    
    // Reconstruct epoch_start_
    epoch_start_ = vulcan::time::NumericEpoch::from_jd_tai(epoch_jd_tai, delta_at);
    
    // Set current epoch to epoch_start + MET
    epoch_ = epoch_start_ + met;
    
    // Now Time() returns met, Epoch() returns full epoch
}
```

### Validation Rules

```cpp
struct ValidationResult {
    bool IsValid() const;
    std::vector<std::string> GetErrors() const;
    std::vector<std::string> GetWarnings() const;
    
    struct Issue {
        enum Severity { ERROR, WARNING, INFO };
        Severity severity;
        std::string message;
    };
    std::vector<Issue> issues;
};

// Validation checks:
// ERROR: Required input signal missing from recording
// ERROR: Type mismatch between recording and current config
// ERROR: Recording schema version incompatible
// ERROR: Time system mismatch (e.g., GPS recording for UTC sim)
// WARNING: Optional signal missing (will use default)
// WARNING: Extra signals in recording (will be ignored)
// WARNING: Leap second boundary crossed since recording
```

### Recording Requirements for Warmstart

For a recording to be warmstart-compatible, it must include:

1. All state signals (identified by `is_integrable` flag)
2. All phase signals
3. Epoch metadata (epoch_jd_tai, delta_at, time system)
4. All component-internal state (if warmstart_compatible: true)

```yaml
services:
  recording:
    warmstart_compatible: true  # Validates completeness at record time
```

---

## Proposed Changes

### Core Warmstart Infrastructure

#### [MODIFY] [Simulator.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp)

- Add `StageMode` enum (NORMAL, WARMSTART)
- Add `Stage(config)` with mode parameter
- Add `ValidateRecording(path)` method
- Add `WarmstartFrom(path, met)` method
- Add epoch restoration logic

---

#### [MODIFY] [StateManager.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/StateManager.hpp)

- Add `RestoreFromRecording(reader, met)` method
- Handle state interpolation if met between recorded frames

---

#### [MODIFY] [RecordingReader.hpp](file:///home/tanged/sources/icarus/include/icarus/io/RecordingReader.hpp)

- Add `GetEpochMetadata()` for epoch reconstruction
- Add `GetStateAt(met)` for interpolated state vector
- Add `GetSignalAt(name, met)` for individual signals
- Add `GetSchemaCompatibility(current_schema)` validation

---

#### [NEW] [ValidationResult.hpp](file:///home/tanged/sources/icarus/include/icarus/core/ValidationResult.hpp)

- Structured validation result with errors/warnings
- Helper methods for error checking

---

### Component Interface

#### [MODIFY] [Component.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Component.hpp)

- Add optional `RestoreState(RecordingReader&, double met)` hook
- Components with internal state implement this for warmstart

```cpp
virtual void RestoreState(const RecordingReader& reader, double met) {
    // Default: no-op (stateless components)
}
```

---

## Verification Plan

### Automated Tests

#### New Tests: `tests/sim/test_warmstart.cpp`

1. `Warmstart_ValidateCompatible` - Compatible recording passes validation
2. `Warmstart_ValidateIncompatible` - Missing signal detected
3. `Warmstart_ValidateTypeMismatch` - Type mismatch detected
4. `Warmstart_RestoreState` - State vector restored correctly
5. `Warmstart_RestorePhase` - Phase signals restored
6. `Warmstart_RestoreEpoch` - Epoch reconstructed from metadata
7. `Warmstart_ContinueSimulation` - Simulation continues normally
8. `Warmstart_Interpolation` - State interpolated at non-frame MET
9. `Warmstart_TimeConsistency` - Time() and Epoch() consistent after warmstart

```bash
./scripts/test.sh
ctest --test-dir build -R "Warmstart" -VV
```

### Integration Verification

1. Run simulation for 100 steps with epoch configured, record
2. Warmstart from step 50
3. Compare remaining trajectory with original run
4. Verify trajectories match within floating-point tolerance
5. Verify `Epoch()` returns correct absolute time after warmstart

### Manual Verification

User should:

1. Record a flight with `warmstart_compatible: true`
2. Warmstart from mid-flight
3. Verify `ISO8601()` output shows correct absolute time
4. Verify visual continuity in Daedalus (future)

---

## Dependencies

- Recording infrastructure (6.2) - Must be implemented first
- Unified signal model (6.0) - State signals registered with backplane
- Time infrastructure (6.2) - Epoch class in Simulator

---

## Open Questions

1. **Interpolation method**: Linear interpolation ok, or need higher-order?
2. **Component state scope**: Which component internals must be recorded for perfect warmstart?
3. **Phase edge cases**: What happens if warmstart MET coincides with phase transition?
4. **Leap second crossing**: What if recording was made before a leap second insertion?
