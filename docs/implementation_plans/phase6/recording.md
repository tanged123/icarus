# Recording Implementation

**Status:** Proposed  
**Phase:** 6.2  
**Related:** [20_recording.md](../../architecture/20_recording.md) | [15_services.md](../../architecture/15_services.md)

---

## Overview

Implement the `.icarec` recording format using HDF5 as the underlying storage. The recorder captures all simulation signals per-frame, with versioned schema for forward/backward compatibility.

### Key Dependencies

This task requires integrating **Vulcan's time infrastructure** which provides:

- `Epoch<Scalar>` class for unified time representation
- Multiple time scales: TAI, UTC, TT, GPS, TDB
- Leap second handling
- Symbolic-compatible time computations

---

## Time System Integration

### Current State

Currently, `Simulator.hpp` only tracks MET (Mission Elapsed Time):

```cpp
class Simulator {
    double time_ = 0.0;  // MET: seconds since simulation start
};
```

### Proposed State

Use Vulcan's `Epoch` as the **single source of truth**. MET is derived, not stored separately:

```cpp
class Simulator {
    vulcan::time::NumericEpoch epoch_;       // Current epoch (THE time)
    vulcan::time::NumericEpoch epoch_start_; // Reference: epoch at t=0
};
```

### Time Access API

```cpp
class Simulator {
public:
    // MET: derived from epoch (backward compatible)
    [[nodiscard]] double Time() const { 
        return epoch_ - epoch_start_;  // Epoch subtraction returns seconds
    }
    
    // Full epoch access
    [[nodiscard]] const vulcan::time::NumericEpoch& Epoch() const { return epoch_; }
    
    // Convenience accessors
    [[nodiscard]] double JD_UTC() const { return epoch_.jd_utc(); }
    [[nodiscard]] double JD_TAI() const { return epoch_.jd_tai(); }
    [[nodiscard]] double JD_TT() const { return epoch_.jd_tt(); }
    [[nodiscard]] double JD_GPS() const { return epoch_.jd_gps(); }
    [[nodiscard]] int GPSWeek() const { return epoch_.gps_week(); }
    [[nodiscard]] double GPSSecondsOfWeek() const { return epoch_.gps_seconds_of_week(); }
    
    // ISO string for logging
    [[nodiscard]] std::string ISO8601() const { return epoch_.to_iso_string(); }
};
```

### Step Integration

Only one thing to update:

```cpp
void Simulator::Step(double dt) {
    // ... existing step logic ...
    
    epoch_ += dt;  // Single time update - MET derived automatically
    
    // ... rest of step ...
}
```

### Time System Configuration

```yaml
time:
  start: 0.0       # MET start (usually 0)
  end: 100.0       # MET end
  dt: 0.01
  
  # Epoch configuration (optional - omit for MET-only mode)
  epoch:
    system: UTC                        # Primary time system
    reference: "2024-12-22T10:30:00Z"  # ISO 8601 epoch start
    # OR
    # system: TAI
    # jd: 2460666.93750
    # OR  
    # system: GPS
    # week: 2323
    # seconds: 158000.0
```

### MET-Only Mode (Backward Compatible)

If no epoch is configured, use a default epoch at J2000.0:

```cpp
void Simulator::Configure(const SimulatorConfig& config) {
    if (config.epoch.reference.empty()) {
        // MET-only mode: use J2000.0 as arbitrary reference
        epoch_start_ = vulcan::time::NumericEpoch();  // Default: J2000.0
    } else {
        // Configured epoch
        epoch_start_ = ParseEpoch(config.epoch);
    }
    epoch_ = epoch_start_;
}
```

This way:

- `Time()` returns MET (seconds since start) - **unchanged API**
- `Epoch()` returns full epoch with all time scales
- Only **one** time variable to track

---

## Recording Design

### File Structure

```
recording.icarec (HDF5 file):
├── /schema                    # Group: metadata
│   ├── version: "1.0.0"      # Schema version string
│   ├── icarus_version        # Icarus version string
│   ├── created_at            # ISO8601 timestamp
│   ├── signals               # Dataset: signal definitions
│   └── checksum              # SHA256 of signal schema
├── /time                      # Group: time metadata (NEW)
│   ├── system: "UTC"         # Primary time system
│   ├── epoch_jd_tai          # Reference epoch as JD TAI
│   ├── epoch_iso             # Reference epoch as ISO8601
│   ├── delta_at              # TAI-UTC offset (leap seconds)
│   └── units: "seconds"      # MET units
├── /data                      # Group: simulation data
│   ├── t                     # Dataset: MET time vector [N]
│   ├── jd_utc                # Dataset: JD UTC vector [N] (optional)
│   ├── Vehicle.Nav.position  # Dataset: [N, 3] position history
│   ├── Vehicle.Aero.alpha    # Dataset: [N] alpha history
│   └── ...                   # All recorded signals
└── /state                     # Group: for warmstart
    ├── indices               # State vector signal mapping
    └── snapshots             # Periodic full-state snapshots
```

### Recorder Configuration

```yaml
services:
  recording:
    enabled: true
    format: ICAREC
    path: "output/sim_{timestamp}.icarec"
    
    # Time recording options
    time:
      include_jd_utc: true      # Record JD UTC in addition to MET
      include_jd_tai: false     # Record JD TAI
      include_gps: false        # Record GPS week/seconds
    
    # Signal selection
    policy: ALL
    include:
      - "*"
    exclude:
      - "Debug.*"
    
    rate_hz: 0  # 0 = every step
    warmstart_compatible: true
```

### Recorder Component

```cpp
template <typename Scalar>
class Recorder {
public:
    struct Config {
        std::string path;
        std::string format;  // "ICAREC", "CSV", "PARQUET"
        
        // Time options
        bool include_jd_utc = true;
        bool include_jd_tai = false;
        bool include_gps = false;
        
        // Signal selection
        std::vector<std::string> include_patterns;
        std::vector<std::string> exclude_patterns;
        
        double rate_hz = 0.0;  // 0 = every step
        bool warmstart_compatible = true;
    };
    
    void Open(const Config& config, const vulcan::time::NumericEpoch& epoch);
    void WriteFrame(double met, const vulcan::time::NumericEpoch& epoch);
    void WriteSnapshot();  // For warmstart
    void Close();
    
private:
    H5::H5File file_;
    std::map<std::string, H5::DataSet> datasets_;
    std::vector<SignalRef> recorded_signals_;
};
```

### Schema Definition

```cpp
struct RecordingSchema {
    std::string version;
    std::string icarus_version;
    std::string created_at;
    
    struct TimeConfig {
        std::string system;          // "UTC", "TAI", "GPS", "MET"
        double epoch_jd_tai;         // Reference epoch in JD TAI
        std::string epoch_iso;       // Reference epoch as ISO8601
        int delta_at;                // Leap seconds at epoch
    };
    TimeConfig time;
    
    struct SignalDef {
        std::string name;
        std::string type;
        std::string units;
        std::string lifecycle;
        size_t offset;
        size_t size;
    };
    std::vector<SignalDef> signals;
    
    std::string ComputeChecksum() const;
    bool IsCompatibleWith(const RecordingSchema& other) const;
};
```

---

## Proposed Changes

### Time Infrastructure

#### [MODIFY] [SimulatorConfig.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/SimulatorConfig.hpp)

Add time epoch configuration:

```diff
 struct SimulatorConfig {
     double t_start = 0.0;
     double t_end = 10.0;
     double dt = 0.01;
-    double reference_epoch_jd = 0.0;  // Optional: absolute time epoch
+    
+    // Epoch configuration
+    struct EpochConfig {
+        std::string system = "MET";      // "MET", "UTC", "TAI", "GPS"
+        std::string reference = "";       // ISO8601 or empty for MET-only
+        double jd = 0.0;                  // JD if system not UTC
+        int gps_week = 0;                 // GPS week if system is GPS
+        double gps_seconds = 0.0;         // GPS seconds if system is GPS
+    };
+    EpochConfig epoch;
 };
```

---

#### [MODIFY] [Simulator.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp)

Add Vulcan Epoch member and accessors:

```diff
 #include <vulcan/time/Epoch.hpp>
 
 class Simulator {
 private:
     double time_ = 0.0;
+    vulcan::time::NumericEpoch epoch_;
+    bool has_epoch_ = false;  // True if epoch configured
 
 public:
     [[nodiscard]] double Time() const { return time_; }
+    [[nodiscard]] bool HasEpoch() const { return has_epoch_; }
+    [[nodiscard]] const vulcan::time::NumericEpoch& Epoch() const;
+    [[nodiscard]] double JD_UTC() const;
+    [[nodiscard]] double JD_TAI() const;
+    [[nodiscard]] std::string ISO8601() const;
 };
```

---

#### [MODIFY] [SimulationLoader.hpp](file:///home/tanged/sources/icarus/include/icarus/io/SimulationLoader.hpp)

Parse epoch configuration from YAML:

```cpp
static void ParseTimeSection(SimulatorConfig &cfg, const vulcan::io::YamlNode &node) {
    cfg.t_start = node.Get<double>("start", cfg.t_start);
    cfg.t_end = node.Get<double>("end", cfg.t_end);
    cfg.dt = node.Get<double>("dt", cfg.dt);
    
    // NEW: Parse epoch
    if (node.Has("epoch")) {
        auto epoch = node["epoch"];
        cfg.epoch.system = epoch.Get<std::string>("system", "MET");
        cfg.epoch.reference = epoch.Get<std::string>("reference", "");
        cfg.epoch.jd = epoch.Get<double>("jd", 0.0);
        cfg.epoch.gps_week = epoch.Get<int>("week", 0);
        cfg.epoch.gps_seconds = epoch.Get<double>("seconds", 0.0);
    }
}
```

---

### Recording Infrastructure

#### [NEW] [Recorder.hpp](file:///home/tanged/sources/icarus/include/icarus/io/Recorder.hpp)

- HDF5 file management
- Frame writing with chunked datasets
- Epoch metadata writing
- Signal pattern matching (include/exclude)

---

#### [NEW] [RecordingSchema.hpp](file:///home/tanged/sources/icarus/include/icarus/io/RecordingSchema.hpp)

- Schema definition struct with time config
- Signal type mapping
- Checksum computation
- Compatibility validation

---

#### [NEW] [RecordingReader.hpp](file:///home/tanged/sources/icarus/include/icarus/io/RecordingReader.hpp)

- Load recordings for playback/analysis
- Schema validation on load
- Signal query interface
- Time interpolation for arbitrary `t`
- Epoch reconstruction from metadata

---

### Dependencies

#### [MODIFY] flake.nix

Add HDF5 C++ library:

```diff
 buildInputs = [
   janus
   vulcan
+  hdf5-cpp
 ];
```

---

## Verification Plan

### Automated Tests

#### New Tests: `tests/sim/test_time_system.cpp`

1. `TimeSystem_METPonly` - Default MET-only behavior
2. `TimeSystem_UTCEpoch` - Configure UTC epoch from ISO8601
3. `TimeSystem_TAIEpoch` - Configure TAI epoch from JD
4. `TimeSystem_GPSEpoch` - Configure GPS epoch from week/seconds
5. `TimeSystem_StepAdvances` - Epoch advances with simulation
6. `TimeSystem_AllScales` - JD_UTC, JD_TAI, JD_TT all consistent

#### New Tests: `tests/io/test_recorder.cpp`

1. `Recorder_CreateFile` - Open creates valid HDF5 file
2. `Recorder_WriteSchema` - Schema with time config written
3. `Recorder_WriteFrames` - Signal data captured per frame
4. `Recorder_TimeMetadata` - Time system recorded correctly
5. `Recorder_PatternFiltering` - Include/exclude patterns work
6. `Recorder_Checksum` - Schema checksum computed

#### New Tests: `tests/io/test_recording_reader.cpp`

1. `RecordingReader_LoadSchema` - Schema loaded and validated
2. `RecordingReader_QuerySignal` - Retrieve signal history
3. `RecordingReader_TimeInterpolation` - Interpolate at arbitrary time
4. `RecordingReader_EpochReconstruction` - Reconstruct epoch from metadata
5. `RecordingReader_CompatibilityCheck` - Detect incompatible schemas

```bash
./scripts/test.sh
ctest --test-dir build -R "TimeSystem|Recorder|Recording" -VV
```

### Integration Verification

1. Run a demo simulation with epoch configured
2. Verify `.icarec` file created with correct time metadata
3. Load recording and verify epoch matches simulation
4. Verify time values at frame N match simulation state at frame N

### Manual Verification

User should:

1. Open recording in HDF5 viewer (HDFView, h5py)
2. Verify `/time` group contains expected epoch metadata
3. Verify time vectors are consistent across scales

---

## Usage Examples

### Simple MET Recording (Backward Compatible)

```yaml
time:
  start: 0.0
  end: 100.0
  dt: 0.01
  # No epoch = MET only

services:
  recording:
    enabled: true
    path: "output/sim.icarec"
```

### UTC Epoch Recording

```yaml
time:
  start: 0.0
  end: 100.0
  dt: 0.01
  epoch:
    system: UTC
    reference: "2024-12-22T10:30:00Z"

services:
  recording:
    enabled: true
    path: "output/sim.icarec"
    time:
      include_jd_utc: true
```

### GPS Time Recording

```yaml
time:
  start: 0.0
  end: 100.0
  dt: 0.01
  epoch:
    system: GPS
    week: 2323
    seconds: 158000.0

services:
  recording:
    enabled: true
    path: "output/sim.icarec"
    time:
      include_gps: true
```

---

## Open Questions

1. **Chunking strategy**: What chunk size optimizes write performance vs. read access?
2. **Compression**: Should we enable HDF5 compression by default?
3. **Symbolic recording**: How to handle recordings from symbolic simulations?
4. **Leap second crossing**: How to handle recordings that cross leap second boundaries?
