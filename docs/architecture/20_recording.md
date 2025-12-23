# Recording & Playback

**Related:** [15_services.md](15_services.md) | [03_signal_backplane.md](03_signal_backplane.md) | [04_lifecycle.md](04_lifecycle.md)

---

## 1. Recording Format

The `.icarec` format captures all simulation data:

```
icarec file structure:
├── schema.json          # Signal map, types, units
├── static_signals.bin   # Static signals (one-time)
├── dynamic_signals.bin  # Dynamic signals (per-frame)
└── metadata.json        # Scenario, timestamps, version
```

---

## 2. Versioned Recording Schema

Every recording includes a versioned schema for forward/backward compatibility:

```json
{
  "schema_version": "2.3.1",
  "icarus_version": "1.5.0",
  "created_at": "2024-12-22T10:30:00Z",

  "signals": [
    {
      "name": "Vehicle.Nav.position",
      "type": "Vec3<double>",
      "units": "m",
      "lifecycle": "dynamic",
      "offset": 0,
      "size": 24
    },
    {
      "name": "Vehicle.Aero.alpha",
      "type": "double",
      "units": "rad",
      "lifecycle": "dynamic",
      "offset": 24,
      "size": 8
    }
  ],

  "components": [
    {
      "name": "Vehicle.EOM",
      "type": "RigidBody6DOF",
      "state_offset": 0,
      "state_size": 13
    }
  ],

  "compatibility": {
    "min_icarus_version": "1.0.0",
    "warmstart_compatible": true,
    "breaking_changes": []
  },

  "checksum": "sha256:a1b2c3d4..."
}
```

### 2.1 Time System Specification

For aerospace applications, the simulation time system must be explicit for ephemeris lookups, ground track calculations, and correlation with real-world events:

```json
{
  "time": {
    "epoch": "2024-12-22T00:00:00Z",
    "system": "MET",
    "units": "seconds"
  }
}
```

| Time System | Description | Use Case |
| :--- | :--- | :--- |
| **MET** | Mission Elapsed Time (t=0 at sim start) | Default, simplest |
| **UTC** | Coordinated Universal Time | Correlation with wall-clock |
| **TAI** | International Atomic Time | Ephemeris, precise timing |
| **GPS** | GPS Time (TAI - 19 seconds) | Navigation, GPS correlation |

> [!NOTE]
> Internal simulation always uses `double t` as seconds from epoch. The `time.system` field specifies how to interpret that value when correlating with external data.

---

## 3. Schema Compatibility Rules

| Scenario | Behavior |
| :--- | :--- |
| **Same schema version** | Full compatibility, warmstart works |
| **Newer schema, additive changes** | Compatible—new signals ignored on load |
| **Newer schema, removed signals** | Warning—missing signals set to defaults |
| **Breaking schema change** | Error with clear message |

### Compatibility Validation

```cpp
ValidationResult Simulator::ValidateRecording(const std::string& path) {
    auto recording_schema = Recording::LoadSchema(path);
    auto current_schema = GetCurrentSchema();

    ValidationResult result;

    // Check version compatibility
    if (recording_schema.version < current_schema.min_compatible_version) {
        result.AddError("Recording schema {} is too old (min: {})",
            recording_schema.version, current_schema.min_compatible_version);
    }

    // Check for missing signals required by current components
    for (const auto& input : GetAllRequiredInputs()) {
        if (!recording_schema.HasSignal(input.name)) {
            if (input.has_default) {
                result.AddWarning("Signal '{}' missing, using default: {}",
                    input.name, input.default_value);
            } else {
                result.AddError("Required signal '{}' not in recording", input.name);
            }
        }
    }

    // Check for type mismatches
    for (const auto& sig : recording_schema.signals) {
        if (current_schema.HasSignal(sig.name)) {
            auto current = current_schema.GetSignal(sig.name);
            if (sig.type != current.type) {
                result.AddError("Type mismatch for '{}': recording={}, current={}",
                    sig.name, sig.type, current.type);
            }
        }
    }

    return result;
}
```

---

## 4. Default Recording Policy

> [!TIP]
> **Recommended: Record everything.** Disk is cheap; debugging time is not.

```yaml
services:
  recording:
    enabled: true
    format: ICAREC
    path: "output/run_{timestamp}.icarec"

    # Record ALL signals by default
    policy: ALL

    # Exclude only high-frequency noise (optional)
    exclude:
      - "Debug.*"
      - "*.raw_sensor_*"

    # Ensure warmstart compatibility
    warmstart_compatible: true  # Validates all inputs are recorded
```

---

## 5. Telemetry → .icarec Pipeline

Live telemetry (UDP) and recording share the same data path. Recording is enabled via Services config:

```yaml
services:
  recording:
    enabled: true
    format: ICAREC  # or HDF5, CSV
    path: "output/run_{timestamp}.icarec"
```

---

## 6. Warmstart from Recording

To warmstart from any point in a recording:

```cpp
sim.provision();

// Validate compatibility BEFORE attempting warmstart
auto validation = sim.ValidateRecording("recording.icarec");
if (!validation.IsValid()) {
    for (const auto& error : validation.errors) {
        ICARUS_ERROR("{}", error);
    }
    throw WarmstartError("Recording incompatible with current simulation");
}

// Warmstart with validated recording
sim.warmstart_from("recording.icarec", t_warmstart);
// State vector is restored to t_warmstart
sim.step(dt);  // Continue from that point
```

> [!WARNING]
> **Warmstart requires all input signals to be recorded.** If a component reads an external input that wasn't recorded, warmstart will fail. Use `warmstart_compatible: true` in recording config to catch this at recording time.

---

## 7. Recording Formats

| Format | Pros | Cons | Use Case |
|--------|------|------|----------|
| **ICAREC** | Native, fast, warmstart-compatible | Icarus-specific | Default |
| **HDF5** | Industry standard, Python/MATLAB friendly | Larger files | Analysis |
| **CSV** | Human-readable, universal | Very large, slow | Debugging |
| **Parquet** | Columnar, efficient queries | Complex | Big data analysis |
