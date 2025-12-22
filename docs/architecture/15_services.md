# Services Configuration

**Related:** [13_configuration.md](13_configuration.md) | [20_recording.md](20_recording.md) | [02_component_protocol.md](02_component_protocol.md)

---

Layer F: Miscellaneous simulation infrastructure services.

---

## 1. Services Configuration

```yaml
services:
  # Data Recording
  recording:
    enabled: true
    format: HDF5
    path: "output/x15_run_{timestamp}.h5"
    signals: ["X15.*", "Environment.*"]  # Glob patterns
    rate_hz: 100

  # Live Telemetry (to Hermes/Daedalus)
  telemetry:
    enabled: true
    protocol: UDP
    port: 5000
    rate_hz: 60
    signals: ["X15.EOM.*", "X15.Aero.mach"]

  # Logging
  logging:
    level: INFO
    file: "logs/sim.log"

  # Custom Signals (computed at runtime, not owned by any component)
  custom_signals:
    - name: "Derived.TotalEnergy"
      expression: "0.5 * X15.EOM.mass * X15.EOM.velocity^2 + X15.EOM.mass * 9.81 * X15.EOM.position.z"

  # External Bindings
  bindings:
    python:
      enabled: true
      module: "pybind11"
    matlab:
      enabled: false
    zmq:
      enabled: true
      port: 5555
```

---

## 2. Data Dictionary & Tooling Integration

The **Data Dictionary** (auto-generated at Provision, see [02_component_protocol.md](02_component_protocol.md)) is the single source of truth for all signals. Tooling reads the dictionary—no separate schema file needed.

### Tooling Uses

**1. Config Validation**

```bash
# Validate config against a provisioned simulation's data dictionary
$ icarus provision scenarios/x15_mission.yaml --output-dict data_dictionary.yaml
$ icarus validate scenarios/x15_mission.yaml --dict data_dictionary.yaml

✓ All signal references valid
⚠ Warning: "X15.Aero.alpah" looks like typo for "X15.Aero.alpha"
✗ Error: "X15.Thruster.force" not found in data dictionary
```

**2. IDE Autocomplete**

```yaml
# VS Code extension reads data_dictionary.yaml for autocomplete
components:
  - type: JetEngine
    config:
      input_altitude: "Env|"  # Suggests: Environment.Atmosphere.Altitude
```

**3. Telemetry/Recording Metadata**

```cpp
// Recorder reads data dictionary for units and descriptions
Recording rec("output.icarec");
rec.LoadDataDictionary("output/data_dictionary.yaml");
rec.AddSignal("X15.Nav.position");  // Dictionary provides: units=m, type=Vec3
```

**4. UI Signal Browser (Daedalus)**

Daedalus reads the data dictionary to populate its signal browser with types, units, and descriptions.

---

## 3. Why Auto-Generated?

| Manual Schema | Auto-Generated Dictionary |
| :--- | :--- |
| Can drift from reality | Always matches actual signals |
| Extra maintenance burden | Zero maintenance—just run Provision |
| May have typos | Generated from code—no typos |
| Incomplete coverage | Complete by definition |

> [!TIP]
> **Workflow:** Run `icarus provision` once to generate the dictionary. Use it for all tooling. Regenerate whenever components change.

---

## 4. Custom Signals

Custom signals are computed expressions that don't belong to any component:

```yaml
custom_signals:
  - name: "Derived.TotalEnergy"
    expression: "0.5 * X15.EOM.mass * X15.EOM.velocity^2 + X15.EOM.mass * 9.81 * X15.EOM.position.z"
    units: "J"
    description: "Total mechanical energy"

  - name: "Derived.DynamicPressure"
    expression: "0.5 * Environment.Atmosphere.Density * X15.EOM.velocity^2"
    units: "Pa"
    description: "Dynamic pressure"
```

These are evaluated by a built-in `DerivedSignals` component during PostStep.

---

## 5. Telemetry Configuration

```yaml
telemetry:
  enabled: true
  protocol: UDP          # UDP, TCP, ZMQ
  host: "localhost"
  port: 5000
  rate_hz: 60

  # Signal selection
  signals:
    - "X15.EOM.*"        # All EOM signals
    - "X15.Aero.alpha"   # Specific signal
    - "!X15.Debug.*"     # Exclude debug signals

  # Compression (for high-bandwidth scenarios)
  compression:
    enabled: false
    algorithm: LZ4
```

---

## 6. Logging Configuration

```yaml
logging:
  level: INFO            # TRACE, DEBUG, INFO, WARNING, ERROR

  outputs:
    - type: console
      level: INFO
      format: "[{time}] [{level}] {message}"

    - type: file
      path: "logs/sim_{timestamp}.log"
      level: DEBUG
      rotate: true
      max_size_mb: 100

    - type: structured
      path: "logs/sim_{timestamp}.jsonl"
      level: TRACE
```
