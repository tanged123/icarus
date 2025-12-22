# Error Handling & Logging

**Related:** [04_lifecycle.md](04_lifecycle.md) | [15_services.md](15_services.md) | [22_testing.md](22_testing.md)

---

## 1. Severity Levels

| Level | Use Case | Behavior |
|-------|----------|----------|
| **ERROR** | Unrecoverable (NaN, solver failure) | Stop simulation, propagate exception |
| **WARNING** | Recoverable issue (constraint violation, clamp) | Log and continue |
| **INFO** | Informational (phase change, event) | Log only |
| **DEBUG** | Development debugging | Log if debug mode enabled |
| **TRACE** | High-frequency (per-step values) | Log if trace mode enabled |

---

## 2. Logging API

```cpp
// Within a component
ICARUS_ERROR("Solver diverged at t={}", t);      // Throws, stops sim
ICARUS_WARN("Clipping thrust to max: {}", max);  // Logs, continues
ICARUS_INFO("Phase transition: {} -> {}", old, new_phase);
ICARUS_DEBUG("State vector: {}", state);

// Global configuration
icarus::Logger::set_level(icarus::LogLevel::INFO);
icarus::Logger::set_output("logs/sim.log");
```

---

## 3. Validation Errors

Signal binding failures, configuration errors, and type mismatches throw during **Provision** or **Stage**—never during **Step**.

---

## 4. Runtime Debug & Trace Mode

Debug and Trace modes can be enabled **without recompilation**—critical for diagnosing issues in production builds.

### 4.1 Runtime Configuration

```cpp
// Enable tracing for specific components/signals at runtime
sim.EnableTrace("Vehicle.Aero.*");           // All Aero signals
sim.EnableTrace("Vehicle.EOM.position");     // Specific signal
sim.EnableTrace("GNC.*", TraceLevel::DEBUG); // Component with level

// Disable tracing
sim.DisableTrace("Vehicle.Aero.*");

// Query current trace status
auto traced_signals = sim.GetTracedSignals();
```

### 4.2 Configuration File

```yaml
services:
  debug:
    enabled: true

    # Enable tracing for signal patterns (glob syntax)
    trace_signals:
      - pattern: "Vehicle.Nav.*"
        level: DEBUG
        rate_hz: 100  # Downsample high-frequency signals
      - pattern: "Vehicle.Aero.alpha"
        level: TRACE
        rate_hz: 1000

    # Enable component-level tracing
    trace_components:
      - name: "Vehicle.GNC"
        level: DEBUG
        log_inputs: true
        log_outputs: true
        log_timing: true  # Measure step duration

    # Breakpoints (pause simulation when condition met)
    breakpoints:
      - condition: "Vehicle.Nav.altitude < 0"
        action: PAUSE
        message: "Vehicle below ground level"
      - condition: "Vehicle.EOM.velocity_norm > 1000"
        action: LOG_WARNING
        message: "Velocity exceeds expected range"

    # Output options
    output:
      console: true
      file: "logs/trace_{timestamp}.log"
      format: JSON  # or TEXT, CSV
```

---

## 5. Implementation

```cpp
template <typename Scalar>
class TracingBackplane : public Backplane<Scalar> {
    std::unordered_set<std::string> traced_signals_;
    std::ofstream trace_file_;

public:
    void EnableTrace(const std::string& pattern) {
        for (const auto& [name, meta] : registry_) {
            if (MatchGlob(name, pattern)) {
                traced_signals_.insert(name);
            }
        }
    }

    // Called after each Step
    void LogTracedSignals(Scalar t) {
        if (traced_signals_.empty()) return;

        json frame;
        frame["t"] = t;
        for (const auto& name : traced_signals_) {
            frame[name] = GetSignalValue(name);
        }
        trace_file_ << frame.dump() << "\n";
    }
};
```

---

## 6. Component Timing Profiler

```cpp
class ComponentProfiler {
    struct Profile {
        std::string name;
        double total_time_us = 0;
        double max_time_us = 0;
        int64_t call_count = 0;
    };

    std::unordered_map<Component*, Profile> profiles_;

public:
    void BeginStep(Component* comp) {
        start_times_[comp] = HighResolutionClock::now();
    }

    void EndStep(Component* comp) {
        auto elapsed = HighResolutionClock::now() - start_times_[comp];
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

        auto& p = profiles_[comp];
        p.total_time_us += us;
        p.max_time_us = std::max(p.max_time_us, static_cast<double>(us));
        p.call_count++;
    }

    void PrintReport() {
        ICARUS_INFO("=== Component Timing Report ===");
        for (const auto& [comp, p] : profiles_) {
            double avg = p.total_time_us / p.call_count;
            ICARUS_INFO("{}: avg={:.2f}us, max={:.2f}us, calls={}",
                p.name, avg, p.max_time_us, p.call_count);
        }
    }
};
```

---

## 7. Interactive Debugging (Future: Daedalus Integration)

```cpp
// For integration with Daedalus visualization tool
class DebugServer {
public:
    void Start(int port = 9999) {
        // ZMQ REP socket for debug commands
        socket_.bind("tcp://*:" + std::to_string(port));
    }

    // Commands from Daedalus:
    // {"cmd": "pause"}
    // {"cmd": "step", "count": 1}
    // {"cmd": "set_breakpoint", "condition": "Nav.alt < 0"}
    // {"cmd": "get_signal", "name": "Nav.position"}
    // {"cmd": "set_signal", "name": "Control.throttle", "value": 0.5}
    // {"cmd": "enable_trace", "pattern": "Aero.*"}
};
```

> [!NOTE]
> **Performance Impact:** Tracing adds overhead only for traced signals. When no signals are traced, cost is a single branch per Step (negligible). Component timing profiler adds ~100ns per component when enabled.
