# Testing & Debugging

**Related:** [21_symbolic_constraints.md](21_symbolic_constraints.md) | [18_error_handling.md](18_error_handling.md) | [03_signal_backplane.md](03_signal_backplane.md)

---

## 1. Component Unit Testing

Test components in isolation using a **MockBackplane**:

```cpp
TEST(AeroComponent, ComputesLiftCorrectly) {
    MockBackplane bp;
    bp.set("Env.Atm.Density", 1.225);
    bp.set("Nav.Velocity", 100.0);
    bp.set("Nav.Alpha", 0.1);

    AeroComponent<double> aero;
    aero.provision(config);
    aero.stage(bp);
    aero.step(0.0, 0.001);

    EXPECT_NEAR(bp.get("Aero.Lift"), expected_lift, 1e-6);
}
```

---

## 2. Symbolic Mode Testing

Every component should be tested in BOTH modes:

```cpp
TEST(AeroComponent, SymbolicModeTraces) {
    MockBackplane<casadi::MX> bp;
    // ... setup symbolic signals ...

    AeroComponent<casadi::MX> aero;
    aero.provision(config);
    aero.stage(bp);
    aero.step(janus::sym("t"), janus::sym("dt"));

    // Verify output is a valid MX expression
    auto lift = bp.get("Aero.Lift");
    EXPECT_TRUE(lift.is_valid_input());
}
```

---

## 3. Mini-Sim Pattern (Python/PyTest)

For integration testing:

```python
import icarus
import pytest

def test_x15_reaches_altitude():
    sim = icarus.Simulator("scenarios/x15_test.yaml")
    sim.provision()
    sim.stage("trim/ground_start.yaml")

    while sim.time < 60.0:
        sim.step(0.001)

    assert sim.get_signal("Nav.Altitude") > 10000.0
```

---

## 4. Test Categories

| Category | Scope | Tools | Frequency |
|----------|-------|-------|-----------|
| **Unit** | Single component | GoogleTest, MockBackplane | Every commit |
| **Integration** | Multiple components | PyTest, mini-sim | Every PR |
| **Regression** | Full scenario | Recorded baselines | Nightly |
| **Symbolic** | Graph correctness | CasADi validation | Every commit |
| **Performance** | Timing benchmarks | Google Benchmark | Weekly |

---

## 5. MockBackplane Implementation

```cpp
template <typename Scalar>
class MockBackplane : public Backplane<Scalar> {
    std::unordered_map<std::string, Scalar> values_;

public:
    void set(const std::string& name, Scalar value) {
        values_[name] = value;
    }

    Scalar get(const std::string& name) const {
        return values_.at(name);
    }

    template <typename T>
    SignalHandle<T> resolve(const std::string& name) override {
        // Return handle pointing to internal storage
        return SignalHandle<T>(&values_[name]);
    }

    void register_output(const std::string& name, Scalar* ptr, ...) override {
        // Track registration for verification
        registered_outputs_.insert(name);
    }
};
```

---

## 6. Regression Testing with Baselines

```yaml
# test/regression/x15_baseline.yaml
scenario: scenarios/x15_mission.yaml
trim: trim/steady_cruise.yaml
duration: 60.0
dt: 0.001

# Expected values at t=60s
assertions:
  - signal: Nav.Altitude
    expected: 15234.5
    tolerance: 1.0

  - signal: Nav.Velocity
    expected: 201.3
    tolerance: 0.1

  - signal: Propulsion.FuelMass
    expected: 1523.2
    tolerance: 0.5
```

```python
def test_x15_regression():
    baseline = load_baseline("test/regression/x15_baseline.yaml")
    sim = run_simulation(baseline.scenario, baseline.trim, baseline.duration)

    for assertion in baseline.assertions:
        actual = sim.get_signal(assertion.signal)
        assert abs(actual - assertion.expected) < assertion.tolerance
```

---

## 7. Debugging / Introspection

> [!NOTE]
> **Future Work:** Full debugging UI is planned for Daedalus. Current introspection is via:
> - `icarus_get_schema_json()` for signal listing
> - Recording + playback for post-mortem analysis
> - Component-level logging at DEBUG level

### Current Debugging Workflow

1. **Enable DEBUG logging:**
   ```yaml
   services:
     logging:
       level: DEBUG
   ```

2. **Enable signal tracing:**
   ```yaml
   services:
     debug:
       trace_signals:
         - pattern: "Vehicle.*"
   ```

3. **Record full simulation:**
   ```yaml
   services:
     recording:
       policy: ALL
   ```

4. **Analyze recording post-mortem:**
   ```python
   import icarus.analysis as ia

   rec = ia.Recording("output/failed_run.icarec")
   rec.plot(["Nav.Altitude", "Nav.Velocity"])
   rec.find_anomalies()  # Detect NaN, discontinuities, etc.
   ```

---

## 8. Common Test Utilities

```cpp
namespace icarus::test {

// Create a minimal simulator for testing
template <typename Scalar>
Simulator<Scalar> MiniSim(const std::vector<Component<Scalar>*>& components) {
    Simulator<Scalar> sim;
    for (auto* c : components) {
        sim.AddComponent(c);
    }
    sim.Provision();
    return sim;
}

// Verify signal exists and has expected type
void AssertSignalExists(Backplane& bp, const std::string& name, DataType type);

// Verify no NaN in any signal
void AssertNoNaN(Backplane& bp);

// Compare two state vectors within tolerance
void AssertStateNear(const StateVector& a, const StateVector& b, double tol);

}  // namespace icarus::test
```

---

## 9. Continuous Integration Workflow

```yaml
# .github/workflows/test.yaml
jobs:
  test:
    steps:
      - name: Unit Tests
        run: ctest --test-dir build -L unit

      - name: Symbolic Tests
        run: ctest --test-dir build -L symbolic

      - name: Integration Tests
        run: pytest tests/integration/

      - name: Regression Tests
        run: pytest tests/regression/ --baseline-dir=baselines/
```
