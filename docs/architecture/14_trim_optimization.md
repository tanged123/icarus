# Trim / Optimization Problem Definition

**Related:** [13_configuration.md](13_configuration.md) | [04_lifecycle.md](04_lifecycle.md) | [07_janus_integration.md](07_janus_integration.md)

---

Defines the **Stage** behavior for equilibrium solving.

---

## 1. Basic Configuration

```yaml
# trim/steady_cruise.yaml
trim:
  mode: EQUILIBRIUM
  solver: IPOPT          # or NEWTON_RAPHSON

  # What to hold fixed
  constraints:
    X15.EOM.position.z: 15000.0   # Hold altitude
    X15.EOM.velocity.x: 200.0     # Hold airspeed

  # What to solve for
  free_variables:
    - X15.Aero.alpha           # Angle of attack
    - X15.MainEngine.throttle  # Throttle setting
    - X15.Aero.elevator        # Trim elevator

  # What must be zero at equilibrium
  targets:
    X15.EOM.accel.x: 0.0
    X15.EOM.accel.z: 0.0
    X15.EOM.angular_accel.y: 0.0  # Pitch moment = 0
```

---

## 2. Trim Problem Formulation

The trim problem is a **constrained optimization** or **root-finding** problem:

```
Given:    Fixed signals (constraints)
Find:     Free variables
Such that: Target signals = target values (typically 0)
```

Mathematically:
```
minimize  ||f(x) - targets||²
   x
subject to:
   x_lower ≤ x ≤ x_upper   (bounds)
   g(x) = constraints       (equality)
```

---

## 3. Extended Trim Configuration

```yaml
trim:
  mode: EQUILIBRIUM  # or INITIAL_GUESS, OPTIMIZATION

  solver:
    type: IPOPT           # NEWTON_RAPHSON, KINSOL, SNOPT
    max_iterations: 100
    tolerance: 1e-8
    verbose: false

  # Fixed values (equality constraints)
  constraints:
    X15.EOM.position.z: 15000.0
    X15.EOM.velocity.x: 200.0
    X15.EOM.attitude.pitch: 0.0  # Level flight

  # Variables to solve for
  free_variables:
    - name: X15.Aero.alpha
      initial_guess: 0.05       # radians
      bounds: [-0.3, 0.3]       # Optional bounds
    - name: X15.MainEngine.throttle
      initial_guess: 0.5
      bounds: [0.0, 1.0]
    - name: X15.Aero.elevator
      initial_guess: 0.0
      bounds: [-0.5, 0.5]

  # What must equal target (typically zero for equilibrium)
  targets:
    X15.EOM.accel.x: 0.0        # No acceleration
    X15.EOM.accel.z: 0.0
    X15.EOM.angular_accel.y: 0.0

  # Advanced: Weighted residuals (for over-determined systems)
  weights:
    X15.EOM.accel.x: 1.0
    X15.EOM.accel.z: 1.0
    X15.EOM.angular_accel.y: 10.0  # Prioritize pitch trim

  # Fallback behavior
  on_failure:
    action: USE_INITIAL_GUESS  # or ERROR, WARN_AND_CONTINUE
    message: "Trim failed, using initial guess"
```

---

## 4. Runtime Mechanics

The `Stage()` function orchestrates trim solving:

```cpp
template <typename Scalar>
void Simulator<Scalar>::Stage(const RunConfig& rc) {
    // 1. Apply fixed constraints to Backplane
    for (const auto& [signal, value] : rc.trim.constraints) {
        backplane_.set(signal, value);
    }

    // 2. Set initial guesses for free variables
    std::vector<double> x0;
    for (const auto& var : rc.trim.free_variables) {
        x0.push_back(var.initial_guess);
    }

    // 3. Choose solver based on mode
    if (rc.trim.mode == TrimMode::EQUILIBRIUM) {
        if constexpr (std::is_same_v<Scalar, double>) {
            // Numeric trim: Newton-Raphson or KINSOL
            SolveNumericTrim(rc, x0);
        } else {
            // Symbolic trim: Build CasADi problem and solve
            SolveSymbolicTrim(rc, x0);
        }
    }

    // 4. Bind signal pointers for Step phase
    BindAllSignals();
}
```

---

## 5. Symbolic Trim Implementation

```cpp
void SolveSymbolicTrim(const RunConfig& rc, std::vector<double>& x0) {
    // Create symbolic variables for free variables
    casadi::MX x = casadi::MX::sym("x", rc.trim.free_variables.size());

    // Map symbolic variables to Backplane signals
    int idx = 0;
    for (const auto& var : rc.trim.free_variables) {
        backplane_.set_symbolic(var.name, x(idx++));
    }

    // Trace one Step to build the dynamics graph
    Step(MX::sym("t"), MX::sym("dt"));

    // Extract target signals as symbolic expressions
    std::vector<casadi::MX> residuals;
    for (const auto& [signal, target] : rc.trim.targets) {
        residuals.push_back(backplane_.get_symbolic(signal) - target);
    }

    // Build NLP
    casadi::MXDict nlp = {
        {"x", x},
        {"f", casadi::MX::sumsqr(casadi::MX::vertcat(residuals))}
    };

    // Add bounds
    casadi::DMDict bounds;
    std::vector<double> lbx, ubx;
    for (const auto& var : rc.trim.free_variables) {
        lbx.push_back(var.bounds.first);
        ubx.push_back(var.bounds.second);
    }
    bounds["lbx"] = lbx;
    bounds["ubx"] = ubx;
    bounds["x0"] = x0;

    // Solve
    auto solver = casadi::nlpsol("trim", "ipopt", nlp, rc.trim.solver_options);
    auto result = solver(bounds);

    // Apply solution to Backplane
    auto x_opt = static_cast<std::vector<double>>(result["x"]);
    idx = 0;
    for (const auto& var : rc.trim.free_variables) {
        backplane_.set(var.name, x_opt[idx++]);
    }
}
```

---

## 6. Trim Validation

After solving, validate the trim solution:

```cpp
TrimResult Simulator::ValidateTrim(const RunConfig& rc) {
    TrimResult result;

    // Check residuals
    for (const auto& [signal, target] : rc.trim.targets) {
        double actual = backplane_.get(signal);
        double error = std::abs(actual - target);
        result.residuals[signal] = error;

        if (error > rc.trim.solver.tolerance * 10) {
            result.warnings.push_back(
                fmt::format("Trim residual for '{}' is {:.2e} (target: {})",
                    signal, error, target));
        }
    }

    // Check bounds
    for (const auto& var : rc.trim.free_variables) {
        double value = backplane_.get(var.name);
        if (value <= var.bounds.first + 1e-6 ||
            value >= var.bounds.second - 1e-6) {
            result.warnings.push_back(
                fmt::format("'{}' at bound: {:.4f} (bounds: [{}, {}])",
                    var.name, value, var.bounds.first, var.bounds.second));
        }
    }

    result.success = result.warnings.empty();
    return result;
}
```

---

## 7. Trim Modes Summary

| Mode | Description | Use Case |
| :--- | :--- | :--- |
| `EQUILIBRIUM` | Solve for zero derivatives | Steady-state flight conditions |
| `INITIAL_GUESS` | Apply initial guesses without solving | Quick setup, debugging |
| `OPTIMIZATION` | Minimize a cost function | Optimal trim (min fuel, max range) |

> [!NOTE]
> **Symbolic Trim Advantage:** Using `Simulator<casadi::MX>` for trim provides exact gradients to the solver, enabling faster convergence for complex, highly nonlinear systems. For simple cases, numeric Newton-Raphson is faster.
