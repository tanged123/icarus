# Phase 4.0.7: Staging Configuration

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Overview

"Staging" prepares the vehicle for launch - like staging a rocket on the launch pad. This phase handles:

1. **Trim optimization** - Finding equilibrium/trim conditions
2. **Linearization** - Computing state-space (A, B, C, D) matrices
3. **Symbolic generation** - Creating symbolic dynamics graphs for analysis

Staging occurs after `Provision()` (wiring complete) and before `Step()` (simulation running).

---

## YAML Schema for Staging

### Per-Entity Staging (in entity template)

```yaml
# entities/rocket.yaml

entity:
  name: Rocket

  staging:
    # =========================================================================
    # Trim Configuration
    # =========================================================================
    trim:
      enabled: true
      method: newton           # newton, gradient, simplex

      # Which derivatives should be zero at trim?
      zero_derivatives:
        - EOM.velocity_dot      # Steady velocity
        - EOM.angular_rate_dot  # No angular acceleration

      # Which signals can be adjusted to achieve trim?
      control_signals:
        - GNC.throttle_cmd      # Adjust thrust
        - GNC.pitch_cmd         # Adjust pitch

      # Control bounds
      control_bounds:
        GNC.throttle_cmd: [0.0, 1.0]
        GNC.pitch_cmd: [-0.5, 0.5]

      # Initial guesses for controls
      initial_guesses:
        GNC.throttle_cmd: 0.7
        GNC.pitch_cmd: 0.1

      # Solver settings
      tolerance: 1.0e-6
      max_iterations: 100

    # =========================================================================
    # Linearization Configuration
    # =========================================================================
    linearization:
      enabled: true

      # State variables (for A matrix rows/cols)
      states:
        - EOM.position
        - EOM.velocity
        - EOM.attitude
        - EOM.angular_rate

      # Input variables (for B matrix cols)
      inputs:
        - GNC.throttle_cmd
        - GNC.pitch_cmd

      # Output variables (for C matrix rows)
      outputs:
        - EOM.position
        - EOM.velocity

      # Export options
      export:
        matlab: true
        numpy: true
        json: true
        output_dir: "./linear_models"

    # =========================================================================
    # Symbolic Generation Configuration
    # =========================================================================
    symbolics:
      enabled: false
      generate_dynamics: true    # Generate f(x, u) symbolic graph
      generate_jacobian: true    # Generate df/dx, df/du
      output_dir: "./symbolic_output"
```

---

## Complete Simulation-Level Staging Example

```yaml
# rocket_sim.yaml - Complete simulation configuration

simulation:
  name: "Rocket 6DOF Test"
  dt: 0.01
  t_start: 0.0
  t_end: 100.0

# Components and their configs
components:
  - type: PointMassGravity
    name: Gravity
    entity: Rocket
    scalars:
      mu: 3.986004418e14

  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    vectors:
      initial_position: [0, 0, 6.471e6]
      initial_velocity: [7500, 0, 0]
      initial_attitude: [1, 0, 0, 0]

# Signal routing (can also be in separate file)
routes:
  - input: Rocket.EOM.gravity_accel
    output: Rocket.Gravity.accel
    gain: 1.0

# Initial conditions (applied during Provision, before trim)
initial_conditions:
  Rocket.GNC.throttle: 0.8
  Rocket.GNC.pitch_cmd: 0.0

# Staging configuration
staging:
  # Trim optimization settings
  trim:
    enabled: true
    method: newton           # newton, gradient, simplex
    tolerance: 1.0e-6
    max_iterations: 100

    # Which derivatives should be zero at trim?
    zero_derivatives:
      - Rocket.EOM.velocity_dot      # Steady velocity
      - Rocket.EOM.angular_rate_dot  # No angular acceleration

    # Which signals can be adjusted to achieve trim?
    control_signals:
      - Rocket.GNC.throttle          # Adjust thrust
      - Rocket.GNC.pitch_cmd         # Adjust pitch

    # Initial guesses for controls
    initial_guesses:
      Rocket.GNC.throttle: 0.7
      Rocket.GNC.pitch_cmd: 0.1

  # Symbolic generation (optional)
  generate_symbolics: false
  generate_jacobian: false
  symbolic_output_dir: "./symbolic_output"

  # Validation
  validate_wiring: true
  warn_on_unwired: true

# Integrator settings
integrator:
  type: RK4                  # Euler, RK4, RK45
  # For adaptive integrators:
  # abs_tol: 1.0e-6
  # rel_tol: 1.0e-3

# Logging settings
logging:
  console_level: Info
  file_enabled: true
  file_path: "sim.log"
  file_level: Debug
  progress_enabled: true
  profiling_enabled: false
```

---

## Trim Optimization

### Purpose

Find control inputs that drive specified derivatives to zero, achieving equilibrium/trim.

### Algorithm

1. Initial state set from ICs
2. Evaluate derivatives with current controls
3. Compute residual (deviation from zero)
4. Adjust controls using optimizer (Newton, gradient, etc.)
5. Repeat until residual < tolerance

### C++ Implementation

```cpp
void Simulator<Scalar>::RunTrimOptimization() {
    const auto& trim_cfg = config_.staging.trim;
    if (!trim_cfg.enabled) return;

    // Build objective function: minimize norm of zero_derivatives
    auto objective = [&](const std::vector<Scalar>& controls) -> Scalar {
        // Apply controls
        for (size_t i = 0; i < controls.size(); ++i) {
            Poke(trim_cfg.control_signals[i], controls[i]);
        }

        // Compute derivatives
        ComputeDerivatives(time_);

        // Compute residual
        Scalar residual = Scalar{0};
        for (const auto& deriv_name : trim_cfg.zero_derivatives) {
            auto deriv = Peek<Scalar>(deriv_name);
            residual = residual + deriv * deriv;
        }
        return janus::sqrt(residual);
    };

    // Build bounds
    std::vector<std::pair<Scalar, Scalar>> bounds;
    for (const auto& sig : trim_cfg.control_signals) {
        if (auto it = trim_cfg.control_bounds.find(sig); it != trim_cfg.control_bounds.end()) {
            bounds.push_back(it->second);
        } else {
            bounds.push_back({-1e10, 1e10});  // No bounds
        }
    }

    // Initial guess
    std::vector<Scalar> x0;
    for (const auto& sig : trim_cfg.control_signals) {
        if (auto it = trim_cfg.initial_guesses.find(sig); it != trim_cfg.initial_guesses.end()) {
            x0.push_back(it->second);
        } else {
            x0.push_back(Peek<Scalar>(sig));  // Use current value
        }
    }

    // Run optimizer (using Janus)
    auto result = janus::optimize::newton(objective, x0, bounds,
                                          trim_cfg.tolerance,
                                          trim_cfg.max_iterations);

    // Apply final controls
    for (size_t i = 0; i < result.x.size(); ++i) {
        Poke(trim_cfg.control_signals[i], result.x[i]);
    }

    logger_.Info("Trim converged in {} iterations, residual = {}",
                 result.iterations, result.residual);
}
```

---

## Linearization

### Purpose

Compute linear state-space model at the current operating point:
- **A** = df/dx (state Jacobian)
- **B** = df/du (input Jacobian)
- **C** = dg/dx (output Jacobian)
- **D** = dg/du (feedthrough)

### C++ Implementation

```cpp
struct LinearModel {
    janus::Matrix<Scalar> A;  // n_states x n_states
    janus::Matrix<Scalar> B;  // n_states x n_inputs
    janus::Matrix<Scalar> C;  // n_outputs x n_states
    janus::Matrix<Scalar> D;  // n_outputs x n_inputs

    std::vector<std::string> state_names;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;

    void ExportMatlab(const std::string& path) const;
    void ExportNumPy(const std::string& path) const;
    void ExportJSON(const std::string& path) const;
};

LinearModel Simulator<Scalar>::ComputeLinearModel() {
    const auto& lin_cfg = config_.staging.linearization;

    LinearModel model;
    model.state_names = lin_cfg.states;
    model.input_names = lin_cfg.inputs;
    model.output_names = lin_cfg.outputs;

    const size_t nx = lin_cfg.states.size();
    const size_t nu = lin_cfg.inputs.size();
    const size_t ny = lin_cfg.outputs.size();

    // Use automatic differentiation to compute Jacobians
    // This leverages Janus symbolic capabilities

    // Get current state/input values
    auto x0 = GetStateSubset(lin_cfg.states);
    auto u0 = GetInputSubset(lin_cfg.inputs);

    // Compute A = df/dx using AD
    model.A = janus::jacobian([&](const auto& x) {
        SetStateSubset(lin_cfg.states, x);
        ComputeDerivatives(time_);
        return GetDerivativeSubset(lin_cfg.states);
    }, x0);

    // Compute B = df/du using AD
    model.B = janus::jacobian([&](const auto& u) {
        SetInputSubset(lin_cfg.inputs, u);
        ComputeDerivatives(time_);
        return GetDerivativeSubset(lin_cfg.states);
    }, u0);

    // Compute C = dg/dx
    model.C = janus::jacobian([&](const auto& x) {
        SetStateSubset(lin_cfg.states, x);
        return GetOutputSubset(lin_cfg.outputs);
    }, x0);

    // Compute D = dg/du
    model.D = janus::jacobian([&](const auto& u) {
        SetInputSubset(lin_cfg.inputs, u);
        return GetOutputSubset(lin_cfg.outputs);
    }, u0);

    return model;
}
```

---

## Symbolic Generation

### Purpose

Generate symbolic computation graphs for:
- Dynamics function f(x, u)
- Jacobians df/dx, df/du
- Export to CasADi or other symbolic frameworks

### C++ Implementation

```cpp
template <typename S = Scalar,
          typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
janus::Function Simulator<Scalar>::GenerateGraph() {
    const auto& sym_cfg = config_.staging.symbolics;

    // Create symbolic inputs
    auto x_sym = janus::symbolic::Variable("x", GetState().size());
    auto u_sym = janus::symbolic::Variable("u", GetInputSize());

    // Set symbolic state
    SetState(x_sym);

    // Set symbolic inputs
    for (const auto& [name, idx] : input_indices_) {
        Poke(name, u_sym[idx]);
    }

    // Compute derivatives symbolically
    auto x_dot = ComputeDerivatives(janus::symbolic::Variable("t"));

    // Build function
    janus::Function f("dynamics", {x_sym, u_sym}, {x_dot});

    // Export if configured
    if (!sym_cfg.output_dir.empty()) {
        f.Export(sym_cfg.output_dir + "/dynamics.casadi");
    }

    return f;
}

template <typename S = Scalar,
          typename = std::enable_if_t<std::is_same_v<S, SymbolicScalar>>>
janus::Function Simulator<Scalar>::GenerateJacobian() {
    auto f = GenerateGraph();

    // Compute Jacobians
    auto df_dx = f.Jacobian(0);  // w.r.t. x
    auto df_du = f.Jacobian(1);  // w.r.t. u

    // Export
    const auto& sym_cfg = config_.staging.symbolics;
    if (!sym_cfg.output_dir.empty()) {
        df_dx.Export(sym_cfg.output_dir + "/jacobian_x.casadi");
        df_du.Export(sym_cfg.output_dir + "/jacobian_u.casadi");
    }

    return df_dx;  // Return state Jacobian
}
```

---

## Usage Examples

### With Trim and Symbolic Mode

```cpp
// Load config with trim enabled
auto sim = Simulator<double>::FromConfig("rocket_with_trim.yaml");

// Stage with custom override
StageConfig<double> stage_cfg;
stage_cfg.trim.enabled = true;
stage_cfg.trim.tolerance = 1e-8;  // Tighter tolerance
stage_cfg.generate_symbolics = true;
stage_cfg.symbolic_output_dir = "./casadi_functions";

sim.Stage(stage_cfg);  // Runs trim, generates CasADi functions

// Now in equilibrium state, ready to step
sim.Step(0.01);
```

### Linearization for Control Design

```cpp
auto sim = Simulator<double>::FromConfig("aircraft.yaml");
sim.Stage();  // Runs trim

// Get linear model at trim point
auto model = sim.ComputeLinearModel();

// Export for MATLAB control design
model.ExportMatlab("./control_design/aircraft_ss.mat");

// Check controllability
auto W_c = janus::controllability_gramian(model.A, model.B);
std::cout << "System is controllable: " << janus::rank(W_c) << " == " << model.A.rows() << std::endl;
```

---

## StageConfig Struct

```cpp
namespace icarus {

/**
 * @brief Trim optimization configuration
 */
template <typename Scalar>
struct TrimConfig {
    bool enabled = false;

    // Trim targets: which derivatives should be zero?
    std::vector<std::string> zero_derivatives;

    // Trim controls: which signals can be adjusted?
    std::vector<std::string> control_signals;

    // Optimization settings
    Scalar tolerance = Scalar{1e-6};
    int max_iterations = 100;
    std::string method = "newton";  // "newton", "gradient", "simplex"

    // Initial guesses for controls (optional)
    std::unordered_map<std::string, Scalar> initial_guesses;

    // Control bounds (optional)
    std::unordered_map<std::string, std::pair<Scalar, Scalar>> control_bounds;
};

/**
 * @brief Linearization configuration
 */
struct LinearizationConfig {
    bool enabled = false;

    std::vector<std::string> states;
    std::vector<std::string> inputs;
    std::vector<std::string> outputs;

    // Export options
    bool export_matlab = false;
    bool export_numpy = false;
    bool export_json = false;
    std::string output_dir;
};

/**
 * @brief Symbolic generation configuration
 */
struct SymbolicsConfig {
    bool enabled = false;
    bool generate_dynamics = true;
    bool generate_jacobian = false;
    std::string output_dir;
};

/**
 * @brief Staging configuration (trim + linearization + symbolics)
 */
template <typename Scalar>
struct StageConfig {
    TrimConfig<Scalar> trim;
    LinearizationConfig linearization;
    SymbolicsConfig symbolics;

    // Validation settings
    bool validate_wiring = true;
    bool warn_on_unwired = true;
};

} // namespace icarus
```

---

## Exit Criteria

### Trim Optimization

- [ ] `RunTrimOptimization()` using Janus optimization
- [ ] Newton method for trim solver
- [ ] Control bounds enforcement
- [ ] Configurable zero_derivatives and control_signals

### Linearization

- [ ] `ComputeLinearModel()` at trim point
- [ ] State-space (A, B, C, D) extraction
- [ ] Export to MATLAB, NumPy, JSON formats

### Symbolic Mode

- [ ] `GenerateSymbolicGraphs()` for dynamics
- [ ] Symbolic Jacobian generation
- [ ] CasADi function export
