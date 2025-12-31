# Phase 4.0.7: Staging Configuration

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Overview

"Staging" prepares the vehicle for launch - like staging a rocket on the launch pad. This phase handles:

1. **Trim optimization** - Finding equilibrium/trim conditions
2. **Linearization** - Computing state-space (A, B, C, D) matrices
3. **Symbolic generation** - Creating symbolic dynamics graphs for analysis

Staging occurs after `Provision()` (wiring complete) and before `Step()` (simulation running).

### Implementation Priority

| Feature | Phase | Status |
|:--------|:------|:-------|
| Basic `Stage()` lifecycle | **4.0.7** | Core infrastructure |
| Symbolic graph extraction | **Exists** | See [symbolic_orbital_demo.cpp](../../../examples/symbolic/symbolic_orbital_demo.cpp) |
| Jacobian computation | **Exists** | Via `janus::jacobian()` |
| Trim optimization | Future | Newton/IPOPT integration |
| Linearization export | Future | MATLAB/NumPy/JSON |
| Trajectory optimization | Future | DirectCollocation/MultipleShooting |

> **Phase 4.0.7 Focus:** The current phase establishes the configuration infrastructure and lifecycle (`FromConfig` → `Stage` → `Step`). Advanced staging features (trim, linearization, trajectory optimization) build on this foundation in later phases.

### Janus Integration

Staging leverages Janus's optimization and rootfinding capabilities:

| Capability | Janus API | Documentation |
|:-----------|:----------|:--------------|
| Newton rootfinding | `janus::NewtonSolver` | [RootFinding.hpp](../../../references/janus/include/janus/math/RootFinding.hpp) |
| Constrained optimization | `janus::Opti` (IPOPT) | [optimization.md](../../../references/janus/docs/user_guides/optimization.md) |
| Parametric sweeps | `opti.solve_sweep()` | [Opti.hpp](../../../references/janus/include/janus/optimization/Opti.hpp) |
| Automatic differentiation | `janus::jacobian()` | [AutoDiff.hpp](../../../references/janus/include/janus/math/AutoDiff.hpp) |
| Symbolic functions | `janus::Function` | [Function.hpp](../../../references/janus/include/janus/core/Function.hpp) |

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

      # Solver selection:
      #   "newton" - janus::NewtonSolver (fast, for simple F(x)=0)
      #   "ipopt"  - janus::Opti/IPOPT (flexible, supports inequality constraints)
      method: newton

      # Which derivatives should be zero at trim?
      zero_derivatives:
        - EOM.velocity_dot      # Steady velocity
        - EOM.angular_rate_dot  # No angular acceleration

      # Which signals can be adjusted to achieve trim?
      control_signals:
        - GNC.throttle_cmd      # Adjust thrust
        - GNC.pitch_cmd         # Adjust pitch

      # Control bounds (enforced differently per method)
      #   newton: line search clamping
      #   ipopt:  hard constraints
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

### Two Trim Modes

Janus provides two approaches for trim optimization:

| Mode | Backend | Best For | Features |
|:-----|:--------|:---------|:---------|
| **Numeric** | `janus::NewtonSolver` | Fast runtime trim | Direct Newton solve, simple F(x)=0 |
| **Symbolic** | `janus::Opti` (IPOPT) | Complex constraints | Inequality bounds, warm-starting, parametric sweeps |

### Algorithm

1. Initial state set from ICs
2. Evaluate derivatives with current controls
3. Compute residual (deviation from zero)
4. Adjust controls using solver
5. Repeat until residual < tolerance

---

### Numeric Trim (Newton Rootfinding)

Uses `janus::NewtonSolver` for direct F(controls) = 0 solution. Fast and simple for typical trim problems.

See: [references/janus/examples/interpolation/rootfinding_demo.cpp](../../../references/janus/examples/interpolation/rootfinding_demo.cpp)

```cpp
void Simulator<Scalar>::RunNumericTrim() {
    const auto& trim_cfg = config_.staging.trim;
    if (!trim_cfg.enabled || trim_cfg.method != "newton") return;

    const int n_controls = trim_cfg.control_signals.size();
    const int n_residuals = trim_cfg.zero_derivatives.size();

    // Build symbolic residual function F(controls) -> derivatives
    auto [controls, controls_mx] = janus::sym_vec_pair("controls", n_controls);

    // Apply symbolic controls to simulation
    for (int i = 0; i < n_controls; ++i) {
        Poke(trim_cfg.control_signals[i], controls(i));
    }

    // Compute derivatives symbolically
    ComputeDerivatives(time_);

    // Collect residuals
    janus::SymbolicVector residuals(n_residuals);
    for (int i = 0; i < n_residuals; ++i) {
        residuals(i) = Peek<SymbolicScalar>(trim_cfg.zero_derivatives[i]);
    }

    // Create F(controls) -> residuals function
    janus::Function F("TrimResidual",
                      {janus::SymbolicArg(controls_mx)},
                      {janus::to_mx(residuals)});

    // Configure Newton solver
    janus::RootFinderOptions opts;
    opts.abstol = trim_cfg.tolerance;
    opts.max_iter = trim_cfg.max_iterations;
    opts.line_search = true;
    opts.verbose = false;

    janus::NewtonSolver solver(F, opts);

    // Initial guess from config or current values
    Eigen::VectorXd x0(n_controls);
    for (int i = 0; i < n_controls; ++i) {
        auto it = trim_cfg.initial_guesses.find(trim_cfg.control_signals[i]);
        x0(i) = (it != trim_cfg.initial_guesses.end())
                    ? it->second
                    : Peek<double>(trim_cfg.control_signals[i]);
    }

    // Solve
    auto result = solver.solve(x0);

    if (result.converged) {
        // Apply final controls
        for (int i = 0; i < n_controls; ++i) {
            Poke(trim_cfg.control_signals[i], result.x(i));
        }
        logger_.Info("Numeric trim converged, residual norm = {}",
                     residuals.norm());
    } else {
        throw TrimError("Newton trim failed: " + result.message);
    }
}
```

---

### Symbolic Trim (IPOPT Optimization)

Uses `janus::Opti` with IPOPT for constrained optimization. More flexible for complex trim problems with inequality constraints.

See: [references/janus/docs/user_guides/optimization.md](../../../references/janus/docs/user_guides/optimization.md)

```cpp
void Simulator<Scalar>::RunSymbolicTrim() {
    const auto& trim_cfg = config_.staging.trim;
    if (!trim_cfg.enabled || trim_cfg.method != "ipopt") return;

    janus::Opti opti;

    // Create optimization variables for controls
    std::vector<janus::SymbolicScalar> control_vars;
    for (const auto& sig : trim_cfg.control_signals) {
        // Get initial guess
        double init = Peek<double>(sig);
        if (auto it = trim_cfg.initial_guesses.find(sig);
            it != trim_cfg.initial_guesses.end()) {
            init = it->second;
        }

        // Create variable with optional bounds
        auto var = opti.variable(init);
        control_vars.push_back(var);

        // Apply bounds if specified
        if (auto it = trim_cfg.control_bounds.find(sig);
            it != trim_cfg.control_bounds.end()) {
            opti.subject_to_bounds(var, it->second.first, it->second.second);
        }
    }

    // Apply symbolic controls to simulation
    for (size_t i = 0; i < control_vars.size(); ++i) {
        Poke(trim_cfg.control_signals[i], control_vars[i]);
    }

    // Compute derivatives symbolically
    ComputeDerivatives(janus::SymbolicScalar(time_));

    // Build objective: minimize sum of squared derivatives
    janus::SymbolicScalar objective = 0;
    for (const auto& deriv_name : trim_cfg.zero_derivatives) {
        auto deriv = Peek<janus::SymbolicScalar>(deriv_name);
        objective = objective + deriv * deriv;
    }
    opti.minimize(objective);

    // Solve with IPOPT
    janus::OptiOptions opts;
    opts.solver = janus::Solver::IPOPT;
    opts.max_iter = trim_cfg.max_iterations;
    opts.tol = trim_cfg.tolerance;
    opts.verbose = false;

    auto sol = opti.solve(opts);

    // Apply optimized controls
    for (size_t i = 0; i < control_vars.size(); ++i) {
        Poke(trim_cfg.control_signals[i], sol.value(control_vars[i]));
    }

    logger_.Info("IPOPT trim converged in {} iterations",
                 sol.stats().at("iter_count"));
}
```

---

### Parametric Trim Sweeps

For analyzing trim across flight conditions, use `janus::Opti::solve_sweep()`:

```cpp
void Simulator<Scalar>::RunTrimSweep(const std::string& sweep_param,
                                      const std::vector<double>& values) {
    janus::Opti opti;

    // Create sweep parameter
    auto param = opti.parameter(values[0]);

    // Set up trim problem using param
    Poke(sweep_param, param);

    // ... (create control variables, objective as above) ...

    // Solve across parameter range with automatic warm-starting
    auto result = opti.solve_sweep(param, values);

    // Result contains solution at each parameter value
    for (size_t i = 0; i < result.size(); ++i) {
        logger_.Info("{} = {}: throttle = {}", sweep_param, result.param_values[i],
                     result.solutions[i].value(throttle_var));
    }
}
```

---

## Linearization

### Purpose

Compute linear state-space model at the current operating point:

```
ẋ = Ax + Bu    (state equation)
y = Cx + Du    (output equation)
```

- **A** = State matrix (∂f/∂x) — n_states × n_states
- **B** = Input matrix (∂f/∂u) — n_states × n_inputs
- **C** = Output matrix (∂g/∂x) — n_outputs × n_states
- **D** = Feedthrough matrix (∂g/∂u) — n_outputs × n_inputs

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

    // Solver selection
    // "newton" - janus::NewtonSolver (fast, for simple F(x)=0 problems)
    // "ipopt"  - janus::Opti/IPOPT (flexible, supports inequality constraints)
    std::string method = "newton";

    // Trim targets: which derivatives should be zero?
    std::vector<std::string> zero_derivatives;

    // Trim controls: which signals can be adjusted?
    std::vector<std::string> control_signals;

    // Solver settings
    Scalar tolerance = Scalar{1e-6};
    int max_iterations = 100;

    // Initial guesses for controls (optional)
    std::unordered_map<std::string, Scalar> initial_guesses;

    // Control bounds (optional, used by both methods)
    // Newton: enforced via line search clamping
    // IPOPT: enforced as hard constraints
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

## Future: Trajectory Optimization Integration

> **Note:** This section describes advanced capabilities for a future phase, building on the staging infrastructure.

### Beyond Static Trim: Optimal Trajectories

Static trim finds a single equilibrium point. Trajectory optimization finds **optimal control histories** over time. Janus provides two transcription methods:

| Method | Class | Best For |
|:-------|:------|:---------|
| **Direct Collocation** | `janus::DirectCollocation` | Smooth trajectories, path constraints |
| **Multiple Shooting** | `janus::MultipleShooting` | Stiff systems, high-accuracy integration |

See:
- [collocation.md](../../../references/janus/docs/user_guides/collocation.md)
- [multiple_shooting.md](../../../references/janus/docs/user_guides/multiple_shooting.md)

### Use Cases

| Application | Description |
|:------------|:------------|
| **Ascent optimization** | Minimize fuel or maximize payload to orbit |
| **Descent/landing** | Fuel-optimal powered descent (SpaceX-style) |
| **Rendezvous** | Time/fuel-optimal approach trajectories |
| **Reentry guidance** | Heat-constrained trajectory design |
| **Trim trajectory** | Find control history that maintains trim over a maneuver |

### Architecture Concept

Trajectory optimization would use Icarus components as the dynamics model:

```cpp
// Future API concept
auto sim = Simulator<janus::SymbolicScalar>::FromConfig("vehicle.yaml");
sim.Stage();  // Prepare symbolic simulation

// Extract dynamics function from simulation
auto dynamics = [&](const auto& x, const auto& u, const auto& t) {
    sim.SetState(x);
    sim.Poke("Vehicle.GNC.throttle", u(0));
    sim.Poke("Vehicle.GNC.gimbal", u(1));
    sim.ComputeDerivatives(t);
    return sim.GetStateDerivative();
};

// Set up trajectory optimization
janus::Opti opti;
janus::DirectCollocation dc(opti);

auto T = opti.variable(300.0);  // Final time (decision variable)
auto [x, u, tau] = dc.setup(
    sim.StateSize(),     // n_states from simulation
    2,                   // n_controls (throttle, gimbal)
    0.0, T,
    {.scheme = janus::CollocationScheme::HermiteSimpson, .n_nodes = 51}
);

dc.set_dynamics(dynamics);
dc.add_defect_constraints();

// Boundary conditions
dc.set_initial_state(sim.GetState());        // Current state
dc.set_final_state(0, target_altitude);      // Target altitude
dc.set_final_state(1, target_velocity);      // Target velocity

// Path constraints
for (int i = 0; i < 51; ++i) {
    opti.subject_to(u(0, i) >= 0.0);         // Throttle >= 0
    opti.subject_to(u(0, i) <= 1.0);         // Throttle <= 1
    opti.subject_to(x(6, i) >= 0.0);         // Fuel >= 0
}

// Objective: minimize fuel (maximize final mass)
opti.maximize(x(6, 50));  // Final fuel mass

auto sol = opti.solve();
```

### Collocation vs Multiple Shooting

```cpp
// Direct Collocation - polynomial defect constraints
janus::DirectCollocation dc(opti);
dc.setup(n_states, n_controls, t0, tf, {
    .scheme = janus::CollocationScheme::HermiteSimpson,  // 4th order
    .n_nodes = 51
});
dc.set_dynamics(dynamics);
dc.add_defect_constraints();  // Polynomial constraints

// Multiple Shooting - numerical integration constraints
janus::MultipleShooting ms(opti);
ms.setup(n_states, n_controls, t0, tf, {
    .n_intervals = 20,
    .integrator = "cvodes",  // SUNDIALS integrator
    .tol = 1e-8
});
ms.set_dynamics(dynamics);
ms.add_continuity_constraints();  // Integration-based constraints
```

### Integration with Staging

Trajectory optimization could be a **post-staging** analysis step:

```yaml
# simulation.yaml

staging:
  trim:
    enabled: true
    method: newton

  # Future: trajectory optimization config
  trajectory_optimization:
    enabled: false
    method: collocation          # collocation, multiple_shooting
    scheme: hermite_simpson      # For collocation
    n_nodes: 51

    # Time configuration
    t0: 0.0
    tf_guess: 300.0
    tf_free: true                # Optimize final time?

    # Control definition
    controls:
      - signal: GNC.throttle
        bounds: [0.0, 1.0]
      - signal: GNC.gimbal
        bounds: [-0.1, 0.1]

    # Boundary conditions
    initial_state: from_trim     # Use trim state as initial
    final_constraints:
      - state: EOM.altitude
        value: 200000.0
      - state: EOM.velocity
        value: 7800.0

    # Path constraints
    path_constraints:
      - signal: Propulsion.fuel_mass
        lower: 0.0

    # Objective
    objective:
      type: maximize
      signal: Propulsion.fuel_mass
      at: final
```

### Exit Criteria (Future Phase)

- [ ] `TrajectoryOptimizer` class wrapping `DirectCollocation` / `MultipleShooting`
- [ ] Extract dynamics function from symbolic `Simulator`
- [ ] Support both collocation schemes (Trapezoidal, Hermite-Simpson)
- [ ] Support multiple shooting with CVODES/IDAS integrators
- [ ] Path constraint support via signal queries
- [ ] Boundary condition support (initial, final, waypoints)
- [ ] Objective configuration (minimize/maximize time, fuel, etc.)
- [ ] Solution interpolation for simulation playback
- [ ] Warm-starting from previous solutions

---

## Exit Criteria

### Phase 4.0.7 (Core Infrastructure)

- [ ] `StageConfig<Scalar>` struct with trim/linearization/symbolics sub-configs
- [ ] `Stage()` lifecycle hook integrated into simulator
- [ ] Wiring validation during `Stage()`
- [ ] Configuration YAML schema for staging section

### Symbolic Mode (Already Exists)

> See [symbolic_orbital_demo.cpp](../../../examples/symbolic/symbolic_orbital_demo.cpp) for working implementation.

- [x] `TraceDynamics()` for dynamics graph extraction
- [x] `TraceStep()` for discrete step function
- [x] Jacobian computation via `janus::jacobian()`
- [x] Graph visualization (HTML, DOT, PDF)
- [x] Data dictionary export (YAML, JSON)

### Future: Trim Optimization

- [ ] *(Future)* `RunNumericTrim()` using `janus::NewtonSolver`
- [ ] *(Future)* `RunSymbolicTrim()` using `janus::Opti` / IPOPT
- [ ] *(Future)* `RunTrimSweep()` for parametric analysis
- [ ] *(Future)* Configurable zero_derivatives and control_signals
- [ ] *(Future)* Control bounds enforcement

### Future: Linearization

- [ ] *(Future)* `ComputeLinearModel()` at trim point
- [ ] *(Future)* State-space (A, B, C, D) extraction
- [ ] *(Future)* Export to MATLAB, NumPy, JSON formats

### Future: Advanced Symbolic Export

- [ ] *(Future)* CasADi function serialization
- [ ] *(Future)* Symbolic Jacobian export to files
