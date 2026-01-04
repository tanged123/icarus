# Trim Solver Implementation

**Status:** Proposed (Deferred from Phase 7)
**Phase:** 8.1
**Related:** [14_trim_optimization.md](../../architecture/14_trim_optimization.md) | [TrimSolver.hpp](../../../include/icarus/staging/TrimSolver.hpp)

---

## Overview

Extend the existing trim solver infrastructure to support full **NLP-based optimization** using `janus::Opti` and IPOPT. The current implementation provides Newton-based root-finding; Phase 7.1 adds constrained optimization for complex trim problems with inequality constraints, weighted residuals, and optimal trim (e.g., minimum fuel trim).

### Current State

The existing `TrimSolver` infrastructure (`include/icarus/staging/TrimSolver.hpp`) provides:

| Solver | Method | Use Case |
|:-------|:-------|:---------|
| `FiniteDifferenceTrim` | Newton with central differences | Simple equilibrium, no symbolics |
| `SymbolicTrim` | `janus::NewtonSolver` with exact Jacobians | Equilibrium with symbolic graph |
| `WarmstartSolver` | HDF5 state restoration | Mid-flight restart |

**Gap:** No support for:
- Constrained optimization (inequality constraints)
- Weighted residuals (over-determined systems)
- Optimal trim (minimize cost function subject to equilibrium)

### Target State

Add `OptiTrim` solver using `janus::Opti` for full NLP formulation:

```
minimize  ||W * (F(u) - targets)||²  +  cost(u)
   u
subject to:
   u_lower ≤ u ≤ u_upper       (bounds)
   g_eq(u) = 0                 (equality constraints)
   g_ineq(u) ≤ 0               (inequality constraints)
```

Where:
- `u` = free variables (controls, states)
- `F(u)` = derivative residuals to zero
- `W` = residual weights
- `cost(u)` = optional objective (e.g., minimize throttle)
- `g_eq`, `g_ineq` = user-defined constraints

---

## Tasks

### 7.1.1 OptiTrim Solver

**Goal:** NLP-based trim using `janus::Opti`.

#### New Files

##### [NEW] `include/icarus/staging/OptiTrim.hpp`

```cpp
#pragma once

#include <icarus/staging/TrimSolver.hpp>
#include <icarus/staging/SymbolicSimulatorCore.hpp>
#include <janus/optimization/Opti.hpp>

namespace icarus::staging {

/**
 * @brief Configuration for optimization-based trim
 */
struct OptiTrimConfig {
    // Inherit from base TrimConfig fields
    std::vector<std::string> control_signals;
    std::vector<std::string> zero_derivatives;
    std::unordered_map<std::string, double> initial_guesses;
    std::unordered_map<std::string, std::pair<double, double>> control_bounds;

    // Extended NLP options
    struct Weights {
        std::unordered_map<std::string, double> residual_weights;  // Per-derivative weights
        double default_weight = 1.0;
    };
    Weights weights;

    // Optional cost function (signal name to minimize/maximize)
    std::optional<std::string> minimize_signal;
    std::optional<std::string> maximize_signal;
    double cost_weight = 0.0;  // Weight on cost term (0 = equilibrium only)

    // Additional constraints
    std::vector<std::string> equality_constraints;    // Signals that must equal their targets
    std::vector<std::string> inequality_constraints;  // Signals that must be <= their targets
    std::unordered_map<std::string, double> constraint_targets;

    // Solver options
    janus::OptiOptions solver_options;
};

/**
 * @brief NLP-based trim solver using janus::Opti and IPOPT
 *
 * Solves constrained optimization problems for trim:
 *   - Supports bounds, equality, and inequality constraints
 *   - Weighted residuals for over-determined systems
 *   - Optional objective (optimal trim)
 *   - Exact gradients via symbolic differentiation
 */
class OptiTrim : public TrimSolver {
public:
    explicit OptiTrim(const OptiTrimConfig& config);

    TrimResult Solve(Simulator& sim, const TrimConfig& base_config) override;

private:
    OptiTrimConfig config_;

    /// Build NLP problem using janus::Opti
    janus::OptiSol BuildAndSolve(SymbolicSimulatorCore& sym_sim);

    /// Build weighted residual objective
    janus::SymbolicScalar BuildObjective(
        janus::Opti& opti,
        const janus::SymbolicVector& controls,
        SymbolicSimulatorCore& sym_sim);

    /// Add constraints to NLP
    void AddConstraints(
        janus::Opti& opti,
        const janus::SymbolicVector& controls,
        SymbolicSimulatorCore& sym_sim);
};

} // namespace icarus::staging
```

#### Implementation Notes

1. **Symbolic Graph Tracing:**
   - Create `SymbolicSimulatorCore` from config
   - Set control signals as symbolic variables
   - Trace one `Step()` to get derivatives as symbolic expressions
   - Build NLP objective from weighted residuals

2. **Weight Handling:**
   ```cpp
   SymbolicScalar BuildObjective(...) {
       SymbolicScalar obj = 0.0;
       for (const auto& deriv_name : config_.zero_derivatives) {
           SymbolicScalar deriv = sym_sim.GetSignal(deriv_name);
           double w = config_.weights.residual_weights.count(deriv_name)
               ? config_.weights.residual_weights.at(deriv_name)
               : config_.weights.default_weight;
           obj = obj + w * deriv * deriv;
       }
       // Optional cost term
       if (config_.minimize_signal && config_.cost_weight > 0) {
           obj = obj + config_.cost_weight * sym_sim.GetSignal(*config_.minimize_signal);
       }
       return obj;
   }
   ```

3. **Constraint Types:**
   - **Bounds:** Applied via `janus::Opti::subject_to_bounds()`
   - **Equality:** `opti.subject_to(signal == target)`
   - **Inequality:** `opti.subject_to(signal <= target)`

---

### 7.1.2 TrimConfig Extension

**Goal:** Extend `TrimConfig` to support NLP options.

#### Modifications

##### [MODIFY] `include/icarus/sim/SimulatorConfig.hpp`

Add NLP-specific fields to `TrimConfig`:

```diff
 struct TrimConfig {
     bool enabled = false;
     std::string mode = "equilibrium";  // "equilibrium", "warmstart", "optimization"
     std::string method = "finite-difference";  // "finite-difference", "newton", "ipopt"

     // Existing fields...
     std::vector<std::string> control_signals;
     std::vector<std::string> zero_derivatives;
     std::unordered_map<std::string, double> initial_guesses;
     std::unordered_map<std::string, std::pair<double, double>> control_bounds;

+    // NLP-specific options (used when method="ipopt")
+    struct NLPOptions {
+        // Weighted residuals
+        std::unordered_map<std::string, double> residual_weights;
+        double default_weight = 1.0;
+
+        // Optimal trim
+        std::optional<std::string> minimize_signal;
+        std::optional<std::string> maximize_signal;
+        double cost_weight = 0.0;
+
+        // Constraints
+        std::vector<std::string> equality_constraints;
+        std::vector<std::string> inequality_constraints;
+        std::unordered_map<std::string, double> constraint_targets;
+
+        // IPOPT options
+        int max_iter = 1000;
+        double tolerance = 1e-8;
+        bool verbose = false;
+    };
+    NLPOptions nlp;

     // Warmstart options...
 };
```

##### [MODIFY] `include/icarus/io/SimulationLoader.hpp`

Parse extended trim config from YAML:

```cpp
static void ParseTrimNLP(TrimConfig& cfg, const vulcan::io::YamlNode& node) {
    if (!node.Has("nlp")) return;
    auto nlp = node["nlp"];

    // Weights
    if (nlp.Has("weights")) {
        auto weights = nlp["weights"];
        for (const auto& key : weights.Keys()) {
            cfg.nlp.residual_weights[key] = weights.Get<double>(key);
        }
        cfg.nlp.default_weight = weights.Get<double>("_default", 1.0);
    }

    // Optimal trim
    cfg.nlp.minimize_signal = nlp.Get<std::string>("minimize", "");
    cfg.nlp.maximize_signal = nlp.Get<std::string>("maximize", "");
    cfg.nlp.cost_weight = nlp.Get<double>("cost_weight", 0.0);

    // Constraints
    if (nlp.Has("equality_constraints")) {
        cfg.nlp.equality_constraints = nlp.GetVector<std::string>("equality_constraints");
    }
    if (nlp.Has("inequality_constraints")) {
        cfg.nlp.inequality_constraints = nlp.GetVector<std::string>("inequality_constraints");
    }
}
```

---

### 7.1.3 Factory Update

**Goal:** Integrate `OptiTrim` into solver factory.

##### [MODIFY] `TrimSolver.hpp` - `CreateTrimSolver()`

```diff
 inline std::unique_ptr<TrimSolver> CreateTrimSolver(const TrimConfig& config,
                                                     bool symbolic_enabled) {
     if (config.IsWarmstart()) {
         return std::make_unique<WarmstartSolver>();
     }

+    // NLP-based optimization (requires symbolics)
+    if (config.method == "ipopt" || config.mode == "optimization") {
+        if (!symbolic_enabled) {
+            throw ConfigError("OptiTrim requires staging.symbolics.enabled = true");
+        }
+        return std::make_unique<OptiTrim>(config);
+    }
+
     // Equilibrium root-finding
     if (symbolic_enabled && config.method == "newton") {
         return std::make_unique<SymbolicTrim>();
     }

     // Default: finite differences
     FiniteDifferenceTrim::Options opts;
     opts.tolerance = config.tolerance;
     opts.max_iterations = config.max_iterations;
     return std::make_unique<FiniteDifferenceTrim>(opts);
 }
```

---

### 7.1.4 YAML Configuration Support

**Goal:** Support full NLP configuration from YAML.

#### Example Configuration

```yaml
# scenarios/x15_steady_cruise.yaml
staging:
  symbolics:
    enabled: true

  trim:
    enabled: true
    mode: optimization    # NLP-based (not just root-finding)
    method: ipopt         # Use janus::Opti with IPOPT

    # What to solve for
    control_signals:
      - X15.Aero.alpha
      - X15.MainEngine.throttle
      - X15.Aero.elevator

    # Initial guesses
    initial_guesses:
      X15.Aero.alpha: 0.05
      X15.MainEngine.throttle: 0.5
      X15.Aero.elevator: 0.0

    # Bounds on controls
    control_bounds:
      X15.Aero.alpha: [-0.3, 0.3]
      X15.MainEngine.throttle: [0.0, 1.0]
      X15.Aero.elevator: [-0.5, 0.5]

    # What must be zero (weighted residuals)
    zero_derivatives:
      - X15.EOM.accel.x
      - X15.EOM.accel.z
      - X15.EOM.angular_accel.y

    # NLP-specific options
    nlp:
      # Weighted residuals (prioritize pitch trim)
      weights:
        X15.EOM.accel.x: 1.0
        X15.EOM.accel.z: 1.0
        X15.EOM.angular_accel.y: 10.0

      # Optional: minimize throttle (fuel-optimal trim)
      minimize: X15.MainEngine.throttle
      cost_weight: 0.01  # Small weight so equilibrium dominates

      # IPOPT options
      max_iter: 1000
      tolerance: 1e-8
      verbose: false
```

---

## Verification Plan

### Unit Tests

#### New: `tests/staging/test_opti_trim.cpp`

```cpp
// 1. Basic Equilibrium
TEST(OptiTrim, SolvesSimpleEquilibrium) {
    // Create sim with point mass + gravity
    // Trim for zero vertical acceleration
    // Verify converged with residual < tolerance
}

// 2. Weighted Residuals
TEST(OptiTrim, RespectsResidualWeights) {
    // Over-determined system with conflicting targets
    // Verify higher-weighted residuals are smaller
}

// 3. Bounds Enforcement
TEST(OptiTrim, EnforcesBounds) {
    // Set tight bounds on controls
    // Verify solution respects bounds
}

// 4. Optimal Trim
TEST(OptiTrim, MinimizesObjective) {
    // Trim with minimize_signal set
    // Compare to baseline equilibrium trim
    // Verify minimized signal is lower
}

// 5. Inequality Constraints
TEST(OptiTrim, HandlesInequalityConstraints) {
    // Add constraint: alpha <= max_alpha
    // Verify constraint satisfied
}

// 6. Convergence Failure
TEST(OptiTrim, ReportsFailure) {
    // Infeasible constraints
    // Verify result.converged = false with message
}
```

### Integration Tests

#### New: `tests/integration/test_trim_x15.cpp`

1. **X15 Steady Cruise Trim**
   - Load full X15 6DOF scenario
   - Trim for level flight at specified altitude/velocity
   - Verify accelerations < 1e-6 m/s²

2. **Compare Solvers**
   - Same trim problem with `finite-difference`, `newton`, and `ipopt`
   - Verify all converge to same solution (within tolerance)

### Example Verification

Add `examples/trim/optimal_trim_demo.cpp`:

```cpp
// Demonstrates:
// 1. Setting up NLP trim problem
// 2. Weighted residuals
// 3. Optimal trim (min fuel)
// 4. Constraint handling
```

---

## Dependencies

### Required

- `janus::Opti` (already available via `janus/optimization/Opti.hpp`)
- `SymbolicSimulatorCore` (existing from Phase 6)
- IPOPT solver (linked via CasADi in Nix flake)

### No New External Dependencies

The Nix flake already includes CasADi with IPOPT support via Janus.

---

## Migration Notes

### Backward Compatibility

- Existing `method: "finite-difference"` and `method: "newton"` continue to work
- `method: "ipopt"` enables new NLP solver
- `mode: "optimization"` (vs `mode: "equilibrium"`) triggers NLP even without explicit method

### Default Behavior

- If `method` not specified and `nlp` section present → use `ipopt`
- If `method` not specified and no `nlp` section → use `finite-difference` (existing default)

---

## Open Questions

1. **Sparse vs Dense Jacobians:** Should we enable CasADi's sparsity detection for large trim problems?

2. **Warm-starting NLP:** Should we cache the NLP solution for parameter sweeps (e.g., varying altitude)?

3. **Multi-start:** For non-convex problems, should we support multiple initial guesses?

4. **Constraint Scaling:** Should weights also apply to constraints, or just residuals?
