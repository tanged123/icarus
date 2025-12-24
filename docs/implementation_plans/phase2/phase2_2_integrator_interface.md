# Phase 2.2: Integrator Interface Implementation Plan

**Status:** Proposed
**Target:** Modular, user-configurable integrator interface with multiple Janus methods

---

## Overview

Phase 2.2 establishes a **modular integration framework** that advances simulation state over time. The architecture provides:

1. **Decoupled integration** — Components compute derivatives, integrators advance state
2. **Fixed-step primary** — RK4 default for deterministic, frame-synchronized simulation
3. **Multiple methods** — Euler, RK2, RK4 (fixed-step), RK45 (adaptive, offline only)
4. **Runtime switching** — Change integrator method without recompilation
5. **User configuration** — Integrator type and parameters configurable via API/config
6. **Symbolic tracing** — Same code generates CasADi computation graphs for optimization

> [!IMPORTANT]
> **Determinism Requirement:** For real-time simulation, hardware-in-the-loop, and reproducible Monte Carlo runs, use **fixed-step integrators only** (Euler, RK2, RK4). Adaptive RK45 is provided for offline analysis and trajectory optimization where frame synchronization is not required.

```
┌─────────────────────────────────────────────────────────────┐
│                  MODULAR INTEGRATOR DESIGN                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   User Config / API                                         │
│        │                                                    │
│        ▼                                                    │
│   ┌─────────────────┐                                      │
│   │IntegratorFactory│ ← Creates integrator from config     │
│   └────────┬────────┘                                      │
│            │                                                │
│   ┌────────▼────────┐                                      │
│   │   Integrator    │ ← Abstract interface                 │
│   │   (polymorphic) │                                      │
│   └────────┬────────┘                                      │
│            │                                                │
│   ┌────────┴────────┬──────────┬──────────┐               │
│   ▼                 ▼          ▼          ▼               │
│ Euler            RK2        RK4        RK45               │
│ (1st order)   (2nd order) (4th order) (adaptive)          │
│                                                             │
│   All wrap corresponding janus::*_step() functions          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

```
┌─────────────────────────────────────────────────────────────┐
│                    INTEGRATION FLOW                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   Simulator.Step(dt)                                        │
│        │                                                    │
│        ▼                                                    │
│   ┌─────────────┐                                          │
│   │ Integrator  │ ─── Uses janus::rk4_step() or rk45_step()│
│   │   .Step()   │                                          │
│   └──────┬──────┘                                          │
│          │                                                  │
│    ┌─────▼─────┐    ┌─────────────┐    ┌─────────────┐    │
│    │ Get State │───►│ Compute     │───►│ Set State   │    │
│    │ X_global_ │    │ Derivatives │    │ X_new       │    │
│    └───────────┘    └─────────────┘    └─────────────┘    │
│                           │                                 │
│                   ┌───────▼───────┐                        │
│                   │  Components   │                        │
│                   │  Step() each  │                        │
│                   │  writes X_dot │                        │
│                   └───────────────┘                        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Architecture Alignment

Per [09_memory_state_ownership.md](../../architecture/09_memory_state_ownership.md):

### Integration Ownership Model

| Entity | Responsibility | Lifetime |
|:-------|:---------------|:---------|
| Integrator | Advance state using derivatives | Owned by Simulator |
| Simulator | Provide derivative function | Permanent |
| Components | Compute derivatives via Step() | Per-component |
| X_global_ | Authoritative state | Simulator-owned |
| X_dot_global_ | Derivative buffer | Simulator-owned |

### Integration Protocol

```
For each Simulator.Step(dt):
  1. Integrator receives derivative function λ(t, X) → X_dot
  2. Integrator calls Janus step function (rk4_step or rk45_step)
  3. Janus step function evaluates λ at multiple points (k₁, k₂, k₃, k₄)
  4. Each λ evaluation:
     a. Simulator.SetState(X)
     b. Simulator.ComputeDerivatives(t)
     c. Return X_dot_global_
  5. Integrator returns X_new
  6. Simulator.SetState(X_new)
  7. Simulator.time_ += dt
```

---

## Janus Integration API

Janus provides four step integrators in [`IntegratorStep.hpp`](file:///home/tanged/sources/references/janus/include/janus/math/IntegratorStep.hpp):

### Fixed-Step Methods

```cpp
// Forward Euler (1st order, 1 evaluation)
template <typename Scalar, typename Func>
JanusVector<Scalar> euler_step(Func&& f, const JanusVector<Scalar>& x, Scalar t, Scalar dt);

// Heun's Method / RK2 (2nd order, 2 evaluations)
template <typename Scalar, typename Func>
JanusVector<Scalar> rk2_step(Func&& f, const JanusVector<Scalar>& x, Scalar t, Scalar dt);

// Classic RK4 (4th order, 4 evaluations)
template <typename Scalar, typename Func>
JanusVector<Scalar> rk4_step(Func&& f, const JanusVector<Scalar>& x, Scalar t, Scalar dt);
```

### Adaptive Method

```cpp
// Dormand-Prince RK45 (4th/5th order, 7 evaluations + error estimate)
template <typename Scalar, typename Func>
RK45Result<Scalar> rk45_step(Func&& f, const JanusVector<Scalar>& x, Scalar t, Scalar dt);

template <typename Scalar>
struct RK45Result {
    JanusVector<Scalar> y5;    // 5th-order solution (use this)
    JanusVector<Scalar> y4;    // 4th-order solution (for error)
    Scalar error;              // ||y5 - y4|| local truncation error
};
```

### Function Signature

All Janus integrators expect:
```cpp
f(Scalar t, const JanusVector<Scalar>& x) -> JanusVector<Scalar>
```

---

## Modular Configuration System

### Integrator Type Enumeration

```cpp
/**
 * @brief Available integrator methods
 *
 * Maps directly to Janus step functions.
 */
enum class IntegratorType {
    Euler,   ///< Forward Euler (1st order, 1 eval) - janus::euler_step
    RK2,     ///< Heun's method (2nd order, 2 evals) - janus::rk2_step
    RK4,     ///< Classic RK4 (4th order, 4 evals) - janus::rk4_step
    RK45     ///< Dormand-Prince adaptive (5th order, 7 evals) - janus::rk45_step
};

/**
 * @brief Convert integrator type to string
 */
[[nodiscard]] inline std::string to_string(IntegratorType type) {
    switch (type) {
        case IntegratorType::Euler: return "Euler";
        case IntegratorType::RK2:   return "RK2";
        case IntegratorType::RK4:   return "RK4";
        case IntegratorType::RK45:  return "RK45";
    }
    return "Unknown";
}

/**
 * @brief Parse integrator type from string
 */
[[nodiscard]] inline IntegratorType parse_integrator_type(const std::string& name) {
    if (name == "Euler" || name == "euler") return IntegratorType::Euler;
    if (name == "RK2" || name == "rk2")     return IntegratorType::RK2;
    if (name == "RK4" || name == "rk4")     return IntegratorType::RK4;
    if (name == "RK45" || name == "rk45")   return IntegratorType::RK45;
    throw std::invalid_argument("Unknown integrator type: " + name);
}
```

### Integrator Configuration

```cpp
/**
 * @brief Configuration for integrator creation
 *
 * Supports both fixed-step and adaptive integrators.
 * Future: loadable from YAML configuration.
 */
template <typename Scalar>
struct IntegratorConfig {
    IntegratorType type = IntegratorType::RK4;  ///< Method to use

    // Adaptive stepping parameters (RK45 only)
    Scalar abs_tol = Scalar{1e-6};    ///< Absolute tolerance
    Scalar rel_tol = Scalar{1e-6};    ///< Relative tolerance
    Scalar min_dt = Scalar{1e-10};    ///< Minimum step size
    Scalar max_dt = Scalar{1.0};      ///< Maximum step size
    Scalar safety_factor = Scalar{0.9}; ///< Step size safety factor

    /**
     * @brief Create default RK4 config
     */
    static IntegratorConfig RK4Default() {
        IntegratorConfig cfg;
        cfg.type = IntegratorType::RK4;
        return cfg;
    }

    /**
     * @brief Create adaptive RK45 config with tolerances
     */
    static IntegratorConfig RK45Adaptive(Scalar abs_tol = Scalar{1e-6},
                                         Scalar rel_tol = Scalar{1e-6}) {
        IntegratorConfig cfg;
        cfg.type = IntegratorType::RK45;
        cfg.abs_tol = abs_tol;
        cfg.rel_tol = rel_tol;
        return cfg;
    }

    /**
     * @brief Create config for testing/comparison
     */
    static IntegratorConfig ForMethod(IntegratorType method) {
        IntegratorConfig cfg;
        cfg.type = method;
        return cfg;
    }
};
```

### Integrator Factory

```cpp
/**
 * @brief Factory for creating integrators from configuration
 *
 * Enables runtime selection of integration method without
 * hardcoding specific integrator types.
 */
template <typename Scalar>
class IntegratorFactory {
public:
    /**
     * @brief Create integrator from configuration
     *
     * @param config Integrator configuration
     * @return Unique pointer to configured integrator
     */
    static std::unique_ptr<Integrator<Scalar>> Create(
        const IntegratorConfig<Scalar>& config
    ) {
        switch (config.type) {
            case IntegratorType::Euler:
                return std::make_unique<EulerIntegrator<Scalar>>();

            case IntegratorType::RK2:
                return std::make_unique<RK2Integrator<Scalar>>();

            case IntegratorType::RK4:
                return std::make_unique<RK4Integrator<Scalar>>();

            case IntegratorType::RK45: {
                auto rk45 = std::make_unique<RK45Integrator<Scalar>>(
                    config.abs_tol, config.rel_tol
                );
                rk45->SetMinDt(config.min_dt);
                rk45->SetMaxDt(config.max_dt);
                rk45->SetSafetyFactor(config.safety_factor);
                return rk45;
            }
        }
        throw std::invalid_argument("Unknown integrator type");
    }

    /**
     * @brief Create integrator from type name string
     *
     * Convenience for configuration file parsing.
     */
    static std::unique_ptr<Integrator<Scalar>> Create(const std::string& type_name) {
        return Create(IntegratorConfig<Scalar>::ForMethod(
            parse_integrator_type(type_name)
        ));
    }

    /**
     * @brief Create default integrator (RK4)
     */
    static std::unique_ptr<Integrator<Scalar>> CreateDefault() {
        return Create(IntegratorConfig<Scalar>::RK4Default());
    }
};
```

### Simulator Integration with Modular Config

```cpp
template <typename Scalar>
class Simulator {
public:
    /**
     * @brief Set integrator using configuration
     *
     * Primary API for user-configurable integration.
     */
    void SetIntegrator(const IntegratorConfig<Scalar>& config) {
        integrator_ = IntegratorFactory<Scalar>::Create(config);
        integrator_config_ = config;
    }

    /**
     * @brief Set integrator by type name
     *
     * Convenience for quick switching during development/testing.
     */
    void SetIntegrator(const std::string& type_name) {
        SetIntegrator(IntegratorConfig<Scalar>::ForMethod(
            parse_integrator_type(type_name)
        ));
    }

    /**
     * @brief Set integrator by type enum
     */
    void SetIntegrator(IntegratorType type) {
        SetIntegrator(IntegratorConfig<Scalar>::ForMethod(type));
    }

    /**
     * @brief Get current integrator configuration
     */
    [[nodiscard]] const IntegratorConfig<Scalar>& GetIntegratorConfig() const {
        return integrator_config_;
    }

    /**
     * @brief Get current integrator type
     */
    [[nodiscard]] IntegratorType GetIntegratorType() const {
        return integrator_config_.type;
    }

private:
    // Default to RK4 integrator
    std::unique_ptr<Integrator<Scalar>> integrator_ =
        std::make_unique<RK4Integrator<Scalar>>();
    IntegratorConfig<Scalar> integrator_config_ =
        IntegratorConfig<Scalar>::RK4Default();
};
```

### Usage Examples

```cpp
// Method 1: Direct type enum
sim.SetIntegrator(IntegratorType::RK4);

// Method 2: String name (from config file)
sim.SetIntegrator("rk45");

// Method 3: Full configuration
IntegratorConfig<double> config;
config.type = IntegratorType::RK45;
config.abs_tol = 1e-8;
config.rel_tol = 1e-8;
config.min_dt = 1e-12;
sim.SetIntegrator(config);

// Method 4: Factory helpers
sim.SetIntegrator(IntegratorConfig<double>::RK45Adaptive(1e-9, 1e-9));

// Method 5: Direct integrator (advanced use)
sim.SetIntegrator(std::make_unique<RK4Integrator<double>>());

// Query current method
std::cout << "Using: " << to_string(sim.GetIntegratorType()) << std::endl;
```

### Future: YAML Configuration

```yaml
# scenarios/example.yaml
simulator:
  integrator:
    type: RK4          # Use fixed-step for determinism
    # type: RK45       # Only for offline analysis
    # abs_tol: 1.0e-8  # RK45 only
    # rel_tol: 1.0e-8  # RK45 only
```

---

## When to Use Each Integrator

| Integrator | Order | Use Case | Deterministic |
|:-----------|:------|:---------|:--------------|
| **RK4** | 4th | Real-time sim, HIL, Monte Carlo, frame-by-frame output | ✅ Yes |
| RK2 | 2nd | Fast prototyping, less accuracy needed | ✅ Yes |
| Euler | 1st | Debugging, understanding dynamics | ✅ Yes |
| RK45 | 5th | Trajectory optimization, offline analysis, validation | ❌ No* |

*RK45 step sizes vary based on error, making it non-deterministic across runs.

### Recommended Patterns

**Real-time / Deterministic Simulation:**
```cpp
sim.SetIntegrator(IntegratorType::RK4);  // Default, fixed-step
sim.Step(0.01);  // Always exactly 10ms steps
```

**Trajectory Optimization (Symbolic Mode):**
```cpp
Simulator<casadi::MX> sim;
sim.SetIntegrator(IntegratorType::RK4);  // Still RK4 for graph structure
// CasADi handles optimization, not adaptive stepping
```

**Offline High-Fidelity Reference:**
```cpp
sim.SetIntegrator(IntegratorConfig<double>::RK45Adaptive(1e-10, 1e-10));
// Use only when comparing against fixed-step or generating truth data
```

---

## Proposed Implementation

### 1. Abstract Integrator Interface

#### [MODIFY] `include/icarus/sim/Integrator.hpp`

```cpp
#pragma once

/**
 * @file Integrator.hpp
 * @brief Abstract interface for numerical integrators
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <icarus/core/Types.hpp>
#include <functional>

namespace icarus {

/**
 * @brief Result from adaptive step integrators
 */
template <typename Scalar>
struct AdaptiveStepResult {
    JanusVector<Scalar> state;     ///< New state at t + dt_actual
    Scalar dt_actual;              ///< Actual step taken (may differ from requested)
    Scalar error_estimate;         ///< Local truncation error estimate
    bool accepted;                 ///< Whether step was accepted
};

/**
 * @brief Abstract interface for numerical integrators
 *
 * Integrators advance state using derivative information from the Simulator.
 * All implementations must be templated on Scalar for symbolic mode support.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar>
class Integrator {
public:
    virtual ~Integrator() = default;

    /**
     * @brief Derivative function signature
     *
     * Maps (t, X) → dX/dt
     */
    using DerivativeFunc =
        std::function<JanusVector<Scalar>(Scalar t, const JanusVector<Scalar>& x)>;

    /**
     * @brief Advance state by one step
     *
     * @param f Derivative function
     * @param x Current state vector
     * @param t Current time
     * @param dt Requested time step
     * @return New state at t + dt
     */
    virtual JanusVector<Scalar> Step(
        const DerivativeFunc& f,
        const JanusVector<Scalar>& x,
        Scalar t,
        Scalar dt
    ) = 0;

    /**
     * @brief Get integrator name for logging
     */
    [[nodiscard]] virtual std::string Name() const = 0;

    /**
     * @brief Get integrator order (for error analysis)
     */
    [[nodiscard]] virtual int Order() const = 0;

    /**
     * @brief Check if integrator supports adaptive stepping
     */
    [[nodiscard]] virtual bool IsAdaptive() const { return false; }
};

/**
 * @brief Interface for adaptive step integrators
 *
 * Extends base Integrator with error-controlled stepping.
 */
template <typename Scalar>
class AdaptiveIntegrator : public Integrator<Scalar> {
public:
    using typename Integrator<Scalar>::DerivativeFunc;

    /**
     * @brief Advance with error control
     *
     * May take a smaller step than requested to meet tolerance.
     *
     * @param f Derivative function
     * @param x Current state vector
     * @param t Current time
     * @param dt Requested time step
     * @return Result with actual step, error, and acceptance status
     */
    virtual AdaptiveStepResult<Scalar> AdaptiveStep(
        const DerivativeFunc& f,
        const JanusVector<Scalar>& x,
        Scalar t,
        Scalar dt
    ) = 0;

    [[nodiscard]] bool IsAdaptive() const override { return true; }

    /**
     * @brief Set absolute tolerance
     */
    virtual void SetAbsTol(Scalar tol) = 0;

    /**
     * @brief Set relative tolerance
     */
    virtual void SetRelTol(Scalar tol) = 0;

    /**
     * @brief Get current absolute tolerance
     */
    [[nodiscard]] virtual Scalar GetAbsTol() const = 0;

    /**
     * @brief Get current relative tolerance
     */
    [[nodiscard]] virtual Scalar GetRelTol() const = 0;
};

} // namespace icarus
```

---

### 2. RK4 Integrator Implementation

#### [NEW] `include/icarus/sim/RK4Integrator.hpp`

```cpp
#pragma once

/**
 * @file RK4Integrator.hpp
 * @brief Classic 4th-order Runge-Kutta integrator
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <icarus/sim/Integrator.hpp>
#include <janus/math/IntegratorStep.hpp>

namespace icarus {

/**
 * @brief Classic 4th-order Runge-Kutta integrator
 *
 * Fixed-step method with 4 function evaluations per step.
 * Wraps janus::rk4_step() for dual-mode compatibility.
 *
 * **Butcher Tableau:**
 * ```
 * k₁ = f(t, x)
 * k₂ = f(t + dt/2, x + dt/2·k₁)
 * k₃ = f(t + dt/2, x + dt/2·k₂)
 * k₄ = f(t + dt, x + dt·k₃)
 * x_{n+1} = x + (dt/6)(k₁ + 2k₂ + 2k₃ + k₄)
 * ```
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar>
class RK4Integrator : public Integrator<Scalar> {
public:
    using typename Integrator<Scalar>::DerivativeFunc;

    JanusVector<Scalar> Step(
        const DerivativeFunc& f,
        const JanusVector<Scalar>& x,
        Scalar t,
        Scalar dt
    ) override {
        return janus::rk4_step(f, x, t, dt);
    }

    [[nodiscard]] std::string Name() const override { return "RK4"; }
    [[nodiscard]] int Order() const override { return 4; }
};

/**
 * @brief Forward Euler integrator (1st order)
 *
 * Simplest integrator, mainly for testing and comparison.
 */
template <typename Scalar>
class EulerIntegrator : public Integrator<Scalar> {
public:
    using typename Integrator<Scalar>::DerivativeFunc;

    JanusVector<Scalar> Step(
        const DerivativeFunc& f,
        const JanusVector<Scalar>& x,
        Scalar t,
        Scalar dt
    ) override {
        return janus::euler_step(f, x, t, dt);
    }

    [[nodiscard]] std::string Name() const override { return "Euler"; }
    [[nodiscard]] int Order() const override { return 1; }
};

/**
 * @brief Heun's method / RK2 integrator (2nd order)
 *
 * Two-stage method, useful for comparison studies.
 */
template <typename Scalar>
class RK2Integrator : public Integrator<Scalar> {
public:
    using typename Integrator<Scalar>::DerivativeFunc;

    JanusVector<Scalar> Step(
        const DerivativeFunc& f,
        const JanusVector<Scalar>& x,
        Scalar t,
        Scalar dt
    ) override {
        return janus::rk2_step(f, x, t, dt);
    }

    [[nodiscard]] std::string Name() const override { return "RK2"; }
    [[nodiscard]] int Order() const override { return 2; }
};

} // namespace icarus
```

---

### 3. Adaptive RK45 Integrator Implementation

#### [NEW] `include/icarus/sim/RK45Integrator.hpp`

```cpp
#pragma once

/**
 * @file RK45Integrator.hpp
 * @brief Adaptive Dormand-Prince RK45 integrator
 *
 * Part of Phase 2.2: Integrator Interface
 */

#include <icarus/sim/Integrator.hpp>
#include <janus/math/IntegratorStep.hpp>
#include <algorithm>
#include <cmath>

namespace icarus {

/**
 * @brief Adaptive Dormand-Prince RK4(5) integrator
 *
 * Uses embedded 4th and 5th order solutions to estimate local
 * truncation error. Adjusts step size to maintain error within tolerance.
 *
 * **Features:**
 * - 7 function evaluations per step (shares stages)
 * - Automatic step size control
 * - Configurable absolute and relative tolerances
 * - Step statistics tracking
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 */
template <typename Scalar>
class RK45Integrator : public AdaptiveIntegrator<Scalar> {
public:
    using typename Integrator<Scalar>::DerivativeFunc;

    /**
     * @brief Construct with default tolerances
     *
     * @param abs_tol Absolute tolerance (default 1e-6)
     * @param rel_tol Relative tolerance (default 1e-6)
     */
    explicit RK45Integrator(Scalar abs_tol = Scalar{1e-6},
                           Scalar rel_tol = Scalar{1e-6})
        : abs_tol_(abs_tol), rel_tol_(rel_tol) {}

    /**
     * @brief Fixed-step interface (uses requested dt directly)
     *
     * For compatibility with base Integrator interface.
     * No step adaptation; returns 5th-order solution.
     */
    JanusVector<Scalar> Step(
        const DerivativeFunc& f,
        const JanusVector<Scalar>& x,
        Scalar t,
        Scalar dt
    ) override {
        auto result = janus::rk45_step(f, x, t, dt);
        return result.y5;
    }

    /**
     * @brief Adaptive step with error control
     *
     * May sub-step to meet tolerance requirements.
     */
    AdaptiveStepResult<Scalar> AdaptiveStep(
        const DerivativeFunc& f,
        const JanusVector<Scalar>& x,
        Scalar t,
        Scalar dt
    ) override {
        // Compute RK45 step
        auto result = janus::rk45_step(f, x, t, dt);

        // Compute error tolerance
        Scalar tol = ComputeTolerance(x, result.y5);

        // Check if step is accepted
        bool accepted = LessThanOrEqual(result.error, tol);

        if (accepted) {
            stats_.accepted_steps++;
        } else {
            stats_.rejected_steps++;
        }

        return AdaptiveStepResult<Scalar>{
            result.y5,
            dt,
            result.error,
            accepted
        };
    }

    // Tolerance setters/getters
    void SetAbsTol(Scalar tol) override { abs_tol_ = tol; }
    void SetRelTol(Scalar tol) override { rel_tol_ = tol; }
    [[nodiscard]] Scalar GetAbsTol() const override { return abs_tol_; }
    [[nodiscard]] Scalar GetRelTol() const override { return rel_tol_; }

    [[nodiscard]] std::string Name() const override { return "RK45"; }
    [[nodiscard]] int Order() const override { return 5; }

    // Step size control parameters
    void SetMinDt(Scalar min_dt) { min_dt_ = min_dt; }
    void SetMaxDt(Scalar max_dt) { max_dt_ = max_dt; }
    void SetSafetyFactor(Scalar factor) { safety_ = factor; }

    [[nodiscard]] Scalar GetMinDt() const { return min_dt_; }
    [[nodiscard]] Scalar GetMaxDt() const { return max_dt_; }

    /**
     * @brief Suggest next step size based on error
     *
     * @param dt Current step size
     * @param error Current error estimate
     * @param tol Error tolerance
     * @return Suggested step size for next step
     */
    [[nodiscard]] Scalar SuggestDt(Scalar dt, Scalar error, Scalar tol) const {
        // Optimal step size formula: dt_new = dt * safety * (tol/error)^(1/5)
        // Use 5th root for RK45 (5th order method)
        Scalar ratio = tol / (error + Scalar{1e-15});  // Avoid division by zero
        Scalar factor = safety_ * std::pow(static_cast<double>(ratio), 0.2);

        // Limit growth/shrinkage
        factor = std::max(0.1, std::min(5.0, factor));

        Scalar dt_new = dt * Scalar{factor};

        // Clamp to min/max
        return std::max(min_dt_, std::min(max_dt_, dt_new));
    }

    /**
     * @brief Statistics for adaptive stepping
     */
    struct Statistics {
        std::size_t accepted_steps = 0;
        std::size_t rejected_steps = 0;

        void Reset() {
            accepted_steps = 0;
            rejected_steps = 0;
        }

        [[nodiscard]] double AcceptanceRate() const {
            auto total = accepted_steps + rejected_steps;
            return total > 0 ? static_cast<double>(accepted_steps) / total : 1.0;
        }
    };

    [[nodiscard]] const Statistics& GetStatistics() const { return stats_; }
    void ResetStatistics() { stats_.Reset(); }

private:
    Scalar abs_tol_;
    Scalar rel_tol_;
    Scalar min_dt_{1e-10};
    Scalar max_dt_{1.0};
    Scalar safety_{0.9};
    Statistics stats_;

    /**
     * @brief Compute error tolerance for given states
     *
     * tolerance = abs_tol + rel_tol * max(|x|, |x_new|)
     */
    [[nodiscard]] Scalar ComputeTolerance(
        const JanusVector<Scalar>& x,
        const JanusVector<Scalar>& x_new
    ) const {
        Scalar max_norm = Scalar{0};
        for (Eigen::Index i = 0; i < x.size(); ++i) {
            Scalar xi_abs = janus::abs(x[i]);
            Scalar xi_new_abs = janus::abs(x_new[i]);
            max_norm = janus::max(max_norm, janus::max(xi_abs, xi_new_abs));
        }
        return abs_tol_ + rel_tol_ * max_norm;
    }

    /**
     * @brief Compare Scalar values (symbolic-safe)
     */
    [[nodiscard]] bool LessThanOrEqual(Scalar a, Scalar b) const {
        // For numeric mode, direct comparison
        // For symbolic mode, this is only used in statistics (not traced)
        if constexpr (std::is_same_v<Scalar, double>) {
            return a <= b;
        } else {
            // For symbolic, evaluate numerically if possible
            // This branch only used in adaptive control, not traced
            return true;  // Accept in symbolic mode
        }
    }
};

} // namespace icarus
```

---

### 4. Simulator Integration

#### [MODIFY] `include/icarus/sim/Simulator.hpp`

Add integrator support:

```cpp
template <typename Scalar>
class Simulator {
public:
    // ... existing interface ...

    // =========================================================================
    // Integrator Interface (Phase 2.2)
    // =========================================================================

    /**
     * @brief Set the integrator to use
     *
     * @param integrator Unique pointer to integrator (Simulator takes ownership)
     */
    void SetIntegrator(std::unique_ptr<Integrator<Scalar>> integrator) {
        integrator_ = std::move(integrator);
    }

    /**
     * @brief Get current integrator
     */
    [[nodiscard]] Integrator<Scalar>* GetIntegrator() const {
        return integrator_.get();
    }

    /**
     * @brief Execute one time step using configured integrator
     */
    void Step(Scalar dt) {
        if (phase_ != Phase::Staged && phase_ != Phase::Running) {
            throw LifecycleError("Step() requires prior Stage()");
        }
        phase_ = Phase::Running;

        // Create derivative function for integrator
        auto deriv_func = [this](Scalar t, const JanusVector<Scalar>& x)
            -> JanusVector<Scalar> {
            this->SetState(x);
            return this->ComputeDerivatives(t);
        };

        // Get current state
        JanusVector<Scalar> X = GetState();

        // Integrate (integrator_ always valid, defaults to RK4)
        JanusVector<Scalar> X_new = integrator_->Step(deriv_func, X, time_, dt);

        // Update state
        SetState(X_new);
        time_ = time_ + dt;
    }

    /**
     * @brief Execute adaptive step (for RK45)
     *
     * Returns actual step taken and error estimate.
     * Requires adaptive integrator.
     */
    AdaptiveStepResult<Scalar> AdaptiveStep(Scalar dt_request) {
        if (phase_ != Phase::Staged && phase_ != Phase::Running) {
            throw LifecycleError("AdaptiveStep() requires prior Stage()");
        }
        phase_ = Phase::Running;

        auto* adaptive = dynamic_cast<AdaptiveIntegrator<Scalar>*>(integrator_.get());
        if (!adaptive) {
            throw IntegratorError("AdaptiveStep() requires an AdaptiveIntegrator");
        }

        auto deriv_func = [this](Scalar t, const JanusVector<Scalar>& x)
            -> JanusVector<Scalar> {
            this->SetState(x);
            return this->ComputeDerivatives(t);
        };

        JanusVector<Scalar> X = GetState();
        auto result = adaptive->AdaptiveStep(deriv_func, X, time_, dt_request);

        if (result.accepted) {
            SetState(result.state);
            time_ = time_ + result.dt_actual;
        }

        return result;
    }

private:
    // Default to RK4 integrator (always valid, no null checks needed)
    std::unique_ptr<Integrator<Scalar>> integrator_ =
        std::make_unique<RK4Integrator<Scalar>>();
    IntegratorConfig<Scalar> integrator_config_ =
        IntegratorConfig<Scalar>::RK4Default();
};
```

---

### 5. Error Types for Integrator

#### [MODIFY] `include/icarus/core/Error.hpp`

Add integrator-related exceptions:

```cpp
namespace icarus {

// ... existing exceptions ...

/**
 * @brief Base class for integrator errors
 */
class IntegratorError : public SimulationError {
public:
    using SimulationError::SimulationError;
};

/**
 * @brief Thrown when integration fails to converge
 */
class IntegrationFailedError : public IntegratorError {
public:
    IntegrationFailedError(Scalar t, Scalar dt, const std::string& reason)
        : IntegratorError("Integration failed at t=" + std::to_string(t) +
                         ", dt=" + std::to_string(dt) + ": " + reason),
          t_(t), dt_(dt) {}

    [[nodiscard]] Scalar time() const { return t_; }
    [[nodiscard]] Scalar dt() const { return dt_; }

private:
    double t_;
    double dt_;
};

/**
 * @brief Thrown when step size becomes too small
 */
class StepSizeTooSmallError : public IntegratorError {
public:
    explicit StepSizeTooSmallError(Scalar min_dt)
        : IntegratorError("Step size below minimum: " + std::to_string(min_dt)) {}
};

} // namespace icarus
```

---

## RK4 Algorithm Details

### Butcher Tableau

```
0   |
1/2 | 1/2
1/2 | 0   1/2
1   | 0   0   1
----+-------------
    | 1/6 1/3 1/3 1/6
```

### Implementation (From Janus)

```cpp
template <typename Scalar, typename Func>
JanusVector<Scalar> rk4_step(Func&& f, const JanusVector<Scalar>& x, Scalar t, Scalar dt) {
    JanusVector<Scalar> k1 = f(t, x);
    JanusVector<Scalar> x1 = (x + dt * 0.5 * k1).eval();
    JanusVector<Scalar> k2 = f(t + dt * 0.5, x1);
    JanusVector<Scalar> x2 = (x + dt * 0.5 * k2).eval();
    JanusVector<Scalar> k3 = f(t + dt * 0.5, x2);
    JanusVector<Scalar> x3 = (x + dt * k3).eval();
    JanusVector<Scalar> k4 = f(t + dt, x3);

    return (x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)).eval();
}
```

**Key Pattern:** `.eval()` forces expression evaluation, ensuring clean symbolic graphs.

---

## RK45 Algorithm Details

### Dormand-Prince Coefficients

The RK45 method uses 7 function evaluations with shared stages:

```
Stage   c_i   |  a_ij coefficients
----------------------------------------------
  1      0    |
  2     1/5   | 1/5
  3     3/10  | 3/40      9/40
  4     4/5   | 44/45    -56/15    32/9
  5     8/9   | 19372/6561  -25360/2187  64448/6561  -212/729
  6      1    | 9017/3168  -355/33  46732/5247  49/176  -5103/18656
  7      1    | 35/384     0       500/1113    125/192  -2187/6784  11/84

5th order:    | 35/384     0       500/1113    125/192  -2187/6784  11/84    0
4th order:    | 5179/57600 0       7571/16695  393/640  -92097/339200  187/2100  1/40
```

### Error Estimation

```cpp
error = ||y5 - y4|| = ||sum(b5_i * k_i) - sum(b4_i * k_i)||
```

### Step Size Adaptation

```cpp
// Optimal step: dt_new = dt * safety * (tol/error)^(1/order)
Scalar factor = safety * pow(tolerance / error, 1.0/5.0);
factor = clamp(factor, 0.1, 5.0);  // Limit growth/shrinkage
dt_new = dt * factor;
dt_new = clamp(dt_new, min_dt, max_dt);
```

---

## Verification Tests

### Test File: `tests/sim/test_integrator.cpp`

```cpp
#include <gtest/gtest.h>
#include <icarus/icarus.hpp>
#include <icarus/sim/RK4Integrator.hpp>
#include <icarus/sim/RK45Integrator.hpp>
#include <cmath>

using namespace icarus;

// ---------------------------------------------------------------------------
// Analytical Test Cases
// ---------------------------------------------------------------------------

// Exponential decay: dx/dt = -x, x(0) = 1 => x(t) = e^(-t)
template <typename Scalar>
JanusVector<Scalar> exponential_decay(Scalar t, const JanusVector<Scalar>& x) {
    return -x;
}

// Harmonic oscillator: x'' + ω²x = 0
// State: [x, v], dx/dt = v, dv/dt = -ω²x
template <typename Scalar>
JanusVector<Scalar> harmonic_oscillator(Scalar t, const JanusVector<Scalar>& x) {
    const Scalar omega_sq = Scalar{4.0};  // ω = 2
    JanusVector<Scalar> dx(2);
    dx[0] = x[1];
    dx[1] = -omega_sq * x[0];
    return dx;
}

// Free fall: y'' = -g
// State: [y, v], dy/dt = v, dv/dt = -g
template <typename Scalar>
JanusVector<Scalar> free_fall(Scalar t, const JanusVector<Scalar>& x) {
    const Scalar g = Scalar{9.81};
    JanusVector<Scalar> dx(2);
    dx[0] = x[1];      // dy/dt = v
    dx[1] = -g;        // dv/dt = -g
    return dx;
}

// ---------------------------------------------------------------------------
// RK4 Tests
// ---------------------------------------------------------------------------

TEST(RK4Integrator, ExponentialDecay) {
    RK4Integrator<double> rk4;
    JanusVector<double> x(1);
    x[0] = 1.0;

    double t = 0.0;
    double dt = 0.01;

    // Integrate to t = 1.0
    for (int i = 0; i < 100; ++i) {
        x = rk4.Step(exponential_decay<double>, x, t, dt);
        t += dt;
    }

    // x(1) = e^(-1) ≈ 0.3679
    EXPECT_NEAR(x[0], std::exp(-1.0), 1e-6);
}

TEST(RK4Integrator, HarmonicOscillator) {
    RK4Integrator<double> rk4;
    JanusVector<double> x(2);
    x[0] = 1.0;  // Initial position
    x[1] = 0.0;  // Initial velocity

    double t = 0.0;
    double dt = 0.001;
    double omega = 2.0;
    double period = 2.0 * M_PI / omega;

    // Integrate one full period
    int steps = static_cast<int>(period / dt);
    for (int i = 0; i < steps; ++i) {
        x = rk4.Step(harmonic_oscillator<double>, x, t, dt);
        t += dt;
    }

    // Should return to initial state after one period
    EXPECT_NEAR(x[0], 1.0, 1e-4);
    EXPECT_NEAR(x[1], 0.0, 1e-3);
}

TEST(RK4Integrator, FreeFall) {
    RK4Integrator<double> rk4;
    JanusVector<double> x(2);
    double y0 = 100.0;  // Initial height
    double v0 = 0.0;    // Initial velocity
    x[0] = y0;
    x[1] = v0;

    double t = 0.0;
    double dt = 0.01;
    double g = 9.81;

    // Integrate to t = 2.0
    for (int i = 0; i < 200; ++i) {
        x = rk4.Step(free_fall<double>, x, t, dt);
        t += dt;
    }

    // Analytical: y(t) = y0 + v0*t - 0.5*g*t²
    double y_analytical = y0 + v0 * t - 0.5 * g * t * t;
    double v_analytical = v0 - g * t;

    EXPECT_NEAR(x[0], y_analytical, 1e-6);
    EXPECT_NEAR(x[1], v_analytical, 1e-6);
}

TEST(RK4Integrator, OrderVerification) {
    RK4Integrator<double> rk4;
    EXPECT_EQ(rk4.Order(), 4);
    EXPECT_EQ(rk4.Name(), "RK4");
    EXPECT_FALSE(rk4.IsAdaptive());
}

// ---------------------------------------------------------------------------
// RK45 Tests
// ---------------------------------------------------------------------------

TEST(RK45Integrator, ExponentialDecay) {
    RK45Integrator<double> rk45(1e-8, 1e-8);
    JanusVector<double> x(1);
    x[0] = 1.0;

    double t = 0.0;
    double dt = 0.1;

    // Integrate to t = 1.0
    while (t < 1.0) {
        x = rk45.Step(exponential_decay<double>, x, t, dt);
        t += dt;
    }

    EXPECT_NEAR(x[0], std::exp(-1.0), 1e-7);
}

TEST(RK45Integrator, AdaptiveStep) {
    RK45Integrator<double> rk45(1e-6, 1e-6);
    JanusVector<double> x(1);
    x[0] = 1.0;

    double t = 0.0;
    double dt = 0.1;

    auto result = rk45.AdaptiveStep(exponential_decay<double>, x, t, dt);

    EXPECT_GT(result.state.size(), 0);
    EXPECT_TRUE(result.accepted);  // Should accept with tight tolerance
    EXPECT_LT(result.error_estimate, 1e-5);
}

TEST(RK45Integrator, StepSizeSuggestion) {
    RK45Integrator<double> rk45(1e-6, 1e-6);

    // If error < tolerance, suggest larger step
    double dt_new = rk45.SuggestDt(0.01, 1e-8, 1e-6);
    EXPECT_GT(dt_new, 0.01);

    // If error > tolerance, suggest smaller step
    dt_new = rk45.SuggestDt(0.01, 1e-4, 1e-6);
    EXPECT_LT(dt_new, 0.01);
}

TEST(RK45Integrator, Statistics) {
    RK45Integrator<double> rk45(1e-6, 1e-6);
    JanusVector<double> x(1);
    x[0] = 1.0;

    rk45.ResetStatistics();

    // Several steps
    for (int i = 0; i < 10; ++i) {
        rk45.AdaptiveStep(exponential_decay<double>, x, 0.0, 0.1);
    }

    auto stats = rk45.GetStatistics();
    EXPECT_GT(stats.accepted_steps, 0);
}

TEST(RK45Integrator, OrderVerification) {
    RK45Integrator<double> rk45;
    EXPECT_EQ(rk45.Order(), 5);
    EXPECT_EQ(rk45.Name(), "RK45");
    EXPECT_TRUE(rk45.IsAdaptive());
}

// ---------------------------------------------------------------------------
// Simulator Integration Tests
// ---------------------------------------------------------------------------

template <typename Scalar>
class FreeFallComponent : public Component<Scalar> {
public:
    std::string Name() const override { return "FreeFall"; }
    std::size_t StateSize() const override { return 2; }

    void Provision(Backplane<Scalar>& bp, const ComponentConfig&) override {
        bp.register_output("y", &y_);
        bp.register_output("v", &v_);
    }

    void Stage(Backplane<Scalar>&, const ComponentConfig&) override {
        y_ = Scalar{100.0};
        v_ = Scalar{0.0};
    }

    void BindState(Scalar* state, Scalar* state_dot, std::size_t size) override {
        state_y_ = state;
        state_v_ = state + 1;
        state_dot_y_ = state_dot;
        state_dot_v_ = state_dot + 1;

        // Initialize state
        *state_y_ = y_;
        *state_v_ = v_;
    }

    void Step(Scalar t, Scalar dt) override {
        // Read state
        y_ = *state_y_;
        v_ = *state_v_;

        // Compute derivatives
        *state_dot_y_ = v_;
        *state_dot_v_ = Scalar{-9.81};
    }

private:
    Scalar y_{100.0};
    Scalar v_{0.0};
    Scalar* state_y_ = nullptr;
    Scalar* state_v_ = nullptr;
    Scalar* state_dot_y_ = nullptr;
    Scalar* state_dot_v_ = nullptr;
};

TEST(SimulatorIntegrator, FreeFallWithRK4) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<FreeFallComponent<double>>());
    sim.SetIntegrator(std::make_unique<RK4Integrator<double>>());

    sim.Provision();
    sim.Stage();

    // Integrate for 2 seconds
    double dt = 0.01;
    for (int i = 0; i < 200; ++i) {
        sim.Step(dt);
    }

    // Check final position
    double t = sim.Time();
    double y = sim.GetSignal("FreeFall.y");
    double y_analytical = 100.0 - 0.5 * 9.81 * t * t;

    EXPECT_NEAR(t, 2.0, 1e-10);
    EXPECT_NEAR(y, y_analytical, 1e-4);
}

TEST(SimulatorIntegrator, FreeFallWithRK45) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<FreeFallComponent<double>>());
    sim.SetIntegrator(std::make_unique<RK45Integrator<double>>(1e-8, 1e-8));

    sim.Provision();
    sim.Stage();

    double dt = 0.1;
    for (int i = 0; i < 20; ++i) {
        sim.Step(dt);
    }

    double t = sim.Time();
    double y = sim.GetSignal("FreeFall.y");
    double y_analytical = 100.0 - 0.5 * 9.81 * t * t;

    EXPECT_NEAR(y, y_analytical, 1e-6);
}

TEST(SimulatorIntegrator, DefaultIsRK4) {
    Simulator<double> sim;

    // Integrator should be valid from construction
    EXPECT_NE(sim.GetIntegrator(), nullptr);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK4);
    EXPECT_EQ(sim.GetIntegrator()->Name(), "RK4");

    sim.AddComponent(std::make_unique<FreeFallComponent<double>>());
    sim.Provision();
    sim.Stage();

    // Should work without explicit SetIntegrator call
    EXPECT_NO_THROW(sim.Step(0.01));
}

// ---------------------------------------------------------------------------
// Symbolic Mode Tests
// ---------------------------------------------------------------------------

TEST(IntegratorSymbolic, RK4Compiles) {
    using MX = casadi::MX;

    RK4Integrator<MX> rk4;
    auto x = janus::sym_vec("x", 2);
    auto t = janus::sym("t");
    auto dt = janus::sym("dt");

    auto x_next = rk4.Step(
        [](MX t, const JanusVector<MX>& x) {
            JanusVector<MX> dx(2);
            dx[0] = x[1];
            dx[1] = -MX{4.0} * x[0];
            return dx;
        },
        x, t, dt
    );

    // Create CasADi function
    casadi::Function step_fn("step",
        {janus::to_mx(x), t, dt},
        {janus::to_mx(x_next)}
    );

    // Evaluate numerically
    std::vector<casadi::DM> args = {
        casadi::DM::vertcat({1.0, 0.0}),
        casadi::DM(0.0),
        casadi::DM(0.01)
    };
    auto res = step_fn(args);

    // Should produce valid result
    EXPECT_EQ(res[0].size1(), 2);
}

TEST(IntegratorSymbolic, SimulatorWithMX) {
    using MX = casadi::MX;

    Simulator<MX> sim;
    sim.AddComponent(std::make_unique<FreeFallComponent<MX>>());
    sim.SetIntegrator(std::make_unique<RK4Integrator<MX>>());

    sim.Provision();
    sim.Stage();

    // Should compile and run symbolic step
    EXPECT_NO_THROW(sim.Step(MX{0.01}));
}

// ---------------------------------------------------------------------------
// Modular Configuration Tests
// ---------------------------------------------------------------------------

TEST(IntegratorConfig, TypeEnumToString) {
    EXPECT_EQ(to_string(IntegratorType::Euler), "Euler");
    EXPECT_EQ(to_string(IntegratorType::RK2), "RK2");
    EXPECT_EQ(to_string(IntegratorType::RK4), "RK4");
    EXPECT_EQ(to_string(IntegratorType::RK45), "RK45");
}

TEST(IntegratorConfig, ParseTypeFromString) {
    EXPECT_EQ(parse_integrator_type("Euler"), IntegratorType::Euler);
    EXPECT_EQ(parse_integrator_type("euler"), IntegratorType::Euler);
    EXPECT_EQ(parse_integrator_type("RK4"), IntegratorType::RK4);
    EXPECT_EQ(parse_integrator_type("rk45"), IntegratorType::RK45);

    EXPECT_THROW(parse_integrator_type("invalid"), std::invalid_argument);
}

TEST(IntegratorConfig, DefaultConfig) {
    auto config = IntegratorConfig<double>::RK4Default();
    EXPECT_EQ(config.type, IntegratorType::RK4);
}

TEST(IntegratorConfig, AdaptiveConfig) {
    auto config = IntegratorConfig<double>::RK45Adaptive(1e-9, 1e-8);
    EXPECT_EQ(config.type, IntegratorType::RK45);
    EXPECT_DOUBLE_EQ(config.abs_tol, 1e-9);
    EXPECT_DOUBLE_EQ(config.rel_tol, 1e-8);
}

TEST(IntegratorFactory, CreateFromType) {
    auto euler = IntegratorFactory<double>::Create(
        IntegratorConfig<double>::ForMethod(IntegratorType::Euler)
    );
    EXPECT_EQ(euler->Name(), "Euler");
    EXPECT_EQ(euler->Order(), 1);

    auto rk4 = IntegratorFactory<double>::Create(
        IntegratorConfig<double>::ForMethod(IntegratorType::RK4)
    );
    EXPECT_EQ(rk4->Name(), "RK4");
    EXPECT_EQ(rk4->Order(), 4);
}

TEST(IntegratorFactory, CreateFromString) {
    auto integrator = IntegratorFactory<double>::Create("rk45");
    EXPECT_EQ(integrator->Name(), "RK45");
    EXPECT_TRUE(integrator->IsAdaptive());
}

TEST(IntegratorFactory, CreateDefault) {
    auto integrator = IntegratorFactory<double>::CreateDefault();
    EXPECT_EQ(integrator->Name(), "RK4");
}

TEST(IntegratorFactory, RK45ConfigApplied) {
    IntegratorConfig<double> config;
    config.type = IntegratorType::RK45;
    config.abs_tol = 1e-10;
    config.rel_tol = 1e-9;
    config.min_dt = 1e-15;
    config.max_dt = 0.5;

    auto integrator = IntegratorFactory<double>::Create(config);
    auto* rk45 = dynamic_cast<RK45Integrator<double>*>(integrator.get());

    ASSERT_NE(rk45, nullptr);
    EXPECT_DOUBLE_EQ(rk45->GetAbsTol(), 1e-10);
    EXPECT_DOUBLE_EQ(rk45->GetRelTol(), 1e-9);
    EXPECT_DOUBLE_EQ(rk45->GetMinDt(), 1e-15);
    EXPECT_DOUBLE_EQ(rk45->GetMaxDt(), 0.5);
}

TEST(SimulatorIntegrator, SetByEnum) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<FreeFallComponent<double>>());

    sim.SetIntegrator(IntegratorType::RK2);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK2);

    sim.SetIntegrator(IntegratorType::RK4);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK4);
}

TEST(SimulatorIntegrator, SetByString) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<FreeFallComponent<double>>());

    sim.SetIntegrator("euler");
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::Euler);

    sim.SetIntegrator("RK45");
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK45);
}

TEST(SimulatorIntegrator, SetByConfig) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<FreeFallComponent<double>>());

    auto config = IntegratorConfig<double>::RK45Adaptive(1e-10, 1e-10);
    sim.SetIntegrator(config);

    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK45);
    EXPECT_DOUBLE_EQ(sim.GetIntegratorConfig().abs_tol, 1e-10);
}

TEST(SimulatorIntegrator, RuntimeSwitching) {
    Simulator<double> sim;
    sim.AddComponent(std::make_unique<FreeFallComponent<double>>());
    sim.Provision();
    sim.Stage();

    // Start with Euler
    sim.SetIntegrator(IntegratorType::Euler);
    sim.Step(0.01);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::Euler);

    // Switch to RK4 mid-simulation
    sim.SetIntegrator(IntegratorType::RK4);
    sim.Step(0.01);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK4);

    // Switch to RK45
    sim.SetIntegrator(IntegratorType::RK45);
    sim.Step(0.01);
    EXPECT_EQ(sim.GetIntegratorType(), IntegratorType::RK45);
}

TEST(SimulatorIntegrator, AllMethodsProduceResults) {
    // Verify all integrators work end-to-end
    std::vector<IntegratorType> methods = {
        IntegratorType::Euler,
        IntegratorType::RK2,
        IntegratorType::RK4,
        IntegratorType::RK45
    };

    for (auto method : methods) {
        Simulator<double> sim;
        sim.AddComponent(std::make_unique<FreeFallComponent<double>>());
        sim.SetIntegrator(method);
        sim.Provision();
        sim.Stage();

        // Run 10 steps
        for (int i = 0; i < 10; ++i) {
            sim.Step(0.01);
        }

        // Should have advanced time
        EXPECT_GT(sim.Time(), 0.0) << "Failed for " << to_string(method);
    }
}
```

---

## Implementation Checklist

### Task 2.2a: Integrator Type & Configuration

- [ ] Create `include/icarus/sim/IntegratorTypes.hpp`
- [ ] Define `IntegratorType` enum (Euler, RK2, RK4, RK45)
- [ ] Implement `to_string(IntegratorType)` function
- [ ] Implement `parse_integrator_type(string)` function
- [ ] Define `IntegratorConfig<Scalar>` struct
- [ ] Add static factory methods (RK4Default, RK45Adaptive, ForMethod)

### Task 2.2b: Abstract Integrator Interface

- [ ] Update `include/icarus/sim/Integrator.hpp`
- [ ] Fix `DerivativeFunc` to use `JanusVector<Scalar>`
- [ ] Add `Name()` and `Order()` virtual methods
- [ ] Add `IsAdaptive()` virtual method
- [ ] Add `GetType()` returning `IntegratorType`
- [ ] Create `AdaptiveIntegrator<Scalar>` subclass
- [ ] Add `AdaptiveStepResult<Scalar>` struct
- [ ] Add tolerance getters/setters to AdaptiveIntegrator

### Task 2.2c: Integrator Implementations

- [ ] Create `include/icarus/sim/RK4Integrator.hpp`
- [ ] Implement `EulerIntegrator<Scalar>` wrapping `janus::euler_step()`
- [ ] Implement `RK2Integrator<Scalar>` wrapping `janus::rk2_step()`
- [ ] Implement `RK4Integrator<Scalar>` wrapping `janus::rk4_step()`
- [ ] Create `include/icarus/sim/RK45Integrator.hpp`
- [ ] Implement `RK45Integrator<Scalar>` wrapping `janus::rk45_step()`
- [ ] Implement tolerance-based step acceptance
- [ ] Implement step size suggestion formula
- [ ] Add statistics tracking (accepted/rejected steps)
- [ ] Add min/max dt bounds with setters

### Task 2.2d: Integrator Factory

- [ ] Create `include/icarus/sim/IntegratorFactory.hpp`
- [ ] Implement `IntegratorFactory<Scalar>::Create(config)`
- [ ] Implement `IntegratorFactory<Scalar>::Create(string)`
- [ ] Implement `IntegratorFactory<Scalar>::CreateDefault()`
- [ ] Add proper error handling for unknown types

### Task 2.2e: Simulator Integration

- [ ] Add `SetIntegrator(IntegratorConfig<Scalar>)` overload
- [ ] Add `SetIntegrator(IntegratorType)` overload
- [ ] Add `SetIntegrator(string)` overload
- [ ] Add `SetIntegrator(unique_ptr<Integrator>)` for advanced use
- [ ] Add `GetIntegrator()` accessor
- [ ] Add `GetIntegratorConfig()` accessor
- [ ] Add `GetIntegratorType()` accessor
- [ ] Modify `Step()` to use integrator (no null check, always valid)
- [ ] Add `AdaptiveStep()` method for RK45
- [ ] Create derivative function lambda wrapping `ComputeDerivatives()`
- [ ] Initialize `integrator_` to RK4 in member declaration
- [ ] Store `IntegratorConfig<Scalar>` member with RK4 default

### Task 2.2f: Error Types

- [ ] Add `IntegratorError` base exception
- [ ] Add `IntegrationFailedError` with time/dt info
- [ ] Add `StepSizeTooSmallError` for adaptive stepping
- [ ] Add `UnknownIntegratorTypeError` for factory

### Task 2.2g: Tests

- [ ] Create `tests/sim/test_integrator.cpp`
- [ ] Add exponential decay tests for all methods (Euler, RK2, RK4, RK45)
- [ ] Add harmonic oscillator tests
- [ ] Add free fall analytical verification
- [ ] Add order verification tests for each integrator
- [ ] Add adaptive step statistics tests
- [ ] Add integrator factory tests
- [ ] Add integrator switching tests
- [ ] Add string parsing tests
- [ ] Add Simulator integration tests with config
- [ ] Add symbolic mode compilation tests
- [ ] Update `tests/CMakeLists.txt`

### Task 2.2h: Integration & Documentation

- [ ] Update `include/icarus/icarus.hpp` with new headers
- [ ] Verify `./scripts/build.sh` succeeds
- [ ] Verify `./scripts/test.sh` all pass
- [ ] Update main implementation plan checkboxes
- [ ] Add integrator selection to example code

---

## Design Decisions

### 1. Modular, User-Configurable Design

**Decision:** Integrators are selectable at runtime via enum, string, or config struct.

**Rationale:**
- Users may want different methods for different scenarios (fast vs. accurate)
- Configuration files can specify integrator without recompilation
- Testing/comparison between methods is trivial
- Factory pattern decouples creation from usage

**API:**
```cpp
sim.SetIntegrator(IntegratorType::RK4);     // Enum
sim.SetIntegrator("rk45");                   // String (config file)
sim.SetIntegrator(config);                   // Full config struct
```

### 2. Wrap Janus Step Functions

**Decision:** Integrators wrap `janus::rk4_step()` and `janus::rk45_step()` directly.

**Rationale:**
- Janus already provides tested, symbolic-compatible implementations
- No need to reimplement Butcher tableau logic
- Guaranteed dual-mode (numeric/symbolic) support

### 3. Derivative Function Signature

**Decision:** `f(Scalar t, const JanusVector<Scalar>& x) -> JanusVector<Scalar>`

**Rationale:**
- Matches Janus API exactly
- Captures time and state as inputs
- Returns derivative vector

### 4. Separate Adaptive Interface

**Decision:** `AdaptiveIntegrator` extends `Integrator` with `AdaptiveStep()`.

**Rationale:**
- Fixed-step integrators don't need tolerance parameters
- Adaptive stepping returns different result type (includes error)
- Clean separation of concerns

### 5. Statistics for Adaptive Stepping

**Decision:** Track accepted/rejected step counts in RK45Integrator.

**Rationale:**
- Useful for tuning tolerances
- Helps diagnose stiff problems
- Cheap to maintain

### 6. Always-Valid Integrator (Default RK4)

**Decision:** `integrator_` is initialized to `RK4Integrator` in the member declaration. No null checks needed.

**Rationale:**
- Simpler code — no conditional in `Step()`
- Always-valid invariant — `integrator_` is never null
- Sensible default for most simulations
- User can override with `SetIntegrator()` at any time

**Implementation:**
```cpp
private:
    std::unique_ptr<Integrator<Scalar>> integrator_ =
        std::make_unique<RK4Integrator<Scalar>>();  // Always valid
```

---

## Janus Compatibility Checklist

- [ ] All integrator code templated on `Scalar`
- [ ] Use `JanusVector<Scalar>` (not `std::vector<double>`)
- [ ] Wrap Janus step functions (not reimplementing)
- [ ] No `std::` math in traced code paths
- [ ] `.eval()` used where needed for expression materialization
- [ ] Verify `Simulator<casadi::MX>` compiles and runs
- [ ] Verify CasADi function generation works

---

## Exit Criteria

- [ ] `IntegratorType` enum with all Janus methods (Euler, RK2, RK4, RK45)
- [ ] `IntegratorConfig<Scalar>` struct with all parameters
- [ ] `IntegratorFactory<Scalar>` creates integrators from config/string/type
- [ ] Abstract `Integrator<Scalar>` interface complete
- [ ] All four integrators implemented (Euler, RK2, RK4, RK45)
- [ ] `RK45Integrator` with adaptive stepping and statistics
- [ ] `Simulator::SetIntegrator()` accepts config, type, string, or unique_ptr
- [ ] `Simulator::GetIntegratorType()` returns current method
- [ ] Runtime switching between integrators works
- [ ] Analytical test cases pass for all methods
- [ ] Symbolic mode tests pass for all methods
- [ ] All Phase 2.2 tests pass for both `double` and `casadi::MX`
- [ ] Existing Phase 2.1 tests continue to pass

---

## Dependencies

| Dependency | Purpose | Status |
|:-----------|:--------|:-------|
| Phase 2.1 | State management, ComputeDerivatives | ✅ Complete |
| Janus `rk4_step` | RK4 implementation | ✅ Available |
| Janus `rk45_step` | RK45 implementation | ✅ Available |
| GoogleTest | Testing framework | ✅ Available |

---

## References

| Topic | Document |
|:------|:---------|
| Janus integrators | [`IntegratorStep.hpp`](file:///home/tanged/sources/references/janus/include/janus/math/IntegratorStep.hpp) |
| Integration guide | [`integration.md`](file:///home/tanged/sources/references/janus/docs/user_guides/integration.md) |
| State ownership | [09_memory_state_ownership.md](../../architecture/09_memory_state_ownership.md) |
| Janus integration | [07_janus_integration.md](../../architecture/07_janus_integration.md) |
| Symbolic constraints | [21_symbolic_constraints.md](../../architecture/21_symbolic_constraints.md) |
