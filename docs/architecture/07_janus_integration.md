# Janus Integration (The "Template-First" Paradigm)

**Related:** [01_core_philosophy.md](01_core_philosophy.md) | [08_vulcan_integration.md](08_vulcan_integration.md) | [21_symbolic_constraints.md](21_symbolic_constraints.md)

---

This architecture is specifically designed to support the **Janus** "Template-First" paradigm, enabling a dual-backend type system.

---

## 1. The "Scalar" Concept

Components are **not** written for `double` or `casadi::MX`. They are written for a generic `Scalar`.

```cpp
template <typename Scalar>
void Aerodynamics::Step(Scalar& t, Scalar dt) {
    // GOOD: Uses Janus dispatch
    Scalar q = 0.5 * rho * janus::pow(v, 2);

    // BAD: Uses std::Math or raw types
    // double q = 0.5 * rho * std::pow(v, 2);
}
```

---

## 2. Numeric Mode (`Simulator<double>`)

* **Backend:** `Janus::Scalar` resolves to `double`.
* **Execution:** Fast, compiled C++ assembly.
* **Use Case:** Real-time HIL, Monte Carlo, Visualizer connection.

---

## 3. Symbolic Mode (`Simulator<janus::SymbolicScalar>`)

* **Backend:** `Janus::Scalar` resolves to `casadi::MX`.
* **Execution:** Graph recording.
* **Use Case:** Trajectory Optimization, Sensitivity Analysis.

---

## 4. Optimization-Only Workflow ("Bridging the Gap")

We can support a workflow that exists purely to bridge Numeric Simulation and Symbolic Optimization:

1. **Provision** the `Simulator<janus::SymbolicScalar>`.
2. **Stage** to set the topology.
3. **Do NOT Step** in a loop.
4. Instead, export the symbolic function:
   `casadi::Function F_dynamics = sim.GenerateGraph("dynamics");`
5. This `F_dynamics` is then handed to an external offline trajectory optimizer (using `janus::Opti` or Direct Collocation).

This ensures that the **Optimizer** and the **Simulator** use the *exact same physics code*, guaranteed by the shared `template <typename Scalar>` component implementation. No more maintaining a separate "low fidelity" model for optimization.

---

## 5. Mode Comparison

| Aspect | Numeric Mode | Symbolic Mode |
|:-------|:-------------|:--------------|
| **Scalar Type** | `double` | `casadi::MX` |
| **Execution** | Direct computation | Graph recording |
| **Performance** | ~ns per operation | ~μs per operation (tracing) |
| **Use Case** | Simulation, HITL | Optimization, Sensitivity |
| **Output** | State values | Symbolic expressions |

---

## 6. Dual-Mode Example

```cpp
template <typename Scalar>
class GravityComponent : public Component<Scalar> {
public:
    void Step(Scalar t, Scalar dt) override {
        Scalar r = janus::sqrt(
            janus::pow(*input_x_, 2) +
            janus::pow(*input_y_, 2) +
            janus::pow(*input_z_, 2)
        );

        Scalar g_mag = mu_ / janus::pow(r, 2);

        // Works identically for double or MX
        *output_gx_ = -g_mag * (*input_x_ / r);
        *output_gy_ = -g_mag * (*input_y_ / r);
        *output_gz_ = -g_mag * (*input_z_ / r);
    }
};

// Numeric instantiation
Simulator<double> sim_numeric(config);
sim_numeric.Step(0.001);  // Computes actual values

// Symbolic instantiation
Simulator<casadi::MX> sim_symbolic(config);
auto F = sim_symbolic.GenerateGraph("gravity");  // Extracts symbolic function
```

---

## 7. Key Constraint

> [!IMPORTANT]
> **All physics code must be templated on Scalar and use `janus::` math functions.**
>
> See [21_symbolic_constraints.md](21_symbolic_constraints.md) for the complete list of rules.
