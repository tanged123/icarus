# Symbolic Mode Constraints (Janus Compatibility)

**Related:** [07_janus_integration.md](07_janus_integration.md) | [08_vulcan_integration.md](08_vulcan_integration.md)

> **Reference:** See `janus/docs/janus_usage_guide.md` for complete details.

---

## 1. MANDATORY Rules for Icarus Components

| Rule | Correct | Wrong |
|------|---------|-------|
| **Template on Scalar** | `template <typename Scalar>` | Hardcoded `double` |
| **Use janus:: math** | `janus::sin(x)`, `janus::pow(x,2)` | `std::sin(x)`, `std::pow(x,2)` |
| **Branching** | `janus::where(cond, a, b)` | `if (cond) { a } else { b }` |
| **Loops** | `for (int i=0; i<N; ++i)` (fixed N) | `while (error > tol)` (dynamic) |
| **Types** | `janus::Vec3<Scalar>` | `Eigen::Vector3d` |

---

## 2. Multi-way Branching

```cpp
// Use janus::select for multiple conditions
Scalar cd = janus::select(
    {mach < 0.3, mach < 0.8, mach < 1.2},
    {Scalar(0.02), Scalar(0.025), Scalar(0.05)},
    Scalar(0.03));  // default
```

---

## 3. What Breaks Symbolic Mode

- `if/else` on `Scalar` (MX can't evaluate to bool)
- `while` loops with dynamic bounds
- `std::` math functions (bypass CasADi tracing)
- Dynamic memory allocation inside Step()
- Non-templated functions

---

## 4. Correct Patterns

### Conditional Logic

```cpp
// WRONG: Runtime branch on Scalar
if (altitude > 10000) {
    thrust = max_thrust;
} else {
    thrust = 0;
}

// CORRECT: janus::where
thrust = janus::where(altitude > 10000, max_thrust, Scalar(0));
```

### Math Functions

```cpp
// WRONG: std:: functions
double q = 0.5 * rho * std::pow(v, 2);
double angle = std::atan2(y, x);

// CORRECT: janus:: functions
Scalar q = 0.5 * rho * janus::pow(v, 2);
Scalar angle = janus::atan2(y, x);
```

### Vector/Matrix Types

```cpp
// WRONG: Eigen with hardcoded double
Eigen::Vector3d force;

// CORRECT: Janus vector templated on Scalar
janus::Vec3<Scalar> force;
```

### Loops

```cpp
// WRONG: Dynamic loop bounds
while (residual > tolerance) {
    // iterate
}

// CORRECT: Fixed loop bounds
for (int i = 0; i < MAX_ITERATIONS; ++i) {
    // iterate with early-exit via janus::where
}
```

---

## 5. Table Lookups

Lookup tables require special handling for symbolic mode:

```cpp
// WRONG: Standard interpolation (not differentiable)
double cl = table.lookup(alpha);

// CORRECT: Use janus::Interpolant (CasADi-compatible)
Scalar cl = janus::Interpolant1D(alpha_breakpoints, cl_values)(alpha);
```

---

## 6. Symbolic Mode Testing

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

## 7. Quick Reference: janus:: Math Functions

| Operation | janus:: Function |
|-----------|------------------|
| Power | `janus::pow(x, n)` |
| Square root | `janus::sqrt(x)` |
| Trig | `janus::sin(x)`, `janus::cos(x)`, `janus::tan(x)` |
| Inverse trig | `janus::asin(x)`, `janus::acos(x)`, `janus::atan(x)` |
| atan2 | `janus::atan2(y, x)` |
| Exponential | `janus::exp(x)`, `janus::log(x)` |
| Absolute | `janus::abs(x)` |
| Min/Max | `janus::fmin(a, b)`, `janus::fmax(a, b)` |
| Conditional | `janus::where(cond, if_true, if_false)` |
| Multi-select | `janus::select({conds}, {values}, default)` |

---

## 8. Debugging Symbolic Issues

When symbolic tracing fails:

1. **Check for `std::` functions** - grep for `std::sin`, `std::pow`, etc.
2. **Check for `if/else` on Scalar** - replace with `janus::where`
3. **Check for dynamic loops** - convert to fixed iteration with masking
4. **Check types** - ensure all math uses `Scalar`, not `double`
5. **Run symbolic test** - instantiate with `casadi::MX` to catch issues early
