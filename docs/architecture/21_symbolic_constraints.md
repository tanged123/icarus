# Symbolic Mode Constraints (Metis Compatibility)

**Related:** [07_metis_integration.md](07_metis_integration.md) | [08_vulcan_integration.md](08_vulcan_integration.md)

> **Reference:** See `metis/docs/metis_usage_guide.md` for complete details.

---

## 1. MANDATORY Rules for Icarus Components

| Rule | Correct | Wrong |
|------|---------|-------|
| **Template on Scalar** | `template <typename Scalar>` | Hardcoded `double` |
| **Use metis:: math** | `metis::sin(x)`, `metis::pow(x,2)` | `std::sin(x)`, `std::pow(x,2)` |
| **Branching** | `metis::where(cond, a, b)` | `if (cond) { a } else { b }` |
| **Loops** | `for (int i=0; i<N; ++i)` (fixed N) | `while (error > tol)` (dynamic) |
| **Types** | `metis::Vec3<Scalar>` | `Eigen::Vector3d` |

---

## 2. Multi-way Branching

```cpp
// Use metis::select for multiple conditions
Scalar cd = metis::select(
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

// CORRECT: metis::where
thrust = metis::where(altitude > 10000, max_thrust, Scalar(0));
```

### Math Functions

```cpp
// WRONG: std:: functions
double q = 0.5 * rho * std::pow(v, 2);
double angle = std::atan2(y, x);

// CORRECT: metis:: functions
Scalar q = 0.5 * rho * metis::pow(v, 2);
Scalar angle = metis::atan2(y, x);
```

### Vector/Matrix Types

```cpp
// WRONG: Eigen with hardcoded double
Eigen::Vector3d force;

// CORRECT: Metis vector templated on Scalar
metis::Vec3<Scalar> force;
```

### Loops

```cpp
// WRONG: Dynamic loop bounds
while (residual > tolerance) {
    // iterate
}

// CORRECT: Fixed loop bounds
for (int i = 0; i < MAX_ITERATIONS; ++i) {
    // iterate with early-exit via metis::where
}
```

---

## 5. Table Lookups

Lookup tables require special handling for symbolic mode:

```cpp
// WRONG: Standard interpolation (not differentiable)
double cl = table.lookup(alpha);

// CORRECT: Use metis::interpn (CasADi-compatible)
Scalar cl = metis::interpn<Scalar>(points, values, query)(0);
```

> **Reference:** See [metis/docs/user_guides/interpolation.md](file:///home/tanged/sources/metis/docs/user_guides/interpolation.md) for full API details.

### 5.1 Interpolation Methods

| Method | Continuity | Symbolic | Use Case |
| :--- | :--- | :--- | :--- |
| `Linear` | C0 | ✅ | General purpose, gradients at knots |
| `BSpline` | C2 | ✅ | Optimization (smoothest) |
| `Hermite` | C1 | ❌ | Smooth trajectories (numeric only) |
| `Nearest` | None | ❌ | Fast lookup (numeric only) |

### 5.2 Multi-Dimensional Tables

```cpp
// 1D: Cl vs alpha
metis::NumericVector alpha_pts, cl_vals;
metis::MetisMatrix<Scalar> query(1, 1);
query(0, 0) = alpha;
Scalar cl = metis::interpn<Scalar>({alpha_pts}, cl_vals, query)(0);

// 2D: Cd vs (Mach, alpha)
metis::MetisMatrix<Scalar> query2d(1, 2);
query2d << mach, alpha;
Scalar cd = metis::interpn<Scalar>({mach_pts, alpha_pts}, cd_grid, query2d,
    metis::InterpolationMethod::BSpline)(0);
```

### 5.3 Extrapolation Modes

```cpp
// Default: Clamp to boundary (safe, but zero gradient outside bounds)
auto result = metis::interpn<Scalar>(points, values, query);

// Linear extrapolation with output bounds (maintains gradient for optimization)
metis::Interpolator interp(x_pts, y_vals,
    metis::InterpolationMethod::BSpline,
    metis::ExtrapolationConfig::linear(lower_bound, upper_bound));
```

| Mode | Behavior | Use Case |
| :--- | :--- | :--- |
| `ExtrapolationConfig::clamp()` | Clamp queries to grid (default, safe) | Most applications |
| `ExtrapolationConfig::linear()` | Linear extrapolation, unbounded | Optimization (non-zero gradient) |
| `ExtrapolationConfig::linear(lo, hi)` | Linear with output bounds | Optimization with safety |

### 5.4 Loading Tables from Config

Tables can be loaded from external sources at **Provision** time and still work symbolically:

```cpp
void Aero::Provision(Backplane& bp, const ComponentConfig& cfg) {
    // Load table data (numeric values only)
    auto [mach_pts, alpha_pts, cd_grid] = load_table(cfg.get("cd_table_path"));

    // Store for later use (Interpolator caches CasADi function)
    cd_interp_ = metis::Interpolator({mach_pts, alpha_pts}, cd_grid,
        metis::InterpolationMethod::BSpline);
}

void Aero::Step(Scalar t, Scalar dt) {
    // Works for both double and MX
    metis::MetisMatrix<Scalar> query(1, 2);
    query << mach, alpha;
    Scalar cd = cd_interp_(query)(0);
}
```

Implementation of file loaders will have to be implemented on Icarus's side. See [23_external_data.md](23_external_data.md) for table file formats, loading API, and high-dimensional table considerations.

> [!NOTE]
> The interpolant's **structure** (breakpoints, grid size) is fixed at Provision. Only the **query point** can be symbolic.

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
    aero.step(metis::sym("t"), metis::sym("dt"));

    // Verify output is a valid MX expression
    auto lift = bp.get("Aero.Lift");
    EXPECT_TRUE(lift.is_valid_input());
}
```

---

## 7. Quick Reference: metis:: Math Functions

| Operation | metis:: Function |
|-----------|------------------|
| Power | `metis::pow(x, n)` |
| Square root | `metis::sqrt(x)` |
| Trig | `metis::sin(x)`, `metis::cos(x)`, `metis::tan(x)` |
| Inverse trig | `metis::asin(x)`, `metis::acos(x)`, `metis::atan(x)` |
| atan2 | `metis::atan2(y, x)` |
| Exponential | `metis::exp(x)`, `metis::log(x)` |
| Absolute | `metis::abs(x)` |
| Min/Max | `metis::fmin(a, b)`, `metis::fmax(a, b)` |
| Conditional | `metis::where(cond, if_true, if_false)` |
| Multi-select | `metis::select({conds}, {values}, default)` |

---

## 8. Debugging Symbolic Issues

When symbolic tracing fails:

1. **Check for `std::` functions** - grep for `std::sin`, `std::pow`, etc.
2. **Check for `if/else` on Scalar** - replace with `metis::where`
3. **Check for dynamic loops** - convert to fixed iteration with masking
4. **Check types** - ensure all math uses `Scalar`, not `double`
5. **Run symbolic test** - instantiate with `casadi::MX` to catch issues early

---

## 9. See Also

- [05_execution_model.md#6-symbolic-mode--multi-rate-interaction](05_execution_model.md) - Multi-rate scheduling and symbolic mode
- [07_metis_integration.md](07_metis_integration.md) - Full Metis integration guide
- [08_vulcan_integration.md](08_vulcan_integration.md) - Vulcan physics models
