# External Data & Table Loading

**Related:** [21_symbolic_constraints.md](21_symbolic_constraints.md) | [13_configuration.md](13_configuration.md) | [02_component_protocol.md](02_component_protocol.md)

---

Many aerospace simulations rely on large lookup tables (aerodynamic coefficients, engine maps, atmospheric profiles). This document specifies how Icarus handles external data loading while maintaining symbolic compatibility.

---

## 1. Table Data Requirements

External tables must provide:

| Element | Description | Example |
| :--- | :--- | :--- |
| **Breakpoints** | Independent variable grid points per dimension | `mach = [0.0, 0.5, 1.0, 1.5, 2.0]` |
| **Values** | Dependent variable data (flattened, Fortran order) | `cd = [0.02, 0.025, 0.05, ...]` |
| **Metadata** | Units, variable names, interpolation hints | `units: "1", method: "bspline"` |

---

## 2. Supported Formats

| Format | Extension | Pros | Cons | Use Case |
|--------|-----------|------|------|----------|
| **HDF5** | `.h5` | Fast, structured, compression | Binary, needs library | Default for large tables |
| **CSV** | `.csv` | Human-readable, universal | Slow for large data | Small tables, debugging |
| **Fortran Binary** | `.bin` | Legacy compatibility | Platform-dependent | Heritage code integration |

> [!TIP]
> **Recommended: HDF5** for production tables. It supports arbitrary dimensions, compression, and self-describing metadata.

---

## 3. Table File Structure

### 3.1 HDF5 Layout

```
/aero_tables.h5
├── /metadata
│   ├── version: "1.0"
│   ├── created: "2024-12-22"
│   └── vehicle: "X-15"
├── /breakpoints
│   ├── mach     [5]     # 1D array
│   ├── alpha    [12]    # 1D array
│   ├── beta     [7]     # 1D array
│   └── alt      [20]    # 1D array
├── /tables
│   ├── cd       [5,12,7,20]    # 4D array (Fortran order)
│   ├── cl       [5,12,7,20]
│   └── cm       [5,12,7,20]
└── /table_metadata
    ├── cd
    │   ├── units: "1"
    │   ├── interpolation: "bspline"
    │   └── extrapolation: "clamp"
    └── ...
```

### 3.2 CSV Layout (Small Tables Only)

```csv
# mach,alpha,cd
# units: 1,deg,1
# interpolation: linear
0.0,0.0,0.020
0.0,5.0,0.025
0.0,10.0,0.035
0.5,0.0,0.022
...
```

---

## 4. Architecture Layering

Icarus does **not** implement its own interpolator. Instead, it uses the existing stack:

```
┌─────────────────────────────────────────────────────────────┐
│  Icarus: icarus::TableLoader                                │
│  - File I/O (HDF5, CSV, binary)                             │
│  - Returns raw breakpoints + values                         │
└───────────────────────────┬─────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  Vulcan: vulcan::Table1D, vulcan::TableND                   │
│  - Thin physics-API wrappers                                │
│  - Fluent operator() syntax                                 │
└───────────────────────────┬─────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  Janus: janus::Interpolator                                 │
│  - Core math (linear, bspline, hermite)                     │
│  - CasADi symbolic backend                                  │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. Loading API

### 5.1 Component-Level Loading

```cpp
#include <vulcan/core/TableInterpolator.hpp>
#include <icarus/io/TableLoader.hpp>

template <typename Scalar>
void Aerodynamics<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // Icarus loads file → raw data
    auto data = icarus::TableLoader::load(cfg.get<std::string>("aero_table"));

    // Vulcan provides interpolator wrapper
    cd_table_ = vulcan::TableND(
        data.breakpoints({"mach", "alpha", "beta"}),
        data.values("cd"),
        data.interpolation_method("cd")
    );
}

template <typename Scalar>
void Aerodynamics<Scalar>::Step(Scalar t, Scalar dt) {
    // Vulcan's operator() is templated on Scalar (works for double and MX)
    janus::JanusVector<Scalar> query(3);
    query << mach, alpha, beta;
    Scalar cd = cd_table_(query);
}
```

### 5.2 Icarus Table Loader Interface

Icarus provides **only** the file I/O layer:

```cpp
namespace icarus {

class TableLoader {
public:
    static TableData load(const std::string& path);       // Auto-detect format
    static TableData load_hdf5(const std::string& path);
    static TableData load_csv(const std::string& path);
};

class TableData {
public:
    // Breakpoint access
    std::vector<janus::NumericVector> breakpoints(
        const std::vector<std::string>& dims) const;

    // Value access (flattened, Fortran order)
    janus::NumericVector values(const std::string& name) const;

    // Metadata
    janus::InterpolationMethod interpolation_method(const std::string& name) const;
    std::string units(const std::string& name) const;

    // Introspection
    std::vector<std::string> table_names() const;
    std::vector<std::string> breakpoint_names() const;
    size_t dimension(const std::string& name) const;
};

}  // namespace icarus
```

### 5.3 Vulcan Table Wrappers

See [vulcan/include/vulcan/core/TableInterpolator.hpp](file:///home/tanged/sources/vulcan/include/vulcan/core/TableInterpolator.hpp):

| Class | Use Case |
| :--- | :--- |
| `vulcan::Table1D` | Single-variable lookups (altitude → temperature) |
| `vulcan::TableND` | Multi-dimensional lookups (Mach × α × β → Cd) |

Both wrap `janus::Interpolator` and support:
- Templated `operator()` for numeric and symbolic evaluation
- Automatic clamping at grid boundaries
- Batch queries
---

## 6. High-Dimensional Tables

Aerospace applications often require 6-8 dimensional tables:

| Dimension | Example Variables |
| :--- | :--- |
| 1-2D | `alpha`, `mach` |
| 3-4D | + `beta`, `altitude` |
| 5-6D | + `control_surface`, `reynolds` |
| 7-8D | + `temperature`, `configuration` |

### 5.1 Memory Considerations

| Dimensions | Grid Size | Points | Memory |
|------------|-----------|--------|--------|
| 4D | 10×10×10×10 | 10,000 | ~80 KB |
| 6D | 10×10×10×10×10×10 | 1,000,000 | ~8 MB |
| 8D | 10×10×10×10×10×10×10×10 | 100,000,000 | ~800 MB |

> [!CAUTION]
> 8D tables with fine grids can exceed available memory. Consider:
> - Coarser grids for less-sensitive dimensions
> - Sparse grid techniques
> - Surrogate models (neural networks, polynomial chaos)

### 6.2 Scattered Data (Irregular Grids)

For non-rectangular domains (e.g., flight envelope boundaries, wind tunnel test points), use Vulcan's **scattered interpolation** instead of gridded lookups:

```cpp
#include <vulcan/core/TableInterpolator.hpp>

// 2D scattered data: (Mach, alpha) pairs from wind tunnel tests
janus::NumericMatrix points(n_samples, 2);  // Each row: [mach, alpha]
janus::NumericVector cd_values(n_samples);

// ... load test points and values ...

// ScatteredTableND handles non-uniform point distributions
vulcan::ScatteredTableND table(points, cd_values, num_rbf_centers);

std::cout << "Reconstruction error: " << table.reconstruction_error() << "\n";

// Query at any point within the convex hull
janus::JanusVector<Scalar> query(2);
query << mach, alpha;
Scalar cd = table(query);
```

### 6.3 Scattered vs Gridded

| Type | Use Case | Vulcan Class |
| :--- | :--- | :--- |
| **Gridded** | Regular rectangular grids | `vulcan::Table1D`, `vulcan::TableND` |
| **Scattered** | Irregular/sparse samples, test data | `vulcan::ScatteredTable1D`, `vulcan::ScatteredTableND` |

> [!TIP]
> See [vulcan/examples/interpolation/interpolation_demo.cpp](file:///home/tanged/sources/vulcan/examples/interpolation/interpolation_demo.cpp) for complete examples of both approaches.

---

## 6. Caching & Performance

### 6.1 Interpolator Caching

The `janus::Interpolator` constructor builds a CasADi function at Provision time. This is cached for the simulation lifetime:

```cpp
// Provision: O(n) construction, builds CasADi graph
cd_interp_ = janus::Interpolator(breakpoints, values, method);

// Step: O(1) lookup, uses cached function
Scalar cd = cd_interp_(query)(0);  // Fast!
```

### 6.2 Table File Caching

Large HDF5 files are memory-mapped by default:

```yaml
# config snippet
table_loader:
  cache_mode: MMAP      # Options: MMAP, PRELOAD, LAZY
  max_cache_mb: 500     # Evict LRU tables if exceeded
```

---

## 7. Validation

Tables are validated at load time:

| Check | Error If |
| :--- | :--- |
| Breakpoint monotonicity | Grid not sorted ascending |
| Value count | `prod(dims)` ≠ value array length |
| NaN/Inf detection | Values contain non-finite numbers |
| Dimension mismatch | Requested dims don't match table |

```cpp
try {
    auto table = icarus::TableLoader::load(path);
} catch (const icarus::TableError& e) {
    ICARUS_ERROR("Table load failed: {}", e.what());
}
```

---

## 8. See Also

- [21_symbolic_constraints.md](21_symbolic_constraints.md) - Symbolic-compatible interpolation
- [janus/docs/user_guides/interpolation.md](file:///home/tanged/sources/janus/docs/user_guides/interpolation.md) - Janus `interpn` API
- [13_configuration.md](13_configuration.md) - Table path configuration
