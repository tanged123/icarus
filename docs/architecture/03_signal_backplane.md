# The Signal Backplane

**Related:** [01_core_philosophy.md](01_core_philosophy.md) | [02_component_protocol.md](02_component_protocol.md) | [09_memory_state_ownership.md](09_memory_state_ownership.md)

---

In a hierarchical system, data flows via pointers: `parent->child->doThing()`.
In IDOA, data flows via the **Signal Backplane**.

## 1. Concept

The Backplane is a globally addressable registry of all "Signals" in the simulation. It aligns with the findings in `design/signal_system_analysis.md`, implementing a **Hybrid Pointer + Metadata** model.

* **Runtime:** Fast pointer-based access (using `SignalID` to resolve pointers during `Stage`).
* **Introspection:** Rich metadata for UI, Telemetry, and Validation.

---

## 2. Signal Definition

A signal is more than just a value. It defines:

| Property | Description | Examples |
| :--- | :--- | :--- |
| **Type** | The data type. Supported: `double` (default), `int32` (modes), `int64` (counters). | `nav.altitude` (double), `gnc.mode` (int32) |
| **Lifecycle** | **Static** (Constant after Stage) or **Dynamic** (Updated every Step). | `mass.dry` (static), `nav.vel` (dynamic) |
| **Units** | Physical unit for validation and display. | "m/s", "kg", "rad" |

---

## 3. Implementation Details

### 3.1 The Registry (`map<string, SignalMetadata>`)

Holds the "Truth" about what exists. Used during **Provision** and **Stage** for resolution.

```cpp
struct SignalMetadata {
    uint32_t id;          // Unique Hash
    string name;          // "Vehicle.Aero.Cl"
    DataType type;        // DOUBLE, INT32...
    Lifecycle lifecycle;  // STATIC, DYNAMIC
    void* data_ptr;       // Pointer to storage
};
```

### 3.2 The Storage (`vector<double>`, `vector<int>`)

Contiguous memory blocks for each type.
* **Optimization:** Static signals are stored separately.
* **Telemetry:** Only the "Dynamic Double" vector is streamed over UDP (60Hz). Static signals are sent once via Schema.

### 3.3 Component Access ("Bind Once, Read Forever")

Components do **not** look up signals by string every Step. They bind pointers during Stage.

```cpp
// Stage()
this->ptr_density_ = backplane.resolve<double>("Env.Atm.Density");

// Step()
double rho = *this->ptr_density_; // Zero overhead
```

### 3.4 Convenience: Grouped Binding (Optional Helper)

For ergonomics, a **convenience layer** can bind multiple signals matching a pattern into a struct. This is **not** a core Backplane feature—it's syntactic sugar.

```cpp
// Define a struct matching the signal namespace
struct NavState {
    double* pos_x;
    double* pos_y;
    double* pos_z;
    double* vel_x;
    // ...
};

// Stage() - bind all Nav.* signals at once
NavState nav = backplane.bind_group<NavState>("Vehicle.Nav");

// Step() - access like normal
double x = *nav.pos_x;
```

Internally, `bind_group` just calls `resolve()` for each member. The Backplane remains flat; this is purely a compile-time convenience.

---

## 4. Type-Safe Signal Access

The raw `void* data_ptr` in `SignalMetadata` is an implementation detail. User-facing APIs enforce type safety:

### 4.1 Typed Signal Handles

```cpp
// Type-safe handle returned by resolve()
template <typename T>
class SignalHandle {
    T* ptr_;
    SignalMetadata meta_;  // For debugging/introspection
public:
    T& operator*() { return *ptr_; }
    const T& operator*() const { return *ptr_; }
    T* operator->() { return ptr_; }

    // Introspection
    const std::string& name() const { return meta_.name; }
    Units units() const { return meta_.units; }
};

// Backplane resolve with compile-time type checking
template <typename Scalar>
class Backplane {
public:
    template <typename T>
    SignalHandle<T> resolve(const std::string& name) {
        auto& meta = registry_.at(name);

        // Runtime type check (throws if mismatch)
        if (meta.type != TypeTraits<T>::type_id) {
            throw TypeMismatchError(name, meta.type, TypeTraits<T>::type_id);
        }

        return SignalHandle<T>{static_cast<T*>(meta.data_ptr), meta};
    }
};
```

### 4.2 Registration with Type Deduction

```cpp
// Component registration - type is deduced
template <typename Scalar>
void JetEngine<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // Type is captured at registration
    bp.register_output("Propulsion.Thrust", &thrust_value_, Units::Newtons);
    bp.register_output("Propulsion.FuelFlow", &fuel_flow_, Units::KgPerSec);

    // Parameters (static signals loaded from config)
    max_thrust_ = cfg.get<double>("max_thrust", 50000.0);
    bp.register_static("Propulsion.MaxThrust", &max_thrust_, Units::Newtons);
}
```

### 4.3 Type Validation Matrix

| Registration Type | Resolve Type | Result |
| :--- | :--- | :--- |
| `double` | `double` | ✅ Success |
| `double` | `int32` | ❌ `TypeMismatchError` |
| `int32` | `double` | ❌ `TypeMismatchError` |
| `Vec3<Scalar>` | `Vec3<Scalar>` | ✅ Success |
| `Vec3<double>` | `Vec3<MX>` | ❌ `TypeMismatchError` (Scalar mismatch) |

> [!IMPORTANT]
> **Type checking happens at Stage, not Step.** All `resolve()` calls occur during `Stage()`. If types mismatch, the simulation fails fast with a clear error—before entering the hot loop.

---

## 5. Signal Namespacing & Collision Prevention

Signal names follow a hierarchical namespace convention:

```
<Entity>.<Component>.<Signal>
```

Examples:
- `Falcon9.Stage1.Engine.Thrust`
- `Environment.Atmosphere.Density`
- `X15.Aero.Alpha`

### 5.1 Collision Detection

```cpp
void Backplane::register_output(const std::string& name, ...) {
    if (registry_.contains(name)) {
        throw DuplicateSignalError(
            name,
            registry_.at(name).owner_component,
            current_component_
        );
    }
    // ... register
}
```

### 5.2 Wildcard Queries (Introspection Only)

```cpp
// For tooling/debugging—NOT for hot-path access
std::vector<SignalMetadata> Backplane::query(const std::string& pattern) {
    // Supports glob patterns: "Falcon9.*.Thrust", "*.Aero.*"
    return match_glob(registry_, pattern);
}
```
