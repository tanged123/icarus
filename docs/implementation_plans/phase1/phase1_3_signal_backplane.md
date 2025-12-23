# Phase 1.3: Signal Backplane Implementation Plan

**Status:** ✅ Complete  
**Target:** Complete Signal Backplane with type-safe access and Vec3/Mat3 support

---

## Overview

Phase 1.3 extends the existing signal infrastructure to provide a full-featured Signal Backplane as specified in [03_signal_backplane.md](../../architecture/03_signal_backplane.md). The current implementation has a basic registry; this phase adds:

1. **Type-safe `SignalHandle<T>`** — bind-once-read-forever pattern
2. **Pointer-based access** — zero-overhead hot path
3. **Vec3/Mat3/Quat signal support** — automatic path expansion
4. **Enhanced metadata** — owner tracking, collision detection
5. **TypeTraits** — compile-time type ID mapping

---

## Current State Analysis

### Existing Infrastructure

| File | Contents | Status |
|:-----|:---------|:-------|
| [Signal.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Signal.hpp) | `SignalType`, `SignalLifecycle`, `SignalDescriptor` | ✅ Exists |
| [Registry.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Registry.hpp) | Basic `SignalRegistry<Scalar>` with index-based access | ⚠️ Needs extension |
| [Descriptor.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Descriptor.hpp) | Stub (re-exports Signal.hpp) | ⚠️ Needs content |
| [Error.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Error.hpp) | `SignalError` exception | ✅ Exists |

### Gap Analysis

| Feature | Architecture Requirement | Current Status |
|:--------|:-------------------------|:---------------|
| `SignalHandle<T>` | Type-safe pointer wrapper with metadata | ❌ Missing |
| Pointer binding | `resolve()` returns pointer for hot path | ❌ Returns index |
| Vec3/Mat3 expansion | Auto-expand `thrust` → `thrust.x/y/z` | ❌ Missing |
| TypeTraits | Compile-time type→DataType mapping | ❌ Missing |
| Owner tracking | Collision detection shows owning component | ❌ Missing |
| `register_output()` | Component registers output with pointer | ❌ Only descriptor |
| `register_static()` | Static signals separate from dynamic | ❌ Not differentiated |

---

## Proposed Changes

### Component 1: Type Traits and Signal Types

#### [MODIFY] [Signal.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Signal.hpp)

Add `TypeTraits` for compile-time type ID mapping and enhance `SignalDescriptor`:

```cpp
// TypeTraits for compile-time type ID dispatch
template <typename T> struct TypeTraits;

template <> struct TypeTraits<double> {
    static constexpr SignalType type_id = SignalType::Float64;
    static constexpr const char* name = "Float64";
};

template <> struct TypeTraits<int32_t> {
    static constexpr SignalType type_id = SignalType::Int32;
    static constexpr const char* name = "Int32";
};

template <> struct TypeTraits<int64_t> {
    static constexpr SignalType type_id = SignalType::Int64;
    static constexpr const char* name = "Int64";
};
```

Extend `SignalDescriptor` with owner tracking:

```cpp
struct SignalDescriptor {
    // ... existing fields ...
    std::string owner_component;  // Component that registered this signal
    void* data_ptr = nullptr;     // Pointer to storage (for pointer-bind pattern)
};
```

---

### Component 2: Type-Safe Signal Handle

#### [NEW] [Handle.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Handle.hpp)

Create the `SignalHandle<T>` class for type-safe, zero-overhead access:

```cpp
template <typename T>
class SignalHandle {
    T* ptr_;
    const SignalDescriptor* meta_;

public:
    SignalHandle(T* ptr, const SignalDescriptor* meta) : ptr_(ptr), meta_(meta) {}

    // Zero-overhead access (hot path)
    T& operator*() { return *ptr_; }
    const T& operator*() const { return *ptr_; }
    T* operator->() { return ptr_; }
    const T* operator->() const { return ptr_; }

    // Raw pointer access for integration with Eigen loops
    T* ptr() { return ptr_; }
    const T* ptr() const { return ptr_; }

    // Introspection (cold path)
    const std::string& name() const { return meta_->name; }
    const std::string& unit() const { return meta_->unit; }
    SignalLifecycle lifecycle() const { return meta_->lifecycle; }

    // Validity check
    explicit operator bool() const { return ptr_ != nullptr; }
};
```

---

### Component 3: Enhanced Registry (Backplane)

#### [MODIFY] [Registry.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Registry.hpp)

Refactor to support the "bind once, read forever" pattern:

**Key Changes:**

1. Add `register_output()` that takes a pointer to component-owned storage
2. Add `register_static()` for immutable parameters
3. Add typed `resolve<T>()` returning `SignalHandle<T>`
4. Add owner tracking for collision detection
5. Add Vec3/Mat3 expansion helpers

```cpp
template <typename Scalar>
class SignalRegistry {
public:
    // Registration with pointer binding (Provision phase)
    template <typename T>
    void register_output(const std::string& name, T* data_ptr,
                        const std::string& unit = "",
                        const std::string& description = "");

    template <typename T>
    void register_static(const std::string& name, const T* data_ptr,
                        const std::string& unit = "",
                        const std::string& description = "");

    // Type-safe resolution (Stage phase)
    template <typename T>
    SignalHandle<T> resolve(const std::string& name);

    template <typename T>
    SignalHandle<const T> resolve_const(const std::string& name) const;

    // Vec3 resolution (returns struct with 3 handles)
    template <typename S>
    Vec3Handle<S> resolve_vec3(const std::string& name);

    // Query/introspection (cold path)
    std::vector<const SignalDescriptor*> query(const std::string& pattern) const;
    
    // Context management
    void set_current_component(const std::string& name);
    void clear_current_component();
};
```

---

### Component 4: Vec3/Mat3 Handle Helpers

#### [NEW] [VecHandle.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/VecHandle.hpp)

Provide structured access to vector signals:

```cpp
template <typename Scalar>
struct Vec3Handle {
    SignalHandle<Scalar> x;
    SignalHandle<Scalar> y;
    SignalHandle<Scalar> z;

    // Convenience: read as Eigen Vec3
    Vec3<Scalar> get() const {
        return Vec3<Scalar>(*x, *y, *z);
    }

    // Convenience: write from Eigen Vec3
    void set(const Vec3<Scalar>& v) {
        *x = v(0); *y = v(1); *z = v(2);
    }
};
```

---

### Component 5: Custom Error Types

#### [MODIFY] [Error.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Error.hpp)

Add specialized signal errors:

```cpp
class TypeMismatchError : public SignalError {
public:
    TypeMismatchError(const std::string& signal_name,
                     SignalType expected, SignalType actual);
};

class DuplicateSignalError : public SignalError {
public:
    DuplicateSignalError(const std::string& signal_name,
                        const std::string& existing_owner,
                        const std::string& new_owner);
};

class SignalNotFoundError : public SignalError {
public:
    explicit SignalNotFoundError(const std::string& signal_name);
};
```

---

## File Summary

| Action | File |
|:-------|:-----|
| MODIFY | [Signal.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Signal.hpp) |
| NEW | [Handle.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Handle.hpp) |
| MODIFY | [Registry.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/Registry.hpp) |
| NEW | [VecHandle.hpp](file:///home/tanged/sources/icarus/include/icarus/signal/VecHandle.hpp) |
| MODIFY | [Error.hpp](file:///home/tanged/sources/icarus/include/icarus/core/Error.hpp) |
| NEW | [tests/signal/test_signal.cpp](file:///home/tanged/sources/icarus/tests/signal/test_signal.cpp) |

---

## Verification Plan

### Automated Tests

All tests use GoogleTest. Run with:

```bash
# Build and run all tests
./scripts/test.sh

# Or run signal tests specifically (after build)
cd build && ctest -R signal --output-on-failure
```

#### Test Cases for `test_signal.cpp`

| Test Name | Description |
|:----------|:------------|
| `TypeTraits_Float64` | Verify `TypeTraits<double>::type_id == SignalType::Float64` |
| `TypeTraits_Int32` | Verify `TypeTraits<int32_t>::type_id == SignalType::Int32` |
| `TypeTraits_Int64` | Verify `TypeTraits<int64_t>::type_id == SignalType::Int64` |
| `SignalHandle_Access` | Create handle, verify `*handle` reads/writes correctly |
| `SignalHandle_Metadata` | Verify handle exposes name/unit from descriptor |
| `Registry_RegisterOutput` | Register output with pointer, verify binding |
| `Registry_RegisterStatic` | Register static signal, verify immutability flag |
| `Registry_ResolveTyped` | Resolve returns `SignalHandle<T>` with correct type |
| `Registry_TypeMismatch` | Resolve with wrong type throws `TypeMismatchError` |
| `Registry_DuplicateSignal` | Re-register same name throws `DuplicateSignalError` |
| `Registry_SignalNotFound` | Resolve unknown signal throws `SignalNotFoundError` |
| `Registry_OwnerTracking` | Verify descriptor contains owning component name |
| `Vec3Handle_ReadWrite` | Register Vec3, resolve as Vec3Handle, read/write |

#### Symbolic Backend Tests

| Test Name | Description |
|:----------|:------------|
| `RegistrySymbolic_RegisterResolve` | Basic registration with `casadi::MX` |
| `RegistrySymbolic_HandleAccess` | Verify handle works with symbolic values |
| `Vec3HandleSymbolic_ReadWrite` | Vec3Handle with symbolic Scalar |

---

### Build System Changes

#### [MODIFY] [tests/CMakeLists.txt](file:///home/tanged/sources/icarus/tests/CMakeLists.txt)

Add signal test:

```cmake
icarus_add_test(test_signal
    signal/test_signal.cpp
)
```

---

## Implementation Checklist

### Signal Types (Signal.hpp)

- [ ] Add `TypeTraits<double>` specialization
- [ ] Add `TypeTraits<int32_t>` specialization
- [ ] Add `TypeTraits<int64_t>` specialization
- [ ] Extend `SignalDescriptor` with `owner_component`
- [ ] Extend `SignalDescriptor` with `data_ptr`

### Signal Handle (Handle.hpp)

- [ ] Create `SignalHandle<T>` with pointer + metadata
- [ ] Implement `operator*()`, `operator->()`
- [ ] Implement `ptr()` for raw access
- [ ] Implement `name()`, `unit()`, `lifecycle()` getters
- [ ] Implement `operator bool()` for validity

### Vec Handle (VecHandle.hpp)

- [ ] Create `Vec3Handle<Scalar>` with x/y/z handles
- [ ] Implement `get()` returning `Vec3<Scalar>`
- [ ] Implement `set(const Vec3<Scalar>&)`
- [ ] Create `Mat3Handle<Scalar>` (if needed; can defer)

### Registry (Registry.hpp)

- [ ] Refactor `SignalRegistry` to store pointers
- [ ] Implement `register_output<T>(name, ptr, unit, desc)`
- [ ] Implement `register_static<T>(name, ptr, unit, desc)`
- [ ] Implement typed `resolve<T>(name)` returning handle
- [ ] Add `set_current_component()` / `clear_current_component()`
- [ ] Add owner tracking in registration
- [ ] Implement `query(pattern)` for introspection

### Error Types (Error.hpp)

- [ ] Add `TypeMismatchError`
- [ ] Add `DuplicateSignalError`
- [ ] Add `SignalNotFoundError`

### Tests (test_signal.cpp)

- [ ] Create `tests/signal/test_signal.cpp`
- [ ] Add TypeTraits tests
- [ ] Add SignalHandle tests
- [ ] Add Registry registration tests
- [ ] Add type mismatch tests
- [ ] Add duplicate signal tests
- [ ] Add Vec3Handle tests
- [ ] Add symbolic backend tests
- [ ] Update `tests/CMakeLists.txt`

---

## Design Decisions

### 1. Pointer-Based vs Index-Based Access

**Decision:** Hybrid approach

- Registration returns `SignalIndex` (backward compatible)
- `resolve<T>()` returns `SignalHandle<T>` with pointer
- Both index and pointer access supported

**Rationale:** Preserves existing tests while enabling zero-overhead hot path.

### 2. Storage Ownership

**Decision:** Component owns storage, registry stores pointers

- Component declares member: `double thrust_;`
- Registration: `register_output("thrust", &thrust_);`
- Registry stores `void*` to component storage

**Rationale:** Matches architecture doc; avoids double indirection.

### 3. Vec3 Expansion Strategy

**Decision:** Register Vec3 as three separate signals with `.x/.y/.z` suffixes

- Caller passes `Vec3<Scalar>*` to registration
- Registry creates 3 descriptors pointing to contiguous elements
- `resolve_vec3()` returns `Vec3Handle` with 3 bound handles

**Rationale:** Flat signal namespace for telemetry/recording; matches architecture.

### 4. Type Checking

**Decision:** Runtime type check at `resolve()` time (Stage phase)

- TypeTraits provides expected type
- Descriptor stores registered type
- Mismatch throws immediately

**Rationale:** Fail-fast before hot loop; matches "Type checking at Stage" principle.

---

## Exit Criteria

- [ ] All existing tests in `test_types.cpp` continue to pass
- [ ] New `test_signal.cpp` tests pass for both `double` and `casadi::MX`
- [ ] `SignalHandle<T>` provides zero-overhead access
- [ ] `Vec3Handle` correctly maps to 3 scalar signals
- [ ] Type mismatches throw clear errors at resolve time
- [ ] Duplicate signal registration detected and throws

---

## Dependencies

| Dependency | Purpose | Status |
|:-----------|:--------|:-------|
| Janus types (`Vec3<Scalar>`) | Vector signal support | ✅ Available via Types.hpp |
| GoogleTest | Test framework | ✅ Available |
| Phase 1.2 Types | `IcarusScalar`, error types | ✅ Complete |
