# Unwired Inputs Implementation Plan

**Status:** Proposed
**Purpose:** Allow inputs to remain unwired with a default value of 0
**Related:** Signal backplane, Hermes integration, external poke interface

---

## Problem Statement

Currently, Icarus throws an error during `Stage()` if any input signals are not wired:

```
IcarusError: [icarus] Wiring: Unwired inputs: Rocket.Engine.throttle_cmd
```

This prevents valid use cases:
1. **External injection** - Signals controlled by external systems (Hermes) via poke
2. **Optional inputs** - Inputs that have sensible defaults (e.g., throttle_cmd = 0)
3. **Incremental development** - Building simulations piece by piece

## Proposed Solution

**Unwired inputs should default to 0 and not cause errors.**

This is the simplest, most intuitive behavior:
- All signal storage is zero-initialized
- Wiring connects an output to an input (copies value each frame)
- Unwired inputs simply retain their value (initially 0, or whatever was poked)

### Design Principles

1. **Zero is the universal default** - Safe for most signals (zero throttle, zero perturbation, etc.)
2. **Poke always works** - External systems can set any input at any time
3. **Wiring is optional** - Components declare inputs; users decide what to connect
4. **Warnings, not errors** - Inform users of unwired inputs but don't block simulation

---

## Implementation

### Phase 1: Remove Unwired Input Error

**File:** `src/sim/Simulator.cpp` (or wherever Stage() validates wiring)

**Change:** Convert the unwired inputs error to a warning.

```cpp
// BEFORE
if (!unwired_inputs.empty()) {
    throw StageError("Wiring: Unwired inputs: " + join(unwired_inputs, ","));
}

// AFTER
if (!unwired_inputs.empty()) {
    LOG_WARN("[WIRE] Unwired inputs (defaulting to 0): {}", join(unwired_inputs, ", "));
}
```

**Estimated change:** ~5 lines

### Phase 2: Ensure Zero Initialization

**File:** `include/icarus/signal/Registry.hpp` or `Backplane.hpp`

**Verify:** Signal storage is zero-initialized when allocated.

This should already be the case if using `std::vector<Scalar>` or similar, but verify:

```cpp
// Signal storage should be value-initialized (zeros for numeric types)
std::vector<Scalar> signal_storage_(total_signals, Scalar(0));
```

**Estimated change:** 0-3 lines (likely already correct)

### Phase 3: Update Configuration

**File:** `include/icarus/sim/SimulatorConfig.hpp`

**Change:** Default `validate_wiring` to `false`, or remove it entirely.

```cpp
struct StageConfig {
    // ...
    bool validate_wiring = false;  // Changed from true
    bool warn_on_unwired = true;   // Keep warnings
};
```

**Estimated change:** 1 line

### Phase 4: Update Documentation

**File:** `docs/architecture/` (signal documentation)

Add note explaining:
- Unwired inputs default to 0
- Use poke for external control
- Wiring is for internal Icarus data flow

---

## Verification

### Test Cases

1. **Unwired input stays at zero**
   ```cpp
   // Component with input that isn't wired
   // Verify input.get() returns 0 throughout simulation
   ```

2. **Poked value persists**
   ```cpp
   sim.stage();
   sim.poke("Component.input", 1.5);
   sim.step();
   EXPECT_EQ(sim.peek("Component.input"), 1.5);
   ```

3. **Wiring overrides poke**
   ```cpp
   // Wire output -> input
   // Poke input to 1.5
   // After step, input should have output's value (wiring wins)
   ```

### Integration Test

The rocket_ascent.yaml should work without modification:
- `Rocket.Engine.throttle_cmd` remains unwired (defaults to 0)
- Python test can poke `throttle_cmd` to 1.0 for full thrust
- Simulation runs successfully

---

## Alternative Considered

**Explicit default values in YAML:**
```yaml
inputs:
  throttle_cmd:
    type: scalar
    default: 0.0  # Explicit default
```

**Rejected because:**
- Adds complexity to component definition
- Zero is almost always the right default
- External systems (Hermes) will poke values anyway
- YAGNI - can add later if needed

---

## Summary

| Change | File | Lines |
|--------|------|-------|
| Remove error, add warning | Simulator.cpp | ~5 |
| Verify zero init | Registry.hpp | ~0-3 |
| Default validate_wiring=false | SimulatorConfig.hpp | 1 |
| **Total** | | **~6-9 lines** |

This is a minimal, clean change that enables external signal injection while maintaining safety (warnings inform users of unwired inputs).
