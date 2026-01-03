# Phase Manager Implementation

**Status:** Proposed  
**Phase:** 6.1  
**Related:** [17_events_phases.md](../../architecture/17_events_phases.md) | [10_entity_lifecycle.md](../../architecture/10_entity_lifecycle.md)

---

## Overview

The Phase Manager coordinates flight phase transitions as a **pure orchestration concern**. Components remain completely phase-agnostic—they have no knowledge of phases. Phasing is configured entirely in YAML and enforced using **existing infrastructure**: the Scheduler (execution gating) and SignalRouter (output gating via gains).

### Design Philosophy

> [!IMPORTANT]
> **Components do NOT know about phases.** Phase logic is applied externally via:
>
> 1. **Scheduler group gating**: Add `active_phases` to scheduler groups (reuses existing frame divisor logic)
> 2. **Route gain gating**: PhaseManager sets route `gain=0.0` for outputs from inactive components
> 3. **Derivative zeroing**: StateManager zeros derivatives for inactive components

This enables any component to be phase-gated without modifying its implementation.

---

## Reuse of Existing Infrastructure

### What We Already Have

| Infrastructure | Location | How We Reuse It |
|----------------|----------|-----------------|
| **Scheduler groups** | `Scheduler.hpp` | Add `active_phases` filter to groups |
| **Route gains** | `SignalRouter.hpp` | Use `gain=0.0` to zero inactive outputs |
| **Route parsing** | `SimulationLoader.hpp` | Add `active_phases` to route YAML parsing |
| **Group-based execution** | `Simulator.hpp` | Extend `should_run()` to check phase |
| **State derivatives** | `StateManager.hpp` | Zero derivatives for inactive components |

### Minimal New Code Required

1. **ConditionParser** - Parse transition conditions (new)
2. **PhaseManager** - Evaluate transitions, track current phase (new, small)
3. **Phase signal** - One `int32` signal per entity (auto-registered)

---

## Design

### Phase Signal Convention

Every entity gets an auto-registered phase signal:

```yaml
Vehicle.phase: 0  # GROUND=0, BOOST=1, COAST=2, REENTRY=3, LANDED=4
```

### YAML Configuration

Phase configuration extends existing YAML structure minimally:

```yaml
# Phase definitions and transitions
phases:
  Vehicle:
    definitions:
      GROUND: 0
      BOOST: 1
      COAST: 2
      DESCENT: 3
      LANDED: 4
    initial: GROUND
    transitions:
      - from: GROUND
        to: BOOST
        condition: "Propulsion.ignition == 1"
      - from: BOOST
        to: COAST
        condition: "Propulsion.fuel_mass < 0.01"

# Components - unchanged from existing format
components:
  - name: Booster
    type: RocketEngine
    # ... config ...
    
  - name: Parachute
    type: ParachuteDrag
    # ... config ...

# Scheduler groups - add active_phases (extends existing)
scheduler:
  groups:
    - name: Propulsion
      rate_hz: 400
      active_phases: [BOOST]          # NEW: Only run during BOOST
      members:
        - component: Vehicle.Booster
        
    - name: Recovery
      rate_hz: 100
      active_phases: [DESCENT, LANDED]  # NEW: Only run during DESCENT/LANDED
      members:
        - component: Vehicle.Parachute
        
    - name: Navigation
      rate_hz: 400
      # No active_phases = runs in ALL phases (default behavior)
      members:
        - component: Vehicle.Nav

# Routes - add active_phases for output gating (extends existing)
routes:
  - input: Vehicle.EOM.thrust
    output: Vehicle.Booster.thrust
    gain: 1.0
    active_phases: [BOOST]           # NEW: Zero output when not in BOOST
    
  - input: Vehicle.EOM.drag
    output: Vehicle.Parachute.drag
    active_phases: [DESCENT, LANDED]  # NEW: Zero output when not in DESCENT/LANDED
```

### Integration with Scheduler

The existing `Scheduler::GetGroupsForFrame()` logic is extended to also check phase:

```cpp
// In Scheduler.hpp
[[nodiscard]] std::vector<std::string> GetGroupsForFrame(
    int frame_count, 
    int32_t current_phase) const 
{
    std::vector<std::string> active_groups;
    for (const auto &group : config_.groups) {
        // Existing frame divisor check
        auto it = config_.group_frame_divisors.find(group.name);
        if (it != config_.group_frame_divisors.end()) {
            int divisor = it->second;
            if (frame_count % divisor != 0) {
                continue;  // Skip: not this frame
            }
        }
        
        // NEW: Phase check (empty = all phases)
        if (!group.active_phases.empty()) {
            if (group.active_phases.find(current_phase) == group.active_phases.end()) {
                continue;  // Skip: not active in this phase
            }
        }
        
        active_groups.push_back(group.name);
    }
    return active_groups;
}
```

### Integration with SignalRouter

Routes with `active_phases` have their gain dynamically adjusted:

```cpp
// In SignalRouter.hpp - extend SignalRoute
struct SignalRoute {
    std::string input_path;
    std::string output_path;
    
    double gain = 1.0;
    double offset = 0.0;
    double delay = 0.0;
    
    // NEW: Phase gating
    std::set<int32_t> active_phases;  // Empty = all phases
    
    // NEW: Get effective gain based on current phase
    double EffectiveGain(int32_t current_phase) const {
        if (active_phases.empty()) {
            return gain;  // No phase restriction
        }
        if (active_phases.contains(current_phase)) {
            return gain;  // Active in this phase
        }
        return 0.0;  // Inactive: zero the output
    }
};
```

### PhaseManager Component

Minimal new component to track phase and evaluate transitions:

```cpp
class PhaseManager {
public:
    struct PhaseConfig {
        std::map<std::string, int32_t> definitions;  // "BOOST" -> 1
        std::string initial_phase;
        
        struct Transition {
            int32_t from_phase;  // -1 = any
            int32_t to_phase;
            std::string condition;
        };
        std::vector<Transition> transitions;
    };

    void Configure(const PhaseConfig& config);
    
    // Called after Step() to evaluate transitions
    void EvaluateTransitions(SignalRegistry<double>& registry);
    
    // Query current phase
    int32_t CurrentPhase() const { return current_phase_; }
    
private:
    int32_t current_phase_ = 0;
    PhaseConfig config_;
    std::unique_ptr<ConditionParser> parser_;
};
```

### Integration with Simulator::Step()

Minimal changes to existing Step() logic:

```cpp
void Simulator::Step(double dt) {
    // ... existing preamble ...
    
    // Get active groups for this frame AND current phase
    int32_t current_phase = phase_manager_.CurrentPhase();
    auto active_groups = scheduler_.GetGroupsForFrame(frame_count_, current_phase);
    
    // ... existing execution loop (unchanged) ...
    
    // After Step(): evaluate phase transitions
    phase_manager_.EvaluateTransitions(registry_);
    
    // Update route gains for next step (if phase changed)
    if (phase_manager_.PhaseChangedThisStep()) {
        UpdateRouteGains(current_phase);
    }
    
    // ... existing integration ...
}

void Simulator::UpdateRouteGains(int32_t current_phase) {
    // Update backplane wiring with effective gains
    for (const auto& route : router_.GetRoutes()) {
        double effective_gain = route.EffectiveGain(current_phase);
        backplane_.UpdateGain(route.input_path, effective_gain);
    }
}
```

### Derivative Gating

StateManager already collects derivatives by component. Extend to zero inactive:

```cpp
// In StateManager.hpp
JanusVector<Scalar> GetDerivatives(
    const std::set<std::string>& active_components) const 
{
    JanusVector<Scalar> xdot(total_states_);
    
    for (const auto& binding : bindings_) {
        bool is_active = active_components.empty() ||
                         active_components.contains(binding.component_name);
        
        for (size_t i = 0; i < binding.size; ++i) {
            if (is_active) {
                xdot[binding.offset + i] = *binding.derivative_ptrs[i];
            } else {
                xdot[binding.offset + i] = Scalar(0);  // Freeze state
            }
        }
    }
    return xdot;
}
```

---

## Proposed Changes

### Existing File Modifications (Minimal)

#### [MODIFY] [SchedulerConfig](file:///home/tanged/sources/icarus/include/icarus/sim/SimulatorConfig.hpp)

```diff
 struct SchedulerGroupConfig {
     std::string name;
     double rate_hz = 100.0;
     int priority = 0;
     std::vector<GroupMember> members;
+    std::set<int32_t> active_phases;  // Empty = all phases
 };
```

---

#### [MODIFY] [SignalRoute](file:///home/tanged/sources/icarus/include/icarus/signal/SignalRouter.hpp)

```diff
 struct SignalRoute {
     std::string input_path;
     std::string output_path;
     double gain = 1.0;
     double offset = 0.0;
     double delay = 0.0;
+    std::set<int32_t> active_phases;
+    
+    double EffectiveGain(int32_t phase) const;
 };
```

---

#### [MODIFY] [SimulationLoader](file:///home/tanged/sources/icarus/include/icarus/io/SimulationLoader.hpp)

- Parse `phases:` section
- Parse `active_phases` in scheduler groups
- Parse `active_phases` in routes

---

#### [MODIFY] [Scheduler](file:///home/tanged/sources/icarus/include/icarus/sim/Scheduler.hpp)

- Add `current_phase` parameter to `GetGroupsForFrame()`
- Filter groups by `active_phases`

---

#### [MODIFY] [Simulator](file:///home/tanged/sources/icarus/include/icarus/sim/Simulator.hpp)

- Add `PhaseManager` member
- Pass current phase to scheduler
- Call `EvaluateTransitions()` after Step
- Update route gains on phase change

---

#### [MODIFY] [StateManager](file:///home/tanged/sources/icarus/include/icarus/sim/StateManager.hpp)

- Add optional `active_components` parameter to `GetDerivatives()`
- Zero derivatives for inactive components

---

### New Files (Minimal)

#### [NEW] [ConditionParser.hpp](file:///home/tanged/sources/icarus/include/icarus/core/ConditionParser.hpp)

- Expression tokenizer for condition strings
- Support for signal references (`Component.signal`)
- Comparison operators: `<`, `>`, `<=`, `>=`, `==`, `!=`
- Boolean logic: `AND`, `OR`, `NOT`, parentheses

---

#### [NEW] [PhaseManager.hpp](file:///home/tanged/sources/icarus/include/icarus/sim/PhaseManager.hpp)

- Phase state tracking
- Transition evaluation using ConditionParser
- Phase change detection

---

## Verification Plan

### Automated Tests

#### Existing Tests (must pass)

All existing scheduler and routing tests should continue to pass (no active_phases = default behavior).

#### New Tests: `tests/sim/test_phase_manager.cpp`

1. `ConditionParser_SimpleComparison` - Parse `"x < 10"`
2. `ConditionParser_BooleanLogic` - Parse `"x > 5 AND y < 3"`
3. `PhaseManager_SingleTransition` - GROUND→BOOST on condition
4. `PhaseManager_NoEventCascade` - Transitions don't cascade
5. `Scheduler_PhaseFiltering` - Groups filtered by active_phases
6. `Route_PhaseGating` - Gain zeros when phase inactive
7. `StateManager_DerivativeZeroing` - Derivatives frozen for inactive components

```bash
./scripts/test.sh
ctest --test-dir build -R "PhaseManager|ConditionParser" -VV
```

### Integration Verification

1. Create multi-phase demo (BOOST → COAST → DESCENT)
2. Verify Booster group only runs during BOOST
3. Verify Booster outputs are zero after BOOST
4. Verify Booster state is frozen after BOOST

---

## Benefits of This Design

| Aspect | Benefit |
|--------|---------|
| **Minimal changes** | Extends existing Scheduler/Router concepts |
| **Component agnostic** | No component code changes ever needed |
| **YAML-driven** | All phase logic in configuration |
| **Backward compatible** | Omitting `active_phases` = existing behavior |
| **Existing patterns** | Follows gain/offset/delay pattern for routes |
| **Testable** | Each layer can be tested independently |

---

## Comparison: Before vs. After

### Before (rejected: component-aware)

```cpp
// Components must implement phase logic
class Booster : public Component<Scalar> {
    void Step(Scalar t, Scalar dt) override {
        if (*phase_ != BOOST) return;  // Component knows about phases!
        // ...
    }
};
```

### After (adopted: orchestration-only)

```cpp
// Components are phase-agnostic
class Booster : public Component<Scalar> {
    void Step(Scalar t, Scalar dt) override {
        // Just compute thrust - orchestration handles activation
        *thrust_ = compute_thrust(...);
    }
};
```

```yaml
# All phase logic in YAML
scheduler:
  groups:
    - name: Propulsion
      active_phases: [BOOST]
      members:
        - component: Vehicle.Booster
```

---

## Open Questions

1. **Symbolic mode**: For symbolic tracing, outputs are gated via `gain=0.0` which is symbolically traceable. Scheduler skipping means inactive paths aren't traced. Is this acceptable?

2. **Route gain updates**: Should we update gains lazily (on phase change) or recompute effective gains every step?

3. **Phase signal access**: Should components be able to read the phase signal (for informational purposes) even though they can't act on it?
