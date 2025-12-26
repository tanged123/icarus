# Phase 3: Symbolic Mode - Implementation Documentation

**Status:** Proposed
**Target:** Dual-mode numeric/symbolic simulation with graph export

---

## Overview

Phase 3 validates that Icarus runs correctly with `SymbolicScalar` (Janus's `janus::SymbolicScalar`, wrapping `casadi::MX`) as the scalar backend, enabling automatic differentiation, optimization, and trajectory analysis.

### Key Deliverables

1. **SimulationBuilder**: Fluent API for simulation configuration
2. **SimulationRunner**: Standardized execution with lifecycle management
3. **SymbolicTracer**: Extract `janus::Function` from simulation dynamics
4. **Dual-Mode Validation**: Prove numeric/symbolic equivalence

---

## Documents

| Document | Description |
|:---------|:------------|
| [phase3_implementation_plan.md](./phase3_implementation_plan.md) | Main implementation plan with all tasks |
| [simulation_builder_spec.md](./simulation_builder_spec.md) | Technical spec for SimulationBuilder & Runner |
| [symbolic_tracer_spec.md](./symbolic_tracer_spec.md) | Technical spec for graph extraction |
| [task_checklist.md](./task_checklist.md) | Detailed task breakdown with checkboxes |

---

## Quick Reference

### Current Architecture Assessment

| System | Status | Notes |
|:-------|:-------|:------|
| SignalRegistry | ✅ Ready | Templated on `<Scalar>` |
| Simulator | ✅ Ready | Clean template chain |
| Components | ✅ Ready | No `std::` math violations |
| Integrator | ✅ Ready | JanusVector throughout |

### New Components

```
include/icarus/
├── sim/
│   ├── SimulationBuilder.hpp    [NEW]
│   ├── SimulationRunner.hpp     [NEW]
│   └── SimulationResults.hpp    [NEW]
└── symbolic/
    └── SymbolicTracer.hpp       [NEW]
```

### Exit Criteria

- [ ] `Simulator<SymbolicScalar>` compiles
- [ ] `GenerateGraph()` returns valid `janus::Function`
- [ ] Numeric/symbolic match within 1e-10
- [ ] Jacobian extraction works
- [ ] Zero Janus violations
- [ ] Examples use SimulationBuilder

---

## Getting Started

1. Read [phase3_implementation_plan.md](./phase3_implementation_plan.md) for full context
2. Start with **3.0.x validation tasks** to verify readiness
3. Implement **3.1 SimulationBuilder** as foundation
4. Proceed to **3.3 SymbolicTracer** once builder is stable
5. Complete **3.4 tests** to verify correctness

---

## Related Architecture Documents

- [07_janus_integration.md](../../architecture/07_janus_integration.md) - Template-first paradigm
- [21_symbolic_constraints.md](../../architecture/21_symbolic_constraints.md) - Janus compliance rules
- [14_trim_optimization.md](../../architecture/14_trim_optimization.md) - Future optimization use
