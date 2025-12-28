# Phase 4.0.7: Simulator Refactor Plan

**Status:** Proposed
**Prerequisite:** Phase 4.0.1-4.0.6 complete

---

## Plan Structure

This plan has been split into focused sub-documents for easier navigation and refinement:

| Document | Description |
|:---------|:------------|
| **[phase4_0_7_overview.md](phase4_0_7_overview.md)** | Architecture overview, configuration hierarchy |
| **[phase4_0_7_scheduler.md](phase4_0_7_scheduler.md)** | Hierarchical multi-rate scheduler configuration |
| **[phase4_0_7_entity_system.md](phase4_0_7_entity_system.md)** | Entity templates, instantiation, swarms |
| **[phase4_0_7_staging.md](phase4_0_7_staging.md)** | Trim optimization, linearization, symbolic generation |
| **[phase4_0_7_simulator_api.md](phase4_0_7_simulator_api.md)** | Simulator API refactor, lifecycle, problem statement |
| **[phase4_0_7_config_structs.md](phase4_0_7_config_structs.md)** | All configuration struct definitions |
| **[phase4_0_7_implementation.md](phase4_0_7_implementation.md)** | Implementation order, exit criteria, migration guide |

---

## Quick Summary

### Goals

1. **Clean Simulator API** - Reduce 50+ public methods to 4 core operations: `FromConfig()`, `Stage()`, `Step()`, `~Simulator()`
2. **Entity-Centric Configuration** - Entity templates are self-contained units with components, routes, scheduler, staging
3. **Group-Based Multi-Rate Scheduler** - Every component belongs to a group with its own rate and priority
4. **Trim/Linearization/Symbolic** - Stage phase handles equilibrium finding, state-space extraction, and symbolic graph generation

### Key Architecture Decisions

| Decision | Choice |
|:---------|:-------|
| Scheduling mode | **Explicit** (user-defined priorities) - automatic is future TODO |
| Simulation rate | **Auto-derived** from fastest group rate across all entities |
| Rate constraints | All rates must be **integer divisors** of simulation rate (513 Hz rejected) |
| Entity architecture | **Per-entity templates** with internal routes, scheduler, staging |
| Scheduling model | **Group-based** - each component belongs to one group with rate_hz + priority |
| Cross-entity signals | See **current frame** values (whatever is currently populated) |
| Configuration format | **YAML** with entity templates as separate files |

### Implementation Phases

| Phase | Description | Tasks |
|:------|:------------|:------|
| A | Configuration Infrastructure | 7 tasks (config structs) |
| B | Entity System | 8 tasks (templates, swarms) |
| C | Scheduler Integration | 5 tasks (multi-rate execution) |
| D | Internal Managers | 2 tasks (StateManager, IntegrationManager) |
| E | Simulator Core Refactor | 4 tasks (API, lifecycle) |
| F | Trim Optimization | 3 tasks (Newton solver) |
| G | Linearization | 2 tasks (A,B,C,D matrices) |
| H | Symbolic Mode | 3 tasks (CasADi export) |
| I | Testing & Cleanup | 7 tasks (tests, docs) |

---

## Start Here

Begin with the **[Overview](phase4_0_7_overview.md)** for the full architecture diagram and configuration hierarchy.
