# Phase 4.0.7: Simulator Refactor Plan

**Status:** Proposed
**Prerequisite:** Phase 4.0.1-4.0.6 complete

---

## Critical Design Decisions

These decisions guide all sub-documents:

| Decision | Choice | Rationale |
|:---------|:-------|:----------|
| **Single Simulator class** | NOT templated on Scalar | User sees one `Simulator`, symbolic mode used internally during `Stage()` |
| **Single-file default** | Everything can be inline | Multi-file is organizational convenience, not requirement |
| **No backwards compatibility** | Breaking changes OK | Dev mode, no external users yet |
| **No swarm expressions** | Remove `${index}` etc. | Jinja-style templating is future work |
| **Rate bridging** | Deferred | Real embedded systems have these constraints too |

---

## Plan Structure

This plan has been split into focused sub-documents for easier navigation and refinement:

| Document | Description |
|:---------|:------------|
| **[phase4_0_7_overview.md](phase4_0_7_overview.md)** | Architecture overview, configuration hierarchy |
| **[phase4_0_7_yaml_config.md](phase4_0_7_yaml_config.md)** | YAML configuration loading, validation, error handling |
| **[phase4_0_7_scheduler.md](phase4_0_7_scheduler.md)** | Hierarchical multi-rate scheduler configuration |
| **[phase4_0_7_entity_system.md](phase4_0_7_entity_system.md)** | Entity templates, instantiation, swarms |
| **[phase4_0_7_staging.md](phase4_0_7_staging.md)** | Trim optimization, linearization, symbolic generation |
| **[phase4_0_7_simulator_api.md](phase4_0_7_simulator_api.md)** | Simulator API refactor, lifecycle, problem statement |
| **[phase4_0_7_config_structs.md](phase4_0_7_config_structs.md)** | All configuration struct definitions |
| **[phase4_0_7_implementation.md](phase4_0_7_implementation.md)** | Implementation order, exit criteria, usage examples |

---

## Quick Summary

### Goals

1. **Clean Simulator API** - Reduce 50+ public methods to 4 core operations: `FromConfig()`, `Stage()`, `Step()`, `~Simulator()`
2. **Single Simulator Class** - NOT templated on Scalar; symbolic mode used internally during `Stage()`
3. **Single-File Default** - Entire sim definable in one YAML; multi-file is optional organization
4. **Entity-Centric Configuration** - Entity templates are self-contained units (optional for complex sims)
5. **Group-Based Multi-Rate Scheduler** - Every component belongs to a group with its own rate and priority

### Implementation Phases

| Phase | Description | Status |
|:------|:------------|:-------|
| A | Configuration Infrastructure | **Core** |
| B | Entity System | **Core** |
| C | Scheduler Integration | **Core** |
| D | Internal Managers | **Core** |
| E | Simulator Core Refactor | **Core** |
| F | Trim Optimization | *Future* |
| G | Linearization | *Future* |
| H | Symbolic Mode | *Exists* (symbolic_orbital_demo.cpp) |
| I | Testing & Cleanup | **Core** |

---

## Start Here

Begin with the **[Overview](phase4_0_7_overview.md)** for the full architecture diagram and configuration hierarchy.
