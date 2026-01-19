# Phase 9: Unified Vehicle Component

**Status:** Proposed
**Goal:** Unify 6DOF dynamics, mass aggregation, and force aggregation into a single auto-discovering component.

---

## Overview

Phase 9 addresses the complexity of wiring separate RigidBody6DOF, MassAggregator, and ForceAggregator components by creating a unified `Vehicle6DOF` component that automatically discovers mass and force providers within its entity namespace.

This phase builds on the IODA flat peer model while recovering the ergonomic benefits of automatic aggregation from hierarchical system-model architectures.

---

## Documents

| Document | Description |
|:---------|:------------|
| [phase9_physical_component.md](./phase9_physical_component.md) | **Part A**: PhysicalComponent intermediate class with body attachment |
| [phase9_vehicle6dof.md](./phase9_vehicle6dof.md) | **Part B**: Vehicle6DOF component with auto-discovery |
| [../unified_vehicle_component.md](../unified_vehicle_component.md) | Original design document |

---

## Key Concepts

### PhysicalComponent Class (Part A)

An intermediate class between `Component<Scalar>` and physical components (mass sources, force sources, sensors). Provides:

- Body position (mounting location in vehicle frame)
- Body orientation (rotation from body to component local frame)
- Config parsing for quaternion and Euler angle formats
- Transform helper methods

### Vehicle6DOF Component (Part B)

A unified dynamics component that:

1. **Discovers** mass and force providers by scanning signals in its entity namespace
2. **Aggregates** mass properties with frame transformation (local → body)
3. **Aggregates** forces with frame transformation (local/ECEF → body)
4. **Computes** 13-state quaternion 6DOF dynamics
5. **Outputs** derived quantities (LLA, NED velocity, Euler angles)

### Signal Naming Conventions

| Pattern | Frame | Example |
|:--------|:------|:--------|
| `{comp}.force_local.*` | Component local | Thrusters, engines |
| `{comp}.force_body.*` | Body | Aerodynamics |
| `{comp}.force_ecef.*` | ECEF | Gravity |
| `{comp}.mass` | - | All mass sources |
| `{comp}.cg_local.*` | Component local | Mass sources |
| `{comp}.inertia_local.*` | Component local | Mass sources |

---

## Implementation Order

1. **Phase 9A**: PhysicalComponent class
   - Virtual accessors in Component base
   - PhysicalComponent intermediate class
   - Rotation utilities (Euler ↔ quaternion)
   - Unit tests

2. **Phase 9B**: Vehicle6DOF component
   - Backplane discovery API
   - Component registry
   - Handle bundles (mass/force sources)
   - Vehicle6DOF implementation
   - Component migrations (optional)
   - Integration tests

---

## Benefits

| Before | After |
|:-------|:------|
| 3 separate components | 1 unified component |
| ~26 manual routes | 0 routes |
| Manual frame transforms in components | Centralized in Vehicle6DOF |
| Fragile wiring | Robust auto-discovery |

---

## Dependencies

- Phase 6: Unified signal model (complete)
- Vulcan: Mass aggregation, 6DOF dynamics
- Janus: Quaternion operations

---

## Related Architecture Docs

- [02_component_protocol.md](../../architecture/02_component_protocol.md) — Component lifecycle
- [12_quantity_aggregation.md](../../architecture/12_quantity_aggregation.md) — Aggregation theory
- [06_entities_namespaces.md](../../architecture/06_entities_namespaces.md) — Entity namespaces
