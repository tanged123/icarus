# Introspection Graph Model

## Motivation

Vehicle6DOF unifies MassAggregator + ForceAggregator + RigidBody6DOF into a single
component, eliminating ~20 internal signal routes. The current introspection model
(`DataDictionary`) only exposes flat component lists with `inputs/outputs/wired_to`.
Upstream visualization tools (Daedalus) cannot see implicit edges (source bindings
via `resolve()`) and thus cannot render the full block diagram.

## Key Insight

The Backplane already records `resolve()` calls per component:

- `Backplane::resolve()` pushes to `resolved_inputs_`
- `Simulator::Stage()` persists these: `inputs_[comp] = backplane_.resolved_inputs()`
- `Simulator::Provision()` persists outputs: `outputs_[comp] = backplane_.registered_outputs()`

Any component using the sourcing pattern automatically has its implicit dependencies
captured — no per-component modifications required.

## Data Structures

### EdgeKind

```cpp
enum class EdgeKind { Route, Resolve };
```

- **Route**: explicit `SignalRouter` connection
- **Resolve**: implicit dependency via `Backplane::resolve()` (e.g., Vehicle6DOF
  reading Engine.force.x)

### IntrospectionEdge

```cpp
struct IntrospectionEdge {
    std::string source;  // Source signal path
    std::string target;  // Signal path (routes) or component name (resolves)
    EdgeKind kind;
};
```

### IntrospectionGraph

```cpp
struct IntrospectionGraph {
    DataDictionary dictionary;             // Existing nodes + signals
    std::vector<IntrospectionEdge> edges;  // Topology edges
    nlohmann::json ToJSON() const;         // Serialization
};
```

## JSON Schema Extension

Additive to existing DataDictionary schema (backward compatible):

```json
{
  "summary": { "...existing fields...", "total_edges": 15 },
  "components": [ "...unchanged..." ],
  "edges": [
    { "source": "Aero.density_input", "target": "Env.Atm.density", "kind": "route" },
    { "source": "Rocket.Engine.force.x", "target": "Rocket.Vehicle", "kind": "resolve" }
  ]
}
```

## Graph Population (Simulator::GetIntrospectionGraph)

1. Call `GetDataDictionary()` for node data
2. Iterate `router_.GetRoutes()` → `EdgeKind::Route` edges
3. Iterate `inputs_` map → `EdgeKind::Resolve` edges

## Files

| File | Change |
|:-----|:-------|
| `include/icarus/io/data/IntrospectionGraph.hpp` | New — graph struct + serialization |
| `include/icarus/sim/Simulator.hpp` | Add `GetIntrospectionGraph()` method |
| `interfaces/python/icarus_python.cpp` | Add `introspection_graph` property |
| `interfaces/c_api/icarus.h` | Add `icarus_get_introspection_graph_json()` |
| `interfaces/c_api/icarus_c.cpp` | Add implementation |
| `tests/io/test_introspection_graph.cpp` | New — unit tests |
| `tests/components/test_vehicle6dof.cpp` | Add resolve-edge verification |
| `tests/python/test_simulator.py` | Add introspection_graph test |

## Not Modified

- `Component.hpp` — no virtual methods added
- `Vehicle6DOF.hpp` — no changes needed
- `DataDictionary.hpp` — backward compatible
- `Backplane.hpp` — already has resolve() tracking
