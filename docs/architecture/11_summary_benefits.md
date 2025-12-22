# Summary of Benefits

**Related:** [00_index.md](00_index.md) | [01_core_philosophy.md](01_core_philosophy.md)

---

## Architecture Benefits

| Feature | Flat Architecture Benefit |
| :--- | :--- |
| **Refactoring** | Moving a component from "Avionics" to "Payload" is just a string rename. No pointer surgery. |
| **Testing** | Unit test any component in isolation by mocking the Signal Backplane. |
| **Parallelism** | Easy to identify independent sub-graphs in the flat list for thread-pool execution. |
| **Optimization** | Trivially exposes the global `f(x,u)` function needed for trajectory optimization. |

---

## Comparison with Hierarchical Architectures

| Aspect | Hierarchical (Scene Graph) | Flat (IDOA) |
|:-------|:--------------------------|:------------|
| **Data Flow** | `parent->child->doThing()` | Backplane signal read/write |
| **Ownership** | Deep nesting, pointer chains | All components are peers |
| **Refactoring** | Pointer surgery required | String rename |
| **Testing** | Must mock parent chain | Mock backplane only |
| **Symbolic Mode** | Deep branching breaks tracing | Linear execution trace |
| **Memory Layout** | Scattered across heap | Contiguous state vector |

---

## Key Design Wins

### 1. Simulink/GNC Compatibility
Aerospace engineers think in block diagrams. IDOA maps directly to this mental model—components are blocks, signals are wires.

### 2. Janus Symbolic Compatibility
CasADi requires a linear execution trace to build symbolic graphs. Deep object hierarchies with virtual dispatch obscure this trace. Flat execution enables clean graph extraction.

### 3. Data Locality
Contiguous state vectors are cache-friendly and solver-compatible. No chasing pointers through a scene graph.

### 4. Monte Carlo Performance
Provision once, Stage many times. Memory allocation happens once; each run only resets state and re-wires inputs.

### 5. Deterministic Execution
Flat component list with topological sort guarantees execution order. No hidden dependencies or callback ordering issues.

---

## When to Use This Architecture

**Good fit:**
- 6DOF aerospace simulation
- Hardware-in-the-loop testing
- Trajectory optimization
- Monte Carlo analysis
- Multi-vehicle simulations

**Less suitable:**
- Game engines with deep scene hierarchies
- GUI applications
- Event-driven systems with sparse interactions
