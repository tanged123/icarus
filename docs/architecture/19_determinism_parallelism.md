# Determinism Guarantees & Parallelism

**Related:** [05_execution_model.md](05_execution_model.md) | [09_memory_state_ownership.md](09_memory_state_ownership.md)

---

## 1. Determinism Guarantees

> [!IMPORTANT]
> **Numeric mode is bit-identical across runs** with the same inputs.

### How Determinism is Ensured

| Mechanism | Guarantee |
|-----------|-----------|
| **Explicit integrators** | RK4, Euler—no adaptive stepping randomness |
| **No dynamic allocation** | All memory allocated in Provision |
| **Strict signal ordering** | Scheduler order is deterministic |
| **No floating-point non-determinism** | Avoid `std::rand`, use seeded RNG from `vulcan::rng` |

### What Breaks Determinism

- Adaptive integrators (CVODES) with loose tolerances
- Multithreading without careful synchronization
- System time dependencies
- Uninitialized memory

---

## 2. Multi-threading & Parallelism

### Primary Strategy: Single-Threaded HITL Compatibility

> [!NOTE]
> **Parallelism is deprioritized.** The primary use case (HITL) requires deterministic single-threaded execution. For symbolic optimization solves, parallelism is baked into the optimizer.

### Hydra for HPC

For large-scale Monte Carlo (10k+ runs), use **Hydra** (external orchestrator):
- SLURM job submission
- HPC cluster management
- Results aggregation

Icarus remains single-threaded per instance; Hydra handles parallelism.

---

## 3. When Parallelism is Acceptable

| Scenario | Threading Strategy |
|----------|-------------------|
| **Single HITL run** | Single-threaded (deterministic) |
| **Monte Carlo batch** | Multiple Icarus instances (Hydra) |
| **Trajectory optimization** | Solver internal parallelism (IPOPT) |
| **Independent sub-graphs** | Future: thread-pool per rate group |

---

## 4. Thread Safety Guarantees

| Component | Thread Safety |
|-----------|---------------|
| **Backplane** | NOT thread-safe (single writer per signal) |
| **Simulator** | NOT thread-safe (single-threaded execution) |
| **Vulcan functions** | Thread-safe (stateless, pure functions) |
| **Janus math** | Thread-safe (no mutable state) |

---

## 5. Future: Parallel Rate Groups

For simulations with independent subsystems:

```cpp
// Conceptual future implementation
void Simulator::StepParallel(double t, double dt) {
    // Identify independent component subgraphs
    auto subgraphs = dependency_graph_.FindIndependentSets();

    // Execute independent subgraphs in parallel
    #pragma omp parallel for
    for (auto& subgraph : subgraphs) {
        for (auto* comp : subgraph) {
            comp->Step(t, dt);
        }
    }

    // Synchronize at aggregation points
    force_aggregator_->Step(t, dt);
    eom_->Step(t, dt);
}
```

> [!WARNING]
> **Not yet implemented.** Current Icarus is single-threaded. Parallel execution is a future optimization for specific use cases.

---

## 6. Reproducibility Checklist

For bit-exact reproducibility:

- [ ] Use fixed-step integrator (RK4, Euler)
- [ ] Seed all RNG with known values
- [ ] Avoid system time in calculations
- [ ] Use deterministic scheduler ordering
- [ ] Initialize all memory explicitly
- [ ] Avoid `std::unordered_map` iteration (non-deterministic order)
- [ ] Use `vulcan::rng` instead of `std::rand`
