# Quick Start: Your First Icarus Component

This guide walks you through implementing a minimal component in under 5 minutes.

---

## The Component Protocol

Every component implements three lifecycle methods:

```cpp
template <typename Scalar>
class MyComponent : public Component<Scalar> {
public:
    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override;
    void Stage(Backplane<Scalar>& bp, const RunConfig& rc) override;
    void Step(Scalar t, Scalar dt) override;
};
```

| Method | When | What to Do |
|:-------|:-----|:-----------|
| `Provision` | Once at startup | Register outputs, load config, allocate memory |
| `Stage` | Each run start | Resolve input pointers, apply ICs |
| `Step` | Every Δt | Read inputs → compute → write outputs |

---

## Example: A Gain Component

```cpp
#include <icarus/Component.hpp>

template <typename Scalar>
class Gain : public Component<Scalar> {
    // Parameters (loaded once)
    Scalar gain_;

    // I/O pointers (bound at Stage)
    const Scalar* input_;
    Scalar* output_;

public:
    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Load parameter from config
        gain_ = cfg.get<double>("gain", 1.0);

        // Register output signal
        bp.register_output(name() + ".output", &output_value_);
    }

    void Stage(Backplane<Scalar>& bp, const RunConfig& rc) override {
        // Resolve input signal from another component
        input_ = bp.resolve<Scalar>(cfg_.get<std::string>("input_signal")).ptr();
    }

    void Step(Scalar t, Scalar dt) override {
        // Core logic: output = gain × input
        *output_ = gain_ * (*input_);
    }

private:
    Scalar output_value_;
};
```

---

## Configuration

```yaml
# config/components/gain.yaml
components:
  - type: Gain
    name: MyGain
    params:
      gain: 2.5
      input_signal: "Sensor.raw_value"
```

---

## Key Rules

1. **No allocation in `Step()`** — all memory allocated in `Provision()`
2. **No string lookups in `Step()`** — resolve pointers in `Stage()`
3. **Use `janus::` math** — `janus::sin()`, not `std::sin()`
4. **Use `janus::where()` for branching** — no `if/else` on `Scalar`
5. **Template on `Scalar`** — enables both `double` and `casadi::MX`

---

## Symbolic Compatibility Checklist

```cpp
// ✅ Correct
Scalar y = janus::where(x > 0, a, b);
Scalar z = janus::sin(angle);

// ❌ Wrong
if (x > 0) { y = a; } else { y = b; }
double z = std::sin(angle);
```

---

## Next Steps

- [02_component_protocol.md](02_component_protocol.md) — Full protocol details
- [03_signal_backplane.md](03_signal_backplane.md) — Signal registration & types
- [04_lifecycle.md](04_lifecycle.md) — Provision/Stage/Step in depth
- [21_symbolic_constraints.md](21_symbolic_constraints.md) — Janus compatibility rules
