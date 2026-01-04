# Component Authoring Guide

**Related:** [02_component_protocol.md](../architecture/02_component_protocol.md) | [04_lifecycle.md](../architecture/04_lifecycle.md) | [05_execution_model.md](../architecture/05_execution_model.md)

---

This guide documents patterns for implementing Icarus components. Components are the fundamental execution units in Icarus, implementing physics models, sensors, actuators, and other simulation elements.

## Quick Checklist

- [ ] Template on `Scalar` for dual-mode (numeric/symbolic) support
- [ ] Use `vulcan::` for physics, `janus::` for math dispatch
- [ ] Register states/outputs/inputs in `Provision()` with backplane APIs
- [ ] Load config and apply ICs in `Stage()` using `GetConfig()`
- [ ] Compute derivatives in `Step()` - this is the hot path, no allocations!
- [ ] States ARE outputs (Phase 6 unified signal model)

---

## Component Lifecycle

Components implement a three-phase lifecycle:

| Phase | When Called | Component Must... |
|:------|:------------|:------------------|
| `Provision(bp)` | Once at startup | Register outputs, inputs, states, params, config |
| `Stage(bp)` | Start of each run | Load config, apply ICs, resolve input wiring |
| `Step(t, dt)` | Every timestep | Read inputs, compute derivatives, (hot path!) |

### Extended Lifecycle Hooks (Optional)

| Hook | When Called | Use Case |
|:-----|:------------|:---------|
| `PreStep(t, dt)` | Before all components Step | Pre-processing, predictions |
| `PostStep(t, dt)` | After all components Step | Post-processing, logging |
| `OnError(error)` | When simulation encounters error | Error handling, cleanup |
| `Shutdown()` | At simulation end | Cleanup, flush buffers |

---

## Minimal Component Template

```cpp
#pragma once

#include <icarus/core/Component.hpp>
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/InputHandle.hpp>

namespace icarus::components {

template <typename Scalar>
class MyComponent : public Component<Scalar> {
  public:
    explicit MyComponent(std::string name = "MyComponent", std::string entity = "")
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // Identity
    [[nodiscard]] std::string Name() const override { return name_; }
    [[nodiscard]] std::string Entity() const override { return entity_; }
    [[nodiscard]] std::string TypeName() const override { return "MyComponent"; }

    // Lifecycle
    void Provision(Backplane<Scalar> &bp) override {
        // Register signals here
    }

    void Stage(Backplane<Scalar> &bp) override {
        // Load config and apply ICs here
    }

    void Step(Scalar t, Scalar dt) override {
        // Compute derivatives here (hot path!)
    }

  private:
    std::string name_;
    std::string entity_;
};

} // namespace icarus::components
```

---

## Signal Registration (Provision Phase)

All signal registration happens in `Provision()` using the Backplane API.

### Outputs

```cpp
void Provision(Backplane<Scalar> &bp) override {
    // Scalar output
    bp.template register_output<Scalar>("temperature", &temperature_, "K", "Surface temp");

    // Vec3 output (creates .x/.y/.z)
    bp.template register_output_vec3<Scalar>("force", &force_, "N", "Applied force");

    // Quaternion output (creates .w/.x/.y/.z)
    bp.template register_output_quat<Scalar>("attitude", &quat_, "", "Body attitude");
}
```

### States (Phase 6: Unified Signal Model)

**States ARE outputs.** When you register a state, it automatically creates:
1. The state value as an output signal
2. The derivative signal (with `_dot` suffix)

```cpp
void Provision(Backplane<Scalar> &bp) override {
    // Scalar state: creates "altitude" and "altitude_dot"
    bp.template register_state<Scalar>("altitude", &altitude_, &altitude_dot_, "m", "Altitude");

    // Vec3 state: creates position.x/y/z and position_dot.x/y/z (6 signals)
    bp.template register_state_vec3<Scalar>("position", &position_, &position_dot_, "m", "Position");

    // Quaternion state: creates attitude.w/x/y/z and attitude_dot.w/x/y/z (8 signals)
    bp.template register_state_quat<Scalar>("attitude", &attitude_, &attitude_dot_, "", "Attitude");
}
```

### Inputs

Inputs use `InputHandle<Scalar>` for type-safe, efficient access:

```cpp
class MyComponent : public Component<Scalar> {
    // Declare input handles as members
    InputHandle<Scalar> mass_input_;
    InputHandle<Scalar> force_x_;
    InputHandle<Scalar> force_y_;
    InputHandle<Scalar> force_z_;

    void Provision(Backplane<Scalar> &bp) override {
        bp.template register_input<Scalar>("mass", &mass_input_, "kg", "Vehicle mass");
        bp.template register_input<Scalar>("force.x", &force_x_, "N", "Force X");
        bp.template register_input<Scalar>("force.y", &force_y_, "N", "Force Y");
        bp.template register_input<Scalar>("force.z", &force_z_, "N", "Force Z");
    }

    void Step(Scalar t, Scalar dt) override {
        // Read inputs with .get()
        Scalar m = mass_input_.get();
        Vec3<Scalar> force{force_x_.get(), force_y_.get(), force_z_.get()};
    }
};
```

### Parameters (Optimizable)

Parameters are Scalar values that can be optimized (e.g., by trim solver):

```cpp
void Provision(Backplane<Scalar> &bp) override {
    bp.register_param("mass", &mass_, mass_, "kg", "Point mass");
    // Also expose as output for other components to read
    bp.template register_output<Scalar>("mass", &mass_, "kg", "Point mass");
}
```

### Config Values

Config values are integer settings (enums, flags):

```cpp
void Provision(Backplane<Scalar> &bp) override {
    bp.register_config("model", &model_int_, model_int_,
                       "Gravity model (0=Constant, 1=PointMass)");
}
```

---

## Configuration Loading (Stage Phase)

Stage is called at the start of each run/episode. Use `GetConfig()` to access YAML configuration:

```cpp
void Stage(Backplane<Scalar> &bp) override {
    const auto &config = this->GetConfig();

    // Load scalar with default
    if (config.template Has<double>("mass")) {
        mass_ = static_cast<Scalar>(config.template Get<double>("mass", 1.0));
    }

    // Load Vec3 initial condition
    if (config.template Has<Vec3<double>>("initial_position")) {
        auto pos = config.template Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
        position_ = Vec3<Scalar>{
            static_cast<Scalar>(pos(0)),
            static_cast<Scalar>(pos(1)),
            static_cast<Scalar>(pos(2))
        };
    }

    // Load enum/int
    if (config.template Has<int>("model")) {
        model_int_ = config.template Get<int>("model", 0);
    }

    // Initialize derivatives to zero
    position_dot_ = Vec3<Scalar>::Zero();
    velocity_dot_ = Vec3<Scalar>::Zero();
}
```

---

## Computing Derivatives (Step Phase)

Step is called every integration substep. **This is the hot path!**

### Rules for Step()

1. **No allocations** - all memory should be pre-allocated
2. **No string lookups** - use pre-resolved handles
3. **Read inputs** - use `handle.get()`
4. **Write derivatives** - directly to member variables

```cpp
void Step(Scalar t, Scalar dt) override {
    (void)t;  // Mark unused if not needed
    (void)dt;

    // 1. Read inputs
    Vec3<Scalar> force{force_x_.get(), force_y_.get(), force_z_.get()};
    Scalar m = mass_input_.get();

    // 2. Compute physics using Vulcan
    Vec3<Scalar> accel = vulcan::dynamics::point_mass_acceleration(force, m);

    // 3. Write derivatives directly to member variables
    // Integrator will read these and update state values
    position_dot_ = velocity_;      // dr/dt = v
    velocity_dot_ = accel;          // dv/dt = F/m
}
```

### Using Vulcan for Physics

Always use Vulcan functions for physics computations:

```cpp
#include <vulcan/dynamics/PointMass.hpp>
#include <vulcan/gravity/PointMass.hpp>
#include <vulcan/core/Constants.hpp>

void Step(Scalar t, Scalar dt) override {
    // Gravity: g = -μ/r³ · r
    Vec3<Scalar> g = vulcan::gravity::point_mass::acceleration(position, mu_);

    // Point mass dynamics: a = F/m
    Vec3<Scalar> a = vulcan::dynamics::point_mass_acceleration(force, mass);

    // Standard gravity constant
    Scalar g0 = vulcan::constants::physics::g0;
}
```

---

## Stateful vs Stateless Components

| Type | Has States | StateSize() | Example |
|:-----|:-----------|:------------|:--------|
| Stateful | Yes (integrated) | > 0 | Dynamics, Integrators |
| Stateless | No | 0 | Gravity, Atmosphere, Sensors |

Stateless components compute outputs purely from inputs - they don't own integrated state.

```cpp
// Stateless example: Gravity component
void Step(Scalar t, Scalar dt) override {
    Vec3<Scalar> pos{pos_x_.get(), pos_y_.get(), pos_z_.get()};
    Scalar m = mass_input_.get();

    // Compute acceleration
    accel_ = vulcan::gravity::point_mass::acceleration(pos, mu_);

    // Output force (no state derivatives)
    force_ = accel_ * m;
}
```

---

## Epoch Access

For time-dependent calculations (ephemeris, atmospheric models), use the epoch:

```cpp
void Step(Scalar t, Scalar dt) override {
    const auto* epoch = this->GetEpoch();
    if (epoch) {
        // Get Julian Date (Terrestrial Time) for ephemeris
        Scalar jd_tt = epoch->jd_tt();

        // Use for lunar/solar position, etc.
    }
}
```

---

## Component Registration (Factory)

Register your component with the factory for YAML instantiation:

```cpp
// In MyComponent.cpp or a registration file
#include <icarus/core/ComponentFactory.hpp>

namespace {
    static bool registered = []() {
        icarus::ComponentFactory::Register<icarus::components::MyComponent<double>>(
            "MyComponent"
        );
        return true;
    }();
}
```

Then use in YAML:

```yaml
components:
  - type: MyComponent
    name: MyInstance
    config:
      mass: 1.0
      initial_position: [0.0, 0.0, 100.0]
```

---

## Signal Wiring in YAML

Components are wired via the `routes` section:

```yaml
routes:
  # Gravity reads position from Dynamics
  - input: Gravity.position.x
    output: Dynamics.position.x
  - input: Gravity.position.y
    output: Dynamics.position.y
  - input: Gravity.position.z
    output: Dynamics.position.z

  # Dynamics reads force from Gravity
  - input: Dynamics.force.x
    output: Gravity.force.x
  - input: Dynamics.force.y
    output: Gravity.force.y
  - input: Dynamics.force.z
    output: Gravity.force.z
```

---

## Common Patterns

### Force Aggregation

Multiple force sources should output force (not acceleration), which gets summed:

```cpp
// In gravity component
force_ = accel_ * mass;  // Output force, not acceleration

// In dynamics component
Vec3<Scalar> total_force{force_x_.get(), force_y_.get(), force_z_.get()};
Vec3<Scalar> accel = total_force / mass_;
```

### Dual-Mode Support (Numeric/Symbolic)

Always template on `Scalar` to support both numeric simulation and symbolic differentiation:

```cpp
template <typename Scalar>
class MyComponent : public Component<Scalar> {
    // Use Scalar for all computations, not double
    Scalar mass_{1.0};
    Vec3<Scalar> position_ = Vec3<Scalar>::Zero();
};
```

### Accessor Methods

Provide accessors for programmatic configuration (before Stage):

```cpp
void SetMass(Scalar mass) { mass_ = mass; }
[[nodiscard]] Scalar GetMass() const { return mass_; }

void SetInitialPosition(const Vec3<Scalar> &pos) { position_ = pos; }
[[nodiscard]] Vec3<Scalar> GetPosition() const { return position_; }
```

---

## Debugging Tips

1. **Check signal wiring**: Use data dictionary to verify inputs are connected
2. **Zero derivatives**: Initialize `_dot` variables in Stage()
3. **NaN propagation**: Check for division by zero, especially with mass
4. **Lifecycle order**: Ensure Provision → Stage → Step sequence

---

## See Also

- [PointMass3DOF.hpp](../../components/dynamics/PointMass3DOF.hpp) - Reference stateful component
- [PointMassGravity.hpp](../../components/environment/PointMassGravity.hpp) - Reference stateless component
- [RigidBody6DOF.hpp](../../components/dynamics/RigidBody6DOF.hpp) - Full 6DOF dynamics
