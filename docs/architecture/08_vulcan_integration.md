# Vulcan Integration (Engineering Library)

**Related:** [07_janus_integration.md](07_janus_integration.md) | [01_core_philosophy.md](01_core_philosophy.md) | [12_force_aggregation.md](12_force_aggregation.md)

---

**Vulcan** is the domain-specific engineering library that sits between Janus (Math) and Icarus (Architecture).

---

## 1. Division of Responsibility: "Bricks vs. Houses"

* **Janus:** The **Math** (Matrices, Autodiff, Solvers).
* **Vulcan:** The **Physics Utilities** (Stateless). Pure functions for dynamics, atmospherics, gravity. No internal state.
* **Icarus:** The **Systems & State** (Stateful). Owns Component lifecycle, state vectors, and signal wiring.

| Feature | Vulcan (Utility) | Icarus (Component) |
| :--- | :--- | :--- |
| **Equations of Motion** | Provides `compute_6dof_derivatives()`, `RigidBodyState`, `MassProperties`. Pure physics math for $\dot{v} = F/m - \omega \times v$ and Euler's equations. | **OWNS** the `RigidBody6DOF` Component. Manages state vectors, signal I/O, calls Vulcan for derivatives. |
| **Aerodynamics** | Provides `calc_dynamic_pressure()`, `calc_mach()`, `aero_angles()`. | **OWNS** the `AeroMap` component. Lookups $C_L$ tables, applies dynamics. |
| **Propulsion** | Provides `thrust_from_mdot()`, `delta_v()`, `altitude_thrust()`. | **OWNS** the `JetEngine` component. Models spool-up lag, fuel consumption state. |
| **Atmosphere** | Provides `usa76()`, `exponential()`, wind models. | **OWNS** `EnvironmentComponent`. Caches values, manages frame transforms. |
| **Gravity** | Provides `point_mass()`, `j2()`, `spherical_harmonics()`. | **USES** gravity models, applies to force accumulator. |

> [!IMPORTANT]
> **Why this split matters for Trajectory Optimization:**
> By keeping the physics math in Vulcan as pure functions, you can use the *exact same dynamics code* in both:
> 1. `Simulator<double>` for numeric simulation (real-time, Monte Carlo)
> 2. `janus::Opti<MX>` for symbolic optimization (direct collocation, MPC)
>
> No need to maintain separate "low-fidelity" models for optimization.

---

## 2. Implementation Strategy

Icarus components **compose** Vulcan utilities.

### Example: Jet Engine Component

A `JetEngine` is a **System** (Icarus). It has state (Spool Speed $\omega$).
It uses **Functions** from Vulcan (Atmosphere, thermodynamics).

```cpp
template <typename Scalar>
void JetEngine::Step(Scalar t, Scalar dt) {
    // 1. Get Inputs
    Scalar h = *input_alt_;
    Scalar throttle = *input_throttle_;

    // 2. Use Vulcan to get environmental context (Functional)
    auto env = vulcan::atmosphere::usa76(h);

    // 3. Compute Derivative (System Logic)
    // Vulcan doesn't know about "spool up time", Icarus does.
    Scalar current_thrust = *state_thrust_;  // Read current state
    Scalar target_thrust = throttle * max_thrust_ * (env.density / rho0);
    Scalar thrust_rate = (target_thrust - current_thrust) / time_constant_;

    // Write derivative - integrator handles the actual integration
    *state_dot_thrust_ = thrust_rate;

    // 4. Publish output (current value, not integrated)
    *output_thrust_ = current_thrust;
}
```

> [!NOTE]
> **Components write derivatives, not integrated values.** The Simulator's integrator (see [09_memory_state_ownership.md](09_memory_state_ownership.md)) handles `state += state_dot * dt`. This ensures solver compatibility and global error control.

---

## 3. Vulcan Namespace Structure

```cpp
namespace vulcan {
    namespace atmosphere {
        template <typename Scalar>
        AtmosphereState usa76(Scalar altitude);

        template <typename Scalar>
        AtmosphereState exponential(Scalar altitude, Scalar scale_height);
    }

    namespace gravity {
        template <typename Scalar>
        Vec3<Scalar> point_mass(Vec3<Scalar> position, Scalar mu);

        template <typename Scalar>
        Vec3<Scalar> j2(Vec3<Scalar> position, Scalar mu, Scalar J2, Scalar Re);
    }

    namespace aero {
        template <typename Scalar>
        Scalar dynamic_pressure(Scalar density, Scalar velocity);

        template <typename Scalar>
        Scalar mach_number(Scalar velocity, Scalar speed_of_sound);
    }

    namespace coordinates {
        template <typename Scalar>
        Mat3<Scalar> dcm_body_to_wind(Scalar alpha, Scalar beta);
    }
}
```

---

## 4. Stateless Guarantee

> [!IMPORTANT]
> **Vulcan functions are pure.** They have no internal state, no side effects. Given the same inputs, they always produce the same outputs.
>
> This is critical for:
> - Symbolic tracing (CasADi needs deterministic function graphs)
> - Parallel evaluation (no shared mutable state)
> - Testing (no setup/teardown)
