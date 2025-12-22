# Entity Lifecycle: Birth, Death, and Separation

**Related:** [06_entities_namespaces.md](06_entities_namespaces.md) | [17_events_phases.md](17_events_phases.md) | [09_memory_state_ownership.md](09_memory_state_ownership.md)

---

While entities are virtual namespaces (see [06_entities_namespaces.md](06_entities_namespaces.md)), real aerospace scenarios require dynamic entity lifecycle management:

- **Stage separation**: One entity becomes two
- **Payload deployment**: Dormant entity becomes active
- **Docking**: Two entities become one (shared state)
- **Debris/destruction**: Entity ceases to exist
- **Expendables**: Boosters that burn up on reentry

This document defines how Icarus handles entity birth, death, and state inheritance while maintaining symbolic compatibility.

---

## 1. Core Principle: Existence as State

> [!IMPORTANT]
> **Entities are never dynamically created or destroyed.** All possible entities are declared at Provision time. "Birth" and "death" are state transitions, not structural changes.

This preserves the fixed computational graph required by CasADi symbolic tracing.

```yaml
# All entities declared in scenario - max entities inferred from config
entities:
  - name: Falcon9.Stage1
    initial_state: ACTIVE

  - name: Falcon9.Stage2
    initial_state: ACTIVE

  - name: Falcon9.Fairing.Left
    initial_state: ACTIVE

  - name: Falcon9.Fairing.Right
    initial_state: ACTIVE

  - name: Payload.Satellite
    initial_state: DORMANT    # Born at deployment

  - name: Debris.FairingLeft
    initial_state: DORMANT    # Born if fairing tracked post-separation
```

---

## 2. Entity Activation Signal

Every entity has a reserved `active` signal:

```yaml
# Auto-generated signals for each entity
Falcon9.Stage1.active: 1      # int32: 0=dead, 1=alive
Falcon9.Stage2.active: 1
Payload.Satellite.active: 0   # Dormant until deployment
```

### Activation States

| State | `active` Value | Behavior |
|:------|:---------------|:---------|
| **ACTIVE** | 1 | Normal execution, contributes to sim |
| **DORMANT** | 0 | Exists but inactive, awaiting birth |
| **DEAD** | 0 | Permanently inactive, was once alive |

> [!NOTE]
> DORMANT and DEAD have the same `active=0` value. The distinction is semantic—DORMANT entities can be born, DEAD entities cannot. This is enforced by the lifecycle rules, not the signal value.

---

## 3. Output Gating by Activation

All entity outputs are gated by the activation signal:

```cpp
template <typename Scalar>
class RigidBody6DOF : public Component<Scalar> {
    SignalHandle<Scalar> entity_active_;  // Wired to Entity.active

    void Step(Scalar t, Scalar dt) override {
        Scalar active = *entity_active_;

        // Compute dynamics normally
        Vec3<Scalar> force_raw = ComputeTotalForce();
        Vec3<Scalar> accel_raw = force_raw / *mass_;

        // Gate all outputs by activation
        *output_force_ = force_raw * active;
        *output_acceleration_ = accel_raw * active;

        // State derivatives gated - state frozen when inactive
        *state_dot_velocity_ = accel_raw * active;
        *state_dot_position_ = *state_velocity_ * active;

        // Angular dynamics also gated
        *state_dot_omega_ = ComputeAngularAccel() * active;
        *state_dot_quaternion_ = ComputeQuaternionRate() * active;
    }
};
```

### Force/Mass Aggregation

Inactive entities contribute zero to aggregators:

```cpp
template <typename Scalar>
class ForceAggregator : public Component<Scalar> {
    void Step(Scalar t, Scalar dt) override {
        Vec3<Scalar> F_total = {0, 0, 0};

        for (auto& src : force_sources_) {
            // Entity activation gates contribution
            Scalar active = *src.entity_active;
            F_total += (*src.force) * active;
        }

        *output_total_force_ = F_total;
    }
};

template <typename Scalar>
class MassAggregator : public Component<Scalar> {
    void Step(Scalar t, Scalar dt) override {
        Scalar total_mass = 0;

        for (auto& src : mass_sources_) {
            Scalar active = *src.entity_active;
            total_mass += (*src.mass) * active;
        }

        *output_total_mass_ = total_mass;
    }
};
```

---

## 4. Lifecycle Events

Entity lifecycle is driven by **physical conditions**, not explicit commands.

### 4.1 Lifecycle Configuration

```yaml
lifecycle:
  # Birth conditions - when dormant entities become active
  birth:
    - entity: Payload.Satellite
      condition: "Falcon9.Stage2.deployment_sequence == 1"
      inherit_state_from: Falcon9.Stage2
      state_offset:
        position: [0, 0, -2.0]  # Deployment direction
        velocity: [0, 0, -0.5]  # Separation velocity

    - entity: Debris.FairingLeft
      condition: "Falcon9.Fairing.Left.separated == 1"
      inherit_state_from: Falcon9.Fairing.Left

  # Death conditions - when active entities become permanently inactive
  death:
    - entity: Falcon9.Fairing.Left
      condition: "Falcon9.Fairing.Left.altitude < 50000"
      reason: "%.0fm - below tracking threshold"

    - entity: Falcon9.Stage1
      condition: "Falcon9.Stage1.altitude < 0"
      reason: "%.0fm - impact"

  # Separation events - one entity becomes two
  separation:
    - event: STAGE_SEPARATION
      condition: "Falcon9.Stage1.fuel_mass < 0.01"
      parent: Falcon9.Combined      # Virtual pre-separation entity
      children: [Falcon9.Stage1, Falcon9.Stage2]
      separation_impulse:
        Stage1: [0, 0, -10]   # m/s delta-v
        Stage2: [0, 0, 10]
```

### 4.2 Lifecycle Manager Component

```cpp
template <typename Scalar>
class EntityLifecycleManager : public Component<Scalar> {
    struct BirthRule {
        std::string entity_name;
        SignalHandle<Scalar> condition_signal;
        std::string inherit_from;
        StateOffset offset;
    };

    struct DeathRule {
        std::string entity_name;
        SignalHandle<Scalar> condition_signal;
        SignalHandle<Scalar> active_signal;
    };

    std::vector<BirthRule> birth_rules_;
    std::vector<DeathRule> death_rules_;

public:
    void Step(Scalar t, Scalar dt) override {
        // Process births
        for (auto& rule : birth_rules_) {
            Scalar should_birth = *rule.condition_signal;
            Scalar currently_active = *rule.active_signal;
            Scalar was_ever_born = *rule.born_flag;

            // Birth: condition met AND not currently active AND never born before
            Scalar do_birth = janus::where(
                (should_birth > 0.5) & (currently_active < 0.5) & (was_ever_born < 0.5),
                Scalar(1),
                Scalar(0)
            );

            // Set active
            *rule.active_signal = janus::where(
                do_birth > 0.5,
                Scalar(1),
                currently_active
            );

            // Copy state from parent if birthing
            if constexpr (std::is_same_v<Scalar, double>) {
                if (do_birth > 0.5) {
                    CopyStateWithOffset(rule.inherit_from, rule.entity_name, rule.offset);
                    *rule.born_flag = 1.0;
                    ICARUS_INFO("Entity '{}' born from '{}' at t={:.3f}",
                        rule.entity_name, rule.inherit_from, t);
                }
            } else {
                // Symbolic: state copy is a conditional assignment
                ApplySymbolicStateCopy(rule, do_birth);
            }
        }

        // Process deaths
        for (auto& rule : death_rules_) {
            Scalar should_die = *rule.condition_signal;
            Scalar currently_active = *rule.active_signal;

            // Death: condition met AND currently active
            *rule.active_signal = janus::where(
                (should_die > 0.5) & (currently_active > 0.5),
                Scalar(0),
                currently_active
            );
        }
    }

private:
    void CopyStateWithOffset(const std::string& from, const std::string& to,
                             const StateOffset& offset) {
        // Copy position with offset
        auto from_pos = backplane_.resolve<double>(from + ".EOM.position");
        auto to_pos = backplane_.resolve<double>(to + ".EOM.position");
        *to_pos = *from_pos + offset.position;

        // Copy velocity with delta-v
        auto from_vel = backplane_.resolve<double>(from + ".EOM.velocity");
        auto to_vel = backplane_.resolve<double>(to + ".EOM.velocity");
        *to_vel = *from_vel + offset.velocity;

        // Copy attitude (typically unchanged)
        auto from_quat = backplane_.resolve<double>(from + ".EOM.quaternion");
        auto to_quat = backplane_.resolve<double>(to + ".EOM.quaternion");
        *to_quat = *from_quat;

        // Copy angular velocity
        auto from_omega = backplane_.resolve<double>(from + ".EOM.omega");
        auto to_omega = backplane_.resolve<double>(to + ".EOM.omega");
        *to_omega = *from_omega + offset.omega;
    }
};
```

---

## 5. Stage Separation Pattern

Stage separation is a special case where a combined entity "splits" into independent entities.

### 5.1 Pre-Separation: Rigid Attachment

Before separation, Stage2 is rigidly attached to Stage1. Two modeling approaches:

**Approach A: Combined Entity (Recommended)**

Model the pre-separation stack as a single entity with combined mass properties:

```yaml
entities:
  - name: Falcon9.Stack
    initial_state: ACTIVE
    components:
      - type: RigidBody6DOF
        name: EOM
      - type: CombinedMassProperties
        name: Mass
        config:
          sources: [Stage1.Mass, Stage2.Mass, Fairing.Mass, Payload.Mass]

  - name: Falcon9.Stage1
    initial_state: DORMANT    # Born at separation

  - name: Falcon9.Stage2
    initial_state: DORMANT    # Born at separation
```

**Approach B: Constraint Coupling**

Model stages as separate entities with a rigid constraint:

```cpp
template <typename Scalar>
class RigidAttachment : public Component<Scalar> {
    void Step(Scalar t, Scalar dt) override {
        Scalar attached = *attachment_active_;

        // When attached, child follows parent with fixed offset
        Vec3<Scalar> parent_pos = *parent_position_;
        Vec3<Scalar> parent_vel = *parent_velocity_;
        Quat<Scalar> parent_att = *parent_attitude_;

        // Child position in parent body frame
        Vec3<Scalar> child_pos_world = parent_pos +
            vulcan::quat_rotate(parent_att, attachment_offset_);

        // Blend between constrained and free based on attachment
        *child_position_ = janus::where(attached > 0.5,
            child_pos_world,
            *child_position_free_);
    }
};
```

### 5.2 Separation Event

```yaml
lifecycle:
  separation:
    - event: MECO  # Main Engine Cut-Off triggers separation
      condition: "Falcon9.Stack.Stage1.fuel_mass < 100"

      # Deactivate combined entity
      deactivate: Falcon9.Stack

      # Activate independent entities
      activate:
        - entity: Falcon9.Stage1
          inherit_state_from: Falcon9.Stack
          state_transform:
            # Stage1 gets the stack state directly
            position: identity
            velocity: [0, 0, -5]  # Separation delta-v (body frame)

        - entity: Falcon9.Stage2
          inherit_state_from: Falcon9.Stack
          state_transform:
            # Stage2 offset by interstage length
            position_offset_body: [0, 0, 15.0]  # meters forward
            velocity: [0, 0, 5]  # Separation delta-v (body frame)
```

### 5.3 Implementation

```cpp
template <typename Scalar>
class SeparationManager : public Component<Scalar> {
    void Step(Scalar t, Scalar dt) override {
        for (auto& sep : separations_) {
            Scalar condition = *sep.condition_signal;
            Scalar already_separated = *sep.separated_flag;

            Scalar do_separate = janus::where(
                (condition > 0.5) & (already_separated < 0.5),
                Scalar(1),
                Scalar(0)
            );

            // Deactivate parent
            *sep.parent_active = janus::where(
                do_separate > 0.5,
                Scalar(0),
                *sep.parent_active
            );

            // Activate and initialize children
            for (auto& child : sep.children) {
                *child.active_signal = janus::where(
                    do_separate > 0.5,
                    Scalar(1),
                    *child.active_signal
                );

                // State inheritance with separation impulse
                ApplyStateInheritance(sep.parent, child, do_separate);
            }

            // Mark as separated (one-shot)
            *sep.separated_flag = janus::where(
                do_separate > 0.5,
                Scalar(1),
                already_separated
            );
        }
    }

    void ApplyStateInheritance(const Entity& parent, const ChildSpec& child,
                               Scalar do_separate) {
        // Position: parent position + offset rotated to world frame
        Vec3<Scalar> parent_pos = GetEntityPosition(parent);
        Quat<Scalar> parent_att = GetEntityAttitude(parent);
        Vec3<Scalar> offset_world = vulcan::quat_rotate(parent_att, child.offset_body);

        Vec3<Scalar> new_pos = parent_pos + offset_world;
        Vec3<Scalar> current_pos = GetEntityPosition(child.entity);

        SetEntityPosition(child.entity, janus::where(
            do_separate > 0.5,
            new_pos,
            current_pos
        ));

        // Velocity: parent velocity + separation delta-v in body frame
        Vec3<Scalar> parent_vel = GetEntityVelocity(parent);
        Vec3<Scalar> delta_v_world = vulcan::quat_rotate(parent_att, child.delta_v_body);

        Vec3<Scalar> new_vel = parent_vel + delta_v_world;
        Vec3<Scalar> current_vel = GetEntityVelocity(child.entity);

        SetEntityVelocity(child.entity, janus::where(
            do_separate > 0.5,
            new_vel,
            current_vel
        ));

        // Attitude: inherit parent attitude
        Quat<Scalar> current_att = GetEntityAttitude(child.entity);
        SetEntityAttitude(child.entity, janus::where(
            do_separate > 0.5,
            parent_att,
            current_att
        ));
    }
};
```

---

## 6. Collision Visibility

> [!IMPORTANT]
> **Dead/dormant entities are invisible to collision detection.** This simplifies implementation and avoids spurious collisions with "ghost" entities.

```cpp
template <typename Scalar>
class CollisionDetector : public Component<Scalar> {
    void Step(Scalar t, Scalar dt) override {
        for (size_t i = 0; i < entities_.size(); ++i) {
            // Skip inactive entities
            if (*entities_[i].active < 0.5) continue;

            for (size_t j = i + 1; j < entities_.size(); ++j) {
                // Skip inactive entities
                if (*entities_[j].active < 0.5) continue;

                // Check collision between active entities
                CheckCollision(entities_[i], entities_[j]);
            }
        }
    }
};
```

For symbolic mode, collision checks are gated:

```cpp
Scalar CheckCollisionSymbolic(const Entity& a, const Entity& b) {
    Scalar both_active = (*a.active) * (*b.active);
    Scalar distance = ComputeDistance(a, b);
    Scalar collision_raw = janus::where(distance < threshold_, Scalar(1), Scalar(0));

    // Only report collision if both entities are active
    return collision_raw * both_active;
}
```

---

## 7. State Vector Layout

Inactive entities still occupy space in the global state vector, but their derivatives are zero:

```
X_global_ layout:
┌─────────────────────────────────────────────────────────────────────────────┐
│ Stack (13 states) │ Stage1 (13 states) │ Stage2 (13 states) │ Payload (13) │
│ [ACTIVE]          │ [DORMANT→ACTIVE]   │ [DORMANT→ACTIVE]   │ [DORMANT]    │
└─────────────────────────────────────────────────────────────────────────────┘

Pre-separation:
  Stack.state_dot = f(Stack.state)     # Normal dynamics
  Stage1.state_dot = 0                  # Frozen (dormant)
  Stage2.state_dot = 0                  # Frozen (dormant)

Post-separation:
  Stack.state_dot = 0                   # Frozen (dead)
  Stage1.state_dot = f(Stage1.state)   # Normal dynamics
  Stage2.state_dot = f(Stage2.state)   # Normal dynamics
```

---

## 8. Optimization Considerations

### 8.1 Entity Activation as Optimization Variable (Advanced)

While entity lifecycle is primarily driven by physical conditions, activation *could* be exposed as an optimization state for problems like optimal staging:

```cpp
// In trajectory optimization setup
opti.subject_to(stage1_active[k+1] <= stage1_active[k]);  // Monotonic: can only deactivate
opti.subject_to(stage2_active[0] == 0);                    // Stage2 starts dormant
opti.subject_to(stage2_active[N] == 1);                    // Stage2 must be active at end

// Let optimizer choose separation time
// The activation signal becomes a decision variable
```

> [!WARNING]
> **Complexity warning:** Treating activation as an optimization variable introduces integer/binary constraints (active ∈ {0,1}) which makes the problem mixed-integer. Relaxation to continuous [0,1] with penalty terms is possible but requires careful formulation.

### 8.2 Recommended Approach: Phase-Based Optimization

For most trajectory optimization, use fixed phase sequences with known entity sets per phase:

```cpp
// Phase 1: Boost (Stack active)
for (int k = 0; k < k_separation; ++k) {
    x_next = dynamics_stack(x[k], u[k], dt);
    opti.subject_to(x[k+1] == x_next);
}

// Separation event
auto [x_stage1, x_stage2] = separation_dynamics(x[k_separation], separation_impulse);

// Phase 2: Coast (Stage1, Stage2 active independently)
for (int k = k_separation; k < N; ++k) {
    x1_next = dynamics_stage1(x1[k], u1[k], dt);
    x2_next = dynamics_stage2(x2[k], u2[k], dt);
    opti.subject_to(x1[k+1] == x1_next);
    opti.subject_to(x2[k+1] == x2_next);
}

// Optimize separation time
opti.minimize(cost + penalty * k_separation);  // If separation time is free
```

---

## 9. Configuration Summary

### 9.1 Entity Declaration

```yaml
entities:
  - name: Falcon9.Stack
    initial_state: ACTIVE
    type: rigid_body_6dof

  - name: Falcon9.Stage1
    initial_state: DORMANT
    type: rigid_body_6dof
    born_from: Falcon9.Stack

  - name: Falcon9.Stage2
    initial_state: DORMANT
    type: rigid_body_6dof
    born_from: Falcon9.Stack

  - name: Payload.Satellite
    initial_state: DORMANT
    type: rigid_body_6dof
    born_from: Falcon9.Stage2
```

### 9.2 Lifecycle Rules

```yaml
lifecycle:
  separation:
    - name: STAGE_SEPARATION
      condition: "Falcon9.Stack.Stage1Engine.fuel_mass < 100"
      parent: Falcon9.Stack
      children:
        - entity: Falcon9.Stage1
          delta_v_body: [0, 0, -5]
        - entity: Falcon9.Stage2
          position_offset_body: [0, 0, 15]
          delta_v_body: [0, 0, 5]

  birth:
    - entity: Payload.Satellite
      condition: "Falcon9.Stage2.deploy_command == 1"
      inherit_from: Falcon9.Stage2
      delta_v_body: [0, 0, -1]

  death:
    - entity: Falcon9.Stage1
      condition: "Falcon9.Stage1.altitude < 0"
```

---

## 10. Implementation Checklist

- [ ] Add `active` signal to all entities automatically
- [ ] Gate all component outputs by entity activation
- [ ] Implement `EntityLifecycleManager` component
- [ ] Implement `SeparationManager` component
- [ ] Add state inheritance with offset/delta-v
- [ ] Gate collision detection by activation
- [ ] Update `ForceAggregator` and `MassAggregator` for activation gating
- [ ] Add lifecycle configuration schema
- [ ] Test symbolic tracing with entity transitions
- [ ] Document interaction with phase system ([17_events_phases.md](17_events_phases.md))
