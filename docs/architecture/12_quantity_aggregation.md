# Quantity Aggregation Pattern

**Related:** [02_component_protocol.md](02_component_protocol.md) | [09_memory_state_ownership.md](09_memory_state_ownership.md) | [08_vulcan_integration.md](08_vulcan_integration.md)

---

In a 6DOF simulation, the Equations of Motion (EOM) require **total force**, **total moment**, **total mass**, **CG position**, and **inertia tensor**. Multiple components contribute to these quantities. This document defines the **registration and aggregation pattern** for force and mass.

---

## 1. What Needs Global Aggregation

| Quantity | Why Global | Consumer |
|:---------|:-----------|:---------|
| **Forces** | EOM needs net force on vehicle | EOM |
| **Moments** | EOM needs net moment about CG | EOM |
| **Mass** | EOM needs total mass for F=ma | EOM, ForceAggregator |
| **CG Position** | ForceAggregator needs CG for moment transfer | ForceAggregator |
| **Inertia Tensor** | EOM needs I for rotational dynamics | EOM |

### What Doesn't Need Global Aggregation

| Quantity | Why Not | Better Approach |
|:---------|:--------|:----------------|
| **Power** | Components have specific power relationships | Explicit wiring between sources/loads |
| **Thermal** | Heat transfer is between specific components | Component-level thermal models |
| **Propellant** | Engines draw from specific tanks | Explicit tank→engine wiring |

---

## 2. Core Principle: Registration Protocol

Components **opt-in** to aggregation by registering as sources during Provision:

- **Explicit**: Components choose to participate
- **Discoverable**: Aggregators find sources automatically
- **Traceable**: All data flows through Backplane signals

```cpp
template <typename Scalar>
class Backplane {
    std::vector<ForceSourceReg<Scalar>> force_sources_;
    std::vector<MassSourceReg<Scalar>> mass_sources_;

public:
    void register_force_source(ForceSourceReg<Scalar> reg);
    void register_mass_source(MassSourceReg<Scalar> reg);

    // Aggregators query these by entity prefix
    auto get_force_sources(const std::string& entity) const;
    auto get_mass_sources(const std::string& entity) const;
};
```

---

## 3. Force/Moment Aggregation

### 3.1 The Problem

- **Aero** computes lift/drag in wind frame
- **Propulsion** computes thrust in nozzle-local frame
- **Gravity** is in inertial frame
- **Control Surfaces** produce forces at hinge points

All must become body-frame forces/moments about the CG.

### 3.2 Force Source Registration

```cpp
template <typename Scalar>
struct ForceSourceReg {
    std::string name;                    // "MainEngine.thrust"

    // Pointers to component's internal storage
    Vec3<Scalar>* force;                 // Force vector
    Vec3<Scalar>* moment;                // Moment about application point (optional)

    // Frame specification
    Frame frame;                         // BODY, WIND, LOCAL, INERTIAL
    Vec3<Scalar>* application_point;     // Position in body frame
    Mat3<Scalar>* dcm_to_body;           // Rotation to body (required if LOCAL frame)

    // Entity activation (for lifecycle gating)
    Scalar* entity_active;               // nullptr if always active
};
```

### 3.3 Registration Examples

```cpp
// Propulsion: thrust is a force in LOCAL (nozzle) frame
template <typename Scalar>
void RocketEngine<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    bp.register_output("thrust", &thrust_mag_, {.units = "N"});
    bp.register_output("fuel_flow", &fuel_flow_, {.units = "kg/s"});

    bp.register_force_source({
        .name = full_name_ + ".thrust",
        .force = &thrust_force_,         // [T, 0, 0] in nozzle frame
        .moment = &thrust_moment_,       // Non-zero if gimbaled off-axis
        .frame = Frame::LOCAL,
        .application_point = &nozzle_position_,
        .dcm_to_body = &nozzle_dcm_,
        .entity_active = entity_active_ptr_
    });
}

// Aerodynamics: lift/drag in WIND frame
template <typename Scalar>
void Aerodynamics<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    bp.register_output("lift", &lift_, {.units = "N"});
    bp.register_output("drag", &drag_, {.units = "N"});

    bp.register_force_source({
        .name = full_name_ + ".aero",
        .force = &aero_force_,           // [-D, 0, -L] in wind frame
        .moment = &aero_moment_,         // Pitching moment, etc.
        .frame = Frame::WIND,
        .application_point = &aero_ref_point_,
        .dcm_to_body = nullptr,          // Aggregator gets DCM from Nav
        .entity_active = entity_active_ptr_
    });
}

// Gravity: weight already transformed to BODY frame
template <typename Scalar>
void GravityModel<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    bp.register_force_source({
        .name = full_name_ + ".weight",
        .force = &weight_body_,
        .moment = nullptr,               // Acts at CG, no moment arm
        .frame = Frame::BODY,
        .application_point = nullptr,    // Applied at CG (no moment transfer)
        .dcm_to_body = nullptr,
        .entity_active = entity_active_ptr_
    });
}
```

### 3.4 Force Aggregator Implementation

```cpp
template <typename Scalar>
class ForceAggregator : public Component<Scalar> {
    std::vector<ForceSourceReg<Scalar>> sources_;
    SignalHandle<Mat3<Scalar>> wind_to_body_dcm_;
    SignalHandle<Mat3<Scalar>> inertial_to_body_dcm_;
    SignalHandle<Vec3<Scalar>> cg_position_;

public:
    void Stage(Backplane<Scalar>& bp, const RunConfig& rc) override {
        // Auto-discover force sources for this entity
        sources_ = bp.get_force_sources(entity_prefix_);

        // Wire frame conversion signals
        wind_to_body_dcm_ = bp.resolve<Mat3<Scalar>>(entity_ + ".Nav.dcm_wind_body");
        inertial_to_body_dcm_ = bp.resolve<Mat3<Scalar>>(entity_ + ".Nav.dcm_inertial_body");
        cg_position_ = bp.resolve<Vec3<Scalar>>(entity_ + ".Mass.cg");

        ICARUS_INFO("ForceAggregator: {} sources for '{}'", sources_.size(), entity_prefix_);
    }

    void Step(Scalar t, Scalar dt) override {
        Vec3<Scalar> F_total = {0, 0, 0};
        Vec3<Scalar> M_total = {0, 0, 0};

        for (const auto& src : sources_) {
            Scalar active = src.entity_active ? *src.entity_active : Scalar(1);

            // Transform force to body frame
            Vec3<Scalar> F_body = TransformToBody(*src.force, src);

            // Transform moment to body frame
            Vec3<Scalar> M_body = src.moment ? TransformToBody(*src.moment, src)
                                             : Vec3<Scalar>{0, 0, 0};

            // Moment transfer to CG: M_cg = M_app + r × F
            if (src.application_point) {
                Vec3<Scalar> r = *src.application_point - *cg_position_;
                M_body += vulcan::cross(r, F_body);
            }

            F_total += F_body * active;
            M_total += M_body * active;
        }

        *output_total_force_ = F_total;
        *output_total_moment_ = M_total;
    }

private:
    Vec3<Scalar> TransformToBody(const Vec3<Scalar>& v, const ForceSourceReg<Scalar>& src) {
        switch (src.frame) {
            case Frame::BODY:     return v;
            case Frame::WIND:     return (*wind_to_body_dcm_) * v;
            case Frame::INERTIAL: return (*inertial_to_body_dcm_) * v;
            case Frame::LOCAL:    return (*src.dcm_to_body) * v;
        }
    }
};
```

---

## 4. Mass/Inertia Aggregation

### 4.1 Mass Source Registration

```cpp
template <typename Scalar>
struct MassSourceReg {
    std::string name;                    // "FuelTank.fuel"

    Scalar* mass;                        // Mass value (can be dynamic)
    Vec3<Scalar>* cg_body;               // CG position in body frame
    Mat3<Scalar>* inertia_cg;            // Inertia about source CG (optional)

    Lifecycle lifecycle;                 // STATIC or DYNAMIC
    Scalar* entity_active;               // For lifecycle gating
};
```

### 4.2 Registration Examples

```cpp
// Static mass (structure, dry mass)
template <typename Scalar>
void Structure<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    dry_mass_ = cfg.get<double>("dry_mass");
    dry_cg_ = cfg.get<Vec3<double>>("dry_cg");
    dry_inertia_ = cfg.get<Mat3<double>>("dry_inertia");

    bp.register_mass_source({
        .name = full_name_ + ".dry",
        .mass = &dry_mass_,
        .cg_body = &dry_cg_,
        .inertia_cg = &dry_inertia_,
        .lifecycle = Lifecycle::STATIC,
        .entity_active = entity_active_ptr_
    });
}

// Dynamic mass (fuel tank)
template <typename Scalar>
void FuelTank<Scalar>::Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    bp.register_output("fuel_mass", &fuel_mass_, {.units = "kg"});

    bp.register_mass_source({
        .name = full_name_ + ".fuel",
        .mass = &fuel_mass_,
        .cg_body = &fuel_cg_,              // May shift as fuel depletes
        .inertia_cg = nullptr,             // Often negligible for liquid
        .lifecycle = Lifecycle::DYNAMIC,
        .entity_active = entity_active_ptr_
    });
}
```

### 4.3 Mass Aggregator Implementation

```cpp
template <typename Scalar>
class MassAggregator : public Component<Scalar> {
    std::vector<MassSourceReg<Scalar>> sources_;

public:
    void Stage(Backplane<Scalar>& bp, const RunConfig& rc) override {
        sources_ = bp.get_mass_sources(entity_prefix_);
        ICARUS_INFO("MassAggregator: {} sources", sources_.size());
    }

    void Step(Scalar t, Scalar dt) override {
        Scalar total_mass = 0;
        Vec3<Scalar> mass_moment = {0, 0, 0};

        // First pass: total mass and mass-weighted CG
        for (const auto& src : sources_) {
            Scalar active = src.entity_active ? *src.entity_active : Scalar(1);
            Scalar m = (*src.mass) * active;

            total_mass += m;
            mass_moment += m * (*src.cg_body);
        }

        Vec3<Scalar> cg = mass_moment / total_mass;

        // Second pass: inertia about composite CG (parallel axis theorem)
        Mat3<Scalar> total_inertia = Mat3<Scalar>::Zero();

        for (const auto& src : sources_) {
            Scalar active = src.entity_active ? *src.entity_active : Scalar(1);
            Scalar m = (*src.mass) * active;

            // Inertia about source CG
            Mat3<Scalar> I_src = src.inertia_cg ? *src.inertia_cg : Mat3<Scalar>::Zero();

            // Parallel axis: I_cg = I_local + m * (|r|² I - r ⊗ r)
            Vec3<Scalar> r = *src.cg_body - cg;
            Scalar r_sq = vulcan::dot(r, r);
            Mat3<Scalar> I_transfer = m * (r_sq * Mat3<Scalar>::Identity() - vulcan::outer(r, r));

            total_inertia += (I_src + I_transfer) * active;
        }

        *output_total_mass_ = total_mass;
        *output_cg_ = cg;
        *output_inertia_ = total_inertia;
    }
};
```

---

## 5. Execution Order

Aggregators must run **after** sources, **before** EOM:

```
1. [Environment]      → Atmosphere, Gravity models
2. [Force Sources]    → Aero, Propulsion (compute forces)
3. [Mass Sources]     → FuelTanks (update mass after consumption)
4. [MassAggregator]   → Compute total mass, CG, inertia
5. [ForceAggregator]  → Transform and sum forces (needs CG)
6. [EOM]              → Integrate dynamics
```

---

## 6. Components Without Aggregated Quantities

Components that don't produce forces or have mass simply don't register:

| Component | Force | Mass |
|:----------|:-----:|:----:|
| `RocketEngine` | ✓ | ✓ |
| `JetEngine` | ✓ | ✓ |
| `Aerodynamics` | ✓ | ✗ |
| `GravityModel` | ✓ | ✗ |
| `FuelTank` | ✗ | ✓ |
| `Structure` | ✗ | ✓ |
| `Payload` | ✗ | ✓ |
| `Autopilot` | ✗ | ✗ |
| `IMU` | ✗ | ✗ |
| `Telemetry` | ✗ | ✗ |

---

## 7. Benefits of Registration Pattern

| Aspect | Old Hierarchical | Registration Pattern |
|:-------|:-----------------|:--------------------|
| **Discovery** | Implicit tree walk | Explicit registry query |
| **Opt-in** | Forced on all | Components choose |
| **Traceability** | Hidden in hierarchy | Signals on Backplane |
| **Frame handling** | Often implicit | Explicit in metadata |
| **Symbolic mode** | Breaks with vtable | Clean signal flow |
| **Testing** | Mock whole hierarchy | Mock Backplane only |

---

## 8. Frame Reference

| Frame | Description | Used By |
|:------|:------------|:--------|
| **BODY** | Vehicle-fixed, origin at reference point | EOM, most outputs |
| **WIND** | Velocity-aligned (X=airspeed direction) | Aerodynamics |
| **LOCAL** | Component-fixed (e.g., nozzle axis) | Thrusters, gimbals |
| **INERTIAL** | ECI or ECEF | Gravity, navigation |
