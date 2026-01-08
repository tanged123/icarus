# Config API Improvement: `read_param` Helper Methods

Streamline component configuration from verbose `if/Has/Get` blocks to one-liners.

## Problem

Current `Stage()` pattern is verbose and error-prone:

```cpp
// Scalar: redundant Has/Get dance
if (config.template Has<double>("dry_mass")) {
    dry_mass_ = config.template Get<double>("dry_mass", 50.0);
}

// Vec3: 4 lines with manual Scalar conversion
if (config.template Has<Vec3<double>>("tank_cg")) {
    auto pos = config.template Get<Vec3<double>>("tank_cg", Vec3<double>::Zero());
    cg_ = Vec3<Scalar>{static_cast<Scalar>(pos(0)), static_cast<Scalar>(pos(1)),
                       static_cast<Scalar>(pos(2))};
}

// Vec4/Quat: 6 lines (RigidBody6DOF)
if (config.template Has<Vec4<double>>("initial_attitude")) {
    Vec4<double> default_quat;
    default_quat << 1.0, 0.0, 0.0, 0.0;
    auto quat = config.template Get<Vec4<double>>("initial_attitude", default_quat);
    attitude_ = Vec4<Scalar>{static_cast<Scalar>(quat(0)), static_cast<Scalar>(quat(1)),
                             static_cast<Scalar>(quat(2)), static_cast<Scalar>(quat(3))};
}
```

**Issues**: Redundant guards, template noise, manual `double→Scalar` conversion, inconsistent with `register_*` pattern.

---

## Proposed Solution

Add `read_param*` helpers to `Component<Scalar>`:

```cpp
// Scalars
dry_mass_ = read_param("dry_mass", 50.0);

// Vectors (auto double→Scalar conversion)
cg_ = read_param_vec3("tank_cg", Vec3<Scalar>::Zero());
attitude_ = read_param_vec4("initial_attitude", Vec4<Scalar>{1, 0, 0, 0});

// Required (throws ConfigError if missing)
mass_ = require_param<double>("mass");
```

---

## Before & After

### RigidBody6DOF::Stage() — 31 → 6 lines

```diff
void Stage(Backplane<Scalar> &) override {
-    const auto &config = this->GetConfig();
-
-    if (config.template Has<Vec3<double>>("initial_position")) {
-        auto pos = config.template Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
-        position_ = Vec3<Scalar>{static_cast<Scalar>(pos(0)), static_cast<Scalar>(pos(1)),
-                                 static_cast<Scalar>(pos(2))};
-    }
-    if (config.template Has<Vec3<double>>("initial_velocity_body")) {
-        auto vel = config.template Get<Vec3<double>>("initial_velocity_body", Vec3<double>::Zero());
-        velocity_body_ = Vec3<Scalar>{static_cast<Scalar>(vel(0)), static_cast<Scalar>(vel(1)),
-                                      static_cast<Scalar>(vel(2))};
-    }
-    if (config.template Has<Vec4<double>>("initial_attitude")) {
-        Vec4<double> default_quat;
-        default_quat << 1.0, 0.0, 0.0, 0.0;
-        auto quat = config.template Get<Vec4<double>>("initial_attitude", default_quat);
-        attitude_ = Vec4<Scalar>{static_cast<Scalar>(quat(0)), static_cast<Scalar>(quat(1)),
-                                 static_cast<Scalar>(quat(2)), static_cast<Scalar>(quat(3))};
-    }
-    if (config.template Has<Vec3<double>>("initial_omega_body")) {
-        auto omega = config.template Get<Vec3<double>>("initial_omega_body", Vec3<double>::Zero());
-        omega_body_ = Vec3<Scalar>{static_cast<Scalar>(omega(0)), static_cast<Scalar>(omega(1)),
-                                   static_cast<Scalar>(omega(2))};
-    }
+    position_      = read_param_vec3("initial_position", Vec3<Scalar>::Zero());
+    velocity_body_ = read_param_vec3("initial_velocity_body", Vec3<Scalar>::Zero());
+    attitude_      = read_param_vec4("initial_attitude", Vec4<Scalar>{1, 0, 0, 0});
+    omega_body_    = read_param_vec3("initial_omega_body", Vec3<Scalar>::Zero());
     // ... rest unchanged
}
```

### FuelTank::Stage() — 21 → 8 lines  

```diff
void Stage(Backplane<Scalar> &) override {
-    const auto &config = this->GetConfig();
-    if (config.template Has<double>("initial_fuel_mass")) {
-        initial_fuel_mass_ = config.template Get<double>("initial_fuel_mass", 1000.0);
-        fuel_mass_ = Scalar(initial_fuel_mass_);
-    }
-    if (config.template Has<double>("dry_mass")) {
-        dry_mass_ = config.template Get<double>("dry_mass", 50.0);
-    }
-    ...
+    initial_fuel_mass_ = read_param("initial_fuel_mass", 1000.0);
+    fuel_mass_ = Scalar(initial_fuel_mass_);
+    dry_mass_  = read_param("dry_mass", 50.0);
+    tank_radius_ = read_param("tank_radius", 0.5);
+    tank_length_ = read_param("tank_length", 2.0);
+    cg_ = read_param_vec3("tank_cg", Vec3<Scalar>::Zero());
     UpdateMassProperties();
}
```

---

## API

```cpp
// In Component<Scalar> protected:
template <typename T> T read_param(const std::string& key, const T& default_val) const;
template <typename T> T require_param(const std::string& key) const;  // throws

Vec3<Scalar> read_param_vec3(const std::string& key, const Vec3<Scalar>& def) const;
Vec4<Scalar> read_param_vec4(const std::string& key, const Vec4<Scalar>& def) const;
Vec3<Scalar> require_param_vec3(const std::string& key) const;
Vec4<Scalar> require_param_vec4(const std::string& key) const;
```

---

## Implementation

1. Add helpers to `Component.hpp` (protected methods)
2. Test in `tests/core/test_component_config_helpers.cpp`
3. Migrate existing components (optional, backward compatible)
