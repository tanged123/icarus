# Phase 4.0: Configuration Infrastructure

**Status:** Required before Phase 4 continues
**Goal:** Standardize component interface and centralize signal routing

---

## Key Architectural Decisions

1. **Uniform Component Interface**: Components only expose Constructor, Provision, Stage, Step, BindState
2. **Centralized Signal Routing**: Components declare ports; a separate SignalRouter handles all wiring
3. **Gain Factor Support**: Routes can include scaling for unit conversion or signal manipulation

---

## Motivation

Current components expose ad-hoc public APIs for configuration:

```cpp
// Current: Each component is a special snowflake
MassAggregator<double> agg("Mass", "Rocket");
agg.AddSource("Structure.mass_properties");      // Custom API
agg.AddSource("FuelTank.mass_properties");       // Custom API

PointMass3DOF<double> dynamics("EOM", "Rocket");
dynamics.SetMass(1000.0);                        // Custom API
dynamics.SetInitialPosition({0, 0, 6.4e6});      // Custom API
dynamics.SetInitialVelocity({7500, 0, 0});       // Custom API
```

**Problems:**
- Each component has a different setup API
- Can't serialize/deserialize simulations
- Can't create simulations from config files alone
- Component authors add arbitrary public methods
- No discoverability of what a component needs

---

## The Principle: Uniform Component Interface

Components expose **only** these public methods:

```cpp
template <typename Scalar>
class Component {
public:
    // === Identity (const, no side effects) ===
    virtual std::string Name() const = 0;
    virtual std::string Entity() const = 0;
    virtual std::string TypeName() const = 0;
    virtual std::size_t StateSize() const { return 0; }

    // === Lifecycle (THE ONLY MUTABLE INTERFACE) ===
    virtual void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) = 0;
    virtual void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) = 0;
    virtual void Step(Scalar t, Scalar dt) = 0;

    // === State binding (optional) ===
    virtual void BindState(Scalar* x, Scalar* x_dot) {}

    // === Destructor handles cleanup implicitly ===
    virtual ~Component() = default;
};
```

**Everything else comes from `ComponentConfig`.**

---

## Component Flexibility

### Private Functions Are Fine

Components can have rich internal implementations. The restriction is on the **public interface**, not internal structure:

```cpp
template <typename Scalar>
class ComplexAeroComponent : public Component<Scalar> {
public:
    // ONLY: Constructor, Name, Entity, TypeName, StateSize
    //       Provision, Stage, Step, BindState

    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Request config values (see below)
        auto table_path = cfg.Get<std::string>("aerodynamic_table_path");
        auto mach_breakpoints = cfg.Get<std::vector<double>>("mach_breakpoints");

        // Use private helpers for complex setup
        LoadAeroTables(table_path);
        BuildInterpolators(mach_breakpoints);

        // Register signals...
    }

private:
    // Private helpers - as many as you need
    void LoadAeroTables(const std::string& path);
    void BuildInterpolators(const std::vector<double>& breakpoints);
    Scalar InterpolateCd(Scalar mach, Scalar alpha) const;
    Scalar InterpolateCl(Scalar mach, Scalar alpha) const;

    // Private state
    AeroTable cd_table_;
    AeroTable cl_table_;
};
```

### What's Forbidden

```cpp
// DON'T: Public setters that bypass config
void SetAeroTablePath(const std::string& path);  // NO
void AddMachBreakpoint(double mach);              // NO
void EnableHighFidelityMode();                    // NO

// DON'T: Public getters for output values - USE SIGNALS
Vec3<Scalar> GetForce() const { return force_; }  // NO - use backplane
Scalar GetMass() const { return mass_; }          // NO - use backplane
const AeroTable& GetCdTable() const;              // NO - not a signal anyway
```

**Why no output getters?**
- Creates two paths to the same data (getter vs signal)
- Requires passing component pointers around
- Bypasses backplane tracking/logging
- Makes testing harder (mock both paths?)
- Undermines signal-first architecture

### What's Allowed

```cpp
// OK: Identity/metadata (required by Component interface)
std::string Name() const override;
std::string Entity() const override;
std::string TypeName() const override;
std::size_t StateSize() const override;
```

**That's it.** Everything else goes through signals or config.

---

## Centralized Signal Routing

### The Problem with Component-Driven Wiring

Current approach mixes concerns:

```cpp
// Component owns both port declaration AND wiring logic
void Provision(...) {
    bp.register_output<Scalar>("force", &force_, ...);  // Port declaration
    bp.register_input<Vec3<Scalar>>("cg", &cg_input_, ...);  // Port declaration
}
void Stage(...) {
    bp.wire_input<Vec3<Scalar>>(my_prefix + "cg", source_path);  // Wiring - shouldn't be here!
}
```

**Problems:**
- Wiring logic scattered across every component
- Can't reroute without code changes
- No visibility into overall signal flow
- No gain/scaling capability at connection level

### The Solution: Separate Port Declaration from Routing

**Components** only declare what they produce and consume:

```cpp
void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // DECLARE outputs (what I produce)
    bp.declare_output<Vec3<Scalar>>("force", &force_, "N", "Thrust force");
    bp.declare_output<Vec3<Scalar>>("application_point", &app_point_, "m", "Thrust location");

    // DECLARE inputs (what I need)
    bp.declare_input<Scalar>("throttle", &throttle_input_, "nd", "Throttle command 0-1");
}

void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) {
    // NO WIRING HERE - SignalRouter handles it
    // Just apply initial conditions, bind state, etc.
}
```

**SignalRouter** handles all connections externally:

```yaml
# signal_routes.yaml
routes:
  # Simple 1:1 routing
  - input: Rocket.Engine.throttle
    output: Rocket.GNC.throttle_cmd

  # With gain (unit conversion or scaling)
  - input: Rocket.EOM.gravity_accel
    output: Rocket.Gravity.accel_ecef
    gain: 1.0

  # Force aggregation sources
  - input: Rocket.Forces.src0_force
    output: Rocket.Gravity.force_body
    gain: 1.0

  - input: Rocket.Forces.src1_force
    output: Rocket.Engine.force
    gain: 1.0
```

### Workflow

1. **Run sim "unrouted"** → generates Data Dictionary with all declared ports
2. **Inspect Data Dictionary** → see all available inputs/outputs
3. **Write signal_routes.yaml** → define all connections
4. **Run sim with routes** → SignalRouter validates and wires everything

```bash
# Generate signal catalog (unrouted mode)
./icarus --config rocket.yaml --dump-signals > signals.txt

# signals.txt shows:
# OUTPUTS:
#   Rocket.Gravity.force_body    Vec3<Scalar>  N    Gravity force in body frame
#   Rocket.Engine.force          Vec3<Scalar>  N    Thrust force
#   Rocket.Engine.application_point Vec3<Scalar> m  Thrust location
#   ...
# INPUTS:
#   Rocket.Forces.src0_force     Vec3<Scalar>  N    Force source 0
#   Rocket.Forces.cg             Vec3<Scalar>  m    Center of gravity
#   ...

# Then create signal_routes.yaml and run
./icarus --config rocket.yaml --routes signal_routes.yaml
```

### Gain Factor Use Cases

```yaml
routes:
  # Unit conversion: km → m
  - input: Sensor.position_m
    output: GPS.position_km
    gain: 1000.0

  # Scaling: normalize to 0-1
  - input: GNC.throttle_normalized
    output: Pilot.throttle_percent
    gain: 0.01

  # Sign flip (e.g., body frame conventions)
  - input: EOM.force_z
    output: Thrust.force_up
    gain: -1.0

  # Pass-through (explicit 1:1)
  - input: Aero.velocity
    output: EOM.velocity
    gain: 1.0
```

---

## Config Request Pattern

Component authors explicitly request configuration values in `Provision()`. This is self-documenting and validates that required config is present.

### Basic Pattern

```cpp
void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    // === Request required config ===
    mass_ = cfg.Require<Scalar>("mass");  // Throws if missing

    // === Request optional config with defaults ===
    Cd_ = cfg.Get<Scalar>("drag_coefficient", 0.5);
    area_ = cfg.Get<Scalar>("reference_area", 1.0);

    // === Request typed values ===
    initial_position_ = cfg.Get<Vec3<double>>("initial_position", Vec3<double>::Zero());

    // === Request paths/strings ===
    auto table_path = cfg.Get<std::string>("aerodynamic_table_path", "");
    if (!table_path.empty()) {
        LoadAeroTables(table_path);
    }

    // === Request lists ===
    source_paths_ = cfg.Get<std::vector<std::string>>("sources", {});

    // Register signals...
}
```

### Self-Documenting Components

The `Provision()` method becomes the **single source of truth** for what a component needs:

```cpp
void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    // =========================================================================
    // Configuration
    // =========================================================================

    // Required: Maximum thrust magnitude [N]
    max_thrust_ = cfg.Require<Scalar>("max_thrust");

    // Required: Thrust direction unit vector in body frame
    thrust_direction_ = cfg.Require<Vec3<double>>("thrust_direction");

    // Required: Nozzle position in body frame [m]
    nozzle_position_ = cfg.Require<Vec3<double>>("nozzle_position");

    // Optional: Specific impulse [s], default 300
    isp_ = cfg.Get<Scalar>("isp", 300.0);

    // Optional: Gimbal limit [rad], default 0 (no gimbal)
    gimbal_limit_ = cfg.Get<Scalar>("gimbal_limit", 0.0);

    // =========================================================================
    // Signal Registration
    // =========================================================================

    bp.register_output_vec3("force", &force_, "N", "Thrust force");
    // ...
}
```

---

## Implementation Tasks

### 4.0.1 ComponentConfig Structure

**File:** `include/icarus/core/ComponentConfig.hpp`

```cpp
#pragma once

#include <icarus/core/Types.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>
#include <stdexcept>

namespace icarus {

/**
 * @brief Configuration container for components
 *
 * Passed to Provision() and Stage(). Components request values using
 * typed accessors. This is the ONLY way to configure a component
 * beyond the constructor (which only takes name/entity).
 */
struct ComponentConfig {
    // =========================================================================
    // Identity (set by loader/factory)
    // =========================================================================

    std::string name;
    std::string entity;
    std::string type;

    // =========================================================================
    // Raw storage (populated by loader)
    // =========================================================================

    std::unordered_map<std::string, double> scalars;
    std::unordered_map<std::string, std::vector<double>> vectors;  // Vec3 stored as [x,y,z]
    std::unordered_map<std::string, std::vector<double>> arrays;
    std::unordered_map<std::string, std::string> strings;
    std::unordered_map<std::string, int64_t> integers;
    std::unordered_map<std::string, bool> booleans;

    // List-based config (for aggregators with variable source counts, etc.)
    std::vector<std::string> sources;

    // NOTE: Signal wiring is NOT in ComponentConfig
    // Wiring is handled by SignalRouter from a separate config file

    // =========================================================================
    // Typed Accessors
    // =========================================================================

    /**
     * @brief Get a required config value (throws if missing)
     */
    template <typename T>
    T Require(const std::string& key) const;

    /**
     * @brief Get an optional config value with default
     */
    template <typename T>
    T Get(const std::string& key, const T& default_value) const;

    /**
     * @brief Check if a key exists
     */
    template <typename T>
    bool Has(const std::string& key) const;
};

// =========================================================================
// Template Specializations
// =========================================================================

template <>
inline double ComponentConfig::Get<double>(const std::string& key, const double& def) const {
    auto it = scalars.find(key);
    return (it != scalars.end()) ? it->second : def;
}

template <>
inline double ComponentConfig::Require<double>(const std::string& key) const {
    auto it = scalars.find(key);
    if (it == scalars.end()) {
        throw ConfigError("Missing required scalar: " + key);
    }
    return it->second;
}

template <>
inline std::string ComponentConfig::Get<std::string>(const std::string& key,
                                                      const std::string& def) const {
    auto it = strings.find(key);
    return (it != strings.end()) ? it->second : def;
}

template <>
inline Vec3<double> ComponentConfig::Get<Vec3<double>>(const std::string& key,
                                                        const Vec3<double>& def) const {
    auto it = vectors.find(key);
    if (it == vectors.end() || it->second.size() != 3) {
        return def;
    }
    return Vec3<double>{it->second[0], it->second[1], it->second[2]};
}

template <>
inline std::vector<std::string> ComponentConfig::Get<std::vector<std::string>>(
    const std::string& key, const std::vector<std::string>& def) const {
    // For sources list, return sources member
    if (key == "sources") {
        return sources.empty() ? def : sources;
    }
    return def;
}

// ... additional specializations for int, bool, Mat3, etc.

} // namespace icarus
```

**Exit Criteria:**
- [ ] `ComponentConfig` struct defined
- [ ] `Require<T>()` throws `ConfigError` if missing
- [ ] `Get<T>()` returns default if missing
- [ ] `Has<T>()` checks existence
- [ ] Specializations for: `double`, `int`, `bool`, `std::string`, `Vec3<double>`, `std::vector<double>`, `std::vector<std::string>`

---

### 4.0.2 ConfigError Exception

**File:** `include/icarus/core/Error.hpp` (extend existing)

```cpp
/**
 * @brief Thrown when required configuration is missing or invalid
 */
class ConfigError : public std::runtime_error {
public:
    explicit ConfigError(const std::string& msg)
        : std::runtime_error("Configuration error: " + msg) {}

    ConfigError(const std::string& component, const std::string& key)
        : std::runtime_error("Configuration error: " + component +
                            " missing required key '" + key + "'") {}
};
```

---

### 4.0.3 YAML Configuration Loader

**File:** `include/icarus/io/ConfigLoader.hpp`

**Dependency:** Vulcan Phase 24 (`vulcan::io::YamlNode`)

```cpp
#pragma once

#include <icarus/core/ComponentConfig.hpp>
#include <vulcan/io/YamlNode.hpp>

#include <string>
#include <vector>

namespace icarus {
namespace io {

/**
 * @brief Loads component configurations from YAML files
 *
 * Uses Vulcan's YamlNode for parsing with Janus type support.
 */
class ConfigLoader {
public:
    /**
     * @brief Load simulation config (multiple components)
     */
    static std::vector<ComponentConfig> LoadSimulation(const std::string& yaml_path);

    /**
     * @brief Load a single component config from a YamlNode
     */
    static ComponentConfig LoadComponent(const vulcan::io::YamlNode& node);
};

} // namespace io
} // namespace icarus
```

**Implementation:**

```cpp
// src/io/ConfigLoader.cpp
#include <icarus/io/ConfigLoader.hpp>

namespace icarus {
namespace io {

std::vector<ComponentConfig> ConfigLoader::LoadSimulation(const std::string& path) {
    auto root = vulcan::io::YamlNode::LoadFile(path);
    std::vector<ComponentConfig> configs;

    auto components = root["components"];
    for (std::size_t i = 0; i < components.Size(); ++i) {
        configs.push_back(LoadComponent(components[i]));
    }

    return configs;
}

ComponentConfig ConfigLoader::LoadComponent(const vulcan::io::YamlNode& node) {
    ComponentConfig cfg;

    // Required fields
    cfg.type = node.Require<std::string>("type");
    cfg.name = node.Require<std::string>("name");
    cfg.entity = node.Get<std::string>("entity", "");

    // Extract typed values into ComponentConfig storage
    if (node.Has("scalars")) {
        node["scalars"].ForEachEntry([&](const std::string& key,
                                          const vulcan::io::YamlNode& val) {
            cfg.scalars[key] = val.As<double>();
        });
    }

    if (node.Has("vectors")) {
        node["vectors"].ForEachEntry([&](const std::string& key,
                                          const vulcan::io::YamlNode& val) {
            cfg.vectors[key] = val.As<std::vector<double>>();
        });
    }

    if (node.Has("strings")) {
        node["strings"].ForEachEntry([&](const std::string& key,
                                          const vulcan::io::YamlNode& val) {
            cfg.strings[key] = val.As<std::string>();
        });
    }

    if (node.Has("integers")) {
        node["integers"].ForEachEntry([&](const std::string& key,
                                           const vulcan::io::YamlNode& val) {
            cfg.integers[key] = val.As<int64_t>();
        });
    }

    if (node.Has("booleans")) {
        node["booleans"].ForEachEntry([&](const std::string& key,
                                           const vulcan::io::YamlNode& val) {
            cfg.booleans[key] = val.As<bool>();
        });
    }

    return cfg;
}

} // namespace io
} // namespace icarus
```

---

### 4.0.4 YAML Schema

#### Component Configuration: `config/rocket_6dof.yaml`

Component configs define **what** each component is and its parameters - NOT how they're connected:

```yaml
# Icarus Simulation Configuration
# Component configuration ONLY - signal routing is separate

simulation:
  name: "Rocket 6DOF Test"
  dt: 0.01
  t_end: 100.0

components:
  # =========================================================================
  # Mass Sources
  # =========================================================================

  - type: Structure
    name: Structure
    entity: Rocket
    scalars:
      mass: 500.0
    vectors:
      cg: [0, 0, 2.0]
      inertia_diagonal: [100, 100, 50]

  - type: FuelTank
    name: FuelTank
    entity: Rocket
    scalars:
      initial_fuel: 1000.0
      tank_volume: 2.0
    vectors:
      tank_cg: [0, 0, 1.0]
      tank_inertia_diagonal: [50, 50, 20]
    # NOTE: No wiring here - that's in signal_routes.yaml

  # =========================================================================
  # Mass Aggregator
  # =========================================================================

  - type: MassAggregator
    name: Mass
    entity: Rocket
    integers:
      num_sources: 2  # Aggregator declares N generic source inputs

  # =========================================================================
  # Force Sources
  # =========================================================================

  - type: PointMassGravity
    name: Gravity
    entity: Rocket
    scalars:
      mu: 3.986004418e14  # Earth GM

  - type: Thrust
    name: Engine
    entity: Rocket
    scalars:
      max_thrust: 50000.0
      isp: 300.0
    vectors:
      thrust_direction: [0, 0, -1]
      nozzle_position: [0, 0, 0]

  # =========================================================================
  # Force Aggregator
  # =========================================================================

  - type: ForceAggregator
    name: Forces
    entity: Rocket
    integers:
      num_sources: 2  # Declares N generic source inputs

  # =========================================================================
  # Dynamics
  # =========================================================================

  - type: RigidBody6DOF
    name: EOM
    entity: Rocket
    vectors:
      initial_position: [0, 0, 6.471e6]
      initial_velocity: [7500, 0, 0]
      initial_attitude: [1, 0, 0, 0]  # quaternion wxyz
      initial_angular_velocity: [0, 0, 0]
```

#### Signal Routing: `config/rocket_routes.yaml`

Signal routing is **completely separate** from component config:

```yaml
# Signal routing file
# Format: input -> output with optional gain
#
# Run with: ./icarus --config rocket_6dof.yaml --routes rocket_routes.yaml

routes:
  # =========================================================================
  # Mass Aggregator inputs
  # =========================================================================

  - input: Rocket.Mass.source_0
    output: Rocket.Structure.mass_properties
    gain: 1.0

  - input: Rocket.Mass.source_1
    output: Rocket.FuelTank.mass_properties
    gain: 1.0

  # =========================================================================
  # Gravity inputs
  # =========================================================================

  - input: Rocket.Gravity.position
    output: Rocket.EOM.position
    gain: 1.0

  - input: Rocket.Gravity.mass
    output: Rocket.Mass.total_mass
    gain: 1.0

  # =========================================================================
  # Thrust inputs
  # =========================================================================

  - input: Rocket.Engine.throttle
    output: Rocket.GNC.throttle_cmd
    gain: 1.0

  # =========================================================================
  # Force Aggregator inputs
  # =========================================================================

  - input: Rocket.Forces.cg
    output: Rocket.Mass.cg
    gain: 1.0

  # Source 0: Gravity (acts at CG, no moment arm)
  - input: Rocket.Forces.src0_force
    output: Rocket.Gravity.force_body
    gain: 1.0

  # Source 1: Engine thrust (with application point for moment arm)
  - input: Rocket.Forces.src1_force
    output: Rocket.Engine.force
    gain: 1.0

  - input: Rocket.Forces.src1_app_point
    output: Rocket.Engine.application_point
    gain: 1.0

  # =========================================================================
  # EOM inputs
  # =========================================================================

  - input: Rocket.EOM.total_force
    output: Rocket.Forces.total_force
    gain: 1.0

  - input: Rocket.EOM.total_moment
    output: Rocket.Forces.total_moment
    gain: 1.0

  - input: Rocket.EOM.total_mass
    output: Rocket.Mass.total_mass
    gain: 1.0

  - input: Rocket.EOM.inertia
    output: Rocket.Mass.inertia
    gain: 1.0

  # =========================================================================
  # Fuel tank (mass depletion from engine)
  # =========================================================================

  - input: Rocket.FuelTank.fuel_flow_rate
    output: Rocket.Engine.fuel_flow
    gain: 1.0
```

---

### 4.0.5 Component Factory

**File:** `include/icarus/core/ComponentFactory.hpp`

```cpp
#pragma once

#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentConfig.hpp>
#include <functional>
#include <memory>
#include <unordered_map>

namespace icarus {

/**
 * @brief Factory for creating components from configuration
 */
template <typename Scalar>
class ComponentFactory {
public:
    using Creator = std::function<std::unique_ptr<Component<Scalar>>(
        const std::string& name, const std::string& entity)>;

    /**
     * @brief Register a component type
     */
    void Register(const std::string& type_name, Creator creator) {
        creators_[type_name] = std::move(creator);
    }

    /**
     * @brief Create a component from config
     */
    std::unique_ptr<Component<Scalar>> Create(const ComponentConfig& config) {
        auto it = creators_.find(config.type);
        if (it == creators_.end()) {
            throw ConfigError("Unknown component type: " + config.type);
        }
        return it->second(config.name, config.entity);
    }

    /**
     * @brief Check if a type is registered
     */
    bool HasType(const std::string& type_name) const {
        return creators_.count(type_name) > 0;
    }

    /**
     * @brief Get singleton instance
     */
    static ComponentFactory& Instance() {
        static ComponentFactory instance;
        return instance;
    }

private:
    std::unordered_map<std::string, Creator> creators_;
};

/**
 * @brief Helper macro for component registration
 *
 * Usage in component .cpp file:
 *   ICARUS_REGISTER_COMPONENT(MassAggregator)
 */
#define ICARUS_REGISTER_COMPONENT(ComponentType) \
    namespace { \
    static bool _reg_##ComponentType = []() { \
        ::icarus::ComponentFactory<double>::Instance().Register( \
            #ComponentType, \
            [](const std::string& name, const std::string& entity) { \
                return std::make_unique<ComponentType<double>>(name, entity); \
            }); \
        return true; \
    }(); \
    }

} // namespace icarus
```

---

### 4.0.6 SignalRouter

**File:** `include/icarus/signal/SignalRouter.hpp`

The SignalRouter handles all signal wiring **externally** from components:

```cpp
#pragma once

#include <icarus/signal/Backplane.hpp>
#include <string>
#include <vector>

namespace icarus {
namespace signal {

/**
 * @brief A single signal route with optional gain
 */
struct SignalRoute {
    std::string input_path;   // Full path: Entity.Component.signal
    std::string output_path;  // Full path: Entity.Component.signal
    double gain = 1.0;        // Scale factor applied on read
};

/**
 * @brief Centralized signal routing configuration
 *
 * Handles all signal wiring separately from component code.
 * Components only declare ports; SignalRouter connects them.
 */
template <typename Scalar>
class SignalRouter {
public:
    /**
     * @brief Add a route
     */
    void AddRoute(const SignalRoute& route) {
        routes_.push_back(route);
    }

    /**
     * @brief Load routes from YAML file
     */
    void LoadRoutes(const std::string& yaml_path);

    /**
     * @brief Apply all routes to the backplane
     *
     * Called by Simulator after all components have declared their ports.
     * Validates that all inputs and outputs exist before wiring.
     */
    void ApplyRoutes(Backplane<Scalar>& bp) {
        // First pass: validate all signals exist
        for (const auto& route : routes_) {
            if (!bp.HasOutput(route.output_path)) {
                throw RoutingError("Output not found: " + route.output_path);
            }
            if (!bp.HasInput(route.input_path)) {
                throw RoutingError("Input not found: " + route.input_path);
            }
        }

        // Second pass: wire with gains
        for (const auto& route : routes_) {
            bp.WireWithGain(route.input_path, route.output_path, route.gain);
        }
    }

    /**
     * @brief Validate routes without applying
     *
     * Returns list of errors (empty if valid)
     */
    std::vector<std::string> ValidateRoutes(const Backplane<Scalar>& bp) const;

    /**
     * @brief Get all routes for inspection
     */
    const std::vector<SignalRoute>& GetRoutes() const { return routes_; }

    /**
     * @brief Get unwired inputs after routing
     *
     * Useful for finding configuration issues
     */
    std::vector<std::string> GetUnwiredInputs(const Backplane<Scalar>& bp) const;

private:
    std::vector<SignalRoute> routes_;
};

/**
 * @brief Exception for routing errors
 */
class RoutingError : public std::runtime_error {
public:
    explicit RoutingError(const std::string& msg)
        : std::runtime_error("Signal routing error: " + msg) {}
};

} // namespace signal
} // namespace icarus
```

**File:** `include/icarus/io/RouteLoader.hpp`

```cpp
#pragma once

#include <icarus/signal/SignalRouter.hpp>
#include <string>

namespace icarus {
namespace io {

/**
 * @brief Loads signal routes from YAML files
 */
class RouteLoader {
public:
    /**
     * @brief Load routes from file
     */
    template <typename Scalar>
    static void LoadRoutes(signal::SignalRouter<Scalar>& router,
                           const std::string& yaml_path);

    /**
     * @brief Parse routes from YAML string
     */
    template <typename Scalar>
    static void ParseRoutes(signal::SignalRouter<Scalar>& router,
                            const std::string& yaml_content);
};

} // namespace io
} // namespace icarus
```

---

### 4.0.7 Simulator Config Loading

**File:** `include/icarus/sim/Simulator.hpp` (extend)

```cpp
/**
 * @brief Load simulation from YAML config and routes
 */
static Simulator<Scalar> FromConfig(const std::string& config_path,
                                    const std::string& routes_path = "") {
    auto configs = io::ConfigLoader::LoadSimulation(config_path);

    Simulator<Scalar> sim;

    // Create all components
    for (const auto& cfg : configs) {
        auto component = ComponentFactory<Scalar>::Instance().Create(cfg);
        sim.AddComponent(std::move(component), cfg);
    }

    // Load routes if provided
    if (!routes_path.empty()) {
        io::RouteLoader::LoadRoutes(sim.GetRouter(), routes_path);
    }

    return sim;
}

/**
 * @brief Run simulation with centralized routing
 */
void Run(Scalar t_end, Scalar dt) {
    // 1. Provision: Components declare ports
    for (auto& comp : components_) {
        comp->Provision(backplane_, configs_[comp.get()]);
    }

    // 2. Apply signal routes (done by SignalRouter, NOT components)
    router_.ApplyRoutes(backplane_);

    // 3. Stage: Components apply ICs, bind state
    for (auto& comp : components_) {
        comp->Stage(backplane_, configs_[comp.get()]);
    }

    // 4. Validate all inputs wired
    auto unwired = router_.GetUnwiredInputs(backplane_);
    if (!unwired.empty()) {
        // Log warnings or throw based on config
        for (const auto& signal : unwired) {
            logger_.Warn("Unwired input: " + signal);
        }
    }

    // 5. Run simulation loop
    // ...
}
```

---

### 4.0.8 Refactor Existing Components

Update all existing components to use config-driven initialization.
**Key change:** Components NO LONGER wire inputs in Stage() - SignalRouter handles that.

#### MassAggregator (refactored)

```cpp
template <typename Scalar>
class MassAggregator : public Component<Scalar> {
public:
    explicit MassAggregator(std::string name, std::string entity)
        : name_(std::move(name)), entity_(std::move(entity)) {}

    // === Identity ===
    std::string Name() const override { return name_; }
    std::string Entity() const override { return entity_; }
    std::string TypeName() const override { return "MassAggregator"; }
    std::size_t StateSize() const override { return 0; }

    // === Lifecycle ===
    void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // Read config: how many sources?
        num_sources_ = cfg.Get<int>("num_sources", 1);

        // === DECLARE outputs ===
        bp.declare_output<Scalar>("total_mass", &total_mass_, "kg", "Total mass");
        bp.declare_output<Vec3<Scalar>>("cg", &cg_, "m", "Center of gravity");
        bp.declare_output<MassProps>("mass_properties", &mass_props_, "", "Combined");

        // === DECLARE inputs (generic named ports) ===
        for (int i = 0; i < num_sources_; ++i) {
            sources_.emplace_back();
            bp.declare_input<MassProps>("source_" + std::to_string(i),
                                        &sources_.back(), "", "Mass source");
        }
        // NOTE: NO wiring here - SignalRouter handles that from routes.yaml
    }

    void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
        // NO WIRING - just initialization if needed
        // (MassAggregator has no ICs to apply)
    }

    void Step(Scalar t, Scalar dt) override {
        AggregateFromSources();
    }

private:
    void AggregateFromSources() {
        // Private helper - implementation unchanged
        MassProps total{};
        for (const auto& src : sources_) {
            if (src.is_wired()) {
                total = total + src.get();  // Vulcan parallel axis theorem
            }
        }
        total_mass_ = total.mass;
        cg_ = total.cg;
        mass_props_ = total;
    }

    std::string name_, entity_;
    int num_sources_ = 1;
    std::vector<InputHandle<MassProps>> sources_;

    // Outputs (SignalRouter connects consumers to these)
    Scalar total_mass_{0};
    Vec3<Scalar> cg_ = Vec3<Scalar>::Zero();
    MassProps mass_props_;
};
```

#### PointMass3DOF (refactored)

```cpp
void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    // === Read configuration ===
    mass_ = cfg.Get<double>("mass", 1.0);
    initial_position_ = cfg.Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
    initial_velocity_ = cfg.Get<Vec3<double>>("initial_velocity", Vec3<double>::Zero());

    // === DECLARE outputs ===
    bp.declare_output<Vec3<Scalar>>("position", &position_, "m", "Position");
    bp.declare_output<Vec3<Scalar>>("velocity", &velocity_, "m/s", "Velocity");
    bp.declare_output<Scalar>("mass", &mass_, "kg", "Mass");

    // === DECLARE inputs ===
    bp.declare_input<Vec3<Scalar>>("force", &force_input_, "N", "External force");
    // NOTE: NO wiring - SignalRouter handles connections
}

void Stage(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
    // Apply initial conditions
    position_ = initial_position_;
    velocity_ = initial_velocity_;

    // Bind state to integrator
    BindState(bp);

    // NOTE: NO wiring here - that's SignalRouter's job
}
```

---

## Component Author Guide

### Checklist for New Components

1. **Constructor**: Only `name` and `entity` parameters
2. **Provision()**:
   - Request all config using `cfg.Require<T>()` or `cfg.Get<T>()`
   - **Declare** all outputs with `bp.declare_output<T>()`
   - **Declare** all inputs with `bp.declare_input<T>()`
   - **NO wiring** - just declare what you produce/consume
3. **Stage()**:
   - Apply initial conditions
   - Bind state if needed
   - **NO wiring** - SignalRouter handles all connections
4. **Step()**: Compute, no config access
5. **Private helpers**: Use freely for internal logic

### Config Request Patterns

```cpp
// Required scalar (throws if missing)
auto mass = cfg.Require<double>("mass");

// Optional scalar with default
auto cd = cfg.Get<double>("drag_coefficient", 0.5);

// Required vector
auto cg = cfg.Require<Vec3<double>>("cg");

// Optional string (file path)
auto table_path = cfg.Get<std::string>("aero_table_path", "");

// List of sources
auto sources = cfg.Get<std::vector<std::string>>("sources", {});

// Integer enum selection
auto model = cfg.Get<int>("gravity_model", 0);

// Boolean flag
auto enabled = cfg.Get<bool>("enable_drag", true);
```

---

## File Structure

```
include/icarus/
├── core/
│   ├── Component.hpp          # No changes (already correct interface)
│   ├── ComponentConfig.hpp    # NEW
│   ├── ComponentFactory.hpp   # NEW
│   └── Error.hpp              # Add ConfigError, RoutingError
├── io/
│   ├── ConfigLoader.hpp       # NEW - component config loading
│   └── RouteLoader.hpp        # NEW - signal route loading
├── signal/
│   ├── Backplane.hpp          # Add declare_input/output, WireWithGain
│   └── SignalRouter.hpp       # NEW - centralized signal routing
└── sim/
    └── Simulator.hpp          # Add FromConfig(), GetRouter()

src/
├── core/
│   └── ComponentConfig.cpp    # Template specializations
├── io/
│   ├── ConfigLoader.cpp       # yaml-cpp implementation
│   └── RouteLoader.cpp        # yaml-cpp implementation
└── signal/
    └── SignalRouter.cpp       # Route validation, gain application

components/
├── aggregators/
│   ├── ForceAggregator.hpp    # Refactor: declare_* only, no wiring
│   └── MassAggregator.hpp     # Refactor: declare_* only, no wiring
├── dynamics/
│   └── PointMass3DOF.hpp      # Refactor: declare_* only, no wiring
└── environment/
    └── PointMassGravity.hpp   # Refactor: declare_* only, no wiring

config/
├── rocket_6dof.yaml           # Example component config
└── rocket_routes.yaml         # Example signal routing
```

---

## Dependencies

| Dependency | Purpose | Notes |
|:-----------|:--------|:------|
| Vulcan Phase 24 | YAML infrastructure | `vulcan::io::YamlNode`, type conversions |

> **Note:** Vulcan provides the YAML parsing utilities. See [`references/vulcan/docs/implementation_plans/phase24_yaml_infrastructure.md`](../../../references/vulcan/docs/implementation_plans/phase24_yaml_infrastructure.md) for details.
>
> Icarus uses Vulcan's `YamlNode` for config loading:
> ```cpp
> #include <vulcan/io/YamlNode.hpp>
>
> auto config = vulcan::io::YamlNode::LoadFile("rocket.yaml");
> auto mass = config["vehicle"].Require<double>("mass");
> auto position = config.Require<janus::Vec3<double>>("initial_position");
> ```

---

## Exit Criteria (Phase 4.0 Complete)

- [ ] **4.0.1** `ComponentConfig` with typed accessors
- [ ] **4.0.2** `ConfigError` and `RoutingError` exception classes
- [ ] **4.0.3** `ConfigLoader` using Vulcan's `YamlNode`
- [ ] **4.0.4** Example YAML config files (component + routes)
- [ ] **4.0.5** `ComponentFactory` with registration macro
- [ ] **4.0.6** `SignalRouter` with gain support:
  - [ ] `SignalRoute` struct with input/output/gain
  - [ ] `ApplyRoutes()` wires all connections
  - [ ] `ValidateRoutes()` checks before wiring
  - [ ] `GetUnwiredInputs()` reports missing connections
- [ ] **4.0.7** `Simulator::FromConfig()` with route loading
- [ ] **4.0.8** `Backplane` updates:
  - [ ] `declare_output<T>()` (separate from wiring)
  - [ ] `declare_input<T>()` (separate from wiring)
  - [ ] `WireWithGain()` for scaled connections
  - [ ] `HasOutput()` / `HasInput()` for validation
- [ ] **4.0.9** All existing components refactored:
  - [ ] `MassAggregator` - declare_* only, no public methods
  - [ ] `ForceAggregator` - declare_* only, no public methods
  - [ ] `PointMass3DOF` - declare_* only, no public methods
  - [ ] `PointMassGravity` - declare_* only, no public methods
- [ ] **4.0.10** Unit tests for config loading
- [ ] **4.0.11** Unit tests for signal routing with gain
- [ ] **4.0.12** Integration test: load YAML config + routes → run simulation

---

## Next Steps

After Phase 4.0 is complete, continue with Phase 4.1+ using:
- Config-driven component initialization
- Centralized signal routing with gain factors
- Uniform component interface (no public setters/getters)
