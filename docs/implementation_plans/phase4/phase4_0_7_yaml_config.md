# Phase 4.0.7: YAML Configuration System

**Parent:** [phase4_0_7_overview.md](phase4_0_7_overview.md)

---

## Foundation: Vulcan YAML Infrastructure

Icarus builds on Vulcan's YAML system. See [vulcan/docs/user_guides/yaml_configuration.md](../../../references/vulcan/docs/user_guides/yaml_configuration.md).

**Vulcan provides:**

| Feature | API | Notes |
|:--------|:----|:------|
| Type-safe loading | `YamlNode::LoadFile()` | Returns wrapped node |
| Required values | `node.Require<T>(key)` | Throws if missing |
| Optional values | `node.Get<T>(key, default)` | Returns default if missing |
| Existence check | `node.Has(key)` | Boolean check |
| Navigation | `node["nested"]["path"]` | Bracket access |
| Iteration | `node.ForEach()`, `node.ForEachEntry()` | Sequences and maps |
| Janus types | `Vec3<double>`, `Quaternion<double>`, `Mat3<double>` | Native support |
| Env vars | `${VAR}`, `${VAR:default}` | Via `YamlEnv` |
| Includes | `!include path.yaml` | Via `YamlFile` |
| Merging | `YamlFile::MergeFiles({...})` | Later overrides earlier |

---

## Icarus Configuration Architecture

### Single-File Mode (Default)

Everything inline - no external files required:

```yaml
# simulation.yaml - COMPLETE simulation in one file

simulation:
  name: "Orbital Demo"
  version: "1.0.0"

time:
  start: 0.0
  end: 1000.0
  dt: 0.01

components:
  - type: PointMassGravity
    name: Gravity
    scalars:
      mu: 3.986004418e14

  - type: PointMass3DOF
    name: Dynamics
    scalars:
      mass: 1000.0
    vectors:
      initial_position: [6.871e6, 0, 0]
      initial_velocity: [0, 7500, 0]

routes:
  - input: Gravity.position
    output: Dynamics.position
  - input: Dynamics.force
    output: Gravity.force

scheduler:
  groups:
    - name: dynamics
      rate_hz: 100
      priority: 1
      members:
        - component: Gravity
          priority: 1
        - component: Dynamics
          priority: 2

integrator:
  type: RK4

logging:
  console_level: Info
```

### Multi-File Mode (Optional)

For complex simulations, split into files using `!include`:

```yaml
# simulation.yaml - Master binding file

simulation:
  name: "Formation Flight"

time:
  start: 0.0
  end: 300.0

entities:
  - template: !include entities/rocket.yaml
    name: Leader
    overrides:
      EOM:
        vectors:
          initial_position: [0, 0, 6.471e6]

  - template: !include entities/rocket.yaml
    name: Follower
    overrides:
      EOM:
        vectors:
          initial_position: [100, 0, 6.471e6]

cross_entity_routes:
  - input: Follower.GNC.leader_position
    output: Leader.EOM.position

integrator: !include config/integrator.yaml
logging: !include config/logging.yaml
```

### Environment Variable Support

```yaml
# Use env vars for paths and runtime config
simulation:
  name: ${MISSION_NAME:Default Mission}

output:
  directory: ${OUTPUT_DIR:/tmp/icarus}

logging:
  file_path: ${LOG_DIR}/sim.log
  console_level: ${LOG_LEVEL:Info}
```

---

## Configuration Loading Implementation

### SimulationLoader Class

```cpp
#pragma once

#include <icarus/core/SimulatorConfig.hpp>
#include <vulcan/io/YamlNode.hpp>
#include <vulcan/io/YamlEnv.hpp>
#include <vulcan/io/YamlFile.hpp>

namespace icarus::io {

/**
 * @brief Loads complete simulation configuration from YAML
 *
 * Handles:
 * - Single-file mode (everything inline)
 * - Multi-file mode (!include directives)
 * - Environment variable expansion
 * - Entity template expansion
 * - Route flattening
 */
class SimulationLoader {
public:
    /**
     * @brief Load simulation config with full feature support
     *
     * Uses Vulcan's YamlEnv::LoadWithIncludesAndEnv() for:
     * - !include directive resolution
     * - ${VAR} and ${VAR:default} expansion
     */
    static SimulatorConfig Load(const std::string& path) {
        // Load with includes and env var expansion
        auto root = vulcan::io::YamlEnv::LoadWithIncludesAndEnv(path);
        return ParseRoot(root, path);
    }

    /**
     * @brief Load from YAML string (for testing)
     */
    static SimulatorConfig Parse(const std::string& yaml_content) {
        auto root = vulcan::io::YamlNode::Parse(yaml_content);
        return ParseRoot(root, "<string>");
    }

private:
    static SimulatorConfig ParseRoot(const vulcan::io::YamlNode& root,
                                      const std::string& source_path);
    static void ParseSimulationSection(SimulatorConfig& cfg,
                                        const vulcan::io::YamlNode& node);
    static void ParseTimeSection(SimulatorConfig& cfg,
                                  const vulcan::io::YamlNode& node);
    static void ParseComponents(SimulatorConfig& cfg,
                                 const vulcan::io::YamlNode& node);
    static void ParseEntities(SimulatorConfig& cfg,
                               const vulcan::io::YamlNode& node);
    static void ParseRoutes(SimulatorConfig& cfg,
                             const vulcan::io::YamlNode& node);
    static void ParseScheduler(SimulatorConfig& cfg,
                                const vulcan::io::YamlNode& node);
    static void ParseIntegrator(SimulatorConfig& cfg,
                                 const vulcan::io::YamlNode& node);
    static void ParseLogging(SimulatorConfig& cfg,
                              const vulcan::io::YamlNode& node);
    static void ParseStaging(SimulatorConfig& cfg,
                              const vulcan::io::YamlNode& node);
};

} // namespace icarus::io
```

### Implementation

```cpp
// src/io/SimulationLoader.cpp

#include <icarus/io/SimulationLoader.hpp>
#include <icarus/core/Error.hpp>

namespace icarus::io {

SimulatorConfig SimulationLoader::ParseRoot(const vulcan::io::YamlNode& root,
                                             const std::string& source_path) {
    SimulatorConfig cfg;
    cfg.source_file = source_path;

    // Parse each section (all optional except components or entities)
    if (root.Has("simulation")) {
        ParseSimulationSection(cfg, root["simulation"]);
    }

    if (root.Has("time")) {
        ParseTimeSection(cfg, root["time"]);
    }

    // Either components (single-file) or entities (multi-file) must exist
    bool has_components = root.Has("components");
    bool has_entities = root.Has("entities");

    if (!has_components && !has_entities) {
        throw ConfigError("Config must have either 'components' or 'entities' section",
                          source_path);
    }

    if (has_components) {
        ParseComponents(cfg, root["components"]);
    }

    if (has_entities) {
        ParseEntities(cfg, root["entities"]);
    }

    if (root.Has("routes")) {
        ParseRoutes(cfg, root["routes"]);
    }

    if (root.Has("cross_entity_routes")) {
        ParseRoutes(cfg, root["cross_entity_routes"]);
    }

    if (root.Has("scheduler")) {
        ParseScheduler(cfg, root["scheduler"]);
    }

    if (root.Has("integrator")) {
        ParseIntegrator(cfg, root["integrator"]);
    }

    if (root.Has("logging")) {
        ParseLogging(cfg, root["logging"]);
    }

    if (root.Has("staging")) {
        ParseStaging(cfg, root["staging"]);
    }

    return cfg;
}

void SimulationLoader::ParseComponents(SimulatorConfig& cfg,
                                        const vulcan::io::YamlNode& node) {
    node.ForEach([&](const vulcan::io::YamlNode& comp_node) {
        ComponentConfig comp;

        // Required fields
        comp.type = comp_node.Require<std::string>("type");
        comp.name = comp_node.Require<std::string>("name");

        // Optional entity prefix
        comp.entity = comp_node.Get<std::string>("entity", "");

        // Extract typed config values
        if (comp_node.Has("scalars")) {
            comp_node["scalars"].ForEachEntry([&](const std::string& key,
                                                   const vulcan::io::YamlNode& val) {
                comp.scalars[key] = val.As<double>();
            });
        }

        if (comp_node.Has("vectors")) {
            comp_node["vectors"].ForEachEntry([&](const std::string& key,
                                                   const vulcan::io::YamlNode& val) {
                comp.vectors[key] = val.ToVector<double>();
            });
        }

        if (comp_node.Has("strings")) {
            comp_node["strings"].ForEachEntry([&](const std::string& key,
                                                   const vulcan::io::YamlNode& val) {
                comp.strings[key] = val.As<std::string>();
            });
        }

        if (comp_node.Has("integers")) {
            comp_node["integers"].ForEachEntry([&](const std::string& key,
                                                    const vulcan::io::YamlNode& val) {
                comp.integers[key] = val.As<int64_t>();
            });
        }

        if (comp_node.Has("booleans")) {
            comp_node["booleans"].ForEachEntry([&](const std::string& key,
                                                    const vulcan::io::YamlNode& val) {
                comp.booleans[key] = val.As<bool>();
            });
        }

        cfg.components.push_back(std::move(comp));
    });
}

void SimulationLoader::ParseRoutes(SimulatorConfig& cfg,
                                    const vulcan::io::YamlNode& node) {
    node.ForEach([&](const vulcan::io::YamlNode& route_node) {
        signal::SignalRoute route;
        route.input_path = route_node.Require<std::string>("input");
        route.output_path = route_node.Require<std::string>("output");
        route.gain = route_node.Get<double>("gain", 1.0);
        route.offset = route_node.Get<double>("offset", 0.0);
        route.delay = route_node.Get<double>("delay", 0.0);
        cfg.routes.push_back(route);
    });
}

// ... additional parsing methods ...

} // namespace icarus::io
```

---

## Error Handling

### Error Types

```cpp
namespace icarus {

/**
 * @brief Base class for configuration errors
 */
class ConfigError : public std::runtime_error {
public:
    ConfigError(const std::string& message,
                const std::string& file = "",
                int line = -1,
                const std::string& hint = "")
        : std::runtime_error(FormatMessage(message, file, line, hint))
        , file_(file)
        , line_(line)
        , hint_(hint) {}

    const std::string& file() const { return file_; }
    int line() const { return line_; }
    const std::string& hint() const { return hint_; }

private:
    static std::string FormatMessage(const std::string& msg,
                                      const std::string& file,
                                      int line,
                                      const std::string& hint) {
        std::string result = "Config error: " + msg;
        if (!file.empty()) {
            result += "\n  at: " + file;
            if (line >= 0) {
                result += ":" + std::to_string(line);
            }
        }
        if (!hint.empty()) {
            result += "\n  hint: " + hint;
        }
        return result;
    }

    std::string file_;
    int line_;
    std::string hint_;
};

/**
 * @brief Missing required key
 */
class MissingKeyError : public ConfigError {
public:
    MissingKeyError(const std::string& key,
                    const std::string& file = "",
                    int line = -1)
        : ConfigError("Missing required key '" + key + "'", file, line)
        , key_(key) {}

    const std::string& key() const { return key_; }

private:
    std::string key_;
};

/**
 * @brief Type mismatch
 */
class TypeMismatchError : public ConfigError {
public:
    TypeMismatchError(const std::string& key,
                      const std::string& expected,
                      const std::string& actual,
                      const std::string& file = "")
        : ConfigError("Key '" + key + "' expected " + expected +
                      " but got " + actual, file) {}
};

/**
 * @brief Unknown component type
 */
class UnknownTypeError : public ConfigError {
public:
    UnknownTypeError(const std::string& type_name,
                     const std::string& file = "",
                     const std::vector<std::string>& suggestions = {})
        : ConfigError("Unknown component type '" + type_name + "'",
                      file, -1, FormatSuggestions(suggestions))
        , type_name_(type_name) {}

private:
    static std::string FormatSuggestions(const std::vector<std::string>& sug) {
        if (sug.empty()) return "";
        return "Did you mean: " + JoinStrings(sug, ", ") + "?";
    }

    std::string type_name_;
};

} // namespace icarus
```

### Validation Integration

```cpp
// During loading, wrap Vulcan errors with context
SimulatorConfig SimulationLoader::Load(const std::string& path) {
    try {
        auto root = vulcan::io::YamlEnv::LoadWithIncludesAndEnv(path);
        return ParseRoot(root, path);
    } catch (const vulcan::io::YamlError& e) {
        // Re-throw with Icarus error type
        throw ConfigError(e.what(), path);
    } catch (const vulcan::io::EnvVarError& e) {
        throw ConfigError("Undefined environment variable: " + e.var_name(),
                          path, -1,
                          "Set the variable or use ${" + e.var_name() + ":default}");
    }
}
```

---

## ComponentConfig Type Mapping

### YAML to C++ Type Mapping

| YAML Section | C++ Type | Accessor |
|:-------------|:---------|:---------|
| `scalars:` | `double` | `cfg.Get<double>(key, default)` |
| `vectors:` | `std::vector<double>` | `cfg.Get<Vec3<double>>(key, default)` |
| `strings:` | `std::string` | `cfg.Get<std::string>(key, default)` |
| `integers:` | `int64_t` | `cfg.Get<int>(key, default)` |
| `booleans:` | `bool` | `cfg.Get<bool>(key, default)` |

### Janus Type Support

Vulcan's YAML system natively supports Janus types:

```yaml
components:
  - type: RigidBody6DOF
    name: EOM
    vectors:
      initial_position: [0, 0, 6.471e6]        # Vec3<double>
      initial_velocity: [7500, 0, 0]           # Vec3<double>
      initial_attitude: [1, 0, 0, 0]           # Quaternion (wxyz)
      inertia:                                  # Mat3<double>
        - [100, 0, 0]
        - [0, 200, 0]
        - [0, 0, 150]
```

```cpp
void Provision(Backplane& bp, const ComponentConfig& cfg) {
    // Vulcan handles conversion automatically
    auto pos = cfg.Get<janus::Vec3<double>>("initial_position", Vec3d::Zero());
    auto quat = cfg.Get<janus::Quaternion<double>>("initial_attitude", Quatd::Identity());
    auto inertia = cfg.Get<janus::Mat3<double>>("inertia", Mat3d::Identity());
}
```

---

## Entity Expansion

### Template Loading

```cpp
EntityTemplate SimulationLoader::LoadEntityTemplate(const vulcan::io::YamlNode& node) {
    EntityTemplate tmpl;

    tmpl.name = node["entity"].Require<std::string>("name");
    tmpl.description = node["entity"].Get<std::string>("description", "");

    // Parse internal components
    if (node["entity"].Has("components")) {
        node["entity"]["components"].ForEach([&](const vulcan::io::YamlNode& comp) {
            tmpl.components.push_back(ParseComponent(comp));
        });
    }

    // Parse internal routes (relative names)
    if (node["entity"].Has("routes")) {
        node["entity"]["routes"].ForEach([&](const vulcan::io::YamlNode& route) {
            signal::SignalRoute r;
            r.input_path = route.Require<std::string>("input");
            r.output_path = route.Require<std::string>("output");
            r.gain = route.Get<double>("gain", 1.0);
            tmpl.routes.push_back(r);
        });
    }

    // Parse scheduler
    if (node["entity"].Has("scheduler")) {
        tmpl.scheduler = ParseScheduler(node["entity"]["scheduler"]);
    }

    return tmpl;
}
```

### Entity Instantiation with Overrides

```cpp
void SimulationLoader::ExpandEntity(SimulatorConfig& cfg,
                                     const EntityInstance& instance,
                                     const EntityTemplate& tmpl) {
    // Expand each component with entity prefix
    for (auto comp_cfg : tmpl.components) {
        // Apply overrides
        if (instance.overrides.count(comp_cfg.name)) {
            comp_cfg = MergeConfigs(comp_cfg, instance.overrides.at(comp_cfg.name));
        }

        // Set entity prefix
        comp_cfg.entity = instance.name;

        cfg.components.push_back(comp_cfg);
    }

    // Expand internal routes (relative -> absolute)
    for (auto route : tmpl.routes) {
        route.input_path = instance.name + "." + route.input_path;
        route.output_path = instance.name + "." + route.output_path;
        cfg.routes.push_back(route);
    }
}

ComponentConfig SimulationLoader::MergeConfigs(const ComponentConfig& base,
                                                const ComponentConfig& overrides) {
    ComponentConfig merged = base;

    // Overrides replace (not merge) at the key level
    for (const auto& [k, v] : overrides.scalars) merged.scalars[k] = v;
    for (const auto& [k, v] : overrides.vectors) merged.vectors[k] = v;
    for (const auto& [k, v] : overrides.strings) merged.strings[k] = v;
    for (const auto& [k, v] : overrides.integers) merged.integers[k] = v;
    for (const auto& [k, v] : overrides.booleans) merged.booleans[k] = v;

    return merged;
}
```

---

## Scheduler Configuration

```yaml
scheduler:
  groups:
    - name: sensors
      rate_hz: 400
      priority: 1
      members:
        - component: IMU
          priority: 1
        - component: GPS
          priority: 2

    - name: gnc
      rate_hz: 100
      priority: 2
      members:
        - component: Guidance
          priority: 1
        - component: Autopilot
          priority: 2

    - name: dynamics
      rate_hz: 400
      priority: 3
      members:
        - component: Gravity
          priority: 1
        - component: Forces
          priority: 2
        - component: EOM
          priority: 3

  topology:
    mode: explicit
    cycle_detection: error
    log_order: true
```

```cpp
void SimulationLoader::ParseScheduler(SimulatorConfig& cfg,
                                       const vulcan::io::YamlNode& node) {
    if (node.Has("groups")) {
        node["groups"].ForEach([&](const vulcan::io::YamlNode& group_node) {
            SchedulerGroupConfig group;
            group.name = group_node.Require<std::string>("name");
            group.rate_hz = group_node.Get<double>("rate_hz", 100.0);
            group.priority = group_node.Get<int>("priority", 0);

            group_node["members"].ForEach([&](const vulcan::io::YamlNode& member) {
                GroupMember m;
                m.component = member.Require<std::string>("component");
                m.priority = member.Get<int>("priority", 0);
                group.members.push_back(m);
            });

            cfg.scheduler.groups.push_back(group);
        });
    }

    if (node.Has("topology")) {
        auto topo = node["topology"];
        auto mode_str = topo.Get<std::string>("mode", "explicit");
        cfg.scheduler.topology.mode =
            (mode_str == "automatic") ? SchedulingMode::Automatic
                                       : SchedulingMode::Explicit;
        auto cycle_str = topo.Get<std::string>("cycle_detection", "error");
        // ... parse other topology settings
    }
}
```

---

## Complete YAML Schema Reference

```yaml
# ===========================================================================
# ICARUS SIMULATION CONFIGURATION
# ===========================================================================

# ---------------------------------------------------------------------------
# Simulation Identity
# ---------------------------------------------------------------------------
simulation:
  name: string              # Required: Simulation name
  version: string           # Optional: Version string
  description: string       # Optional: Description

# ---------------------------------------------------------------------------
# Time Configuration
# ---------------------------------------------------------------------------
time:
  start: number             # Start time [s], default 0
  end: number               # End time [s], default 100
  dt: number                # Timestep [s], may be auto-derived from scheduler
  reference_epoch_jd: number  # Julian date for absolute time, default J2000

# ---------------------------------------------------------------------------
# Components (Single-File Mode)
# ---------------------------------------------------------------------------
components:
  - type: string            # Required: Component type name
    name: string            # Required: Instance name
    entity: string          # Optional: Entity prefix
    config_file: string     # Optional: External config file path
    scalars:
      key: number           # Scalar config values
    vectors:
      key: [x, y, z]        # Vec3 values
    strings:
      key: string           # String values
    integers:
      key: number           # Integer values
    booleans:
      key: bool             # Boolean values

# ---------------------------------------------------------------------------
# Entities (Multi-File Mode)
# ---------------------------------------------------------------------------
entities:
  - template: path.yaml     # Path to entity template (or !include)
    name: string            # Instance name (becomes signal prefix)
    overrides:
      ComponentName:        # Override values for specific components
        scalars:
          key: value
        vectors:
          key: [x, y, z]

# ---------------------------------------------------------------------------
# Swarms (Bulk Entity Instantiation)
# ---------------------------------------------------------------------------
swarms:
  - template: path.yaml     # Entity template
    name_prefix: string     # Name prefix (e.g., "Drone" -> Drone_000, Drone_001, ...)
    count: number           # Number of instances

# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
routes:                     # For single-file mode
  - input: string           # Destination: Component.signal or Entity.Component.signal
    output: string          # Source: Component.signal or Entity.Component.signal
    gain: number            # Optional: Scale factor, default 1.0
    offset: number          # Optional: Offset, default 0.0
    delay: number           # Optional: Transport delay [s], default 0.0

cross_entity_routes:        # For multi-file mode (between entities)
  - input: Entity.Component.signal
    output: Entity.Component.signal

# ---------------------------------------------------------------------------
# Scheduler
# ---------------------------------------------------------------------------
scheduler:
  groups:
    - name: string          # Group name
      rate_hz: number       # Execution rate [Hz]
      priority: number      # Group execution order (lower = first)
      members:
        - component: string # Component name
          priority: number  # Within-group order (lower = first)
  topology:
    mode: explicit | automatic  # Explicit: user-defined; Automatic: future
    cycle_detection: error | warn | break_at_delay
    log_order: bool

# ---------------------------------------------------------------------------
# Integrator
# ---------------------------------------------------------------------------
integrator:
  type: Euler | RK4 | RK45 | DormandPrince
  abs_tol: number           # For adaptive integrators
  rel_tol: number           # For adaptive integrators
  dt_min: number            # Minimum step size
  dt_max: number            # Maximum step size

# ---------------------------------------------------------------------------
# Staging (Future)
# ---------------------------------------------------------------------------
staging:
  trim:
    enabled: bool
    method: newton | ipopt
    zero_derivatives: [signal_paths]
    control_signals: [signal_paths]
    tolerance: number
    max_iterations: number
  linearization:
    enabled: bool
    states: [signal_paths]
    inputs: [signal_paths]
    outputs: [signal_paths]
  symbolics:
    enabled: bool
    generate_dynamics: bool
    generate_jacobian: bool
    output_dir: string

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging:
  console_level: Trace | Debug | Info | Warning | Error | Off
  file_enabled: bool
  file_path: string
  file_level: Trace | Debug | Info | Warning | Error | Off
  progress_enabled: bool
  profiling_enabled: bool
  telemetry_enabled: bool
  telemetry_path: string
  telemetry_signals: [signal_paths]

# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
output:
  directory: string
  data_dictionary: bool
  data_dictionary_format: yaml | json | csv
  telemetry: bool
  telemetry_format: hdf5 | csv | binary
  timing_report: bool
```

---

## Exit Criteria

- [ ] `SimulationLoader` class using Vulcan's YAML infrastructure
- [ ] Single-file mode: all config inline
- [ ] Multi-file mode: `!include` directive support via Vulcan
- [ ] Environment variable expansion via `YamlEnv`
- [ ] Error types: `ConfigError`, `MissingKeyError`, `TypeMismatchError`, `UnknownTypeError`
- [ ] Error messages include file path and hint
- [ ] Entity template loading and expansion
- [ ] Override merging (replace semantics)
- [ ] Route parsing with gain/offset/delay
- [ ] Scheduler group parsing
- [ ] Janus type support (Vec3, Quaternion, Mat3) via Vulcan
- [ ] Unit tests for all parsing paths
- [ ] Integration test: load complete sim from YAML
