#pragma once

/**
 * @file ConfigLoader.hpp
 * @brief Loads component configurations from YAML files
 *
 * Part of Phase 4.0: Configuration Infrastructure.
 * Uses Vulcan's YamlNode for parsing with Janus type support.
 */

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
 *
 * Example usage:
 * @code
 * auto configs = ConfigLoader::LoadSimulation("rocket.yaml");
 * for (const auto& cfg : configs) {
 *     auto component = factory.Create(cfg);
 *     sim.AddComponent(std::move(component), cfg);
 * }
 * @endcode
 */
class ConfigLoader {
  public:
    /**
     * @brief Load simulation config (multiple components)
     *
     * Expected YAML structure:
     * @code
     * simulation:
     *   name: "Simulation Name"
     *   dt: 0.01
     *   t_end: 100.0
     *
     * components:
     *   - type: ComponentType
     *     name: ComponentName
     *     entity: EntityName  # optional
     *     scalars:
     *       mass: 1000.0
     *     vectors:
     *       position: [0, 0, 6.4e6]
     * @endcode
     *
     * @param yaml_path Path to YAML configuration file
     * @return Vector of ComponentConfig for each component
     */
    static std::vector<ComponentConfig> LoadSimulation(const std::string &yaml_path);

    /**
     * @brief Load a single component config from a YamlNode
     *
     * @param node YamlNode containing component configuration
     * @return ComponentConfig populated from YAML
     */
    static ComponentConfig LoadComponent(const vulcan::io::YamlNode &node);

    /**
     * @brief Parse simulation config from YAML string
     *
     * @param yaml_content YAML content as string
     * @return Vector of ComponentConfig for each component
     */
    static std::vector<ComponentConfig> ParseSimulation(const std::string &yaml_content);
};

// =============================================================================
// Implementation
// =============================================================================

inline std::vector<ComponentConfig> ConfigLoader::LoadSimulation(const std::string &yaml_path) {
    auto root = vulcan::io::YamlNode::LoadFile(yaml_path);
    std::vector<ComponentConfig> configs;

    if (!root.Has("components")) {
        return configs; // Empty simulation
    }

    auto components = root["components"];
    for (std::size_t i = 0; i < components.Size(); ++i) {
        configs.push_back(LoadComponent(components[i]));
    }

    return configs;
}

inline std::vector<ComponentConfig> ConfigLoader::ParseSimulation(const std::string &yaml_content) {
    auto root = vulcan::io::YamlNode::Parse(yaml_content);
    std::vector<ComponentConfig> configs;

    if (!root.Has("components")) {
        return configs;
    }

    auto components = root["components"];
    for (std::size_t i = 0; i < components.Size(); ++i) {
        configs.push_back(LoadComponent(components[i]));
    }

    return configs;
}

inline ComponentConfig ConfigLoader::LoadComponent(const vulcan::io::YamlNode &node) {
    ComponentConfig cfg;

    // Required fields
    cfg.type = node.Require<std::string>("type");
    cfg.name = node.Require<std::string>("name");
    cfg.entity = node.Get<std::string>("entity", "");

    // Extract scalars
    if (node.Has("scalars")) {
        node["scalars"].ForEachEntry([&](const std::string &key, const vulcan::io::YamlNode &val) {
            cfg.scalars[key] = val.As<double>();
        });
    }

    // Extract vectors (Vec3, Vec4, etc.)
    if (node.Has("vectors")) {
        node["vectors"].ForEachEntry([&](const std::string &key, const vulcan::io::YamlNode &val) {
            cfg.vectors[key] = val.ToVector<double>();
        });
    }

    // Extract arrays (arbitrary length double arrays)
    if (node.Has("arrays")) {
        node["arrays"].ForEachEntry([&](const std::string &key, const vulcan::io::YamlNode &val) {
            cfg.arrays[key] = val.ToVector<double>();
        });
    }

    // Extract strings
    if (node.Has("strings")) {
        node["strings"].ForEachEntry([&](const std::string &key, const vulcan::io::YamlNode &val) {
            cfg.strings[key] = val.As<std::string>();
        });
    }

    // Extract integers
    if (node.Has("integers")) {
        node["integers"].ForEachEntry([&](const std::string &key, const vulcan::io::YamlNode &val) {
            cfg.integers[key] = val.As<int64_t>();
        });
    }

    // Extract booleans
    if (node.Has("booleans")) {
        node["booleans"].ForEachEntry([&](const std::string &key, const vulcan::io::YamlNode &val) {
            cfg.booleans[key] = val.As<bool>();
        });
    }

    // Extract sources list (for aggregators)
    if (node.Has("sources")) {
        cfg.sources = node["sources"].ToVector<std::string>();
    }

    return cfg;
}

} // namespace io
} // namespace icarus
