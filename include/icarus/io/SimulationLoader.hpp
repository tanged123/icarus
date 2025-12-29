#pragma once

/**
 * @file SimulationLoader.hpp
 * @brief Loads complete simulation configuration from YAML
 *
 * Part of Phase 4.0.7: Configuration Infrastructure.
 * Renamed from ConfigLoader.hpp with extended functionality.
 *
 * Uses Vulcan's YAML infrastructure for:
 * - !include directive resolution
 * - ${VAR} environment variable expansion
 * - Type-safe value extraction
 */

#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/sim/SimulatorConfig.hpp>

#include <vulcan/io/YamlEnv.hpp>
#include <vulcan/io/YamlNode.hpp>

#include <string>
#include <vector>

namespace icarus::io {

/**
 * @brief Loads complete simulation configuration from YAML
 *
 * Supports:
 * - Single-file mode: all configuration inline
 * - Multi-file mode: !include directive for modular configs
 * - Environment variables: ${VAR} and ${VAR:default}
 * - Janus type support via Vulcan
 *
 * Example usage:
 * @code
 * auto config = SimulationLoader::Load("simulation.yaml");
 * Simulator<double> sim(config);
 * @endcode
 */
class SimulationLoader {
  public:
    // =========================================================================
    // Main Loading Methods
    // =========================================================================

    /**
     * @brief Load complete simulation config from file
     *
     * Uses Vulcan's YamlEnv::LoadWithIncludesAndEnv() for:
     * - !include directive resolution
     * - ${VAR} and ${VAR:default} expansion
     *
     * @param path Path to simulation YAML file
     * @return Complete SimulatorConfig
     * @throws ConfigError on parsing or validation errors
     */
    static SimulatorConfig Load(const std::string &path) {
        try {
            auto root = vulcan::io::YamlEnv::LoadWithIncludesAndEnv(path);
            return ParseRoot(root, path);
        } catch (const vulcan::io::EnvVarError &e) {
            // EnvVarError derives from YamlError, must catch first
            throw icarus::ConfigError("Undefined environment variable: " + e.var_name(), path, -1,
                                      "Set the variable or use ${" + e.var_name() + ":default}");
        } catch (const vulcan::io::YamlError &e) {
            throw icarus::ConfigError(e.what(), path);
        }
    }

    /**
     * @brief Parse simulation config from YAML string (for testing)
     *
     * @param yaml_content YAML content as string
     * @return Complete SimulatorConfig
     */
    static SimulatorConfig Parse(const std::string &yaml_content) {
        auto root = vulcan::io::YamlNode::Parse(yaml_content);
        return ParseRoot(root, "<string>");
    }

    // =========================================================================
    // Legacy Methods (for backward compatibility)
    // =========================================================================

    /**
     * @brief Load just the component list from a file
     * @deprecated Use Load() to get full SimulatorConfig
     */
    static std::vector<ComponentConfig> LoadComponents(const std::string &yaml_path) {
        auto root = vulcan::io::YamlNode::LoadFile(yaml_path);
        std::vector<ComponentConfig> configs;
        if (root.Has("components")) {
            ParseComponentList(configs, root["components"]);
        }
        return configs;
    }

    /**
     * @brief Load a single component config from a YamlNode
     */
    static ComponentConfig LoadComponent(const vulcan::io::YamlNode &node) {
        return ParseComponent(node);
    }

  private:
    // =========================================================================
    // Root Parsing
    // =========================================================================

    static SimulatorConfig ParseRoot(const vulcan::io::YamlNode &root,
                                     const std::string &source_path) {
        SimulatorConfig cfg;
        cfg.source_file = source_path;

        // Parse identity section
        if (root.Has("simulation")) {
            ParseSimulationSection(cfg, root["simulation"]);
        }

        // Parse time section
        if (root.Has("time")) {
            ParseTimeSection(cfg, root["time"]);
        }

        // Either components (single-file) or entities (multi-file) must exist
        bool has_components = root.Has("components");
        bool has_entities = root.Has("entities");

        if (!has_components && !has_entities) {
            throw icarus::ConfigError("Config must have either 'components' or 'entities' section",
                                      source_path);
        }

        if (has_components) {
            ParseComponentList(cfg.components, root["components"]);
        }

        // TODO: Entity expansion for multi-file mode
        // if (has_entities) {
        //     ParseEntities(cfg, root["entities"]);
        // }

        // Parse routes
        if (root.Has("routes")) {
            ParseRoutes(cfg.routes, root["routes"]);
        }
        if (root.Has("cross_entity_routes")) {
            ParseRoutes(cfg.routes, root["cross_entity_routes"]);
        }

        // Parse scheduler
        if (root.Has("scheduler")) {
            ParseScheduler(cfg.scheduler, root["scheduler"]);
        }

        // Parse integrator
        if (root.Has("integrator")) {
            ParseIntegrator(cfg.integrator, root["integrator"]);
        }

        // Parse logging
        if (root.Has("logging")) {
            ParseLogging(cfg.logging, root["logging"]);
        }

        // Parse staging
        if (root.Has("staging")) {
            ParseStaging(cfg.staging, root["staging"]);
        }

        // Parse output
        if (root.Has("output")) {
            ParseOutput(cfg.output, root["output"]);
        }

        return cfg;
    }

    // =========================================================================
    // Section Parsers
    // =========================================================================

    static void ParseSimulationSection(SimulatorConfig &cfg, const vulcan::io::YamlNode &node) {
        cfg.name = node.Get<std::string>("name", cfg.name);
        cfg.version = node.Get<std::string>("version", cfg.version);
        cfg.description = node.Get<std::string>("description", cfg.description);
    }

    static void ParseTimeSection(SimulatorConfig &cfg, const vulcan::io::YamlNode &node) {
        cfg.t_start = node.Get<double>("start", cfg.t_start);
        cfg.t_end = node.Get<double>("end", cfg.t_end);
        cfg.dt = node.Get<double>("dt", cfg.dt);
        cfg.reference_epoch_jd = node.Get<double>("reference_epoch_jd", cfg.reference_epoch_jd);
    }

    static void ParseComponentList(std::vector<ComponentConfig> &components,
                                   const vulcan::io::YamlNode &node) {
        node.ForEach([&](const vulcan::io::YamlNode &comp_node) {
            components.push_back(ParseComponent(comp_node));
        });
    }

    static ComponentConfig ParseComponent(const vulcan::io::YamlNode &node) {
        ComponentConfig cfg;

        // Required fields
        cfg.type = node.Require<std::string>("type");
        cfg.name = node.Require<std::string>("name");

        // Optional entity prefix
        cfg.entity = node.Get<std::string>("entity", "");

        // Extract scalars
        if (node.Has("scalars")) {
            node["scalars"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.scalars[key] = val.As<double>();
                });
        }

        // Extract vectors
        if (node.Has("vectors")) {
            node["vectors"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.vectors[key] = val.ToVector<double>();
                });
        }

        // Extract arrays
        if (node.Has("arrays")) {
            node["arrays"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.arrays[key] = val.ToVector<double>();
                });
        }

        // Extract strings
        if (node.Has("strings")) {
            node["strings"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.strings[key] = val.As<std::string>();
                });
        }

        // Extract integers
        if (node.Has("integers")) {
            node["integers"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.integers[key] = val.As<int64_t>();
                });
        }

        // Extract booleans
        if (node.Has("booleans")) {
            node["booleans"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.booleans[key] = val.As<bool>();
                });
        }

        // Extract sources list (for aggregators)
        if (node.Has("sources")) {
            cfg.sources = node["sources"].ToVector<std::string>();
        }

        return cfg;
    }

    static void ParseRoutes(std::vector<signal::SignalRoute> &routes,
                            const vulcan::io::YamlNode &node) {
        node.ForEach([&](const vulcan::io::YamlNode &route_node) {
            signal::SignalRoute route;
            route.input_path = route_node.Require<std::string>("input");
            route.output_path = route_node.Require<std::string>("output");
            route.gain = route_node.Get<double>("gain", 1.0);
            route.offset = route_node.Get<double>("offset", 0.0);
            route.delay = route_node.Get<double>("delay", 0.0);
            routes.push_back(route);
        });
    }

    static void ParseScheduler(SchedulerConfig &scheduler, const vulcan::io::YamlNode &node) {
        // Clear default groups if explicit groups are provided
        if (node.Has("groups")) {
            scheduler.groups.clear();
            node["groups"].ForEach([&](const vulcan::io::YamlNode &group_node) {
                SchedulerGroupConfig group;
                group.name = group_node.Require<std::string>("name");
                group.rate_hz = group_node.Get<double>("rate_hz", 100.0);
                group.priority = group_node.Get<int>("priority", 0);

                if (group_node.Has("members")) {
                    group_node["members"].ForEach([&](const vulcan::io::YamlNode &member_node) {
                        GroupMember m;
                        m.component = member_node.Require<std::string>("component");
                        m.priority = member_node.Get<int>("priority", 0);
                        group.members.push_back(m);
                    });
                }

                scheduler.groups.push_back(group);
            });
        }

        if (node.Has("topology")) {
            auto topo = node["topology"];
            auto mode_str = topo.Get<std::string>("mode", "explicit");
            scheduler.topology.mode =
                (mode_str == "automatic") ? SchedulingMode::Automatic : SchedulingMode::Explicit;

            auto cycle_str = topo.Get<std::string>("cycle_detection", "error");
            if (cycle_str == "warn") {
                scheduler.topology.cycle_detection = TopologyConfig::CycleHandling::Warn;
            } else if (cycle_str == "break_at_delay") {
                scheduler.topology.cycle_detection = TopologyConfig::CycleHandling::BreakAtDelay;
            } else {
                scheduler.topology.cycle_detection = TopologyConfig::CycleHandling::Error;
            }

            scheduler.topology.log_order =
                topo.Get<bool>("log_order", scheduler.topology.log_order);
        }
    }

    static void ParseIntegrator(IntegratorConfig<double> &integrator,
                                const vulcan::io::YamlNode &node) {
        auto type_str = node.Get<std::string>("type", "RK4");
        integrator.type = parse_integrator_type(type_str);
        integrator.abs_tol = node.Get<double>("abs_tol", integrator.abs_tol);
        integrator.rel_tol = node.Get<double>("rel_tol", integrator.rel_tol);
        integrator.min_dt = node.Get<double>("dt_min", integrator.min_dt);
        integrator.max_dt = node.Get<double>("dt_max", integrator.max_dt);
    }

    static void ParseLogging(LogConfig &logging, const vulcan::io::YamlNode &node) {
        // Console level
        if (node.Has("console_level")) {
            auto level_str = node.Require<std::string>("console_level");
            logging.console_level = ParseLogLevel(level_str);
        }

        // File logging
        logging.file_enabled = node.Get<bool>("file_enabled", logging.file_enabled);
        logging.file_path = node.Get<std::string>("file_path", logging.file_path);
        if (node.Has("file_level")) {
            logging.file_level = ParseLogLevel(node.Require<std::string>("file_level"));
        }

        // Features
        logging.progress_enabled = node.Get<bool>("progress_enabled", logging.progress_enabled);
        logging.profiling_enabled = node.Get<bool>("profiling_enabled", logging.profiling_enabled);

        // Telemetry
        logging.telemetry_enabled = node.Get<bool>("telemetry_enabled", logging.telemetry_enabled);
        logging.telemetry_path = node.Get<std::string>("telemetry_path", logging.telemetry_path);
        if (node.Has("telemetry_signals")) {
            logging.telemetry_signals = node["telemetry_signals"].ToVector<std::string>();
        }
    }

    static void ParseStaging(StageConfig &staging, const vulcan::io::YamlNode &node) {
        // Trim config
        if (node.Has("trim")) {
            auto trim = node["trim"];
            staging.trim.enabled = trim.Get<bool>("enabled", false);
            staging.trim.method = trim.Get<std::string>("method", staging.trim.method);
            staging.trim.tolerance = trim.Get<double>("tolerance", staging.trim.tolerance);
            staging.trim.max_iterations =
                trim.Get<int>("max_iterations", staging.trim.max_iterations);
            if (trim.Has("zero_derivatives")) {
                staging.trim.zero_derivatives = trim["zero_derivatives"].ToVector<std::string>();
            }
            if (trim.Has("control_signals")) {
                staging.trim.control_signals = trim["control_signals"].ToVector<std::string>();
            }
        }

        // Linearization config
        if (node.Has("linearization")) {
            auto lin = node["linearization"];
            staging.linearization.enabled = lin.Get<bool>("enabled", false);
            if (lin.Has("states")) {
                staging.linearization.states = lin["states"].ToVector<std::string>();
            }
            if (lin.Has("inputs")) {
                staging.linearization.inputs = lin["inputs"].ToVector<std::string>();
            }
            if (lin.Has("outputs")) {
                staging.linearization.outputs = lin["outputs"].ToVector<std::string>();
            }
            staging.linearization.export_matlab = lin.Get<bool>("export_matlab", false);
            staging.linearization.export_numpy = lin.Get<bool>("export_numpy", false);
            staging.linearization.export_json = lin.Get<bool>("export_json", false);
            staging.linearization.output_dir = lin.Get<std::string>("output_dir", "");
        }

        // Symbolics config
        if (node.Has("symbolics")) {
            auto sym = node["symbolics"];
            staging.symbolics.enabled = sym.Get<bool>("enabled", false);
            staging.symbolics.generate_dynamics = sym.Get<bool>("generate_dynamics", true);
            staging.symbolics.generate_jacobian = sym.Get<bool>("generate_jacobian", false);
            staging.symbolics.output_dir = sym.Get<std::string>("output_dir", "");
        }

        // Validation settings
        staging.validate_wiring = node.Get<bool>("validate_wiring", staging.validate_wiring);
        staging.warn_on_unwired = node.Get<bool>("warn_on_unwired", staging.warn_on_unwired);
    }

    static void ParseOutput(OutputConfig &output, const vulcan::io::YamlNode &node) {
        output.directory = node.Get<std::string>("directory", output.directory);
        output.data_dictionary = node.Get<bool>("data_dictionary", output.data_dictionary);
        output.data_dictionary_format =
            node.Get<std::string>("data_dictionary_format", output.data_dictionary_format);
        output.telemetry = node.Get<bool>("telemetry", output.telemetry);
        output.telemetry_format =
            node.Get<std::string>("telemetry_format", output.telemetry_format);
        output.timing_report = node.Get<bool>("timing_report", output.timing_report);
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    static LogLevel ParseLogLevel(const std::string &level_str) {
        if (level_str == "Trace" || level_str == "trace")
            return LogLevel::Trace;
        if (level_str == "Debug" || level_str == "debug")
            return LogLevel::Debug;
        if (level_str == "Info" || level_str == "info")
            return LogLevel::Info;
        if (level_str == "Warning" || level_str == "warning")
            return LogLevel::Warning;
        if (level_str == "Error" || level_str == "error")
            return LogLevel::Error;
        if (level_str == "Off" || level_str == "off")
            return LogLevel::Fatal; // Off not in enum, use Fatal
        return LogLevel::Info;      // Default
    }
};

} // namespace icarus::io

// =============================================================================
// SimulatorConfig::FromFile() implementation
// =============================================================================
// Defined here to avoid circular include issues

namespace icarus {

inline SimulatorConfig SimulatorConfig::FromFile(const std::string &path) {
    return io::SimulationLoader::Load(path);
}

} // namespace icarus
