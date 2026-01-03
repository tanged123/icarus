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
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <vulcan/io/YamlEnv.hpp>
#include <vulcan/io/YamlNode.hpp>

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

    /**
     * @brief Load entity template from YAML node (public API)
     */
    static EntityTemplate LoadEntityTemplate(const vulcan::io::YamlNode &node) {
        // Handle both "entity:" wrapper and direct template
        if (node.Has("entity")) {
            return ParseEntityTemplateContent(node["entity"]);
        }
        return ParseEntityTemplateContent(node);
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
        bool has_swarms = root.Has("swarms");

        if (!has_components && !has_entities && !has_swarms) {
            throw icarus::ConfigError(
                "Config must have 'components', 'entities', or 'swarms' section", source_path);
        }

        if (has_components) {
            ParseComponentList(cfg.components, root["components"]);
        }

        // Entity expansion for multi-file mode
        if (has_entities) {
            ParseEntities(cfg, root["entities"], source_path);
        }

        // Swarm expansion for bulk entity spawning
        if (has_swarms) {
            ParseSwarms(cfg, root["swarms"], source_path);
        }

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

        // Parse recording
        if (root.Has("recording")) {
            ParseRecording(cfg.recording, root["recording"]);
        }

        // Parse staging
        if (root.Has("staging")) {
            ParseStaging(cfg.staging, root["staging"]);
        }

        // Parse phases
        if (root.Has("phases")) {
            ParsePhases(cfg.phases, root["phases"]);
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

        // Parse epoch configuration if present
        if (node.Has("epoch")) {
            auto epoch_node = node["epoch"];
            cfg.epoch.system = epoch_node.Get<std::string>("system", cfg.epoch.system);
            cfg.epoch.reference = epoch_node.Get<std::string>("reference", cfg.epoch.reference);
            cfg.epoch.jd = epoch_node.Get<double>("jd", cfg.epoch.jd);
            cfg.epoch.gps_week = epoch_node.Get<int>("week", cfg.epoch.gps_week);
            cfg.epoch.gps_seconds = epoch_node.Get<double>("seconds", cfg.epoch.gps_seconds);
        }
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

                // Parse active_phases for phase-based gating
                if (group_node.Has("active_phases")) {
                    auto phases = group_node["active_phases"].ToVector<int32_t>();
                    group.active_phases = std::set<int32_t>(phases.begin(), phases.end());
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

    static void ParseRecording(RecordingConfig &recording, const vulcan::io::YamlNode &node) {
        recording.enabled = node.Get<bool>("enabled", recording.enabled);
        recording.path = node.Get<std::string>("path", recording.path);
        recording.mode = node.Get<std::string>("mode", recording.mode);

        // Include patterns
        if (node.Has("include")) {
            recording.include = node["include"].ToVector<std::string>();
        }

        // Exclude patterns
        if (node.Has("exclude")) {
            recording.exclude = node["exclude"].ToVector<std::string>();
        }

        recording.include_derivatives =
            node.Get<bool>("include_derivatives", recording.include_derivatives);
        recording.include_inputs = node.Get<bool>("include_inputs", recording.include_inputs);
        recording.flush_interval = node.Get<int>("flush_interval", recording.flush_interval);
        recording.decimation = node.Get<int>("decimation", recording.decimation);
        recording.export_csv = node.Get<bool>("export_csv", recording.export_csv);
    }

    static void ParseStaging(StageConfig &staging, const vulcan::io::YamlNode &node) {
        // Trim config
        if (node.Has("trim")) {
            auto trim = node["trim"];
            staging.trim.enabled = trim.Get<bool>("enabled", false);
            staging.trim.mode = trim.Get<std::string>("mode", staging.trim.mode);
            staging.trim.method = trim.Get<std::string>("method", staging.trim.method);
            staging.trim.tolerance = trim.Get<double>("tolerance", staging.trim.tolerance);
            staging.trim.max_iterations =
                trim.Get<int>("max_iterations", staging.trim.max_iterations);

            // Equilibrium mode settings
            if (trim.Has("zero_derivatives")) {
                staging.trim.zero_derivatives = trim["zero_derivatives"].ToVector<std::string>();
            }
            if (trim.Has("control_signals")) {
                staging.trim.control_signals = trim["control_signals"].ToVector<std::string>();
            }

            // Warmstart mode settings
            staging.trim.recording_path =
                trim.Get<std::string>("recording_path", staging.trim.recording_path);
            staging.trim.resume_time = trim.Get<double>("resume_time", staging.trim.resume_time);
            staging.trim.validate_schema =
                trim.Get<bool>("validate_schema", staging.trim.validate_schema);
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

    static void ParsePhases(PhaseConfig &phases, const vulcan::io::YamlNode &node) {
        // Parse phase definitions: name -> integer value
        if (node.Has("definitions")) {
            node["definitions"].ForEachEntry(
                [&](const std::string &name, const vulcan::io::YamlNode &val) {
                    phases.definitions[name] = val.As<int32_t>();
                });
        }

        // Parse initial phase
        phases.initial_phase = node.Get<std::string>("initial", "");

        // Parse entity prefix (optional)
        phases.entity_prefix = node.Get<std::string>("entity_prefix", "");

        // Parse transitions
        if (node.Has("transitions")) {
            node["transitions"].ForEach([&](const vulcan::io::YamlNode &trans_node) {
                PhaseTransition trans;

                // Parse 'from' phase (can be string name or -1 for any)
                if (trans_node.Has("from")) {
                    auto from_str = trans_node.Get<std::string>("from", "");
                    if (from_str == "*" || from_str == "any") {
                        trans.from_phase = -1;
                    } else {
                        auto it = phases.definitions.find(from_str);
                        if (it != phases.definitions.end()) {
                            trans.from_phase = it->second;
                        } else {
                            // Try parsing as integer
                            trans.from_phase = std::stoi(from_str);
                        }
                    }
                }

                // Parse 'to' phase
                auto to_str = trans_node.Require<std::string>("to");
                auto to_it = phases.definitions.find(to_str);
                if (to_it != phases.definitions.end()) {
                    trans.to_phase = to_it->second;
                } else {
                    // Try parsing as integer
                    trans.to_phase = std::stoi(to_str);
                }

                // Parse condition
                trans.condition = trans_node.Get<std::string>("condition", "");

                phases.transitions.push_back(trans);
            });
        }
    }

    // =========================================================================
    // Entity Parsing and Expansion
    // =========================================================================

    /**
     * @brief Parse entities section and expand to flat components
     */
    static void ParseEntities(SimulatorConfig &cfg, const vulcan::io::YamlNode &node,
                              const std::string &source_path) {
        node.ForEach([&](const vulcan::io::YamlNode &entity_node) {
            // Parse entity instance
            EntityInstance instance;
            instance.name = entity_node.Require<std::string>("name");

            // Load template (inline or from file via !include)
            if (entity_node.Has("template")) {
                // Template is either a YamlNode (from !include) or needs to be loaded
                auto template_node = entity_node["template"];
                instance.entity_template = LoadEntityTemplate(template_node);
            } else if (entity_node.Has("entity")) {
                // Inline entity definition
                instance.entity_template = LoadEntityTemplate(entity_node);
            } else {
                throw icarus::ConfigError("Entity instance must have 'template' or inline 'entity'",
                                          source_path);
            }

            // Parse overrides
            if (entity_node.Has("overrides")) {
                entity_node["overrides"].ForEachEntry(
                    [&](const std::string &comp_name, const vulcan::io::YamlNode &override_node) {
                        instance.overrides[comp_name] = ParseComponentOverride(override_node);
                    });
            }

            // Expand this entity into flat components and routes
            ExpandEntity(cfg, instance);
        });
    }

    /**
     * @brief Parse swarms section and expand to flat components
     */
    static void ParseSwarms(SimulatorConfig &cfg, const vulcan::io::YamlNode &node,
                            const std::string &source_path) {
        node.ForEach([&](const vulcan::io::YamlNode &swarm_node) {
            SwarmConfig swarm;
            swarm.name_prefix = swarm_node.Require<std::string>("name_prefix");
            swarm.count = swarm_node.Get<int>("count", 1);

            // Load template
            if (swarm_node.Has("template")) {
                swarm.entity_template = LoadEntityTemplate(swarm_node["template"]);
            } else if (swarm_node.Has("entity")) {
                swarm.entity_template = LoadEntityTemplate(swarm_node);
            } else {
                throw icarus::ConfigError("Swarm must have 'template' or inline 'entity'",
                                          source_path);
            }

            // Expand swarm into entity instances
            ExpandSwarm(cfg, swarm);
        });
    }

    /**
     * @brief Expand swarm into multiple entity instances
     *
     * Generates count copies with names: prefix_000, prefix_001, etc.
     */
    static void ExpandSwarm(SimulatorConfig &cfg, const SwarmConfig &swarm) {
        for (int i = 0; i < swarm.count; ++i) {
            // Generate instance name with zero-padded index
            std::ostringstream oss;
            oss << swarm.name_prefix << "_" << std::setfill('0') << std::setw(3) << i;
            std::string instance_name = oss.str();

            // Create entity instance (no overrides for swarm copies)
            EntityInstance instance;
            instance.entity_template = swarm.entity_template;
            instance.name = instance_name;

            // Expand as normal entity
            ExpandEntity(cfg, instance);
        }
    }

    /**
     * @brief Parse entity template content
     */
    static EntityTemplate ParseEntityTemplateContent(const vulcan::io::YamlNode &content) {
        EntityTemplate tmpl;

        tmpl.name = content.Get<std::string>("name", "");
        tmpl.description = content.Get<std::string>("description", "");

        // Parse components
        if (content.Has("components")) {
            ParseComponentList(tmpl.components, content["components"]);
        }

        // Parse internal routes
        if (content.Has("routes")) {
            ParseRoutes(tmpl.routes, content["routes"]);
        }

        // Parse scheduler
        if (content.Has("scheduler")) {
            ParseScheduler(tmpl.scheduler, content["scheduler"]);
        }

        // Parse staging
        if (content.Has("staging")) {
            ParseStaging(tmpl.staging, content["staging"]);
        }

        return tmpl;
    }

    /**
     * @brief Parse component override (partial ComponentConfig)
     */
    static ComponentConfig ParseComponentOverride(const vulcan::io::YamlNode &node) {
        ComponentConfig cfg;

        // Only parse the override fields that are present
        if (node.Has("scalars")) {
            node["scalars"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.scalars[key] = val.As<double>();
                });
        }

        if (node.Has("vectors")) {
            node["vectors"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.vectors[key] = val.ToVector<double>();
                });
        }

        if (node.Has("strings")) {
            node["strings"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.strings[key] = val.As<std::string>();
                });
        }

        if (node.Has("integers")) {
            node["integers"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.integers[key] = val.As<int64_t>();
                });
        }

        if (node.Has("booleans")) {
            node["booleans"].ForEachEntry(
                [&](const std::string &key, const vulcan::io::YamlNode &val) {
                    cfg.booleans[key] = val.As<bool>();
                });
        }

        return cfg;
    }

    /**
     * @brief Expand entity instance into flat components and routes
     */
    static void ExpandEntity(SimulatorConfig &cfg, const EntityInstance &instance) {
        const auto &tmpl = instance.entity_template;

        // Expand components with entity prefix
        for (auto comp_cfg : tmpl.components) {
            // Apply overrides if present
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

        // Merge scheduler groups (prefix with entity name)
        for (auto group : tmpl.scheduler.groups) {
            group.name = instance.name + "." + group.name;
            for (auto &member : group.members) {
                member.component = instance.name + "." + member.component;
            }
            cfg.scheduler.groups.push_back(group);
        }
    }

    /**
     * @brief Merge override config into base config
     */
    static ComponentConfig MergeConfigs(const ComponentConfig &base,
                                        const ComponentConfig &overrides) {
        ComponentConfig merged = base;

        // Overrides replace at the key level
        for (const auto &[k, v] : overrides.scalars)
            merged.scalars[k] = v;
        for (const auto &[k, v] : overrides.vectors)
            merged.vectors[k] = v;
        for (const auto &[k, v] : overrides.strings)
            merged.strings[k] = v;
        for (const auto &[k, v] : overrides.integers)
            merged.integers[k] = v;
        for (const auto &[k, v] : overrides.booleans)
            merged.booleans[k] = v;

        return merged;
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

inline EntityTemplate EntityTemplate::FromFile(const std::string &yaml_path) {
    auto root = vulcan::io::YamlEnv::LoadWithIncludesAndEnv(yaml_path);
    return io::SimulationLoader::LoadEntityTemplate(root);
}

inline std::tuple<std::vector<ComponentConfig>, std::vector<signal::SignalRoute>, SchedulerConfig>
EntitySystemConfig::ExpandAll() const {
    std::vector<ComponentConfig> expanded_components;
    std::vector<signal::SignalRoute> expanded_routes;
    SchedulerConfig merged_scheduler;

    // Helper to merge overrides
    auto merge_configs = [](const ComponentConfig &base, const ComponentConfig &overrides) {
        ComponentConfig merged = base;
        for (const auto &[k, v] : overrides.scalars)
            merged.scalars[k] = v;
        for (const auto &[k, v] : overrides.vectors)
            merged.vectors[k] = v;
        for (const auto &[k, v] : overrides.strings)
            merged.strings[k] = v;
        for (const auto &[k, v] : overrides.integers)
            merged.integers[k] = v;
        for (const auto &[k, v] : overrides.booleans)
            merged.booleans[k] = v;
        return merged;
    };

    // Expand entities
    for (const auto &instance : entities) {
        const auto &tmpl = instance.entity_template;

        // Expand components with entity prefix
        for (auto comp_cfg : tmpl.components) {
            if (instance.overrides.count(comp_cfg.name)) {
                comp_cfg = merge_configs(comp_cfg, instance.overrides.at(comp_cfg.name));
            }
            comp_cfg.entity = instance.name;
            expanded_components.push_back(comp_cfg);
        }

        // Expand internal routes
        for (auto route : tmpl.routes) {
            route.input_path = instance.name + "." + route.input_path;
            route.output_path = instance.name + "." + route.output_path;
            expanded_routes.push_back(route);
        }

        // Merge scheduler groups
        for (auto group : tmpl.scheduler.groups) {
            group.name = instance.name + "." + group.name;
            for (auto &member : group.members) {
                member.component = instance.name + "." + member.component;
            }
            merged_scheduler.groups.push_back(group);
        }
    }

    // Expand swarms
    for (const auto &swarm : swarms) {
        for (int i = 0; i < swarm.count; ++i) {
            std::ostringstream oss;
            oss << swarm.name_prefix << "_" << std::setfill('0') << std::setw(3) << i;
            std::string instance_name = oss.str();

            // Expand components
            for (auto comp_cfg : swarm.entity_template.components) {
                comp_cfg.entity = instance_name;
                expanded_components.push_back(comp_cfg);
            }

            // Expand routes
            for (auto route : swarm.entity_template.routes) {
                route.input_path = instance_name + "." + route.input_path;
                route.output_path = instance_name + "." + route.output_path;
                expanded_routes.push_back(route);
            }

            // Merge scheduler groups
            for (auto group : swarm.entity_template.scheduler.groups) {
                group.name = instance_name + "." + group.name;
                for (auto &member : group.members) {
                    member.component = instance_name + "." + member.component;
                }
                merged_scheduler.groups.push_back(group);
            }
        }
    }

    // Add cross-entity routes (already absolute)
    for (const auto &route : cross_entity_routes) {
        expanded_routes.push_back(route);
    }

    return {expanded_components, expanded_routes, merged_scheduler};
}

} // namespace icarus
