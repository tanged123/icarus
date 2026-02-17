#pragma once

/**
 * @file IntrospectionGraph.hpp
 * @brief Full topology graph for block diagram visualization
 *
 * Extends DataDictionary with typed edges (route, resolve) for upstream
 * tools like Daedalus that need the complete signal flow graph.
 *
 * Route edges come from explicit SignalRouter connections.
 * Resolve edges come from Backplane::resolve() calls tracked per component,
 * capturing implicit source bindings (e.g., Vehicle6DOF reading Engine.force.x).
 *
 * Backward compatible: the existing DataDictionary schema is unchanged.
 * New "edges" array is additive.
 */

#include <icarus/io/data/DataDictionary.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace icarus {

/**
 * @brief Edge classification for the introspection graph
 */
enum class EdgeKind {
    Route,  ///< Explicit SignalRouter connection (input_path <- output_path)
    Resolve ///< Implicit dependency via Backplane::resolve() (source binding)
};

/**
 * @brief A single directed edge in the introspection graph
 */
struct IntrospectionEdge {
    std::string source; ///< Source signal path (e.g., "Rocket.Engine.force.x")
    std::string target; ///< Target: signal path (routes) or component name (resolves)
    EdgeKind kind;      ///< Edge classification
};

/**
 * @brief Complete introspection graph: nodes (components) + typed edges
 *
 * Composes with DataDictionary (which provides component/signal node data)
 * and adds topology edges for block diagram rendering.
 *
 * ## Edge Sources
 *
 * - **Route edges**: From SignalRouter — explicit input-to-output wiring
 * - **Resolve edges**: From Backplane::resolve() tracking — implicit source
 *   bindings where a component reads another component's output directly
 *   (e.g., Vehicle6DOF reading Engine.force.x via SignalHandle)
 *
 * ## JSON Schema
 *
 * Extends the DataDictionary schema with an additive "edges" array:
 * ```json
 * {
 *   "summary": { "...existing...", "total_edges": 15 },
 *   "components": [ "...unchanged..." ],
 *   "edges": [
 *     { "source": "Rocket.Engine.force.x", "target": "Rocket.Vehicle", "kind": "resolve" }
 *   ]
 * }
 * ```
 */
struct IntrospectionGraph {
    /// The existing data dictionary (component nodes with signal lists)
    DataDictionary dictionary;

    /// All topology edges (routes + resolves)
    std::vector<IntrospectionEdge> edges;

    /**
     * @brief Serialize the full graph to JSON
     *
     * Produces the existing DataDictionary schema with an additional
     * top-level "edges" array and "total_edges" in summary.
     */
    [[nodiscard]] nlohmann::json ToJSON() const {
        nlohmann::json j = dictionary.ToJSONValue();
        j["summary"]["total_edges"] = edges.size();

        // Edges (NEW — additive)
        j["edges"] = nlohmann::json::array();
        for (const auto &edge : edges) {
            nlohmann::json jedge;
            jedge["source"] = edge.source;
            jedge["target"] = edge.target;
            jedge["kind"] = (edge.kind == EdgeKind::Route) ? "route" : "resolve";
            j["edges"].push_back(jedge);
        }

        return j;
    }

    /**
     * @brief Write the full graph to a JSON file
     */
    void ToJSONFile(const std::string &path) const {
        detail::WriteToFile(path, [&](std::ofstream &file) { file << ToJSON().dump(2); });
    }

    /**
     * @brief Write the full graph to a YAML file
     */
    void ToYAMLFile(const std::string &path) const {
        YAML::Emitter out;
        out << YAML::BeginMap;

        // Summary
        out << YAML::Key << "summary" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "total_outputs" << YAML::Value << dictionary.total_outputs;
        out << YAML::Key << "total_inputs" << YAML::Value << dictionary.total_inputs;
        out << YAML::Key << "total_parameters" << YAML::Value << dictionary.total_parameters;
        out << YAML::Key << "total_config" << YAML::Value << dictionary.total_config;
        out << YAML::Key << "integrable_states" << YAML::Value << dictionary.integrable_states;
        out << YAML::Key << "unwired_inputs" << YAML::Value << dictionary.unwired_inputs;
        out << YAML::Key << "total_edges" << YAML::Value << edges.size();
        out << YAML::EndMap;

        dictionary.AppendComponentsToYAML(out);

        // Edges
        out << YAML::Key << "edges" << YAML::Value << YAML::BeginSeq;
        for (const auto &edge : edges) {
            out << YAML::BeginMap;
            out << YAML::Key << "source" << YAML::Value << edge.source;
            out << YAML::Key << "target" << YAML::Value << edge.target;
            out << YAML::Key << "kind" << YAML::Value
                << ((edge.kind == EdgeKind::Route) ? "route" : "resolve");
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        out << YAML::EndMap;

        detail::WriteToFile(path, [&](std::ofstream &file) { file << out.c_str(); });
    }
};

} // namespace icarus
