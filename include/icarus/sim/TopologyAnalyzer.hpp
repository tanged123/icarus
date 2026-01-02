#pragma once

/**
 * @file TopologyAnalyzer.hpp
 * @brief Dependency graph analysis and topological sorting for component scheduling
 *
 * Part of Phase 5: Advanced Configuration & Scheduling.
 * Implements automatic execution order from signal dependencies.
 */

#include <icarus/core/Error.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/sim/SimulatorConfig.hpp>

#include <algorithm>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace icarus {

/**
 * @brief Information about a detected cycle in the dependency graph
 */
struct CycleInfo {
    std::vector<std::string> components; ///< Components forming the cycle
    std::vector<std::string> signals;    ///< Signals involved in the cycle

    [[nodiscard]] std::string ToString() const {
        std::ostringstream oss;
        oss << "Cycle detected: ";
        for (size_t i = 0; i < components.size(); ++i) {
            oss << components[i];
            if (i < components.size() - 1) {
                oss << " -> ";
            }
        }
        if (!components.empty()) {
            oss << " -> " << components[0]; // Close the cycle
        }
        return oss.str();
    }
};

/**
 * @brief Result of topological analysis
 */
struct TopologyResult {
    std::vector<std::string> execution_order; ///< Sorted component names
    std::vector<CycleInfo> cycles;            ///< Detected cycles (if any)
    bool has_cycles = false;                  ///< Quick check for cycles

    [[nodiscard]] bool IsValid() const { return !has_cycles; }
};

/**
 * @brief Dependency graph for component execution ordering
 *
 * Builds a directed graph where:
 * - Nodes = Components
 * - Edges = Signal dependencies (A -> B means A must run before B)
 *
 * Supports topological sorting with cycle detection.
 */
class DependencyGraph {
  public:
    /**
     * @brief Add a dependency edge: producer must execute before consumer
     *
     * @param producer Component that produces the signal
     * @param consumer Component that consumes the signal
     * @param signal_name Signal connecting them (for diagnostics)
     */
    void AddEdge(const std::string &producer, const std::string &consumer,
                 const std::string &signal_name = "") {
        // Register both nodes
        if (adjacency_.find(producer) == adjacency_.end()) {
            adjacency_[producer] = {};
            in_degree_[producer] = 0;
        }
        if (adjacency_.find(consumer) == adjacency_.end()) {
            adjacency_[consumer] = {};
            in_degree_[consumer] = 0;
        }

        // Add edge (avoid duplicates)
        auto &edges = adjacency_[producer];
        bool already_exists = false;
        for (const auto &e : edges) {
            if (e.target == consumer) {
                already_exists = true;
                break;
            }
        }

        if (!already_exists) {
            edges.push_back({consumer, signal_name});
            in_degree_[consumer]++;
        }
    }

    /**
     * @brief Add a node without any edges
     *
     * Ensures components with no dependencies are included in the graph.
     *
     * @param component Component name
     */
    void AddNode(const std::string &component) {
        if (adjacency_.find(component) == adjacency_.end()) {
            adjacency_[component] = {};
            in_degree_[component] = 0;
        }
    }

    /**
     * @brief Get all nodes in the graph
     */
    [[nodiscard]] std::vector<std::string> GetNodes() const {
        std::vector<std::string> nodes;
        nodes.reserve(adjacency_.size());
        for (const auto &kv : adjacency_) {
            nodes.push_back(kv.first);
        }
        return nodes;
    }

    /**
     * @brief Perform topological sort using Kahn's algorithm
     *
     * @return TopologyResult with execution order and cycle info
     */
    [[nodiscard]] TopologyResult TopologicalSort() const {
        TopologyResult result;

        // Copy in-degrees for mutation
        std::unordered_map<std::string, int> in_deg = in_degree_;

        // Queue of nodes with no incoming edges
        std::queue<std::string> ready;
        for (const auto &kv : in_deg) {
            if (kv.second == 0) {
                ready.push(kv.first);
            }
        }

        // Process nodes
        while (!ready.empty()) {
            std::string node = ready.front();
            ready.pop();
            result.execution_order.push_back(node);

            // Reduce in-degree of neighbors
            auto it = adjacency_.find(node);
            if (it != adjacency_.end()) {
                for (const auto &edge : it->second) {
                    in_deg[edge.target]--;
                    if (in_deg[edge.target] == 0) {
                        ready.push(edge.target);
                    }
                }
            }
        }

        // Check for cycles
        if (result.execution_order.size() != adjacency_.size()) {
            result.has_cycles = true;
            result.cycles = DetectCycles();
        }

        return result;
    }

    /**
     * @brief Detect cycles using DFS
     *
     * @return List of cycles found in the graph
     */
    [[nodiscard]] std::vector<CycleInfo> DetectCycles() const {
        std::vector<CycleInfo> cycles;

        // Track visited state: 0=unvisited, 1=in-stack, 2=done
        std::unordered_map<std::string, int> state;
        for (const auto &kv : adjacency_) {
            state[kv.first] = 0;
        }

        std::vector<std::string> path;
        std::vector<std::string> path_signals;

        for (const auto &kv : adjacency_) {
            if (state[kv.first] == 0) {
                DetectCyclesDFS(kv.first, state, path, path_signals, cycles);
            }
        }

        return cycles;
    }

    /**
     * @brief Get number of nodes
     */
    [[nodiscard]] size_t Size() const { return adjacency_.size(); }

    /**
     * @brief Check if graph is empty
     */
    [[nodiscard]] bool Empty() const { return adjacency_.empty(); }

    /**
     * @brief Get dependents of a component (what runs after it)
     */
    [[nodiscard]] std::vector<std::string> GetDependents(const std::string &component) const {
        std::vector<std::string> deps;
        auto it = adjacency_.find(component);
        if (it != adjacency_.end()) {
            for (const auto &edge : it->second) {
                deps.push_back(edge.target);
            }
        }
        return deps;
    }

    /**
     * @brief Get dependencies of a component (what runs before it)
     */
    [[nodiscard]] std::vector<std::string> GetDependencies(const std::string &component) const {
        std::vector<std::string> deps;
        for (const auto &kv : adjacency_) {
            for (const auto &edge : kv.second) {
                if (edge.target == component) {
                    deps.push_back(kv.first);
                    break;
                }
            }
        }
        return deps;
    }

  private:
    struct Edge {
        std::string target;
        std::string signal; // For diagnostics
    };

    std::unordered_map<std::string, std::vector<Edge>> adjacency_;
    std::unordered_map<std::string, int> in_degree_;

    void DetectCyclesDFS(const std::string &node, std::unordered_map<std::string, int> &state,
                         std::vector<std::string> &path, std::vector<std::string> &path_signals,
                         std::vector<CycleInfo> &cycles) const {
        state[node] = 1; // In stack
        path.push_back(node);

        auto it = adjacency_.find(node);
        if (it != adjacency_.end()) {
            for (const auto &edge : it->second) {
                if (state[edge.target] == 1) {
                    // Found cycle - extract it
                    CycleInfo cycle;
                    bool in_cycle = false;
                    for (size_t i = 0; i < path.size(); ++i) {
                        if (path[i] == edge.target) {
                            in_cycle = true;
                        }
                        if (in_cycle) {
                            cycle.components.push_back(path[i]);
                            if (i < path_signals.size()) {
                                cycle.signals.push_back(path_signals[i]);
                            }
                        }
                    }
                    cycle.signals.push_back(edge.signal);
                    cycles.push_back(cycle);
                } else if (state[edge.target] == 0) {
                    path_signals.push_back(edge.signal);
                    DetectCyclesDFS(edge.target, state, path, path_signals, cycles);
                    path_signals.pop_back();
                }
            }
        }

        state[node] = 2; // Done
        path.pop_back();
    }
};

/**
 * @brief Utility class for building and analyzing component dependency graphs
 */
class TopologyAnalyzer {
  public:
    /**
     * @brief Build dependency graph from configuration
     *
     * Analyzes routes to determine which components depend on which.
     * A route from A.output to B.input creates edge A -> B.
     *
     * @param config Simulator configuration with components and routes
     * @return DependencyGraph representing component dependencies
     */
    [[nodiscard]] static DependencyGraph
    BuildGraph(const SimulatorConfig &config, const std::vector<signal::SignalRoute> &routes = {}) {
        DependencyGraph graph;

        // Build list of known component names for matching
        std::vector<std::string> sorted_names;
        for (const auto &comp : config.components) {
            std::string name = comp.FullPath();
            sorted_names.push_back(name);
            graph.AddNode(name);
        }

        // Sort by descending length to ensure we match the longest prefix first.
        // If lengths are equal, sort lexicographically for determinism.
        std::sort(sorted_names.begin(), sorted_names.end(),
                  [](const std::string &a, const std::string &b) {
                      if (a.size() != b.size()) {
                          return a.size() > b.size();
                      }
                      return a < b;
                  });

        // Use provided routes or config routes
        const auto &route_list = routes.empty() ? config.routes : routes;

        // Helper to find component name from signal path
        // Matches the longest known component prefix
        auto find_component = [&](const std::string &signal_path) -> std::string {
            // Try each component and see if signal starts with it
            for (const auto &comp_name : sorted_names) {
                if (signal_path.size() > comp_name.size() &&
                    signal_path.compare(0, comp_name.size(), comp_name) == 0 &&
                    signal_path[comp_name.size()] == '.') {
                    return comp_name;
                }
            }
            // Fallback: take first segment
            size_t dot = signal_path.find('.');
            return (dot != std::string::npos) ? signal_path.substr(0, dot) : signal_path;
        };

        // Add edges from routes
        for (const auto &route : route_list) {
            std::string producer = find_component(route.output_path);
            std::string consumer = find_component(route.input_path);

            if (!producer.empty() && !consumer.empty() && producer != consumer) {
                graph.AddEdge(producer, consumer, route.output_path);
            }
        }

        return graph;
    }

    /**
     * @brief Compute execution order from configuration
     *
     * @param config Simulator configuration
     * @param cycle_handling How to handle detected cycles
     * @return TopologyResult with execution order
     * @throws ConfigError if cycles detected and handling is Error
     */
    [[nodiscard]] static TopologyResult ComputeExecutionOrder(
        const SimulatorConfig &config,
        TopologyConfig::CycleHandling cycle_handling = TopologyConfig::CycleHandling::Error) {
        DependencyGraph graph = BuildGraph(config);
        TopologyResult result = graph.TopologicalSort();

        if (result.has_cycles) {
            switch (cycle_handling) {
            case TopologyConfig::CycleHandling::Error: {
                std::ostringstream oss;
                oss << "Cyclic dependencies detected in component graph:\n";
                for (const auto &cycle : result.cycles) {
                    oss << "  " << cycle.ToString() << "\n";
                }
                throw ConfigError(oss.str());
            }
            case TopologyConfig::CycleHandling::Warn:
                // Caller should log warning
                // Return partial order (whatever was resolved)
                break;
            case TopologyConfig::CycleHandling::BreakAtDelay:
                // TODO: Filter out edges with delay > 0 and retry
                // For now, treat as Warn
                break;
            }
        }

        return result;
    }

    /**
     * @brief Generate scheduler groups from topological order
     *
     * Creates a single group with all components in dependency order.
     *
     * @param result Topology result from ComputeExecutionOrder
     * @param group_name Name for the generated group
     * @param rate_hz Rate for the group
     * @return SchedulerGroupConfig with ordered members
     */
    [[nodiscard]] static SchedulerGroupConfig
    GenerateSchedulerGroup(const TopologyResult &result, const std::string &group_name = "auto",
                           double rate_hz = 400.0) {
        SchedulerGroupConfig group;
        group.name = group_name;
        group.rate_hz = rate_hz;
        group.priority = 0;

        int priority = 0;
        for (const auto &component : result.execution_order) {
            group.members.emplace_back(component, priority++);
        }

        return group;
    }

    /**
     * @brief Merge topology order into existing scheduler config
     *
     * Reorders members within each group based on topological sort.
     * Components not in any group are added to a new "unscheduled" group.
     *
     * @param config Scheduler config to modify
     * @param result Topology result
     */
    static void ApplyTopologyOrder(SchedulerConfig &config, const TopologyResult &result) {
        // Build priority map from topology order
        std::unordered_map<std::string, int> priority_map;
        for (size_t i = 0; i < result.execution_order.size(); ++i) {
            priority_map[result.execution_order[i]] = static_cast<int>(i);
        }

        // Track which components are already scheduled
        std::set<std::string> scheduled;

        // Reorder members within each group
        for (auto &group : config.groups) {
            for (auto &member : group.members) {
                auto it = priority_map.find(member.component);
                if (it != priority_map.end()) {
                    member.priority = it->second;
                    scheduled.insert(member.component);
                }
            }

            // Sort members by their topology priority
            std::sort(
                group.members.begin(), group.members.end(),
                [](const GroupMember &a, const GroupMember &b) { return a.priority < b.priority; });
        }

        // Add unscheduled components to a new group
        std::vector<std::string> unscheduled;
        for (const auto &comp : result.execution_order) {
            if (scheduled.find(comp) == scheduled.end()) {
                unscheduled.push_back(comp);
            }
        }

        if (!unscheduled.empty()) {
            SchedulerGroupConfig unscheduled_group;
            unscheduled_group.name = "unscheduled";
            unscheduled_group.rate_hz = config.groups.empty() ? 400.0 : config.groups[0].rate_hz;
            unscheduled_group.priority = static_cast<int>(config.groups.size());

            for (const auto &comp : unscheduled) {
                auto it = priority_map.find(comp);
                int prio = (it != priority_map.end()) ? it->second : 0;
                unscheduled_group.members.emplace_back(comp, prio);
            }

            config.groups.push_back(unscheduled_group);
        }
    }
};

} // namespace icarus
