/**
 * @file test_topology_analyzer.cpp
 * @brief Unit tests for TopologyAnalyzer and DependencyGraph
 *
 * Part of Phase 5: Advanced Configuration & Scheduling
 */

#include <gtest/gtest.h>
#include <icarus/sim/TopologyAnalyzer.hpp>

namespace icarus {
namespace {

// Helper to create ComponentConfig
ComponentConfig MakeComponent(const std::string &name, const std::string &entity,
                              const std::string &type) {
    ComponentConfig cfg;
    cfg.name = name;
    cfg.entity = entity;
    cfg.type = type;
    return cfg;
}

// =============================================================================
// DependencyGraph Basic Tests
// =============================================================================

TEST(DependencyGraphTest, EmptyGraphHasNoNodes) {
    DependencyGraph graph;
    EXPECT_TRUE(graph.Empty());
    EXPECT_EQ(graph.Size(), 0u);
}

TEST(DependencyGraphTest, AddNodeCreatesNode) {
    DependencyGraph graph;
    graph.AddNode("ComponentA");

    EXPECT_FALSE(graph.Empty());
    EXPECT_EQ(graph.Size(), 1u);

    auto nodes = graph.GetNodes();
    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes[0], "ComponentA");
}

TEST(DependencyGraphTest, AddEdgeCreatesBothNodes) {
    DependencyGraph graph;
    graph.AddEdge("Producer", "Consumer", "signal");

    EXPECT_EQ(graph.Size(), 2u);

    auto nodes = graph.GetNodes();
    EXPECT_EQ(nodes.size(), 2u);
}

TEST(DependencyGraphTest, AddDuplicateEdgeIgnored) {
    DependencyGraph graph;
    graph.AddEdge("A", "B", "sig1");
    graph.AddEdge("A", "B", "sig2"); // Duplicate edge

    // Still only 2 nodes
    EXPECT_EQ(graph.Size(), 2u);

    // B has only one dependency
    auto deps = graph.GetDependencies("B");
    EXPECT_EQ(deps.size(), 1u);
}

// =============================================================================
// Topological Sort Tests
// =============================================================================

TEST(DependencyGraphTest, SingleNodeSorts) {
    DependencyGraph graph;
    graph.AddNode("A");

    auto result = graph.TopologicalSort();

    EXPECT_TRUE(result.IsValid());
    EXPECT_FALSE(result.has_cycles);
    ASSERT_EQ(result.execution_order.size(), 1u);
    EXPECT_EQ(result.execution_order[0], "A");
}

TEST(DependencyGraphTest, LinearChainSortsCorrectly) {
    DependencyGraph graph;
    // A -> B -> C
    graph.AddEdge("A", "B");
    graph.AddEdge("B", "C");

    auto result = graph.TopologicalSort();

    EXPECT_TRUE(result.IsValid());
    EXPECT_FALSE(result.has_cycles);
    ASSERT_EQ(result.execution_order.size(), 3u);

    // A must come before B, B before C
    auto find_pos = [&](const std::string &name) {
        for (size_t i = 0; i < result.execution_order.size(); ++i) {
            if (result.execution_order[i] == name)
                return i;
        }
        return size_t(-1);
    };

    EXPECT_LT(find_pos("A"), find_pos("B"));
    EXPECT_LT(find_pos("B"), find_pos("C"));
}

TEST(DependencyGraphTest, DiamondPatternSortsCorrectly) {
    DependencyGraph graph;
    //     A
    //    / \
    //   B   C
    //    \ /
    //     D
    graph.AddEdge("A", "B");
    graph.AddEdge("A", "C");
    graph.AddEdge("B", "D");
    graph.AddEdge("C", "D");

    auto result = graph.TopologicalSort();

    EXPECT_TRUE(result.IsValid());
    EXPECT_FALSE(result.has_cycles);
    ASSERT_EQ(result.execution_order.size(), 4u);

    // Verify ordering constraints
    auto find_pos = [&](const std::string &name) {
        for (size_t i = 0; i < result.execution_order.size(); ++i) {
            if (result.execution_order[i] == name)
                return i;
        }
        return size_t(-1);
    };

    EXPECT_LT(find_pos("A"), find_pos("B"));
    EXPECT_LT(find_pos("A"), find_pos("C"));
    EXPECT_LT(find_pos("B"), find_pos("D"));
    EXPECT_LT(find_pos("C"), find_pos("D"));
}

TEST(DependencyGraphTest, DisconnectedComponentsAllIncluded) {
    DependencyGraph graph;
    // Two separate chains: A->B and C->D
    graph.AddEdge("A", "B");
    graph.AddEdge("C", "D");

    auto result = graph.TopologicalSort();

    EXPECT_TRUE(result.IsValid());
    ASSERT_EQ(result.execution_order.size(), 4u);

    // Both chains respect their internal ordering
    auto find_pos = [&](const std::string &name) {
        for (size_t i = 0; i < result.execution_order.size(); ++i) {
            if (result.execution_order[i] == name)
                return i;
        }
        return size_t(-1);
    };

    EXPECT_LT(find_pos("A"), find_pos("B"));
    EXPECT_LT(find_pos("C"), find_pos("D"));
}

// =============================================================================
// Cycle Detection Tests
// =============================================================================

TEST(DependencyGraphTest, SimpleCycleDetected) {
    DependencyGraph graph;
    // A -> B -> A (cycle)
    graph.AddEdge("A", "B");
    graph.AddEdge("B", "A");

    auto result = graph.TopologicalSort();

    EXPECT_FALSE(result.IsValid());
    EXPECT_TRUE(result.has_cycles);
    EXPECT_FALSE(result.cycles.empty());
}

TEST(DependencyGraphTest, TriangleCycleDetected) {
    DependencyGraph graph;
    // A -> B -> C -> A
    graph.AddEdge("A", "B");
    graph.AddEdge("B", "C");
    graph.AddEdge("C", "A");

    auto result = graph.TopologicalSort();

    EXPECT_FALSE(result.IsValid());
    EXPECT_TRUE(result.has_cycles);
}

TEST(DependencyGraphTest, CycleWithTailDetected) {
    DependencyGraph graph;
    // D -> A -> B -> C -> A (cycle with D as entry point)
    graph.AddEdge("D", "A");
    graph.AddEdge("A", "B");
    graph.AddEdge("B", "C");
    graph.AddEdge("C", "A");

    auto result = graph.TopologicalSort();

    EXPECT_FALSE(result.IsValid());
    EXPECT_TRUE(result.has_cycles);
}

TEST(DependencyGraphTest, CycleInfoContainsCycleComponents) {
    DependencyGraph graph;
    graph.AddEdge("A", "B");
    graph.AddEdge("B", "A");

    auto cycles = graph.DetectCycles();

    EXPECT_FALSE(cycles.empty());
    // Cycle should contain both A and B
    bool has_a = false, has_b = false;
    for (const auto &cycle : cycles) {
        for (const auto &comp : cycle.components) {
            if (comp == "A")
                has_a = true;
            if (comp == "B")
                has_b = true;
        }
    }
    EXPECT_TRUE(has_a);
    EXPECT_TRUE(has_b);
}

// =============================================================================
// Dependency Query Tests
// =============================================================================

TEST(DependencyGraphTest, GetDependentsReturnsConsumers) {
    DependencyGraph graph;
    graph.AddEdge("A", "B");
    graph.AddEdge("A", "C");
    graph.AddEdge("B", "D");

    auto deps = graph.GetDependents("A");
    EXPECT_EQ(deps.size(), 2u);

    // A's dependents are B and C
    bool has_b = std::find(deps.begin(), deps.end(), "B") != deps.end();
    bool has_c = std::find(deps.begin(), deps.end(), "C") != deps.end();
    EXPECT_TRUE(has_b);
    EXPECT_TRUE(has_c);
}

TEST(DependencyGraphTest, GetDependenciesReturnsProducers) {
    DependencyGraph graph;
    graph.AddEdge("A", "C");
    graph.AddEdge("B", "C");

    auto deps = graph.GetDependencies("C");
    EXPECT_EQ(deps.size(), 2u);

    // C depends on A and B
    bool has_a = std::find(deps.begin(), deps.end(), "A") != deps.end();
    bool has_b = std::find(deps.begin(), deps.end(), "B") != deps.end();
    EXPECT_TRUE(has_a);
    EXPECT_TRUE(has_b);
}

// =============================================================================
// TopologyAnalyzer BuildGraph Tests
// =============================================================================

TEST(TopologyAnalyzerTest, BuildGraphFromEmptyConfig) {
    SimulatorConfig config;

    auto graph = TopologyAnalyzer::BuildGraph(config);

    EXPECT_TRUE(graph.Empty());
}

TEST(TopologyAnalyzerTest, BuildGraphAddsAllComponents) {
    SimulatorConfig config;
    config.components.push_back(MakeComponent("EOM", "", "RigidBody6DOF"));
    config.components.push_back(MakeComponent("Aero", "", "AeroBody"));
    config.components.push_back(MakeComponent("Engine", "", "JetEngine"));

    auto graph = TopologyAnalyzer::BuildGraph(config);

    EXPECT_EQ(graph.Size(), 3u);
}

TEST(TopologyAnalyzerTest, BuildGraphFromRoutes) {
    SimulatorConfig config;
    config.components.push_back(MakeComponent("EOM", "", "RigidBody6DOF"));
    config.components.push_back(MakeComponent("Aero", "", "AeroBody"));
    config.components.push_back(MakeComponent("Forces", "", "ForceAggregator"));

    // Aero.lift -> Forces.input
    // Forces.total -> EOM.force
    config.routes.push_back(signal::SignalRoute{"Forces.input_lift", "Aero.lift"});
    config.routes.push_back(signal::SignalRoute{"EOM.force", "Forces.total"});

    auto graph = TopologyAnalyzer::BuildGraph(config);

    // Aero -> Forces -> EOM
    auto result = graph.TopologicalSort();
    EXPECT_TRUE(result.IsValid());

    auto find_pos = [&](const std::string &name) {
        for (size_t i = 0; i < result.execution_order.size(); ++i) {
            if (result.execution_order[i] == name)
                return i;
        }
        return size_t(-1);
    };

    EXPECT_LT(find_pos("Aero"), find_pos("Forces"));
    EXPECT_LT(find_pos("Forces"), find_pos("EOM"));
}

TEST(TopologyAnalyzerTest, BuildGraphWithEntityPrefix) {
    SimulatorConfig config;
    config.components.push_back(MakeComponent("EOM", "Rocket", "RigidBody6DOF"));
    config.components.push_back(MakeComponent("Aero", "Rocket", "AeroBody"));

    // Rocket.Aero.lift -> Rocket.EOM.force
    config.routes.push_back(signal::SignalRoute{"Rocket.EOM.force", "Rocket.Aero.lift"});

    auto graph = TopologyAnalyzer::BuildGraph(config);

    auto result = graph.TopologicalSort();
    EXPECT_TRUE(result.IsValid());

    auto find_pos = [&](const std::string &name) {
        for (size_t i = 0; i < result.execution_order.size(); ++i) {
            if (result.execution_order[i] == name)
                return i;
        }
        return size_t(-1);
    };

    EXPECT_LT(find_pos("Rocket.Aero"), find_pos("Rocket.EOM"));
}

// =============================================================================
// TopologyAnalyzer ComputeExecutionOrder Tests
// =============================================================================

TEST(TopologyAnalyzerTest, ComputeExecutionOrderThrowsOnCycle) {
    SimulatorConfig config;
    config.components.push_back(MakeComponent("A", "", "TypeA"));
    config.components.push_back(MakeComponent("B", "", "TypeB"));

    // A -> B -> A (cycle)
    config.routes.push_back(signal::SignalRoute{"B.input", "A.output"});
    config.routes.push_back(signal::SignalRoute{"A.input", "B.output"});

    EXPECT_THROW(
        (void)TopologyAnalyzer::ComputeExecutionOrder(config, TopologyConfig::CycleHandling::Error),
        ConfigError);
}

TEST(TopologyAnalyzerTest, ComputeExecutionOrderWarnOnCycle) {
    SimulatorConfig config;
    config.components.push_back(MakeComponent("A", "", "TypeA"));
    config.components.push_back(MakeComponent("B", "", "TypeB"));

    // A -> B -> A (cycle)
    config.routes.push_back(signal::SignalRoute{"B.input", "A.output"});
    config.routes.push_back(signal::SignalRoute{"A.input", "B.output"});

    // Should not throw with Warn
    auto result =
        TopologyAnalyzer::ComputeExecutionOrder(config, TopologyConfig::CycleHandling::Warn);
    EXPECT_TRUE(result.has_cycles);
}

// =============================================================================
// TopologyAnalyzer GenerateSchedulerGroup Tests
// =============================================================================

TEST(TopologyAnalyzerTest, GenerateSchedulerGroupFromOrder) {
    TopologyResult result;
    result.execution_order = {"A", "B", "C"};
    result.has_cycles = false;

    auto group = TopologyAnalyzer::GenerateSchedulerGroup(result, "auto_group", 200.0);

    EXPECT_EQ(group.name, "auto_group");
    EXPECT_DOUBLE_EQ(group.rate_hz, 200.0);
    EXPECT_EQ(group.priority, 0);

    ASSERT_EQ(group.members.size(), 3u);
    EXPECT_EQ(group.members[0].component, "A");
    EXPECT_EQ(group.members[0].priority, 0);
    EXPECT_EQ(group.members[1].component, "B");
    EXPECT_EQ(group.members[1].priority, 1);
    EXPECT_EQ(group.members[2].component, "C");
    EXPECT_EQ(group.members[2].priority, 2);
}

// =============================================================================
// TopologyAnalyzer ApplyTopologyOrder Tests
// =============================================================================

TEST(TopologyAnalyzerTest, ApplyTopologyOrderReordersMembersWithinGroup) {
    SchedulerConfig sched_config;
    SchedulerGroupConfig group{"dynamics", 400.0, 1};
    group.members = {{"C", 0}, {"A", 0}, {"B", 0}}; // Unordered
    sched_config.groups.push_back(group);

    TopologyResult result;
    result.execution_order = {"A", "B", "C"};
    result.has_cycles = false;

    TopologyAnalyzer::ApplyTopologyOrder(sched_config, result);

    // Members should now be ordered A, B, C
    auto &members = sched_config.groups[0].members;
    ASSERT_EQ(members.size(), 3u);
    EXPECT_EQ(members[0].component, "A");
    EXPECT_EQ(members[1].component, "B");
    EXPECT_EQ(members[2].component, "C");
}

TEST(TopologyAnalyzerTest, ApplyTopologyOrderAddsUnscheduledGroup) {
    SchedulerConfig sched_config;
    SchedulerGroupConfig group{"dynamics", 400.0, 1};
    group.members = {{"A", 0}}; // Only A is scheduled
    sched_config.groups.push_back(group);

    TopologyResult result;
    result.execution_order = {"A", "B", "C"}; // B and C are not in any group
    result.has_cycles = false;

    TopologyAnalyzer::ApplyTopologyOrder(sched_config, result);

    // Should have added an "unscheduled" group for B and C
    ASSERT_EQ(sched_config.groups.size(), 2u);
    EXPECT_EQ(sched_config.groups[1].name, "unscheduled");
    EXPECT_EQ(sched_config.groups[1].members.size(), 2u);
}

// =============================================================================
// Prefix Matching Tests
// =============================================================================

TEST(TopologyAnalyzerTest, PrefersLongestComponentMatch) {
    SimulatorConfig config;
    // We have a component "Body" and a component "Body.Sub"
    config.components.push_back(MakeComponent("Body", "", "TypeA"));
    config.components.push_back(MakeComponent("Sub", "Body", "TypeB")); // FullPath: Body.Sub
    config.components.push_back(MakeComponent("Consumer", "", "TypeC"));

    // Route from Body.Sub.output to Consumer.input
    config.routes.push_back(signal::SignalRoute{"Consumer.input", "Body.Sub.output"});

    auto graph = TopologyAnalyzer::BuildGraph(config);

    // If it incorrectly picks "Body", then Body -> Consumer
    // If it correctly picks "Body.Sub", then Body.Sub -> Consumer

    auto deps_body = graph.GetDependents("Body");
    auto deps_sub = graph.GetDependents("Body.Sub");

    // "Body" should NOT have "Consumer" as dependent if "Body.Sub" is the producer
    bool body_has_consumer =
        std::find(deps_body.begin(), deps_body.end(), "Consumer") != deps_body.end();
    bool sub_has_consumer =
        std::find(deps_sub.begin(), deps_sub.end(), "Consumer") != deps_sub.end();

    EXPECT_FALSE(body_has_consumer) << "Should NOT have matched shorter prefix 'Body'";
    EXPECT_TRUE(sub_has_consumer) << "Should HAVE matched longer prefix 'Body.Sub'";
}

// =============================================================================
// Integration Tests
// =============================================================================

TEST(TopologyAnalyzerTest, FullPipelineIntegration) {
    // Build a realistic simulation config
    SimulatorConfig config;

    // Components with entity prefix
    config.components.push_back(MakeComponent("Atmosphere", "Environment", "US76Atmosphere"));
    config.components.push_back(MakeComponent("Gravity", "Environment", "WGS84Gravity"));
    config.components.push_back(MakeComponent("Aero", "Rocket", "AeroBody"));
    config.components.push_back(MakeComponent("Engine", "Rocket", "JetEngine"));
    config.components.push_back(MakeComponent("Forces", "Rocket", "ForceAggregator"));
    config.components.push_back(MakeComponent("EOM", "Rocket", "RigidBody6DOF"));

    // Routes:
    // Atmosphere.density -> Aero.density
    // Aero.force -> Forces.src0
    // Engine.thrust -> Forces.src1
    // Forces.total -> EOM.force
    // Gravity.accel -> EOM.gravity
    config.routes.push_back(
        signal::SignalRoute{"Rocket.Aero.density", "Environment.Atmosphere.density"});
    config.routes.push_back(signal::SignalRoute{"Rocket.Forces.src0", "Rocket.Aero.force"});
    config.routes.push_back(signal::SignalRoute{"Rocket.Forces.src1", "Rocket.Engine.thrust"});
    config.routes.push_back(signal::SignalRoute{"Rocket.EOM.force", "Rocket.Forces.total"});
    config.routes.push_back(signal::SignalRoute{"Rocket.EOM.gravity", "Environment.Gravity.accel"});

    // Compute order
    auto result = TopologyAnalyzer::ComputeExecutionOrder(config);

    EXPECT_TRUE(result.IsValid());
    EXPECT_EQ(result.execution_order.size(), 6u);

    // Verify key ordering constraints
    auto find_pos = [&](const std::string &name) {
        for (size_t i = 0; i < result.execution_order.size(); ++i) {
            if (result.execution_order[i] == name)
                return i;
        }
        return size_t(-1);
    };

    // Atmosphere must run before Aero
    EXPECT_LT(find_pos("Environment.Atmosphere"), find_pos("Rocket.Aero"));

    // Aero and Engine must run before Forces
    EXPECT_LT(find_pos("Rocket.Aero"), find_pos("Rocket.Forces"));
    EXPECT_LT(find_pos("Rocket.Engine"), find_pos("Rocket.Forces"));

    // Forces and Gravity must run before EOM
    EXPECT_LT(find_pos("Rocket.Forces"), find_pos("Rocket.EOM"));
    EXPECT_LT(find_pos("Environment.Gravity"), find_pos("Rocket.EOM"));
}

} // namespace
} // namespace icarus
