#include <filesystem>
#include <gtest/gtest.h>
#include <icarus/core/Error.hpp>
#include <icarus/io/data/IntrospectionGraph.hpp>
#include <nlohmann/json.hpp>

// ============================================================================
// Helpers
// ============================================================================

static icarus::IntrospectionGraph MakeTestGraph() {
    icarus::IntrospectionGraph graph;

    // Add a component to the dictionary
    icarus::DataDictionary::ComponentEntry comp;
    comp.name = "Rocket.Vehicle";
    comp.type = "Vehicle6DOF";
    icarus::SignalDescriptor out;
    out.name = "Rocket.Vehicle.total_mass";
    out.unit = "kg";
    out.description = "Total mass";
    comp.outputs.push_back(out);
    graph.dictionary.components.push_back(comp);
    graph.dictionary.ComputeStats();

    // Add a route edge
    graph.edges.push_back({"Rocket.Env.density", "Rocket.Aero.density", icarus::EdgeKind::Route});

    // Add resolve edges
    graph.edges.push_back({"Rocket.Engine.force.x", "Rocket.Vehicle", icarus::EdgeKind::Resolve});
    graph.edges.push_back({"Rocket.Structure.mass", "Rocket.Vehicle", icarus::EdgeKind::Resolve});

    return graph;
}

// ============================================================================
// JSON Serialization
// ============================================================================

TEST(IntrospectionGraph, ToJSON_HasEdges) {
    auto graph = MakeTestGraph();
    auto j = graph.ToJSON();

    ASSERT_TRUE(j.contains("edges"));
    EXPECT_EQ(j["edges"].size(), 3u);
}

TEST(IntrospectionGraph, ToJSON_EdgeKindStrings) {
    auto graph = MakeTestGraph();
    auto j = graph.ToJSON();

    EXPECT_EQ(j["edges"][0]["kind"], "route");
    EXPECT_EQ(j["edges"][1]["kind"], "resolve");
    EXPECT_EQ(j["edges"][2]["kind"], "resolve");
}

TEST(IntrospectionGraph, ToJSON_EdgeFields) {
    auto graph = MakeTestGraph();
    auto j = graph.ToJSON();

    auto &edge = j["edges"][0];
    EXPECT_EQ(edge["source"], "Rocket.Env.density");
    EXPECT_EQ(edge["target"], "Rocket.Aero.density");
    EXPECT_EQ(edge["kind"], "route");
}

TEST(IntrospectionGraph, ToJSON_ResolveEdgeFields) {
    auto graph = MakeTestGraph();
    auto j = graph.ToJSON();

    auto &edge = j["edges"][1];
    EXPECT_EQ(edge["source"], "Rocket.Engine.force.x");
    EXPECT_EQ(edge["target"], "Rocket.Vehicle");
    EXPECT_EQ(edge["kind"], "resolve");
}

TEST(IntrospectionGraph, ToJSON_SummaryHasEdgeCount) {
    auto graph = MakeTestGraph();
    auto j = graph.ToJSON();

    ASSERT_TRUE(j["summary"].contains("total_edges"));
    EXPECT_EQ(j["summary"]["total_edges"], 3u);
}

TEST(IntrospectionGraph, ToJSON_ComponentsPreserved) {
    auto graph = MakeTestGraph();
    auto j = graph.ToJSON();

    ASSERT_TRUE(j.contains("components"));
    EXPECT_EQ(j["components"].size(), 1u);
    EXPECT_EQ(j["components"][0]["name"], "Rocket.Vehicle");
    EXPECT_EQ(j["components"][0]["type"], "Vehicle6DOF");
}

TEST(IntrospectionGraph, ToJSON_BackwardCompatible) {
    // An empty graph should produce the same structure as DataDictionary
    icarus::IntrospectionGraph graph;
    graph.dictionary.ComputeStats();
    auto j = graph.ToJSON();

    EXPECT_TRUE(j.contains("summary"));
    EXPECT_TRUE(j.contains("components"));
    EXPECT_TRUE(j.contains("edges"));
    EXPECT_EQ(j["edges"].size(), 0u);
    EXPECT_EQ(j["summary"]["total_edges"], 0u);
}

// ============================================================================
// File I/O
// ============================================================================

TEST(IntrospectionGraph, ToJSONFile_Success) {
    auto graph = MakeTestGraph();
    std::string path = "test_introspection_graph.json";
    EXPECT_NO_THROW(graph.ToJSONFile(path));
    EXPECT_TRUE(std::filesystem::exists(path));

    // Verify contents are valid JSON with edges
    std::ifstream f(path);
    auto j = nlohmann::json::parse(f);
    EXPECT_TRUE(j.contains("edges"));
    EXPECT_EQ(j["edges"].size(), 3u);

    std::filesystem::remove(path);
}

TEST(IntrospectionGraph, ToJAMLFile_Success) {
    auto graph = MakeTestGraph();
    std::string path = "test_introspection_graph.yaml";
    EXPECT_NO_THROW(graph.ToYAMLFile(path));
    EXPECT_TRUE(std::filesystem::exists(path));
    std::filesystem::remove(path);
}

TEST(IntrospectionGraph, ToJSONFile_WriteError) {
    icarus::IntrospectionGraph graph;
    std::string invalid_path = "/nonexistent_directory/file.json";
    EXPECT_THROW(graph.ToJSONFile(invalid_path), icarus::IOError);
}

TEST(IntrospectionGraph, ToYAMLFile_WriteError) {
    icarus::IntrospectionGraph graph;
    std::string invalid_path = "/nonexistent_directory/file.yaml";
    EXPECT_THROW(graph.ToYAMLFile(invalid_path), icarus::IOError);
}
