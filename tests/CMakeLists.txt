find_package(GTest REQUIRED)
include(GoogleTest)

# Helper function to create test executables
function(icarus_add_test NAME)
  add_executable(${NAME} ${ARGN})
  target_link_libraries(${NAME} PRIVATE icarus GTest::gtest_main)
  gtest_discover_tests(${NAME})
endfunction()

# Core tests
icarus_add_test(test_types
    core/test_types.cpp
)

icarus_add_test(test_component
    core/test_component.cpp
)
target_include_directories(test_component PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Signal backplane tests (Phase 1.3)
icarus_add_test(test_signal
    signal/test_signal.cpp
)

# State management tests (Phase 2.1)
icarus_add_test(test_state_management
    sim/test_state_management.cpp
)
target_include_directories(test_state_management PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Integrator tests (Phase 2.2)
icarus_add_test(test_integrator
    sim/test_integrator.cpp
)
target_include_directories(test_integrator PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Point mass component tests (Phase 2.3)
icarus_add_test(test_point_mass
    components/test_point_mass.cpp
)
target_include_directories(test_point_mass PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Mass aggregator tests (Phase 4)
icarus_add_test(test_mass_aggregator
    components/test_mass_aggregator.cpp
)
target_include_directories(test_mass_aggregator PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Force aggregator tests (Phase 4)
icarus_add_test(test_force_aggregator
    components/test_force_aggregator.cpp
)
target_include_directories(test_force_aggregator PRIVATE ${PROJECT_SOURCE_DIR}/components)

# RigidBody6DOF tests (Phase 4.4)
icarus_add_test(test_rigid_body_6dof
    components/test_rigid_body_6dof.cpp
)
target_include_directories(test_rigid_body_6dof PRIVATE ${PROJECT_SOURCE_DIR}/components)

# IO tests
icarus_add_test(test_data_dictionary
    io/test_data_dictionary.cpp
)

icarus_add_test(test_logging
    io/test_logging.cpp
)

# Numeric/Symbolic comparison tests (Phase 3.4)
icarus_add_test(test_numeric_symbolic_comparison
    symbolic/numeric_symbolic_comparison_test.cpp
)

icarus_add_test(test_generate_graph
    symbolic/generate_graph_test.cpp
)
target_include_directories(test_generate_graph PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Phase 4.0: Configuration Infrastructure tests
icarus_add_test(test_component_config
    core/test_component_config.cpp
)

icarus_add_test(test_signal_router
    signal/test_signal_router.cpp
)

# Phase 4.0.7: Configuration structs tests
icarus_add_test(test_config_structs
    sim/test_config_structs.cpp
)

# Phase 4.0.7: SimulationLoader tests
icarus_add_test(test_simulation_loader
    io/test_simulation_loader.cpp
)

# Phase 4.0.7: Scheduler tests
icarus_add_test(test_scheduler
    sim/test_scheduler.cpp
)

# Phase 5: Topology analyzer tests
icarus_add_test(test_topology_analyzer
    sim/test_topology_analyzer.cpp
)

# Phase 4.0.7: Internal managers tests
icarus_add_test(test_managers
    sim/test_managers.cpp
)

# Phase 4.0.7ai: YAML Integration tests (FromConfig → Stage → Step)
icarus_add_test(test_yaml_integration
    sim/test_yaml_integration.cpp
)
target_include_directories(test_yaml_integration PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Phase 4: Staging tests (TrimSolver, Linearizer, LinearModel)
icarus_add_test(test_staging
    staging/test_staging.cpp
)
target_include_directories(test_staging PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Phase 4: Symbolic core tests (SymbolicSimulatorCore, SymbolicStager)
icarus_add_test(test_symbolic_core
    staging/test_symbolic_core.cpp
)
target_include_directories(test_symbolic_core PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Phase 4.5: Tumbling body integration test
icarus_add_test(test_tumbling_body
    integration/test_tumbling_body.cpp
)
target_include_directories(test_tumbling_body PRIVATE ${PROJECT_SOURCE_DIR}/components)
target_link_libraries(test_tumbling_body PRIVATE $<LINK_LIBRARY:WHOLE_ARCHIVE,icarus_components>)
