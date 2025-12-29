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


# IO tests
icarus_add_test(test_data_dictionary
    io/test_data_dictionary.cpp
)

icarus_add_test(test_logging
    io/test_logging.cpp
)

# SimulationBuilder tests (Phase 3.1)
icarus_add_test(test_simulation_builder
    sim/simulation_builder_test.cpp
)
target_include_directories(test_simulation_builder PRIVATE ${PROJECT_SOURCE_DIR}/components)

# Runner and Reset tests
icarus_add_test(test_runner_and_reset
    sim/runner_and_reset_test.cpp
)
target_include_directories(test_runner_and_reset PRIVATE ${PROJECT_SOURCE_DIR}/components)


# SymbolicTracer tests (Phase 3.3)
icarus_add_test(test_symbolic_tracer
    symbolic/symbolic_tracer_test.cpp
)
target_include_directories(test_symbolic_tracer PRIVATE ${PROJECT_SOURCE_DIR}/components)

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

# Phase 4.0.7: Internal managers tests
icarus_add_test(test_managers
    sim/test_managers.cpp
)
