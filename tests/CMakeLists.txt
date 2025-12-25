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
