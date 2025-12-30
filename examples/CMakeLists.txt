# Icarus Examples
# Phase 4.0.7: Updated for new Simulator API

# Helper function to add examples
function(icarus_add_example NAME)
    add_executable(${NAME} ${ARGN})
    target_link_libraries(${NAME} PRIVATE icarus)
    target_include_directories(${NAME} PRIVATE ${PROJECT_SOURCE_DIR}/components)
endfunction()

# Point Mass Free-Fall Demo
# Demonstrates: programmatic setup, AddRoutes, Stage/Step lifecycle
icarus_add_example(point_mass_freefall
    point_mass_freefall.cpp
)

# Configuration-Driven Demo
# Demonstrates: YAML config loading with SimulationLoader::Load()
icarus_add_example(config_driven_demo
    config_driven_demo.cpp
)

