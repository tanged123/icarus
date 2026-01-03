# Icarus Examples
# Phase 4.0.7: Updated for new Simulator API

# Helper function to add examples
function(icarus_add_example NAME)
    add_executable(${NAME} ${ARGN})
    # Link to icarus (headers) and icarus_components (for component factory registration)
    # Use whole-archive to ensure static registration code is included even though
    # there are no direct symbol references from the executable.
    target_link_libraries(${NAME} PRIVATE icarus)
    target_link_libraries(${NAME} PRIVATE
        $<LINK_LIBRARY:WHOLE_ARCHIVE,icarus_components>
    )
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

# Tumbling Body Demo
# Demonstrates: RigidBody6DOF, quaternion attitude, conservation laws
icarus_add_example(tumbling_body_demo
    tumbling_body_demo.cpp
)

# Aggregation Demo
# Demonstrates: MassAggregator, ForceAggregator, multi-body systems
icarus_add_example(aggregation_demo
    aggregation_demo.cpp
)

# Topology Analyzer Demo
# Demonstrates: automatic execution order from signal dependencies
icarus_add_example(topology_demo
    topology_demo.cpp
)

# Phase Switching Demo
# Demonstrates: PhaseManager for multi-phase flight simulation
icarus_add_example(phase_switching_demo
    phase_switching_demo.cpp
)
