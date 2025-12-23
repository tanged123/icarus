# Core Icarus library implementation
add_library(icarus_core STATIC
  core/Backplane.cpp
  simulator/Simulator.cpp
)

target_include_directories(icarus_core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(icarus_core
  PUBLIC
    Eigen3::Eigen
    casadi
    janus::janus
    vulcan::vulcan
    HighFive
    HDF5::HDF5
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
    spdlog::spdlog
)

# Convenience alias for downstream use
add_library(icarus::core ALIAS icarus_core)
