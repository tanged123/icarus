cmake_minimum_required(VERSION 3.20)
project(
  icarus
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- ccache support ---
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# --- Options ---
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)
option(BUILD_INTERFACES "Build language bindings (C, Python, MATLAB)" OFF)
option(BUILD_TESTING "Build test suite" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_COMPONENTS "Build standard component models" ON)

# --- Dependencies ---
find_package(Eigen3 3.4 REQUIRED)
find_package(casadi REQUIRED)
find_package(janus REQUIRED)
find_package(vulcan REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(HighFive REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# --- Core Library (src/) ---
# Icarus core is a compiled static library
add_subdirectory(src)

# --- Component Models (components/) ---
if(BUILD_COMPONENTS)
  add_subdirectory(components)
endif()

# --- Umbrella target for convenience ---
add_library(icarus INTERFACE)
target_link_libraries(icarus INTERFACE icarus_core)
if(BUILD_COMPONENTS)
  target_link_libraries(icarus INTERFACE icarus_components)
endif()
add_library(icarus::icarus ALIAS icarus)

# --- Coverage ---
if(ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    add_link_options(--coverage)
  else()
    message(WARNING "Coverage not supported for this compiler")
  endif()
endif()

# --- Install ---
include(GNUInstallDirs)

install(
  TARGETS icarus_core
  EXPORT icarusTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT icarusTargets
  FILE icarusConfig.cmake
  NAMESPACE icarus::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/icarus)

# --- Testing ---
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

# --- Examples ---
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# --- External Interfaces (interfaces/) ---
if(BUILD_INTERFACES)
  add_subdirectory(interfaces)
endif()
