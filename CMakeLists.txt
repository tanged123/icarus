cmake_minimum_required(VERSION 3.20)
project(
  icarus
  VERSION 0.6.0
  LANGUAGES CXX)

# --- Build Type Configuration ---
# Default to RelWithDebInfo if not specified (good balance for development)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      "RelWithDebInfo"
      CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "RelWithDebInfo" "MinSizeRel")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Define ICARUS_DEBUG for Debug builds (enables runtime assertions)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(ICARUS_DEBUG=1)
  message(STATUS "Debug mode enabled - ICARUS_DEBUG=1")
endif()

# Check for LTO support
include(CheckIPOSupported)
check_ipo_supported(RESULT result OUTPUT output)

if(result)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
  message(STATUS "LTO enabled for Release builds")
else()
  message(WARNING "LTO not supported: ${output}")
endif()

# Explicitly ensure -O3 for Release builds
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_C_FLAGS_RELEASE "-O3 ${CMAKE_C_FLAGS_RELEASE}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Fix for clangd on NixOS: CMake strips implicit system includes (like
# /nix/store/...) from compile_commands.json. This forces them back in so clangd
# can find them.
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

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
option(BUILD_PYTHON "Build Python bindings (requires pybind11)" ON)
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

# --- Core Library (header-only) ---
# Icarus is a header-only library - all templates in include/
add_library(icarus INTERFACE)
add_library(icarus::icarus ALIAS icarus)

target_include_directories(
  icarus INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(
  icarus
  INTERFACE Eigen3::Eigen
            casadi
            janus::janus
            vulcan::vulcan
            HighFive
            HDF5::HDF5
            nlohmann_json::nlohmann_json
            yaml-cpp::yaml-cpp
            spdlog::spdlog)

# --- Component Models (components/) ---
if(BUILD_COMPONENTS)
  add_subdirectory(components)
  target_link_libraries(icarus INTERFACE icarus_components)
endif()

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

# Export icarus (header-only core)
install(
  TARGETS icarus
  EXPORT icarusTargets
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export icarus_components if built
if(BUILD_COMPONENTS)
  install(
    TARGETS icarus_components
    EXPORT icarusTargets
    INCLUDES
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

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
# Include when C API (BUILD_INTERFACES) or Python bindings requested
if(BUILD_INTERFACES OR BUILD_PYTHON)
  add_subdirectory(interfaces)
endif()
