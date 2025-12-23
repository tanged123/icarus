# Icarus Repository Bootstrap Guide

> **Purpose**: This document provides comprehensive instructions for bootstrapping the **Icarus** 6DOF simulation engine repository. Icarus is a data-oriented simulation framework that utilizes both **Janus** (math library) and **Vulcan** (engineering utilities) as dependencies.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Architectural Context](#2-architectural-context)
3. [API Surface](#3-api-surface)
4. [Repository Structure](#4-repository-structure)
5. [Nix Setup](#5-nix-setup)
6. [CMake Configuration](#6-cmake-configuration)
7. [Scripts Directory](#7-scripts-directory)
8. [Core Headers](#8-core-headers)
9. [Testing Framework](#9-testing-framework)
10. [Examples Organization](#10-examples-organization)
11. [CI/CD Workflows](#11-cicd-workflows)
12. [Agent Rules](#12-agent-rules)
13. [Git Configuration](#13-git-configuration)
14. [Initial Module Roadmap](#14-initial-module-roadmap)
15. [Verification Checklist](#15-verification-checklist)

---

## 1. Project Overview

### What is Icarus?

**Icarus** is a 6DOF (Six Degrees of Freedom) simulation engine for aerospace applications. Named after the mythological figure who flew with wax wings, it represents the ambition to simulate complex flight dynamics. Icarus provides:

- **Data-Oriented Architecture**: Flat component topology where all simulation elements (gravity, atmosphere, propulsion, aerodynamics) are structural peers
- **Component-Based Design**: Modular components that implement `Provision()`, `Stage()`, and `Step()` lifecycle methods
- **Signal Backplane**: Centralized registry for all observable/configurable numeric data
- **Dual-Mode Execution**: Numeric (`double`) for real-time simulation and Symbolic (`casadi::MX`) for optimization, utilizing the Janus (`janus::Scalar`) type system
- **Integration with Janus/Vulcan**: Uses Janus for math and Vulcan for physics utilities

### The Icarus Stack

```
┌─────────────────────────────────────────────────────────────┐
│                        ICARUS                                │
│   (Simulation Engine: Components, State, Scheduling)         │
├─────────────────────────────────────────────────────────────┤
│                        VULCAN                                │
│   (Physics Utilities: Atmosphere, Gravity, Coordinates)      │
├─────────────────────────────────────────────────────────────┤
│                        JANUS                                 │
│   (Math Library: Autodiff, Optimization, Linear Algebra)     │
└─────────────────────────────────────────────────────────────┘
```

### Design Philosophy: "The Flat Simulation"

> **"The Gravity Model, the Atmosphere, the Spacecraft, and the Fuel Pump are all structural peers."**

There is no "World" object that *contains* the vehicle. There is a simulation that contains a list of components, some of which calculate gravity, and some of which calculate fuel flow. This design:

1. **Aligns with Simulink/GNC patterns**: Block diagram modeling, not inheritance trees
2. **Enables Janus symbolic compatibility**: Linear execution trace for graph generation
3. **Optimizes data locality**: Contiguous state vectors for ODE solvers

---

## 2. Architectural Context

### Core Entities and Relationships

| Term | Definition | Example |
|:-----|:-----------|:--------|
| **Component** | Fundamental unit of execution. Implements `Provision()`, `Stage()`, `Step()` | `JetEngine`, `Aerodynamics`, `GravityJ2` |
| **Model** | Stateless physics function (usually from Vulcan) | `vulcan::atmosphere::usa76()` |
| **Entity** | Virtual namespace for signal organization (NOT a C++ object) | "Falcon9" in `Falcon9.Propulsion.Thrust` |
| **Signal** | Discrete unit of data on the Backplane (`double`, `int32`, `int64`) | `nav.altitude`, `gnc.mode` |
| **State** | Subset of signals requiring integration | `Velocity`, `FuelMass`, `SpoolSpeed` |

### Component Lifecycle

```
PROVISION (once at app launch)
    │
    │  → Allocate memory, register signals, parse config
    ▼
STAGE (once per episode/run)
    │
    │  → Wire inputs, apply initial conditions, run trim
    ▼
STEP (every Δt, thousands per run)
    │
    │  → Read inputs, compute derivatives, write outputs
    │  → NO allocation, NO string lookups (hot path)
    ▼
(repeat per Δt)
```

### Division of Responsibility

| Feature | Vulcan (Utility) | Icarus (Component) |
|:--------|:-----------------|:-------------------|
| **EOM** | `compute_6dof_derivatives()` | `RigidBody6DOF` component that owns state |
| **Aero** | `calc_dynamic_pressure()`, `mach()` | `AeroMap` component with table lookups |
| **Propulsion** | `thrust_from_mdot()` | `JetEngine` component with spool dynamics |
| **Atmosphere** | `usa76()`, `exponential()` | `EnvironmentComponent` caching values |

> [!IMPORTANT]
> **Vulcan functions are pure and stateless. Icarus components own state and lifecycle.**

---

## 3. API Surface

Icarus exposes **three tiers** of interfaces for different users:

| Interface | Users | Use Case | Performance |
|:----------|:------|:---------|:------------|
| **C/C++ Headers** | FSW teams, Monte Carlo devs | Real-time simulation, optimization loops | Fastest |
| **Python/MATLAB** | Vehicle builders, analysts | Config-driven development, scripting | Good |
| **Config Files** | Everyone | Vehicle/scenario definition | N/A |

```
┌─────────────────────────────────────────────────────────────┐
│                      USER PROJECTS                          │
│  (Vehicle builders, FSW teams, Monte Carlo engineers)       │
├─────────────────────────────────────────────────────────────┤
│  Config Files │ Python/MATLAB │ C/C++ Headers              │
│  (YAML)       │ (Scripting)   │ (Performance-Critical)     │
├───────────────┴───────────────┴─────────────────────────────┤
│                     ICARUS LIBRARY                          │
│        include/     src/     components/    interfaces/      │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. Repository Structure

```plaintext
icarus/
├── .agent/workflows/           # Agent automation
├── .github/
│   ├── ISSUE_TEMPLATE/
│   └── workflows/              # CI/CD (ci.yml, coverage.yml, format.yml)
│
├── docs/
│   ├── architecture/           # IDOA design documents (existing)
│   ├── api/                    # Generated Doxygen (built by CI)
│   ├── guides/                 # User-facing documentation
│   │   ├── getting_started.md
│   │   ├── component_authoring.md
│   │   ├── configuration.md
│   │   └── python_api.md
│   ├── theory/                 # Math/physics background
│   │   ├── eom_derivations.md
│   │   └── coordinate_frames.md
│   └── implementation_plans/
│
├── include/icarus/             # PUBLIC C/C++ API (installed)
│   ├── icarus.hpp              # Umbrella header (includes all)
│   ├── core/                   # Core abstractions
│   │   ├── Component.hpp
│   │   ├── Types.hpp
│   │   ├── Concepts.hpp
│   │   └── Error.hpp
│   ├── signal/                 # Signal backplane
│   │   ├── Signal.hpp
│   │   ├── Registry.hpp
│   │   └── Descriptor.hpp
│   ├── sim/                    # Simulation runtime
│   │   ├── Simulator.hpp
│   │   ├── Integrator.hpp
│   │   └── Scheduler.hpp
│   └── io/                     # Input/output
│       ├── Config.hpp
│       ├── Recorder.hpp
│       └── Playback.hpp
│
├── src/                        # IMPLEMENTATION (not installed)
│   ├── CMakeLists.txt
│   ├── core/                   # Engine internals (co-located headers)
│   │   ├── Backplane.hpp
│   │   ├── Backplane.cpp
│   │   ├── EntityManager.hpp
│   │   └── EntityManager.cpp
│   ├── signal/
│   ├── integrator/
│   ├── scheduler/
│   ├── aggregator/
│   ├── config/
│   ├── recording/
│   ├── events/
│   ├── services/
│   └── simulator/
│
├── components/                 # COMPONENT MODELS (reusable library)
│   ├── CMakeLists.txt
│   ├── README.md
│   ├── eom/                    # Equations of Motion
│   ├── environment/            # Atmosphere, Gravity, Wind (uses Vulcan)
│   ├── propulsion/             # RocketEngine, JetEngine, FuelTank
│   ├── aerodynamics/           # AeroBody, AeroTable
│   ├── structure/              # Mass properties
│   ├── gnc/                    # Autopilot, Navigation
│   ├── sensors/                # IMU, GPS
│   └── actuators/              # TVC, ControlSurface
│
├── interfaces/                 # EXTERNAL BINDINGS
│   ├── CMakeLists.txt
│   ├── c_api/                  # C API for FFI
│   ├── python/                 # Python: `import icarus`
│   └── matlab/                 # MATLAB: `+icarus`
│
├── examples/                   # CONFIG-DRIVEN DEMOS
│   ├── CMakeLists.txt
│   ├── point_mass/
│   ├── rocket_launch/
│   ├── trajectory_opt/
│   └── custom_component/
│
├── tests/
│   ├── CMakeLists.txt
│   ├── core/                   # Core engine tests
│   ├── components/             # Component model tests
│   ├── integration/            # Full simulation tests
│   └── symbolic/               # Symbolic mode / graph generation
│
├── tools/                      # UTILITY SCRIPTS
│   ├── config_validator.py     # Validate YAML configs
│   ├── signal_graph.py         # Visualize signal dependencies
│   ├── hdf5_inspector.py       # Inspect recording files
│   └── trim_analyzer.py        # Analyze trim solutions
│
├── scripts/                    # BUILD AUTOMATION (auto-enters Nix)
│   ├── dev.sh
│   ├── build.sh
│   ├── test.sh
│   ├── ci.sh
│   └── verify.sh
│
├── logs/                       # Build/test logs (gitignored)
├── CMakeLists.txt
├── flake.nix
├── Doxyfile
├── .cursorrules
├── CLAUDE.md
├── .gitignore
├── .clang-format
└── README.md
```

### Design Rationale

| Directory | Purpose | Installed? |
|:----------|:--------|:-----------|
| `include/icarus/` | Public C/C++ API (nested for scalability) | ✓ |
| `src/` | Internal implementation (co-located headers) | ✗ |
| `components/` | Built-in component library | ✓ |
| `interfaces/` | Python/MATLAB/C bindings | Conditional |
| `tools/` | Utility scripts for development | ✗ |
| `docs/api/` | Generated Doxygen documentation | ✗ |

> [!TIP]
> **Headers in `src/` are co-located with .cpp files** for easy navigation. The nested `include/icarus/{core,signal,sim,io}/` structure scales to 50+ headers without losing navigability.

### Extensibility Points

| To add... | Location | Notes |
|:----------|:---------|:------|
| New component type | `components/<domain>/` | Link against `icarus::core` |
| New integrator | `src/integrator/` | Expose in `include/icarus/sim/` |
| Language binding | `interfaces/<lang>/` | Use `icarus` umbrella target |
| Utility tool | `tools/` | Python recommended |
| External/proprietary component | User's project | See below |

### External Component Pattern

Downstream users can add proprietary components without modifying Icarus:

```
my_vehicle_sim/
├── components/              # User's custom components
│   └── MyThruster.hpp
├── config/
│   ├── components/my_thruster.yaml
│   └── scenarios/mission.yaml
├── src/main.cpp
└── CMakeLists.txt           # Links icarus::icarus
```

```cmake
# my_vehicle_sim/CMakeLists.txt
find_package(icarus REQUIRED)

add_library(my_components STATIC components/MyThruster.cpp)
target_link_libraries(my_components PUBLIC icarus::core)

add_executable(my_sim src/main.cpp)
target_link_libraries(my_sim PRIVATE icarus::icarus my_components)
```

### Key Differences from Vulcan

| Aspect | Vulcan | Icarus |
|:-------|:-------|:-------|
| **Library Type** | Header-only (INTERFACE) | Compiled static library |
| **Dependencies** | Janus only | Janus + Vulcan |
| **Scope** | Stateless utilities | Stateful simulation engine |
| **Primary Users** | C++ developers | Vehicle builders (config + Python) |
| **Bindings** | None | C API, Python, MATLAB |

---

## 5. Nix Setup

### flake.nix

Create a Nix flake that brings in both Janus and Vulcan as dependencies:

```nix
{
  description = "Icarus: 6DOF Simulation Engine";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    treefmt-nix.url = "github:numtide/treefmt-nix";

    # Dependencies as flake inputs
    janus = {
      url = "github:tanged123/janus";
      # For local development:
      # url = "path:/home/tanged/sources/janus";
    };
    vulcan = {
      url = "github:tanged123/vulcan";
      # For local development:
      # url = "path:/home/tanged/sources/vulcan";
    };
  };

  outputs = { self, nixpkgs, flake-utils, treefmt-nix, janus, vulcan }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        stdenv = pkgs.llvmPackages_latest.stdenv;

        # Get packages from inputs
        janusPackage = janus.packages.${system}.default;
        vulcanPackage = vulcan.packages.${system}.default;

        # Treefmt configuration
        treefmtEval = treefmt-nix.lib.evalModule pkgs {
          projectRootFile = "flake.nix";
          programs.nixfmt.enable = true;
          programs.clang-format.enable = true;
          programs.cmake-format.enable = true;
        };
      in
      {
        packages.default = stdenv.mkDerivation {
          pname = "icarus";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [
            pkgs.cmake
            pkgs.ninja
            pkgs.pkg-config
          ];

          buildInputs = [
            pkgs.eigen
            pkgs.casadi
            pkgs.hdf5
            pkgs.highfive
            pkgs.nlohmann_json  # For configuration parsing
            pkgs.yaml-cpp       # For YAML config support
            pkgs.spdlog         # For structured logging
            janusPackage
            vulcanPackage
          ];

          cmakeFlags = [
            "-DENABLE_COVERAGE=OFF"
            "-DBUILD_BINDINGS=OFF"
          ];
        };

        devShells.default = pkgs.mkShell.override { inherit stdenv; } {
          packages = with pkgs; [
            cmake
            ninja
            pkg-config
            ccache
            eigen
            casadi
            hdf5
            highfive
            nlohmann_json
            yaml-cpp
            spdlog
            gtest
            clang-tools
            doxygen
            graphviz
            lcov
            llvmPackages_latest.llvm
            # Python bindings dependencies (optional)
            python3
            python3Packages.pybind11
          ] ++ [
            janusPackage
            vulcanPackage
            treefmtEval.config.build.wrapper
          ];

          shellHook = ''
            export CMAKE_PREFIX_PATH=${pkgs.eigen}:${pkgs.casadi}:${pkgs.gtest}:${pkgs.hdf5}:${pkgs.highfive}:${pkgs.nlohmann_json}:${pkgs.yaml-cpp}:${pkgs.spdlog}:${janusPackage}:${vulcanPackage}
          '';
        };

        formatter = treefmtEval.config.build.wrapper;

        checks = {
          formatting = treefmtEval.config.build.check self;
        };
      }
    );
}
```

### Local Development Configuration

For development with local Janus and Vulcan:

```nix
# In flake.nix inputs section:
janus = {
  url = "path:/home/tanged/sources/janus";
};
vulcan = {
  url = "path:/home/tanged/sources/vulcan";
};
```

> [!TIP]
> When developing locally with path inputs, run `nix flake update janus vulcan` after making changes to the dependencies.

---

## 6. CMake Configuration

```cmake
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
option(BUILD_COMPONENTS \"Build standard component models\" ON)

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
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

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
```

### src/CMakeLists.txt

```cmake
# Core Icarus library implementation
add_library(icarus_core STATIC
  core/Backplane.cpp
  core/Scheduler.cpp
  core/StateVector.cpp
  core/EntityManager.cpp
  config/ConfigLoader.cpp
  config/YamlParser.cpp
  integrator/RungeKutta4.cpp
  integrator/RungeKutta45.cpp
  aggregator/ForceAggregator.cpp
  aggregator/MassAggregator.cpp
  recording/HDF5Recorder.cpp
  recording/Playback.cpp
  events/EventQueue.cpp
  events/PhaseManager.cpp
  services/LoggingService.cpp
  services/TelemetryService.cpp
  simulator/Simulator.cpp
)

target_include_directories(icarus_core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
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
    yaml-cpp
    spdlog::spdlog
)

# Convenience alias for downstream use
add_library(icarus::core ALIAS icarus_core)
```

### components/CMakeLists.txt

```cmake
# Component Models library
add_library(icarus_components STATIC
  # EOM
  eom/RigidBody3DOF.cpp
  eom/RigidBody6DOF.cpp
  # Environment
  environment/AtmosphereComponent.cpp
  environment/GravityComponent.cpp
  environment/WindComponent.cpp
  # Propulsion
  propulsion/RocketEngine.cpp
  propulsion/JetEngine.cpp
  propulsion/FuelTank.cpp
  # Aerodynamics
  aerodynamics/AeroBody.cpp
  aerodynamics/AeroTable.cpp
  # Structure
  structure/Structure.cpp
  # GNC
  gnc/Autopilot.cpp
  gnc/NavigationFilter.cpp
  # Sensors
  sensors/IMU.cpp
  sensors/GPS.cpp
  # Actuators
  actuators/TVC.cpp
  actuators/ControlSurface.cpp
)

target_include_directories(icarus_components
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/icarus/components>
)

target_link_libraries(icarus_components
  PUBLIC icarus_core
)

add_library(icarus::components ALIAS icarus_components)
```

### interfaces/CMakeLists.txt (optional)

```cmake
# C API
add_library(icarus_c SHARED
  c_api/icarus_c.cpp
)
target_link_libraries(icarus_c PRIVATE icarus)

# Python bindings (pybind11)
find_package(pybind11 QUIET)
if(pybind11_FOUND)
  pybind11_add_module(py_icarus python/py_icarus.cpp)
  target_link_libraries(py_icarus PRIVATE icarus)
endif()
```

### tests/CMakeLists.txt

```cmake
find_package(GTest REQUIRED)

# Helper function to create test executables
function(icarus_add_test NAME)
  add_executable(${NAME} ${ARGN})
  target_link_libraries(${NAME} PRIVATE icarus GTest::gtest_main)
  gtest_discover_tests(${NAME})
endfunction()

# Core tests
icarus_add_test(test_component
    core/test_component.cpp
)

icarus_add_test(test_concepts
    core/test_concepts.cpp
)

# Signal tests
icarus_add_test(test_signal_registry
    signal/test_registry.cpp
)

icarus_add_test(test_signal_access
    signal/test_access.cpp
)

# Lifecycle tests
icarus_add_test(test_lifecycle
    lifecycle/test_lifecycle.cpp
)

# Scheduler tests
icarus_add_test(test_scheduler
    scheduler/test_scheduler.cpp
)

# State tests
icarus_add_test(test_state_vector
    state/test_state_vector.cpp
)

# Integrator tests
icarus_add_test(test_integrators
    integrator/test_integrators.cpp
)

# Integration tests
icarus_add_test(test_simple_sim
    integration/test_simple_sim.cpp
)
```

### examples/CMakeLists.txt

```cmake
# Helper function to create example executables with config directory
# Each example is its own subdirectory with main.cpp and config/
function(icarus_add_example NAME DIR)
  add_executable(${NAME} ${DIR}/main.cpp)
  target_link_libraries(${NAME} PRIVATE icarus)

  # Copy config directory to build output for runtime access
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/config)
    add_custom_command(
      TARGET ${NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/config
        $<TARGET_FILE_DIR:${NAME}>/config
    )
  endif()

  # Copy data directory if present
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/data)
    add_custom_command(
      TARGET ${NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/data
        $<TARGET_FILE_DIR:${NAME}>/data
    )
  endif()
endfunction()

# =============================================================================
# Intro Example (minimal, no config)
# =============================================================================
add_executable(hello_world intro/hello_world.cpp)
target_link_libraries(hello_world PRIVATE icarus)

# =============================================================================
# Configuration-Driven Examples
# =============================================================================
icarus_add_example(point_mass         point_mass)
icarus_add_example(simple_aircraft    simple_aircraft)
icarus_add_example(rocket_launch      rocket_launch)
icarus_add_example(trajectory_opt     trajectory_opt)
icarus_add_example(custom_component   custom_component)
```

---

## 7. Scripts Directory

All scripts auto-detect if running inside a Nix environment and re-enter via `dev.sh` if not. This enables seamless agentic execution without manual shell management.

Create the following scripts in `scripts/`:

### dev.sh
```bash
#!/usr/bin/env bash
# Enter the Icarus development environment or run a command within it
if [ $# -gt 0 ]; then
    nix develop --command "$@"
else
    nix develop
fi
```

### build.sh
```bash
#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Handle arguments
CLEAN=false

for arg in "$@"; do
    case $arg in
        --clean)
        CLEAN=true
        shift
        ;;
    esac
done

if [ "$CLEAN" = true ]; then
    echo "Clean build requested."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/clean.sh"
fi

# Create build directory if it doesn't exist or reconfigure
cmake -B build -G Ninja

# Build the project
ninja -C build
```

### clean.sh
```bash
#!/usr/bin/env bash
set -e
echo "Cleaning build directory..."
rm -rf build
echo "Clean complete."
```

### test.sh
```bash
#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Ensure we have a build
if [ ! -d "build" ]; then
    echo "Build directory not found. Building..."
    ./scripts/build.sh
fi

# Rebuild to ensure latest changes
ninja -C build

# Run tests
mkdir -p logs
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="logs/tests_${TIMESTAMP}.log"

ctest --test-dir build -VV 2>&1 | tee "$LOG_FILE"

# Create symlink to latest
ln -sf "tests_${TIMESTAMP}.log" logs/tests.log
```

### ci.sh
```bash
#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Ensure logs directory exists
mkdir -p "$PROJECT_ROOT/logs"

# Create timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/ci_${TIMESTAMP}.log"

# Run build and test scripts
echo "Running CI..."
cd "$PROJECT_ROOT"
(./scripts/build.sh --clean && ./scripts/test.sh) 2>&1 | tee "$LOG_FILE"

# Create symlink to latest
ln -sf "ci_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/ci.log"

echo "CI Complete. Logs available at logs/ci_${TIMESTAMP}.log (symlinked to logs/ci.log)"
```

### verify.sh
```bash
#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$PROJECT_ROOT/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/verify_${TIMESTAMP}.log"

echo "Starting full verification..." | tee "$LOG_FILE"

echo "=== Building ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/build.sh" 2>&1 | tee -a "$LOG_FILE"

echo "=== Running Tests ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/test.sh" 2>&1 | tee -a "$LOG_FILE"

echo "=== Running Examples ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/run_examples.sh" 2>&1 | tee -a "$LOG_FILE"

ln -sf "verify_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/verify.log"
echo "Verification complete! Logs at logs/verify_${TIMESTAMP}.log"
```

### coverage.sh
```bash
#!/usr/bin/env bash
set -e

# Ensure we are in a Nix environment
if [ -z "$IN_NIX_SHELL" ]; then
    echo "Not in Nix environment. Re-running inside Nix..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/dev.sh" "$0" "$@"
    exit $?
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

echo "=== Cleaning and rebuilding with coverage ==="
rm -rf "$BUILD_DIR"
cmake -B "$BUILD_DIR" -G Ninja -DENABLE_COVERAGE=ON
ninja -C "$BUILD_DIR"

echo "=== Running tests ==="
ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "=== Generating coverage report ==="
mkdir -p "$BUILD_DIR/coverage"

# Capture coverage data
lcov --capture \
    --directory "$BUILD_DIR" \
    --output-file "$BUILD_DIR/coverage/coverage.info" \
    --ignore-errors mismatch

# Remove external dependencies from coverage
lcov --remove "$BUILD_DIR/coverage/coverage.info" \
    '/nix/*' \
    '*/tests/*' \
    '*/examples/*' \
    --output-file "$BUILD_DIR/coverage/coverage_clean.info"

# Generate HTML report
genhtml "$BUILD_DIR/coverage/coverage_clean.info" \
    --output-directory "$BUILD_DIR/coverage/html"

echo "=== Coverage report generated ==="
echo "Open $BUILD_DIR/coverage/html/index.html in a browser"
```

### install-hooks.sh
```bash
#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HOOK_DIR="$PROJECT_ROOT/.git/hooks"
SOURCE_HOOK="$PROJECT_ROOT/.github/hooks/pre-commit"

if [ ! -d "$HOOK_DIR" ]; then
    echo "Error: .git/hooks directory not found. Are you in a git repository?"
    exit 1
fi

if [ ! -f "$SOURCE_HOOK" ]; then
    echo "Error: Pre-commit hook not found at $SOURCE_HOOK"
    exit 1
fi

cp "$SOURCE_HOOK" "$HOOK_DIR/pre-commit"
chmod +x "$HOOK_DIR/pre-commit"

echo "✅ Pre-commit hook installed successfully!"
echo "   Your code will be auto-formatted before each commit."
```

### run_examples.sh
```bash
#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

# Build if needed
if [ ! -d "$BUILD_DIR" ]; then
    echo "Build directory not found. Building..."
    "$SCRIPT_DIR/build.sh"
fi

echo "=== Running all examples ==="
shopt -s nullglob
for example in "$BUILD_DIR"/examples/*; do
    if [ -x "$example" ] && [ -f "$example" ]; then
        name=$(basename "$example")
        echo "--- Running $name ---"
        "$example" || echo "FAILED: $name"
        echo ""
    fi
done
shopt -u nullglob
echo "=== All examples completed ==="
```

---

## 8. Core Headers

### include/icarus/core/Component.hpp (Skeleton)

```cpp
#pragma once

#include <icarus/core/Types.hpp>
#include <icarus/signal/Signal.hpp>
#include <icarus/signal/Registry.hpp>
#include <string>

namespace icarus {

/**
 * @brief Base class for all simulation components.
 *
 * Components are the fundamental unit of execution in Icarus. They own
 * state and implement the Provision/Stage/Step lifecycle.
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar>
class Component {
public:
    virtual ~Component() = default;

    /**
     * @brief Provision phase - called once at application launch.
     *
     * Heavy lifting: allocate memory, register signals, parse config.
     *
     * @param registry Signal registry for registering outputs
     * @param config Component configuration
     */
    virtual void Provision(SignalRegistry<Scalar>& registry,
                          const ComponentConfig& config) = 0;

    /**
     * @brief Stage phase - called at start of each run/episode.
     *
     * Wire inputs, apply initial conditions, prepare for t=0.
     *
     * @param registry Signal registry for resolving inputs
     */
    virtual void Stage(SignalRegistry<Scalar>& registry) = 0;

    /**
     * @brief Step phase - called every time step (hot path!).
     *
     * Read inputs, compute derivatives, write outputs.
     * NO allocation, NO string lookups.
     *
     * @param t Current simulation time
     * @param dt Time step size
     */
    virtual void Step(Scalar t, Scalar dt) = 0;

    // Optional lifecycle hooks
    virtual void PreStep(Scalar t, Scalar dt) {}
    virtual void PostStep(Scalar t, Scalar dt) {}
    virtual void OnPhaseEnter(Phase phase) {}
    virtual void OnPhaseExit(Phase phase) {}
    virtual void OnError(const Error& error) {}
    virtual void Shutdown() {}

    // Metadata
    [[nodiscard]] virtual std::string Name() const = 0;
    [[nodiscard]] virtual std::string Entity() const { return ""; }
};

}  // namespace icarus
```

### include/icarus/signal/Signal.hpp (Skeleton)

```cpp
#pragma once

#include <cstdint>
#include <string>

namespace icarus {

/**
 * @brief Supported signal data types
 */
enum class SignalType : uint8_t {
    Float64,   // double
    Int32,     // int32_t (also used for booleans: 0/1)
    Int64      // int64_t
};

/**
 * @brief Signal lifecycle classification
 */
enum class SignalLifecycle : uint8_t {
    Static,    // Set at Provision/Stage, immutable during run (parameters)
    Dynamic    // Updated every Step (state, outputs)
};

/**
 * @brief Descriptor for a signal on the backplane
 */
struct SignalDescriptor {
    std::string name;           // Full path: "Falcon9.Propulsion.Thrust"
    std::string unit;           // Physical unit: "N", "m/s", "rad"
    SignalType type;            // Data type
    SignalLifecycle lifecycle;  // Static or Dynamic
    std::string description;    // Human-readable description

    // Optional metadata
    double min_value = -std::numeric_limits<double>::infinity();
    double max_value = std::numeric_limits<double>::infinity();
    bool is_state = false;      // Requires integration
};

}  // namespace icarus
```

---

## 9. Testing Framework

### Test Pattern

Every Icarus module must have tests for both numeric AND symbolic modes:

```cpp
#include <gtest/gtest.h>
#include <icarus/core/Component.hpp>
#include <janus/janus.hpp>

// ============================================
// Numeric Tests
// ============================================
TEST(Component, NumericLifecycle) {
    icarus::Simulator<double> sim;
    // Test component lifecycle in numeric mode
}

// ============================================
// Symbolic Tests (Graph Generation)
// ============================================
TEST(Component, SymbolicTracing) {
    icarus::Simulator<casadi::MX> sim;
    // Verify symbolic graph can be extracted
    auto F = sim.GenerateGraph("dynamics");
    EXPECT_FALSE(F.is_null());
}

// ============================================
// Integration Tests
// ============================================
TEST(Simulator, PointMassIntegration) {
    // Test a simple point mass simulation
    icarus::Simulator<double> sim;
    // Configure with gravity only
    // Run for 1 second
    // Verify position matches analytical solution
}
```

### Testing Dual Modes

```cpp
// Template test function that works for both modes
template <typename Scalar>
Scalar test_gravity_component(const Scalar& alt) {
    // Use Vulcan for physics
    return vulcan::gravity::point_mass::acceleration(alt, vulcan::constants::earth::mu);
}

TEST(GravityComponent, NumericMode) {
    double accel = test_gravity_component(100000.0);
    EXPECT_NEAR(accel, 9.51, 0.01);
}

TEST(GravityComponent, SymbolicMode) {
    auto alt = janus::sym("altitude");
    auto accel = test_gravity_component(alt);

    // Verify graph was created
    EXPECT_FALSE(accel.is_constant());

    // Evaluate
    double result = janus::eval(accel, {{"altitude", 100000.0}});
    EXPECT_NEAR(result, 9.51, 0.01);
}
```

---

## 10. Examples Organization

Icarus examples are **configuration-driven**. Each example is a self-contained directory with:
- `main.cpp` - Entry point that loads configuration and runs the simulation
- `config/` - Configuration files following the [6-layer architecture](architecture/13_configuration.md)
- `data/` - Optional lookup tables and external data files
- `README.md` - Documentation and usage instructions

### Directory Structure Pattern

```
examples/rocket_launch/
├── main.cpp                 # Entry point
├── config/
│   ├── components/          # Layer A: Component type defaults
│   │   ├── rocket_engine.yaml
│   │   ├── rigid_body_6dof.yaml
│   │   └── atmosphere.yaml
│   ├── entities/            # Layer A': Entity bundles
│   │   ├── falcon9_stage1.yaml
│   │   └── environment.yaml
│   ├── scenarios/           # Layer B: World setup + ICs
│   │   └── launch_to_orbit.yaml
│   ├── scheduler.yaml       # Layer D: Execution order
│   └── services.yaml        # Layer F: Logging/recording
├── data/
│   └── engine_performance.csv
└── README.md
```

---

### Example 1: hello_world (Minimal, No Config)

#### examples/intro/hello_world.cpp
```cpp
#include <icarus/icarus.hpp>
#include <iostream>

int main() {
    std::cout << "Icarus v" << icarus::Version() << std::endl;
    std::cout << "6DOF Simulation Engine" << std::endl;
    std::cout << "Built on Janus (Math) + Vulcan (Physics)" << std::endl;
    return 0;
}
```

---

### Example 2: point_mass (Simple Config-Driven)

#### examples/point_mass/main.cpp
```cpp
#include <icarus/icarus.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
    // Load configuration from scenario file
    std::string config_path = "config/scenarios/free_fall.yaml";
    if (argc > 1) config_path = argv[1];

    // Create and configure simulator from config
    icarus::Simulator<double> sim;
    sim.LoadScenario(config_path);

    // Provision: allocate memory, register signals
    sim.Provision();

    // Stage: wire inputs, apply initial conditions
    sim.Stage();

    // Run simulation
    double dt = sim.GetTimeStep();
    double t_end = sim.GetEndTime();

    while (sim.Time() < t_end) {
        sim.Step(dt);
    }

    // Report final state
    std::cout << "=== Simulation Complete ===" << std::endl;
    std::cout << "Final altitude: " << sim.GetSignal("Mass.EOM.position_z") << " m" << std::endl;
    std::cout << "Final velocity: " << sim.GetSignal("Mass.EOM.velocity_z") << " m/s" << std::endl;

    return 0;
}
```

#### examples/point_mass/config/components/rigid_body.yaml
```yaml
# Layer A: Component Type Definition
type: RigidBody3DOF
description: "Simple 3-DOF point mass dynamics"
defaults:
  mass: 1.0           # kg
  drag_area: 0.0      # m^2 (no drag by default)
  drag_cd: 0.0        # dimensionless
```

#### examples/point_mass/config/entities/mass.yaml
```yaml
# Layer A': Entity Definition
entity:
  name: Mass
  description: "Simple point mass"

  components:
    - type: RigidBody3DOF
      name: EOM
      params:
        mass: 10.0

    - type: PointMassGravity
      name: Gravity
      params:
        mu: 3.986004418e14  # Earth GM [m^3/s^2]

  internal_wiring:
    - source: Gravity.acceleration
      target: EOM.input_gravity

  ports:
    outputs:
      - name: position
        binds_to: EOM.output_position
      - name: velocity
        binds_to: EOM.output_velocity
```

#### examples/point_mass/config/scenarios/free_fall.yaml
```yaml
# Layer B: Scenario Definition
scenario:
  name: "Free Fall Test"
  description: "Point mass falling under gravity"

# Simulation parameters
simulation:
  dt: 0.01              # Time step [s]
  t_end: 10.0           # End time [s]
  integrator: RK4

# Entities in this scenario
entities:
  - type: Mass
    name: Mass

# Initial conditions
initial_conditions:
  Mass.EOM.position_x: 0.0
  Mass.EOM.position_y: 0.0
  Mass.EOM.position_z: 6471000.0   # ~100km altitude [m]
  Mass.EOM.velocity_x: 0.0
  Mass.EOM.velocity_y: 0.0
  Mass.EOM.velocity_z: 0.0

# Layer C: Wiring (minimal for this example)
wiring: []
```

#### examples/point_mass/config/services.yaml
```yaml
# Layer F: Services Configuration
logging:
  level: INFO
  file: "logs/point_mass.log"

recording:
  enabled: true
  file: "output/point_mass.h5"
  signals:
    - Mass.EOM.position_z
    - Mass.EOM.velocity_z
  sample_rate: 100  # Hz
```

---

### Example 3: rocket_launch (Multi-Entity, Full Config)

This example demonstrates:
- Multiple entities (Stage 1, Stage 2, Environment)
- Stage separation events
- Multi-rate scheduling
- Trim for launch conditions

See `examples/rocket_launch/README.md` for full documentation.

#### examples/rocket_launch/config/scenarios/launch_to_orbit.yaml (Summary)
```yaml
scenario:
  name: "Falcon 9 Launch to Orbit"

simulation:
  dt: 0.001
  t_end: 600.0
  integrator: RK45

entities:
  - type: Falcon9Stage1
    name: Stage1
  - type: Falcon9Stage2
    name: Stage2
  - type: Environment
    name: Env

initial_conditions:
  Stage1.EOM.position: [0, 0, 0]
  Stage1.EOM.velocity: [0, 0, 0]
  # ... more ICs

wiring:
  - source: Env.Atmosphere.density
    target: Stage1.Aero.input_density
  - source: Env.Gravity.acceleration
    target: Stage1.EOM.input_gravity
  # ... more wiring

events:
  - name: MECO
    condition: "Stage1.Propulsion.fuel_mass < 100"
    actions:
      - ghost: Stage1
      - activate: Stage2
```

#### examples/rocket_launch/config/scheduler.yaml
```yaml
# Layer D: Scheduler Configuration
scheduler:
  policy: TOPOLOGICAL

  rate_groups:
    - rate_hz: 1000
      components: [Stage1.EOM, Stage1.Aero, Stage2.EOM]
    - rate_hz: 100
      components: [Stage1.Propulsion, Stage2.Propulsion]
    - rate_hz: 10
      components: [Logger, Telemetry]

  execution_order:
    - Env.Atmosphere
    - Env.Gravity
    - Stage1.Aero
    - Stage1.Propulsion
    - Stage1.EOM
```

---

### Configuration Loading Flow

```
                       ┌────────────────────────────────────┐
                       │    scenarios/launch_to_orbit.yaml  │
                       │              (Layer B)              │
                       └────────────────┬───────────────────┘
                                        │ references
            ┌───────────────────────────┼───────────────────────────┐
            ▼                           ▼                           ▼
┌───────────────────┐       ┌───────────────────┐       ┌───────────────────┐
│  entities/*.yaml  │       │  scheduler.yaml   │       │  services.yaml    │
│    (Layer A')     │       │    (Layer D)      │       │    (Layer F)      │
└─────────┬─────────┘       └───────────────────┘       └───────────────────┘
          │ references
          ▼
┌───────────────────┐
│ components/*.yaml │
│    (Layer A)      │
└───────────────────┘
```

---

## 11. CI/CD Workflows

### .github/workflows/ci.yml
```yaml
name: Icarus CI

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  build-and-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install Nix
        uses: DeterminateSystems/nix-installer-action@main

      - name: Setup Nix Cache
        uses: DeterminateSystems/magic-nix-cache-action@main

      - name: Build & Test
        run: nix develop --command bash -c "./scripts/build.sh && ./scripts/test.sh"
```

### .github/workflows/format.yml
```yaml
name: Format Check

on: push

jobs:
  format:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: DeterminateSystems/nix-installer-action@main

      - uses: DeterminateSystems/magic-nix-cache-action@main

      - name: Check formatting
        run: |
          if ! nix fmt -- --fail-on-change; then
            echo ""
            echo "❌ Code is not formatted correctly!"
            echo "💡 Run 'nix fmt' locally to fix formatting"
            echo "💡 Or run './scripts/install-hooks.sh' to auto-format on every commit"
            exit 1
          fi
          echo "✅ All files are properly formatted!"
```

### .github/workflows/coverage.yml
```yaml
name: Code Coverage

on:
  push:
    branches: [main]

jobs:
  coverage:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install Nix
        uses: DeterminateSystems/nix-installer-action@main

      - name: Setup Nix Cache
        uses: DeterminateSystems/magic-nix-cache-action@main

      - name: Run Coverage Script
        run: nix develop --command ./scripts/coverage.sh

      - name: Upload to Codecov
        uses: codecov/codecov-action@v5
        with:
          token: ${{ secrets.CODECOV_TOKEN }}
          files: ./build/coverage/coverage_clean.info
          fail_ci_if_error: false
```

---

## 12. Agent Rules

### .cursorrules
```markdown
# Agent Ruleset: Icarus Project

You are an advanced AI coding assistant working on **Icarus**, a 6DOF simulation engine built on the Janus and Vulcan frameworks. Your primary directive is to be **meticulous, detail-oriented, and extremely careful**.

## Global Behavioral Rules

1.  **Safety First**: You must NEVER "nuke" a repository. Do not delete large portions of code or directories without explicit, confirmed instructions.
2.  **Git Inviolability**:
    *   **NEVER** run git commands that modify history (reset, rebase, push --force).
    *   **NEVER** commit or push changes automatically unless explicitly asked.
    *   **ALWAYS** leave git state management to the user.
3.  **Meticulousness**:
    *   Read all provided context before generating code.
    *   Double-check types, templates, and constraints.
    *   When refactoring, ensure no functionality is lost.
4.  **No Hallucinations**: Do not invent APIs. Search the Janus/Vulcan/Icarus codebase first.
5.  **Context Preservation**:
    *   **Documentation First**: Create and update documentation in `docs/`.
    *   **Handover**: Write down your plan and progress so the next agent can resume.

## Icarus-Specific Rules (CRITICAL)

### 1. Janus Compatibility (The "Red Line")
*   **Template-First**: ALL components and physics MUST be templated on `Scalar`.
*   **Dual-Backend**: Code must work for both `double` and `casadi::MX`.

### 2. Math & Control Flow (MANDATORY)
*   **Math Dispatch**: ALWAYS use `janus::` namespace (e.g., `janus::sin`, `janus::pow`).
*   **Branching**: NEVER use `if/else` on `Scalar` types. Use `janus::where()`.
*   **Loops**: Bounds must be structural (integers/constants).

### 3. Component Design
*   **Components own state**: State vectors, derivatives, lifecycle.
*   **Components use Vulcan**: Call Vulcan for physics (pure functions).
*   **Step() is hot path**: NO allocation, NO string lookups.

### 4. Coding Style & Standards
*   **Language Standard**: C++20.
*   **Formatting**: Adhere to `treefmt` (clang-format) rules.
*   **Testing**: Write GoogleTest cases for both numeric and symbolic backends.

### 5. Project Structure to Respect
*   `include/icarus/core/`: Component base class, types, concepts.
*   `include/icarus/signal/`: Signal backplane system.
*   `include/icarus/lifecycle/`: Provision/Stage/Step phases.
*   `include/icarus/scheduler/`: Execution ordering.
*   `include/icarus/simulator/`: Top-level Simulator class.
*   `tests/`: Test suite mirroring include structure.
*   `docs/architecture/`: IDOA architecture documents (READ FIRST!).

## Workflow Commands
*   Dev: `./scripts/dev.sh` (enter Nix environment)
*   Build: `./scripts/build.sh`
*   Test: `./scripts/test.sh`
*   CI: `./scripts/ci.sh`
*   Full Verify: `./scripts/verify.sh`
```

### CLAUDE.md
```markdown
# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project Overview

Icarus is a 6DOF simulation engine for aerospace applications, built on Janus (math) and Vulcan (physics utilities). It uses a data-oriented architecture where all simulation components are structural peers.

## Build Commands

```bash
./scripts/build.sh          # Build the project
./scripts/build.sh --clean  # Clean rebuild
./scripts/test.sh           # Run all tests
./scripts/run_examples.sh   # Run all examples
./scripts/verify.sh         # Full verification
nix develop                 # Enter dev environment
```

## Critical Janus Compatibility Rules

**These rules are INVIOLABLE. Breaking them causes symbolic mode to fail.**

### 1. Template-First Design
```cpp
template <typename Scalar>
Scalar my_function(const Scalar& x);  // CORRECT
double my_function(double x);          // WRONG
```

### 2. Math Dispatch - Use `janus::` Namespace
```cpp
janus::sin(x), janus::pow(x, 2)  // CORRECT
std::sin(x), std::pow(x, 2)      // WRONG
```

### 3. Branching - Use `janus::where()`
```cpp
Scalar result = janus::where(x > 0, x, -x);  // CORRECT
if (x > 0) { result = x; }                   // WRONG
```

## Architecture

- **`include/icarus/core/`**: Component base, types, concepts
- **`include/icarus/signal/`**: Signal backplane system
- **`include/icarus/lifecycle/`**: Provision/Stage/Step phases
- **`include/icarus/scheduler/`**: Execution ordering
- **`include/icarus/simulator/`**: Top-level Simulator
- **`docs/architecture/`**: IDOA architecture documents

## Key Dependencies

- **Janus**: Math, autodiff, optimization, linear algebra
- **Vulcan**: Atmosphere, gravity, coordinates, rotations, time

## Documentation

- **`docs/architecture/`**: Data-oriented architecture specification
- **`docs/user_guides/`**: Component authoring, signal system
- **`docs/implementation_plans/`**: Phased implementation plans
```

---

## 13. Git Configuration

### .gitignore
```gitignore
# Prerequisites
*.d

# Compiled Object files
*.slo
*.lo
*.o
*.obj

# Precompiled Headers
*.gch
*.pch

# Linker files
*.ilk

# Debugger Files
*.pdb

# Compiled Dynamic libraries
*.so
*.dylib
*.dll

# Fortran module files
*.mod
*.smod

# Compiled Static libraries
*.lai
*.la
*.a
*.lib

# Executables
*.exe
*.out
*.app

# debug information files
*.dwo

# logs
logs/

# Build Directories
build/
bin/
lib/
cmake-build-*/

# Temporary Files
*.swp
*.swo
*~
.DS_Store
*.log
*.tmp
.cache/
*.pdf
*.png
*.dot
*.json

# Preserve doc images
!docs/images/*.png

# Python
__pycache__/
*.py[cod]
*.egg-info/
.eggs/
```

### .clang-format
```yaml
BasedOnStyle: LLVM
IndentWidth: 4
ColumnLimit: 100
```

### .github/hooks/pre-commit
```bash
#!/usr/bin/env bash
# Pre-commit hook for formatting

# Check if we're in a nix shell
if command -v treefmt &> /dev/null; then
    treefmt --fail-on-change
else
    echo "Warning: treefmt not found. Run 'nix develop' first."
    exit 1
fi
```

---

## 14. Initial Module Roadmap

### Phase 1: Core Infrastructure (Bootstrap) ✓
- [ ] Set up repository structure
- [ ] Configure Nix flake with Janus + Vulcan dependencies
- [ ] Configure CMake build system
- [ ] Set up CI/CD workflows
- [ ] Create initial documentation
- [ ] Agent rules and context files

### Phase 2: Core Framework
- [ ] `Component` base class
- [ ] `IcarusTypes.hpp` and `IcarusConcepts.hpp`
- [ ] Basic error handling

### Phase 3: Signal Backplane
- [ ] `Signal` type definitions
- [ ] `SignalRegistry` implementation
- [ ] Type-safe signal access

### Phase 4: Lifecycle Management
- [ ] `Provision`, `Stage`, `Step` contexts
- [ ] Lifecycle phase enum and transitions
- [ ] Optional hooks (PreStep, PostStep, etc.)

### Phase 5: State Management
- [ ] `StateVector` global state container
- [ ] State registration and ownership
- [ ] Derivative accumulation

### Phase 6: Scheduler
- [ ] Component execution ordering
- [ ] Topological sort for dependencies
- [ ] Rate groups for multi-rate simulation

### Phase 7: Integration
- [ ] Integrator interface
- [ ] RK4 implementation
- [ ] Adaptive RK45 implementation

### Phase 8: Aggregators
- [ ] Force aggregation pattern
- [ ] Moment aggregation pattern

### Phase 9: Configuration
- [ ] Configuration loading (YAML/JSON)
- [ ] Layer A-D configuration system
- [ ] Entity definitions

### Phase 10: Simulator
- [ ] Top-level `Simulator<Scalar>` class
- [ ] Numeric mode execution
- [ ] Symbolic mode graph generation

### Phase 11: Trim Solver
- [ ] Trim problem NLP formulation
- [ ] Integration with `janus::Opti`
- [ ] Trim result handling

### Phase 12: Recording & Playback
- [ ] HDF5-based recording
- [ ] Schema versioning
- [ ] Warmstart/replay

### Phase 13: Events & Phases
- [ ] Event definition and queue
- [ ] Flight phase management
- [ ] Component ghosting

### Phase 14: Services
- [ ] Structured logging (spdlog)
- [ ] Telemetry service
- [ ] Debug mode support

### Phase 15: External Bindings
- [ ] C API
- [ ] Python bindings (pybind11)
- [ ] MATLAB bindings (optional)

---

## 15. Verification Checklist

Before considering the repository "bootstrapped", verify:

- [ ] `nix develop` enters the development shell without errors
- [ ] `./scripts/build.sh` compiles without errors
- [ ] `./scripts/test.sh` runs and passes basic tests
- [ ] `nix fmt` passes without changes
- [ ] CI workflows trigger on push
- [ ] A simple example compiles and runs:
  - [ ] Numeric mode evaluation works
  - [ ] Symbolic mode graph generation works
- [ ] Dependencies are properly linked:
  - [ ] Janus headers and functions accessible
  - [ ] Vulcan headers and functions accessible

---

## Appendix A: Minimal Hello World

A minimal verification that the repo is set up correctly:

```cpp
// examples/intro/hello_world.cpp
#include <icarus/icarus.hpp>
#include <vulcan/vulcan.hpp>
#include <janus/janus.hpp>
#include <iostream>

int main() {
    // Verify Icarus
    std::cout << "Icarus: OK" << std::endl;

    // Verify Vulcan integration
    double rho = vulcan::atmosphere::standard::density(10000.0);
    std::cout << "Vulcan: rho at 10km = " << rho << " kg/m³" << std::endl;

    // Verify Janus integration (numeric)
    double x = janus::sin(0.5);
    std::cout << "Janus (numeric): sin(0.5) = " << x << std::endl;

    // Verify Janus integration (symbolic)
    auto sym_x = janus::sym("x");
    auto sym_y = janus::sin(sym_x);
    double y = janus::eval(sym_y, {{"x", 0.5}});
    std::cout << "Janus (symbolic): sin(0.5) = " << y << std::endl;

    return 0;
}
```

---

This document provides all the context needed to bootstrap the Icarus repository with proper structure, tooling, and Janus/Vulcan integration. The modular roadmap allows incremental implementation while maintaining architectural consistency.
