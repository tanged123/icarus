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
- **`include/icarus/sim/`**: Simulator, Integrator, Scheduler
- **`include/icarus/io/`**: Config, Recorder, Playback
- **`src/`**: Implementation files (co-located headers)
- **`components/`**: Built-in component library
- **`docs/architecture/`**: IDOA architecture documents

## Key Dependencies

- **Janus**: Math, autodiff, optimization, linear algebra
- **Vulcan**: Atmosphere, gravity, coordinates, rotations, time

## Documentation

- **`docs/architecture/`**: Data-oriented architecture specification
- **`docs/guides/`**: User guides (component authoring, signal system)
- **`docs/implementation_plans/`**: Phased implementation plans
