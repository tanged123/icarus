# Icarus

**6DOF Simulation Engine for Aerospace Applications**

Icarus is a data-oriented simulation framework that utilizes both **Janus** (math library) and **Vulcan** (engineering utilities) as dependencies. It provides a component-based architecture for building complex flight simulations with support for both real-time numeric execution and symbolic optimization.

## Features

- **Data-Oriented Architecture**: Flat component topology where all simulation elements are structural peers
- **Component-Based Design**: Modular components implementing `Provision()`, `Stage()`, `Step()` lifecycle
- **Signal Backplane**: Centralized registry for all observable/configurable numeric data
- **Dual-Mode Execution**: Numeric (`double`) for real-time and Symbolic (`casadi::MX`) for optimization
- **Integration with Janus/Vulcan**: Uses Janus for math and Vulcan for physics utilities

## Quick Start

```bash
# Enter development environment
./scripts/dev.sh

# Build
./scripts/build.sh

# Run tests
./scripts/test.sh

# Run examples
./scripts/run_examples.sh

# Or just one example
./scripts/run_example.sh <example_name>

# Or do it all at once!
./scripts/ci.sh # Build, test
./scripts/verify.sh  # Build, test, run examples

# Helper scripts 
./scripts/bump_version.sh [major|minor|patch] # Bump version
./scripts/coverage.sh # Run coverage
./scripts/generate_docs.sh # Generate docs
./scripts/install-hooks.sh # Install pre-commit formatter hook
```

## Project Structure

```
icarus/
├── include/icarus/    # Public C/C++ API
│   ├── core/          # Component, Types, Concepts, Error
│   ├── signal/        # Signal backplane system
│   ├── sim/           # Simulator, Integrator, Scheduler
│   └── io/            # Config, Recorder, Playback
├── src/               # Implementation files
├── components/        # Built-in component library
├── tests/             # Test suite
├── examples/          # Example programs
├── docs/              # Documentation
│   └── architecture/  # IDOA architecture specification
└── scripts/           # Build automation
```

## Documentation

- [Bootstrap Guide](docs/icarus_bootstrap_guide.md) - Repository setup instructions
- [Implementation Plan](docs/implementation_plans/icarus_implementation_plan.md) - Development roadmap
- [Architecture](docs/architecture/) - Data-oriented architecture specification

## Dependencies

- **Janus**: Math library with autodiff, optimization, linear algebra
- **Vulcan**: Physics utilities for atmosphere, gravity, coordinates

## License

[TBD]
