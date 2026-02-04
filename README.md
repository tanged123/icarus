# Icarus 🪽

[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://tanged123.github.io/icarus/) [![icarus CI](https://github.com/tanged123/icarus/actions/workflows/ci.yml/badge.svg)](https://github.com/tanged123/icarus/actions/workflows/ci.yml) [![Clang-Format Check](https://github.com/tanged123/icarus/actions/workflows/format.yml/badge.svg)](https://github.com/tanged123/icarus/actions/workflows/format.yml) [![codecov](https://codecov.io/github/tanged123/icarus/graph/badge.svg?token=67F2PGBQAU)](https://codecov.io/github/tanged123/icarus)

**6DOF Simulation Engine for Aerospace Applications**

Icarus is a data-oriented simulation framework that utilizes both **Janus** (math library) and **Vulcan** (engineering utilities) as dependencies. It provides a component-based architecture for building complex flight simulations with support for both real-time numeric execution and symbolic optimization.

## Features

- **Data-Oriented Architecture**: Flat component topology where all simulation elements are structural peers
- **Component-Based Design**: Modular components implementing `Provision()`, `Stage()`, `Step()` lifecycle
- **Signal Backplane**: Centralized registry for all observable/configurable numeric data
- **Dual-Mode Execution**: Numeric (`double`) for real-time and Symbolic (`casadi::MX`) for optimization
- **Multi-Language Bindings**: C API for FFI, Python bindings via pybind11
- **Integration with Janus/Vulcan**: Uses Janus for math and Vulcan for physics utilities

## Quick Start

Icarus uses Nix for reproducible builds. Install [Nix](https://nixos.org/download.html) first.

```bash
# Enter development environment
./scripts/dev.sh

# Build (Debug mode by default for local development)
./scripts/build.sh           # Debug build
./scripts/build.sh --release # Release build (optimized)
./scripts/build.sh --clean   # Clean rebuild

# Run tests
./scripts/test.sh            # Tests (inherits build type)
./scripts/test.sh --debug    # Explicit Debug
./scripts/test.sh --release  # Explicit Release

# Run examples
./scripts/run_examples.sh    # Run all examples
./scripts/run_example.sh <name>  # Run specific example

# CI & Verification
./scripts/ci.sh              # Clean Release build + tests (for CI)
./scripts/verify.sh          # Full verification (Debug + Release)

# Utilities
./scripts/bump_version.sh [major|minor|patch]  # Bump version
./scripts/coverage.sh        # Run coverage
./scripts/generate_docs.sh   # Generate docs
./scripts/install-hooks.sh   # Install pre-commit formatter hook
```

### Python Bindings

```bash
# Build with Python bindings enabled
./scripts/dev.sh
cmake -B build -DBUILD_INTERFACES=ON
cmake --build build
PYTHONPATH=build/python python -c "import icarus; print(icarus.__version__)"

# Or use the pre-built Nix package
nix develop .#python
python -c "import icarus; print(icarus.__version__)"
```

### Nix Packages

For downstream consumption or CI:

```bash
nix build .#default   # C++ headers + components library
nix build .#python    # Python bindings
nix build .#c-api     # C API shared library (libicarus_c.so)
```

### Using as a Dependency

**In another Nix flake (Python):**

```nix
{
  inputs.icarus.url = "github:tanged123/icarus";

  outputs = { self, nixpkgs, icarus, ... }:
    let
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
      icarusPython = icarus.packages.x86_64-linux.python;
    in {
      devShells.default = pkgs.mkShell {
        packages = [
          (pkgs.python3.withPackages (ps: [ icarusPython ps.numpy ]))
        ];
      };
    };
}
```

**In another Nix flake (C++):**

```nix
{
  inputs.icarus.url = "github:tanged123/icarus";

  outputs = { self, nixpkgs, icarus, ... }:
    let
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
    in {
      packages.default = pkgs.stdenv.mkDerivation {
        # ...
        buildInputs = [ icarus.packages.x86_64-linux.default ];
      };
    };
}
```

## Project Structure

```
icarus/
├── include/icarus/    # Public C++ API (header-only)
│   ├── core/          # Component, Types, Concepts, Error
│   ├── signal/        # Signal backplane system
│   ├── sim/           # Simulator, Integrator, Scheduler
│   └── io/            # Config, Recorder, Playback
├── components/        # Built-in component library
├── interfaces/        # Language bindings
│   ├── c_api/         # C API (icarus.h, libicarus_c.so)
│   └── python/        # Python bindings (pybind11)
├── tests/             # Test suite
├── examples/          # Example programs
├── docs/              # Documentation
│   └── architecture/  # IDOA architecture specification
└── scripts/           # Build automation
```

## CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_INTERFACES` | OFF | Build language bindings (C, Python) |
| `BUILD_PYTHON` | ON | Build Python bindings (requires pybind11) |
| `BUILD_TESTING` | ON | Build test suite |
| `BUILD_EXAMPLES` | ON | Build example programs |
| `BUILD_COMPONENTS` | ON | Build standard component models |
| `ENABLE_COVERAGE` | OFF | Enable coverage reporting |

## Documentation

- [Bootstrap Guide](docs/icarus_bootstrap_guide.md) - Repository setup instructions
- [Implementation Plan](docs/implementation_plans/icarus_implementation_plan.md) - Development roadmap
- [Architecture](docs/architecture/) - Data-oriented architecture specification

**API Guides:**

- [Component Authoring](docs/api/component_authoring_guide.md) - How to write components
- [Python API](docs/api/python_api_guide.md) - Python bindings usage
- [C API](docs/api/c_api_guide.md) - C API for FFI integrations

## Dependencies

- **Janus**: Math library with autodiff, optimization, linear algebra
- **Vulcan**: Physics utilities for atmosphere, gravity, coordinates
- **CasADi**: Symbolic math and optimization
- **Eigen**: Linear algebra
- **spdlog**: Logging
- **yaml-cpp**: Configuration parsing
- **HDF5/HighFive**: Data recording

## License

MIT License - See [LICENSE](LICENSE) for details.

