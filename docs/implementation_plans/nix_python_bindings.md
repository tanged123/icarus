# Nix Python Bindings Distribution

**Status:** Draft
**Date:** January 2026
**Purpose:** Enable Icarus to vend Python bindings through Nix for downstream consumers (Hermes)

---

## 1. Problem Statement

Icarus has working Python bindings via pybind11, but they are not exposed through the Nix flake:

```nix
# Current flake.nix
cmakeFlags = [
    "-DENABLE_COVERAGE=OFF"
    "-DBUILD_INTERFACES=OFF"    # ← Python bindings disabled
];
```

Downstream projects like **Hermes** (the STEP middleware) need to import `icarus` as a Python module:

```python
import icarus
sim = icarus.Simulator("config.yaml")
```

Currently, this requires manually building Icarus with `BUILD_INTERFACES=ON` and adding the build output to `PYTHONPATH`. This breaks the Nix reproducibility model.

---

## 2. Current State

### CMakeLists.txt Issues

```cmake
# interfaces/CMakeLists.txt
if(BUILD_PYTHON)    # ← This option doesn't exist!
    find_package(pybind11 QUIET)
    ...
endif()
```

The `BUILD_PYTHON` option is referenced but never defined. It should be in the root `CMakeLists.txt`.

### Flake Outputs

Current:
```
packages.default  → C++ headers + libicarus_components.a
```

Needed:
```
packages.default  → C++ headers + components (unchanged)
packages.python   → Python module (site-packages/icarus/)
```

---

## 3. Proposed Changes

### 3.1 CMakeLists.txt

Add missing option to root `CMakeLists.txt`:

```cmake
# --- Options ---
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)
option(BUILD_INTERFACES "Build language bindings (C, Python, MATLAB)" OFF)
option(BUILD_PYTHON "Build Python bindings (requires pybind11)" ON)  # ← ADD THIS
option(BUILD_TESTING "Build test suite" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_COMPONENTS "Build standard component models" ON)
```

Update `interfaces/CMakeLists.txt` to use it properly:

```cmake
# Python bindings (pybind11) - Phase 7.2
if(BUILD_INTERFACES AND BUILD_PYTHON)
    find_package(pybind11 QUIET)
    if(pybind11_FOUND)
        add_subdirectory(python)
    else()
        message(WARNING "pybind11 not found - Python bindings disabled")
    endif()
endif()
```

### 3.2 Flake.nix Updates

```nix
{
  description = "Icarus: 6DOF Simulation Engine";

  nixConfig = {
    extra-substituters = [ "https://tanged123.cachix.org" ];
    extra-trusted-public-keys = [
      "tanged123.cachix.org-1:S79iH77XKs7/Ap+z9oaafrhmrw6lQ21QDzxyNqg1UVI="
    ];
  };

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    treefmt-nix.url = "github:numtide/treefmt-nix";

    janus = {
      url = "github:tanged123/janus";
    };
    vulcan = {
      url = "github:tanged123/vulcan";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      treefmt-nix,
      janus,
      vulcan,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        stdenv = pkgs.llvmPackages_latest.stdenv;
        python = pkgs.python3;

        janusPackage = janus.packages.${system}.default;
        vulcanPackage = vulcan.packages.${system}.default;

        # Treefmt configuration
        treefmtEval = treefmt-nix.lib.evalModule pkgs {
          projectRootFile = "flake.nix";
          programs.nixfmt.enable = true;
          programs.clang-format.enable = true;
          programs.cmake-format.enable = true;
        };

        # =======================================================
        # C++ Library Package (existing, unchanged)
        # =======================================================
        icarusCpp = stdenv.mkDerivation {
          pname = "icarus";
          version = "0.6.0";
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
            pkgs.nlohmann_json
            pkgs.yaml-cpp
            pkgs.spdlog
            janusPackage
            vulcanPackage
          ];

          cmakeFlags = [
            "-DENABLE_COVERAGE=OFF"
            "-DBUILD_INTERFACES=OFF"
            "-DBUILD_TESTING=OFF"
          ];
        };

        # =======================================================
        # Python Bindings Package (NEW)
        # =======================================================
        icarusPython = python.pkgs.buildPythonPackage {
          pname = "icarus";
          version = "0.6.0";
          format = "other";  # Not setuptools, custom cmake build

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
            pkgs.nlohmann_json
            pkgs.yaml-cpp
            pkgs.spdlog
            janusPackage
            vulcanPackage
            python.pkgs.pybind11
          ];

          propagatedBuildInputs = [
            python.pkgs.numpy
          ];

          cmakeFlags = [
            "-DENABLE_COVERAGE=OFF"
            "-DBUILD_INTERFACES=ON"
            "-DBUILD_PYTHON=ON"
            "-DBUILD_TESTING=OFF"
            "-DBUILD_EXAMPLES=OFF"
          ];

          # CMake installs to Python_SITEARCH, need to set it correctly
          preConfigure = ''
            export Python_SITEARCH=$out/${python.sitePackages}
          '';

          # Skip Python-specific build phases (cmake handles it)
          dontUsePythonBuild = true;
          dontUsePythonInstall = true;

          # But do run cmake install
          installPhase = ''
            runHook preInstall
            cmake --install . --prefix $out
            runHook postInstall
          '';

          # Ensure the module can find its dependencies
          postFixup = ''
            # Add rpath for shared libraries if needed
          '';

          pythonImportsCheck = [ "icarus" ];

          meta = {
            description = "Icarus 6DOF Simulation Engine - Python Bindings";
            homepage = "https://github.com/tanged123/icarus";
          };
        };

        # Python environment for development
        pythonEnv = python.withPackages (ps: [
          ps.pybind11
          ps.numpy
          ps.pytest
        ]);

      in
      {
        # =======================================================
        # Package Outputs
        # =======================================================
        packages = {
          default = icarusCpp;       # C++ library (backwards compatible)
          cpp = icarusCpp;           # Explicit C++ alias
          python = icarusPython;     # Python bindings
        };

        # =======================================================
        # Development Shell (unchanged)
        # =======================================================
        devShells.default = pkgs.mkShell.override { inherit stdenv; } {
          packages =
            with pkgs;
            [
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
              pythonEnv
              janusPackage
              vulcanPackage
              treefmtEval.config.build.wrapper
              cachix
            ];

          shellHook = ''
            export CMAKE_PREFIX_PATH=${pkgs.eigen}:${pkgs.casadi}:${pkgs.gtest}:${pkgs.hdf5}:${pkgs.highfive}:${pkgs.nlohmann_json}:${pkgs.yaml-cpp}:${pkgs.spdlog}:${pythonEnv}/${python.sitePackages}/pybind11:${janusPackage}:${vulcanPackage}
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

---

## 4. Downstream Usage (Hermes)

Once Icarus vends Python bindings, Hermes can consume them:

```nix
# hermes/flake.nix
{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    icarus.url = "github:tanged123/icarus";
  };

  outputs = { self, nixpkgs, icarus, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        icarusPython = icarus.packages.${system}.python;
      in
      {
        devShells.default = pkgs.mkShell {
          packages = [
            (pkgs.python3.withPackages (ps: [
              icarusPython  # ← Clean dependency!
              ps.websockets
              ps.pydantic
              # ...
            ]))
          ];
        };
      }
    );
}
```

Or in Hermes Python code:

```python
# This "just works" in nix develop
import icarus
from hermes.adapters.icarus import IcarusAdapter

adapter = IcarusAdapter("sim", "config.yaml")
```

---

## 5. Alternative: Single Package with Python

Instead of separate `packages.cpp` and `packages.python`, we could build a single package that includes both:

```nix
packages.default = stdenv.mkDerivation {
  # ... existing config ...

  cmakeFlags = [
    "-DBUILD_INTERFACES=ON"
    "-DBUILD_PYTHON=ON"
  ];

  buildInputs = [
    # ... existing ...
    python.pkgs.pybind11
  ];

  postInstall = ''
    # Copy Python module to a predictable location
    mkdir -p $out/lib/python${python.pythonVersion}/site-packages
    cp -r $out/${python.sitePackages}/icarus $out/lib/python${python.pythonVersion}/site-packages/
  '';
};
```

**Trade-offs:**

| Approach | Pros | Cons |
|----------|------|------|
| Separate packages | Clean separation, smaller C++ package | Two packages to maintain |
| Combined package | Single source of truth | Larger package, mixed concerns |

**Recommendation:** Separate packages. The C++ library is used by other C++ projects (if any), while Python bindings are used by Hermes.

---

## 6. Implementation Checklist

### CMake Changes

- [ ] Add `option(BUILD_PYTHON ...)` to root `CMakeLists.txt`
- [ ] Update `interfaces/CMakeLists.txt` condition to `BUILD_INTERFACES AND BUILD_PYTHON`
- [ ] Ensure `Python_SITEARCH` is configurable for nix install path

### Flake Changes

- [ ] Add `icarusPython` derivation using `buildPythonPackage`
- [ ] Add `packages.python` output
- [ ] Test with `nix build .#python`
- [ ] Test import: `nix develop -c python -c "import icarus; print(icarus.__version__)"`

### Hermes Integration

- [ ] Update `hermes/flake.nix` to use `icarus.packages.${system}.python`
- [ ] Remove cffi from dependencies (using pybind11 directly)
- [ ] Test full integration

---

## 7. Testing the Changes

```bash
# Build Python package
nix build .#python

# Test import
nix develop -c python -c "import icarus; print(icarus.__version__)"

# Run Python tests
nix develop -c pytest interfaces/python/tests/

# Verify Hermes can use it
cd ../hermes
nix develop
python -c "from hermes.adapters.icarus import IcarusAdapter; print('OK')"
```

---

## 8. Open Questions

1. **Version pinning:** Should `packages.python` pin numpy version, or leave it flexible?

2. **Python version:** Currently hardcoded to python3. Support python3.11 and python3.12?

3. **Wheel distribution:** Should we also produce wheels for non-nix users via CI?
   ```bash
   pip install icarus  # From PyPI or GitHub releases
   ```

4. **RPATH issues:** The pybind11 module links against libcasadi, etc. Need to ensure rpath is set correctly in nix build.
