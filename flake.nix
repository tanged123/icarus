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

    # Dependencies as flake inputs
    janus = {
      url = "github:tanged123/janus";
      # For local development:
      #url = "path:/home/tanged/sources/janus";
    };
    vulcan = {
      url = "github:tanged123/vulcan";
      # For local development:
      # url = "path:/home/tanged/sources/vulcan";
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

        # Get packages from inputs
        janusPackage = janus.packages.${system}.default;
        vulcanPackage = vulcan.packages.${system}.default;

        # Python environment for development
        pythonEnv = python.withPackages (ps: [
          ps.pybind11
          ps.numpy
          ps.pytest
        ]);

        # Treefmt configuration
        treefmtEval = treefmt-nix.lib.evalModule pkgs {
          projectRootFile = "flake.nix";
          programs.nixfmt.enable = true;
          programs.clang-format.enable = true;
          programs.cmake-format.enable = true;
        };

        # Common build inputs for all packages
        commonBuildInputs = [
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

        # =======================================================
        # C++ Library Package (header-only core + components)
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

          buildInputs = commonBuildInputs;

          cmakeFlags = [
            "-DENABLE_COVERAGE=OFF"
            "-DBUILD_INTERFACES=OFF"
            "-DBUILD_TESTING=OFF"
          ];
        };

        # =======================================================
        # C API Package (shared library for FFI)
        # =======================================================
        icarusCApi = stdenv.mkDerivation {
          pname = "icarus-c-api";
          version = "0.6.0";
          src = ./.;

          nativeBuildInputs = [
            pkgs.cmake
            pkgs.ninja
            pkgs.pkg-config
          ];

          buildInputs = commonBuildInputs;

          cmakeFlags = [
            "-DENABLE_COVERAGE=OFF"
            "-DBUILD_INTERFACES=ON"
            "-DBUILD_PYTHON=OFF"
            "-DBUILD_TESTING=OFF"
            "-DBUILD_EXAMPLES=OFF"
          ];
        };

        # =======================================================
        # Python Bindings Package
        # =======================================================
        icarusPython = python.pkgs.buildPythonPackage {
          pname = "icarus";
          version = "0.6.0";
          format = "other"; # Not setuptools, custom cmake build

          src = ./.;

          # Use same LLVM stdenv as C++ packages for ABI compatibility
          inherit stdenv;

          nativeBuildInputs = [
            pkgs.cmake
            pkgs.ninja
            pkgs.pkg-config
          ];

          buildInputs = commonBuildInputs ++ [
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

          # Override Python install path so CMake installs to our output
          preConfigure = ''
            cmakeFlags="$cmakeFlags -DICARUS_PYTHON_INSTALL_DIR=$out/${python.sitePackages}"
          '';

          # Skip Python-specific build phases (cmake handles it)
          dontUsePythonBuild = true;
          dontUsePythonInstall = true;

          # Run cmake install
          installPhase = ''
            runHook preInstall
            cmake --install . --prefix $out
            runHook postInstall
          '';

          pythonImportsCheck = [ "icarus" ];

          meta = {
            description = "Icarus 6DOF Simulation Engine - Python Bindings";
            homepage = "https://github.com/tanged123/icarus";
          };
        };

      in
      {
        # =======================================================
        # Package Outputs
        # =======================================================
        packages = {
          default = icarusCpp; # C++ library (backwards compatible)
          cpp = icarusCpp; # Explicit C++ alias
          c-api = icarusCApi; # C API shared library
          python = icarusPython; # Python bindings
        };

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
            ]
            ++ [
              # Python environment with all required packages
              pythonEnv
            ]
            ++ [
              janusPackage
              vulcanPackage
              treefmtEval.config.build.wrapper
              cachix
            ];

          shellHook = ''
            export CMAKE_PREFIX_PATH=${pkgs.eigen}:${pkgs.casadi}:${pkgs.gtest}:${pkgs.hdf5}:${pkgs.highfive}:${pkgs.nlohmann_json}:${pkgs.yaml-cpp}:${pkgs.spdlog}:${pythonEnv}/${python.sitePackages}/pybind11:${janusPackage}:${vulcanPackage}
          '';
        };

        # Python shell with pre-built icarus bindings for testing
        devShells.python = pkgs.mkShell {
          packages = [
            (python.withPackages (ps: [
              icarusPython
              ps.numpy
              ps.pytest
            ]))
          ];

          shellHook = ''
            echo "Icarus Python shell - run 'python' to use icarus bindings"
          '';
        };

        formatter = treefmtEval.config.build.wrapper;

        checks = {
          formatting = treefmtEval.config.build.check self;
        };
      }
    );
}
