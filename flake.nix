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
          version = "0.1.1";
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
          ];
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
              # Python bindings dependencies (optional)
              python3
              python3Packages.pybind11
            ]
            ++ [
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
