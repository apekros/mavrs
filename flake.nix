{
  description = "mavrs development and SITL environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # Gazebo Harmonic. It keeps its own nixpkgs, and everything that links
    # against Gazebo (the PX4 build, the px4-gazebo shell) uses that same set
    # so glibc and libstdc++ agree. The Rust toolchain does not come from it.
    gazebros2nix.url = "github:Gepetto/gazebros2nix";
  };

  outputs =
    {
      self,
      nixpkgs,
      rust-overlay,
      gazebros2nix,
    }:
    let
      # ArduPilot only publishes x86_64 Linux SITL binaries, and gazebros2nix
      # only builds Gazebo there.
      supportedSystems = [ "x86_64-linux" ];
      forEachSystem =
        f:
        nixpkgs.lib.genAttrs supportedSystems (
          system:
          f (
            import nixpkgs {
              inherit system;
              overlays = [ rust-overlay.overlays.default ];
            }
          )
        );
      gazeboFor = system: {
        pkgs = gazebros2nix.inputs.nixpkgs.legacyPackages.${system};
        gz-harmonic = gazebros2nix.packages.${system}.gz-harmonic;
      };
    in
    {
      packages = forEachSystem (
        pkgs:
        let
          gz = gazeboFor pkgs.stdenv.hostPlatform.system;
        in
        {
          ardupilot-sitl = pkgs.callPackage ./nix/ardupilot-sitl.nix { };
          px4-sitl = gz.pkgs.callPackage ./nix/px4-sitl.nix {
            px4Python = gz.pkgs.callPackage ./nix/px4-python.nix { };
            gazebo = gz.gz-harmonic;
          };
          # No Gazebo in the closure. Copter and VTOL missions fly fine on
          # SIH, but its airplane model cannot reliably take off from the
          # ground, so the fixed-wing tests need the Gazebo build.
          px4-sitl-sih = pkgs.callPackage ./nix/px4-sitl.nix {
            px4Python = pkgs.callPackage ./nix/px4-python.nix { };
          };
        }
      );

      devShells = forEachSystem (
        pkgs:
        let
          inherit (pkgs.stdenv.hostPlatform) system;
          rustToolchain = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;
          sitl = self.packages.${system};
          gz = gazeboFor system;
        in
        {
          # Rust plus ready-to-run ArduPilot and PX4 (Gazebo) simulators.
          default = pkgs.mkShell {
            packages = [
              rustToolchain
              sitl.ardupilot-sitl
              sitl.px4-sitl
              # PX4's startup scripts launch `gz sim` from PATH.
              gz.gz-harmonic
            ];
            MAVRS_ARDUPILOT_DIR = sitl.ardupilot-sitl;
            MAVRS_PX4_DIR = sitl.px4-sitl;
            MAVRS_PX4_SIMULATOR = "gz";
          };

          # Build PX4 yourself against Gazebo Harmonic. Point MAVRS_PX4_DIR at
          # that checkout after `make px4_sitl`.
          px4-gazebo = gz.pkgs.mkShell {
            packages = [
              rustToolchain
              sitl.ardupilot-sitl
              gz.gz-harmonic
              (gz.pkgs.callPackage ./nix/px4-python.nix { })
            ]
            ++ (with gz.pkgs; [
              astyle
              bc
              ccache
              cmake
              cppcheck
              gcc
              gdb
              git
              gnumake
              lcov
              libxml2
              ninja
              opencv
              openssl
              pkg-config
              protobuf
              rsync
              shellcheck
              unzip
              xxd
              zip
            ]);
            MAVRS_ARDUPILOT_DIR = sitl.ardupilot-sitl;
            MAVRS_PX4_SIMULATOR = "gz";

            shellHook = ''
              export CCACHE_DIR="''${CCACHE_DIR:-$HOME/.cache/ccache}"
            '';
          };
        }
      );
    };
}
