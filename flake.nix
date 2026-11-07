{
  description = "mavrs development and PX4 Gazebo SITL environment";

  inputs = {
    gazebros2nix.url = "github:Gepetto/gazebros2nix";
    nixpkgs.follows = "gazebros2nix/nixpkgs";
  };

  outputs =
    { self, nixpkgs, gazebros2nix }:
    let
      supportedSystems = [ "x86_64-linux" ];
      forEachSystem = nixpkgs.lib.genAttrs supportedSystems;
    in
    {
      devShells = forEachSystem (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
          python = pkgs.python3;
          empy3 = python.pkgs.buildPythonPackage rec {
            pname = "empy";
            version = "3.3.4";
            format = "setuptools";
            src = pkgs.fetchPypi {
              inherit pname version;
              hash = "sha256-c6xJeFtgFHnfTqGKfHm8EwSop8NMArlHLPEgauiPAbM=";
            };
          };
          pyrosGenmsg = python.pkgs.buildPythonPackage rec {
            pname = "pyros-genmsg";
            version = "0.5.8";
            format = "setuptools";
            src = pkgs.fetchurl {
              url = "https://files.pythonhosted.org/packages/f8/ca/96c243af4feb684bbb0f4126e6b3d2d330cc935e6a3c31fb1d7194ef4729/pyros_genmsg-${version}.tar.gz";
              hash = "sha256-PBywfZxA+eYIcph+7Jg6rFvbWSDqd5gDA/scRsMI9Hg=";
            };
          };
          pyulog = python.pkgs.buildPythonPackage rec {
            pname = "pyulog";
            version = "1.2.3";
            pyproject = true;
            build-system = [
              python.pkgs.setuptools
              python.pkgs.setuptools-scm
            ];
            dependencies = [ python.pkgs.numpy ];
            src = pkgs.fetchPypi {
              inherit pname version;
              hash = "sha256-6mfoKE8zr5xqLOYHKq1ZwgcyNlEkNkhmHGgUHpqAJzQ=";
            };
          };
          px4Python = python.withPackages (
            ps: with ps; [
              argcomplete
              cerberus
              coverage
              empy3
              jinja2
              jsonschema
              kconfiglib
              lark
              lxml
              matplotlib
              nunavut
              numpy
              packaging
              pandas
              pkgconfig
              psutil
              pycryptodome
              pygments
              pymavlink
              pynacl
              pyrosGenmsg
              pyserial
              pyulog
              pyyaml
              requests
              setuptools
              six
              sympy
              toml
              wheel
            ]
          );
        in
        {
          default = pkgs.mkShell {
            packages = [
              gazebros2nix.packages.${system}.gz-harmonic
              pkgs.astyle
              pkgs.bc
              pkgs.ccache
              pkgs.cmake
              pkgs.cppcheck
              pkgs.gcc
              pkgs.gdb
              pkgs.git
              pkgs.gnumake
              pkgs.lcov
              pkgs.libxml2
              pkgs.ninja
              pkgs.opencv
              pkgs.openssl
              pkgs.pkg-config
              pkgs.protobuf
              pkgs.rsync
              pkgs.rustPlatform.bindgenHook
              pkgs.rustc
              pkgs.cargo
              pkgs.shellcheck
              pkgs.unzip
              pkgs.zip
              px4Python
            ];

            shellHook = ''
              export CCACHE_DIR="''${CCACHE_DIR:-$HOME/.cache/ccache}"
              export GZ_CONFIG_PATH="''${GZ_CONFIG_PATH:-$HOME/.gz}"
              if [ -z "''${MAVRS_PX4_DIR:-}" ] && [ -d "$HOME/work/PX4-Autopilot" ]; then
                export MAVRS_PX4_DIR="$HOME/work/PX4-Autopilot"
              fi
            '';
          };
        }
      );
    };
}
