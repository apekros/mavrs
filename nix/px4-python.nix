# Python environment for PX4's build-time code generators (uORB, parameters,
# MAVLink, ROMFS). Shared by the packaged SITL build and the Gazebo shell.
{ python3, fetchPypi, fetchurl }:
let
  empy3 = python3.pkgs.buildPythonPackage rec {
    pname = "empy";
    version = "3.3.4";
    format = "setuptools";
    src = fetchPypi {
      inherit pname version;
      hash = "sha256-c6xJeFtgFHnfTqGKfHm8EwSop8NMArlHLPEgauiPAbM=";
    };
  };
  pyrosGenmsg = python3.pkgs.buildPythonPackage rec {
    pname = "pyros-genmsg";
    version = "0.5.8";
    format = "setuptools";
    src = fetchurl {
      url = "https://files.pythonhosted.org/packages/f8/ca/96c243af4feb684bbb0f4126e6b3d2d330cc935e6a3c31fb1d7194ef4729/pyros_genmsg-${version}.tar.gz";
      hash = "sha256-PBywfZxA+eYIcph+7Jg6rFvbWSDqd5gDA/scRsMI9Hg=";
    };
  };
  pyulog = python3.pkgs.buildPythonPackage rec {
    pname = "pyulog";
    version = "1.2.3";
    pyproject = true;
    build-system = [
      python3.pkgs.setuptools
      python3.pkgs.setuptools-scm
    ];
    dependencies = [ python3.pkgs.numpy ];
    src = fetchPypi {
      inherit pname version;
      hash = "sha256-6mfoKE8zr5xqLOYHKq1ZwgcyNlEkNkhmHGgUHpqAJzQ=";
    };
  };
in
python3.withPackages (
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
)
