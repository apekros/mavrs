# PX4 SITL built from source.
#
# Gazebo support in PX4's CMake is optional: without gz-transport on the
# search path the gz bridge and plugins are skipped and only the built-in SIH
# physics is available. Pass `gazebo` (gz-harmonic) to build the bridge and
# plugins too. Build Gazebo and PX4 from the same nixpkgs so they agree on
# glibc and libstdc++.
#
# The output mirrors a PX4 checkout (`build/px4_sitl_default/{bin,etc}`, plus
# the Gazebo models, worlds, plugins and server config when enabled) so
# `MAVRS_PX4_DIR` can point at either.
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  ninja,
  git,
  xxd,
  px4Python,
  gazebo ? null,
  opencv,
  pkg-config,
  protobuf,
}:
let
  withGazebo = gazebo != null;
in
let
  # Micro-XRCE-DDS-Client's superbuild clones this at build time. The tag is
  # pinned upstream in Micro-XRCE-DDS-Client/CMakeLists.txt (_microcdr_tag).
  microcdr = fetchFromGitHub {
    owner = "eProsima";
    repo = "Micro-CDR";
    tag = "v2.0.1";
    hash = "sha256-X5kE8dMpwXL2hzpT6vY+BHa70Fw21z++vs8nVARpxNk=";
  };
in
stdenv.mkDerivation (finalAttrs: {
  pname = if withGazebo then "px4-sitl" else "px4-sitl-sih";
  version = "1.18.0-rc1";

  src = fetchFromGitHub {
    owner = "PX4";
    repo = "PX4-Autopilot";
    tag = "v${finalAttrs.version}";
    fetchSubmodules = true;
    hash = "sha256-lmfXtDVpjOk1FdwZMVn71lECQzBuzQZzkZgrEJyK5Gk=";
  };

  nativeBuildInputs = [
    cmake
    ninja
    git
    xxd
    px4Python
  ]
  # The GStreamer plugin requires pkg-config even though GStreamer itself is
  # optional.
  ++ lib.optional withGazebo pkg-config;

  # Once Gazebo is found the optical-flow plugin hard-requires OpenCV, and
  # the bridge links gz_msgs, which is only generated when protobuf is found.
  buildInputs = lib.optionals withGazebo [
    gazebo
    opencv
    protobuf
  ];

  postPatch = ''
    patchShebangs Tools src platforms ROMFS boards msg

    # Every px4_add_git_submodule() adds a ninja edge that depends on
    # <submodule>/.git and runs `git submodule update`. fetchgit strips .git
    # and the sandbox has no network, so the submodules are already as good
    # as they are going to get.
    cat > cmake/px4_git.cmake <<'EOF'
    function(px4_add_git_submodule)
      cmake_parse_arguments(ARG "" "TARGET;PATH" "" ''${ARGN})
      add_custom_target(''${ARG_TARGET})
    endfunction()
    EOF

    # The uXRCE-DDS client is started unconditionally by rcS, so keep the
    # module and hand its superbuild the prefetched Micro-CDR.
    uxrce=src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
    cp -r ${microcdr} $uxrce/microcdr
    chmod -R u+w $uxrce/microcdr
    substituteInPlace $uxrce/cmake/SuperBuild.cmake --replace-fail \
      $'        GIT_REPOSITORY\n            https://github.com/eProsima/Micro-CDR.git\n        GIT_TAG\n            ''${_microcdr_tag}' \
      $'        SOURCE_DIR\n            '"$PWD/$uxrce/microcdr"

    # PX4 links the client libraries from <build>/''${CMAKE_INSTALL_LIBDIR},
    # but never tells the nested superbuild which libdir that is. On Ubuntu
    # GNUInstallDirs happens to agree (lib); nixpkgs' CMake picks lib64.
    substituteInPlace src/modules/uxrce_dds_client/CMakeLists.txt --replace-fail \
      '-DCMAKE_INSTALL_PREFIX:PATH=''${CMAKE_CURRENT_BINARY_DIR}' \
      '-DCMAKE_INSTALL_PREFIX:PATH=''${CMAKE_CURRENT_BINARY_DIR} -DCMAKE_INSTALL_LIBDIR:PATH=''${lib_dir}'
    substituteInPlace $uxrce/cmake/SuperBuild.cmake --replace-fail \
      '-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>' \
      '-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_INSTALL_LIBDIR=''${CMAKE_INSTALL_LIBDIR}'

    # PX4 derives its version from `git describe` and `git rev-parse` at
    # configure and build time. Give it a one-commit repo tagged with the
    # release it actually is.
    git init -q
    git add -A
    git -c user.name=nix -c user.email=nix@localhost commit -q -m "v${finalAttrs.version}"
    git tag "v${finalAttrs.version}"

    # px_update_git_header.py only defines the MAVLink library hash when the
    # submodule is a repo, but mavlink_main.cpp links against it regardless.
    git -C src/modules/mavlink/mavlink init -q
    git -C src/modules/mavlink/mavlink add -A
    git -C src/modules/mavlink/mavlink -c user.name=nix -c user.email=nix@localhost \
      commit -q -m mavlink
  '';

  cmakeFlags = [
    (lib.cmakeFeature "CONFIG" "px4_sitl_default")
    (lib.cmakeFeature "CMAKE_BUILD_TYPE" "RelWithDebInfo")
    # The uXRCE-DDS module joins this onto its build directory, so the
    # absolute $out/lib the Nix CMake hook passes produces a bogus path. See
    # postPatch for how it reaches the nested superbuild.
    (lib.cmakeFeature "CMAKE_INSTALL_LIBDIR" "lib")
  ];

  # PX4's install() rules target a ROS-style share/ layout that does not
  # include the posix module symlinks the startup scripts call.
  installPhase = ''
    runHook preInstall
    mkdir -p $out/build/px4_sitl_default
    cp -a bin etc $out/build/px4_sitl_default/
    ln -s build/px4_sitl_default/bin $out/bin
  ''
  + lib.optionalString withGazebo ''
    plugins=src/modules/simulation/gz_plugins
    mkdir -p $out/build/px4_sitl_default/$plugins $out/Tools/simulation/gz \
      $out/src/modules/simulation/gz_bridge
    cp -a $plugins/*.so $out/build/px4_sitl_default/$plugins/
    # The optical-flow system plugin finds its sibling libOpticalFlow.so
    # through an RPATH into the build tree. Point it at the installed copy.
    for lib in $out/build/px4_sitl_default/$plugins/*.so; do
      rpath=$(patchelf --print-rpath "$lib" | tr : '\n' | grep -v "^$NIX_BUILD_TOP" | paste -sd:)
      patchelf --set-rpath "$out/build/px4_sitl_default/$plugins''${rpath:+:$rpath}" "$lib"
    done
    cp -r ../Tools/simulation/gz/models ../Tools/simulation/gz/worlds $out/Tools/simulation/gz/
    cp ../src/modules/simulation/gz_bridge/server.config $out/src/modules/simulation/gz_bridge/
  ''
  + ''
    runHook postInstall
  '';

  meta = {
    description =
      "PX4 software-in-the-loop simulator"
      + (if withGazebo then " with Gazebo Harmonic plugins" else " with the built-in SIH physics");
    homepage = "https://docs.px4.io/main/en/simulation/";
    license = lib.licenses.bsd3;
    platforms = lib.platforms.linux;
    mainProgram = "px4";
  };
})
