# ArduPilot SITL from the official prebuilt release binaries.
#
# firmware.ardupilot.org publishes statically linked SITL builds for every
# release, so there is nothing to compile or patch. The simulation parameter
# files only live in the source tree, so they come from a sparse checkout of
# the exact commit the binaries were built from (see git-version.txt next to
# each binary).
#
# The output mirrors an ArduPilot checkout (`build/sitl/bin`,
# `Tools/autotest/...`) so `MAVRS_ARDUPILOT_DIR` can point at either.
{
  lib,
  stdenvNoCC,
  fetchurl,
  fetchFromGitHub,
}:
let
  version = "4.7.2";
  binary =
    vehicle: name: hash:
    fetchurl {
      url = "https://firmware.ardupilot.org/${vehicle}/stable-${version}/SITL_x86_64_linux_gnu/${name}";
      inherit hash;
    };
  arduplane = binary "Plane" "arduplane" "sha256-aGfBYpQcb17T4G22Z1YaoJNBPhhsvVUR6kRPxoVBaPE=";
  arducopter = binary "Copter" "arducopter" "sha256-yu015Mt5bsBsrECwlWahglF3FIaxy6xeK/Frel7VBF0=";
  autotest = fetchFromGitHub {
    owner = "ArduPilot";
    repo = "ardupilot";
    # Plane-4.7.2 and Copter-4.7.2 both point here.
    rev = "fb773567896014da007bca58358704d4a0d8516f";
    sparseCheckout = [
      "Tools/autotest/models"
      "Tools/autotest/default_params"
    ];
    hash = "sha256-mEYTmcQRZXD34voAUk9uOUsJGUEmpoNfKHDuhz0bEIE=";
  };
in
stdenvNoCC.mkDerivation {
  pname = "ardupilot-sitl";
  inherit version;

  dontUnpack = true;
  # Upstream ships static, unstripped binaries; leave them byte for byte.
  dontStrip = true;
  dontPatchELF = true;

  installPhase = ''
    runHook preInstall
    install -Dm755 ${arduplane} $out/build/sitl/bin/arduplane
    install -Dm755 ${arducopter} $out/build/sitl/bin/arducopter
    mkdir -p $out/Tools/autotest
    cp -r ${autotest}/Tools/autotest/models ${autotest}/Tools/autotest/default_params \
      $out/Tools/autotest/
    ln -s build/sitl/bin $out/bin
    runHook postInstall
  '';

  meta = {
    description = "ArduPilot Plane and Copter software-in-the-loop simulators";
    homepage = "https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html";
    license = lib.licenses.gpl3Plus;
    sourceProvenance = [ lib.sourceTypes.binaryNativeCode ];
    platforms = [ "x86_64-linux" ];
    mainProgram = "arduplane";
  };
}
