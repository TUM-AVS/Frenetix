{ 
  fetchFromGitLab

, python3Packages

, eigen
, boost
, taskflow
, spdlog

, gfortran
, cmake
, ninja
}:
let
  ccm = fetchFromGitLab {
    owner = "tum-cps";
    repo = "commonroad-cmake";
    domain = "gitlab.lrz.de";
    rev = "b6ec634c8299a98746def2960b3cb6544c6845e4";
    hash = "sha256-XE5po9mnuZJhJ2mEfpTP3Ij4mF8YXiNpx8u5ardAMcI=";
  };
in python3Packages.buildPythonPackage {
  pname = "frenetix";
  version = "0.4.0-rc7";

  pyproject = true;

  src = ./.;

  dontUseCmakeConfigure = true;

  build-system = with python3Packages; [
    scikit-build-core
    nanobind
    setuptools-scm
    pathspec
    pyproject-metadata
  ] ++ [ cmake ninja ];

  dependencies = [ python3Packages.numpy ];

  nativeBuildInputs = [
    gfortran
  ];

  pypaBuildFlags = [
    "-C cmake.define.FETCHCONTENT_SOURCE_DIR_COMMONROAD_CMAKE=${ccm}"
  ];

  buildInputs = [
    eigen
    boost
    taskflow
    spdlog
  ];

  pythonImportsCheck = [
    "frenetix"
  ];
}
    
