#!/bin/bash
base_dir=./thirdParty/commonroad-drivability-checker

git -C $base_dir submodule update --init
mkdir -p $base_dir/build_debug && cd $base_dir/build_debug

cmake -DCMAKE_INSTALL_PREFIX=../../driveabilityChecker -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -- -j $(nproc)
cmake --install .
