#!/bin/bash
git -C /app/thirdParty/commonroad-driveability-checker submodule update --init
mkdir /app/thirdParty/commonroad-driveability-checker/build_debug && cd /app/thirdParty/commonroad-driveability-checker/build_debug

cmake -DCMAKE_INSTALL_PREFIX=/app/thirdParty/driveabilityChecker -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -- -j 1
cmake --install .