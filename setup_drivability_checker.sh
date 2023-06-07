#!/bin/bash
git -C /app/thirdParty/commonroad-drivability-checker submodule update --init
mkdir /app/thirdParty/commonroad-drivability-checker/build_debug && cd /app/thirdParty/commonroad-drivability-checker/build_debug

cmake -DCMAKE_INSTALL_PREFIX=/app/thirdParty/driveabilityChecker -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -- -j $(nproc)
cmake --install .