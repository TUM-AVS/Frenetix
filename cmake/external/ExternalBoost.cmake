include(FetchContent)

find_package(Boost 1.70 OPTIONAL_COMPONENTS program_options filesystem)

if(Boost_FOUND)
    message(STATUS "Boost - SYSTEM")
    return()
endif()

message(STATUS "Boost - falling back to external version")

FetchContent_Declare(boost_src
        URL https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz
        URL_HASH SHA256=4b2136f98bdd1f5857f1c3dea9ac2018effe65286cf251534b6ae20cc45e1847
)
FetchContent_MakeAvailable(boost_src)

# Enable this for debugging Boost discovery
# set(Boost_DEBUG ON)

set(Boost_NO_SYSTEM_PATHS ON)
set(BOOST_ROOT ${boost_src_SOURCE_DIR})
set(Boost_NO_BOOST_CMAKE ON)

find_package(Boost 1.80.0 MODULE REQUIRED)
message(VERBOSE "Boost - found: ${Boost_FOUND} (version ${Boost_VERSION})")
message(VERBOSE "Boost - Boost_INCLUDE_DIRS=${Boost_INCLUDE_DIRS}")
