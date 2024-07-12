include(FetchContent)

find_package(Boost 1.70 OPTIONAL_COMPONENTS program_options filesystem)

if(Boost_FOUND)
    message(STATUS "Boost - SYSTEM")
    return()
endif()

message(STATUS "Boost - falling back to external version")

# set(_boost_mirror "https://boostorg.jfrog.io/artifactory/main")
set(_boost_mirror "https://archives.boost.io")

FetchContent_Declare(boost_src
        URL ${boost_mirror}/release/1.85.0/source/boost_1_85_0.tar.gz
        URL_HASH SHA256=be0d91732d5b0cc6fbb275c7939974457e79b54d6f07ce2e3dfdd68bef883b0b
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
