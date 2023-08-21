include(FetchContent)

find_package(Threads REQUIRED)

# Required for LINK_LIBRARIES_ONLY_TARGETS (gtest links directly to pthread)
add_library(pthread ALIAS Threads::Threads)

FetchContent_Declare(
    crdc
    GIT_REPOSITORY https://access_alienware:glpat-_rXLgimNhyGoxQ_8n-n9@gitlab.lrz.de:motionplanning1/commonroad-drivability-checker.git
    GIT_TAG        wip-experimental-fallible-conversion
)

set(BUILD_S11N FALSE CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)


