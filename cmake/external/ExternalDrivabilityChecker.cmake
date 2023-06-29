include(FetchContent)

find_package(Threads REQUIRED)

# Required for LINK_LIBRARIES_ONLY_TARGETS (gtest links directly to pthread)
add_library(pthread ALIAS Threads::Threads)

FetchContent_Declare(
    crdc
    GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
    # branch feature/cmake-fetch-content
    GIT_TAG        a2cf908db1ab123bef1a4c40f58dd94e3bdd756d
)

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)


