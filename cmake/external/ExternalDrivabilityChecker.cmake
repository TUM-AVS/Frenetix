include(FetchContent)

find_package(Threads REQUIRED)

# Required for LINK_LIBRARIES_ONLY_TARGETS (gtest links directly to pthread)
add_library(pthread ALIAS Threads::Threads)

set(_crdc_local ON)

if(DEFINED ENV{CIBUILDWHEEL})
    set(_crdc_local ON)
endif()

if(_crdc_local)
    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/crdc)

    set_property(DIRECTORY ${PROJECT_SOURCE_DIR}/third_party/crdc PROPERTY EXCLUDE_FROM_ALL ON)
else()
    FetchContent_Declare(
        crdc
        GIT_REPOSITORY git@gitlab.lrz.de:motionplanning1/commonroad-drivability-checker.git
        GIT_TAG        feature/projection-domain-check-perf
    )

    FetchContent_MakeAvailable(crdc)

    set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)
endif()

set(BUILD_S11N FALSE CACHE BOOL "" FORCE)

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)


