include(FetchContent)

message(STATUS "DrivabilityChecker - using bundled version")
FetchContent_Declare(
    crdc
    GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad-drivability-checker.git
    #GIT_TAG "wip-skbuild"
    GIT_TAG e8e75d6033cbcd5831d9425deb9a3b5b548baac2
    GIT_SUBMODULES third_party/gpc # triangle not required
)

FetchContent_MakeAvailable(crdc)

set_property(DIRECTORY ${crdc_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

mark_as_advanced(
    ADD_MODULE_GEOMETRY
    ADD_MODULE_COLLISION
    ADD_TRIANGLE
    BUILD_S11N
)


