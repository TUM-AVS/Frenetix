include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    Eigen3

    SYSTEM

    GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
    # We need the fix provided by 68e03ab240aa340b91f0b6fea8d382ef5cfb9258
    GIT_TAG 23299632c246b77937fb78e8607863a2f02e191b

    #URL "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"
    #URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72

    FIND_PACKAGE_ARGS 3.3.7
)

set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_BUILD_CMAKE_PACKAGE OFF CACHE INTERNAL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE INTERNAL "" FORCE)

FetchContent_MakeAvailable(Eigen3)

if(TARGET eigen)
    install(TARGETS eigen
        EXPORT ${PROJECT_NAME}_Targets
        )
endif()
