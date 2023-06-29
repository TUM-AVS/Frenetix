include(FetchContent)
include(utils/FetchContentHelper)

FetchContent_Declare_Fallback(
    taskflow

    SYSTEM

    URL "https://github.com/taskflow/taskflow/archive/refs/tags/v3.6.0.tar.gz"
    URL_HASH SHA256=5a1cd9cf89f93a97fcace58fd73ed2fc8ee2053bcb43e047acb6bc121c3edf4c

    #GIT_REPOSITORY "https://github.com/taskflow/taskflow.git"
    #GIT_TAG 12f8bd4e970ab27fd3dee3bffa24b5b48b54ba39
)

set(TF_BUILD_EXAMPLES OFF)
set(TF_BUILD_TESTS OFF)

FetchContent_MakeAvailable(taskflow)

set_property(DIRECTORY ${taskflow_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL ON)

