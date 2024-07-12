include(FetchContent)
include(utils/FetchContentHelper)

set(spdlog_version 1.14.1)
set(spdlog_minimum_version 1.13.0)

FetchContent_Declare_Fallback(
    spdlog

    SYSTEM

    URL "https://github.com/gabime/spdlog/archive/refs/tags/v${spdlog_version}.tar.gz"
    URL_HASH SHA256=1586508029a7d0670dfcb2d97575dcdc242d3868a259742b69f100801ab4e16b

    FIND_PACKAGE_ARGS ${spdlog_minimum_version}
)

FetchContent_MakeAvailable(spdlog)
