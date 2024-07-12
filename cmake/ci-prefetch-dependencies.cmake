set(_base_prefix /opt/dependencies)
set(_fetch_prefix ${_base_prefix}/fetch)
set(_extract_prefix ${_base_prefix}/extract)

message(STATUS "Prefetching dependencies")

file(MAKE_DIRECTORY ${_fetch_prefix})

file(DOWNLOAD
    https://boostorg.jfrog.io/artifactory/main/release/1.85.0/source/boost_1_85_0.tar.gz
    ${_fetch_prefix}/boost_1_85_0.tar.gz
    EXPECTED_HASH SHA256=be0d91732d5b0cc6fbb275c7939974457e79b54d6f07ce2e3dfdd68bef883b0b
)

file(DOWNLOAD
    https://github.com/gabime/spdlog/archive/refs/tags/v1.14.1.tar.gz
    ${_fetch_prefix}/spdlog-v1.14.1.tar.gz
    EXPECTED_HASH SHA256=1586508029a7d0670dfcb2d97575dcdc242d3868a259742b69f100801ab4e16b
)

file(DOWNLOAD
    https://github.com/taskflow/taskflow/archive/refs/tags/v3.7.0.tar.gz
    ${_fetch_prefix}/taskflow-v3.7.0.tar.gz
    EXPECTED_HASH SHA256=788b88093fb3788329ebbf7c7ee05d1f8960d974985a301798df01e77e04233b
)

file(ARCHIVE_EXTRACT
    INPUT ${_fetch_prefix}/boost_1_85_0.tar.gz
    DESTINATION ${_extract_prefix}
)
set(ENV{CI_PREFETCH_BOOST} ${_extract_prefix}/boost_1_85_0)

file(ARCHIVE_EXTRACT
    INPUT ${_fetch_prefix}/spdlog-v1.14.1.tar.gz
    DESTINATION ${_extract_prefix}
)
set(ENV{CI_PREFETCH_SPDLOG} ${_extract_prefix}/spdlog-1.14.1)

file(ARCHIVE_EXTRACT
    INPUT ${_fetch_prefix}/taskflow-v3.7.0.tar.gz
    DESTINATION ${_extract_prefix}
)
set(ENV{CI_PREFETCH_TASKFLOW} ${_extract_prefix}/taskflow-3.7.0)

