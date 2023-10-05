# Ugly hack to make CMake discover libomp from Homebrew on Github Actions
if(NOT APPLE)
    return()
endif()

set(_omp_path NOTFOUND)

foreach(omp_prefix /usr/local /opt/homebrew)
    set(candidate_path ${omp_prefix}/opt/libomp)
    if(EXISTS ${candidate_path})
        set(_omp_path ${candidate_path})
        break()
    endif()
endforeach()

if(_omp_path STREQUAL "NOTFOUND")
    message(FATAL_ERROR "could not find OpenMP path!")
endif()

list(APPEND CMAKE_PREFIX_PATH ${_omp_path})

message(STATUS "OpenMP prefix: ${_omp_path}")

find_path(_omp_include_dir
    NAMES omp.h
    PATHS ${_omp_path}/include
)
message(STATUS "OpenMP include dir: ${_omp_include_dir}")

foreach(_lang IN ITEMS C CXX)
    if(CMAKE_${_lang}_COMPILER_ID MATCHES "AppleClang")
        set(OpenMP_${_lang}_LIB_NAMES "omp")
        set(OpenMP_${_lang}_INCLUDE_DIR ${_omp_include_dir})
    endif()
endforeach()

find_library(OpenMP_omp_LIBRARY
    NAMES omp
    PATHS ${omp_path}/lib
  )

set(CMAKE_DISABLE_PRECOMPILE_HEADERS ON)

