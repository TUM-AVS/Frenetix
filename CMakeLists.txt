### Definition of crccosy

set(CRCCOSY_SRC_ROOT ${CRCCOSY_ROOT}/src)

set(LIB_CRCCOSY_SRC
    ${CRCCOSY_SRC_ROOT}/segment.cc
    ${CRCCOSY_SRC_ROOT}/curvilinear_coordinate_system.cc
    ${CRCCOSY_SRC_ROOT}/util.cc
    ${CRCCOSY_SRC_ROOT}/serialize/serialize_curvil_cs.cc
    ${CRCCOSY_SRC_ROOT}/serialize/public/serialize_public.cc
  )


find_package( Boost REQUIRED)

add_library(crccosy STATIC ${LIB_CRCCOSY_SRC})

set_property(TARGET crccosy PROPERTY POSITION_INDEPENDENT_CODE ON)

if(NOT WIN32)
target_link_libraries(crccosy
    PUBLIC
        Eigen3::Eigen
	OpenMP::OpenMP_CXX)
else()
	target_compile_options(crccosy PRIVATE "-fopenmp")
	target_link_libraries(crccosy
    PUBLIC
        Eigen3::Eigen)
	target_link_libraries(crccosy  PRIVATE "-fopenmp")
endif()


target_compile_features(crccosy
    PUBLIC
        cxx_lambdas cxx_auto_type
    )
target_include_directories(crccosy
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CRCCOSY_ROOT}/include>
    PRIVATE
        ${CRCCOSY_ROOT}
        ${Boost_INCLUDE_DIRS} 
    )



if(NOT TARGET s11n::s11n)
    message(STATUS "Serialization and pickling support have been disabled for crccosy")
    target_compile_definitions(crccosy PRIVATE -DENABLE_SERIALIZER=0)
else()
    message(STATUS "Serialization and pickling support have been enabled for crccosy")
    target_link_libraries(crccosy PUBLIC s11n::s11n)
    target_compile_definitions(crccosy PRIVATE -DENABLE_SERIALIZER=1)
endif()
