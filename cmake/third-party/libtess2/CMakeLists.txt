cmake_minimum_required ( VERSION 2.8.5 )
project ( libtess2 )

set ( CMAKE_CURRENT_SOURCE_DIR "${MOAI_ROOT}/3rdparty/libtess2/" )

set ( LIBTESS2_INCLUDES "../../3rdparty/libtess2/Include" CACHE INTERNAL "doc string" )

file ( GLOB_RECURSE LIBTESS2_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Source/*.c" )
file ( GLOB_RECURSE LIBTESS2_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/Source/*.h" )

add_library( libtess2 STATIC ${LIBTESS2_SOURCES} ${LIBTESS2_HEADERS} )

target_include_directories ( libtess2 PUBLIC ${LIBTESS2_INCLUDES})
set_target_properties( libtess2 PROPERTIES FOLDER Third-Party )