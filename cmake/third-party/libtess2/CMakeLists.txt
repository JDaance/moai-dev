cmake_minimum_required ( VERSION 2.8.5 )
project ( libtess2 )

set ( CMAKE_CURRENT_SOURCE_DIR "../../3rdparty/libtess2/Source/" )

set ( LIBTESS2_INCLUDES "../../3rdparty/libtess2/Include" CACHE INTERNAL "doc string" )

set ( LIBTESS2_SOURCES 

    ${CMAKE_CURRENT_SOURCE_DIR}bucketalloc.c
    ${CMAKE_CURRENT_SOURCE_DIR}dict.c
    ${CMAKE_CURRENT_SOURCE_DIR}geom.c
    ${CMAKE_CURRENT_SOURCE_DIR}mesh.c
    ${CMAKE_CURRENT_SOURCE_DIR}priorityq.c
    ${CMAKE_CURRENT_SOURCE_DIR}sweep.c
    ${CMAKE_CURRENT_SOURCE_DIR}tess.c

CACHE INTERNAL "doc string" )
