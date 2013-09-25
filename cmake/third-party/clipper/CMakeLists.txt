cmake_minimum_required ( VERSION 2.8.5 )
project ( clipper )

set ( CMAKE_CURRENT_SOURCE_DIR "../../3rdparty/clipper/" )

set ( CLIPPER_INCLUDES "../../3rdparty/clipper/" CACHE INTERNAL "doc string" )

set ( CLIPPER_SOURCES 

    ${CMAKE_CURRENT_SOURCE_DIR}clipper.cpp

CACHE INTERNAL "doc string" )
