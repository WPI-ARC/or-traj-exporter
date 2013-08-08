cmake_minimum_required(VERSION 2.6)
project(ortconv)

message (STATUS "Creating Makefile for ortconv")

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
       FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")

message("CMAKE_DEBUG_POSTFIX : " ${CMAKE_DEBUG_POSTFIX})

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_scripts")

find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
message("LIBXML2_INCLUDE_DIR : " ${LIBXML2_INCLUDE_DIR})
message("LIBXML2_LIBRARY_DIRS : " ${LIBXML2_LIBRARY_DIRS})
message("LIBXML2_LIBRARIES : " ${LIBXML2_LIBRARIES})

# Search all header and source files
set( project_src src/converter.cpp src/or_trajectory.cpp src/utils.cpp )
set( project_hdr src/converter.hpp src/or_trajectory.hpp src/utils.hpp src/joint_map.hpp )

add_library( ${PROJECT_NAME} SHARED ${project_src} ${project_hdr} ${lib_source} ${RobotSim_libs} ${LIBXML2_LIBRARIES} )
add_executable( tconv src/converter.cpp )
target_link_libraries( tconv ${PROJECT_NAME} ${LIBXML2_LIBRARIES} )

install(FILES ${project_hdr} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ortconv)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

