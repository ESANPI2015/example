cmake_minimum_required(VERSION 2.8)

project(octacore)

# they are rebuilt completely from scratch inside our build-tree
# each subdir is a "project", so some standard variables are defined therein (like protocol_SOURCE_DIR for example)
# but we ourself may be add_subdirectory'ed, with some of the targets already present. so check this!

# This is not possible without mars cmake file :/ So we have to compile this on our own
if(NOT TARGET ndlcom)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/ndlcom ndlcom)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/ndlcom/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/ndlcom/src)
endif(NOT TARGET ndlcom)
if(NOT TARGET representations)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/representations representations)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/representations/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/representations/src)
endif(NOT TARGET representations)

SET(ENV{PKG_CONFIG_PATH}
    ${CMAKE_BINARY_DIR}:$ENV{PKG_CONFIG_PATH}
    )

find_package(PkgConfig)
pkg_check_modules(octacore_PKGCONFIG REQUIRED
    ndlcom representations
    )

include_directories(${octacore_PKGCONFIG_INCLUDE_DIRS})
message(${octacore_PKGCONFIG_INCLUDE_DIRS})
link_directories(${octacore_PKGCONFIG_LIBRARY_DIRS})
add_definitions(${octacore_PKGCONFIG_CFLAGS_OTHER} -ggdb -Wall -pedantic -lm)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# add the executables
add_executable(cpu1
    cpu1_toplvl.c
    cpu1_comm.c
    cpu1_graph.c
    )
target_link_libraries(cpu1
    ndlcom representations
    )
add_dependencies(cpu1
    ndlcom representations
    )
add_executable(cpu2
    cpu2_toplvl.c
    cpu2_comm.c
    cpu2_graph.c
    )
target_link_libraries(cpu2
    ndlcom representations
    )
add_dependencies(cpu2
    ndlcom representations
    )
add_executable(cpu3
    cpu3_toplvl.c
    cpu3_comm.c
    cpu3_graph.c
    )
target_link_libraries(cpu3
    ndlcom representations
    )
add_dependencies(cpu3
    ndlcom representations
    )
add_executable(cpu4
    cpu4_toplvl.c
    cpu4_comm.c
    cpu4_graph.c
    )
target_link_libraries(cpu4
    ndlcom representations
    )
add_dependencies(cpu4
    ndlcom representations
    )
add_executable(cpu5
    cpu5_toplvl.c
    cpu5_comm.c
    cpu5_graph.c
    )
target_link_libraries(cpu5
    ndlcom representations
    )
add_dependencies(cpu5
    ndlcom representations
    )
add_executable(cpu6
    cpu6_toplvl.c
    cpu6_comm.c
    cpu6_graph.c
    )
target_link_libraries(cpu6
    ndlcom representations
    )
add_dependencies(cpu6
    ndlcom representations
    )
add_executable(cpu7
    cpu7_toplvl.c
    cpu7_comm.c
    cpu7_graph.c
    )
target_link_libraries(cpu7
    ndlcom representations
    )
add_dependencies(cpu7
    ndlcom representations
    )
add_executable(cpu8
    cpu8_toplvl.c
    cpu8_comm.c
    cpu8_graph.c
    )
target_link_libraries(cpu8
    ndlcom representations
    )
add_dependencies(cpu8
    ndlcom representations
    )
# add custom targets (symlinks)
add_custom_target(cpu1-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu1 ${CMAKE_SOURCE_DIR}/cpu1
    DEPENDS cpu1)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu1")
add_custom_target(cpu2-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu2 ${CMAKE_SOURCE_DIR}/cpu2
    DEPENDS cpu2)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu2")
add_custom_target(cpu3-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu3 ${CMAKE_SOURCE_DIR}/cpu3
    DEPENDS cpu3)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu3")
add_custom_target(cpu4-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu4 ${CMAKE_SOURCE_DIR}/cpu4
    DEPENDS cpu4)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu4")
add_custom_target(cpu5-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu5 ${CMAKE_SOURCE_DIR}/cpu5
    DEPENDS cpu5)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu5")
add_custom_target(cpu6-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu6 ${CMAKE_SOURCE_DIR}/cpu6
    DEPENDS cpu6)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu6")
add_custom_target(cpu7-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu7 ${CMAKE_SOURCE_DIR}/cpu7
    DEPENDS cpu7)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu7")
add_custom_target(cpu8-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/cpu8 ${CMAKE_SOURCE_DIR}/cpu8
    DEPENDS cpu8)
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/cpu8")
