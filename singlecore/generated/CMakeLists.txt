cmake_minimum_required(VERSION 2.8)

project(test)

# they are rebuilt completely from scratch inside our build-tree
# each subdir is a "project", so some standard variables are defined therein (like protocol_SOURCE_DIR for example)
# but we ourself may be add_subdirectory'ed, with some of the targets already present. so check this!

# This is not possible without mars cmake file :/ So we have to compile this on our own
if(NOT TARGET ndlcom)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/ndlcom ndlcom)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/ndlcom/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/ndlcom/src)
endif(NOT TARGET ndlcom)
if(NOT TARGET representations)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/representations representations)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/representations/include)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/representations/src)
endif(NOT TARGET representations)

SET(ENV{PKG_CONFIG_PATH}
    ${CMAKE_BINARY_DIR}:$ENV{PKG_CONFIG_PATH}
    )

find_package(PkgConfig)
pkg_check_modules(test_PKGCONFIG REQUIRED
    ndlcom representations
    )

include_directories(${test_PKGCONFIG_INCLUDE_DIRS})
message(${test_PKGCONFIG_INCLUDE_DIRS})
link_directories(${test_PKGCONFIG_LIBRARY_DIRS})
add_definitions(${test_PKGCONFIG_CFLAGS_OTHER} -ggdb -Wall -pedantic -lm)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# add the executables
add_executable(transformer
    cpu.c
    cpu_comm.c
    cpu_graph.c
    )
target_link_libraries(transformer
    ndlcom representations
    )
add_dependencies(transformer
    ndlcom representations
    )
# add custom targets (symlinks)
add_custom_target(transformer-lazy-symlink ALL
    ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/transformer ${CMAKE_SOURCE_DIR}/transformer
    DEPENDS transformer)
# and clean them up
set_directory_properties(PROPERTIES
    ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_SOURCE_DIR}/transformer")
