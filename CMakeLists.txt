# Copyright (C) 2018-2019 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required (VERSION 2.8)

project(Drone)
set(TARGET_NAME "drone")

if (CMAKE_BUILD_TYPE STREQUAL "")
    message(STATUS "CMAKE_BUILD_TYPE not defined, 'Release' will be used")
    set(CMAKE_BUILD_TYPE "Release")
endif()

if (NOT(BIN_FOLDER))
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
        set (ARCH armv7l)
    elseif("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
        set (ARCH intel64)
    else()
        set (ARCH ia32)
    endif()

    set (BIN_FOLDER ${ARCH})
endif()

set (IE_MAIN_SAMPLES_DIR ${CMAKE_CURRENT_BINARY_DIR})

if(NOT(UNIX))
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER})
    set (CMAKE_LIBRARY_PATH ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER})
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER})
    set (CMAKE_COMPILE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER})
    set (CMAKE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER})
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER})
    set (LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER})
    set (LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_DIRECTORY}) # compatibility issue: linux uses LIBRARY_OUTPUT_PATH, windows uses LIBRARY_OUTPUT_DIRECTORY
else ()
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE}/lib)
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE}/lib)
    set (CMAKE_COMPILE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE})
    set (CMAKE_PDB_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE})
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE})
    set (LIBRARY_OUTPUT_DIRECTORY ${IE_MAIN_SAMPLES_DIR}/${BIN_FOLDER}/${CMAKE_BUILD_TYPE}/lib)
    set (LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_DIRECTORY}/lib)
endif()

if (WIN32)
    if (NOT "${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
        message(FATAL_ERROR "Only 64-bit supported on Windows")
    endif()

    set_property (DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_SCL_SECURE_NO_WARNINGS -DNOMINMAX")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc") #no asynchronous structured exception handling
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
    if (${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251 /wd4275 /wd4267") #disable some warnings
    endif()
else()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror") #treating warnings as errors
    if (APPLE)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-command-line-argument")
    elseif(UNIX)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self")
        if(NOT ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wmaybe-uninitialized")
        endif()
    endif()
endif()

####################################
## to use C++11
set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
if (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
    set (CMAKE_CXX_STANDARD 11)
    set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()
####################################

set (GFLAGS_IS_SUBPROJECT TRUE)
set (HAVE_SYS_STAT_H 1)
set (HAVE_INTTYPES_H 1)

add_subdirectory(thirdparty/gflags)
add_subdirectory(thirdparty/ceSerial)

if (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

include_directories (
        ${CMAKE_CURRENT_SOURCE_DIR}/common
        ${CMAKE_CURRENT_SOURCE_DIR}/common/format_reader
)
add_subdirectory(common/format_reader)

find_package(InferenceEngine 1.6 REQUIRED)

if (UNIX)
    set (LIB_DL dl)
endif()

# Find OpenCV components if exist
find_package(OpenCV COMPONENTS highgui QUIET)
if(NOT(OpenCV_FOUND))
    message(WARNING "OPENCV is disabled or not found, " ${TARGET_NAME} " skipped")
    return()
endif()

file (GLOB MAIN_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
     )

file (GLOB MAIN_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
     )

# Create named folders for the sources within the .vcproj
# Empty name lists them directly under the .vcproj
source_group("src" FILES ${MAIN_SRC})
source_group("include" FILES ${MAIN_HEADERS})

link_directories(${LIB_FOLDER})

# Create library file from sources.
add_executable(${TARGET_NAME} ${MAIN_SRC} ${MAIN_HEADERS})

add_dependencies(${TARGET_NAME} gflags)
add_dependencies(${TARGET_NAME} ceSerial)

if(APPLE)
    set_target_properties(${TARGET_NAME} PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Info.plist
    )
endif()

set_target_properties(
    ${TARGET_NAME} PROPERTIES
    "CMAKE_CXX_FLAGS" "${CMAKE_CXX_FLAGS} -fPIE"
    COMPILE_PDB_NAME ${TARGET_NAME}
)

target_link_libraries(${TARGET_NAME} IE::ie_cpu_extension ${InferenceEngine_LIBRARIES} gflags ceSerial ${OpenCV_LIBRARIES})

if(UNIX)
    target_link_libraries( ${TARGET_NAME} ${LIB_DL} pthread)
endif()