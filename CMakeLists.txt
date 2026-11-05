cmake_minimum_required(VERSION 3.16)
project(m31enprs_framing VERSION 1.0.0 LANGUAGES C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Top-level detection. As a subproject (add_subdirectory / FetchContent) we
# must not change the consumer's build type, build our tests, or sanitizer-
# instrument the archive they link against.
# ---------------------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(FRAMING_IS_TOP_LEVEL ON)
else()
    set(FRAMING_IS_TOP_LEVEL OFF)
endif()

if(FRAMING_IS_TOP_LEVEL AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(FRAMING_IS_TOP_LEVEL AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(_tests_default ON)
else()
    set(_tests_default OFF)
endif()

option(FRAMING_BUILD_TESTS     "Build framing unit tests (GoogleTest)"              ${_tests_default})
option(FRAMING_TEST_SANITIZERS "Build library + tests with ASan/UBSan (tests only)" ON)
option(FRAMING_INSTALL         "Generate install/export rules"                       ${FRAMING_IS_TOP_LEVEL})

message(STATUS "framing: build type  = ${CMAKE_BUILD_TYPE}")
message(STATUS "framing: build tests = ${FRAMING_BUILD_TESTS}")

# ---------------------------------------------------------------------------
# Payload buffer size — must be set explicitly, no default.
# Affects struct layout and is part of the ABI.
# ---------------------------------------------------------------------------
if(NOT DEFINED FRM_MAX_PAYLOAD_LEN)
    message(FATAL_ERROR
        "FRM_MAX_PAYLOAD_LEN is required. Pass it via -DFRM_MAX_PAYLOAD_LEN=<bytes>.")
endif()
if(NOT FRM_MAX_PAYLOAD_LEN MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR
        "FRM_MAX_PAYLOAD_LEN must be a positive integer, got '${FRM_MAX_PAYLOAD_LEN}'.")
endif()
message(STATUS "FRM_MAX_PAYLOAD_LEN = ${FRM_MAX_PAYLOAD_LEN}")

# ---------------------------------------------------------------------------
# Dependency(s)
# ---------------------------------------------------------------------------
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cobs-c/cobs.c)
    message(FATAL_ERROR
        "cobs-c/cobs.c not found. Run: git submodule update --init")
endif()

# ---------------------------------------------------------------------------
# Library
# STATIC: FRM_MAX_PAYLOAD_LEN changes struct sizes so every consumer must
# agree with the library.
# ---------------------------------------------------------------------------
add_library(m31enprs_framing STATIC framing.c cobs-c/cobs.c)
add_library(m31enprs_framing::m31enprs_framing ALIAS m31enprs_framing)

set_target_properties(m31enprs_framing PROPERTIES
    EXPORT_NAME   m31enprs_framing
    PUBLIC_HEADER framing.h)

target_include_directories(m31enprs_framing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/m31enprs>)

target_compile_features(m31enprs_framing PRIVATE c_std_99)

target_compile_definitions(m31enprs_framing PUBLIC
    FRM_MAX_PAYLOAD_LEN=${FRM_MAX_PAYLOAD_LEN})

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(m31enprs_framing PRIVATE -Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# Tests (GoogleTest) with sanitizer
# ---------------------------------------------------------------------------
set(FRAMING_SANITIZE OFF)
if(FRAMING_BUILD_TESTS AND FRAMING_TEST_SANITIZERS AND CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set(FRAMING_SANITIZE ON)
    set(_san_flags -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_compile_options(m31enprs_framing PRIVATE ${_san_flags})
endif()

if(FRAMING_BUILD_TESTS)
    enable_language(CXX)
    include(CTest)
    include(FetchContent)

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        if(POLICY CMP0135)
            cmake_policy(SET CMP0135 NEW)
        endif()
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
        FetchContent_MakeAvailable(googletest)
    endif()
    include(GoogleTest)

    file(GLOB _test_sources CONFIGURE_DEPENDS test/*.cpp)
    foreach(_src ${_test_sources})
        get_filename_component(_exe ${_src} NAME_WE)
        add_executable(${_exe} ${_src})
        target_link_libraries(${_exe} PRIVATE m31enprs_framing GTest::gtest_main)
        if(FRAMING_SANITIZE)
            target_compile_options(${_exe} PRIVATE ${_san_flags})
            target_link_options(${_exe}    PRIVATE -fsanitize=address,undefined)
        endif()
        gtest_discover_tests(${_exe} PROPERTIES TIMEOUT 10)
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Install + export
#   find_package(m31enprs_framing) -> m31enprs_framing::m31enprs_framing
# ---------------------------------------------------------------------------
if(FRAMING_INSTALL AND NOT FRAMING_SANITIZE)
    set(_cmake_dir   ${CMAKE_INSTALL_LIBDIR}/cmake/m31enprs_framing)
    set(_include_dir ${CMAKE_INSTALL_INCLUDEDIR}/m31enprs)

    install(TARGETS m31enprs_framing
        EXPORT        m31enprs_framingTargets
        ARCHIVE       DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${_include_dir})

    # framing.h does #include "cobs-c/cobs.h", so it must sit next to it.
    # PUBLIC_HEADER flattens paths, hence the separate install.
    install(FILES cobs-c/cobs.h DESTINATION ${_include_dir}/cobs-c)

    install(EXPORT m31enprs_framingTargets
        NAMESPACE   m31enprs_framing::
        DESTINATION ${_cmake_dir})

    configure_package_config_file(
        cmake/m31enprs_framingConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/m31enprs_framingConfig.cmake
        INSTALL_DESTINATION ${_cmake_dir})

    # Arch-dependent on purpose: the archive is compiled for one target, so
    # find_package() rejects a 32-bit package for a 64-bit consumer and vice versa.
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/m31enprs_framingConfigVersion.cmake
        VERSION       ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/m31enprs_framingConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/m31enprs_framingConfigVersion.cmake
        DESTINATION ${_cmake_dir})

elseif(FRAMING_INSTALL AND FRAMING_SANITIZE)
    message(STATUS "framing: install skipped for sanitizer build")
endif()