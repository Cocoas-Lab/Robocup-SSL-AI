cmake_minimum_required(VERSION 3.10)

project(ai CXX C)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Disable in-source build
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Set compiler flags
set(CMAKE_CXX_EXTENSIONS  OFF)
set(CMAKE_CXX_STANDARD    17)
set(CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS}         -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS}  -Wl,-O1,--sort-common,--as-needed,-z,relro")

if(CMAKE_GENERATOR STREQUAL "Ninja")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
  endif()
endif()

# Sanitizers
if(ENABLE_SANITIZERS)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Enabling GCC's address sanitizer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Enabling Clang's address sanitizer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
  endif()
endif()

# Code coverage
if(ENABLE_CODE_COVERAGE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

# Required libraries
find_package(Boost 1.61.0 COMPONENTS coroutine system unit_test_framework REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Protobuf 3.2 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Gtkmm3 gtkmm-3.0)

enable_testing()

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${Boost_INCLUDE_DIRS}
  ${Gtkmm3_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

add_subdirectory(3rd)
add_subdirectory(src)
add_subdirectory(test)
