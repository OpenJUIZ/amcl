cmake_minimum_required(VERSION 3.10)

project(amcl VERSION 1.1.0 LANGUAGES C CXX)

# AMCL (Adaptive Monte Carlo Localization) library from ROS navigation, without ROS.
#
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
#
# As a subproject (add_subdirectory), link the target amcl::amcl. The tests are built
# only when this is the top-level project (AMCL_BUILD_TESTS).

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(AMCL_IS_TOP_LEVEL ON)
else ()
  set(AMCL_IS_TOP_LEVEL OFF)
endif ()
option(AMCL_BUILD_TESTS "Build the tests" ${AMCL_IS_TOP_LEVEL})
option(AMCL_INSTALL "Generate the install target" ${AMCL_IS_TOP_LEVEL})

add_library(amcl
  src/amcl/map/map.c
  src/amcl/map/map_cspace.cpp
  src/amcl/map/map_draw.c
  src/amcl/map/map_range.c
  src/amcl/map/map_store.c
  src/amcl/pf/eig3.c
  src/amcl/pf/pf.c
  src/amcl/pf/pf_draw.c
  src/amcl/pf/pf_kdtree.c
  src/amcl/pf/pf_pdf.c
  src/amcl/pf/pf_vector.c
  src/amcl/sensors/amcl_laser.cpp
  src/amcl/sensors/amcl_odom.cpp
  src/amcl/sensors/amcl_sensor.cpp
)
add_library(amcl::amcl ALIAS amcl)

target_include_directories(amcl
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/amcl/include
)

# drand48()/srand48() are POSIX. Where they are missing (e.g. Windows), portable_utils.hpp
# provides replacements built on rand()/srand().
include(CheckSymbolExists)
check_symbol_exists(drand48 "stdlib.h" AMCL_HAVE_DRAND48)
if (AMCL_HAVE_DRAND48)
  target_compile_definitions(amcl PRIVATE HAVE_DRAND48)
endif ()

if (MSVC)
  target_compile_definitions(amcl PRIVATE _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
endif ()

find_library(AMCL_MATH_LIBRARY m)
if (AMCL_MATH_LIBRARY)
  target_link_libraries(amcl PUBLIC ${AMCL_MATH_LIBRARY})
endif ()

if (AMCL_BUILD_TESTS)
  include(CTest)
  if (BUILD_TESTING)
    add_subdirectory(test)
  endif ()
endif ()

if (AMCL_INSTALL)
  include(GNUInstallDirs)
  install(TARGETS amcl EXPORT amclTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(DIRECTORY include/amcl DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(EXPORT amclTargets NAMESPACE amcl:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/amcl
    FILE amclConfig.cmake)
endif ()
