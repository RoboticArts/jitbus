cmake_minimum_required(VERSION 3.8)
project(jitbus VERSION 0.0.0)

# ----------------------------------------
# Compiler
# ----------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------
# Dependencies
# ----------------------------------------
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED IMPORTED_TARGET libserial)

# ----------------------------------------
# Target
# ----------------------------------------

add_library(jitbus INTERFACE)

target_include_directories(jitbus
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(jitbus INTERFACE cxx_std_20)

target_link_libraries(jitbus
  INTERFACE
    PkgConfig::SERIAL
    Threads::Threads
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS jitbus EXPORT jitbusTargets)

install(
  DIRECTORY src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  EXPORT jitbusTargets
  FILE jitbusTargets.cmake
  NAMESPACE jitbus::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/jitbus
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/jitbusConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/jitbusConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/jitbusConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/jitbus
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/jitbusConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/jitbusConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/jitbus
)

# ----------------------------------------
# Examples
# ----------------------------------------

option(BUILD_EXAMPLES "Build examples" ON)

if(BUILD_EXAMPLES)

  foreach(name serial_example serial_loopback serial_read serial_write)
    add_executable(${name} examples/linux/${name}.cpp)
    target_link_libraries(${name} PRIVATE jitbus)
  endforeach()

endif()