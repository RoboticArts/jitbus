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
  add_executable(serial_example
    examples/linux/serial_example.cpp
  )

  target_link_libraries(serial_example
    PRIVATE jitbus
  )
endif()

# cmake_minimum_required(VERSION 3.8)
# project(jitbus)

# option(BUILD_ROS_EXAMPLES "Build ROS examples" OFF)
# option(BUILD_GCC_EXAMPLES "Build GCC examples" OFF)

# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     std_msgs
#     serial
# )

# # Bind Python
# find_package(Boost REQUIRED COMPONENTS python)
# find_package(PythonLibs 2.7 REQUIRED)

# catkin_python_setup()

# catkin_package(
#    INCLUDE_DIRS src
#    CATKIN_DEPENDS roscpp std_msgs serial
# )

# include_directories(
# 	${catkin_INCLUDE_DIRS}
# 	${Boost_INCLUDE_DIRS}
# 	${PYTHON_INCLUDE_DIRS}
# 	)

# # Bind Python 
# add_library(_jitbus_binding_so src/python/jitbus/jitbus_binding.cpp)
# target_link_libraries(_jitbus_binding_so ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# set_target_properties(_jitbus_binding_so PROPERTIES
# 	PREFIX ""
# 	LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

# install( 
#   DIRECTORY src/core/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# # Build examples
# if(BUILD_ROS_EXAMPLES)

#   include_directories(
#     src
#   )

#   add_library(${PROJECT_NAME}_ros
#     examples/linux/serial_example_ros.cpp
#   )

#   add_executable(serial_example_ros examples/linux/serial_example_ros.cpp)
#   target_link_libraries(serial_example_ros ${catkin_LIBRARIES})

# endif()


# if(BUILD_GCC_EXAMPLES)

#   include_directories(
#     src
#   )

#   add_library(${PROJECT_NAME}_gcc
#     examples/linux/serial_example_gcc.cpp
#   )

#   add_executable(serial_example_gcc examples/linux/serial_example_gcc.cpp)
#   target_link_libraries(serial_example_gcc ${catkin_LIBRARIES})

# endif()
