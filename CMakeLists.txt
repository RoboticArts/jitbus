cmake_minimum_required(VERSION 3.0.2)
project(jitbus)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    serial
)

# Bind Python
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS src
   CATKIN_DEPENDS roscpp std_msgs serial
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	)

# Bind Python 
add_library(_jitbus_binding_so src/python/jitbus/jitbus_binding.cpp)
target_link_libraries(_jitbus_binding_so ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(_jitbus_binding_so PROPERTIES
	PREFIX ""
	LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install( 
  DIRECTORY src/core/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)