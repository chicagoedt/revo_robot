cmake_minimum_required(VERSION 2.8.3)
project(camera1394stereo)

find_package(catkin REQUIRED COMPONENTS 
	camera_info_manager 
	driver_base 
	dynamic_reconfigure 
	image_transport 
	nodelet 
	roscpp
	rospy
	tf)

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread)

# Resolve system dependency on libdc1394, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(DC1394_PKGCONF REQUIRED libdc1394-2)
find_path(DC1394_INCLUDE_DIR
  NAMES dc1394.h
  PATHS ${DC1394_PKGCONF_INCLUDE_DIRS}
)
find_library(DC1394_LIBRARY
  NAMES dc1394
  PATHS ${DC1394_PKGCONF_LIBRARY_DIRS}
)

# auto-generated dynamic reconfiguration GUI
generate_dynamic_reconfigure_options(cfg/Camera1394Stereo.cfg)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(cfg/cpp ${catkin_INCLUDE_DIRS})

# common source files for both node and nodelet driver interfaces
set(DRIVER_SOURCES 
  src/nodes/driver1394stereo.cpp
  src/nodes/dev_camera1394stereo.cpp
  src/nodes/featuresstereo.cpp
  src/nodes/format7stereo.cpp
  src/nodes/modes.cpp
  src/nodes/trigger.cpp)

# Mac OSX requires extra linker parameters
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(DC1394 dc1394 "-framework CoreServices")
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(DC1394 dc1394)
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")


add_executable(camera1394stereo_node src/nodes/camera1394stereo_node.cpp ${DRIVER_SOURCES})
add_dependencies(camera1394stereo_node camera1394stereo_gencfg) 
add_dependencies(camera1394stereo_node camera1394stereo_EXPORTED_TARGETS) 
target_link_libraries(camera1394stereo_node ${DC1394} ${catkin_LIBRARIES})

add_library(camera1394stereo_nodelet src/nodes/nodelet.cpp ${DRIVER_SOURCES})
add_dependencies(camera1394stereo_nodelet camera1394stereo_gencfg) 
add_dependencies(camera1394stereo_nodelet camera1394stereo_EXPORTED_TARGETS) 
target_link_libraries(camera1394stereo_nodelet ${DC1394} ${catkin_LIBRARIES})

# driver unit tests
#
#  * When -DDEVICE_TEST=true is configured: run tests that require
#    some real 1394 camera to be attached.
#
#  * Otherwise, run only tests that require no attached camera, as is
#    done on the ROS build machines.
#

#if (${DEVICE_TEST})
#  add_rostest(tests/camera_node_hz.test)
#  add_rostest(tests/camera_nodelet_hz.test)
#endif (${DEVICE_TEST})

#add_rostest(tests/no_device_node.test)
#add_rostest(tests/no_device_nodelet.test)
