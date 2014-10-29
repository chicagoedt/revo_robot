cmake_minimum_required(VERSION 2.8.3)
project(line_detection)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/LineDetection.cfg
  cfg/ImageResizer.cfg
)

#catkin_python_setup()

#add_message_files(
#        FILES
#)

#generate_messages(
#	DEPENDENCIES
#	std_msgs
#)

#catkin_package(
#       	CATKIN_DEPENDS
#	message_runtime
#)



include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(line_detection_test src/line_detection_test_main.cpp src/line_detection_test.cpp)
#target_link_libraries(line_detection_test ${catkin_LIBRARIES})

#add_dependencies(keyboard_teleop base_controller_gencpp)
