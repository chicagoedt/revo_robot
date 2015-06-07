cmake_minimum_required(VERSION 2.8.3)
project(line_detection)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs dynamic_reconfigure image_geometry sensor_msgs message_filters tf2_ros geometry_msgs image_transport cv_bridge)

generate_dynamic_reconfigure_options(
  cfg/LineDetection.cfg
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


include_directories(include ${catkin_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIR})

#add_executable( lswms include/LSWMS.cpp include/LSWMS.h src/lswms_node.cpp)
#TARGET_LINK_LIBRARIES( lswms ${OpenCV_LIBS} ${catkin_LIBRARIES})
