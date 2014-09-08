cmake_minimum_required(VERSION 2.8.3)
project(robot_setup_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

#Generate added messages and services with any dependencies listed here
#generate_messages(
#	DEPENDENCIES
#	geometry_msgs
#)

catkin_package(

 CATKIN_DEPENDS geometry_msgs roscpp tf

)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(robot_setup_tf
#   src/${PROJECT_NAME}/robot_setup_tf.cpp
# )

## Declare a cpp executable
add_executable(robot_tf_publisher src/broadcaster/tf_broadcaster.cpp)
#add_executable(robot_tf_listener src/listener/tf_listener.cpp src/listener/TFListener.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot_setup_tf_node robot_setup_tf_generate_messages_cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(robot_tf_publisher ${catkin_LIBRARIES})
#target_link_libraries(robot_tf_listener  ${catkin_LIBRARIES})
