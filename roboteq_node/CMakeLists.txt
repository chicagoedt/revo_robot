cmake_minimum_required(VERSION 2.8.3)
project(roboteq_node)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

add_message_files(
    FILES
    wheels_msg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
 )

#catkin_package(
#    CATKIN_DEPENDS
#    message_runtime
#)

catkin_package(
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(roboteq_node src/rosRoboteqDrv/main.cpp src/rosRoboteqDrv/rosRoboteqDrv.cpp src/roboteqCom/roboteqCom.cpp src/roboteqCom/roboteqThread.cpp src/serialConnector/serialPort.cpp)
target_link_libraries(roboteq_node ${catkin_LIBRARIES})
set_target_properties(roboteq_node PROPERTIES COMPILE_FLAGS -g)

add_dependencies(roboteq_node ${catkin_EXPORTED_TARGETS})
add_dependencies(roboteq_node roboteq_node_generate_messages_cpp)

