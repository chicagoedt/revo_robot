cmake_minimum_required(VERSION 2.8.3)
project(roboteq_node)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)


add_message_files(
        FILES
    wheels_msg.msg
    # Xbox_Button_Msg.msg
    # diagnostics_msg.msg
)

generate_messages(
     DEPENDENCIES
     std_msgs
 )

catkin_package(
        CATKIN_DEPENDS
    message_runtime
)



include_directories(include ${catkin_INCLUDE_DIRS})

add_library(roboteq_node_lib src/rosRoboteqDrv/rosRoboteqDrv.cpp src/roboteqCom/roboteqCom.cpp src/roboteqCom/roboteqThread.cpp src/serialConnector/serialPort.cpp)
target_link_libraries(roboteq_node_lib ${catkin_LIBRARIES})

add_executable(roboteq_node src/rosRoboteqDrv/main.cpp src/rosRoboteqDrv/rosRoboteqDrv.cpp src/roboteqCom/roboteqCom.cpp src/roboteqCom/roboteqThread.cpp src/serialConnector/serialPort.cpp)
target_link_libraries(roboteq_node ${catkin_LIBRARIES})
set_target_properties(roboteq_node PROPERTIES COMPILE_FLAGS -g)

add_dependencies(roboteq_node ${catkin_EXPORTED_TARGETS})
add_dependencies(roboteq_node_lib ${catkin_EXPORTED_TARGETS})
#add_dependencies(keyboard_teleop base_controller_gencpp)

catkin_add_gtest(roboteq_utest test/roboteq_utest.cpp)
target_link_libraries(roboteq_utest roboteq_node_lib)
