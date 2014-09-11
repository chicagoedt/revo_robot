cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)



add_message_files(
    FILES
    wheels_msg.msg
    Xbox_Button_Msg.msg
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

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})
set_target_properties(joystick_teleop PROPERTIES COMPILE_FLAGS -g)

add_dependencies(roboteq_node ${catkin_EXPORTED_TARGETS})

add_dependencies(joystick_teleop ${catkin_EXPORTED_TARGETS})

