cmake_minimum_required(VERSION 2.8.3)
project(husky_interactive_markers)

find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs)

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(husky_interactive_markers_server src/marker_server.cpp)
set_target_properties(husky_interactive_markers_server PROPERTIES 
  OUTPUT_NAME marker_server PREFIX "")

target_link_libraries(husky_interactive_markers_server
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS husky_interactive_markers_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

