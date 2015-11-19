cmake_minimum_required(VERSION 2.8.3)
project(rqt_igvc_dashboard)

INCLUDE(CheckCCompilerFlag)

#CHECK_C_COMPILER_FLAG(-fvisibility=hidden HAS_VISIBILITY)
#IF(HAS_VISIBILITY)
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
#ENDIF()

#IF(NOT $ENV{STRICT_CHECK} STREQUAL true)
#  CHECK_C_COMPILER_FLAG(-Wl,--no-undefined HAS_NO_UNDEFINED)
#  IF(HAS_NO_UNDEFINED)
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-undefined")
#  ENDIF()
#  CHECK_C_COMPILER_FLAG(-Wl,--no-allow-shlib-undefined HAS_NO_ALLOW_SHLIB_UNDEFINED)
#  IF(HAS_NO_ALLOW_SHLIB_UNDEFINED)
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-allow-shlib-undefined")
#  ENDIF()
#  CHECK_C_COMPILER_FLAG(-Wl,--as-needed HAS_AS_NEEDED)
#  IF(HAS_AS_NEEDED)
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--as-needed")
#  ENDIF()
#ENDIF()

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    message_generation
    roscpp
    rqt_gui
    rqt_gui_cpp
    sensor_msgs
    std_msgs
)
# Find Qwt
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules/")
#find_package(Qwt REQUIRED)
#include_directories(
#    /opt/qwt-6.1.2/include
#)

#find_package(Boost REQUIRED COMPONENTS
#    program_options
#)

find_package(Qt4 REQUIRED
    QtCore
    QtGui
    REQUIRED
)

include(${QT_USE_FILE})

##SET( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/kde4/apps/cmake/modules/" )

set(rqt_igvc_dashboard_SRCS
  src/igvcDashboard.cpp
)

set(rqt_igvc_dashboard_HDRS
  include/rqt_igvc_dashboard/igvcDashboard.h
)

set(rqt_igvc_dashboard_UIS
  resource/igvcDashboard.ui
)

set(rqt_igvc_dashboard_INCLUDE_DIRECTORIES
  include/rqt_igvc_dashboard
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS  ${rqt_igvc_dashboard_INCLUDE_DIRECTORIES}
  LIBRARIES     ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure message_generation roscpp rqt_gui rqt_gui_cpp sensor_msgs std_msgs
)

qt4_wrap_cpp(
    rqt_igvc_dashboard_MOCS
    ${rqt_igvc_dashboard_HDRS}
)

qt4_wrap_ui(
    rqt_igvc_dashboard_UIS_H
    ${rqt_igvc_dashboard_UIS}
)

include_directories(
    include
    ${rqt_igvc_dashboard_INCLUDE_DIRECTORIES}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(
    ${PROJECT_NAME}
    ${rqt_igvc_dashboard_SRCS}
    ${rqt_igvc_dashboard_MOCS}
    ${rqt_igvc_dashboard_UIS_H}
)

#find_library( qwt /opt/qwt-6.1.2/lib/)

target_link_libraries(
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${QT_QTCORE_LIBRARY}
   ${QT_QTGUI_LIBRARY}
   ${Boost_LIBRARIES}
#   qwt
)

add_dependencies(${PROJECT_NAME} rqt_igvc_dashboard_gencpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install(PROGRAMS scripts/auto_campus
##  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##)

##install(DIRECTORY include/${PROJECT_NAME}/
install(DIRECTORY resource
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



