cmake_minimum_required(VERSION 3.0.2)
project(balise)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Obj.msg
  ObjArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(

 CATKIN_DEPENDS rospy geometry_msgs std_msgs message_runtime

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ros_main.py
  scripts/ros_sub.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
