Steps to use ROS


(https://docs.ros.org/en/eloquent/Tutorials/Creating-Your-First-ROS2-Package.html)


Create a ROS2 packages: On a terminal 

//Create a workspace:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

//Create a package build for our future code:
ros2 pkg create my_robot_controller

//Build
cd ~/ros2_ws
colcon build
source install/setup.bash


Put your code here: ~/ros2_ws/src/my_robot_controller/src
Put your messages here: ~/ros2_ws/src/my_robot_controller/msg


------------------------------------------------
To build another package next time:
colcon build --packages-select my_robot_controller
source install/setup.bash
------------------------------------------------

if you have a problem with "source install/setup.bash" (follow these steps otherwise continue) -------------------


echo $0; echo $SHELL

if your output is :
zsh
/bin/zsh

then write:
exec bash

and retry : source install/setup.bash
----------------------------------------------------------------------------------

//Running
exec bash
cd ros2_workspace
colcon build
source install/setup.bash
cd src/my_robot_controller/src

in 3 terminals, launch each
python3 robot_publisher.py
python3 robot_subscriber.py
python3 plant_publisher.py 


to open the database robot_coordinates.db
cd ~/ros2_workspace/src/my_robot_controller/src
sqlite3 robot_coordinates.db

to see the robot coord:
SELECT * FROM robot_coordinates;

to see the plant coord:
SELECT * FROM plant_coordinates;

to quit:
.exit

CMakeLists.txt:-----------------------------------------------
cmake_minimum_required(VERSION 3.8)

project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCoordinates.msg"
  "msg/PlantCoordinates.msg"
)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
-------------------------------------------------------------
package.xml:----------------------------------------------------
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="student@todo.todo">student</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rosidl_default_generators</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
-----------------------------------------------------------------

