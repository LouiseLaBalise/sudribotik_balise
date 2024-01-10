                                     ROS with docker
(Not recommended)
Run:
sudo docker run -it ros:noetic

To open a new terminal: 
sudo docker exec -it 6a317b60732e bash 

Quit:
Ctrl + D

Source setup bash
source /opt/ros/noetic/setup.bash

(Recommended)

->To run in a terminal container ros (best method: in your bash terminal):
-------------------------------------------------------------
sudo docker run -it --rm --user=$(id -u $USER):$(id -g $USER) --env="DISPLAY" --volume="/etc/group:/etc/group:ro" --volume="/etc/passwd:/etc/passwd:ro" --volume="/etc/shadow:/etc/shadow:ro" --volume="/etc/sudoers.d:/etc/sudoers.d:ro" --net host -v /home:/home -v ~/Volumes:/home/usr/ ros-noetic-container

To open a window terminal (just for esthetics):
terminator -u

Verify your container: (You need to be in Ubuntu 20.04)
lsb_release -a

Path:
cd ../student/Desktop/Test/

ROS PROCEDURES:
Create ROS1 package: (in \src directory)
catkin_create_pkg balise roscpp std_msgs message_generation

inside ~/Test/src/balise/scripts
your code (.py) here

inside ~/Test/src/balise/msg
your message (.msg) here

To build ROS1 package: (in workspace directory (~/Test/)
catkin_make

Unfortunately you need to build yourself the CMakeLists.txt (~/Test/src/balise/). Don't touch the one in (~/Test/src/).
You need to build the package.xml either.
To help you, I put mine below.

Don't forget the setup bash:
source devel/setup.bash

To RUN ROS
Open 4 terminals
roscore
rosrun balise robot_publisher.py
rosrun balise plant_publisher.py
rosrun balise robot_subscriber.py

ou
inside ~/Test/src/balise/scripts

python3 robot_publisher.py 
python3 plant_publisher.py 
python3 robot_subscriber.py 

SQL
to open the database robot_coordinates.db
cd ~/Desktop/Test/src/balise/src
sqlite3 robot_coordinates.db

to see the robot coord:
SELECT * FROM robot_coordinates;

to see the plant coord:
SELECT * FROM plant_coordinates;

to quit:
.exit
----------------------------------------------------------------------------------------------------------------
CMAKELIST.TXT
cmake_minimum_required(VERSION 2.8.3)
project(balise)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation  # Include if you have custom messages
)

# Add message files if you have any
add_message_files(
  FILES
  msg/RobotCoordinates.msg
  msg/PlantCoordinates.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime  # Add other dependencies if needed
)

# Specify include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/robot_publisher.py
  scripts/robot_subscriber.py
  scripts/plant_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
-----------------------------------------------------------------------
PACKAGE.XML
<?xml version="1.0"?>
<package format="2">
  <name>balise</name>
  <version>0.0.0</version>
  <description>The balise package</description>

  <maintainer email="student@todo.todo">student</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>

  <exec_depend>std_msgs</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>



  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
-----------------------------------------------------------------------
SETUP.PY

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['balise'],
        package_dir={'': 'src'}

)

setup(**d)
-----------------------------------------------------------------------
