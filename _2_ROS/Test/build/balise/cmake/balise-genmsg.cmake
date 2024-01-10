# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "balise: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ibalise:/home/student/Desktop/Test/src/balise/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(balise_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/Obj.msg" NAME_WE)
add_custom_target(_balise_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "balise" "/home/student/Desktop/Test/src/balise/msg/Obj.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg" NAME_WE)
add_custom_target(_balise_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "balise" "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg" "balise/Obj:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(balise
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balise
)
_generate_msg_cpp(balise
  "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg"
  "${MSG_I_FLAGS}"
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balise
)

### Generating Services

### Generating Module File
_generate_module_cpp(balise
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balise
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(balise_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(balise_generate_messages balise_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/Obj.msg" NAME_WE)
add_dependencies(balise_generate_messages_cpp _balise_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg" NAME_WE)
add_dependencies(balise_generate_messages_cpp _balise_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balise_gencpp)
add_dependencies(balise_gencpp balise_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balise_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(balise
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balise
)
_generate_msg_eus(balise
  "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg"
  "${MSG_I_FLAGS}"
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balise
)

### Generating Services

### Generating Module File
_generate_module_eus(balise
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balise
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(balise_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(balise_generate_messages balise_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/Obj.msg" NAME_WE)
add_dependencies(balise_generate_messages_eus _balise_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg" NAME_WE)
add_dependencies(balise_generate_messages_eus _balise_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balise_geneus)
add_dependencies(balise_geneus balise_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balise_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(balise
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balise
)
_generate_msg_lisp(balise
  "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg"
  "${MSG_I_FLAGS}"
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balise
)

### Generating Services

### Generating Module File
_generate_module_lisp(balise
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balise
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(balise_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(balise_generate_messages balise_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/Obj.msg" NAME_WE)
add_dependencies(balise_generate_messages_lisp _balise_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg" NAME_WE)
add_dependencies(balise_generate_messages_lisp _balise_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balise_genlisp)
add_dependencies(balise_genlisp balise_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balise_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(balise
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balise
)
_generate_msg_nodejs(balise
  "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg"
  "${MSG_I_FLAGS}"
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balise
)

### Generating Services

### Generating Module File
_generate_module_nodejs(balise
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balise
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(balise_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(balise_generate_messages balise_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/Obj.msg" NAME_WE)
add_dependencies(balise_generate_messages_nodejs _balise_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg" NAME_WE)
add_dependencies(balise_generate_messages_nodejs _balise_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balise_gennodejs)
add_dependencies(balise_gennodejs balise_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balise_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(balise
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise
)
_generate_msg_py(balise
  "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg"
  "${MSG_I_FLAGS}"
  "/home/student/Desktop/Test/src/balise/msg/Obj.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise
)

### Generating Services

### Generating Module File
_generate_module_py(balise
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(balise_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(balise_generate_messages balise_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/Obj.msg" NAME_WE)
add_dependencies(balise_generate_messages_py _balise_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/Desktop/Test/src/balise/msg/ObjArray.msg" NAME_WE)
add_dependencies(balise_generate_messages_py _balise_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(balise_genpy)
add_dependencies(balise_genpy balise_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS balise_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balise)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/balise
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(balise_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(balise_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balise)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/balise
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(balise_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(balise_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balise)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/balise
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(balise_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(balise_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balise)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/balise
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(balise_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(balise_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/balise
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(balise_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(balise_generate_messages_py geometry_msgs_generate_messages_py)
endif()
