# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "f110_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-If110_msgs:/home/patrick/f1_ws/src/f110_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(f110_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg" NAME_WE)
add_custom_target(_f110_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "f110_msgs" "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg" ""
)

get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg" NAME_WE)
add_custom_target(_f110_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "f110_msgs" "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg" "std_msgs/Header:f110_msgs/Wpnt"
)

get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg" NAME_WE)
add_custom_target(_f110_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "f110_msgs" "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg" ""
)

get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg" NAME_WE)
add_custom_target(_f110_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "f110_msgs" "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg" "std_msgs/Header:f110_msgs/Obstacle"
)

get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg" NAME_WE)
add_custom_target(_f110_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "f110_msgs" "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg" "std_msgs/Header:f110_msgs/Wpnt"
)

get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg" NAME_WE)
add_custom_target(_f110_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "f110_msgs" "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
)
_generate_msg_cpp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
)
_generate_msg_cpp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
)
_generate_msg_cpp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
)
_generate_msg_cpp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
)
_generate_msg_cpp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(f110_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(f110_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(f110_msgs_generate_messages f110_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_cpp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_cpp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_cpp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_cpp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_cpp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_cpp _f110_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f110_msgs_gencpp)
add_dependencies(f110_msgs_gencpp f110_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f110_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
)
_generate_msg_eus(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
)
_generate_msg_eus(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
)
_generate_msg_eus(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
)
_generate_msg_eus(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
)
_generate_msg_eus(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(f110_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(f110_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(f110_msgs_generate_messages f110_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_eus _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_eus _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_eus _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_eus _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_eus _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_eus _f110_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f110_msgs_geneus)
add_dependencies(f110_msgs_geneus f110_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f110_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
)
_generate_msg_lisp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
)
_generate_msg_lisp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
)
_generate_msg_lisp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
)
_generate_msg_lisp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
)
_generate_msg_lisp(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(f110_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(f110_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(f110_msgs_generate_messages f110_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_lisp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_lisp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_lisp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_lisp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_lisp _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_lisp _f110_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f110_msgs_genlisp)
add_dependencies(f110_msgs_genlisp f110_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f110_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
)
_generate_msg_nodejs(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
)
_generate_msg_nodejs(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
)
_generate_msg_nodejs(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
)
_generate_msg_nodejs(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
)
_generate_msg_nodejs(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(f110_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(f110_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(f110_msgs_generate_messages f110_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_nodejs _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_nodejs _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_nodejs _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_nodejs _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_nodejs _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_nodejs _f110_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f110_msgs_gennodejs)
add_dependencies(f110_msgs_gennodejs f110_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f110_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
)
_generate_msg_py(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
)
_generate_msg_py(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
)
_generate_msg_py(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
)
_generate_msg_py(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
)
_generate_msg_py(f110_msgs
  "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(f110_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(f110_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(f110_msgs_generate_messages f110_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Wpnt.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_py _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/WpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_py _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_py _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_py _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/OTWpntArray.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_py _f110_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patrick/f1_ws/src/f110_msgs/msg/FrenetState.msg" NAME_WE)
add_dependencies(f110_msgs_generate_messages_py _f110_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f110_msgs_genpy)
add_dependencies(f110_msgs_genpy f110_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f110_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f110_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(f110_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(f110_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f110_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(f110_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(f110_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f110_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(f110_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(f110_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f110_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(f110_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(f110_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f110_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(f110_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(f110_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
