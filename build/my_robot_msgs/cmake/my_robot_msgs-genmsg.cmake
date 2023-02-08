# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "my_robot_msgs: 7 messages, 0 services")

set(MSG_I_FLAGS "-Imy_robot_msgs:/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(my_robot_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg" NAME_WE)
add_custom_target(_my_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_robot_msgs" "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg" "actionlib_msgs/GoalStatus:my_robot_msgs/MoveUntilActionResult:my_robot_msgs/MoveUntilActionGoal:my_robot_msgs/MoveUntilResult:actionlib_msgs/GoalID:my_robot_msgs/MoveUntilGoal:my_robot_msgs/MoveUntilActionFeedback:std_msgs/Header:my_robot_msgs/MoveUntilFeedback"
)

get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg" NAME_WE)
add_custom_target(_my_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_robot_msgs" "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:my_robot_msgs/MoveUntilGoal"
)

get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg" NAME_WE)
add_custom_target(_my_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_robot_msgs" "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:my_robot_msgs/MoveUntilResult:std_msgs/Header"
)

get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg" NAME_WE)
add_custom_target(_my_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_robot_msgs" "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:my_robot_msgs/MoveUntilFeedback"
)

get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg" NAME_WE)
add_custom_target(_my_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_robot_msgs" "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg" ""
)

get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg" NAME_WE)
add_custom_target(_my_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_robot_msgs" "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg" ""
)

get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg" NAME_WE)
add_custom_target(_my_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_robot_msgs" "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_cpp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_cpp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_cpp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_cpp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_cpp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_cpp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(my_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(my_robot_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(my_robot_msgs_generate_messages my_robot_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_cpp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_cpp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_cpp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_cpp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_cpp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_cpp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_cpp _my_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_robot_msgs_gencpp)
add_dependencies(my_robot_msgs_gencpp my_robot_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_robot_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_eus(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_eus(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_eus(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_eus(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_eus(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_eus(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(my_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(my_robot_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(my_robot_msgs_generate_messages my_robot_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_eus _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_eus _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_eus _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_eus _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_eus _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_eus _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_eus _my_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_robot_msgs_geneus)
add_dependencies(my_robot_msgs_geneus my_robot_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_robot_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_lisp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_lisp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_lisp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_lisp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_lisp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_lisp(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(my_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(my_robot_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(my_robot_msgs_generate_messages my_robot_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_lisp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_lisp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_lisp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_lisp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_lisp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_lisp _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_lisp _my_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_robot_msgs_genlisp)
add_dependencies(my_robot_msgs_genlisp my_robot_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_robot_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_nodejs(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_nodejs(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_nodejs(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_nodejs(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_nodejs(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_nodejs(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(my_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(my_robot_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(my_robot_msgs_generate_messages my_robot_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_nodejs _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_nodejs _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_nodejs _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_nodejs _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_nodejs _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_nodejs _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_nodejs _my_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_robot_msgs_gennodejs)
add_dependencies(my_robot_msgs_gennodejs my_robot_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_robot_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_py(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_py(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_py(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_py(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_py(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
)
_generate_msg_py(my_robot_msgs
  "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(my_robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(my_robot_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(my_robot_msgs_generate_messages my_robot_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilAction.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_py _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_py _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_py _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilActionFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_py _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilGoal.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_py _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilResult.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_py _my_robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bonato/catkin_ws/devel/share/my_robot_msgs/msg/MoveUntilFeedback.msg" NAME_WE)
add_dependencies(my_robot_msgs_generate_messages_py _my_robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_robot_msgs_genpy)
add_dependencies(my_robot_msgs_genpy my_robot_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_robot_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_robot_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(my_robot_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(my_robot_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(my_robot_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_robot_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(my_robot_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(my_robot_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(my_robot_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_robot_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(my_robot_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(my_robot_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(my_robot_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_robot_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(my_robot_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(my_robot_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(my_robot_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_robot_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(my_robot_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(my_robot_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(my_robot_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
