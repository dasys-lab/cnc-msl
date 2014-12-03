# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "msl_simulator: 15 messages, 0 services")

set(MSG_I_FLAGS "-Imsl_simulator:/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(msl_simulator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg" "msl_simulator/sim_robot_command"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg" ""
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg" ""
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg" "msl_simulator/messages_robocup_ssl_detection:msl_simulator/log_frame:msl_simulator/ssl_detection_robot:msl_simulator/ssl_detection_ball"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg" "msl_simulator/ssl_detection_robot:msl_simulator/ssl_detection_ball"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg" "msl_simulator/messages_robocup_ssl_geometry:msl_simulator/messages_robocup_ssl_detection:msl_simulator/ssl_geometry_camera_calibration:msl_simulator/ssl_detection_robot:msl_simulator/ssl_geometry_field_size:msl_simulator/ssl_detection_ball"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg" "msl_simulator/messages_robocup_ssl_detection:msl_simulator/ssl_detection_robot:msl_simulator/ssl_detection_ball"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg" ""
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg" ""
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg" "msl_simulator/sim_robot_command:msl_simulator/sim_commands:msl_simulator/sim_replacement:msl_simulator/sim_robot_replacement:msl_simulator/sim_ball_replacement"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg" "msl_simulator/ssl_geometry_camera_calibration:msl_simulator/ssl_geometry_field_size"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg" ""
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg" "msl_simulator/sim_robot_replacement:msl_simulator/sim_ball_replacement"
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg" ""
)

get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg" NAME_WE)
add_custom_target(_msl_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msl_simulator" "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)
_generate_msg_cpp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
)

### Generating Services

### Generating Module File
_generate_module_cpp(msl_simulator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(msl_simulator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(msl_simulator_generate_messages msl_simulator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_cpp _msl_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msl_simulator_gencpp)
add_dependencies(msl_simulator_gencpp msl_simulator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msl_simulator_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)
_generate_msg_lisp(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
)

### Generating Services

### Generating Module File
_generate_module_lisp(msl_simulator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(msl_simulator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(msl_simulator_generate_messages msl_simulator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_lisp _msl_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msl_simulator_genlisp)
add_dependencies(msl_simulator_genlisp msl_simulator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msl_simulator_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg"
  "${MSG_I_FLAGS}"
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg;/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)
_generate_msg_py(msl_simulator
  "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
)

### Generating Services

### Generating Module File
_generate_module_py(msl_simulator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(msl_simulator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(msl_simulator_generate_messages msl_simulator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_commands.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_robot.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_refbox_log.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_detection.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_wrapper.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/log_frame.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_camera_calibration.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_detection_ball.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_packet.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/messages_robocup_ssl_geometry.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_robot_command.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/sim_ball_replacement.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/snook/Development/workspace_alica/src/cnc-msl/msl_simulator/msg/ssl_geometry_field_size.msg" NAME_WE)
add_dependencies(msl_simulator_generate_messages_py _msl_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msl_simulator_genpy)
add_dependencies(msl_simulator_genpy msl_simulator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msl_simulator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msl_simulator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(msl_simulator_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msl_simulator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(msl_simulator_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msl_simulator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(msl_simulator_generate_messages_py std_msgs_generate_messages_py)
