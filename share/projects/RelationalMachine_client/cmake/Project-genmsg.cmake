# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "Project: 7 messages, 0 services")

set(MSG_I_FLAGS "-IProject:/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/groovy/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(Project_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionGoal.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionResult.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionFeedback.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
)
_generate_msg_cpp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
)
_generate_msg_cpp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
)
_generate_msg_cpp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
)
_generate_msg_cpp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
)
_generate_msg_cpp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
)
_generate_msg_cpp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
)

### Generating Services

### Generating Module File
_generate_module_cpp(Project
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(Project_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(Project_generate_messages Project_generate_messages_cpp)

# target for backward compatibility
add_custom_target(Project_gencpp)
add_dependencies(Project_gencpp Project_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Project_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionGoal.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionResult.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionFeedback.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
)
_generate_msg_lisp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
)
_generate_msg_lisp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
)
_generate_msg_lisp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
)
_generate_msg_lisp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
)
_generate_msg_lisp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
)
_generate_msg_lisp(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
)

### Generating Services

### Generating Module File
_generate_module_lisp(Project
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(Project_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(Project_generate_messages Project_generate_messages_lisp)

# target for backward compatibility
add_custom_target(Project_genlisp)
add_dependencies(Project_genlisp Project_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Project_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionGoal.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionResult.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionFeedback.msg;/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
)
_generate_msg_py(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
)
_generate_msg_py(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
)
_generate_msg_py(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
)
_generate_msg_py(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
)
_generate_msg_py(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandFeedback.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/groovy/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/groovy/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
)
_generate_msg_py(Project
  "/home/bais/git/mlr/share/projects/RelationalMachine/devel/share/Project/msg/ActionCommandResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
)

### Generating Services

### Generating Module File
_generate_module_py(Project
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(Project_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(Project_generate_messages Project_generate_messages_py)

# target for backward compatibility
add_custom_target(Project_genpy)
add_dependencies(Project_genpy Project_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Project_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Project
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(Project_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(Project_generate_messages_cpp actionlib_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Project
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(Project_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(Project_generate_messages_lisp actionlib_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Project
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(Project_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(Project_generate_messages_py actionlib_msgs_generate_messages_py)
