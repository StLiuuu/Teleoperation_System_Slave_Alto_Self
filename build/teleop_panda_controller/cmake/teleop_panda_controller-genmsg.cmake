# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "teleop_panda_controller: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iteleop_panda_controller:/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(teleop_panda_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg" NAME_WE)
add_custom_target(_teleop_panda_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teleop_panda_controller" "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(teleop_panda_controller
  "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_panda_controller
)

### Generating Services

### Generating Module File
_generate_module_cpp(teleop_panda_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_panda_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(teleop_panda_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(teleop_panda_controller_generate_messages teleop_panda_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(teleop_panda_controller_generate_messages_cpp _teleop_panda_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_panda_controller_gencpp)
add_dependencies(teleop_panda_controller_gencpp teleop_panda_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_panda_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(teleop_panda_controller
  "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_panda_controller
)

### Generating Services

### Generating Module File
_generate_module_eus(teleop_panda_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_panda_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(teleop_panda_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(teleop_panda_controller_generate_messages teleop_panda_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(teleop_panda_controller_generate_messages_eus _teleop_panda_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_panda_controller_geneus)
add_dependencies(teleop_panda_controller_geneus teleop_panda_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_panda_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(teleop_panda_controller
  "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_panda_controller
)

### Generating Services

### Generating Module File
_generate_module_lisp(teleop_panda_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_panda_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(teleop_panda_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(teleop_panda_controller_generate_messages teleop_panda_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(teleop_panda_controller_generate_messages_lisp _teleop_panda_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_panda_controller_genlisp)
add_dependencies(teleop_panda_controller_genlisp teleop_panda_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_panda_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(teleop_panda_controller
  "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_panda_controller
)

### Generating Services

### Generating Module File
_generate_module_nodejs(teleop_panda_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_panda_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(teleop_panda_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(teleop_panda_controller_generate_messages teleop_panda_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(teleop_panda_controller_generate_messages_nodejs _teleop_panda_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_panda_controller_gennodejs)
add_dependencies(teleop_panda_controller_gennodejs teleop_panda_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_panda_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(teleop_panda_controller
  "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_panda_controller
)

### Generating Services

### Generating Module File
_generate_module_py(teleop_panda_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_panda_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(teleop_panda_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(teleop_panda_controller_generate_messages teleop_panda_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/panda/Sitong/Teleoperation_System_Slave_Alto/src/teleop_panda_controller/msg/JointTorqueComparison.msg" NAME_WE)
add_dependencies(teleop_panda_controller_generate_messages_py _teleop_panda_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_panda_controller_genpy)
add_dependencies(teleop_panda_controller_genpy teleop_panda_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_panda_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_panda_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_panda_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_panda_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_panda_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_panda_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_panda_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_panda_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_panda_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_panda_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_panda_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_panda_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
