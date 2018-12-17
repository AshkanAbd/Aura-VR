# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "aura_core: 0 messages, 0 services")

set(MSG_I_FLAGS "-Iaura_core:/home/ashkan/Aura_VR/src/aura_core/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genjava REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(aura_core_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;geneus;genjava;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(aura_core
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aura_core
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(aura_core_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(aura_core_generate_messages aura_core_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(aura_core_gencpp)
add_dependencies(aura_core_gencpp aura_core_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aura_core_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_eus(aura_core
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aura_core
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(aura_core_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(aura_core_generate_messages aura_core_generate_messages_eus)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(aura_core_geneus)
add_dependencies(aura_core_geneus aura_core_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aura_core_generate_messages_eus)

### Section generating for lang: genjava
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_java(aura_core
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/aura_core
  "${ALL_GEN_OUTPUT_FILES_java}"
)

add_custom_target(aura_core_generate_messages_java
  DEPENDS ${ALL_GEN_OUTPUT_FILES_java}
)
add_dependencies(aura_core_generate_messages aura_core_generate_messages_java)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(aura_core_genjava)
add_dependencies(aura_core_genjava aura_core_generate_messages_java)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aura_core_generate_messages_java)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(aura_core
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aura_core
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(aura_core_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(aura_core_generate_messages aura_core_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(aura_core_genlisp)
add_dependencies(aura_core_genlisp aura_core_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aura_core_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_nodejs(aura_core
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aura_core
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(aura_core_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(aura_core_generate_messages aura_core_generate_messages_nodejs)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(aura_core_gennodejs)
add_dependencies(aura_core_gennodejs aura_core_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aura_core_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(aura_core
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aura_core
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(aura_core_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(aura_core_generate_messages aura_core_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(aura_core_genpy)
add_dependencies(aura_core_genpy aura_core_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS aura_core_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aura_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/aura_core
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aura_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/aura_core
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genjava_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/aura_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/aura_core
    DESTINATION ${genjava_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aura_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/aura_core
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aura_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/aura_core
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aura_core)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aura_core\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/aura_core
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
