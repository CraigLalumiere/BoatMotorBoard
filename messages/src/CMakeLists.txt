set(PC_COM_PATH                    ${CMAKE_CURRENT_SOURCE_DIR}/../../pc_com)

# list all desired packet proto files here
set(PROTO_FILES
        MessageType.proto
        LogPrint.proto
        CLIData.proto
        ConfigDB.proto
)


# odd that they call this cpp, but it generates C files
nanopb_generate_cpp(TARGET generated-message-c-files ${PROTO_FILES})

# copy all the build generated headers into the repo to enable
# committing these files when finalized.
add_custom_command(TARGET generated-message-c-files POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                           ${CMAKE_CURRENT_BINARY_DIR}/*.h
                           ${CMAKE_CURRENT_SOURCE_DIR}/../generated/c/
)
add_custom_command(TARGET generated-message-c-files POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                           ${CMAKE_CURRENT_BINARY_DIR}/*.c
                           ${CMAKE_CURRENT_SOURCE_DIR}/../generated/c/
)

set(PROTO_PYTHON_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../generated/python)
protobuf_generate_python(
        GENERATED_PYTHON_FILES
        ${PROTO_FILES}
)

add_custom_target(generated-message-python-files ALL DEPENDS ${GENERATED_PYTHON_FILES})

# copy all the build generated python files into the repo to enable
# committing these files when finalized.
add_custom_command(TARGET generated-message-python-files POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/*.py
        ${PC_COM_PATH}/messages/
)
# 'messages' target.
# configure cmake and build for this target to generate
# C nanopb and python output.
add_custom_target(messages)
add_dependencies(messages
        generated-message-c-files
        generated-message-python-files
)
