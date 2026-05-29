set(TEST_APP_NAME pc-com-packet-tests)

include_directories(${TEST_SUPPORT_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR}/bsp)
include_directories(${SHARED_SRC_TOP_DIR}/services)
include_directories(${SHARED_SRC_TOP_DIR}/services/pc_com)
include_directories(${ROOT_PATH}/messages/generated)
include_directories(${ROOT_PATH}/messages/generated/c)
include_directories(${nanopb_SOURCE_DIR})
include_directories(${LIBRARY_TOP_DIR}/embedded-cli)

set(TEST_SOURCES
    pc_com_packet_tests.cpp
    ${TEST_SUPPORT_TOP_DIR}/pc_com_test_mocks.cpp
    ${SHARED_SRC_TOP_DIR}/services/pc_com/pc_com.c
    ${SHARED_SRC_TOP_DIR}/services/pc_com/crc16.c
    ${SHARED_SRC_TOP_DIR}/services/pc_com/hdlc.c
    ${SHARED_SRC_TOP_DIR}/services/safe_strncpy.c
    ${ROOT_PATH}/messages/generated/c/CLIData.pb.c
    ${ROOT_PATH}/messages/generated/c/ConfigDB.pb.c
    ${ROOT_PATH}/messages/generated/c/LogPrint.pb.c
    ${ROOT_PATH}/messages/generated/c/MessageType.pb.c
    ${ROOT_PATH}/messages/generated/c/MotorData.pb.c
    ${nanopb_SRCS}
)

include(${CMS_CMAKE_DIR}/cpputestCMake.cmake)

target_compile_definitions(${TEST_APP_NAME} PRIVATE CLI_BUFFER_SIZE=2048)
target_link_libraries(${TEST_APP_NAME} cpputest-for-qpc-lib ${CPPUTEST_LDFLAGS})
