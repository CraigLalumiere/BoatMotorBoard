set(TEST_APP_NAME protocol-unit-tests)

include_directories(${SHARED_SRC_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR}/services)
include_directories(${SHARED_SRC_TOP_DIR}/services/pc_com)
include_directories(${SHARED_SRC_TOP_DIR}/bsp)
include_directories(${TEST_SUPPORT_TOP_DIR})

set(TEST_SOURCES
    protocol_tests.cpp
    ${SHARED_SRC_TOP_DIR}/services/pc_com/crc16.c
    ${SHARED_SRC_TOP_DIR}/services/pc_com/hdlc.c
    ${SHARED_SRC_TOP_DIR}/services/safe_strncpy.c
)

include(${CMS_CMAKE_DIR}/cpputestCMake.cmake)

target_link_libraries(${TEST_APP_NAME} cpputest-for-qpc-lib ${CPPUTEST_LDFLAGS})
