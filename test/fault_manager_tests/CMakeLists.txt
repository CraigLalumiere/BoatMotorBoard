set(TEST_APP_NAME fault-manager-tests)

include_directories(${TEST_SUPPORT_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR}/services)
include_directories(${SHARED_SRC_TOP_DIR}/bsp)

set(TEST_SOURCES
    fault_manager_tests.cpp
    ${SHARED_SRC_TOP_DIR}/services/fault_manager.c
    ${SHARED_SRC_TOP_DIR}/services/safe_strncpy.c
)

include(${CMS_CMAKE_DIR}/cpputestCMake.cmake)

target_link_libraries(${TEST_APP_NAME} cpputest-for-qpc-lib ${CPPUTEST_LDFLAGS})
