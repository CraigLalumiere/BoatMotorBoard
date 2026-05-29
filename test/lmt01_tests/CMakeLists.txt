set(TEST_APP_NAME lmt01-tests)

include_directories(${TEST_SUPPORT_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR}/bsp)
include_directories(${SHARED_SRC_TOP_DIR}/services)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../motor/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../motor/src/services)

set(TEST_SOURCES
    lmt01_tests.cpp
    ${SHARED_SRC_TOP_DIR}/services/fault_manager.c
    ${SHARED_SRC_TOP_DIR}/services/safe_strncpy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motor/src/services/LMT01.c
)

include(${CMS_CMAKE_DIR}/cpputestCMake.cmake)

target_link_libraries(${TEST_APP_NAME} cpputest-for-qpc-lib ${CPPUTEST_LDFLAGS})
