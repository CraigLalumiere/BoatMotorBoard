set(TEST_APP_NAME gauge-director-tests)

include_directories(${TEST_SUPPORT_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR}/bsp)
include_directories(${SHARED_SRC_TOP_DIR}/services)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../gauge/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../gauge/src/services)

set(TEST_SOURCES
    gauge_director_tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../gauge/src/services/director.c
)

include(${CMS_CMAKE_DIR}/cpputestCMake.cmake)

target_link_libraries(${TEST_APP_NAME} cpputest-for-qpc-lib ${CPPUTEST_LDFLAGS})
