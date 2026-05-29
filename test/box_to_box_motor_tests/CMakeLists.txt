set(TEST_APP_NAME box-to-box-motor-tests)

include_directories(${TEST_SUPPORT_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR})
include_directories(${SHARED_SRC_TOP_DIR}/bsp)
include_directories(${SHARED_SRC_TOP_DIR}/services)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../motor/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../motor/src/bsp)

set(TEST_SOURCES
    box_to_box_motor_tests.cpp
    ${TEST_SUPPORT_TOP_DIR}/bsp_box_to_box_mock.cpp
    ${SHARED_SRC_TOP_DIR}/services/box_to_box.c
)

include(${CMS_CMAKE_DIR}/cpputestCMake.cmake)

target_compile_definitions(${TEST_APP_NAME} PRIVATE BOARD_MOTOR)
target_link_libraries(${TEST_APP_NAME} cpputest-for-qpc-lib ${CPPUTEST_LDFLAGS})
