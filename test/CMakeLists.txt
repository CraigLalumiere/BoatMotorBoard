set(TEST_SUPPORT_TOP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/support)
set(TEST_MOCKS_TOP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks)

add_subdirectory(protocol_unit_tests)
add_subdirectory(pc_com_packet_tests)
add_subdirectory(box_to_box_motor_tests)
add_subdirectory(box_to_box_gauge_tests)
