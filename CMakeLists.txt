cmake_minimum_required(VERSION 3.22)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_HOST_UNIT_TESTS)
    project(BoatMotorBoardHostTests)
    enable_language(C CXX)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    set(SHARED_SRC_TOP_DIR ${ROOT_PATH}/shared)
    set(LIBRARY_TOP_DIR ${ROOT_PATH}/library)
    set(CMS_QPC_TOP_DIR ${LIBRARY_TOP_DIR}/qpc)

    include(${ROOT_PATH}/messages/fetch_nanopb.cmake)

    add_compile_options(-Wall -Wextra -Werror)

    add_subdirectory(${LIBRARY_TOP_DIR}/cpputest-for-qpc)
    add_subdirectory(test)
else()
    message(FATAL_ERROR
        "This top-level CMakeLists.txt is for host unit tests only. "
        "Configure motor/ or gauge/ for firmware builds, or set CMAKE_HOST_UNIT_TESTS=ON.")
endif()
