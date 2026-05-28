cmake_minimum_required(VERSION 3.22)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(PORTAGE-MESSAGES)

find_package(Protobuf REQUIRED)
include(fetch_nanopb.cmake)
find_package(Nanopb REQUIRED)

add_subdirectory(src)
