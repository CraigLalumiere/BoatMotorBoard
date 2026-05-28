include(FetchContent)

# NOTE: this should be synchronized with the firmware's version to guarantee
# compatibility
FetchContent_Declare(
        nanopb
        GIT_REPOSITORY https://github.com/nanopb/nanopb.git
        GIT_TAG        99902a343c9074e5f0b0419a79878b6ed08b5363 # latest, July 3, 2024
)

FetchContent_MakeAvailable(nanopb)
FetchContent_GetProperties(nanopb)
set(CMAKE_MODULE_PATH ${nanopb_SOURCE_DIR}/extra)

set(nanopb_SRCS
        ${nanopb_SOURCE_DIR}/pb_common.c
        ${nanopb_SOURCE_DIR}/pb_decode.c
        ${nanopb_SOURCE_DIR}/pb_encode.c
)