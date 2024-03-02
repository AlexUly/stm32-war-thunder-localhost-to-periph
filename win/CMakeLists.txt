cmake_minimum_required(VERSION 3.25)
project(wt_stats)

set(CMAKE_CXX_STANDARD 17)

set(BUILD_CPR_TESTS OFF)
set(CMAKE_USE_OPENSSL OFF)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(JSON_BuildTests OFF CACHE INTERNAL "")

include(FetchContent)

FetchContent_Declare(cpr GIT_REPOSITORY https://github.com/libcpr/cpr.git  GIT_TAG 3b15fa82ea74739b574d705fea44959b58142eb8)
FetchContent_MakeAvailable(cpr)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

FetchContent_Declare(libusb URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)


add_executable(wt_stats SRC/main.cpp)

target_link_libraries(wt_stats PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(wt_stats PRIVATE cpr::cpr)