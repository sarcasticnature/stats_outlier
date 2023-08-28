cmake_minimum_required(VERSION 3.14)
project(stats_outlier)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

FetchContent_Declare(matplotplusplus
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
    GIT_TAG origin/master
)

FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

find_package(Eigen3 REQUIRED)

add_executable(mad src/mad.cpp)
target_link_libraries(mad PUBLIC matplot)

add_executable(ransac src/ransac.cpp)
target_link_libraries(ransac PUBLIC matplot Eigen3::Eigen)
