cmake_minimum_required(VERSION 3.10)
project(energy_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB_RECURSE SRC_FILES "src/*.cpp")
add_executable(energy_demo ${SRC_FILES})

target_link_libraries(energy_demo ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})
