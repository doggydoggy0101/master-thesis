cmake_minimum_required(VERSION 3.11)
project(registration_example VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)


add_subdirectory(${CMAKE_SOURCE_DIR}/../.. ${CMAKE_BINARY_DIR}/registration)

add_executable(registration_example registration_example.cpp)
target_link_libraries(registration_example PRIVATE Registration::registration)
