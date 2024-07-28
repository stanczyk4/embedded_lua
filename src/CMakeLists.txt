cmake_minimum_required(VERSION 3.26)

add_executable(${PROJECT_NAME} temp.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC .)
