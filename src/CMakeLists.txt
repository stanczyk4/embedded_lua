cmake_minimum_required(VERSION 3.26)

project(embedded_lua)

add_library(${PROJECT_NAME} lua.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC .)
target_link_libraries(${PROJECT_NAME} lua)
