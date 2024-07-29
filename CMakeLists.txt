cmake_minimum_required(VERSION 3.26)

include(cmake/add_cpm.cmake)
include(cmake/add_lua.cmake)

add_subdirectory(src)

if (EMBEDDED_LUA_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()
