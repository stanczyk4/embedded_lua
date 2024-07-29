cmake_minimum_required(VERSION 3.26)

CPMAddPackage (
    NAME Lua
    GIT_REPOSITORY "https://github.com/lua/lua"
    GIT_TAG v5.4.7
    EXCLUDE_FROM_ALL TRUE
    DOWNLOAD_ONLY TRUE
)

set(LUA_SOURCE_DIR ${CPM_PACKAGE_Lua_SOURCE_DIR})

add_library(lua STATIC
    "${LUA_SOURCE_DIR}/lapi.c"
    "${LUA_SOURCE_DIR}/lcode.c"
    "${LUA_SOURCE_DIR}/lctype.c"
    "${LUA_SOURCE_DIR}/ldebug.c"
    "${LUA_SOURCE_DIR}/ldo.c"
    "${LUA_SOURCE_DIR}/ldump.c"
    "${LUA_SOURCE_DIR}/lfunc.c"
    "${LUA_SOURCE_DIR}/lgc.c"
    "${LUA_SOURCE_DIR}/llex.c"
    "${LUA_SOURCE_DIR}/lmem.c"
    "${LUA_SOURCE_DIR}/lobject.c"
    "${LUA_SOURCE_DIR}/lopcodes.c"
    "${LUA_SOURCE_DIR}/lparser.c"
    "${LUA_SOURCE_DIR}/lstate.c"
    "${LUA_SOURCE_DIR}/lstring.c"
    "${LUA_SOURCE_DIR}/ltable.c"
    "${LUA_SOURCE_DIR}/ltm.c"
    "${LUA_SOURCE_DIR}/lundump.c"
    "${LUA_SOURCE_DIR}/lvm.c"
    "${LUA_SOURCE_DIR}/lzio.c"
    "${LUA_SOURCE_DIR}/lauxlib.c"
    "${LUA_SOURCE_DIR}/lbaselib.c"
    "${LUA_SOURCE_DIR}/lcorolib.c"
    "${LUA_SOURCE_DIR}/ldblib.c"
    "${LUA_SOURCE_DIR}/liolib.c"
    "${LUA_SOURCE_DIR}/lmathlib.c"
    "${LUA_SOURCE_DIR}/loadlib.c"
    "${LUA_SOURCE_DIR}/loslib.c"
    "${LUA_SOURCE_DIR}/lstrlib.c"
    "${LUA_SOURCE_DIR}/ltablib.c"
    "${LUA_SOURCE_DIR}/lutf8lib.c"
    "${LUA_SOURCE_DIR}/linit.c"
)

target_include_directories(lua PUBLIC ${LUA_SOURCE_DIR})

if(UNIX)
    find_library(LIB_MATH NAMES m)
    if(LIB_MATH)
      target_link_libraries(lua PUBLIC ${LIB_MATH})
    endif()
    mark_as_advanced(LIB_MATH)
endif()

target_compile_options(lua PRIVATE "-Os")

# Specify the file to be removed
set(LUA_CONF_FILE "${LUA_SOURCE_DIR}/luaconf.h")

# Remove the file after the download
add_custom_target(remove_lua_conf ALL
    COMMAND ${CMAKE_COMMAND} -E remove ${LUA_CONF_FILE}
    COMMENT "Removing unnecessary file from Lua package"
)

add_dependencies(lua remove_lua_conf)
