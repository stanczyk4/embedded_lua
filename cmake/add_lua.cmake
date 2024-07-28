cmake_minimum_required(VERSION 3.26)

CPMAddPackage (
    NAME Lua
    GITHUB_REPOSITORY "https://github.com/lua/lua"
    GIT_TAG v5.4.7
    EXCLUDE_FROM_ALL TRUE
    DOWNLOAD_ONLY TRUE
)

add_library(lua-lib STATIC
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

target_include_directories(lua-lib PUBLIC ${LUA_SOURCE_DIR})
target_compile_definitions(lua-lib PUBLIC "-DLUA_32BITS")

if(UNIX)
    find_library(LIB_MATH NAMES m)
    if(LIB_MATH)
      target_link_libraries(lua-lib PUBLIC ${LIB_MATH})
    endif()
    mark_as_advanced(LIB_MATH)
endif()

# install
install(
  FILES ${_LUA_SOURCE_DIR}/lualib.h ${_LUA_SOURCE_DIR}/lua.h
        ${_LUA_SOURCE_DIR}/luaconf.h ${_LUA_SOURCE_DIR}/lauxlib.h
  DESTINATION include
  COMPONENT dev)

# when compiling as c++, skip the `extern "C"` wrapper
if(NOT LUA_COMPILE_AS_CPP)
  install(
    FILES ${_LUA_SOURCE_DIR}/lua.hpp
    DESTINATION include
    COMPONENT dev)
endif()

include(GNUInstallDirs)
install(
  TARGETS lua
  EXPORT Lua-targets
  COMPONENT dev
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

include(CMakePackageConfigHelpers)
set(_LuaCMakePath "${CMAKE_CURRENT_BINARY_DIR}/cmake/${_Lua}")
set(_LuaVersionFile "${_LuaCMakePath}/${_Lua}-config-version.cmake")
write_basic_package_version_file(
  ${_LuaVersionFile}
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

set(_LuaConfigFile "${_LuaCMakePath}/${_Lua}-config.cmake")
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${_Lua}-config.cmake.in ${_LuaConfigFile}
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${_Lua})

install(
  EXPORT ${_Lua}-targets
  FILE ${_Lua}-targets.cmake
  NAMESPACE ${_Lua}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${_Lua})

install(FILES ${_LuaVersionFile} ${_LuaConfigFile}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${_Lua})
