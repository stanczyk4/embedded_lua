#include "lua.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

// clang-format off
static const luaL_Reg loadedlibs[] = {
    {LUA_GNAME, luaopen_base},
    // {LUA_LOADLIBNAME, luaopen_package},
    // {LUA_COLIBNAME, luaopen_coroutine},
    {LUA_TABLIBNAME, luaopen_table},
    // {LUA_IOLIBNAME, luaopen_io},
    // {LUA_OSLIBNAME, luaopen_os},
    {LUA_STRLIBNAME, luaopen_string},
    {LUA_MATHLIBNAME, luaopen_math},
    {LUA_UTF8LIBNAME, luaopen_utf8},
    // {LUA_DBLIBNAME, luaopen_debug},
    {NULL, NULL}
};
// clang-format on

extern "C"
{
    static void* custom_malloc(void* ud, void* ptr, size_t osize, size_t nsize)
    {
        (void)ud;
        (void)osize; /* not used */
        if (nsize == 0)
        {
            free(ptr);
            return NULL;
        }
        else
            return realloc(ptr, nsize);
    }
}

void open_libs(lua_State* L)
{
    const luaL_Reg* lib;
    /* "require" functions from 'loadedlibs' and set results to global table */
    for (lib = loadedlibs; lib->func; lib++)
    {
        luaL_requiref(L, lib->name, lib->func, 1);
        lua_pop(L, 1); /* remove lib */
    }
}

lua_State* lua_state = nullptr;

void init_lua()
{
    lua_state = lua_newstate(custom_malloc, NULL);
    open_libs(lua_state);
}

void close_lua()
{
    lua_close(lua_state);
}

void run_lua_script(const char* script)
{
    // Load and run the Lua script
    if (luaL_dostring(lua_state, script) != LUA_OK)
    {
        fprintf(stderr, "Error: %s\n", lua_tostring(lua_state, -1));
        lua_pop(lua_state, 1);  // Remove error message from the stack
    }
}
