#pragma once

extern "C"
{
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"

    void init_lua();
    void close_lua();
    void run_lua_script(const char* script);
}
