#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "lua.hpp"

// const char *lua_script = "function calculateY(m, x, b)\n"
//                          "    return m * x + b\n"
//                          "end\n"
//                          "\n"
//                          "local m = 2\n"
//                          "local x = 5\n"
//                          "local b = 3\n"
//                          "\n"
//                          "local y = calculateY(m, x, b)\n"
//                          "\n"
//                          "print(\"The value of y is: \" .. y)\n";

const char* lua_script =
    "function calculateTrig(a, b, x, c, d)\n"
    "    return a * math.sin(b * x + c) + d\n"
    "end\n"
    "\n"
    "local a = 2\n"
    "local b = 3\n"
    "local x = 1\n"
    "local c = 0.5\n"
    "local d = 4\n"
    "\n"
    "local y = calculateTrig(a, b, x, c, d)\n"
    "\n"
    "print(\"The value of y is: \" .. y)\n";

int main()
{
    init_lua();
    run_lua_script(lua_script);
    close_lua();

    return 0;
}
