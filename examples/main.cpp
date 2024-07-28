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

const char *lua_script = "function calculateTrig(a, b, x, c, d)\n"
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

extern size_t largest_allocation;
extern size_t current_usage;
extern size_t peak_usage;

extern "C" {

static void *custom_malloc(void *ud, void *ptr, size_t osize, size_t nsize) {
  (void)ud;
  (void)osize; /* not used */
  if (nsize == 0) {
    free(ptr);
    return NULL;
  } else
    return realloc(ptr, nsize);
}
}

int main() {
  // Create a new Lua state
  lua_State *L = lua_newstate(custom_malloc, NULL);
  if (L == NULL) {
    // Handle memory allocation error
  }

  // Open the standard libraries
  luaL_openlibs(L);

  // Load and run the Lua script
  if (luaL_dostring(L, lua_script) != LUA_OK) {
    fprintf(stderr, "Error: %s\n", lua_tostring(L, -1));
    lua_pop(L, 1); // Remove error message from the stack
  }

  // Close the Lua state
  lua_close(L);

  // Print memory usage statistics
  printf("Largest allocation: %lu\n", largest_allocation);
  printf("Current usage: %lu\n", current_usage);

  return 0;
}
