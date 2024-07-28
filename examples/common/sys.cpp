#if defined(__GNUC__) && !defined(__linux__)
#include <sys/time.h>

#include <cassert>
#include <cstdint>

#include "SEGGER_RTT.h"
// #include "cmsis_compiler.h"
#include "hal/device/device_info.h"

#ifdef __cplusplus
extern "C" {
#endif

int _write(int fd, const void *buffer, unsigned int count) {
  if (fd == 1) {
    SEGGER_RTT_Write(0, buffer, count);
  } else {
    assert(0); // only handle cases we know to handle
  }

  return count;
}

void __assert(const char *, int, const char *) {
  // GetHwLayer().GetOutputDebugLedRed().SetState(hal::gpio::State::kLow);  //
  // low turns LED on
  SEGGER_RTT_WriteString(0, "Asserted\r\n");
  // Do a while(1) to prevent GCC from complaining about us returning
  __BKPT(0);
  while (1) {
  }
}

void __assert_func(const char *file, int line, const char *func,
                   const char *failedexpr) {
  SEGGER_RTT_WriteString(0, "Asserted\r\n");
  // Do a while(1) to prevent GCC from complaining about us returning
  __BKPT(0);
  while (1) {
  }
}

void _exit(int status) {
  __BKPT(0);
  while (1) {
  }
}

int _kill(int pid, int sig) { return -1; }

struct _stat {};

int _isatty(int fd) { return 0; }

int _gettimeofday(struct timeval *__tp, void *__tzp) {
  static uint64_t roll = 0;
  static uint64_t last = 0;

  const uint32_t current = HAL_GetTick();
  if (current < last) {
    roll += 0xFFFFFFFF;
  }
  last = current;

  // TODO be more efficient
  const uint64_t total = roll + last;
  __tp->tv_sec = total / 1000; // do integer divison, floor the remainder
                               // milliseconds off calculation
  // __tp->tv_usec = (total % 1000) * 1000;
  __tp->tv_usec = (total - (__tp->tv_sec * 1000)) * 1000;
  return 0;
}

int _system(const char *) { return -1; }
int _rename(const char *, const char *) { return -1; }
clock_t _times(struct tms *) { return (clock_t)(-1); }
void _raise(void) { return; }
int _unlink(const char *) { return -1; }
int _link(const char *, const char *) { return -1; }
int _stat(const char *, struct stat *) { return -1; }
int _fstat(int, struct stat *) { return -1; }
pid_t _getpid(void) { return -1; }
int _close(int) { return -1; }
int _swiclose(int) { return -1; }
int _open(const char *, int, ...) { return -1; }
int _swiopen(const char *, int) { return -1; }
int _swiwrite(int, const void *, size_t) { return -1; }
off_t _lseek(int, off_t, int) { return (off_t)(-1); }
off_t _swilseek(int, off_t, int) { return (off_t)(-1); }
int _read(int, void *, size_t) { return -1; }
int _swiread(int, void *, size_t) { return -1; }

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

size_t largest_allocation = 0;
size_t current_usage = 0;
size_t peak_usage = 0;

#define ALIGNMENT sizeof(size_t)
#define ALIGN_UP(size) (((size) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1))

void *__real_malloc(size_t size);
void __real_free(void *ptr);
void *__real_realloc(void *ptr, size_t size);

void *__wrap_realloc(void *ptr, size_t size) {
  void *original_ptr = __real_realloc(ptr, size);

  if (largest_allocation < size) {
    largest_allocation = size;
  }

  current_usage += size;

  return original_ptr;
}

void *__wrap_malloc(size_t size) {
  void *original_ptr = __real_malloc(size);

  if (largest_allocation < size) {
    largest_allocation = size;
  }

  current_usage += size;

  return original_ptr;
}

void __wrap_free(void *ptr) {
  // Free the original pointer
  __real_free(ptr);
}

#ifdef __cplusplus
}
#endif
#endif // #ifdef __GNUC__
