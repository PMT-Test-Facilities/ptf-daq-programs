#include "debug.hpp"

int64_t __stack_level = -1;


void __debug_incr_stack() {
  __stack_level++;
}


void __debug_decr_stack() {
  __stack_level--;
}


int64_t __debug_stack() {
  return __stack_level;
}
