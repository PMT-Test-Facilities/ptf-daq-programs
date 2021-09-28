#ifndef __DEBUG_STUFF_
#define __DEBUG_STUFF_

#include <cstdint>
#include <iostream>
using std::cout;
#include "col.hpp"

#ifdef DEBUG

void __debug_incr_stack();
void __debug_decr_stack();
int64_t __debug_stack();

#ifdef DEBUG_PRINT

#define DEBUG_ENTER(x) __debug_incr_stack(); for (size_t ___it = 0; ___it < __debug_stack(); ___it++) { cout << (___it+1 == __debug_stack() ? "   \033[34m\u256d\u2574 \033[0m" : "   \033[34m\u2502\033[0m"); } { cout << C_BOLD << C_BR_CYAN << x << C_RESET << "\n"; }
#define DEBUG_LEAVE for (size_t ___it = 0; ___it < __debug_stack(); ___it++) { cout << (___it+1 == __debug_stack() ? "   \033[34m\u2570\u2574 \033[0m" : "   \033[34m\u2502\033[0m"); } cout << "\n" << std::flush; __debug_decr_stack();
#define DEBUG_COUT(x) std::cout << x << std::endl;//for (size_t ___it = 0; ___it < __debug_stack(); ___it++) { cout << "   \033[34m\u2502\033[0m"; } { cout << x << "\n"; }

#else

#define DEBUG_ENTER(x)
#define DEBUG_LEAVE
#define DEBUG_COUT(x) std::cout << x << std::endl;

#endif

#else

#define DEBUG_ENTER(x)
#define DEBUG_LEAVE
#define DEBUG_COUT(x) std::cout << x << std::endl;

#endif


#endif
