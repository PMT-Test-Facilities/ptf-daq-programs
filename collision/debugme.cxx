#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <chrono>

#include "pathgen.hpp"
#include "serialization.hpp"

namespace PG = PathGeneration;
namespace SD = Serialization;

#include "geom.hpp"


void sighandler(int sig) {
  void* stack[20];
  size_t size;

  size = backtrace(stack, 20);

  cerr << "\n\nReceived signal " << C_BOLD << sig << C_RESET << ". Stack trace:\n" << std::flush;
  backtrace_symbols_fd(stack, size, STDERR_FILENO);
  exit(1);
}


int main(void) {
  signal(SIGSEGV, sighandler);

  // fixed_vec<int, 12> v;

  // v.push_checked(5);
  // auto a = v.get_checked(0);
  // if (a) {
  //   cout << "a: " << *a << "\n";
  // } else {
  //   cout << "a: (none)\n";
  // }

  // v.pop_checked();

  // cout << "Pop twice: " << v.pop_checked();

  // cout << std::flush;

  // auto start_time = std::chrono::steady_clock::now();

  // const static auto N_ITER = 10000;

  // for (size_t __i = 0; __i < N_ITER; __i++) {

  //   const PG::MovePoint from = {
  //     {{0.05,0.05,0.05},{0,0}},
  //     {{0.35,0.35,0.35},{0,0}},
  //   },
  //   to = {
  //     {{0.05,0.05,0.05},{PI/4,PI/4}},
  //     {{0.35,0.35,0.35},{0,0}},
  //   };

  //   vector<Intersectable> geom = {
  //     (Sphere){Vec3(0.5,0.5,0.5),0.1},
  //     (Sphere){Vec3(0.55,0.55,0.55),0.1},
  //     Prism(Vec3(-0.3, -0.3, -0.3), 0.05, 0.05, 0.05, Quaternion::identity()),
  //   };

  //   auto p = PG::single_move(from, to, geom);
  //   auto is_err = has<PG::ErrorType>(p);
  // }

  // auto end_time = std::chrono::steady_clock::now();

  // std::cout << (end_time - start_time).count() / 1e9 / N_ITER << std::endl;

  // if (is_err) {
  //   cerr << C_BR_RED << "Pathgen error: " << PG::error_message(get<PG::ErrorType>(p)) << C_RED << "\n" << std::flush;
  // } else {
  //   cerr << "Found path." << std::flush;
  // }
  return 0;
}