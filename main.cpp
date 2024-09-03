#include "casadi/mem.h"
#include "gen.h"
#include <stdio.h>

#include <mach/mach.h>

void print_memory_usage() {
  struct task_basic_info info;
  mach_msg_type_number_t size = TASK_BASIC_INFO_COUNT;
  kern_return_t kerr =
      task_info(mach_task_self(), TASK_BASIC_INFO, (task_info_t)&info, &size);
  if (kerr == KERN_SUCCESS) {
    printf("Memory used: %u bytes\n", (unsigned int)info.resident_size);
  }
}

int main() {
  printf("sizeof(casadi_mem) = %lu\n", sizeof(casadi_mem));
  print_memory_usage();
  casadi_mem *mem = casadi_alloc(f_functions());
  print_memory_usage();
  if (mem) {
    printf("successful casadi_alloc\n");
  }
  const double x_val = 1.0;
  const double y_val[4] = {1.0, 2.0, 3.0, 4.0};
  double res = 0.0;
  mem->arg[0] = &x_val;
  mem->arg[1] = y_val;
  mem->res[0] = &res;
  casadi_eval(mem);
  printf("res = %.10f\n", res);
  casadi_free(mem);

  mem = casadi_alloc(discrete_dynamics_functions());
  if (mem) {
    printf("successful casadi_alloc\n");
  }
  const double x[4] = {};
  const double u[2] = {};
  double x_next[4] = {};
  mem->arg[0] = x;
  mem->arg[1] = u;
  mem->res[0] = x_next;
  casadi_eval(mem);
  printf("x_next = [%.10f, %")


  return 0;
}
