/* This file was automatically generated by CasADi 3.6.6.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#include <casadi/mem.h>
int f(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int f_alloc_mem(void);
int f_init_mem(int mem);
void f_free_mem(int mem);
int f_checkout(void);
void f_release(int mem);
void f_incref(void);
void f_decref(void);
casadi_int f_n_in(void);
casadi_int f_n_out(void);
casadi_real f_default_in(casadi_int i);
const char* f_name_in(casadi_int i);
const char* f_name_out(casadi_int i);
const casadi_int* f_sparsity_in(casadi_int i);
const casadi_int* f_sparsity_out(casadi_int i);
int f_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int f_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define f_SZ_ARG 2
#define f_SZ_RES 1
#define f_SZ_IW 0
#define f_SZ_W 3
casadi_functions* f_functions(void);
int discrete_dynamics(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int discrete_dynamics_alloc_mem(void);
int discrete_dynamics_init_mem(int mem);
void discrete_dynamics_free_mem(int mem);
int discrete_dynamics_checkout(void);
void discrete_dynamics_release(int mem);
void discrete_dynamics_incref(void);
void discrete_dynamics_decref(void);
casadi_int discrete_dynamics_n_in(void);
casadi_int discrete_dynamics_n_out(void);
casadi_real discrete_dynamics_default_in(casadi_int i);
const char* discrete_dynamics_name_in(casadi_int i);
const char* discrete_dynamics_name_out(casadi_int i);
const casadi_int* discrete_dynamics_sparsity_in(casadi_int i);
const casadi_int* discrete_dynamics_sparsity_out(casadi_int i);
int discrete_dynamics_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int discrete_dynamics_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define discrete_dynamics_SZ_ARG 2
#define discrete_dynamics_SZ_RES 1
#define discrete_dynamics_SZ_IW 0
#define discrete_dynamics_SZ_W 32
casadi_functions* discrete_dynamics_functions(void);
#ifdef __cplusplus
} /* extern "C" */
#endif
