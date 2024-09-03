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

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) gen_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#include <casadi/mem.h>

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_log1p CASADI_PREFIX(log1p)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_log1p(casadi_real x) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return log(1+x);
#else
  return log1p(x);
#endif
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s1[9] = {2, 2, 0, 2, 4, 0, 1, 0, 1};
static const casadi_int casadi_s2[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};

/* f:(i0,i1[2x2])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2;
    a0=arg[0]? arg[0][0] : 0;
    a0=casadi_log1p(a0);
    a1=arg[1]? arg[1][0] : 0;
    a2=arg[1]? arg[1][3] : 0;
    a1=(a1+a2);
    a0=(a0+a1);
    if (res[0]!=0) res[0][0]=a0;
    return 0;
}

CASADI_SYMBOL_EXPORT int f(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
    return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int f_alloc_mem(void) {
    return 0;
}

CASADI_SYMBOL_EXPORT int f_init_mem(int mem) {
    return 0;
}

CASADI_SYMBOL_EXPORT void f_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int f_checkout(void) {
    return 0;
}

CASADI_SYMBOL_EXPORT void f_release(int mem) {
}

CASADI_SYMBOL_EXPORT void f_incref(void) {
}

CASADI_SYMBOL_EXPORT void f_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int f_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int f_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real f_default_in(casadi_int i) {
    switch (i) {
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const char* f_name_in(casadi_int i) {
    switch (i) {
        case 0: return "i0";
        case 1: return "i1";
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const char* f_name_out(casadi_int i) {
    switch (i) {
        case 0: return "o0";
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const casadi_int* f_sparsity_in(casadi_int i) {
    switch (i) {
        case 0: return casadi_s0;
        case 1: return casadi_s1;
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const casadi_int* f_sparsity_out(casadi_int i) {
    switch (i) {
        case 0: return casadi_s0;
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT int f_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
    if (sz_arg) *sz_arg = 2;
    if (sz_res) *sz_res = 1;
    if (sz_iw) *sz_iw = 0;
    if (sz_w) *sz_w = 0;
    return 0;
}

CASADI_SYMBOL_EXPORT int f_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
    if (sz_arg) *sz_arg = 2*sizeof(const casadi_real*);
    if (sz_res) *sz_res = 1*sizeof(casadi_real*);
    if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
    if (sz_w) *sz_w = 0*sizeof(casadi_real);
    return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* f_functions(void) {
    static casadi_functions fun = {
        f_incref,
        f_decref,
        f_checkout,
        f_release,
        f_default_in,
        f_n_in,
        f_n_out,
        f_name_in,
        f_name_out,
        f_sparsity_in,
        f_sparsity_out,
        f_work,
        f
    };
    return &fun;
}
/* discrete_dynamics:(i0[4],i1[2])->(o0[4]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a4, a5, a6, a7, a8, a9;
    a0=arg[0]? arg[0][0] : 0;
    a1=8.3333333333333332e-03;
    a2=arg[0]? arg[0][3] : 0;
    a3=arg[0]? arg[0][2] : 0;
    a4=5.0000000000000000e-01;
    a5=arg[1]? arg[1][1] : 0;
    a6=(a4*a5);
    a7=(a3+a6);
    a7=cos(a7);
    a7=(a2*a7);
    a8=2.;
    a9=2.5000000000000001e-02;
    a10=4.9500000000000002e+00;
    a11=arg[1]? arg[1][0] : 0;
    a12=(a10*a11);
    a13=2.9702999999999997e+02;
    a14=1.6664999999999999e+01;
    a15=cos(a6);
    a15=(a2*a15);
    a16=(a14*a15);
    a16=(a13+a16);
    a17=6.7840000000000000e-01;
    a18=casadi_sq(a15);
    a18=(a17*a18);
    a16=(a16+a18);
    a18=10.;
    a15=(a18*a15);
    a15=tanh(a15);
    a16=(a16*a15);
    a12=(a12-a16);
    a16=230.;
    a12=(a12/a16);
    a15=(a9*a12);
    a15=(a2+a15);
    a19=sin(a6);
    a19=(a2*a19);
    a20=7.8530000000000000e-01;
    a19=(a19/a20);
    a21=(a9*a19);
    a21=(a3+a21);
    a22=(a4*a5);
    a23=(a21+a22);
    a23=cos(a23);
    a23=(a15*a23);
    a23=(a8*a23);
    a7=(a7+a23);
    a23=(a10*a11);
    a24=cos(a22);
    a24=(a15*a24);
    a25=(a14*a24);
    a25=(a13+a25);
    a26=casadi_sq(a24);
    a26=(a17*a26);
    a25=(a25+a26);
    a24=(a18*a24);
    a24=tanh(a24);
    a25=(a25*a24);
    a23=(a23-a25);
    a23=(a23/a16);
    a25=(a9*a23);
    a25=(a2+a25);
    a24=sin(a22);
    a24=(a15*a24);
    a24=(a24/a20);
    a9=(a9*a24);
    a9=(a3+a9);
    a26=(a4*a5);
    a27=(a9+a26);
    a27=cos(a27);
    a27=(a25*a27);
    a27=(a8*a27);
    a7=(a7+a27);
    a27=5.0000000000000003e-02;
    a28=(a10*a11);
    a29=cos(a26);
    a29=(a25*a29);
    a30=(a14*a29);
    a30=(a13+a30);
    a31=casadi_sq(a29);
    a31=(a17*a31);
    a30=(a30+a31);
    a29=(a18*a29);
    a29=tanh(a29);
    a30=(a30*a29);
    a28=(a28-a30);
    a28=(a28/a16);
    a30=(a27*a28);
    a30=(a2+a30);
    a29=sin(a26);
    a29=(a25*a29);
    a29=(a29/a20);
    a27=(a27*a29);
    a27=(a3+a27);
    a4=(a4*a5);
    a5=(a27+a4);
    a5=cos(a5);
    a5=(a30*a5);
    a7=(a7+a5);
    a7=(a1*a7);
    a0=(a0+a7);
    if (res[0]!=0) res[0][0]=a0;
    a0=arg[0]? arg[0][1] : 0;
    a6=(a3+a6);
    a6=sin(a6);
    a6=(a2*a6);
    a21=(a21+a22);
    a21=sin(a21);
    a15=(a15*a21);
    a15=(a8*a15);
    a6=(a6+a15);
    a9=(a9+a26);
    a9=sin(a9);
    a25=(a25*a9);
    a25=(a8*a25);
    a6=(a6+a25);
    a27=(a27+a4);
    a27=sin(a27);
    a27=(a30*a27);
    a6=(a6+a27);
    a6=(a1*a6);
    a0=(a0+a6);
    if (res[0]!=0) res[0][1]=a0;
    a24=(a8*a24);
    a19=(a19+a24);
    a29=(a8*a29);
    a19=(a19+a29);
    a29=sin(a4);
    a29=(a30*a29);
    a29=(a29/a20);
    a19=(a19+a29);
    a19=(a1*a19);
    a3=(a3+a19);
    if (res[0]!=0) res[0][2]=a3;
    a23=(a8*a23);
    a12=(a12+a23);
    a8=(a8*a28);
    a12=(a12+a8);
    a10=(a10*a11);
    a4=cos(a4);
    a30=(a30*a4);
    a14=(a14*a30);
    a13=(a13+a14);
    a14=casadi_sq(a30);
    a17=(a17*a14);
    a13=(a13+a17);
    a18=(a18*a30);
    a18=tanh(a18);
    a13=(a13*a18);
    a10=(a10-a13);
    a10=(a10/a16);
    a12=(a12+a10);
    a1=(a1*a12);
    a2=(a2+a1);
    if (res[0]!=0) res[0][3]=a2;
    return 0;
}

CASADI_SYMBOL_EXPORT int discrete_dynamics(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
    return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int discrete_dynamics_alloc_mem(void) {
    return 0;
}

CASADI_SYMBOL_EXPORT int discrete_dynamics_init_mem(int mem) {
    return 0;
}

CASADI_SYMBOL_EXPORT void discrete_dynamics_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int discrete_dynamics_checkout(void) {
    return 0;
}

CASADI_SYMBOL_EXPORT void discrete_dynamics_release(int mem) {
}

CASADI_SYMBOL_EXPORT void discrete_dynamics_incref(void) {
}

CASADI_SYMBOL_EXPORT void discrete_dynamics_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int discrete_dynamics_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int discrete_dynamics_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real discrete_dynamics_default_in(casadi_int i) {
    switch (i) {
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const char* discrete_dynamics_name_in(casadi_int i) {
    switch (i) {
        case 0: return "i0";
        case 1: return "i1";
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const char* discrete_dynamics_name_out(casadi_int i) {
    switch (i) {
        case 0: return "o0";
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const casadi_int* discrete_dynamics_sparsity_in(casadi_int i) {
    switch (i) {
        case 0: return casadi_s2;
        case 1: return casadi_s3;
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT const casadi_int* discrete_dynamics_sparsity_out(casadi_int i) {
    switch (i) {
        case 0: return casadi_s2;
        default: return 0;
    }
}

CASADI_SYMBOL_EXPORT int discrete_dynamics_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
    if (sz_arg) *sz_arg = 2;
    if (sz_res) *sz_res = 1;
    if (sz_iw) *sz_iw = 0;
    if (sz_w) *sz_w = 0;
    return 0;
}

CASADI_SYMBOL_EXPORT int discrete_dynamics_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
    if (sz_arg) *sz_arg = 2*sizeof(const casadi_real*);
    if (sz_res) *sz_res = 1*sizeof(casadi_real*);
    if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
    if (sz_w) *sz_w = 0*sizeof(casadi_real);
    return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* discrete_dynamics_functions(void) {
    static casadi_functions fun = {
        discrete_dynamics_incref,
        discrete_dynamics_decref,
        discrete_dynamics_checkout,
        discrete_dynamics_release,
        discrete_dynamics_default_in,
        discrete_dynamics_n_in,
        discrete_dynamics_n_out,
        discrete_dynamics_name_in,
        discrete_dynamics_name_out,
        discrete_dynamics_sparsity_in,
        discrete_dynamics_sparsity_out,
        discrete_dynamics_work,
        discrete_dynamics
    };
    return &fun;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
