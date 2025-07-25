/* This file was automatically generated by CasADi 3.6.3.
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
  #define CASADI_PREFIX(ID) open_phi_controller_solver_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[69] = {65, 1, 0, 65, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64};
static const casadi_int casadi_s1[4] = {0, 1, 0, 0};
static const casadi_int casadi_s2[73] = {69, 1, 0, 69, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* open_phi_controller_solver:(i0[65],i1[0],i2[69])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[2]? arg[2][68] : 0;
  a1=6.5000000000000002e-01;
  a2=arg[2]? arg[2][1] : 0;
  a3=arg[2]? arg[2][3] : 0;
  a3=(a2-a3);
  a3=(a1*a3);
  a3=casadi_sq(a3);
  a4=1.0000000000000000e-02;
  a5=arg[0]? arg[0][0] : 0;
  a6=(a4*a5);
  a6=casadi_sq(a6);
  a3=(a3+a6);
  a6=2.;
  a7=arg[2]? arg[2][2] : 0;
  a7=(a5-a7);
  a7=(a6*a7);
  a7=casadi_sq(a7);
  a3=(a3+a7);
  a7=2.3999999999999999e-01;
  a8=2.0408163265306121e-06;
  a9=-2.6152000000000001e+01;
  a10=8.3650000000000002e+00;
  a11=(a10*a2);
  a11=(a9-a11);
  a12=1.9139999999999999e+00;
  a13=casadi_sq(a2);
  a13=(a12*a13);
  a11=(a11-a13);
  a11=(a8*a11);
  a13=1000.;
  a14=(a13*a5);
  a15=490000.;
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][4] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][1] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][5] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][2] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][6] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][3] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][7] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][4] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][8] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][5] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][9] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][6] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][10] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][7] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][11] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][8] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][12] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][9] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][13] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][10] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][14] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][11] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][15] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][12] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][16] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][13] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][17] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][14] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][18] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][15] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][19] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][16] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][20] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][17] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][21] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][18] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][22] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][19] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][23] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][20] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][24] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][21] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][25] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][22] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][26] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][23] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][27] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][24] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][28] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][25] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][29] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][26] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][30] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][27] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][31] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][28] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][32] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][29] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][33] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][30] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][34] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][31] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][35] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][32] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][36] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][33] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][37] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][34] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][38] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][35] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][39] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][36] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][40] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][37] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][41] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][38] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][42] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][39] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][43] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][40] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][44] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][41] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][45] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][42] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][46] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][43] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][47] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][44] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][48] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][45] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][49] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][46] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][50] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][47] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][51] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][48] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][52] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][49] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][53] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][50] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][54] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][51] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][55] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][52] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][56] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][53] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][57] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][54] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][58] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][55] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][59] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][56] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][60] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][57] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][61] : 0;
  a11=(a2-a11);
  a11=(a1*a11);
  a11=casadi_sq(a11);
  a14=arg[0]? arg[0][58] : 0;
  a16=(a4*a14);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a3=(a3+a11);
  a5=(a14-a5);
  a5=(a6*a5);
  a5=casadi_sq(a5);
  a3=(a3+a5);
  a5=(a10*a2);
  a5=(a9-a5);
  a11=casadi_sq(a2);
  a11=(a12*a11);
  a5=(a5-a11);
  a5=(a8*a5);
  a11=(a13*a14);
  a11=(a11/a15);
  a5=(a5+a11);
  a5=(a7*a5);
  a2=(a2+a5);
  a5=arg[2]? arg[2][62] : 0;
  a5=(a2-a5);
  a5=(a1*a5);
  a5=casadi_sq(a5);
  a11=arg[0]? arg[0][59] : 0;
  a16=(a4*a11);
  a16=casadi_sq(a16);
  a5=(a5+a16);
  a3=(a3+a5);
  a14=(a11-a14);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a14=(a10*a2);
  a14=(a9-a14);
  a5=casadi_sq(a2);
  a5=(a12*a5);
  a14=(a14-a5);
  a14=(a8*a14);
  a5=(a13*a11);
  a5=(a5/a15);
  a14=(a14+a5);
  a14=(a7*a14);
  a2=(a2+a14);
  a14=arg[2]? arg[2][63] : 0;
  a14=(a2-a14);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a5=arg[0]? arg[0][60] : 0;
  a16=(a4*a5);
  a16=casadi_sq(a16);
  a14=(a14+a16);
  a3=(a3+a14);
  a11=(a5-a11);
  a11=(a6*a11);
  a11=casadi_sq(a11);
  a3=(a3+a11);
  a11=(a10*a2);
  a11=(a9-a11);
  a14=casadi_sq(a2);
  a14=(a12*a14);
  a11=(a11-a14);
  a11=(a8*a11);
  a14=(a13*a5);
  a14=(a14/a15);
  a11=(a11+a14);
  a11=(a7*a11);
  a2=(a2+a11);
  a11=arg[2]? arg[2][64] : 0;
  a14=(a2-a11);
  a14=(a1*a14);
  a14=casadi_sq(a14);
  a16=arg[0]? arg[0][61] : 0;
  a17=(a4*a16);
  a17=casadi_sq(a17);
  a14=(a14+a17);
  a3=(a3+a14);
  a14=(a16-a5);
  a14=(a6*a14);
  a14=casadi_sq(a14);
  a3=(a3+a14);
  a10=(a10*a2);
  a9=(a9-a10);
  a10=casadi_sq(a2);
  a12=(a12*a10);
  a9=(a9-a12);
  a8=(a8*a9);
  a13=(a13*a16);
  a13=(a13/a15);
  a8=(a8+a13);
  a7=(a7*a8);
  a2=(a2+a7);
  a2=(a2-a11);
  a1=(a1*a2);
  a1=casadi_sq(a1);
  a4=(a4*a16);
  a4=casadi_sq(a4);
  a1=(a1+a4);
  a16=(a16-a5);
  a6=(a6*a16);
  a6=casadi_sq(a6);
  a1=(a1+a6);
  a3=(a3+a1);
  a0=(a0*a3);
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int open_phi_controller_solver(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int open_phi_controller_solver_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int open_phi_controller_solver_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_phi_controller_solver_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int open_phi_controller_solver_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void open_phi_controller_solver_release(int mem) {
}

CASADI_SYMBOL_EXPORT void open_phi_controller_solver_incref(void) {
}

CASADI_SYMBOL_EXPORT void open_phi_controller_solver_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int open_phi_controller_solver_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int open_phi_controller_solver_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real open_phi_controller_solver_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_phi_controller_solver_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* open_phi_controller_solver_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_phi_controller_solver_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* open_phi_controller_solver_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int open_phi_controller_solver_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
