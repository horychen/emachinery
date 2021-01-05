#ifndef ACMSIM_H
#define ACMSIM_H

/* standard lib */
// #include <stdbool.h> // bool for _Bool and true for 1
#include <stdio.h> // printf #include <stdbool.h> // bool for _Bool and true for 1
#include <process.h>//reqd. for system function prototype
#include <conio.h> // for clrscr, and getch()
#include "stdlib.h" // for rand()
#include "math.h"
#include "time.h"

/* iSMC */
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int int16;
typedef unsigned int Uint16;
typedef long int32;
typedef unsigned long Uint32;
typedef long long int64;
typedef unsigned long long Uint64;
typedef float float32;
typedef long double float64;
#endif
// typedef float32 REAL;
typedef double REAL;

#define CONST_PI_OVER_180 (0.0174533)
#define CONST_180_OVER_PI (57.2958)
#define CONST_1_OVER_SQRT3 (0.57735)

/* Macro for Park transformation*/
#define AB2M(A, B, COS, SIN)  ( (A)*COS  + (B)*SIN )
#define AB2T(A, B, COS, SIN)  ( (A)*-SIN + (B)*COS )
#define MT2A(M, T, COS, SIN)  ( (M)*COS - (T)*SIN )
#define MT2B(M, T, COS, SIN)  ( (M)*SIN + (T)*COS )

/* Macro for two-phase Amplitude-invariant Clarke transformation*/
#define UV2A_AI(U, V) ( U )
#define UV2B_AI(U, V) ( (U + 2*(V)) * CONST_1_OVER_SQRT3 )
#define AB2U_AI(A, B) ( ( A ) )
#define AB2V_AI(A, B) ( ( (A)*-0.5 + (B)*0.866 ) )
#define AB2W_AI(A, B) ( ( (A)*-0.5 + (B)*-0.866 ) )

// Macro for Power-invariant inverse Clarke transformation
#define AB2U_PI(A, B) ( 0.816496580927726 * ( A ) )
#define AB2V_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*0.8660254037844387 ) )
#define AB2W_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*-0.8660254037844385 ) )

/* General Constants */
#define TRUE  (1)
#define FALSE (0)
#define ONE_OVER_2PI          0.15915494309189535 // 1/(2*pi)
#define TWO_PI_OVER_3         2.0943951023931953
#define SIN_2PI_SLASH_3       0.86602540378443871 // sin(2*pi/3)
#define SIN_DASH_2PI_SLASH_3 -0.86602540378443871 // sin(-2*pi/3)
#define SQRT_2_SLASH_3        0.81649658092772603 // sqrt(2.0/3.0)
#define abs                   use_fabs_instead_or_you_will_regret
#define RAD_PER_SEC_2_RPM ( 60.0/(2*M_PI*ACM.npp) )
#define RPM_2_RAD_PER_SEC ( (2*M_PI*ACM.npp)/60.0 )
#define M_PI_OVER_180   0.017453292519943295

#define PHASE_NUMBER 3 // three phase machine

// 模拟测量环节，可以在此为测量电机添加噪声、温飘等，也可以在此实现类似光电编码器的转速测量环节。
#define RANDOM ( ((double) rand() / (RAND_MAX))*2 - 1 )


// 畸变后的定子电压
#define UAL_C_DIST ACM.ual_c_dist // alpha-component of the distorted phase voltage = sine voltage + distored component
#define UBE_C_DIST ACM.ube_c_dist
// 仅畸变部分
#define DIST_AL ACM.dist_al // alpha-component of the distorted component of the voltage
#define DIST_BE ACM.dist_be

// Everthing that is configurable is in here
#include "ACMConfig.h"

#include "synchronous_motor.h"
#include "pmsm_controller.h"
#include "pmsm_observer.h"

#include "induction_motor.h"
#include "im_controller.h"
#include "im_observer.h"

#include "pid_regulator.h"
#include "measurement.h"
#include "inverter.h"
#include "load.h"

#include "pmsm_comm.h"
#include "sweep_frequency.h"




/* Declaration of Utility Function */
void write_header_to_file(FILE *fw);
void write_data_to_file(FILE *fw);
int isNumber(double x);
double sign(double x);
double fabs(double x);
REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv);

// #include <unistd.h> // getcwd
    // dir
    // char cwd[1024];
    // printf(getcwd(cwd, sizeof(cwd)));

#endif