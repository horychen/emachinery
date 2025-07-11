#ifndef TYPEDEF_H
#define TYPEDEF_H

#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef          int int16;
typedef unsigned int Uint16;
typedef          long int32;
typedef unsigned long Uint32;
typedef          long long int64;
typedef unsigned long long Uint64;
typedef float float32;
typedef double float64;
typedef long double float128;
#endif

typedef float32 REAL;

/* Constants */
#define CONST_PI_OVER_180 (0.0174533)
#define CONST_180_OVER_PI (57.2958)
#define CONST_1_OVER_SQRT3 (0.57735)

/* Macro for two-phase Amplitude-invariant Clarke transformation*/
#define UV2A_AI(U, V) ( U )
#define UV2B_AI(U, V) ( (U + 2*(V)) * CONST_1_OVER_SQRT3 )

#define UVW2A_AI(U, V, W) ( 0.666667 * U - 0.333333 * V - 0.333333 * W )
#define UVW2B_AI(U, V, W) ( 0 * U + (V - W) * CONST_1_OVER_SQRT3 )
#define UVW2G_AI(U, V, W) ( 0.333333 * (U + V + W))

#define AB2U_AI(A, B) ( ( A ) )
#define AB2V_AI(A, B) ( ( (A)*-0.5 + (B)*0.866 ) )
#define AB2W_AI(A, B) ( ( (A)*-0.5 + (B)*-0.866 ) )

// Macro for Power-invariant inverse Clarke transformation
#define AB2U_PI(A, B) ( 0.816496580927726 * ( A ) )
#define AB2V_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*0.8660254037844387 ) )
#define AB2W_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*-0.8660254037844385 ) )

/* Macro for Park transformation*/
#define AB2M(A, B, COS, SIN) ((A) * COS + (B) * SIN)
#define AB2T(A, B, COS, SIN) ((A) * -SIN + (B) * COS)
#define MT2A(M, T, COS, SIN) ((M) * COS - (T) * SIN)
#define MT2B(M, T, COS, SIN) ((M) * SIN + (T) * COS)

/* General Constants */
#ifndef BOOL
    #define BOOL int
#endif
#ifndef TRUE
    #define TRUE (1)
#endif
#ifndef FALSE
    #define FALSE (0)
#endif
#define M_PI		3.14159265358979323846
#define M_SQRT2 	1.41421356237309504880
#define ONE_OVER_2PI 0.15915494309189535 // 1/(2*pi)
#define ONE_OVER_60 0.01666666666666667
#define TWO_PI_OVER_3 2.0943951023931953
#define SIN_2PI_SLASH_3 0.86602540378443871       // sin(2*pi/3)
#define SIN_DASH_2PI_SLASH_3 -0.86602540378443871 // sin(-2*pi/3)
#define SQRT_2_SLASH_3 0.81649658092772603        // sqrt(2.0/3.0)
#define abs use_fabsf_instead_or_you_will_regret
#define MECH_RAD_PER_SEC_2_RPM (60.0 * ONE_OVER_2PI)
#define ELEC_RAD_PER_SEC_2_RPM (60.0 * ONE_OVER_2PI * (*CTRL).motor->npp_inv)
// #define ELEC_RAD_PER_SEC_2_RPM ( 60.0/(2*M_PI*(*CTRL).motor->npp) )
#define RPM_2_ELEC_RAD_PER_SEC ((2 * M_PI * (*CTRL).motor->npp) * ONE_OVER_60)
#define RPM_2_MECH_RAD_PER_SEC ((2 * M_PI * ONE_OVER_60))
// #define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*(*CTRL).motor->npp)/60.0 )
// New convention
#define M_PI_OVER_180 0.017453292519943295
#define CONST_PI_OVER_180 (0.0174533)
#define CONST_180_OVER_PI (57.2958)
#define CONST_1_OVER_SQRT3 (0.57735)


/* Motor Control Related Utility Macros */
#define CLARKE_TRANS_TORQUE_GAIN (1.5) // consistent with experiment
#define CLARKE_TRANS_TORQUE_GAIN_INVERSE (0.666666667)
#define POW2AMPL (0.816496581) // = 1/sqrt(1.5) power-invariant to aplitude-invariant (the dqn vector becomes shorter to have the same length as the abc vector)
#define AMPL2POW (1.22474487)

// 模拟测量环节，可以在此为测量电机添加噪声、温飘等，也可以在此实现类似光电编码器的转速测量环节。
#define RANDOM (((REAL)rand() / (RAND_MAX)) * 2 - 1) // [-1, 1]

extern REAL one_over_six;



/* User */
#include "super_config.h"
#include "main_switch.h"

// 速度环 increase to 3 times because of the bug in dynamics clamping
#define VL_SERIES_KI_CODE             (VL_SERIES_KI*VL_SERIES_KP*VL_TS)
#define SPEED_LOOP_LIMIT_AMPERE       (1 * d_sim.init.npp) // (1.0*1.414*INIT_IN)

/* 控制策略 */
    // #define VOLTAGE_CURRENT_DECOUPLING_CIRCUIT FALSE
    #define SATURATED_MAGNETIC_CIRCUIT FALSE

#define CL_TS          (d_sim.sim.CLTS)
#define CL_TS_INVERSE  (1.0 / CL_TS)
#define VL_TS          (d_sim.FOC.VL_EXE_PER_CL_EXE*CL_TS)
#define PL_TS          VL_TS
#define SPEED_LOOP_CEILING (d_sim.FOC.VL_EXE_PER_CL_EXE)
    // #define TS_UPSAMPLING_FREQ_EXE (CL_TS / MACHINE_TS)
    // #define TS_UPSAMPLING_FREQ_EXE_INVERSE (1.0/TS_UPSAMPLING_FREQ_EXE)
    #define MACHINE_TS         (CL_TS/d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD)
    #define MACHINE_TS_INVERSE (1.0/MACHINE_TS)

// #define LOAD_INERTIA    0.0
#define LOAD_TORQUE     0.2 // 0.0
#define VISCOUS_COEFF   0.0007 // 0.0007



#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE d_sim.init.KE
    #define MOTOR_RATED_POWER_WATT 750
    #define MOTOR_RATED_SPEED_RPM 3000
    #define MOTOR_RATED_TORQUE ( MOTOR_RATED_POWER_WATT / (MOTOR_RATED_SPEED_RPM/60.0*2*3.1415926) )
    #define MOTOR_TORQUE_CONSTANT ( MOTOR_RATED_TORQUE / (d_sim.init.IN*1.414) )
    #define MOTOR_BACK_EMF_CONSTANT ( MOTOR_TORQUE_CONSTANT / 1.5 / d_sim.init.npp )
    #define MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM ( MOTOR_BACK_EMF_CONSTANT * 1e3 / (1.0/d_sim.init.npp/2/3.1415926*60) )
    #define SPEED_LOOP_LIMIT_NEWTON_METER (1.0*MOTOR_RATED_TORQUE)










#endif
