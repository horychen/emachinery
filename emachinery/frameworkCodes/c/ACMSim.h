#ifndef ACMSIM_H
#define ACMSIM_H
/* useful macros and constants */
#include "typedef.h"

/* This is not Experiment */
#define PC_SIMULATION TRUE
#if PC_SIMULATION
    #define __INVERTER_NONLINEARITY 0   
#endif

#define CURRENT_LOOP_KI_TIMES_TEN FALSE
#define WUBO_ONLINE_TUNING FALSE

/* Standard lib */
// #include <stdbool.h> // bool for _Bool and true for 1
// #include <process.h>//reqd. for system function prototype
// #include <conio.h> // for clrscr, and getch()
// #include <unistd.h> // getcwd
// char cwd[1024]; // current working directory
// printf(getcwd(cwd, sizeof(cwd)));
#include <stdio.h>  // printf #include <stdbool.h> // bool for _Bool and true for 1
#include "stdlib.h" // for rand()
#include "math.h"
#include "time.h"

/* Header files in one place*/
// Everthing that is configurable is in here
#include "super_config.h"
// Shared Flux Estimators
// #include "shared_flux_estimator.h"
// PID Regulator
#include "pi_math.h" /* from CYM */
// SM + IM = ACM
// #include "pmsm_observer.h"
#include "pmsm_comm.h"
#include "simuser_yzz.h"
// Sensor & Inverter
// framework
#include "simuser_bezier.h"
#include "main_switch.h"
#include "simuser_wb.h"
// user algorithms







/* Declaration of Utility Function defined in utility.c */
// For main.c
void write_header_to_file(FILE *fw);
void write_data_to_file(FILE *fw);
void print_info();
// General ones
int isNumber(REAL x);
//inline
REAL sign(REAL x);
//inline
int32 sign_integer(int32 x);
// low pass filter
REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv);

extern REAL one_over_six;
REAL difference_between_two_angles(REAL first, REAL second);




/* MAIN SIMULATION */

#define MACHINE_NUMBER_OF_STATES 5

struct MachineSimulated{ // 仿真电机结构体变量声明
    // # name plate data
    int npp;
    REAL npp_inv;
    REAL IN;
    // # electrical parameters
    REAL R;
    REAL Ld;
    REAL Lq;
    REAL KE;
    REAL Rreq;
    // # mechanical parameters
    REAL Js;
    REAL Js_inv;
    // # states
    int NS;
    REAL x[MACHINE_NUMBER_OF_STATES];
    REAL x_dot[MACHINE_NUMBER_OF_STATES];
    REAL timebase;
    // # inputs
    REAL uAB_dist[2];
    REAL uAB_inverter[2];
    REAL uAB[2];
    REAL uDQ[2];
    REAL TLoad;
    // # output
    REAL varTheta;
    REAL varOmega;
    REAL omega_syn;
    REAL omega_slip;
    REAL theta_d;
    REAL KA;
    REAL iDQ[2];
    REAL iAB[2];
    // REAL psi_DQ[2];
    REAL psi_AB[2];
    REAL emf_AB[2];
    REAL iuvw[3];
    REAL Tem;
    REAL cosT;
    REAL sinT;
    REAL cosT_delay_1p5omegaTs;
    REAL sinT_delay_1p5omegaTs;
    // # simulation settings
    int MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;
    REAL Ts;
    REAL current_theta;
    REAL voltage_theta;
    REAL powerfactor;

    REAL dist_al;
    REAL dist_be;

    REAL ual_c_dist;
    REAL ube_c_dist;
};
extern struct MachineSimulated ACM;

void init_Machine();
void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]);
void RK4(REAL t, REAL *x, REAL hs);
int machine_simulation();
void inverter_model();
void measurement();


// #if PC_SIMULATION == TRUE
#define SYSTEM_HALF_PWM_MAX_COUNT 10000
// #define DATA_FILE_NAME "../dat/10_Flux_Estimator_Simulation_Report.dat"
// #endif

// #if MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL || MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL
//     #define STRUCT_MACHINE_SIMULATED struct InductionMachineSimulated
// #elif MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE
//     #define STRUCT_MACHINE_SIMULATED struct SynchronousMachineSimulated
// #endif


#define SYSTEM_QEP_PULSES_PER_REV (10000)
#define SYSTEM_QEP_REV_PER_PULSE (1e-4)
#define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
#define SYSTEM_QEP_QPOSMAX (9999)
#define SYSTEM_QEP_QPOSMAX_PLUS_1 (10000)
#define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 4860 // cjh tuned with id_cmd = 3A @2024-09-17
#define positive_current_QPOSCNT_counting_down (-1) // 正向旋转的电流导致增量式编码器QEP读数减少 则填 -1，否则默认为 1。

// #define SYSTEM_QEP_PULSES_PER_REV (131072) // 2^17
// #define SYSTEM_QEP_REV_PER_PULSE (7.6293945e-6) // 1 / 2^17
// #define CNT_2_ELEC_RAD (SYSTEM_QEP_REV_PER_PULSE * 2 * M_PI * d_sim.init.npp)
// #define SYSTEM_QEP_QPOSMAX (SYSTEM_QEP_PULSES_PER_REV - 1)
// #define SYSTEM_QEP_QPOSMAX_PLUS_1 (SYSTEM_QEP_PULSES_PER_REV)
// #define MOTOR1_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 30190 // ym tuned with id_cmd = 3A, 20240308
// #define MOTOR2_OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 41668 // cjh tuned with id_cmd = 3A, 20240715

#endif
