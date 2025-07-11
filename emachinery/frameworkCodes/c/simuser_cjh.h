#ifndef SIMUSER_CJH_H
#define SIMUSER_CJH_H

#if WHO_IS_USER == USER_CJH

#include "ACMSim.h"

// void control(REAL speed_cmd, REAL speed_cmd_dot);


/* 逆变器非线性 */
/* 查表法 */
void get_distorted_voltage_via_LUT_indexed(REAL ial, REAL ibe, REAL *ualbe_dist);
void get_distorted_voltage_via_LUT(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist, REAL *lut_voltage, REAL *lut_current, int length_of_lut);
void get_distorted_voltage_via_CurveFitting(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist);

/* ParkSul2012 梯形波 */
// #define GAIN_THETA_TRAPEZOIDAL (20) // 20
void inverterNonlinearity_Initialization();
REAL u_comp_per_phase(REAL Vsat, REAL thetaA, REAL theta_trapezoidal, REAL oneOver_theta_trapezoidal);
REAL lpf1(REAL x, REAL y_tminus1);
REAL shift2pi(REAL thetaA);

void Modified_ParkSul_Compensation(void);
void Online_PAA_Based_Compensation(void);

/* Main */
void _main_inverter_voltage_command(int bool_use_iab_cmd);

















#ifndef IM_CONTROLLER_H
#define IM_CONTROLLER_H
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

// 这个结构体声明的是基本的IFOC中所没有的变量的集合体。
struct Marino2005{
    REAL kz;     // zd, zq
    REAL k_omega; // e_omega
    REAL kappa;  // e_omega
    REAL gamma_inv; // TL
    REAL delta_inv; // alpha
    REAL lambda_inv; // omega

    REAL xTL_Max;
    REAL xAlpha_Max;
    REAL xAlpha_min;

    REAL xRho;
    REAL xTL;
    REAL xAlpha;
    REAL xOmg;

    REAL deriv_xTL;
    REAL deriv_xAlpha;
    REAL deriv_xOmg;

    REAL psi_Dmu;
    REAL psi_Qmu;

    REAL zD;
    REAL zQ;
    REAL e_iDs;
    REAL e_iQs;
    REAL e_psi_Dmu;
    REAL e_psi_Qmu;

    REAL deriv_iD_cmd;
    REAL deriv_iQ_cmd;

    REAL Gamma_D;
    REAL Gamma_Q;

    REAL torque_cmd;
    REAL torque__fb;
};
extern struct Marino2005 marino;

// 控制器
void controller_IFOC();
void controller_marino2005();
void controller_marino2005_with_commands();
void init_im_controller();

// 初始化
// void experiment_init();
// void CTRL_init();
// void allocate_CTRL(struct ControllerForExperiment *CTRL);


// void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
// void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);


#endif
#endif
#endif







































#ifndef ADD_IM_OBSERVER_H
#define ADD_IM_OBSERVER_H

void rk4_init();
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

/* One Big Struct for all PMSM observers */
// struct ObserverForExperiment{
//     /* Common */
//     struct RK4_DATA{
//         REAL us[2];
//         REAL is[2];
//         REAL us_curr[2];
//         REAL is_curr[2];
//         REAL us_prev[2];
//         REAL is_prev[2];

//         REAL is_lpf[2];
//         REAL is_hpf[2];
//         REAL is_bpf[2];

//         REAL current_lpf_register[2];
//         REAL current_hpf_register[2];
//         REAL current_bpf_register1[2];
//         REAL current_bpf_register2[2];

//         // REAL omg_elec; // omg_elec = npp * omg_mech
//         // REAL theta_d;
//     } rk4;

// /* EXAMPLE */
// #if PC_SIMULATION || SELECT_ALGORITHM == ALG_EXAMPLE

//     struct Declare_EXAMPLE{
//         #define NS_EXAMPLE 5
//         REAL x[NS_EXAMPLE];
//     } obsv_example;
// #endif

// };
// extern struct ObserverForExperiment OBSV;



struct ObserverControl{

    REAL k_AP_I;
    REAL k_AP_P;
    REAL k_AP_D;
    REAL k_RP_I;
    REAL k_RP_P;
    REAL k_RP_D;

    REAL xIs[2];    // \psi_\sigma
    REAL xPsiMu[2];       // \psi_\mu
    REAL xOmg;
    REAL xTL; 
    REAL xTL_integral_part_AP; 
    REAL xTL_integral_part_RP; 
    REAL xTem;
    // REAL refined_omg;
    REAL actual_iTs;

    REAL xUps_al[6];     // The alpha component of the filtered regressors
    REAL xUps_be[6];     // The beta component of the filtered regressors
    REAL xTheta[2]; 

    REAL mismatch[3];
    REAL error[2];
    REAL varepsilon_AP;
    REAL varepsilon_RP;

    REAL epsilon_AP; // estimate of varepsilon
    REAL varsigma_AP; // estimate of dot varepsilon
    REAL epsilon_RP; // estimate of varepsilon
    REAL varsigma_RP; // estimate of dot varepsilon
    REAL lambda1;
    REAL lambda2;

    REAL taao_alpha; 
    REAL taao_omg_integralPart; // 纯积分的自适应律就用不到这个
    REAL taao_speed; // 机械速度rpm！再强调一遍，不是电气速度rpm，而是机械速度rpm。

    REAL timebase;
    REAL Ts;

    REAL omega_e;
    REAL Tem;

    REAL taao_flux_cmd;
    int taao_flux_cmd_on;

    REAL cosT;
    REAL sinT;
    REAL theta_M;

    REAL actual_flux[2];
    REAL actual_TL;
    REAL actual_z;

    REAL k_Gopinath;
    REAL k_1minusGopinath_inv;
    REAL xXi[2];
};
extern struct ObserverControl ob;

void observer_init();
void simulation_only_flux_estimator();
void observer_marino2005();
void observer_PMSMife();

// struct Chen21_ESO_AF{
//     #define NS_CHEN_2021 4
//     REAL xPos;
//     REAL xOmg;
//     REAL xTL;
//     REAL xPL; // rotatum
//     REAL x[NS_CHEN_2021];
//     REAL ell[NS_CHEN_2021];

//     int bool_ramp_load_torque; // TRUE for 4th order ESO
//     REAL omega_ob; // one parameter tuning
//     REAL set_omega_ob;

//     REAL output_error_sine; // sin(\tilde\vartheta_d)
//     REAL output_error; // \tilde\vartheta_d, need to detect bound jumping 

//     REAL xTem;
// };
// extern struct Chen21_ESO_AF OFSR.esoaf;
void Main_esoaf_chen2021();

/********************************************
 * Collections of VM based Flux Estimators 
 ********************************************/
void flux_observer();

struct Variables_SimulatedVM{
    REAL emf[2];
    REAL emf_DQ[2];

    REAL psi_1[2];
    REAL psi_2[2];
    REAL psi_2_ampl;
    REAL u_offset[2];
    REAL psi_D2_ode1_v2;
    REAL psi_Q2_ode1_v2;

    REAL psi_D1_ode1;
    REAL psi_Q1_ode1;
    REAL psi_D2_ode1;
    REAL psi_Q2_ode1;

    REAL psi_D1_ode4;
    REAL psi_Q1_ode4;
    REAL psi_D2_ode4;
    REAL psi_Q2_ode4;

    REAL psi_D2;
    REAL psi_Q2;
};
extern struct Variables_SimulatedVM simvm;


// #define VM_HoltzQuan2003 VM_Saturated_ExactOffsetCompensation
void VM_Ohtani1992();
void VM_HoltzQuan2002();
void VM_HoltzQuan2003();
void init_LascuAndreescus2006();
void VM_LascuAndreescus2006();
void VM_HuWu1998();
void VM_Stojic2015();
void VM_Harnefors2003_SCVM();
/* A */
void VM_ExactCompensation();
/* B */
void VM_ProposedCmdErrFdkCorInFrameRho();
/* C */
void VM_ClosedLoopFluxEstimator();

void VM_Saturated_ExactOffsetCompensation();
void VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
void VM_Saturated_ExactOffsetCompensation_WithParallelNonSaturatedEstimator();
// void stableFME();

// extern struct Variables_Ohtani1992 ohtani;
// extern struct Variables_HuWu1998 huwu;
// extern struct Variables_HoltzQuan2002 holtz02;
// extern struct Variables_Holtz2003 htz;
// extern struct Variables_Harnefors2003_SCVM harnefors;
// extern struct Variables_LascuAndreescus2006 lascu;
// extern struct Variables_Stojic2015 stojic;
// extern struct Variables_fluxModulusEstimator fme;
// /* A */
// extern struct Variables_ExactCompensationMethod exact;
// /* B */
// extern struct Variables_ProposedxRhoFramePICorrectionMethod picorr;
// /* C */
// extern struct Variables_ClosedLoopFluxEstimator clest;

#endif
#endif
#endif






#ifndef MEASUREMENT_H
#define MEASUREMENT_H

extern void measurement();


// Hall Sensor
struct HallSensor{

    REAL omg_elec_hall;
    REAL omg_mech_hall;
    REAL theta_d_hall;

    REAL last_omg_elec_hall;
    REAL acceleration_avg;

    int16 hallA;
    int16 hallB;
    int16 hallC;
    int16 hallABC;
    int16 last_hallABC;
    int16 speed_direction;
    int16 bool_position_correction_pending;

    REAL timebase;
    REAL timeStamp;
    REAL timeDifference;
    REAL timeDifferenceStamp;



    /* In hall_resolver() */

    REAL hall_speedRad; // 电气转速输出
    REAL hall_speedRad_pre; // 用于低通滤波（无效）
    REAL hall_speedRad_tmp; // 这个变量其实是不必要的，等于60°/时间

    Uint32 hall_speed_cnt;
    Uint32 hall_speed_cnt_pre; // 作为扇区内计数的一个指标，标志着电机转速是否慢于上一个扇区。只能判断电机是不是变慢了。

    REAL hall_change_cnt; // 电机走过了几个扇区？一般情况下是1。
    Uint16 hall_change_angle; // 两个扇区角度的平均值，在切换扇区时，等于N极的位置

    Uint16 hall_angle_est; // 电气角度输出
    REAL hall_angle_est_tmp; // 基于霍尔反馈的角度估计的增量，限幅在[-60, 60] elec. deg

    Uint16 hall_previous_previous_angle; // 上上个扇区角度
    Uint16 hall_previous_angle; // 上个扇区角度
    Uint16 hall_current_angle; // 当前扇区角度
};
extern struct HallSensor HALL;


void sensors();


// #define QEP_DEFAULTS {0, 0,0,0,0,0,{0,0,0,0},0,0}
struct eQEP_Variables{
    // REAL excitation_angle_deg; // init in synchronous_motor.c
    // REAL theta_d__state;   // init in synchronous_motor.c
    // REAL theta_d_offset;
    REAL QPOSLAT;
    int32 RoundedPosInCnt;
    int32 RoundedPosInCnt_Previous;
    int32 number_of_revolution;
    int32 difference_in_cnt;
    REAL theta_d;
    REAL theta_d_mech;
    REAL theta_d_accum;
    REAL omg_mech;
    REAL omg_elec;
    REAL moving_average[4];
    int64 ActualPosInCnt;
    int64 ActualPosInCnt_Previous;
};
extern struct eQEP_Variables qep;
#endif

#endif

















#endif
