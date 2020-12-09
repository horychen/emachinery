#include "ACMSim.h"

// DSP中用到的同步电机结构体变量声明
struct SynchronousMachine sm;

void sm_init(){
    int i;
    for(i=0; i<2; ++i){
        sm.us[i] = 0;
        sm.is[i] = 0;
        sm.us_curr[i] = 0;
        sm.is_curr[i] = 0;
        sm.us_prev[i] = 0;
        sm.is_prev[i] = 0;
        sm.is_lpf[i]  = 0;
        sm.is_hpf[i]  = 0;
        sm.is_bpf[i]  = 0;

        sm.current_lpf_register[i] = 0;
        sm.current_hpf_register[i] = 0;
        sm.current_bpf_register1[i] = 0;
        sm.current_bpf_register2[i] = 0;
    }

    sm.npp     = PMSM_NUMBER_OF_POLE_PAIRS;
    sm.npp_inv = 1.0/sm.npp;

    sm.R      = PMSM_RESISTANCE;
    sm.Ld     = PMSM_D_AXIS_INDUCTANCE;
    sm.Ld_inv = 1/sm.Ld;
    sm.Lq     = PMSM_Q_AXIS_INDUCTANCE;
    sm.KE     = PMSM_PERMANENT_MAGNET_FLUX_LINKAGE; // Vs/rad

    sm.Js     = PMSM_SHAFT_INERTIA;
    sm.Js_inv = 1./sm.Js;

    sm.omg_elec = 0.0;
    sm.omg_mech = sm.omg_elec * sm.npp_inv;
    sm.theta_d = 0.0;

    sm.omg_elec_hall = 0.0;
    sm.omg_mech_hall = 0.0;
    sm.theta_d_hall = 0.0;

    sm.acceleration_avg = 0.0;
    sm.last_omg_elec_hall = 0.0;

    sm.hallA = 0;
    sm.hallB = 1;
    sm.hallC = 1;
    sm.hallABC = 0x3;
    sm.last_hallABC = 0x1;
    sm.speed_direction = +1;
    sm.bool_position_correction_pending = FALSE;

    sm.timebase = 0.0;
    sm.timeStamp = 0.0;
    sm.timeDifference = 0.0;
    sm.timeDifferenceStamp = 100000.0;



    REAL hall_speedRad = 0;
    REAL hall_speedRad_pre = 0;
    REAL hall_speedRad_tmp = 0;

    Uint32 hall_speed_cnt = 0;
    Uint32 hall_speed_cnt_pre = 65535;

    REAL hall_change_cnt = 1;
    REAL hall_change_angle = 330;

    REAL hall_angle_est = 0;
    REAL hall_angle_est_tmp = 0;

    Uint32 hall_previous_previous_angle = 0;
    Uint32 hall_previous_angle = 0;
    Uint32 hall_current_angle = 300;
}




// Harnefors 2006
double theta_d_harnefors = 0.0;
double omg_harnefors = 0.0;
void harnefors_scvm(){
    #define KE_MISMATCH 1.0 // 0.7
    double d_axis_emf;
    double q_axis_emf;
    #define LAMBDA 2 // 2
    #define CJH_TUNING_A 25 // 1
    #define CJH_TUNING_B 1 // 1
    double lambda_s = LAMBDA * sign(omg_harnefors);
    double alpha_bw_lpf = CJH_TUNING_A*0.1*(1500*RPM_2_RAD_PER_SEC) + CJH_TUNING_B*2*LAMBDA*fabs(omg_harnefors);
    // d_axis_emf = CTRL.ud_cmd - 1*CTRL.R*CTRL.id_cmd + omg_harnefors*1.0*CTRL.Lq*CTRL.iq_cmd; // If Ld=Lq.
    // q_axis_emf = CTRL.uq_cmd - 1*CTRL.R*CTRL.iq_cmd - omg_harnefors*1.0*CTRL.Ld*CTRL.id_cmd; // If Ld=Lq.
    d_axis_emf = CTRL.ud_cmd - 1*CTRL.R*CTRL.id_cmd + omg_harnefors*1.0*CTRL.Lq*CTRL.iq_cmd; // eemf
    q_axis_emf = CTRL.uq_cmd - 1*CTRL.R*CTRL.iq_cmd - omg_harnefors*1.0*CTRL.Ld*CTRL.id_cmd; // eemf
    // Note it is bad habit to write numerical integration explictly like this. The states on the right may be accencidentally modified on the run.
    theta_d_harnefors += CL_TS * omg_harnefors;
    omg_harnefors += CL_TS * alpha_bw_lpf * ( (q_axis_emf - lambda_s*d_axis_emf)/(CTRL.KE*KE_MISMATCH+(CTRL.Ld-CTRL.Lq)*CTRL.id_cmd) - omg_harnefors );
    while(theta_d_harnefors>M_PI) theta_d_harnefors-=2*M_PI;
    while(theta_d_harnefors<-M_PI) theta_d_harnefors+=2*M_PI;   
}


