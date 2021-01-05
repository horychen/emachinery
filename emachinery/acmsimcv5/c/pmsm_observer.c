#include "ACMSim.h"
#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

// DSP中用到的同步电机结构体变量声明
struct SynchronousMachine sm;
struct Harnefors2006 harnefors;

void acm_init(){
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

    sm.npp     = MOTOR_NUMBER_OF_POLE_PAIRS;
    sm.npp_inv = 1.0/sm.npp;

    sm.R      = PMSM_RESISTANCE;
    sm.Ld     = PMSM_D_AXIS_INDUCTANCE;
    sm.Ld_inv = 1/sm.Ld;
    sm.Lq     = PMSM_Q_AXIS_INDUCTANCE;
    sm.KE     = PMSM_PERMANENT_MAGNET_FLUX_LINKAGE; // Vs/rad

    sm.Js     = MOTOR_SHAFT_INERTIA;
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



    // REAL hall_speedRad = 0;
    // REAL hall_speedRad_pre = 0;
    // REAL hall_speedRad_tmp = 0;

    // Uint32 hall_speed_cnt = 0;
    // Uint32 hall_speed_cnt_pre = 65535;

    // REAL hall_change_cnt = 1;
    // REAL hall_change_angle = 330;

    // REAL hall_angle_est = 0;
    // REAL hall_angle_est_tmp = 0;

    // Uint32 hall_previous_previous_angle = 0;
    // Uint32 hall_previous_angle = 0;
    // Uint32 hall_current_angle = 300;

    harnefors.theta_d = 0.0;
    harnefors.omg_elec = 0.0;
}


// Harnefors 2006
void harnefors_scvm(){
    #define LAMBDA 2 // 2
    #define CJH_TUNING_C 0
    REAL lambda_s = CJH_TUNING_C * LAMBDA * sign(harnefors.omg_elec);

    // #define CJH_TUNING_A  25 // low voltage servo motor (<48 Vdc)
    // #define CJH_TUNING_B  1  // low voltage servo motor (<48 Vdc)
    // REAL alpha_bw_lpf = CJH_TUNING_A*0.1*(1500*RPM_2_RAD_PER_SEC) + CJH_TUNING_B*2*LAMBDA*fabs(harnefors.omg_elec);

    #define CJH_TUNING_A  10 // low voltage servo motor (<48 Vdc)
    #define CJH_TUNING_B  1  // low voltage servo motor (<48 Vdc)
    REAL alpha_bw_lpf = CJH_TUNING_A*0.1*(750*RPM_2_RAD_PER_SEC) + CJH_TUNING_B*2*LAMBDA*fabs(harnefors.omg_elec);

    static REAL last_id = 0.0;
    static REAL last_iq = 0.0;

    // #define D_AXIS_CURRENT CTRL.id_cmd
    // #define Q_AXIS_CURRENT CTRL.iq_cmd
    // REAL deriv_id = (CTRL.id_cmd - last_id) * CL_TS_INVERSE;
    // REAL deriv_iq = (CTRL.iq_cmd - last_iq) * CL_TS_INVERSE;
    // last_id = CTRL.id_cmd;
    // last_iq = CTRL.iq_cmd;

    #define D_AXIS_CURRENT CTRL.id__fb
    #define Q_AXIS_CURRENT CTRL.iq__fb
    REAL deriv_id = (CTRL.id__fb - last_id) * CL_TS_INVERSE;
    REAL deriv_iq = (CTRL.iq__fb - last_iq) * CL_TS_INVERSE;
    last_id = CTRL.id__fb;
    last_iq = CTRL.iq__fb;

    REAL d_axis_emf = CTRL.ud_cmd - 1*CTRL.R*D_AXIS_CURRENT + harnefors.omg_elec*1.0*CTRL.Lq*Q_AXIS_CURRENT - 1.0*CTRL.Ld*deriv_id; // eemf
    REAL q_axis_emf = CTRL.uq_cmd - 1*CTRL.R*Q_AXIS_CURRENT - harnefors.omg_elec*1.0*CTRL.Ld*D_AXIS_CURRENT - 1.0*CTRL.Lq*deriv_iq; // eemf

    // printf("%g: %g, %g\n", CTRL.timebase, CTRL.Ld*deriv_id, CTRL.Lq*deriv_iq);

    // Note it is bad habit to write numerical integration explictly like this. The states on the right may be accencidentally modified on the run.
    #define KE_MISMATCH 1.0 // 0.7
    harnefors.theta_d += CL_TS * harnefors.omg_elec;
    harnefors.omg_elec += CL_TS * alpha_bw_lpf * ( (q_axis_emf - lambda_s*d_axis_emf)/(CTRL.KE*KE_MISMATCH+(CTRL.Ld-CTRL.Lq)*D_AXIS_CURRENT) - harnefors.omg_elec );

    while(harnefors.theta_d>M_PI) harnefors.theta_d-=2*M_PI;
    while(harnefors.theta_d<-M_PI) harnefors.theta_d+=2*M_PI;
}


#endif