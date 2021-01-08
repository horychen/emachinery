#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11
#ifndef IM_CONTROLLER_H
#define IM_CONTROLLER_H

struct ControllerForExperiment {

    REAL timebase;

    REAL iDs;
    REAL iQs;
    REAL iDs_cmd;
    REAL iQs_cmd;

    REAL uDs_cmd;
    REAL uQs_cmd;
    REAL ual_cmd;
    REAL ube_cmd;

    REAL omg_cmd;
    REAL deriv_omg_cmd;
    REAL dderiv_omg_cmd;
    REAL torque_cmd;

    REAL speed_ctrl_err;
    // REAL omg_ctrl_err;

    REAL theta_D;
    REAL cosT;
    REAL sinT;

    REAL e_M;
    REAL e_T;

    // struct PI_Reg pi_speed;
    // struct PI_Reg pi_iMs;
    // struct PI_Reg pi_iTs;

    REAL omega_sl;
    REAL omega_syn;

    // REAL tajima_omg;
    // REAL K_PEM;

    REAL omg__fb;
    REAL ial__fb;
    REAL ibe__fb;

    // REAL speed_fb;
    // REAL psial_fb;
    // REAL psibe_fb;
    // REAL psimod_fb;
    // REAL psimod_fb_inv;

    REAL psi_cmd_raw;
    REAL psi_cmd;
    REAL psi_cmd_inv;
    REAL dderiv_psi_cmd;
    REAL deriv_psi_cmd;
    REAL m0;
    REAL m1;
    REAL omega1;

    int npp;
    REAL npp_inv;
    REAL rs;
    REAL rreq;
    REAL Lsigma;
    REAL Lsigma_inv;
    REAL Lmu;
    REAL Lmu_inv;
    REAL alpha;
    REAL alpha_inv;

    REAL Js;
    REAL Js_inv;

    REAL TLoad;


    // REAL gamma_omg_transient;
    // REAL gamma_omg_transient_shape;
    // REAL gamma_res_transient;
    // REAL gamma_res_transient_shape;
    // REAL gamma_res2_transient;
    // REAL gamma_res2_transient_shape;
    // int resistance_id_on;

    int sensorless;
    int ctrl_strategy;

};
extern struct ControllerForExperiment CTRL;

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

struct Holtz2003{
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
extern struct Holtz2003 holtz;

void CTRL_init();
void control(REAL speed_cmd, REAL speed_cmd_dot);


void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);


void controller();


#endif
#endif
