#ifndef INDUCTION_MOTOR_H
#define INDUCTION_MOTOR_H
#if MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL || MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL

// The function name for the dynamics
#if MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL
    #define MACHINE_DYNAMICS IM_linear_Dynamics
#elif MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL
    #define MACHINE_DYNAMICS IM_saturated_Dynamics
#endif
// 4 elec + 2 mech
#define NUMBER_OF_STATES 7

struct InductionMachineSimulated{
    double timebase;
    double x[NUMBER_OF_STATES];
    double x_dot[NUMBER_OF_STATES];
    double rpm;
    double rpm_cmd;
    double rpm_deriv_cmd;
    double TLoad;
    double Tem;

    double theta_d; // 转子（假想）d轴位置
    double theta_d_accum; // 转子（假想）d轴位置

    double theta_M;
    double cosT; // cosine for Park Transform
    double sinT; // sine for Park Transform

    double omg_elec;

    double Lsigma;
    double Lsigma_inv;
    double rs;
    double rreq;
    double Lmu;
    double Lmu_inv;
    double alpha;

    double Lm;
    double rr;
    double Lls;
    double Llr;
    double Lm_slash_Lr;
    double Lr_slash_Lm;
    
    double LSigmal;

    double Js;
    double npp;
    double mu_m;
    double Ts;

    double izq;
    double izd;
    double iz;
    
    double psimq;
    double psimd;
    double psim;
    double im;

    double ids; // d-axis is aligned with alpha-axis
    double iqs; // q-axis is aligned with beta-axis
    double idr;
    double iqr;
    double iMs; // rotor field oritented M-axis current
    double iTs; // rotor field oritented T-axis current
    double psi_Dmu;
    double psi_Qmu;

    double ual;
    double ube;
    double ial;
    double ibe;

    double lst_psim[16];

    double ual_c_dist;
    double ube_c_dist;
    double dist_al;
    double dist_be;

    double cur_offset;

    double pis_curr[2];
    double pis_prev[2];
};
extern struct InductionMachineSimulated ACM;

extern int machine_simulation();
extern void Machine_init();

#endif
#endif