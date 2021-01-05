#ifndef ADD_IM_OBSERVER_H
#define ADD_IM_OBSERVER_H
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

/* Macro for External Access Interface */
#define US(X) im.us[X]
#define IS(X) im.is[X]
#define US_C(X) im.us_curr[X]
#define IS_C(X) im.is_curr[X]
#define US_P(X) im.us_prev[X]
#define IS_P(X) im.is_prev[X]

struct InductionMachine{
    double us[2];
    double is[2];
    double us_curr[2];
    double is_curr[2];
    double us_prev[2];
    double is_prev[2];

    double Js;
    double Js_inv;
    double npp;
    double npp_inv;
    double mu_m;
    double mu;

    // double Lm;
    // double Lm_inv;
    // double Lls;
    // double Llr;
    // double Lr;
    // double sigma;
    // double rr;
    double Lmu;
    double Lmu_inv;
    double Lsigma;
    double Lsigma_inv;
    double Ls;
    double rs;
    double rreq;
    double alpha;
    double Tr;

    // double omg;
    double omg_elec;
    double omg_mech;
};
extern struct InductionMachine im;

struct ObserverControl{
    
    double k_AP_I;
    double k_AP_P;
    double k_AP_D;
    double k_RP_I;
    double k_RP_P;
    double k_RP_D;

    double xIs[2];    // \psi_\sigma
    double xPsiMu[2];       // \psi_\mu
    double xOmg;
    double xTL; 
    double xTL_integral_part_AP; 
    double xTL_integral_part_RP; 
    double xTem;
    // double refined_omg;
    double actual_iTs;

    double xUps_al[6];     // The alpha component of the filtered regressors
    double xUps_be[6];     // The beta component of the filtered regressors
    double xTheta[2]; 

    double mismatch[3];
    double error[2];
    double varepsilon_AP;
    double varepsilon_RP;

    double epsilon_AP; // estimate of varepsilon
    double varsigma_AP; // estimate of dot varepsilon
    double epsilon_RP; // estimate of varepsilon
    double varsigma_RP; // estimate of dot varepsilon
    double lambda1;
    double lambda2;

    double taao_alpha; 
    double taao_omg_integralPart; // 纯积分的自适应律就用不到这个
    double taao_speed; // 机械速度rpm！再强调一遍，不是电气速度rpm，而是机械速度rpm。

    double timebase;
    double Ts;

    double omega_e;
    double Tem;

    double taao_flux_cmd;
    int taao_flux_cmd_on;

    double cosT;
    double sinT;
    double theta_M;

    double actual_flux[2];
    double actual_TL;
    double actual_z;

    double k_Gopinath;
    double k_1minusGopinath_inv;
    double xXi[2];
};
extern struct ObserverControl ob;

void acm_init();

void improved_Holtz_method();
void observer_marino2005();

#endif
#endif