#include "ACMSim.h"
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

struct RK4_DATA rk4;
// struct InductionMachine im;

void acm_init(){

    rk4.us[0] = 0.0;
    rk4.us[1] = 0.0;
    rk4.is[0] = 0.0;
    rk4.is[1] = 0.0;
    rk4.us_curr[0] = 0.0;
    rk4.us_curr[1] = 0.0;
    rk4.is_curr[0] = 0.0;
    rk4.is_curr[1] = 0.0;
    rk4.us_prev[0] = 0.0;
    rk4.us_prev[1] = 0.0;
    rk4.is_prev[0] = 0.0;
    rk4.is_prev[1] = 0.0;

    //     // im.Js = 0.017876; // 3000rpm阶跃测转动惯量
    //     // im.Js = 0.172; 
    //     // im.Js = 0.072; 
    //     // im.Js = 0.017876*0.7;
    //     // im.Js = MOTOR_SHAFT_INERTIA;
    //     im.Js = MOTOR_SHAFT_INERTIA * (1.0+LOAD_INERTIA);
    //     im.Js_inv = 1.0/im.Js;
    //     im.npp = MOTOR_NUMBER_OF_POLE_PAIRS; // no. of pole pair
    //     im.npp_inv = 1.0/im.npp; // no. of pole pair
    //     im.mu_m = im.npp*im.Js_inv;
    //     im.mu   = im.npp*im.mu_m;
    //     // printf("im.mu_m = %g; im.mu = %g\n", im.mu_m, im.mu);

    //     im.Lmu = IM_MAGNETIZING_INDUCTANCE;
    //     im.Lmu_inv = 1.0/im.Lmu;
    //     im.Lsigma = IM_TOTAL_LEAKAGE_INDUCTANCE;
    //     im.Lsigma_inv = 1.0/im.Lsigma;

    //     im.Ls = im.Lsigma + im.Lmu;

    //     im.rs = IM_STAOTR_RESISTANCE;
    //     im.rreq = IM_ROTOR_RESISTANCE;

    //     im.alpha = im.rreq/im.Lmu;
    //     im.Tr = im.Lmu/im.rreq; 
    //     im.omg_elec = 0.0;
    //     im.omg_mech = 0.0;
}

#endif
