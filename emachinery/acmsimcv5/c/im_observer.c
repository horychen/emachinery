#include "ACMSim.h"
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11

struct InductionMachine im;

void acm_init(){

    im.us[0] = 0.0;
    im.us[1] = 0.0;
    im.is[0] = 0.0;
    im.is[1] = 0.0;
    im.us_curr[0] = 0.0;
    im.us_curr[1] = 0.0;
    im.is_curr[0] = 0.0;
    im.is_curr[1] = 0.0;
    im.us_prev[0] = 0.0;
    im.us_prev[1] = 0.0;
    im.is_prev[0] = 0.0;
    im.is_prev[1] = 0.0;

    // im.Js = 0.017876; // 3000rpm阶跃测转动惯量
    // im.Js = 0.172; 
    // im.Js = 0.072; 
    // im.Js = 0.017876*0.7;
    im.Js = MOTOR_SHAFT_INERTIA;
    im.Js_inv = 1.0/im.Js;
    im.npp = MOTOR_NUMBER_OF_POLE_PAIRS; // no. of pole pair
    im.npp_inv = 1.0/im.npp; // no. of pole pair
    im.mu_m = im.npp*im.Js_inv;
    im.mu   = im.npp*im.mu_m;
    // printf("im.mu_m = %g; im.mu = %g\n", im.mu_m, im.mu);

    im.Lmu = IM_MAGNETIZING_INDUCTANCE;
    im.Lmu_inv = 1.0/im.Lmu;
    im.Lsigma = IM_TOTAL_LEAKAGE_INDUCTANCE;
    im.Lsigma_inv = 1.0/im.Lsigma;

    im.Ls = im.Lsigma + im.Lmu;

    im.rs = IM_STAOTR_RESISTANCE;
    im.rreq = IM_ROTOR_RESISTANCE;

    im.alpha = im.rreq/im.Lmu;
    im.Tr = im.Lmu/im.rreq; 
    im.omg_elec = 0.0;
    im.omg_mech = 0.0;
}




// the fourth-order dynamic system by Marino2005@Wiley
void rhs_func_marino2005(double *increment_n, double xRho, double xTL, double xAlpha, double xOmg, double hs){
    // pointer to increment_n: state increment at n-th stage of RK4, where n=1,2,3,4.
    // *x???: pointer to state variables
    // x????: state variable
    // hs: step size of numerical integral

    /* 把磁链观测嫁到这里来？？*/
    /* 把磁链观测嫁到这里来？？*/
    /* 把磁链观测嫁到这里来？？*/

    // time-varying quantities
    CTRL.cosT = cos(xRho); // 这里体现了该观测器的非线性——让 q 轴电流给定和观测转速之间通过 iQs 的值产生了联系
    CTRL.sinT = sin(xRho); // 这里体现了该观测器的非线性——让 q 轴电流给定和观测转速之间通过 iQs 的值产生了联系
    CTRL.iDs = AB2M(IS(0), IS(1), CTRL.cosT, CTRL.sinT);
    CTRL.iQs = AB2T(IS(0), IS(1), CTRL.cosT, CTRL.sinT);

    // f = \dot x = the time derivative
    double f[4];
    // xRho
    f[0] = xOmg + xAlpha*im.Lmu*CTRL.iQs_cmd*CTRL.psi_cmd_inv;
    // xTL
    f[1] = - marino.gamma_inv * im.Js * CTRL.psi_cmd * marino.e_psi_Qmu;
    // xAlpha
    f[2] = marino.delta_inv * ( marino.e_psi_Dmu*(im.Lmu*CTRL.iDs_cmd - CTRL.psi_cmd) + marino.e_psi_Qmu*im.Lmu*CTRL.iQs_cmd);
    // xOmg
    REAL xTem = CLARKE_TRANS_TORQUE_GAIN*im.npp*( marino.psi_Dmu*CTRL.iQs - marino.psi_Qmu*CTRL.iDs );
    f[3] = im.npp*im.Js_inv*(xTem - xTL) + 2*marino.lambda_inv*CTRL.psi_cmd*marino.e_psi_Qmu;

    increment_n[0] = ( f[0] )*hs;
    increment_n[1] = ( f[1] )*hs;
    increment_n[2] = ( f[2] )*hs;
    increment_n[3] = ( f[3] )*hs;
}
void rK4(double hs){
    static double increment_1[4];
    static double increment_2[4];
    static double increment_3[4];
    static double increment_4[4];
    static double x_temp[4];
    static double *p_x_temp=x_temp;

    /* Theoritically speaking, rhs_func should be time-varing like rhs_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhs_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhs_func_marino2005( increment_1, marino.xRho, marino.xTL, marino.xAlpha, marino.xOmg, hs); 
    x_temp[0]  = marino.xRho   + increment_1[0]*0.5;
    x_temp[1]  = marino.xTL    + increment_1[1]*0.5;
    x_temp[2]  = marino.xAlpha + increment_1[2]*0.5;
    x_temp[3]  = marino.xOmg   + increment_1[3]*0.5;

    // time instant t+hs/2
    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    rhs_func_marino2005( increment_2, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    x_temp[0]  = marino.xRho   + increment_2[0]*0.5;
    x_temp[1]  = marino.xTL    + increment_2[1]*0.5;
    x_temp[2]  = marino.xAlpha + increment_2[2]*0.5;
    x_temp[3]  = marino.xOmg   + increment_2[3]*0.5;

    // time instant t+hs/2
    rhs_func_marino2005( increment_3, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    x_temp[0]  = marino.xRho   + increment_3[0];
    x_temp[1]  = marino.xTL    + increment_3[1];
    x_temp[2]  = marino.xAlpha + increment_3[2];
    x_temp[3]  = marino.xOmg   + increment_3[3];

    // time instant t+hs
    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    rhs_func_marino2005( increment_4, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    // \+=[^\n]*1\[(\d+)\][^\n]*2\[(\d+)\][^\n]*3\[(\d+)\][^\n]*4\[(\d+)\][^\n]*/ ([\d]+)
    // +=   (increment_1[$5] + 2*(increment_2[$5] + increment_3[$5]) + increment_4[$5])*0.166666666666667; // $5
    marino.xRho        += (increment_1[0] + 2*(increment_2[0] + increment_3[0]) + increment_4[0])*0.166666666666667; // 0
    marino.xTL         += (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667; // 1
    marino.xAlpha      += (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667; // 2
    marino.xOmg        += (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667; // 3

    // Also get derivatives:
    CTRL.omega_syn = marino.xOmg + marino.xAlpha*im.Lmu*CTRL.iQs_cmd*CTRL.psi_cmd_inv;
    marino.deriv_xTL    = (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667 * CL_TS_INVERSE;
    marino.deriv_xAlpha = (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667 * CL_TS_INVERSE;
    marino.deriv_xOmg   = (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667 * CL_TS_INVERSE;

    // Projection Algorithm
    // xRho   \in [-M_PI, M_PI]
    if(marino.xRho > M_PI){
        marino.xRho -= 2*M_PI;        
    }else if(marino.xRho < -M_PI){
        marino.xRho += 2*M_PI; // 反转！
    }
    // xTL    \in [-xTL_Max, xTL_Max]
    ;
    // xAlpha \in [-xAlpha_min, xAlpha_Max]
    if(marino.xAlpha > marino.xAlpha_Max){
        marino.xAlpha = marino.xAlpha_Max;
    }else if(marino.xAlpha < marino.xAlpha_min){
        marino.xAlpha = marino.xAlpha_min;
    }


}

void improved_Holtz_method(){
    // Flux Measurements
    holtz.psi_D2 = ACM.psi_Dmu;
    holtz.psi_Q2 = ACM.psi_Qmu;

    // Flux Estimator (Open Loop Voltage Model + ODE1)
    REAL deriv_psi_D1 = CTRL.uDs_cmd - CTRL.rs*CTRL.iDs + CTRL.omega_syn*holtz.psi_Q1_ode1;
    REAL deriv_psi_Q1 = CTRL.uQs_cmd - CTRL.rs*CTRL.iQs - CTRL.omega_syn*holtz.psi_D1_ode1;
    holtz.psi_D1_ode1 += CL_TS * (deriv_psi_D1);
    holtz.psi_Q1_ode1 += CL_TS * (deriv_psi_Q1);
    holtz.psi_D2_ode1 = holtz.psi_D1_ode1 - CTRL.Lsigma*CTRL.iDs;
    holtz.psi_Q2_ode1 = holtz.psi_Q1_ode1 - CTRL.Lsigma*CTRL.iQs;
}
void observer_marino2005(){

    /* OBSERVATION */
    rK4(1*CL_TS);

    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    //  2017年1月20日，将控制器放到了观测器的后面。
    // * 所以，上一步电压US_P的更新也要延后了。
    // US_P(0) = US_C(0); 
    // US_P(1) = US_C(1);
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}

#endif
