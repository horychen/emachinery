#include "ACMSim.h"
#if MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL || MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL

#define IM ACM

// 仿真电机结构体变量声明
struct InductionMachineSimulated ACM;

// 仿真电机结构体的初始化
void Machine_init(){
    int i;
    for(i=0;i<6;++i){
        IM.x[i] = 0.0;
        IM.x_dot[i] = 0.0;
    }
    // IM.x[4] = 20*RAD_PER_SEC_2_RPM;
    // #ifndef TURN_OFF_TDDA
    //     IM.x[9] = ob.tdda_Rs;
    //     IM.x[10] = ob.tdda_alpha;
    //     IM.x[11] = ob.tdda_omg;
    //     IM.x[12] = ob.tdda_Lsigma;
    // #endif
    IM.rpm = 0.0;

    IM.iqs = 0.0;
    IM.ids = 0.0;
    IM.iqr = 0.0;
    IM.idr = 0.0;
    IM.psimq = 0.0;
    IM.psimd = 0.0;

    IM.pis_curr[0] = 0.0;
    IM.pis_curr[1] = 0.0;
    IM.pis_prev[0] = 0.0;
    IM.pis_prev[1] = 0.0;

    IM.TLoad = 0.0;
    IM.rpm_cmd = 0.0;
    IM.rpm_deriv_cmd = 0.0;


    IM.Lmu    = IM_MAGNETIZING_INDUCTANCE;
    IM.Lmu_inv= 1.0/IM.Lmu;

    IM.Lsigma = IM_TOTAL_LEAKAGE_INDUCTANCE;
    IM.Lsigma_inv = 1.0/IM.Lsigma;

    IM.Lls = IM.Lsigma * 0.5;
    IM.Llr = IM.Lls;

    IM.Lm = IM.Lmu + IM.Lsigma - IM.Lls; // Lsigma = Ls * (1.0 - IM.Lm*IM.Lm/Ls/Lr);
    // IM.Lm     = 0.5*(IM.Lmu+sqrt(IM.Lmu*IM.Lmu+4*IM.Llr*IM.Lmu));
    // IM.Lmu = IM.Lm * IM.Lm / (IM.Lm + IM.Llr);

    IM.Lm_slash_Lr = IM.Lm/(IM.Lm+IM.Llr);
    IM.Lr_slash_Lm = (IM.Lm+IM.Llr)/IM.Lm;

    IM.rreq = IM_ROTOR_RESISTANCE;
    IM.alpha = IM.rreq/IM.Lmu;
    // IM.rreq  = IM.Lmu * IM.alpha;
    // IM.alpha  = IM.rr / (IM.Lm + IM.Llr);

    IM.rr     = IM.rreq*IM.Lr_slash_Lm*IM.Lr_slash_Lm;
    IM.rs     = IM_STAOTR_RESISTANCE;


    IM.LSigmal = 1.0 / (1.0 / IM.Lls + 1.0 / IM.Llr);
    #if NO_ROTOR_LEAKAGE
        IM.LSigmal = 0.0;
        IM.Llr= 0.0;
    #endif

    // IM.Js = (0.0636)*(1+); // Awaya92 using im.omg
    IM.Js = MOTOR_SHAFT_INERTIA * (1.0+LOAD_INERTIA);

    IM.npp = MOTOR_NUMBER_OF_POLE_PAIRS;
    IM.mu_m = IM.npp/IM.Js;

    IM.Ts  = MACHINE_TS;

    IM.ual = 0.0;
    IM.ube = 0.0;


    IM.ual_c_dist = 0.0;
    IM.ube_c_dist = 0.0;
    IM.dist_al = 0.0;
    IM.dist_be = 0.0;

    // IM.cur_offset = CURRENT_OFFSET; 
}

// 根据定、转子磁链计算定、转子电流
void collectCurrents(double *x){
    // Generalised Current by Therrien2013
    IM.izd = x[0]/IM.Lls + x[2]/IM.Llr;
    IM.izq = x[1]/IM.Lls + x[3]/IM.Llr;
    IM.iz = sqrt(IM.izd*IM.izd + IM.izq*IM.izq);

    // zero iz is not acceptable
    if(IM.iz>1e-8){
        #if SATURATED_MAGNETIC_CIRCUIT
            IM.psim = sat_lookup(IM.iz, satLUT);
            IM.im = IM.iz - IM.psim/IM.LSigmal;
            {
                IM.Lm = IM.psim/IM.im; // This is not very proper?
                /* Lm_anal = LmN * psim / im(psim) */
                // IM.Lm = 0.4144 * IM.psim/(5.0709*0.4144) \
                //                / (0.9 * IM.psim/(5.0709*0.4144) + (0.1)*powf(IM.psim/(5.0709*0.4144),7));
                IM.Lmu = IM.Lm*IM.Lm/(IM.Lm+IM.Llr);
                IM.alpha = IM.rr/(IM.Lm+IM.Llr);
                IM.rreq = IM.Lmu*IM.alpha;
                IM.Lsigma = (IM.Lls+IM.Lm) - IM.Lmu;            
            }
        #else
            IM.psim = 1.0/(1.0/IM.Lm+1.0/IM.Lls+1.0/IM.Llr)*IM.iz;
        #endif

        IM.psimd = IM.psim/IM.iz*IM.izd;        
        IM.psimq = IM.psim/IM.iz*IM.izq;
    }
    // else{
    //     // printf("how to handle zero iz?\n");
    //     // IM.psimq = IM.psimd = 0
    // }

    IM.ids = (x[0] - IM.psimd) / IM.Lls;
    IM.iqs = (x[1] - IM.psimq) / IM.Lls;
    IM.idr = (x[2] - IM.psimd) / IM.Llr;    
    IM.iqr = (x[3] - IM.psimq) / IM.Llr;

    // /* Direct compute is ir from psis psir */
    // IM.iqs = (x[1] - (IM.Lm/(IM.Lm+IM.Llr))*x[3])/IM.Lsigma;
    // IM.ids = (x[0] - (IM.Lm/(IM.Lm+IM.Llr))*x[2])/IM.Lsigma;
    // IM.iqr = (x[3] - (IM.Lm/(IM.Lm+IM.Lls))*x[1])/(IM.Lm+IM.Llr-IM.Lm*IM.Lm/(IM.Lm+IM.Lls));
    // IM.idr = (x[2] - (IM.Lm/(IM.Lm+IM.Lls))*x[0])/(IM.Lm+IM.Llr-IM.Lm*IM.Lm/(IM.Lm+IM.Lls));
}
// 感应电机的动态方程
void IM_saturated_Dynamics(double t, double *x, double *fx){
    #define IM ACM
    // 记电机的动态方程为 d/dt x = f(x)
    // 电机参数化选为反Γ等效电路。
    // 本函数内出现的 fx 均表示电机状态的时间导数。
    // 定义 αβ 系为定子静止坐标系，即α轴的方向和a相绕组相轴的方向保持一致。
    // 0：α轴定子磁链 [Wb]
    // 1：β轴定子磁链 [Wb]
    // 2：α轴转子磁链 [Wb]
    // 3：β轴转子磁链 [Wb]
    // 4：电气转子转速 [elec. rad/s]
    // 5：电气转子位置 [elec. rad]

    /* argument t is omitted but can be used for simulating slow varying parameters */

    /* STEP ZERO: collect all the currents: is, ir, iz */
    collectCurrents(x);

    /* STEP ONE: Inverter Nonlinearity - now it is voltages' turn */
    #if INVERTER_NONLINEARITY
        // InverterNonlinearity_SKSul96(IM.ual, IM.ube, IM.ids, IM.iqs);
        InverterNonlinearity_Tsuji01(IM.ual, IM.ube, IM.ids, IM.iqs);
        DIST_AL = UAL_C_DIST - IM.ual;
        DIST_BE = UBE_C_DIST - IM.ube;

        // Bolognani02 or GP&Bojoi10
        // Phase A current transformation
        // double magCurrent_inv = 1.0/sqrt(ial*ial+ibe*ibe);
        // inv._distD_atA = AB2M(DIST_AL, DIST_BE, ial*magCurrent_inv, ibe*magCurrent_inv);
        // inv._distQ_atA = AB2T(DIST_AL, DIST_BE, ial*magCurrent_inv, ibe*magCurrent_inv);

        // Phase A current's fundamental component transformation
        inv._distD_atA = AB2M(DIST_AL, DIST_BE, inv.cos_thetaA, inv.sin_thetaA);
        inv._distQ_atA = AB2T(DIST_AL, DIST_BE, inv.cos_thetaA, inv.sin_thetaA);
    #else
        UAL_C_DIST = IM.ual;
        UBE_C_DIST = IM.ube;  
    #endif

    // US_C(0) = UAL_C_DIST; // 拿示波器去测电压，记得关闭GAMMA_BIAS
    // US_C(1) = UBE_C_DIST;
    // US_P(0) = US_C(0); // 在观测器中，电压不分Previos和Current，就用上一步的电压给定值。
    // US_P(1) = US_C(1);

    /* STEP TWO: electromagnetic model with full flux states in alpha-beta frame */
    fx[0] = UAL_C_DIST - IM.rs*IM.ids; // d-axis
    fx[1] = UBE_C_DIST - IM.rs*IM.iqs; // q-axis
    fx[2] =            - IM.rr*IM.idr - x[4]*x[3]; // d-axis
    fx[3] =            - IM.rr*IM.iqr + x[4]*x[2]; // q-axis

    // deriv_psiStator2 = (US_C(0)_DIST - IM.rs*IM.ids); ///////////////////////////////////

    /* STEP THREE: mechanical model */
    // IM.Tem = IM.npp*(IM.Lm/(IM.Lm+IM.Llr))*(IM.iqs*x[2]-IM.ids*x[3]); // this is not better 
    IM.Tem = CLARKE_TRANS_TORQUE_GAIN * IM.npp * (IM.iqs*x[0]-IM.ids*x[1]);
    fx[4] = (IM.Tem - IM.TLoad)*IM.mu_m;
    fx[5] = x[4];
    fx[6] = x[4];
    #undef IM
}
void IM_linear_Dynamics(double t, double *x, double *fx){
    #define IM ACM
    // 记电机的动态方程为 d/dt x = f(x)
    // 本函数内出现的 fx 均表示电机状态的时间导数。
    // 电机参数化选为反Γ等效电路。
    // 定义 αβ 系为定子静止坐标系，即α轴的方向和a相绕组相轴的方向保持一致。
    // 0：α轴定子磁链 [Wb]
    // 1：β轴定子磁链 [Wb]
    // 2：α轴转子磁链 [Wb]
    // 3：β轴转子磁链 [Wb]
    // 4：电气转子转速 [elec. rad/s]
    // 5：电气转子位置 [elec. rad]

    /* argument t is omitted*/

    // ## STEP ONE: collect all the states
    // x[0] i_{\alpha s}
    // x[1] i_{\beta s}
    // x[2] psi_{\alpha \mu}
    // x[3] psi_{\beta \mu}

    // Inverter Nonlinearity
    #if INVERTER_NONLINEARITY
        printf("Not yet support inverter for linear dynamics.\n");
        getch();
    #else
        UAL_C_DIST = IM.ual;
        UBE_C_DIST = IM.ube;  
    #endif

    /* T-equivalent Circuit x[2] and x[3] are \psi_r rather than \psi_\mu 
        // ## STEP TWO: electromagnetic model
        fx[2] = IM.alpha*(IM.Lm*x[0] - x[2]) - x[4]*x[3];
        fx[3] = IM.alpha*(IM.Lm*x[1] - x[3]) + x[4]*x[2];
        fx[0] = (UAL_C_DIST - IM.rs*x[0] - IM.Lm_slash_Lr*fx[2])/IM.Lsigma;
        fx[1] = (UBE_C_DIST - IM.rs*x[1] - IM.Lm_slash_Lr*fx[3])/IM.Lsigma;
        IM.Tem = IM.Lm_slash_Lr*CLARKE_TRANS_TORQUE_GAIN*IM.npp*(x[1]*x[2]-x[0]*x[3]);
        // Or you can go with:
        // fx[2] = IM.Lm_slash_Lr*IM.rr*x[0] - IM.alpha*x[2] - x[4]*x[3];
        // fx[3] = IM.Lm_slash_Lr*IM.rr*x[1] - IM.alpha*x[3] + x[4]*x[2];
        // fx[0] = (UAL_C - IM.rs*x[0] - fx[2])/IM.Lsigma;
        // fx[1] = (UBE_C - IM.rs*x[1] - fx[3])/IM.Lsigma;
    */

    /* Inverse-Gamma-equivalent Circuit x[2] and x[3] are \psi_\mu */
    // ## STEP TWO: electromagnetic model
    fx[2] = IM.alpha*(IM.Lmu*x[0] - x[2]) - x[4]*x[3];
    fx[3] = IM.alpha*(IM.Lmu*x[1] - x[3]) + x[4]*x[2];
    fx[0] = (UAL_C_DIST - IM.rs*x[0] - fx[2])*IM.Lsigma_inv;
    fx[1] = (UBE_C_DIST - IM.rs*x[1] - fx[3])*IM.Lsigma_inv;
    IM.Tem = CLARKE_TRANS_TORQUE_GAIN*IM.npp*(x[1]*x[2]-x[0]*x[3]);

    // ## STEP THREE: mechanical model
    fx[4] = (IM.Tem - IM.TLoad)*IM.mu_m;
    fx[5] = x[4];
    fx[6] = x[4];
    #undef IM
}
// 四阶龙格库塔法
double one_over_six = 1.0/6.0;
void RK4(double t, double *x, double hs){
    #define NS NUMBER_OF_STATES

    double k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    double fx[NS];
    int i;

    MACHINE_DYNAMICS(t, x, fx); // timer.t,
    for(i=0;i<NS;++i){        
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }
    
    MACHINE_DYNAMICS(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }
    
    MACHINE_DYNAMICS(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }
    
    MACHINE_DYNAMICS(t, xk, fx); // timer.t+hs, 
    for(i=0;i<NS;++i){        
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;

        // derivatives
        ACM.x_dot[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six / hs; 
    }
    #undef NS
}



// 调用龙格库塔法求解电机状态，并为接口变量赋值
int machine_simulation(){

    // 调用龙格库塔法求解电机状态，电机状态为 ACM.x = [d轴电流，q轴电流，电气转子转速，电气转子位置]
    // 电机输入分别为 d轴电压 ACM.ud 和 q轴电压 ACM.ud。
    RK4(ACM.timebase, ACM.x, ACM.Ts);

    // 电机电流接口
    #if MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL
        ACM.ids = ACM.x[0];
        ACM.iqs = ACM.x[1];
    #elif MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL
        collectCurrents(ACM.x);
    #endif
    ACM.ial = ACM.ids;
    ACM.ibe = ACM.iqs;
    // get M-T frame quantities for fun
    ACM.theta_M = atan2(ACM.x[3], ACM.x[2]);
    ACM.cosT = cos(ACM.theta_M); 
    ACM.sinT = sin(ACM.theta_M);
    ACM.iMs = AB2M(ACM.ial, ACM.ibe, ACM.cosT, ACM.sinT);
    ACM.iTs = AB2T(ACM.ial, ACM.ibe, ACM.cosT, ACM.sinT);

    // 电机转速接口
    ACM.omg_elec = ACM.x[4]; // 电气转速 [elec. rad/s]
    ACM.rpm = ACM.x[4] * RAD_PER_SEC_2_RPM; // 机械转速 [mech. r/min]

    // 电机转子位置接口
    ACM.theta_d_accum = ACM.x[6];
    ACM.theta_d = ACM.x[5];
    // 转子（假想）d轴位置限幅
    if(ACM.theta_d > M_PI) ACM.theta_d -= 2*M_PI;
    if(ACM.theta_d < -M_PI) ACM.theta_d += 2*M_PI; // 反转！
    // 将状态变量x[3]和接口变量theta_d的值进行同步
    ACM.x[5] = ACM.theta_d;

    #if PC_SIMULATION
        // 电机磁链接口
        #if MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL || MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL
            ACM.psi_Dmu = AB2M(ACM.x[2], ACM.x[3], ACM.cosT, ACM.sinT);
            ACM.psi_Qmu = AB2T(ACM.x[2], ACM.x[3], ACM.cosT, ACM.sinT);
        #endif
    #endif

    // 简单的程序跑飞检测，比如电机转速无穷大则停止程序
    if(isNumber(ACM.rpm)){
        return FALSE;
    }else{
        printf("ACM.rpm is %g\n", ACM.rpm);
        return TRUE;        
    }
}

#endif
