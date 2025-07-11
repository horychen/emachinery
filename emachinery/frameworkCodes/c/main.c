#include "ACMSim.h"
struct MachineSimulated ACM;

int main(){            // 主函数
    CTRL  = &CTRL_1;    // 指针指向你想调用的电机控制器（最多有四台，仿真只需要仿真一台）
    debug = &debug_1;   // 同理上面的CTRL_1
    init_d_sim();      // do this only once here
    init_debug();      // do this only once here
    init_experiment(); // 控制器结构体初始化（同实验）
    init_Machine();    // 仿真电机初始化
    if(d_sim.user.verbose)print_info();      // 打印
    FILE *fw; fw = fopen(DATA_FILE_NAME, "w"); write_header_to_file(fw); // 声明文件，并将变量名写入文件
    clock_t begin, end; begin = clock(); // c代码执行计时
    int _, dfe_counter=0; // _ for the outer iteration // dfe_counter for down frequency execution （降频执行变量）
    for(_=0;_<d_sim.sim.NUMBER_OF_STEPS*d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;++_){
        ACM.timebase += ACM.Ts;
        _user_time_varying_parameters(); // 用戶：定义时变参数
        if(machine_simulation()){printf("\t[main.c] Break the loop.\n"); break;} // 每隔 MACHINE_TS 调用电机仿真代码一次
        if(++dfe_counter == d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD){ // 降频执行控制器代码（为了逼近连续系统或huo仿真PWM载波，电机仿真需要每执行100次或更多次才执行控制器代码一次）
            dfe_counter = 0; (*CTRL).timebase += CL_TS; // DSP中的时间
            measurement();             // 采样，包括DSP中的ADC采样等
            write_data_to_file(fw);    // 写数据到文件（没错就是要在测完以后，DSP内各个数字信号更新前将数据写入硬盘）
            #if WHO_IS_USER == USER_BEZIER
                _user_Bezier_printInfo(d_sim.user.BOOL_BEZIER_RUN_IN_MAIN);
                //TODO:等会写成一个函数，or把print写到其他地方去
                if (d_sim.user.BOOL_BEZIER_RUN_IN_MAIN == TRUE){
                    bezier_controller_run_in_main();
                }
            #endif
            main_switch((*debug).mode_select);
            if((*debug).mode_select != MODE_SELECT_GENERATOR){
                ACM.uAB[0] = (*CTRL).o->cmd_uAB_to_inverter[0]; //*sqrt(CLARKE_TRANS_TORQUE_GAIN); *AMPL2POW
                ACM.uAB[1] = (*CTRL).o->cmd_uAB_to_inverter[1]; //*sqrt(CLARKE_TRANS_TORQUE_GAIN); *AMPL2POW
            }
        }
        if(_%100000==0){
            printf("_=%d\n", _);
        }

    }
    end = clock(); if(d_sim.user.verbose)printf("\t[main.c] The simulation in C costs %g sec.\n", (REAL)(end - begin)/CLOCKS_PER_SEC);
    fclose(fw); // getch(); // system("cd ../../tools && python cplot.py"); system("pause"); // 调用python脚本绘图
    return 0;
}


void init_Machine(){ // 仿真电机结构体的初始化
    ACM.npp = d_sim.init.npp;
    ACM.npp_inv = 1.0/ACM.npp;
    ACM.IN  = d_sim.init.IN;
    // electrical parameters
    ACM.R   = d_sim.init.R * 1.0;
    ACM.Ld  = d_sim.init.Ld;
    ACM.Lq  = d_sim.init.Lq * 1.0;
    ACM.KE  = d_sim.init.KE * 1.0;
    ACM.KA = ACM.KE;
    ACM.Rreq  = d_sim.init.Rreq;
    // mechanical parameters
    ACM.Js  = d_sim.init.Js; // kg.m^2
    ACM.Js_inv = 1.0 / ACM.Js;
    // states
    ACM.NS = MACHINE_NUMBER_OF_STATES;
    int i;
    for(i=0;i<ACM.NS;++i){
        ACM.x[i] = 0.0;
        ACM.x_dot[i] = 0.0;
    }
    if(ACM.Rreq<=0){
        ACM.x[2] = ACM.KA;
    }
    // ACM.x[1] = 12000 * RPM_2_MECH_RAD_PER_SEC;
    // inputs
    ACM.uAB_dist[0] = 0.0;
    ACM.uAB_dist[1] = 0.0;
    ACM.uAB_inverter[0] = 0.0;
    ACM.uAB_inverter[1] = 0.0;
    ACM.uAB[0] = 0.0;
    ACM.uAB[1] = 0.0;
    ACM.uDQ[0] = 0.0;
    ACM.uDQ[1] = 0.0;
    ACM.TLoad = 0;
    // output
    ACM.varTheta = 0.0;
    ACM.varOmega = 0.0;
    ACM.omega_syn = 0.0;
    ACM.omega_slip = 0.0;
    ACM.theta_d = 0.0;
    ACM.iDQ[0] = 0.0;
    ACM.iDQ[1] = 0.0;
    ACM.iAB[0] = 0.0;
    ACM.iAB[1] = 0.0;
    ACM.iuvw[0] = 0.0;
    ACM.iuvw[1] = 0.0;
    ACM.iuvw[2] = 0.0;
    ACM.Tem = 0.0;
    ACM.cosT_delay_1p5omegaTs = cos(ACM.x[0]*ACM.npp);
    ACM.sinT_delay_1p5omegaTs = sin(ACM.x[0]*ACM.npp);
    ACM.cosT = cos(ACM.x[0]*ACM.npp);
    ACM.sinT = sin(ACM.x[0]*ACM.npp);
    // simulation settings
    ACM.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD;
    ACM.Ts = MACHINE_TS;
    ACM.current_theta = 0.0;
    ACM.voltage_theta = 0.0;
    ACM.powerfactor = 0.0;
}

void DYNAMICS_MACHINE(REAL t, REAL x[], REAL fx[]){

    // varTheta = x[0]
    // varOmega = x[1]
    // ACM.theta_d_elec = x[0]*ACM.npp
    // ACM.omega_r = x[1]*ACM.npp
    REAL KA = x[2];
    REAL iD = x[3];
    REAL iQ = x[4];
    if(KA == 0.0){
        ACM.omega_slip = 0.0;
    }else{
        ACM.omega_slip = ACM.Rreq * iQ / KA;
    }
    ACM.omega_syn  = x[1]*ACM.npp + ACM.omega_slip;

    // 电磁子系统 (KA, iD, iQ as x[2], x[3], x[4])
    if (ACM.Rreq > 0){
        // s KA
        fx[2] = ACM.Rreq*iD - ACM.Rreq / (ACM.Ld - ACM.Lq) * KA; // [Apply Park Transorm to (31b)]
        // s iD
        fx[3] = (ACM.uDQ[0] - ACM.R*iD + ACM.omega_syn*ACM.Lq*iQ - fx[2]) / ACM.Lq; // (6a)
    }else if (ACM.Rreq < 0){
        printf("ACM.Rreq is used to calculate slip so it must be zero for PMSM.");
    }else{
            // note fx[3] * ACM.Lq = ACM.uDQ[0] - ACM.R*iD + omega*ACM.Lq*iQ - fx[2]
            //  =>  fx[3] * ACM.Lq = ACM.uDQ[0] - ACM.R*iD + omega*ACM.Lq*iQ - (ACM.Ld - ACM.Lq) * fx[3] - 0.0
            //  =>  fx[3] * ACM.Ld = ACM.uDQ[0] - ACM.R*iD + omega*ACM.Lq*iQ
            //  =>  s iD
        // s iD
        fx[3] = (ACM.uDQ[0] - ACM.R*iD + ACM.omega_syn*ACM.Lq*iQ) / ACM.Ld;
        // s KA 
        fx[2] = (ACM.Ld - ACM.Lq) * fx[3] + 0.0;
    }
    // s iQ
    fx[4] = (ACM.uDQ[1] - ACM.R*iQ - ACM.omega_syn*ACM.Lq*iD - ACM.omega_syn * KA) / ACM.Lq;
    // printf("%g, %g, %g, %g,  |  %g, %g, %g, %g \n",
    //     ACM.timebase, ACM.uDQ[1], iQ, ACM.omega_syn, iD, ACM.omega_syn, KA, fx[4]
    // );

    // 机械子系统 (varTheta, varOmega as x[0], x[1])
    ACM.Tem = CLARKE_TRANS_TORQUE_GAIN * ACM.npp * KA * iQ; // 电磁转矩计算
    fx[0] = x[1] + ACM.omega_slip / ACM.npp; // mech. angular rotor position (accumulated)
    fx[1] = (ACM.Tem - ACM.TLoad) / ACM.Js;  // mech. angular rotor speed
}

void RK4(REAL t, REAL *x, REAL hs){ // 四阶龙格库塔法
    #define NS MACHINE_NUMBER_OF_STATES

    REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    REAL fx[NS];
    int i;

    DYNAMICS_MACHINE(t, x, fx); // timer.t,
    for(i=0;i<NS;++i){        
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }
    
    DYNAMICS_MACHINE(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }
    
    DYNAMICS_MACHINE(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }
    
    DYNAMICS_MACHINE(t, xk, fx); // timer.t+hs, 
    for(i=0;i<NS;++i){        
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;
        // derivatives
        ACM.x_dot[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six / hs; 
    }
    #undef NS
}

int machine_simulation(){

    inverter_model();
    if(FALSE){
        // TODO: 这里实际上缺一个延时环节
        ACM.cosT_delay_1p5omegaTs = cosf(ACM.theta_d - 1.5 * ACM.omega_syn *CL_TS);
        ACM.sinT_delay_1p5omegaTs = sinf(ACM.theta_d - 1.5 * ACM.omega_syn *CL_TS);
        ACM.uDQ[0] = AB2M(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT_delay_1p5omegaTs, ACM.sinT_delay_1p5omegaTs);
        ACM.uDQ[1] = AB2T(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT_delay_1p5omegaTs, ACM.sinT_delay_1p5omegaTs);
    }else{
        ACM.uDQ[0] = AB2M(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);
        ACM.uDQ[1] = AB2T(ACM.uAB_inverter[0], ACM.uAB_inverter[1], ACM.cosT, ACM.sinT);
    }

    // 数值积分
    RK4(ACM.timebase, ACM.x, ACM.Ts);

    // 电机转速接口
    ACM.varOmega = ACM.x[1]; // 电气转速 [elec. rad/s]

    // 电机转子位置接口
    // get M-T frame quantities for fun
    ACM.varTheta = ACM.x[0];
    ACM.theta_d = ACM.varTheta*ACM.npp;
    ACM.cosT = cos(ACM.theta_d);
    ACM.sinT = sin(ACM.theta_d);

    // 电机电流接口
    ACM.iDQ[0] = ACM.x[3];
    ACM.iDQ[1] = ACM.x[4];
    ACM.iAB[0] = MT2A(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);
    ACM.iAB[1] = MT2B(ACM.iDQ[0], ACM.iDQ[1], ACM.cosT, ACM.sinT);

    // 电机磁链接口
    ACM.KA = ACM.x[2];
    ACM.psi_AB[0] = ACM.KA*ACM.cosT; // AB2M(ACM.x[2], ACM.x[3], ACM.cosT, ACM.sinT);
    ACM.psi_AB[1] = ACM.KA*ACM.sinT; // AB2T(ACM.x[2], ACM.x[3], ACM.cosT, ACM.sinT);
    ACM.emf_AB[0] = ACM.x_dot[2]*ACM.cosT + ACM.KA*-sin(ACM.theta_d) * (ACM.npp*ACM.x_dot[0]);
    ACM.emf_AB[1] = ACM.x_dot[2]*ACM.sinT + ACM.KA* cos(ACM.theta_d) * (ACM.npp*ACM.x_dot[0]);

    // 转子（假想）d轴位置限幅
    while(ACM.theta_d > M_PI) ACM.theta_d  -= 2*M_PI;
    while(ACM.theta_d < -M_PI) ACM.theta_d += 2*M_PI;  // 反转！

    // 简单的程序跑飞检测，比如电机转速无穷大则停止程序
    if(isNumber(ACM.varOmega)){
        return FALSE;
    }else{
        printf("ACM.varOmega is %g\n", ACM.varOmega);
        return TRUE;
    }

    // 简单的程序跑飞检测，比如电机转速无穷大则停止程序
    if(isNumber(FE.CMDC.omega_elec)){
        return FALSE;
    }else{
        printf("ACM.varOmega is %g\n", ACM.varOmega);
        return TRUE;
    }
}

void measurement(){
    // 本函数每隔采样时间 CL_TS 执行一次 

    // 下面出现的US_C，IS_C等，都是全局的宏变量，方便在不同的.c文件内共享。

    // 电压测量
    (*CTRL).i->Vdc = d_sim.init.Vdc;
    #define CURRENT_OFFSET_A 0//0.1//0.2//0.05
    #define CURRENT_OFFSET_B 0//0.1//0.2//0.15
    (*CTRL).i->iAB[0] = ACM.iAB[0] + 1*CURRENT_OFFSET_A;
    (*CTRL).i->iAB[1] = ACM.iAB[1] + 1*CURRENT_OFFSET_B;

    // exact measurement of d-axis angle
    (*CTRL).i->theta_d_elec_previous = (*CTRL).i->theta_d_elec;
    (*CTRL).i->theta_d_elec = ACM.theta_d;
    (*CTRL).i->varTheta = ACM.varTheta;
    (*CTRL).i->varOmega = ACM.varOmega; // + 1.5 * sin(6 * (*CTRL).i->cmd_varOmega * d_sim.init.npp * CTRL->timebase);
    // 电流测量存在噪声，只会影响反馈控制，实际的扰动可能是逆变器引入的，见逆变器建模
    // power-invariant to amplitude-invariant via Clarke transformation???
    // REAL sqrt_2slash3 = sqrt(2.0/3.0);
    // REAL ia = sqrt_2slash3 * ACM.ial                              + 0*2*5e-2*RANDOM;
    // REAL ib = sqrt_2slash3 * (-0.5*ACM.ial + 0.5*sqrt(3)*ACM.ibe) + 0*2*5e-2*RANDOM;
    // REAL ic = sqrt_2slash3 * (-0.5*ACM.ial - 0.5*sqrt(3)*ACM.ibe) + 0*2*5e-2*RANDOM;
    // IS_C(0) = 2.0/3.0 * (ia - 0.5*ib - 0.5*ic);
    // IS_C(1) = 2.0/3.0 * 0.5*sqrt(3.0) * (ib - ic);

    // 转速和位置传感器建模
    // #ifdef USE_INCREMENTAL_ENCODER
    //     // sensors();
    //     // 将位置（-pi到+pi），这里除以的分母为2pi，再乘以电机编码器一圈的脉冲数，就可以将电机的机械角度位置转换为脉冲信号
    //     int64 ActualPosInCnt = (ACM.varTheta / (2*M_PI) * SYSTEM_QEP_PULSES_PER_REV); 
    //     // 判断ActualPosInCnt的正负
    //     int64 the_sign = sign_integer(ActualPosInCnt);
    //     // BUG：这里干嘛要两个the_sign相乘，这不永远是1了吗
    //     Uint32 QPOSCNT = the_sign * (the_sign * ActualPosInCnt) % (int64)SYSTEM_QEP_PULSES_PER_REV; //EQep1Regs.QPOSCNT;
    //     ENC.rpm          = PostionSpeedMeasurement_MovingAvergage(QPOSCNT, (*CTRL).enc);
    //     ENC.varOmega     = ENC.rpm * RPM_2_ELEC_RAD_PER_SEC; // 机械转速（单位：RPM）-> 电气角速度（单位：elec.rad/s)
    //     ENC.theta_d_elec = ENC.theta_d__state;
    //     (*CTRL).i->varOmega = ENC.varOmega;
    // #endif
}



void InverterNonlinearity_SKSul96(REAL ual, REAL ube, REAL ial, REAL ibe);

/* Inverter Model @ZJU (Valid for the case of 1/CL_TS = 4 kHz) */
// #define _Vce0  1.8 // V
// #define _Vd0   1.3 // V
// #define _Udc   300 // V
// #define _Toff  0.32e-6 // sec
// #define _Ton   0.15e-6 // sec
// #define _Tdead 3.30e-6 // sec
// #define _Tcomp 0.0*(_Tdead+_Ton-_Toff) //3.13e-6; //8e-6; // 过补偿 //3.13e-6; // 只补偿死区

/* Inverter Model @NTU (Valid for the case of 1/CL_TS = 10 kHz) */
#define _Vce0  1.8 // V
#define _Vd0   1.3 // V
#define _Udc   180 // V
#define _Toff  0.32e-6 // sec
#define _Ton   0.15e-6 // sec
#define _Tdead 5e-6 // sec
#define _Tcomp 0.0*(_Tdead+_Ton-_Toff) //3.13e-6; //8e-6; // 过补偿 //3.13e-6; // 只补偿死区


// 逆变器建模
void inverter_model(){

    // amplitude-invariant to power-invariant
    // 考虑控制器和电机所用Clarke变换不同导致的系数变化

    // 根据给定电压(*CTRL).o->cmd_uAB[0]和实际的电机电流ACM.ial，计算畸变的逆变器输出电压ACM.ual。
    #if __INVERTER_NONLINEARITY == 4
        InverterNonlinearity_ExperimentalLUT_Indexed(uAB_inverter, uAB, ACM.iAB);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
    #elif __INVERTER_NONLINEARITY == 3
        InverterNonlinearity_ExperimentalLUT( (*CTRL).o->cmd_uAB_to_inverter[0], \
                                            (*CTRL).o->cmd_uAB_to_inverter[1], \
                                            ACM.iAB[0], \
                                            ACM.iAB[1]);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - (*CTRL).o->cmd_uAB[0];
        // DIST_BE = ACM.ube - (*CTRL).o->cmd_uAB[1];
    #elif __INVERTER_NONLINEARITY == 2
        InverterNonlinearity_ExperimentalSigmoid( (*CTRL).o->cmd_uAB_to_inverter[0], \
                                            (*CTRL).o->cmd_uAB_to_inverter[1], \
                                            ACM.iAB[0], \
                                            ACM.iAB[1]);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
        printf("ACM.ual_c_dist = %g, ACM.ube_c_dist = %g\n", ACM.ual_c_dist, ACM.ube_c_dist);
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - (*CTRL).o->cmd_uAB[0];
        // DIST_BE = ACM.ube - (*CTRL).o->cmd_uAB[1];
    #elif __INVERTER_NONLINEARITY == 1
        InverterNonlinearity_SKSul96( (*CTRL).o->cmd_uAB_to_inverter[0], \
                                    (*CTRL).o->cmd_uAB_to_inverter[1], \
                                    ACM.ial, \
                                    ACM.ibe);
        ACM.uAB_inverter[0] = ACM.ual_c_dist;
        ACM.uAB_inverter[1] = ACM.ube_c_dist;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - (*CTRL).o->cmd_uAB[0];
        // DIST_BE = ACM.ube - (*CTRL).o->cmd_uAB[1];
    #else
        ACM.uAB_inverter[0] = ACM.uAB[0];
        ACM.uAB_inverter[1] = ACM.uAB[1];
    #endif
}

#if __INVERTER_NONLINEARITY != 0
    void InverterNonlinearity_ExperimentalLUT_Indexed(REAL uAB_inverter[2], REAL uAB[2], REAL iAB[2]){

        /* 查表法-逆变器建模 */
        get_distorted_voltage_via_LUT_indexed( iAB[0], iAB[1], ACM.uAB_dist );

        // 最终考虑了逆变器压降的电压 = 给定到电机的电压 减去 逆变器非线性电压
        ACM.ual_c_dist = uAB[0] - ACM.uAB_dist[0];
        ACM.ube_c_dist = uAB[1] - ACM.uAB_dist[1];
    }
    void InverterNonlinearity_ExperimentalLUT(REAL ual, REAL ube, REAL ial, REAL ibe){
        // #define LENGTH_OF_LUT 21 // 80 V?
        // REAL lut_current[LENGTH_OF_LUT] = {-4.19999, -3.77999, -3.36001, -2.94002, -2.51999, -2.10004, -1.68004, -1.26002, -0.840052, -0.419948, 5.88754e-06, 0.420032, 0.839998, 1.26003, 1.67998, 2.10009, 2.51996, 2.87326, 3.36001, 3.78002, 4.2};
        // REAL lut_voltage[LENGTH_OF_LUT] = {-5.20719, -5.2079, -5.18934, -5.15954, -5.11637, -5.04723, -4.93463, -4.76367, -4.42522, -3.46825, 0.317444, 3.75588, 4.55737, 4.87773, 5.04459, 5.15468, 5.22904, 5.33942, 5.25929, 5.28171, 5.30045};

        #define LENGTH_OF_LUT  40 // 180 V
        REAL lut_current[LENGTH_OF_LUT] = {-0.83999, -0.798, -0.756, -0.714, -0.672, -0.63001, -0.588, -0.546, -0.504, -0.462, -0.42, -0.37801, -0.336, -0.294, -0.25199, -0.21, -0.16799, -0.126, -0.084, -0.042, 0.04199, 0.084, 0.12601, 0.168, 0.20999, 0.25204, 0.294, 0.336, 0.378, 0.42001, 0.46201, 0.50401, 0.546, 0.588, 0.62999, 0.67199, 0.714, 0.75599, 0.798, 0.84};
        REAL lut_voltage[LENGTH_OF_LUT] = {-1.77117, -1.77931, -1.78735, -1.79405, -1.79993, -1.80485, -1.80874, -1.81138, -1.81242, -1.81034, -1.80851, -1.80191, -1.79168, -1.77531, -1.75139, -1.71197, -1.64914, -1.54365, -1.32004, -0.74209, 0.52448, 1.24959, 1.51527, 1.63594, 1.70546, 1.74656, 1.7773, 1.79556, 1.80696, 1.81291, 1.81772, 1.8183, 1.81841, 1.81407, 1.81123, 1.80695, 1.80056, 1.79348, 1.7859, 1.77881};

        REAL ualbe_dist[2];
        /* 查表法-逆变器建模 */
        get_distorted_voltage_via_LUT( ual, ube, ial, ibe, ualbe_dist, lut_voltage, lut_current, LENGTH_OF_LUT);

        // 我们把逆变器产生的电压视作压降，所以要从给定的电压中减去
        ACM.ual_c_dist = ual - ualbe_dist[0];
        ACM.ube_c_dist = ube - ualbe_dist[1];

        ACM.dist_al = ualbe_dist[0];
        ACM.dist_be = ualbe_dist[1];
    }
    void InverterNonlinearity_ExperimentalSigmoid(REAL ual, REAL ube, REAL ial, REAL ibe){

        REAL ualbe_dist[2];
        /* 拟合法-逆变器建模 */
        get_distorted_voltage_via_CurveFitting( ual, ube, ial, ibe, ualbe_dist);

        // 我们把逆变器产生的电压视作压降，所以要从给定的电压中减去
        ACM.ual_c_dist = ual - ualbe_dist[0];
        ACM.ube_c_dist = ube - ualbe_dist[1];

        ACM.dist_al = ualbe_dist[0];
        ACM.dist_be = ualbe_dist[1];
    }
    void InverterNonlinearity_SKSul96(REAL ual, REAL ube, REAL ial, REAL ibe){
        REAL ua,ub,uc;
        REAL ia,ib,ic;
        REAL Udist;
        REAL TM;
        REAL Rce=0.04958, Rdiode=0.05618;

        TM    = _Toff - _Ton - _Tdead + _Tcomp; // Sul1996
        Udist = (_Udc*TM*CL_TS_INVERSE - _Vce0 - _Vd0) / 6.0; // Udist = (_Udc*TM/1e-4 - _Vce0 - _Vd0) / 6.0;
        /* 我规定Udist为正值 */
        Udist = fabsf(Udist);
        // Udist = (_Udc*TM*TS_INVERSE) / 6.0;
        // Udist = 0.0;
        static int bool_printed = FALSE;
        if(bool_printed==FALSE){
            printf("\t[inverter.c] Vsat = %g V\n", INV.Vsat);
            bool_printed=TRUE;
        }

        ia = 1 * (       ial                              );
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3      * ibe );

        if(FALSE){
            /* compute in abc frame (in Amplitude Invariant Transformation) */
            ua = 1 * (       ual                              );
            ub = 1 * (-0.5 * ual - SIN_DASH_2PI_SLASH_3 * ube );
            uc = 1 * (-0.5 * ual - SIN_2PI_SLASH_3      * ube );
            REAL ua_dist, ub_dist, uc_dist;
            ua_dist = Udist * (2*sign(ia) - sign(ib) - sign(ic)) + 0 * 0.5*(Rce+Rdiode)*ia;
            ub_dist = Udist * (2*sign(ib) - sign(ic) - sign(ia)) + 0 * 0.5*(Rce+Rdiode)*ib;
            uc_dist = Udist * (2*sign(ic) - sign(ia) - sign(ib)) + 0 * 0.5*(Rce+Rdiode)*ic;
            ACM.dist_al = 2.0/3.0 *             (ua_dist - 0.5*ub_dist - 0.5*uc_dist);
            ACM.dist_be = 2.0/3.0 * sqrt(3)/2 * (              ub_dist -     uc_dist);
            // 我们把逆变器产生的电压视作压降，所以要从给定的电压中减去
            ua -= ua_dist;
            ub -= ub_dist;
            uc -= uc_dist;
            ACM.ual_c_dist = 2.0/3.0 *             (ua - 0.5*ub - 0.5*uc); // sqrt(2/3.)
            ACM.ube_c_dist = 2.0/3.0 * sqrt(3)/2 * (         ub -     uc); // sqrt(2/3.)*sin(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)            
        }else{
            /* directly compute in alpha-beta frame (Do note doing this injects a zero-sqeuence voltage!!!) */
            // CHECK the sign of the distortion voltage!
            ACM.dist_al =         Udist*(2*sign(ia) - sign(ib) - sign(ic)) + 0 * 0.5*(Rce+Rdiode)*ial;
            ACM.dist_be = sqrt(3)*Udist*(             sign(ib) - sign(ic)) + 0 * 0.5*(Rce+Rdiode)*ibe;
            // Sul把Udist视为补偿的电压（假定上升下降时间都已经知道了而且是“补偿”上去的）
            ACM.ual_c_dist = ual - ACM.dist_al;
            ACM.ube_c_dist = ube - ACM.dist_be;
        }
    }
#endif


