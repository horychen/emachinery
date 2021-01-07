#include "ACMSim.h"


#define QEP_DEFAULTS {0,0,0,0,{0,0,0,0},0,0}
struct eQEP{
    double theta_d;
    double theta_d_prev;
    double omg_mech;
    double omg_elec;
    double moving_average[4];
    Uint32 ActualPosInCnt;
    Uint32 ActualPosInCnt_Previous;
} qep=QEP_DEFAULTS;
void measurement(){
    // 本函数每隔采样时间 CL_TS 执行一次

    // 下面出现的US_C，IS_C等，都是全局的宏变量，方便在不同的.c文件内共享。

    // 电压测量
    US_C(0) = CTRL.ual_cmd; // 后缀_C表示当前步的电压，C = Current
    US_C(1) = CTRL.ube_cmd;
    US_P(0) = US_C(0); // 后缀_P表示上一步的电压，P = Previous
    US_P(1) = US_C(1);

    // 电流测量存在噪声，只会影响反馈控制，实际的扰动可能是逆变器引入的，见逆变器建模
        // power-invariant to amplitude-invariant via Clarke transformation???
    // REAL sqrt_2slash3 = sqrt(2.0/3.0);
    // REAL ia = sqrt_2slash3 * ACM.ial                              + 0*2*5e-2*RANDOM;
    // REAL ib = sqrt_2slash3 * (-0.5*ACM.ial + 0.5*sqrt(3)*ACM.ibe) + 0*2*5e-2*RANDOM;
    // REAL ic = sqrt_2slash3 * (-0.5*ACM.ial - 0.5*sqrt(3)*ACM.ibe) + 0*2*5e-2*RANDOM;
    // IS_C(0) = 2.0/3.0 * (ia - 0.5*ib - 0.5*ic);
    // IS_C(1) = 2.0/3.0 * 0.5*sqrt(3.0) * (ib - ic);
    IS_C(0)      = ACM.ial;
    IS_C(1)      = ACM.ibe;
    CTRL.ial__fb = ACM.ial;
    CTRL.ibe__fb = ACM.ibe;

    // 转速和位置测量
    #define USE_EQEP FALSE
    #define USE_HALL FALSE
    #if USE_EQEP == TRUE
        #define TOTAL_PULSE_PER_REV          1e4 // 2500 line per rev + eQEP
        #define TOTAL_PULSE_PER_REV_INVERSE 1e-4 // 2500 line per rev + eQEP
        #define QEP_SPEED_ERROR_RPM (60.0 / (TOTAL_PULSE_PER_REV * VL_TS))

        // 这里的建模没有考虑初始d轴的偏置角！

        反转的时候qep的转速好像是错的！

        static int vc_count = 0;
        if(vc_count++ == SPEED_LOOP_CEILING){
            vc_count = 0;

            qep.ActualPosInCnt_Previous = qep.ActualPosInCnt;

            REAL rotor_position_mech_rad = ACM.x[4]*ACM.npp_inv;
            qep.ActualPosInCnt = (Uint32)( rotor_position_mech_rad / (2*M_PI) * TOTAL_PULSE_PER_REV);
            qep.theta_d  = (sm.npp*qep.ActualPosInCnt*TOTAL_PULSE_PER_REV_INVERSE)*2*M_PI;

            qep.moving_average[3] = qep.moving_average[2];
            qep.moving_average[2] = qep.moving_average[1];
            qep.moving_average[1] = qep.moving_average[0];
            qep.moving_average[0] = 60*(qep.ActualPosInCnt - qep.ActualPosInCnt_Previous) / (TOTAL_PULSE_PER_REV * VL_TS); // [rpm]
            // qep.omg_mech = 0.25*(qep.moving_average[3]+qep.moving_average[2]+qep.moving_average[1]+qep.moving_average[0]);
            
            qep.omg_elec = qep.moving_average[0] * RPM_2_RAD_PER_SEC;
            qep.omg_mech = qep.omg_elec * sm.npp_inv;
        }
    #endif
    #if USE_HALL == TRUE

        sm.theta_d  = ACM.theta_d; // + 30.0/180*M_PI;
        sm.omg_elec = ACM.omg_elec;
        sm.omg_mech = sm.omg_elec * sm.npp_inv;

        // 霍尔传感器检测
        sm.hallABC = hall_sensor(ACM.theta_d/M_PI*180.0);
        hall_resolver(sm.hallABC, &sm.theta_d_hall, &sm.omg_elec_hall);

        // 测量转子d轴位置限幅
        if(sm.theta_d_hall > M_PI){
            sm.theta_d_hall -= 2*M_PI;
        }else if(sm.theta_d_hall < -M_PI){
            sm.theta_d_hall += 2*M_PI;
        }

        // // if(CTRL.timebase>19){
        //     sm.theta_d  = sm.theta_d_hall;
        //     sm.omg_elec = sm.omg_elec_hall;
        // // }
    #endif

    #if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE
    // sm.theta_d  = ACM.x[3]; // + 30.0/180*M_PI;
    // sm.omg_elec = ACM.x[2];
    sm.theta_d  = ACM.theta_d; // + 30.0/180*M_PI;
    sm.omg_elec = ACM.omg_elec;
    sm.omg_mech = sm.omg_elec * sm.npp_inv;
    #endif

    #if MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL || MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL
    im.omg_elec = ACM.omg_elec;
    im.omg_mech = im.omg_elec * im.npp_inv;    
    #endif
}