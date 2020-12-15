#include "ACMSim.h"

// 声明控制器结构体变量
struct ControllerForExperiment CTRL;
PID_REG pid1_id  = PID_REG_DEFAULTS;
PID_REG pid1_iq  = PID_REG_DEFAULTS;
PID_REG pid1_pos = PID_REG_DEFAULTS;
PID_REG pid1_spd = PID_REG_DEFAULTS;
PID_REG pid1_ia = PID_REG_DEFAULTS;
PID_REG pid1_ib = PID_REG_DEFAULTS;
PID_REG pid1_ic = PID_REG_DEFAULTS;

#define INCREMENTAL_PID FALSE
#if INCREMENTAL_PID
void PID_calc(PID_Reg *r){

    r->Err = r->Ref - r->Fdb;
    r->Out = r->OutPrev \
             + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki * r->Err;

    if(r->Out > r->OutLimit)
        r->Out = r->OutLimit;
    else if(r->Out < -r->OutLimit)
        r->Out = -r->OutLimit;

    r->ErrPrev = r->Err; 
    r->OutPrev = r->Out;
}
#else
void PID_calc(PID_REG *r){
    #define DYNAMIC_CLAPMING TRUE

    // 误差
    r->Err = r->Ref - r->Fdb;

    // 比例
    r->P_Term = r->Err * r->Kp;

    // 积分
    r->I_Term += r->Err * r->Ki;
    r->OutNonSat = r->I_Term;

    // 添加积分饱和特性
    #if DYNAMIC_CLAPMING
        // dynamic clamping
        if( r->I_Term > r->OutLimit - r->Out)
            r->I_Term = r->OutLimit - r->Out;
        else if( r->I_Term < -r->OutLimit - r->Out)
            r->I_Term = -r->OutLimit - r->Out;
    #else
        // static clamping
        if( r->I_Term > r->OutLimit)
            r->I_Term = r->OutLimit; 
        else if( r->I_Term < -r->OutLimit)
            r->I_Term = -r->OutLimit;
    #endif

    // 微分
    // r->D_Term = r->Kd * (r->Err - r->ErrPrev);

    // 输出
    r->Out = r->I_Term + r->P_Term; // + r->D_Term
    r->OutNonSat += r->P_Term; // + r->D_Term
    // 输出限幅
    if(r->Out > r->OutLimit)
        r->Out = r->OutLimit;
    else if(r->Out < -r->OutLimit)
        r->Out = -r->OutLimit;

    // 当前步误差赋值为上一步误差
    r->ErrPrev = r->Err;
    // 记录饱和输出和未饱和输出的差
    r->SatDiff = r->Out - r->OutNonSat;
}
#endif

// 初始化函数
void ACMSIMC_PIDTuner(){

    pid1_id.Kp = CURRENT_KP;
    pid1_iq.Kp = CURRENT_KP;
    pid1_ia.Kp = CURRENT_KP;
    pid1_ib.Kp = CURRENT_KP;
    pid1_ic.Kp = CURRENT_KP;

    pid1_id.Ki = CURRENT_KI_CODE;
    pid1_iq.Ki = CURRENT_KI_CODE;
    pid1_ia.Ki = CURRENT_KI_CODE;
    pid1_ib.Ki = CURRENT_KI_CODE;
    pid1_ic.Ki = CURRENT_KI_CODE;

    pid1_spd.Kp = SPEED_KP;
    pid1_spd.Ki = SPEED_KI_CODE;
}
