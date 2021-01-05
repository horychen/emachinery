#include "ACMSim.h"

// 声明控制器结构体变量
struct ControllerForExperiment CTRL;
PID_REG pid1_iM  = PID_REG_DEFAULTS;
PID_REG pid1_iT  = PID_REG_DEFAULTS;
PID_REG pid1_pos = PID_REG_DEFAULTS;
PID_REG pid1_spd = PID_REG_DEFAULTS;
PID_REG pid1_ia = PID_REG_DEFAULTS;
PID_REG pid1_ib = PID_REG_DEFAULTS;
PID_REG pid1_ic = PID_REG_DEFAULTS;

#define INCREMENTAL_PID FALSE
#if INCREMENTAL_PID
void PID_calc(PID_Reg *r){

    r->Err = r->Ref - r->Fbk;
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
    r->Err = r->Ref - r->Fbk;

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

    pid1_iM.Kp = CURRENT_KP;
    pid1_iT.Kp = CURRENT_KP;
    pid1_ia.Kp = CURRENT_KP;
    pid1_ib.Kp = CURRENT_KP;
    pid1_ic.Kp = CURRENT_KP;

    pid1_iM.Ki = CURRENT_KI_CODE;
    pid1_iT.Ki = CURRENT_KI_CODE;
    pid1_ia.Ki = CURRENT_KI_CODE;
    pid1_ib.Ki = CURRENT_KI_CODE;
    pid1_ic.Ki = CURRENT_KI_CODE;

    pid1_spd.Kp = SPEED_KP;
    pid1_spd.Ki = SPEED_KI_CODE;

    pid1_spd.OutLimit = SPEED_LOOP_LIMIT_AMPERE;
    pid1_iM.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_iT.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_ia.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_ib.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_ic.OutLimit = CURRENT_LOOP_LIMIT_VOLTS;
}

struct SweepFreq sf={0.0, 1, SWEEP_FREQ_INIT_FREQ-1, 0.0, 0.0};

void commands(REAL *p_rpm_speed_command, REAL *p_amp_current_command){
    #define rpm_speed_command (*p_rpm_speed_command)
    #define amp_current_command (*p_amp_current_command)

    // 位置环 in rad
    #if EXCITATION_TYPE == 0
        REAL position_command = 10*M_PI;
        if(CTRL.timebase>5){
            position_command = -10*M_PI;
        }
        REAL position_error = position_command - ACM.theta_d_accum;
        REAL position_KP = 8;
        REAL rad_speed_command = position_KP*position_error;
        rpm_speed_command = rad_speed_command*RAD_PER_SEC_2_RPM;
    #endif

    // 扫频建模
    #if EXCITATION_TYPE == 2
        // REAL amp_current_command;
        sf.time += CL_TS;
        if(sf.time > sf.current_freq_end_time){
            // next frequency
            sf.current_freq += sf.freq_step_size;
            // next end time
            sf.last_current_freq_end_time = sf.current_freq_end_time;
            sf.current_freq_end_time += 1.0/sf.current_freq; // 1.0 Duration for each frequency
        }
        if(sf.current_freq > SWEEP_FREQ_MAX_FREQ){
            rpm_speed_command = 0.0;
            amp_current_command = 0.0;
        }else{
            // # closed-cloop sweep
            rpm_speed_command   = SWEEP_FREQ_VELOCITY_AMPL * sin(2*M_PI*sf.current_freq*(sf.time - sf.last_current_freq_end_time));

            // open-loop sweep
            amp_current_command = SWEEP_FREQ_CURRENT_AMPL * sin(2*M_PI*sf.current_freq*(sf.time - sf.last_current_freq_end_time));
        }
    #endif

    // 转速运动模式 in rpm
    #if EXCITATION_TYPE == 1
        #define RPM1 100
        if(CTRL.timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
            rpm_speed_command = 0;
        }else if(CTRL.timebase<2){
            rpm_speed_command = RPM1;
        }else if(CTRL.timebase<4){
            rpm_speed_command = -RPM1;
        }else if(CTRL.timebase<6){
            rpm_speed_command = 0;
        }else if(CTRL.timebase<8){
            rpm_speed_command = RPM1;
        }
    #endif
}