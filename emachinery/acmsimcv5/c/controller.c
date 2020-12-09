#include "ACMSim.h"

// 声明控制器结构体变量
struct ControllerForExperiment CTRL;
PIDREG3 pid1_id  = PIDREG3_DEFAULTS;
PIDREG3 pid1_iq  = PIDREG3_DEFAULTS;
PIDREG3 pid1_pos = PIDREG3_DEFAULTS;
PIDREG3 pid1_spd = PIDREG3_DEFAULTS;
PIDREG3 pid1_ia = PIDREG3_DEFAULTS;
PIDREG3 pid1_ib = PIDREG3_DEFAULTS;
PIDREG3 pid1_ic = PIDREG3_DEFAULTS;


void pid_reg3_calc(PIDREG3 *v){
    // Error
    v->Err = v->Ref - v ->Fdb;

    // Proportional regulator
    v->Up = v->Kp * v->Err;

    // Integral regulator with static clamping
    if(    ( v->SatStatus== 1 ) && (v->Err<0 ) \
        || ( v->SatStatus==-1 ) && (v->Err>0 ) \
        || ( v->SatStatus== 0)
      ){
        // Integral regulator
        v->Ui += v->Ki * v->Err;

        // Clamping
        if(v->Ui > v->OutMax) v->Ui = v->OutMax;
        if(v->Ui < v->OutMin) v->Ui = v->OutMin;
    }

    // Derivative regulator
    // v->Ud = v->Kd * (v->Up - v->Up1);

    // Saturation
    v->OutPreSat = v->Up + v->Ui;
    if( v->OutPreSat > v->OutMax ){
        v->Out = v->OutMax;
        v->Ui -= (v->OutPreSat - v->OutMax); // <- Dynamic campling
        v->SatStatus = 1;
    }else if( v->OutPreSat < v->OutMin ){
        v->Out = v->OutMin;
        v->Ui -= (v->OutPreSat - v->OutMin); // <- Dynamic campling
        v->SatStatus = -1;
    }else{
        v->Out = v->OutPreSat;
        v->SatStatus = 0;
    }

    // Staturion difference
    v->SatErr = v->Out - v->OutPreSat;

    // Update the previous proportional output
    v->Up1 = v->Up;
}

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
void CTRL_init(){

    if(SENSORLESS_CONTROL==TRUE){
        printf("Sensorless using observer.\n");
    }else{
        printf("Sensored control.\n");
    }
    printf("NUMBER_OF_STEPS: %d\n\n", NUMBER_OF_STEPS);


    int i=0,j=0;

    CTRL.timebase = 0.0;

    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    CTRL.R  = ACM.R;
    CTRL.KE = ACM.KE;
    CTRL.Ld = ACM.Ld;
    CTRL.Lq = ACM.Lq;

    // CTRL.Tload = 0.0;
    // CTRL.rpm_cmd = 0.0;

    CTRL.npp = ACM.npp;
    CTRL.Js = ACM.Js;
    CTRL.Js_inv = 1.0 / CTRL.Js;

    CTRL.omg__fb = 0.0;
    CTRL.ial__fb = 0.0;
    CTRL.ibe__fb = 0.0;
    CTRL.psi_mu_al__fb = 0.0;
    CTRL.psi_mu_be__fb = 0.0;

    CTRL.rotor_flux_cmd = 0.0; // id=0 control

    // CTRL.omg_ctrl_err = 0.0;
    // CTRL.speed_ctrl_err = 0.0;

    CTRL.cosT = 1.0;
    CTRL.sinT = 0.0;

    CTRL.omega_syn = 0.0;

    CTRL.theta_d__fb = 0.0;
    CTRL.id__fb = 0.0;
    CTRL.iq__fb = 0.0;
    CTRL.ud_cmd = 0.0;
    CTRL.uq_cmd = 0.0;
    CTRL.id_cmd = 0.0;
    CTRL.iq_cmd = 0.0;

    CTRL.Tem = 0.0;
    CTRL.Tem_cmd = 0.0;

    // 转速PID结构体
    pid1_spd.Kp = 0; //SPEED_LOOP_PID_PROPORTIONAL_GAIN;
    pid1_spd.Ki = 0; //pid1_spd.Kp / SPEED_LOOP_PID_INTEGRAL_TIME_CONSTANT * (VL_TS); // 4.77 = 1 / (npp*1/60*2*pi)
    pid1_spd.OutMax = SPEED_LOOP_LIMIT_AMPERE;
    pid1_spd.OutMin = -SPEED_LOOP_LIMIT_AMPERE;

    // 电流PID结构体
    pid1_id.Kp = 0; //CURRENT_LOOP_PID_PROPORTIONAL_GAIN;
    pid1_id.Ki = 0; //pid1_id.Kp/CURRENT_LOOP_PID_INTEGRAL_TIME_CONSTANT*CL_TS; 
    pid1_id.OutMax = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_id.OutMin = -CURRENT_LOOP_LIMIT_VOLTS;
    // 电流PID结构体
    pid1_iq.Kp = 0; //CURRENT_LOOP_PID_PROPORTIONAL_GAIN;
    pid1_iq.Ki = 0; //pid1_iq.Kp/CURRENT_LOOP_PID_INTEGRAL_TIME_CONSTANT*CL_TS;
    pid1_iq.OutMax = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_iq.OutMin = -CURRENT_LOOP_LIMIT_VOLTS;

    // 电流PID结构体
    pid1_ia.Kp = 0;
    pid1_ia.Ki = 0;
    pid1_ia.OutMax = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_ia.OutMin = -CURRENT_LOOP_LIMIT_VOLTS;
    // 电流PID结构体
    pid1_ib.Kp = 0;
    pid1_ib.Ki = 0;
    pid1_ib.OutMax = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_ib.OutMin = -CURRENT_LOOP_LIMIT_VOLTS;
    // 电流PID结构体
    pid1_ic.Kp = 0;
    pid1_ic.Ki = 0;
    pid1_ic.OutMax = CURRENT_LOOP_LIMIT_VOLTS;
    pid1_ic.OutMin = -CURRENT_LOOP_LIMIT_VOLTS;

    // 调谐
    ACMSIMC_PIDTuner();

    printf("Speed PID: Kp=%g, Ki=%g, limit=%g Nm\n", pid1_spd.Kp, pid1_spd.Ki/CL_TS, pid1_spd.OutMin);
    printf("Current PID: Kp=%g, Ki=%g, limit=%g V\n", pid1_id.Kp, pid1_id.Ki/CL_TS, pid1_id.OutMin);
}

// 控制器：输入为电机的转速指令，根据反馈的电流、转速、位置信息，计算电机的电压指令。
void control(double speed_cmd, double speed_cmd_dot){
    // Input 1 is feedback: estimated speed/position or measured speed/position
    #if SENSORLESS_CONTROL
        harnefors_scvm();
        CTRL.omg__fb     = omg_harnefors;
        CTRL.theta_d__fb = theta_d_harnefors;
    #else
        // from measurement() in main.c
        CTRL.omg__fb     = sm.omg_elec;
        CTRL.theta_d__fb = sm.theta_d; 
    #endif

    // Input 2 is feedback: measured current 
    CTRL.ial__fb = IS_C(0);
    CTRL.ibe__fb = IS_C(1);

    // Input 3 is the flux linkage command 
    #if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
        CTRL.rotor_flux_cmd = 0.0;
        CTRL.cosT = cos(CTRL.theta_d__fb); 
        CTRL.sinT = sin(CTRL.theta_d__fb);
    #else
        getch("Not Implemented");
    #endif

    // d-axis current command
    CTRL.id_cmd = CTRL.rotor_flux_cmd / CTRL.Ld;

    // q-axis current command
    static int vc_count = 0;
    if(vc_count++ == SPEED_LOOP_CEILING){
        // velocity control loop execution frequency is 40 times slower than current control loop execution frequency
        vc_count = 0;

        pid1_spd.Ref = speed_cmd*RPM_2_RAD_PER_SEC;
        pid1_spd.Fdb = CTRL.omg__fb;
        pid1_spd.calc(&pid1_spd);
        CTRL.iq_cmd = pid1_spd.Out;

        // for plot
        // CTRL.speed_ctrl_err = CTRL.omg_ctrl_err * RAD_PER_SEC_2_RPM;
    }

    // Measured current in d-q frame
    CTRL.id__fb = AB2M(CTRL.ial__fb, CTRL.ibe__fb, CTRL.cosT, CTRL.sinT);
    CTRL.iq__fb = AB2T(CTRL.ial__fb, CTRL.ibe__fb, CTRL.cosT, CTRL.sinT);

    // For luenberger position observer for HFSI
    CTRL.Tem     = CTRL.npp * (CTRL.KE*CTRL.iq__fb + (CTRL.Ld-CTRL.Lq)*CTRL.id__fb*CTRL.iq__fb);
    CTRL.Tem_cmd = CTRL.npp * (CTRL.KE*CTRL.iq_cmd + (CTRL.Ld-CTRL.Lq)*CTRL.id_cmd*CTRL.iq_cmd);

    // Voltage command in d-q frame
    double vd, vq;
    pid1_id.Fdb = CTRL.id__fb;
    pid1_id.Ref = CTRL.id_cmd;
    pid1_id.calc(&pid1_id);
    vd = pid1_id.Out;
    pid1_iq.Fdb = CTRL.iq__fb;
    pid1_iq.Ref = CTRL.iq_cmd;
    pid1_iq.calc(&pid1_iq);
    vq = pid1_iq.Out;

    // Current loop decoupling (skipped for now)
    CTRL.ud_cmd = vd;
    CTRL.uq_cmd = vq;

    // Voltage command in alpha-beta frame
    CTRL.ual = MT2A(CTRL.ud_cmd, CTRL.uq_cmd, CTRL.cosT, CTRL.sinT);
    CTRL.ube = MT2B(CTRL.ud_cmd, CTRL.uq_cmd, CTRL.cosT, CTRL.sinT);
}


// 定义特定的测试指令，如快速反转等
void cmd_fast_speed_reversal(double timebase, double instant, double interval, double rpm_cmd){
    if(timebase > instant+2*interval){
        ACM.rpm_cmd = 1*1500 + rpm_cmd;
    }else if(timebase > instant+interval){
        ACM.rpm_cmd = 1*1500 + -rpm_cmd;
    }else if(timebase > instant){
        ACM.rpm_cmd = 1*1500 + rpm_cmd;
    }else{
        ACM.rpm_cmd = 20; // default initial command
    }
}


struct SweepFreq sf={0.0, 1, SWEEP_FREQ_INIT_FREQ-1, 0.0, 0.0};

void controller(){

    // 位置环
    REAL position_command = 10*2;
    if(CTRL.timebase>5){
        position_command = -10*2; 
    }
    REAL position_error = position_command - ACM.theta_d_accum;
    REAL position_KP = 8;
    REAL rad_speed_command = position_KP*position_error;
    REAL rpm_speed_command = rad_speed_command*RAD_PER_SEC_2_RPM;
    REAL amp_current_command;


    #if EXCITATION_TYPE == 2
        // 扫频建模
        // #define SWEEP_FREQ_AMPL 500 // rpm
        // #define MAX_FREQ SWEEP_FREQ_MAX_FREQ // Hz
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
            // amp_current_command = (0.05 * PMSM_RATED_CURRENT_RMS*1.414) * sin(2*M_PI*sf.current_freq*(sf.time - sf.last_current_freq_end_time));
            amp_current_command = SWEEP_FREQ_CURRENT_AMPL * sin(2*M_PI*sf.current_freq*(sf.time - sf.last_current_freq_end_time));
        }
    #endif

    ACM.rpm_cmd = rpm_speed_command; // for plot


    // 转速运动模式
    // rpm_speed_command = 500*sin(2*M_PI*88*CTRL.timebase); // overwrite

    //（霍尔反馈）
    // CTRL.omg__fb     = sm.omg_elec_hall;
    // CTRL.theta_d__fb = sm.theta_d_hall;
    
    //（编码器反馈）
    // CTRL.omg__fb     = qep.omg_elec;
    // CTRL.theta_d__fb = qep.theta_d;

    //（实际反馈，实验中不可能）
    CTRL.omg__fb     = sm.omg_elec;
    CTRL.theta_d__fb = sm.theta_d;
    
    //（无感）
    // harnefors_scvm();
    // CTRL.omg__fb     = omg_harnefors;
    // CTRL.theta_d__fb = theta_d_harnefors;

    CTRL.cosT = cos(CTRL.theta_d__fb);
    CTRL.sinT = sin(CTRL.theta_d__fb);
    REAL id_fb = AB2M(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT);
    REAL iq_fb = AB2T(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT);
    REAL omg_elec_fb = CTRL.omg__fb;

    REAL error = rpm_speed_command*RPM_2_RAD_PER_SEC - omg_elec_fb;
    CTRL.speed_ctrl_err = error;

    // Current Commands
    static int vc_count = 0;
    if(vc_count++ == SPEED_LOOP_CEILING){
        vc_count = 0;

        pid1_spd.Ref = rpm_speed_command*RPM_2_RAD_PER_SEC;
        pid1_spd.Fdb = omg_elec_fb;
        pid1_spd.calc(&pid1_spd);
        pid1_iq.Ref = pid1_spd.Out;
    }

    #if SWEEP_FREQ_C2V == TRUE
        pid1_iq.Ref = amp_current_command; 
    #endif
    #if SWEEP_FREQ_C2C == TRUE
        pid1_iq.Ref = 0.0;
        pid1_id.Ref = amp_current_command;
    #else
        pid1_id.Ref = 0.0;
    #endif 


    pid1_id.Fdb = id_fb;
    pid1_iq.Fdb = iq_fb;

    // voltage output
    pid1_id.calc(&pid1_id);
    pid1_iq.calc(&pid1_iq);
    REAL decoupled_d_axis_voltage = pid1_id.Out - pid1_iq.Fdb*CTRL.Lq*CTRL.omg__fb;
    REAL decoupled_q_axis_voltage = pid1_iq.Out + (pid1_id.Fdb*CTRL.Ld+CTRL.KE)*CTRL.omg__fb;

    CTRL.ual = MT2A(decoupled_d_axis_voltage, decoupled_q_axis_voltage, CTRL.cosT, CTRL.sinT);
    CTRL.ube = MT2B(decoupled_d_axis_voltage, decoupled_q_axis_voltage, CTRL.cosT, CTRL.sinT);

    // for harnefors observer
    CTRL.ud_cmd = pid1_id.Out;
    CTRL.uq_cmd = pid1_iq.Out;
    CTRL.id_cmd = pid1_id.Ref;
    CTRL.iq_cmd = pid1_iq.Ref;
}

