#include "ACMSim.h"
#if MACHINE_TYPE == 2

// 初始化函数
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

    // PID调谐
    ACMSIMC_PIDTuner();
    printf("Speed PID: Kp=%g, Ki=%g, limit=%g Nm\n", pid1_spd.Kp, pid1_spd.Ki/CL_TS, pid1_spd.OutLimit);
    printf("Current PID: Kp=%g, Ki=%g, limit=%g V\n", pid1_id.Kp, pid1_id.Ki/CL_TS, pid1_id.OutLimit);
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
    REAL rpm_speed_command;

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

    #if EXCITATION_TYPE == 2
        // 扫频建模
        REAL amp_current_command;
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

    #if EXCITATION_TYPE == 1
        // 转速运动模式 in rpm
        rpm_speed_command = 500;
        if(CTRL.timebase>5){
            rpm_speed_command = -500;
        }
    #endif

    // for plot
    ACM.rpm_cmd = rpm_speed_command;



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

    // for plot
    CTRL.speed_ctrl_err = rpm_speed_command*RPM_2_RAD_PER_SEC - omg_elec_fb;

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

#endif