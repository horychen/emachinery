#include "ACMSim.h"
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11
struct ControllerForExperiment CTRL;


#define IM ACM // 权宜之计

void CTRL_init(){
    int i=0,j=0;

    CTRL.timebase = 0.0;

        /* Parameter (including speed) Adaptation */ 
        CTRL.rs     = IM.rs;
        CTRL.rreq   = IM.rreq;
        CTRL.Lsigma = IM.Lsigma;
        CTRL.alpha  = IM.alpha;
        CTRL.Lmu    = IM.Leq;
        CTRL.Lmu_inv = 1.0/IM.Leq;
        CTRL.Js     = IM.Js;
        CTRL.Js_inv = 1.0/IM.Js;    

    CTRL.iMs = 0.0;
    CTRL.iTs = 0.0;
    CTRL.iMs_cmd = 0.0;
    CTRL.iTs_cmd = 0.0;

    CTRL.torque_cmd = 0.0;

    CTRL.uMs_cmd = 0.0;
    CTRL.uTs_cmd = 0.0;
    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    // CTRL.torque_integralPart = 0.0;
    // CTRL.torque_integralLimit = 200;

    // CTRL.uMs_integralPart = 0.0;
    // CTRL.uMs_integralLimit = 500;
    // CTRL.uTs_integralPart = 0.0;
    // CTRL.uTs_integralLimit = 800;

    CTRL.theta_M = 0.0;
    CTRL.cosT = 1.0;
    CTRL.sinT = 0.0;

    CTRL.e_M = 0.0;
    CTRL.e_T = 0.0;



    CTRL.omega_sl = 0.0;
    CTRL.omega_syn = 0.0;

    CTRL.tajima_omg = 0.0;
    CTRL.K_PEM = 0.1;
    
    CTRL.omg__fb = 0.0;
    CTRL.ial__fb = 0.0;
    CTRL.ibe__fb = 0.0;

    CTRL.psial_fb = 0.0;
    CTRL.psibe_fb = 0.0;
    CTRL.psimod_fb = 0.0;
    CTRL.psimod_fb_inv = 1.0;
    CTRL.deriv_fluxModSim = 0.0;




    CTRL.omg_ctrl_err = 0.0;
    CTRL.gamma_res_transient = 0;
    CTRL.gamma_res_transient_shape = 2000; //2000~5000都不错
    CTRL.gamma_res2_transient = 0;
    CTRL.gamma_res2_transient_shape = 2; // 10
    CTRL.gamma_omg_transient = 0;
    CTRL.gamma_omg_transient_shape = 1; // 1 数值越小，那么区域越宽，避免转速稳态下转速辨识增益振荡。
    CTRL.resistance_id_on = FALSE;




    CTRL.sensorless = SENSORLESS_CONTROL;
    CTRL.CtrMode = CONTROL_STRATEGY;





    CTRL.FSET = 0.0;        // VVVF: Frequency Set(Hz), others: rpm_cmd
    CTRL.Goal = 0.0;        // 斜坡转速给定的值
    CTRL.Goal_dot = 0.0;        // 斜坡转速给定的值
    CTRL.FCUR = 0.5;        // Current Frequent (Hz)
    CTRL.Speed = 0.0;       // 转速(rpm)
    CTRL.SpStart = 0;       // Boolean for starting
    CTRL.SpCount = 0.0;

    CTRL.VCUR = 0.0;           // Current Voltag
    CTRL.VCurPerUnit = 0.0;    // 2*Current Voltag/Udc
    CTRL.Cvf0 = 3;//6.22;          // VVVF曲线，Cvf0=最大输出电压幅值/转折频率=150/50;

    CTRL.Udc = 0.0;
    CTRL.lUdc = 0.0;

    CTRL.VaPU = 0.0;               // 标幺化voltage in a-b-c frame
    CTRL.VbPU = 0.0;
    CTRL.VcPU = 0.0;

    CTRL.IAmp = 0.0;       //输出电流幅值
    CTRL.ITMP = 0.0;       //计算电流有效值用中间变量
    CTRL.NT = 0;           //一个同步速周期进入同步中断的次数
    CTRL.NAdc = 0;         //在一个同步中断周期内ADC的次数

    for(i=0;i<PHASE_NUMBER;i++)
    {
        CTRL.CmparBeforeCmpnst[i] = 0;     //死区补偿前的比较值
        CTRL.CmparAfterCmpnst[i] = 0;      //死区补偿后的比较值
    }

    // ============错误报警==============
    CTRL.SetOverMaxFreq = FALSE;
    CTRL.SetBelowMinFreq = FALSE;
    CTRL.UdcBelowZero = FALSE;
    CTRL.OverModulation = FALSE;


    #define AKATSU00 FALSE
    #if AKATSU00 == TRUE
    int ind;
    for(ind=0;ind<2;++ind){
    // for(i=0;i<2;++i){
        hav.emf_stator[ind] = 0;

        hav.psi_1[ind] = 0;
        hav.psi_2[ind] = 0;
        hav.psi_2_prev[ind] = 0;

        hav.psi_1_nonSat[ind] = 0;
        hav.psi_2_nonSat[ind] = 0;

        hav.psi_1_min[ind] = 0;
        hav.psi_1_max[ind] = 0;
        hav.psi_2_min[ind] = 0;
        hav.psi_2_max[ind] = 0;

        hav.rs_est = 3.04;
        hav.rreq_est = 1.6;

        hav.Delta_t = 1;
        hav.u_off[ind] = 0;
        hav.u_off_integral_input[ind] = 0;
        hav.gain_off = 0.025;

        hav.flag_pos2negLevelA[ind] = 0;
        hav.flag_pos2negLevelB[ind] = 0;
        hav.time_pos2neg[ind] = 0;
        hav.time_pos2neg_prev[ind] = 0;

        hav.flag_neg2posLevelA[ind] = 0;
        hav.flag_neg2posLevelB[ind] = 0;
        hav.time_neg2pos[ind] = 0;
        hav.time_neg2pos_prev[ind] = 0;    

        hav.sat_min_time[ind] = 0.0;
        hav.sat_max_time[ind] = 0.0;
    }

    a92v.awaya_lambda = 31.4*1;
    a92v.q0 = 0.0;
    a92v.q1_dot = 0.0;
    a92v.q1 = 0.0;
    a92v.tau_est = 0.0;
    a92v.sum_A = 0.0;
    a92v.sum_B = 0.0;
    a92v.est_Js_variation = 0.0;
    a92v.est_Js = 0.0;
    #endif



    // /*Jadot2009*/
    // CTRL.is_ref[0] = 0.0;
    // CTRL.is_ref[1] = 0.0;
    // CTRL.psi_ref[0] = 0.0;
    // CTRL.psi_ref[1] = 0.0;

    // CTRL.pi_vsJadot_0.Kp = 7; 
    // CTRL.pi_vsJadot_0.Ti = 1.0/790.0; 
    // CTRL.pi_vsJadot_0.Kp = 15; 
    // CTRL.pi_vsJadot_0.Ti = 0.075; 
    // CTRL.pi_vsJadot_0.Ki = CTRL.pi_vsJadot_0.Kp / CTRL.pi_vsJadot_0.Ti * TS;
    // CTRL.pi_vsJadot_0.i_state = 0.0;
    // CTRL.pi_vsJadot_0.i_limit = 300.0; // unit: Volt

    // CTRL.pi_vsJadot_1.Kp = 7; 
    // CTRL.pi_vsJadot_1.Ti = 1.0/790.0; 
    // CTRL.pi_vsJadot_1.Kp = 15; 
    // CTRL.pi_vsJadot_1.Ti = 0.075; 
    // CTRL.pi_vsJadot_1.Ki = CTRL.pi_vsJadot_1.Kp / CTRL.pi_vsJadot_1.Ti * TS;
    // CTRL.pi_vsJadot_1.i_state = 0.0;
    // CTRL.pi_vsJadot_1.i_limit = 300.0; // unit: Volt

    // PID调谐
    ACMSIMC_PIDTuner();
    printf("Speed PID: Kp=%g, Ki=%g, limit=%g Nm\n", pid1_spd.Kp, pid1_spd.Ki/CL_TS, pid1_spd.OutLimit);
    printf("Current PID: Kp=%g, Ki=%g, limit=%g V\n", pid1_id.Kp, pid1_id.Ki/CL_TS, pid1_id.OutLimit);
}

// 控制器：输入为电机的转速指令，根据反馈的电流、转速、位置信息，计算电机的电压指令。
// void control(double speed_cmd, double speed_cmd_dot){
//     // Input 1 is feedback: estimated speed/position or measured speed/position
//     #if SENSORLESS_CONTROL
//         // harnefors_scvm();
//         CTRL.omg__fb     = omg_harnefors;
//         // CTRL.theta_d__fb = theta_d_harnefors;
//     #else
//         // from measurement() in main.c
//         CTRL.omg__fb     = sm.omg_elec;
//         // CTRL.theta_M = sm.theta_d; 
//     #endif

//     // Input 2 is feedback: measured current 
//     CTRL.ial__fb = IS_C(0);
//     CTRL.ibe__fb = IS_C(1);

//     // Input 3 is the flux linkage command 
//     #if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
//         CTRL.rotor_flux_cmd = 0.0;
//         CTRL.cosT = cos(CTRL.theta_d__fb); 
//         CTRL.sinT = sin(CTRL.theta_d__fb);
//     #else
//         getch("Not Implemented");
//     #endif

//     // d-axis current command
//     CTRL.id_cmd = CTRL.rotor_flux_cmd / CTRL.Ld;

//     // q-axis current command
//     static int vc_count = 0;
//     if(vc_count++ == SPEED_LOOP_CEILING){
//         // velocity control loop execution frequency is 40 times slower than current control loop execution frequency
//         vc_count = 0;

//         pid1_spd.Ref = speed_cmd*RPM_2_RAD_PER_SEC;
//         pid1_spd.Fdb = CTRL.omg__fb;
//         pid1_spd.calc(&pid1_spd);
//         CTRL.iq_cmd = pid1_spd.Out;

//         // for plot
//         // CTRL.speed_ctrl_err = CTRL.omg_ctrl_err * RAD_PER_SEC_2_RPM;
//     }

//     // Measured current in d-q frame
//     CTRL.id__fb = AB2M(CTRL.ial__fb, CTRL.ibe__fb, CTRL.cosT, CTRL.sinT);
//     CTRL.iq__fb = AB2T(CTRL.ial__fb, CTRL.ibe__fb, CTRL.cosT, CTRL.sinT);

//     // For luenberger position observer for HFSI
//     CTRL.Tem     = CTRL.npp * (CTRL.KE*CTRL.iq__fb + (CTRL.Ld-CTRL.Lq)*CTRL.id__fb*CTRL.iq__fb);
//     CTRL.Tem_cmd = CTRL.npp * (CTRL.KE*CTRL.iq_cmd + (CTRL.Ld-CTRL.Lq)*CTRL.id_cmd*CTRL.iq_cmd);

//     // Voltage command in d-q frame
//     double vd, vq;
//     pid1_id.Fdb = CTRL.id__fb;
//     pid1_id.Ref = CTRL.id_cmd;
//     pid1_id.calc(&pid1_id);
//     vd = pid1_id.Out;
//     pid1_iq.Fdb = CTRL.iq__fb;
//     pid1_iq.Ref = CTRL.iq_cmd;
//     pid1_iq.calc(&pid1_iq);
//     vq = pid1_iq.Out;

//     // Current loop decoupling (skipped for now)
//     CTRL.ud_cmd = vd;
//     CTRL.uq_cmd = vq;

//     // Voltage command in alpha-beta frame
//     CTRL.ual = MT2A(CTRL.ud_cmd, CTRL.uq_cmd, CTRL.cosT, CTRL.sinT);
//     CTRL.ube = MT2B(CTRL.ud_cmd, CTRL.uq_cmd, CTRL.cosT, CTRL.sinT);
// }


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
    #if EXCITATION_TYPE == 1
        REAL position_command = 10*2;
        if(CTRL.timebase>5){
            position_command = -10*2; 
        }
        REAL position_error = position_command - ACM.x[5];
        REAL position_KP = 8;
        REAL rad_speed_command = position_KP*position_error;
        REAL rpm_speed_command = rad_speed_command*RAD_PER_SEC_2_RPM;
    #endif

    #if EXCITATION_TYPE == 2
        // 扫频建模
        // #define SWEEP_FREQ_AMPL 500 // rpm
        // #define MAX_FREQ SWEEP_FREQ_MAX_FREQ // Hz
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
            // amp_current_command = (0.05 * PMSM_RATED_CURRENT_RMS*1.414) * sin(2*M_PI*sf.current_freq*(sf.time - sf.last_current_freq_end_time));
            amp_current_command = SWEEP_FREQ_CURRENT_AMPL * sin(2*M_PI*sf.current_freq*(sf.time - sf.last_current_freq_end_time));
        }
    #endif

    #if EXCITATION_TYPE == 0
        // 转速运动模式
        rpm_speed_command = 500*sin(2*M_PI*88*CTRL.timebase); // overwrite
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
    // CTRL.theta_d__fb = sm.theta_d;
    
    //（无感）
    // harnefors_scvm();
    // CTRL.omg__fb     = omg_harnefors;
    // CTRL.theta_d__fb = theta_d_harnefors;



    CTRL.cosT = cos(CTRL.theta_M);
    CTRL.sinT = sin(CTRL.theta_M);
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
    // REAL decoupled_d_axis_voltage = pid1_id.Out - pid1_iq.Fdb*CTRL.Lq*CTRL.omg__fb;
    // REAL decoupled_q_axis_voltage = pid1_iq.Out + (pid1_id.Fdb*CTRL.Ld+CTRL.KE)*CTRL.omg__fb;
    REAL decoupled_d_axis_voltage = pid1_id.Out;
    REAL decoupled_q_axis_voltage = pid1_iq.Out;

    CTRL.ual = MT2A(decoupled_d_axis_voltage, decoupled_q_axis_voltage, CTRL.cosT, CTRL.sinT);
    CTRL.ube = MT2B(decoupled_d_axis_voltage, decoupled_q_axis_voltage, CTRL.cosT, CTRL.sinT);

    // for harnefors observer
    // CTRL.ud_cmd = pid1_id.Out;
    // CTRL.uq_cmd = pid1_iq.Out;
    // CTRL.id_cmd = pid1_id.Ref;
    // CTRL.iq_cmd = pid1_iq.Ref;
}

#endif