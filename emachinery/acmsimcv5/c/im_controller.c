#include "ACMSim.h"
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11
struct ControllerForExperiment CTRL;


#define IM ACM // 权宜之计

void CTRL_init(){


    int i=0,j=0;

    CTRL.timebase = 0.0;

        /* Parameter (including speed) Adaptation */ 
        CTRL.npp    = IM.npp;
        CTRL.npp_inv    = 1.0/CTRL.npp;
        CTRL.rs     = IM.rs;
        CTRL.rreq   = IM.rreq;
        CTRL.Lsigma = IM.Lsigma;
        CTRL.Lsigma_inv = 1.0/IM.Lsigma;
        CTRL.Lmu    = IM.Lmu;
        CTRL.Lmu_inv = 1.0/IM.Lmu;
        CTRL.alpha  = CTRL.rreq/CTRL.Lmu;
        CTRL.alpha_inv = 1.0/CTRL.alpha;
        CTRL.Js     = IM.Js;
        CTRL.Js_inv = 1.0/IM.Js;

        CTRL.TLoad  = 0.0;

    CTRL.iDs = 0.0;
    CTRL.iQs = 0.0;
    CTRL.iDs_cmd = 0.0;
    CTRL.iQs_cmd = 0.0;

    CTRL.uDs_cmd = 0.0;
    CTRL.uQs_cmd = 0.0;
    CTRL.ual_cmd = 0.0;
    CTRL.ube_cmd = 0.0;

    CTRL.omg_cmd = 0.0;
    CTRL.deriv_omg_cmd;
    CTRL.dderiv_omg_cmd;
    CTRL.torque_cmd = 0.0;

    // CTRL.torque_integralPart = 0.0;
    // CTRL.torque_integralLimit = 200;

    // CTRL.uMs_integralPart = 0.0;
    // CTRL.uMs_integralLimit = 500;
    // CTRL.uTs_integralPart = 0.0;
    // CTRL.uTs_integralLimit = 800;

    CTRL.theta_D = 0.0;
    CTRL.cosT = 1.0;
    CTRL.sinT = 0.0;

    CTRL.e_M = 0.0;
    CTRL.e_T = 0.0;



    CTRL.omega_sl = 0.0;
    CTRL.omega_syn = 0.0;

    // CTRL.tajima_omg = 0.0;
    // CTRL.K_PEM = 0.1;

    CTRL.omg__fb = 0.0;
    CTRL.ial__fb = 0.0;
    CTRL.ibe__fb = 0.0;

    // CTRL.psial_fb = 0.0;
    // CTRL.psibe_fb = 0.0;
    // CTRL.psimod_fb = 0.0;
    // CTRL.psimod_fb_inv = 1.0;
    // CTRL.deriv_fluxModSim = 0.0;

    CTRL.psi_cmd_raw    = 0.0;
    CTRL.psi_cmd        = 0.0;
    CTRL.dderiv_psi_cmd = 0.0;
    CTRL.deriv_psi_cmd  = 0.0;
    CTRL.m0 = IM_FLUX_COMMAND_DC_PART;
    CTRL.m1 = IM_FLUX_COMMAND_SINE_PART;
    CTRL.omega1 = 2*M_PI*IM_FLUX_COMMAND_SINE_HERZ;



    // CTRL.omg_ctrl_err = 0.0;
    // CTRL.gamma_res_transient = 0;
    // CTRL.gamma_res_transient_shape = 2000; //2000~5000都不错
    // CTRL.gamma_res2_transient = 0;
    // CTRL.gamma_res2_transient_shape = 2; // 10
    // CTRL.gamma_omg_transient = 0;
    // CTRL.gamma_omg_transient_shape = 1; // 1 数值越小，那么区域越宽，避免转速稳态下转速辨识增益振荡。
    // CTRL.resistance_id_on = FALSE;


    CTRL.sensorless    = SENSORLESS_CONTROL;
    CTRL.ctrl_strategy = CONTROL_STRATEGY;


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
    printf("Current PID: Kp=%g, Ki=%g, limit=%g V\n", pid1_iM.Kp, pid1_iM.Ki/CL_TS, pid1_iM.OutLimit);
}

// 定义特定的测试指令，如快速反转等
void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd){
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


void controller_IFOC();
void controller(){

    // 1. 生成转速指令
    static REAL rpm_speed_command=0.0, amp_current_command=0.0;

    // commands(&rpm_speed_command, &amp_current_command);

    static REAL local_dc_rpm_cmd = 0.0; 
    REAL OMG1;
    if(CTRL.timebase>3){
        if(CTRL.timebase>3.875){
            OMG1 = (2*M_PI*32);
        }else if(CTRL.timebase>3.75){
            OMG1 = (2*M_PI*16);
        }else if(CTRL.timebase>3.5){
            OMG1 = (2*M_PI*8);
        }else{
            OMG1 = (2*M_PI*4);
        }
        rpm_speed_command   = 100          * sin(OMG1*CTRL.timebase) + local_dc_rpm_cmd;
        CTRL.omg_cmd        = (100         * sin(OMG1*CTRL.timebase) + local_dc_rpm_cmd)*RPM_2_RAD_PER_SEC;
        CTRL.deriv_omg_cmd  = 100*OMG1     * cos(OMG1*CTRL.timebase)                    *RPM_2_RAD_PER_SEC;
        CTRL.dderiv_omg_cmd = 100*OMG1*OMG1*-sin(OMG1*CTRL.timebase)                    *RPM_2_RAD_PER_SEC;
    }else if(CTRL.timebase>2){
        rpm_speed_command   = local_dc_rpm_cmd;
        CTRL.omg_cmd        = rpm_speed_command*RPM_2_RAD_PER_SEC;
        CTRL.deriv_omg_cmd  = 0;
        CTRL.dderiv_omg_cmd = 0;
    }else if(CTRL.timebase>1){
        static REAL last_omg_cmd;
        rpm_speed_command += CL_TS*150;
        local_dc_rpm_cmd    = rpm_speed_command;
        CTRL.omg_cmd        = rpm_speed_command*RPM_2_RAD_PER_SEC;
        CTRL.deriv_omg_cmd  = (CTRL.omg_cmd - last_omg_cmd)*CL_TS_INVERSE; //50*RPM_2_RAD_PER_SEC;
        CTRL.dderiv_omg_cmd = 0;

        last_omg_cmd = CTRL.omg_cmd;
    }else{
        rpm_speed_command   = 0;
        CTRL.omg_cmd        = rpm_speed_command*RPM_2_RAD_PER_SEC;
        CTRL.deriv_omg_cmd  = 0;
        CTRL.dderiv_omg_cmd = 0;
    }

    // 2. 生成磁链指令
    if(CTRL.timebase<1){
        CTRL.psi_cmd_raw   += CL_TS*CTRL.m0;
        CTRL.psi_cmd        = CTRL.psi_cmd_raw;
        CTRL.deriv_psi_cmd  = CTRL.m0;
        CTRL.dderiv_psi_cmd = 0.0;
    }else{
        // CTRL.m1 = 0.0;
        CTRL.psi_cmd_raw    = CTRL.m0 + CTRL.m1 * sin(CTRL.omega1*CTRL.timebase);
        CTRL.psi_cmd        = CTRL.psi_cmd_raw; // _lpf(CTRL.psi_cmd_raw, CTRL.psi_cmd, 5);
        CTRL.deriv_psi_cmd  =           CTRL.m1 * CTRL.omega1 * cos(CTRL.omega1*CTRL.timebase);
        CTRL.dderiv_psi_cmd =           CTRL.m1 * CTRL.omega1 * CTRL.omega1 * -sin(CTRL.omega1*CTRL.timebase);
    }
    CTRL.psi_cmd_inv = 1.0/ CTRL.psi_cmd;


    #if CONTROL_STRATEGY == INDIRECT_FOC
        controller_IFOC();
    #elif CONTROL_STRATEGY == MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL
        controller_marino2005();
    #endif

    // debug marino05 observer---not work as expected. For Marino05 you cannot tune observer and controller separately. They are just coupled together.
    // controller_IFOC();

    // for plot
    ACM.rpm_cmd = rpm_speed_command;
    CTRL.speed_ctrl_err = (CTRL.omg__fb - CTRL.omg_cmd)*RAD_PER_SEC_2_RPM;
}



void controller_IFOC(){

    // 3. 电气转子位置和电气转子转速反馈
        //（编码器反馈）
        // CTRL.omg__fb     = qep.omg_elec;
        // CTRL.theta_d__fb = qep.theta_d;

    //（实际反馈，实验中不可能）
    // CTRL.omg__fb     = im.omg_elec;

        //（无感）
        // harnefors_scvm();
        // CTRL.omg__fb     = omg_harnefors;
        // CTRL.theta_d__fb = theta_d_harnefors;

    // 4. 帕克变换
    #define THE_FIELD_IS_KNOWN FALSE
    #if THE_FIELD_IS_KNOWN
        CTRL.theta_D = atan2(IM.x[3], IM.x[2]); 
        CTRL.cosT = cos(CTRL.theta_D); 
        CTRL.sinT = sin(CTRL.theta_D);
    #else
        // 间接磁场定向第一部分
        CTRL.theta_D += CL_TS * CTRL.omega_syn;
        CTRL.cosT = cos(CTRL.theta_D); 
        CTRL.sinT = sin(CTRL.theta_D);
        if(CTRL.theta_D > M_PI){
            CTRL.theta_D -= 2*M_PI;        
        }else if(CTRL.theta_D < -M_PI){
            CTRL.theta_D += 2*M_PI; // 反转！
        }
    #endif
    CTRL.iDs = AB2M(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT);
    CTRL.iQs = AB2T(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT);
    pid1_iM.Fbk = CTRL.iDs;
    pid1_iT.Fbk = CTRL.iQs;


    // 5. 转速环
    static int vc_count = 0;
    if(vc_count++ == SPEED_LOOP_CEILING){
        vc_count = 0;

        pid1_spd.Ref = CTRL.omg_cmd; //rpm_speed_command*RPM_2_RAD_PER_SEC;
        pid1_spd.Fbk = CTRL.omg__fb;
        pid1_spd.calc(&pid1_spd);
        pid1_iT.Ref = pid1_spd.Out;
        CTRL.iQs_cmd = pid1_iT.Ref;
    }
    // 磁链环
    // if(ob.taao_flux_cmd_on){
    //     CTRL.iDs_cmd = IM_FLUX_COMMAND_DC_PART * CTRL.Lmu_inv   \
    //                    + (   M1*OMG1*cos(OMG1*ob.timebase)  )/ CTRL.rreq; ///////////////////////////////// 
    // }else{
        // CTRL.iDs_cmd =  * CTRL.Lmu_inv + (deriv_fluxModCmd)/ CTRL.rreq; 
        CTRL.psi_cmd = IM_FLUX_COMMAND_DC_PART;
        CTRL.iDs_cmd = CTRL.psi_cmd * CTRL.Lmu_inv;
    // }
    pid1_iM.Ref = CTRL.iDs_cmd;
    // 计算转矩
    CTRL.torque_cmd = CLARKE_TRANS_TORQUE_GAIN * CTRL.npp * CTRL.iQs_cmd * (CTRL.psi_cmd);
    // 间接磁场定向第二部分
    CTRL.omega_sl = CTRL.rreq*CTRL.iQs_cmd/(CTRL.psi_cmd);
    CTRL.omega_syn = CTRL.omg__fb + CTRL.omega_sl;

    // 扫频将覆盖上面产生的励磁、转矩电流指令
    #if EXCITATION_TYPE == EXCITATION_SWEEP_FREQUENCY
        #if SWEEP_FREQ_C2V == TRUE
            pid1_iT.Ref = amp_current_command; 
        #endif
        #if SWEEP_FREQ_C2C == TRUE
            pid1_iT.Ref = 0.0;
            pid1_iM.Ref = amp_current_command;
        #else
            pid1_iM.Ref = 0.0;
        #endif
    #endif



    // 6. 电流环
    REAL decoupled_M_axis_voltage=0.0, decoupled_T_axis_voltage=0.0;
    pid1_iM.calc(&pid1_iM);
    pid1_iT.calc(&pid1_iT);
    {   // Steady state dynamics based decoupling circuits for current regulation
        #if VOLTAGE_CURRENT_DECOUPLING_CIRCUIT == TRUE
            // decoupled_M_axis_voltage = vM + (CTRL.rs+CTRL.rreq)*CTRL.iMs + CTRL.Lsigma*(-CTRL.omega_syn*CTRL.iTs) - CTRL.alpha*CTRL.psimod_fb; // Jadot09
            // decoupled_T_axis_voltage = vT + (CTRL.rs+CTRL.rreq)*CTRL.iTs + CTRL.Lsigma*( CTRL.omega_syn*CTRL.iMs) + CTRL.omg_fb*CTRL.psimod_fb;
            // decoupled_T_axis_voltage = vT + CTRL.omega_syn*(CTRL.Lsigma+CTRL.Lmu)*CTRL.iMs; // 这个就不行，说明：CTRL.Lmu*iMs != ob.taao_flux_cmd，而是会因iMs的波动在T轴控制上引入波动和不稳定

            decoupled_M_axis_voltage = pid1_iM.Out + (CTRL.Lsigma) * (-CTRL.omega_syn*CTRL.iQs); // Telford03/04
            decoupled_T_axis_voltage = pid1_iT.Out + CTRL.omega_syn*(CTRL.psi_cmd + CTRL.Lsigma*CTRL.iDs); // 这个行，但是无速度运行时，会导致M轴电流在转速暂态高频震荡。
            // decoupled_T_axis_voltage = pid1_iT.Out; // 无感用这个
        #else
            decoupled_M_axis_voltage = pid1_iM.Out;
            decoupled_T_axis_voltage = pid1_iT.Out;
        #endif
    }
    CTRL.uDs_cmd = decoupled_M_axis_voltage;
    CTRL.uQs_cmd = decoupled_T_axis_voltage;

    // 7. 反帕克变换
    CTRL.ual_cmd = MT2A(decoupled_M_axis_voltage, decoupled_T_axis_voltage, CTRL.cosT, CTRL.sinT);
    CTRL.ube_cmd = MT2B(decoupled_M_axis_voltage, decoupled_T_axis_voltage, CTRL.cosT, CTRL.sinT);
}
#endif
