#include "ACMSim.h"
#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11
struct ControllerForExperiment CTRL;


#define IM ACM // 权宜之计

void CTRL_init(){

    // struct Marino2005
    marino.kz         = 1*700.0; // zd, zq

    marino.k_omega    = 0.5*88*60.0; // 6000  // e_omega // 增大这个可以消除稳态转速波形中的正弦扰动（源自q轴电流给定波形中的正弦扰动，注意实际的q轴电流里面是没有正弦扰动的）
    marino.kappa      = 24;      //0.05;  // e_omega // 增大这个意义不大，转速控制误差基本上已经是零了，所以kappa取0.05和24没有啥区别。

    marino.lambda_inv = 1e-1 * 6000.0;          // omega 磁链反馈为实际值时，这两个增益取再大都没有意义。

    marino.gamma_inv  = 3e0 * 180/MOTOR_SHAFT_INERTIA; // TL    磁链反馈为实际值时，这两个增益取再大都没有意义。
    marino.delta_inv  = 0*75.0; // alpha 要求磁链幅值时变

    marino.xTL_Max = 8.0;
    marino.xAlpha_Max = 7.0;
    marino.xAlpha_min = 3.0;
    printf("alpha: %g in [%g, %g]?\n", im.alpha, marino.xAlpha_min, marino.xAlpha_Max);

    marino.xRho = 0.0;
    marino.xTL = 0.0;
    marino.xAlpha = im.alpha;
    marino.xOmg = 0.0;

    marino.deriv_xTL = 0.0;
    marino.deriv_xAlpha = 0.0;
    marino.deriv_xOmg = 0.0;

    marino.psi_Dmu = 0.0;
    marino.psi_Qmu = 0.0;

    marino.zD = 0.0;
    marino.zQ = 0.0;
    marino.e_iDs = 0.0;
    marino.e_iQs = 0.0;
    marino.e_psi_Dmu = 0.0;
    marino.e_psi_Qmu = 0.0;

    marino.deriv_iD_cmd = 0.0;
    marino.deriv_iQ_cmd = 0.0;

    marino.Gamma_D = 0.0;
    marino.Gamma_Q = 0.0;

    marino.torque_cmd = 0.0;
    marino.torque__fb = 0.0;

    // struct Holtz2003
    holtz.psi_D2 = 0.0;
    holtz.psi_Q2 = 0.0;
    holtz.psi_D1_ode1 = 0.0;
    holtz.psi_Q1_ode1 = 0.0;
    holtz.psi_D2_ode1 = 0.0;
    holtz.psi_Q2_ode1 = 0.0;
    holtz.psi_D1_ode4 = 0.0;
    holtz.psi_Q1_ode4 = 0.0;
    holtz.psi_D2_ode4 = 0.0;
    holtz.psi_Q2_ode4 = 0.0;





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


struct Marino2005 marino;
struct Holtz2003 holtz;

REAL sat_kappa(REAL x){
    if(x>marino.kappa){
        return marino.kappa;
    }else if(x<-marino.kappa){
        return -marino.kappa;
    }else{
        return x;
    }
}
REAL deriv_sat_kappa(REAL x){
    if(x>marino.kappa){
        return 0;
    }else if(x<-marino.kappa){
        return -0;
    }else{
        return 1;
    }
}
void controller_marino2005(){

    // Cascaded from other system
    /* VM based Flux Est. */
    // Akatsu_RrId();
    improved_Holtz_method();

    // flux feedback
    // marino.psi_Dmu = holtz.psi_D2;
    // marino.psi_Qmu = holtz.psi_Q2;
    marino.psi_Dmu = holtz.psi_D2_ode1;
    marino.psi_Qmu = holtz.psi_Q2_ode1;

    // flux error quantities
    marino.e_psi_Dmu = marino.psi_Dmu - CTRL.psi_cmd;
    marino.e_psi_Qmu = marino.psi_Qmu - 0.0;

    // API to the fourth-order system of observer and identifiers
    observer_marino2005();
    CTRL.theta_D = marino.xRho;
    CTRL.omg__fb = marino.xOmg;
    CTRL.alpha   = marino.xAlpha;
    CTRL.TLoad   = marino.xTL;

    // 磁场可测 debug
    // CTRL.theta_D = ACM.theta_M;
    // CTRL.omg__fb = im.omg_elec;
    // CTRL.alpha   = ACM.alpha;
    // CTRL.TLoad   = ACM.TLoad;

    CTRL.alpha_inv = 1.0/CTRL.alpha;

    // αβ to DQ
    CTRL.cosT = cos(CTRL.theta_D);
    CTRL.sinT = sin(CTRL.theta_D);
    CTRL.iDs = AB2M(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT);
    CTRL.iQs = AB2T(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT);

    // TODO
    // CTRL.deriv_omg_cmd
    marino.deriv_iD_cmd = 1.0*CTRL.Lmu_inv*(  CTRL.deriv_psi_cmd \
                                            + CTRL.dderiv_psi_cmd*CTRL.alpha_inv \
                                            - CTRL.deriv_psi_cmd*CTRL.alpha_inv*CTRL.alpha_inv*marino.deriv_xAlpha);
    //TODO 重新写！
    REAL mu_temp = CTRL.npp*CTRL.Js_inv * CLARKE_TRANS_TORQUE_GAIN*CTRL.npp; 
    marino.deriv_iQ_cmd =   1.0*(-marino.k_omega*deriv_sat_kappa(CTRL.omg__fb-CTRL.omg_cmd) * (marino.deriv_xOmg - CTRL.deriv_omg_cmd) + CTRL.Js_inv*CTRL.npp*marino.deriv_xTL + CTRL.dderiv_omg_cmd ) / (mu_temp * CTRL.psi_cmd)\
                          - 1.0*(-marino.k_omega*sat_kappa(CTRL.omg__fb-CTRL.omg_cmd) + CTRL.Js_inv*CTRL.TLoad+CTRL.deriv_omg_cmd) * (CTRL.deriv_psi_cmd/(mu_temp*CTRL.psi_cmd*CTRL.psi_cmd));

    // current error quantities
    CTRL.iDs_cmd = ( CTRL.psi_cmd + CTRL.deriv_psi_cmd*CTRL.alpha_inv ) * CTRL.Lmu_inv;
    CTRL.iQs_cmd = ( CTRL.npp_inv*CTRL.Js *( 1*CTRL.deriv_omg_cmd - marino.k_omega*sat_kappa(CTRL.omg__fb-CTRL.omg_cmd) ) + CTRL.TLoad ) / (CLARKE_TRANS_TORQUE_GAIN*CTRL.npp*CTRL.psi_cmd);
    marino.e_iDs = CTRL.iDs - CTRL.iDs_cmd;
    marino.e_iQs = CTRL.iQs - CTRL.iQs_cmd;

    marino.torque_cmd = CLARKE_TRANS_TORQUE_GAIN * CTRL.npp * (CTRL.iQs_cmd * CTRL.psi_cmd   - CTRL.iDs_cmd*0);
    marino.torque__fb = CLARKE_TRANS_TORQUE_GAIN * CTRL.npp * (CTRL.iQs     * marino.psi_Dmu - CTRL.iDs * marino.psi_Qmu);
    // marino.torque__fb = CLARKE_TRANS_TORQUE_GAIN * CTRL.npp * (CTRL.iQs     * marino.psi_Dmu);


    // linear combination of error
    marino.zD = marino.e_iDs + CTRL.Lsigma_inv*marino.e_psi_Dmu;
    marino.zQ = marino.e_iQs + CTRL.Lsigma_inv*marino.e_psi_Qmu;

    // known signals to cancel
    marino.Gamma_D = CTRL.Lsigma_inv * (-CTRL.rs*CTRL.iDs -CTRL.alpha*CTRL.Lmu*CTRL.iDs_cmd +CTRL.alpha  *CTRL.psi_cmd +CTRL.omega_syn*marino.e_psi_Qmu) +CTRL.omega_syn*CTRL.iQs - marino.deriv_iD_cmd;
    marino.Gamma_Q = CTRL.Lsigma_inv * (-CTRL.rs*CTRL.iQs -CTRL.alpha*CTRL.Lmu*CTRL.iQs_cmd -CTRL.omg__fb*CTRL.psi_cmd -CTRL.omega_syn*marino.e_psi_Dmu) -CTRL.omega_syn*CTRL.iDs - marino.deriv_iQ_cmd;

    // voltage commands
    CTRL.uDs_cmd = CTRL.Lsigma * (-(marino.kz+0.25*CTRL.Lsigma*CTRL.Lmu*marino.xAlpha_Max)*marino.zD - marino.Gamma_D);
    CTRL.uQs_cmd = CTRL.Lsigma * (-(marino.kz+0.25*CTRL.Lsigma*CTRL.Lmu*marino.xAlpha_Max)*marino.zQ - marino.Gamma_Q);
    CTRL.ual_cmd = MT2A(CTRL.uDs_cmd, CTRL.uQs_cmd, CTRL.cosT, CTRL.sinT);
    CTRL.ube_cmd = MT2B(CTRL.uDs_cmd, CTRL.uQs_cmd, CTRL.cosT, CTRL.sinT);
}


void controller_IFOC();
void controller(){

    // 1. 生成转速指令
    static REAL rpm_speed_command=0.0, amp_current_command=0.0;

    // commands(&rpm_speed_command, &amp_current_command);

    static REAL local_dc_rpm_cmd = 0.0; 
    REAL OMG1;
    if(CTRL.timebase>3){
        if(CTRL.timebase>3.5){
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
        CTRL.psi_cmd_raw += CL_TS*CTRL.m0;
        CTRL.psi_cmd     = CTRL.psi_cmd_raw;
        CTRL.deriv_psi_cmd  = CTRL.m0;
        CTRL.dderiv_psi_cmd = 0.0;
    }else{
        // CTRL.m1 = 0.0;
        CTRL.psi_cmd_raw = CTRL.m0 + CTRL.m1 * sin(CTRL.omega1*CTRL.timebase);
        CTRL.psi_cmd     = CTRL.psi_cmd_raw; // _lpf(CTRL.psi_cmd_raw, CTRL.psi_cmd, 5);
        CTRL.psi_cmd_inv = 1.0/ CTRL.psi_cmd;
        CTRL.deriv_psi_cmd  = CTRL.m1 * CTRL.omega1 * cos(CTRL.omega1*CTRL.timebase);
        CTRL.dderiv_psi_cmd = CTRL.m1 * CTRL.omega1 * CTRL.omega1 * -sin(CTRL.omega1*CTRL.timebase);
    }
    CTRL.psi_cmd_inv = 1.0/ CTRL.psi_cmd;


    #if CONTROL_STRATEGY == INDIRECT_FOC
        controller_IFOC();
    #elif CONTROL_STRATEGY == MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL
        controller_marino2005();
    #endif

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
    CTRL.omg__fb     = im.omg_elec;

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
    CTRL.torque_cmd = CLARKE_TRANS_TORQUE_GAIN * im.npp * CTRL.iQs_cmd * (CTRL.psi_cmd);
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

            decoupled_M_axis_voltage = pid1_iM.Out + (CTRL.Lsigma) * (-CTRL.omega_syn*CTRL.iQs); // Telford03/04
            // decoupled_T_axis_voltage = pid1_iT.Out + CTRL.omega_syn*(CTRL.psi_cmd + im.Lsigma*CTRL.iMs); // 这个行，但是无速度运行时，会导致M轴电流在转速暂态高频震荡。
            // decoupled_T_axis_voltage = vT + CTRL.omega_syn*(CTRL.Lsigma+CTRL.Lmu)*CTRL.iMs; // 这个就不行，说明：CTRL.Lmu*iMs != ob.taao_flux_cmd，而是会因iMs的波动在T轴控制上引入波动和不稳定
            decoupled_T_axis_voltage = pid1_iT.Out; // 无感用这个
        #else
            decoupled_M_axis_voltage = pid1_iM.Out;
            decoupled_T_axis_voltage = pid1_iT.Out;
        #endif
    }

    // 7. 反帕克变换
    CTRL.ual_cmd = MT2A(decoupled_M_axis_voltage, decoupled_T_axis_voltage, CTRL.cosT, CTRL.sinT);
    CTRL.ube_cmd = MT2B(decoupled_M_axis_voltage, decoupled_T_axis_voltage, CTRL.cosT, CTRL.sinT);
}
#endif
