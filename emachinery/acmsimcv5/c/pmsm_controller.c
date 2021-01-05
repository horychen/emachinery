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

    CTRL.ual_cmd = 0.0;
    CTRL.ube_cmd = 0.0;

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


// double theta_d_harnefors = 0.0;
// double omg_harnefors = 0.0;
// void harnefors_scvm(){
//     #define KE_MISMATCH 1.0 // 0.7
//     double d_axis_emf;
//     double q_axis_emf;
//     #define LAMBDA 2 // 2
//     #define CJH_TUNING_A 25 // 1
//     #define CJH_TUNING_B 1 // 1
//     double lambda_s = LAMBDA * sign(omg_harnefors);
//     double alpha_bw_lpf = CJH_TUNING_A*0.1*(1500*RPM_2_RAD_PER_SEC) + CJH_TUNING_B*2*LAMBDA*fabs(omg_harnefors);
//     // d_axis_emf = CTRL.ud_cmd - 1*CTRL.R*CTRL.id_cmd + omg_harnefors*1.0*CTRL.Lq*CTRL.iq_cmd; // If Ld=Lq.
//     // q_axis_emf = CTRL.uq_cmd - 1*CTRL.R*CTRL.iq_cmd - omg_harnefors*1.0*CTRL.Ld*CTRL.id_cmd; // If Ld=Lq.
//     d_axis_emf = CTRL.ud_cmd - 1*CTRL.R*CTRL.id_cmd + omg_harnefors*1.0*CTRL.Lq*CTRL.iq_cmd; // eemf
//     q_axis_emf = CTRL.uq_cmd - 1*CTRL.R*CTRL.iq_cmd - omg_harnefors*1.0*CTRL.Lq*CTRL.id_cmd; // eemf
//     // Note it is bad habit to write numerical integration explictly like this. The states on the right may be accencidentally modified on the run.
//     theta_d_harnefors += CL_TS * omg_harnefors;
//     omg_harnefors += CL_TS * alpha_bw_lpf * ( (q_axis_emf - lambda_s*d_axis_emf)/(CTRL.KE*KE_MISMATCH+(CTRL.Ld-CTRL.Lq)*CTRL.id_cmd) - omg_harnefors );
//     while(theta_d_harnefors>M_PI) theta_d_harnefors-=2*M_PI;
//     while(theta_d_harnefors<-M_PI) theta_d_harnefors+=2*M_PI;   
// }
void controller(){

    // 1. 生成转速指令
    double rpm_speed_command, amp_current_command;
    commands(&rpm_speed_command, &amp_current_command);


    // 2. 电气转子位置和电气转子转速反馈
    harnefors_scvm();
    // harnefors.omg_elec = omg_harnefors;
    // harnefors.theta_d = theta_d_harnefors;
    #if SENSORLESS_CONTROL
        //（无感）
        CTRL.omg__fb     = harnefors.omg_elec;
        CTRL.theta_d__fb = harnefors.theta_d;
    #else
        //（霍尔反馈）
        // CTRL.omg__fb     = sm.omg_elec_hall;
        // CTRL.theta_d__fb = sm.theta_d_hall;
    
        //（编码器反馈）
        // CTRL.omg__fb     = qep.omg_elec;
        // CTRL.theta_d__fb = qep.theta_d;

        //（实际反馈，实验中不可能）
        CTRL.omg__fb     = sm.omg_elec;
        CTRL.theta_d__fb = sm.theta_d;
    #endif


    // 帕克变换
    // Input 2 is feedback: measured current 
    CTRL.cosT = cos(CTRL.theta_d__fb);
    CTRL.sinT = sin(CTRL.theta_d__fb);
    CTRL.id__fb = AB2M(CTRL.ial__fb, CTRL.ibe__fb, CTRL.cosT, CTRL.sinT);
    CTRL.iq__fb = AB2T(CTRL.ial__fb, CTRL.ibe__fb, CTRL.cosT, CTRL.sinT);
    pid1_id.Fbk = CTRL.id__fb;
    pid1_iq.Fbk = CTRL.iq__fb;


    // 转速环
    static int vc_count = 0;
    if(vc_count++ == SPEED_LOOP_CEILING){
        vc_count = 0;

        pid1_spd.Ref = rpm_speed_command*RPM_2_RAD_PER_SEC;
        pid1_spd.Fbk = CTRL.omg__fb;
        pid1_spd.calc(&pid1_spd);
        pid1_iq.Ref = pid1_spd.Out;
    }
    CTRL.iq_cmd = pid1_spd.Out;
    // 磁链环
    #if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
        CTRL.rotor_flux_cmd = 0.0;
        pid1_id.Ref = CTRL.rotor_flux_cmd / CTRL.Ld;
    #else
        getch("Not Implemented");
    #endif

    // For luenberger position observer for HFSI
    CTRL.Tem     = CTRL.npp * (CTRL.KE*CTRL.iq__fb + (CTRL.Ld-CTRL.Lq)*CTRL.id__fb*CTRL.iq__fb);
    CTRL.Tem_cmd = CTRL.npp * (CTRL.KE*CTRL.iq_cmd + (CTRL.Ld-CTRL.Lq)*CTRL.id_cmd*CTRL.iq_cmd);

    // 扫频将覆盖上面产生的励磁、转矩电流指令
    #if SWEEP_FREQ_C2V == TRUE
        pid1_iq.Ref = amp_current_command; 
    #endif
    #if SWEEP_FREQ_C2C == TRUE
        pid1_iq.Ref = 0.0;
        pid1_id.Ref = amp_current_command;
    #endif 

    // 电流环
    pid1_id.calc(&pid1_id);
    pid1_iq.calc(&pid1_iq);
    // 解耦
    #if VOLTAGE_CURRENT_DECOUPLING_CIRCUIT
        REAL decoupled_d_axis_voltage = pid1_id.Out -             pid1_iq.Fbk*CTRL.Lq *CTRL.omg__fb;
        REAL decoupled_q_axis_voltage = pid1_iq.Out + ( CTRL.KE + pid1_id.Fbk*CTRL.Ld)*CTRL.omg__fb;
    #else
        REAL decoupled_d_axis_voltage = pid1_id.Out;
        REAL decoupled_q_axis_voltage = pid1_iq.Out;
    #endif

    // 反帕克变换
    CTRL.ual_cmd = MT2A(decoupled_d_axis_voltage, decoupled_q_axis_voltage, CTRL.cosT, CTRL.sinT);
    CTRL.ube_cmd = MT2B(decoupled_d_axis_voltage, decoupled_q_axis_voltage, CTRL.cosT, CTRL.sinT);

    // for harnefors observer
    CTRL.ud_cmd = decoupled_d_axis_voltage; //pid1_id.Out;
    CTRL.uq_cmd = decoupled_q_axis_voltage; //pid1_iq.Out;
    CTRL.id_cmd = pid1_id.Ref;
    CTRL.iq_cmd = pid1_iq.Ref;

    // for plot
    ACM.rpm_cmd = rpm_speed_command;
    CTRL.speed_ctrl_err = rpm_speed_command*RPM_2_RAD_PER_SEC - CTRL.omg__fb;
}

#endif
