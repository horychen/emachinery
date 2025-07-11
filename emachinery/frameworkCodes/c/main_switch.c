// This file is used in both simulation and experiment, so editing this file in experiment will result in overwritten.

#include "ACMSim.h"
#if PC_SIMULATION
    #define DC_BUS_VOLTAGE_INVERSE (1.732 / d_sim.init.Vdc)
#else
    #include "All_Definition.h"
    extern st_axis *Axis;
    #define DC_BUS_VOLTAGE_INVERSE (1.732 / Axis->vdc)
#endif
/* 定义顶级结构体（指针的集合） */
ST_D_SIM d_sim;
int axisCnt = 0;
int use_first_set_three_phase = 1;
struct ControllerForExperiment CTRL_1;
struct ControllerForExperiment *CTRL;
struct DebugExperiment debug_1;
struct DebugExperiment *debug = &debug_1;
struct ObserverForSpeedReconstruction OFSR;
REAL one_over_six = 1.0/6.0;
// 定义内存空间（结构体）
st_motor_parameters     t_motor_1={0};
st_enc                  t_enc_1={0};
st_psd                  t_psd_1={0};
st_controller_inputs    t_I_1={0};
st_controller_states    t_S_1={0};
st_controller_outputs   t_O_1={0};
st_InverterNonlinearity t_inv_1={0}; // Because of the sv_count bug, I cannot declare t_inv in this .c file. // extern st_InverterNonlinearity t_inv; 
st_capture              t_cap_1={0};
st_global_variables     t_g_1={0};
st_pid_regulator _PID_iD_1       = st_pid_regulator_DEFAULTS;
st_pid_regulator _PID_iQ_1       = st_pid_regulator_DEFAULTS;
st_pid_regulator _PID_Speed_1    = st_pid_regulator_DEFAULTS;
st_pid_regulator _PID_Position_1 = st_pid_regulator_DEFAULTS;
/* 为全局结构体分配具体的内存，为实验中由于多台电机的控制结构体进行初始化 */
#if PC_SIMULATION == FALSE
    //#pragma DATA_SECTION(CTRL     ,"MYGLOBALS"); //
    #pragma DATA_SECTION(CTRL_1       ,"MYGLOBALS_1"); //
    // #pragma DATA_SECTION(debug_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_motor_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_enc_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_psd_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_I_1        ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_S_1        ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_O_1        ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_inv_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_cap_1      ,"MYGLOBALS_1");
    #pragma DATA_SECTION(t_g_1        ,"MYGLOBALS_1");
    // #pragma DATA_SECTION(_PID_iX_1    ,"MYGLOBALS_1");
    // #pragma DATA_SECTION(_PID_iY_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_iD_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_iQ_1    ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_Position_1   ,"MYGLOBALS_1");
    #pragma DATA_SECTION(_PID_Speed_1   ,"MYGLOBALS_1");

    #if NUMBER_OF_AXES == 2
        // extern and pragma should come in pair?
        #pragma DATA_SECTION(CTRL_2     ,"MYGLOBALS_2");
        // #pragma DATA_SECTION(debug_2      ,"MYGLOBALS_1");
        #pragma DATA_SECTION(t_motor_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_enc_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_psd_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_I_2        ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_S_2        ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_O_2        ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_inv_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_cap_2      ,"MYGLOBALS_2");
        #pragma DATA_SECTION(t_g_2        ,"MYGLOBALS_2");
        // #pragma DATA_SECTION(_PID_iX_2    ,"MYGLOBALS_2");
        // #pragma DATA_SECTION(_PID_iY_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_iD_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_iQ_2    ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_Position_2   ,"MYGLOBALS_2");
        #pragma DATA_SECTION(_PID_Speed_2   ,"MYGLOBALS_2");
        struct ControllerForExperiment CTRL_2;
        struct DebugExperiment debug_2;

        st_motor_parameters     t_motor_2={0};
        st_enc                  t_enc_2={0};
        st_psd                  t_psd_2={0};
        st_controller_inputs    t_I_2={0};
        st_controller_states    t_S_2={0};
        st_controller_outputs   t_O_2={0};
        st_InverterNonlinearity t_inv_2={0}; // Because of the sv_count bug, I cannot declare t_inv in this .c file. // extern st_InverterNonlinearity t_inv;
        st_capture              t_cap_2={0};
        st_global_variables     t_g_2={0};

        st_pid_regulator _PID_iD_2       = st_pid_regulator_DEFAULTS;
        st_pid_regulator _PID_iQ_2       = st_pid_regulator_DEFAULTS;
        st_pid_regulator _PID_Speed_2    = st_pid_regulator_DEFAULTS;
        st_pid_regulator _PID_Position_2 = st_pid_regulator_DEFAULTS;

        // st_PIDController _PID_iX_2 = {
        //                     SUSPENSION_PID_KP, SUSPENSION_PID_KI, SUSPENSION_PID_KD,
        //                     SUSPENSION_PID_TAU,
        //                     SUSPENSION_PID_OUT_LIMIT,
        //                     SUSPENSION_PID_INT_LIMIT, CL_TS };
        // st_PIDController _PID_iY_2 = {
        //                     SUSPENSION_PID_KP, SUSPENSION_PID_KI, SUSPENSION_PID_KD,
        //                     SUSPENSION_PID_TAU,
        //                     SUSPENSION_PID_OUT_LIMIT,
        //                     SUSPENSION_PID_INT_LIMIT, CL_TS };
    #endif
#endif

/* 初始化顶级结构体指针，指向定义好的内存空间 */
void allocate_CTRL(struct ControllerForExperiment *p){
    /* My attemp to use calloc with TI's compiler in CCS has failed. */
        // p->motor = calloc(1,sizeof(st_pmsm_parameters)); // 意思是，一个，st_pmsm_parameters那么大的空间
        // p->I = calloc(1,sizeof(st_controller_inputs));
        // p->S = calloc(1,sizeof(st_controller_states));
        // p->O = calloc(1,sizeof(st_controller_outputs));

    if(axisCnt==0){
        p->motor = &t_motor_1;
        p->enc   = &t_enc_1;
        p->psd   = &t_psd_1;
        p->i     = &t_I_1;
        p->s     = &t_S_1;
        p->o     = &t_O_1;
        p->inv   = &t_inv_1;
        p->cap   = &t_cap_1;
        p->g     = &t_g_1;
        p->s->iD  = &_PID_iD_1;
        p->s->iQ  = &_PID_iQ_1;
        p->s->Speed = &_PID_Speed_1;
        p->s->Position = &_PID_Position_1;
        // p->S->iX = &_PID_iX_1;
        // p->S->iY = &_PID_iY_1;
    }

    #if PC_SIMULATION == FALSE
        if(axisCnt==1){
            #if NUMBER_OF_AXES == 2
                p->motor = &t_motor_2;
                p->enc   = &t_enc_2;
                p->psd   = &t_psd_2;
                p->i     = &t_I_2;
                p->s     = &t_S_2;
                p->o     = &t_O_2;
                p->inv   = &t_inv_2;
                p->cap   = &t_cap_2;
                p->g     = &t_g_2;
                p->s->iD  = &_PID_iD_2;
                p->s->iQ  = &_PID_iQ_2;
                p->s->Speed = &_PID_Speed_2;
                p->s->Position = &_PID_Position_2;
                // p->S->iX = &_PID_iX_2;
                // p->S->iY = &_PID_iY_2;
            #endif
        }
    #endif
}
void init_debug(){
    debug = &debug_1;
    (*debug).error = 0;
    (*debug).who_is_user = d_sim.user.who_is_user;
    if(d_sim.init.Rreq>0){
        (*debug).mode_select = d_sim.user.mode_select_induction_motor;
    }else{
        (*debug).mode_select = d_sim.user.mode_select_synchronous_motor;
    }
        /* Open Loop  */
        // (*debug).mode_select = MODE_SELECT_PWM_DIRECT;                            //  1
        // (*debug).mode_select = MODE_SELECT_VOLTAGE_OPEN_LOOP;                     // 11
        /*  Without the Encoder */
        // (*debug).mode_select = MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE; //  2
        /* FOC */
        // (*debug).mode_select = MODE_SELECT_FOC;                                   //  3
        // (*debug).mode_select = MODE_SELECT_FOC_SENSORLESS;                        // 31
        // (*debug).mode_select = MODE_SELECT_INDIRECT_FOC;                          // 32
        /* Speed Loop  */
        // (*debug).mode_select = MODE_SELECT_VELOCITY_LOOP;                         //  4
        // (*debug).mode_select = MODE_SELECT_VELOCITY_LOOP_SENSORLESS;              // 41
        // (*debug).mode_select = MODE_SELECT_VELOCITY_LOOP_WC_TUNER;                // 43
        /* Position Loop  */
        // (*debug).mode_select = MODE_SELECT_POSITION_LOOP;                         //  5
        /* Commission  */
        // (*debug).mode_select = MODE_SELECT_COMMISSIONING;                         //  9

    (*debug).Overwrite_Current_Frequency = 50.0;
    (*debug).Overwrite_theta_d           = 0.0;

    
    (*debug).set_id_command              = 0.0;
    (*debug).set_iq_command              = d_sim.user.set_iq_command;
    (*debug).set_rpm_speed_command       = d_sim.user.set_rpm_speed_command;
    (*debug).set_deg_position_command    = 0.0;
    (*debug).vvvf_voltage = 3.0;
    (*debug).vvvf_frequency = 5.0;

    //* due to user WB's habit, make all cmd to zero to make it clear
    #if (PC_SIMULATION == FALSE) && (WHO_IS_USER == USER_WB)
    // #if (PC_SIMULATION == FALSE)
        (*debug).set_id_command              = 0.0;
        (*debug).set_iq_command              = 0.0;
        (*debug).set_rpm_speed_command       = 0.0;
        (*debug).set_deg_position_command    = 0.0; // Unit: Degree
    #endif

    (*debug).delta                                                = d_sim.FOC.delta;
    (*debug).CLBW_HZ                                              = d_sim.FOC.CLBW_HZ;
    (*debug).VL_EXE_PER_CL_EXE                                    = d_sim.FOC.VL_EXE_PER_CL_EXE;
    (*debug).LIMIT_DC_BUS_UTILIZATION                             = d_sim.CL.LIMIT_DC_BUS_UTILIZATION;
    (*debug).LIMIT_OVERLOAD_FACTOR                                = d_sim.VL.LIMIT_OVERLOAD_FACTOR;
    (*debug).Select_exp_operation                                 = d_sim.user.Select_exp_operation;
    (*debug).bool_apply_decoupling_voltages_to_current_regulation = d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation;
    (*debug).INVERTER_NONLINEARITY_COMPENSATION_INIT              = d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD;

    #if WHO_IS_USER == USER_YZZ
        (*debug).SENSORLESS_CONTROL      = d_sim.user.SENSORLESS_CONTROL;
        (*debug).SENSORLESS_CONTROL_HFSI = 0;
    #endif
    #if WHO_IS_USER == 2023231051
        //For WuBo
        (*debug).zeta                                                 = d_sim.user.zeta;
        (*debug).omega_n                                              = d_sim.user.omega_n;
        (*debug).max_CLBW_PER_min_CLBW                                = d_sim.user.max_CLBW_PER_min_CLBW;
        (*debug).bool_apply_WC_tunner_for_speed_loop                  = d_sim.user.bool_apply_WC_tunner_for_speed_loop;
        (*debug).bool_sweeping_frequency_for_speed_loop               = d_sim.user.bool_sweeping_frequency_for_speed_loop;
        (*debug).bool_Null_D_Control                                  = d_sim.user.bool_Null_D_Control;
        (*debug).bool_apply_sweeping_frequency_excitation             = d_sim.user.bool_apply_sweeping_frequency_excitation;
        (*debug).bool_Parameter_Mismatch_test                         = d_sim.user.bool_Parameter_Mismatch_test;
        //For Sweeping
        (*debug).CMD_CURRENT_SINE_AMPERE                              = d_sim.user.CMD_CURRENT_SINE_AMPERE;
        (*debug).CMD_SPEED_SINE_RPM                                   = d_sim.user.CMD_SPEED_SINE_RPM;
        (*debug).CMD_SPEED_SINE_HZ                                    = d_sim.user.CMD_SPEED_SINE_HZ;
        (*debug).CMD_SPEED_SINE_STEP_SIZE                             = d_sim.user.CMD_SPEED_SINE_STEP_SIZE;
        (*debug).CMD_SPEED_SINE_LAST_END_TIME                         = d_sim.user.CMD_SPEED_SINE_LAST_END_TIME;
        (*debug).CMD_SPEED_SINE_END_TIME                              = d_sim.user.CMD_SPEED_SINE_END_TIME;
        (*debug).CMD_SPEED_SINE_HZ_CEILING                            = d_sim.user.CMD_SPEED_SINE_HZ_CEILING;
    #endif
}
void init_CTRL(){
    allocate_CTRL(CTRL);

    /* Basic quantities */
    (*CTRL).timebase = 0.0;

    /* Machine parameters */
    // elec
    (*CTRL).motor->R  = d_sim.init.R;
    (*CTRL).motor->KE = d_sim.init.KE; // * (0.1/0.1342); // 【实验编号：】
    (*CTRL).motor->Ld = d_sim.init.Ld;
    (*CTRL).motor->Lq = d_sim.init.Lq;
    (*CTRL).motor->Ld_inv = 1.0 / (*CTRL).motor->Ld;
    (*CTRL).motor->Lq_inv = 1.0 / (*CTRL).motor->Lq;
    (*CTRL).motor->DeltaL = (*CTRL).motor->Ld - (*CTRL).motor->Lq; // for IPMSM or IM.Lmu
    (*CTRL).motor->KActive = (*CTRL).motor->KE;                    // TODO:
    (*CTRL).motor->Rreq = d_sim.init.Rreq;
    // mech
    (*CTRL).motor->npp = d_sim.init.npp;
    (*CTRL).motor->npp_inv = 1.0 / (*CTRL).motor->npp;
    (*CTRL).motor->Js = d_sim.init.Js;
    (*CTRL).motor->Js_inv = 1.0 / (*CTRL).motor->Js;
    // /* Peripheral configurations */

    /* Inverter */
    // (*CTRL).inv->filter_pole = 3000 * 2 * M_PI;
    // // inverterNonlinearity_Initialization();
    // G.FLAG_INVERTER_NONLINEARITY_COMPENSATION = (*debug).INVERTER_NONLINEARITY_COMPENSATION_INIT;
        // G.FLAG_TUNING_CURRENT_SCALE_FACTOR = TUNING_CURRENT_SCALE_FACTOR_INIT;

    /* Console */
    // See init_experiment_overwrite() in CJHMainISR.c
    G.flag_do_inverter_characteristics = 1;
    G.overwrite_vdc = 20;

    /* Black Box Model | Controller quantities */

    // commands
    (*CTRL).i->cmd_psi = d_sim.init.KE;

    (*CTRL).s->xRho = 0.0;
    (*CTRL).s->cosT = 1.0;
    (*CTRL).s->sinT = 0.0;
    (*CTRL).s->cosT_compensated_1p5omegaTs = 1.0;
    (*CTRL).s->sinT_compensated_1p5omegaTs = 0.0;
    (*CTRL).s->cosT2 = 1.0;
    (*CTRL).s->sinT2 = 0.0;
    (*CTRL).s->omega_syn = 0.0;
    (*CTRL).s->the_vc_count = 1;


    /* Controller Parameter Initializaiton */
    // TODO: 在这里可以tune你的pi系数
    // KT = 1.5*npp*KE
    // d_currentKp = CLBW_Hz * 2 * np.pi * Ld
    // d_currentKi = R / Ld
    // q_currentKp = CLBW_Hz * 2 * np.pi * Lq
    // q_currentKi = R / Lq
    // speedKi = 2*np.pi * CLBW_Hz / delta**2  # THIS IS INTEGRAL GAIN
    // speedKp = delta * speedKi / KT * Js     # 这里不需要npp

    #if WHO_IS_USER == USER_WB
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
        PID_Position->Kp       = d_sim.user.Position_Loop_Kp;
        PID_Position->Ki_CODE  = 0.0;
        PID_Position->Kd       = 0.0;
        PID_Position->OutLimit = d_sim.user.Position_Output_Limit;
        PID_Position->Out      = 0.0;
        /* WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING*/
    #endif

    PID_Speed->Kp = d_sim.VL.SERIES_KP;
    PID_Speed->Ki_CODE = d_sim.VL.SERIES_KI      * d_sim.VL.SERIES_KP        * VL_TS            *( (int)!d_sim.user.Set_SpeedLoop_KI_as_Zero );
    PID_Speed->OutLimit = d_sim.VL.LIMIT_OVERLOAD_FACTOR * d_sim.init.IN;
    PID_iD->Kp  = d_sim.CL.SERIES_KP_D_AXIS;
    PID_iQ->Kp  = d_sim.CL.SERIES_KP_Q_AXIS;
    #if CURRENT_LOOP_KI_TIMES_TEN
        PID_iD->Ki_CODE  = d_sim.CL.SERIES_KI_D_AXIS * d_sim.CL.SERIES_KP_D_AXIS * CL_TS * 10;
        PID_iQ->Ki_CODE  = d_sim.CL.SERIES_KI_Q_AXIS * d_sim.CL.SERIES_KP_Q_AXIS * CL_TS * 10;
    #else
        PID_iD->Ki_CODE  = d_sim.CL.SERIES_KI_D_AXIS * d_sim.CL.SERIES_KP_D_AXIS * CL_TS;
        PID_iQ->Ki_CODE  = d_sim.CL.SERIES_KI_Q_AXIS * d_sim.CL.SERIES_KP_Q_AXIS * CL_TS;
    #endif
    PID_iD->OutLimit  = 0.57735 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc; // TODO：初始化的时候如果没有给母线供电这里会有问题，但在中断里如果持续刷新限幅就没事。
    PID_iQ->OutLimit  = 0.57735 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc; // TODO：初始化的时候如果没有给母线供电这里会有问题，但在中断里如果持续刷新限幅就没事。
    /* Capture */
    (*CTRL).cap->flag_nonlinear_filtering = FALSE;
    (*CTRL).cap->flag_bad_U_capture = FALSE;
    (*CTRL).cap->flag_bad_V_capture = FALSE;
    (*CTRL).cap->flag_bad_W_capture = FALSE;
    (*CTRL).cap->good_capture_U[0] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_U[1] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_U[2] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_U[3] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[0] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[1] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[2] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_V[3] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[0] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[1] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[2] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->good_capture_W[3] = SYSTEM_HALF_PWM_MAX_COUNT;
    (*CTRL).cap->ECapIntCount[0] = 0;
    (*CTRL).cap->ECapIntCount[1] = 0;
    (*CTRL).cap->ECapIntCount[2] = 0;
    (*CTRL).cap->ECapPassCount[0] = 0;
    (*CTRL).cap->ECapPassCount[1] = 0;
    (*CTRL).cap->ECapPassCount[2] = 0;

}
void overwrite_d_sim(){
}
void init_experiment(){

    // init_d_sim();   // initilizating d_sim is removed into main.c to execute only once
    // init_debug;   // initilizating debug is removed into main.c to execute only once
    overwrite_d_sim(); // overwrite d_sim with user's algorithm
    init_CTRL(); // 控制器结构体初始化

    //OFSR
    init_rk4();
    //ESO
    init_esoaf();
    #if WHO_IS_USER == USER_BEZIER
        set_points(&BezierVL);
        set_points(&BezierVL_AdaptVersion);
        // set_points_cl(&BezierCL);
        // set_points_cl(&BezierCL_AdaptVersion);
    #endif

    #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        init_FE();  // flux estimator
        rk4_init(); // 
        // observer_init();
        init_pmsm_observers(); // 
        inverterNonlinearity_Initialization();
    #endif

    #if WHO_IS_USER == USER_CJH
        init_im_controller();
    #endif

    #if WHO_IS_USER == USER_WB
        /* init here will tune a new PID value. Make sure this init run after init_CTRL() */
        // This should be placed at the front of init_WC_Tuner() to make sure ParaMis do not overwrite the MotorParameters
        _init_wubo_SignalGE();
        _init_wubo_Hit_Wall();
        _init_wubo_ParaMis();
        if (d_sim.user.bool_apply_WC_tunner_for_speed_loop == TRUE){
            _init_WC_Tuner();
        }
        _init_Harnerfors_1998_BackCalc(); // should be placed after init_wctuner, cuz it needs to use the variable from wctuner
    #endif
}
/* 公用的核心电机控制实现代码，不要修改！*/
void incremental_PI(st_pid_regulator *r){
    r->Err = r->Ref - r->Fbk;
    r->Out = r->OutPrev + r->Kp * ( r->Err - r->ErrPrev ) + r->Ki_CODE * r->Err;
    if(r->Out > r->OutLimit) r->Out = r->OutLimit;
    else if(r->Out < -r->OutLimit) r->Out = -r->OutLimit;
    r->ErrPrev = r->Err;
    r->OutPrev = r->Out;
}

void tustin_PI(st_pid_regulator *r){
    #define DYNAMIC_CLAPMING TRUE
    r->Err = r->Ref - r->Fbk;// 误差
    r->P_Term = r->Err * r->Kp;    // 比例
    r->I_Term += r->Err * r->Ki_CODE;    // 积分
    r->OutNonSat = r->I_Term;

    // 添加积分饱和特性
    #if DYNAMIC_CLAPMING
        // dynamic clamping
        if( r->I_Term > r->OutLimit - r->P_Term)     /* BUGGY if use r->Out instead of r->P_Term!!! */
            r->I_Term = r->OutLimit - r->P_Term;
        else if( r->I_Term < -r->OutLimit + r->P_Term)
            r->I_Term =      -r->OutLimit + r->P_Term; // OutLimit is a positive constant
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
REAL _veclocityController(REAL cmd_varOmega, REAL varOmega){
    /* 想清楚你的速度控制器到底要不要主动降频！ */
    if ((*CTRL).s->the_vc_count++ >= d_sim.FOC.VL_EXE_PER_CL_EXE){
        (*CTRL).s->the_vc_count = 1;
        PID_Speed->Ref = cmd_varOmega;
        PID_Speed->Fbk = varOmega;
        /* Here is the algorithem*/
        #if WHO_IS_USER == USER_BEZIER
            control_output(PID_Speed, &BezierVL);
        #elif WHO_IS_USER == USER_WB
            if(d_sim.user.bool_apply_WC_tunner_for_speed_loop) _user_wubo_SpeedInnerLoop_controller(PID_Speed, &SIL_Controller);
            else PID_Speed->calc(PID_Speed);
        #else
            PID_Speed->calc(PID_Speed);
        #endif
    }   
    return PID_Speed->Out;
}

#if WHO_USER == USER_YZZ
REAL _RK4_veclocityController(REAL cmd_varOmega, REAL varOmega){
    /* 想清楚你的速度控制器到底要不要主动降频！ */
    if ((*CTRL).s->the_vc_count++ >= SPEED_LOOP_CEILING){
        (*CTRL).s->the_vc_count = 1;
        PID_Speed->Ref = cmd_varOmega;
        PID_Speed->Fbk = varOmega;
        /* Here is the algorithem*/
        General_PI_Dynamics(PID_Speed, &rhf_PI_DynamicsforSpeed);
    }
    return PID_Speed->Out;
}
#endif

void FOC_with_vecocity_control(REAL theta_d_elec, REAL varOmega, REAL cmd_varOmega, REAL cmd_iDQ[2], REAL iAB[2]){
    /* Default is the Null D control */
    cmd_iDQ[0] = 0;
    cmd_iDQ[1] = _veclocityController(cmd_varOmega, varOmega);

    /* FOC */
    #if WHO_IS_USER == USER_WB
        if (d_sim.user.bool_enable_Harnefors_back_calculation) _user_wubo_FOC( (*CTRL).i->theta_d_elec, iAB );
        else {
            d_sim.user.Check_Harnerfors_1998_On = -1;
            _onlyFOC( (*CTRL).i->theta_d_elec, iAB );
        }
    #else
        _onlyFOC(theta_d_elec, iAB, varOmega );
    #endif
}


void _pseudoEncoder(){
    /* 断开编码器，开环控制电流矢量旋转、跳跃，逆（变器）闭着眼 */
    (*CTRL).i->cmd_iDQ[0] = (*debug).set_id_command;
    (*CTRL).i->cmd_iDQ[1] = (*debug).set_iq_command;
    if (fabsf((*debug).Overwrite_Current_Frequency) > 0)
    {
        (*debug).Overwrite_theta_d += CL_TS * (*debug).Overwrite_Current_Frequency * 2 * M_PI;
        if ((*debug).Overwrite_theta_d > M_PI)  (*debug).Overwrite_theta_d -= 2 * M_PI;
        if ((*debug).Overwrite_theta_d < -M_PI) (*debug).Overwrite_theta_d += 2 * M_PI;
    }
    else
    {
        (*debug).Overwrite_theta_d = 0.0;
    }
    _onlyFOC((*debug).Overwrite_theta_d, (*CTRL).i->iAB, (*CTRL).i->varOmega);
}
#if WHO_USER == USER_YZZ

void RK4_FOC_with_vecocity_control(REAL theta_d_elec, REAL varOmega, REAL cmd_varOmega, REAL cmd_iDQ[2], REAL iAB[2]){
    /* Default is the Null D control */
    cmd_iDQ[0] = 0;
    cmd_iDQ[1] = _RK4_veclocityController(cmd_varOmega, varOmega);

    /* FOC */
    _RK4_PI_Controller_FOC((*CTRL).i->theta_d_elec, iAB);
}

void rhf_PI_DynamicsforSpeed(REAL t, REAL *x, REAL *fx){
    fx[0] = 15 * CTRL->s->Speed->Err;
}   

void rhf_PI_DynamicsforQcurrent(REAL t, REAL *x, REAL *fx){
    // fx[0] = CTRL->s->iQ->Ki_CODE * CTRL->s->iQ->Err;
    fx[0] = 1500 * CTRL->s->iQ->Err;
}   

void rhf_PI_DynamicsforDcurrent(REAL t, REAL *x, REAL *fx){
    // fx[0] = CTRL->s->iD->Ki_CODE * CTRL->s->iD->Err;
    fx[0] = 1500 * CTRL->s->iD->Err;
}   

void General_PI_Dynamics(st_pid_regulator *r, void (*dynamic_func)(REAL, REAL *, REAL *)) {
    r->Err = r->Ref - r->Fbk;
    // 调用 RK4 数值积分器来计算积分项
    general_1states_rk4_solver(dynamic_func, (*CTRL).timebase, &(r->I_Term), CL_TS);
    // PI 控制器输出计算
    r->Out = r->Kp * r->Err + r->I_Term;
    // 限幅操作
    if (r->Out > r->OutLimit) r->Out = r->OutLimit;
    else if (r->Out < -r->OutLimit) r->Out = -r->OutLimit;
}

void _RK4_PI_Controller_FOC(REAL theta_d_elec, REAL iAB[2]){
    (*CTRL).s->cosT = cos(theta_d_elec);
    (*CTRL).s->sinT = sin(theta_d_elec);
    (*CTRL).i->iDQ[0] = AB2M(iAB[0], iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T(iAB[0], iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
    PID_iQ->Fbk = (*CTRL).i->iDQ[1];
    PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
    General_PI_Dynamics(CTRL->s->iQ, &rhf_PI_DynamicsforQcurrent);
    General_PI_Dynamics(CTRL->s->iD, &rhf_PI_DynamicsforDcurrent);
        // 电流环前馈DQ轴解耦
    REAL decoupled_d_axis_voltage;
    REAL decoupled_q_axis_voltage;
    if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
        decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * (*CTRL).i->varOmega * MOTOR.npp;
        decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KActive + PID_iD->Fbk * MOTOR.Ld) * (*CTRL).i->varOmega * MOTOR.npp;
    }else{
        decoupled_d_axis_voltage = PID_iD->Out;
        decoupled_q_axis_voltage = PID_iQ->Out;
    }
    if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
    else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
    if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
    else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;
    (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    /* 7. 反帕克变换 */
    // See D:\Users\horyc\Downloads\Documents\2003 TIA Bae SK Sul A compensation method for time delay of.pdf
    // (*CTRL).s->cosT_compensated_1p5omegaTs = cosf(used_theta_d_elec + 1.5omg_elec*CL_TS);
    // (*CTRL).s->sinT_compensated_1p5omegaTs = sinf(used_theta_d_elec + 1.5omg_elec*CL_TS);
    (*CTRL).s->cosT_compensated_1p5omegaTs = (*CTRL).s->cosT;
    (*CTRL).s->sinT_compensated_1p5omegaTs = (*CTRL).s->sinT;
    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
    (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];

    (*CTRL).o->dc_bus_utilization_ratio = DC_BUS_VOLTAGE_INVERSE * sqrtf( (*CTRL).o->cmd_uAB_to_inverter[0]
                                                                        * (*CTRL).o->cmd_uAB_to_inverter[0]
                                                                        + (*CTRL).o->cmd_uAB_to_inverter[1]
                                                                        * (*CTRL).o->cmd_uAB_to_inverter[1] );
}
#endif

void _onlyFOC(REAL theta_d_elec, REAL iAB[2], REAL varOmega){
    // 帕克变换
    (*CTRL).s->cosT = cos(theta_d_elec);
    (*CTRL).s->sinT = sin(theta_d_elec);
    (*CTRL).i->iDQ[0] = AB2M(iAB[0], iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T(iAB[0], iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    /* 更新依赖于dq轴电流的物理量 */
    REAL Tem     = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0]) * (*CTRL).i->iDQ[1];     // 转矩 For luenberger position observer for HFSI
    REAL cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->cmd_iDQ[0]) * (*CTRL).i->cmd_iDQ[1];
    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];
    // if(CTRL->timebase > 1.0){
    //     if(d_sim.user.ell_feedback_on == 1){
    //         MOTOR.KActive = FE.no_sat.ell_1 + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];
    //     }else if(d_sim.user.ell_feedback_on == 2){
    //         MOTOR.KActive = FE.Bernard.psi_PM + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];
    //     }
    // }
    #if USE_LAOMING_PI
        /* New Sytle from Lao Ming */
        pi_id.Kp = PID_iD->Kp;
        pi_id.Ki = d_sim.CL.SERIES_KI_D_AXIS * CL_TS;
        pi_id.Umax = PID_iD->OutLimit;
        pi_id.Umin = -PID_iD->OutLimit;
        // printf("id max is %f\n", pi_id.Umax);
        
        pi_iq.Kp = PID_iQ->Kp;
        pi_iq.Ki = d_sim.CL.SERIES_KI_Q_AXIS * CL_TS;
        pi_iq.Umax = PID_iQ->OutLimit;
        pi_iq.Umin = -PID_iQ->OutLimit;
        
        pi_id.Fbk = (*CTRL).i->iDQ[0];
        pi_id.Ref = (*CTRL).i->cmd_iDQ[0];
        pi_id.Out = PI_MACRO(pi_id);

        pi_iq.Fbk = (*CTRL).i->iDQ[1];
        pi_iq.Ref = (*CTRL).i->cmd_iDQ[1];
        pi_iq.Out = PI_MACRO(pi_iq);

        REAL decoupled_d_axis_voltage;
        REAL decoupled_q_axis_voltage;
        if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
            decoupled_d_axis_voltage = pi_id.Out - pi_iq.Fbk * MOTOR.Lq * (*CTRL).i->varOmega * MOTOR.npp;
            decoupled_q_axis_voltage = pi_iq.Out + (MOTOR.KActive + pi_id.Fbk * MOTOR.Ld) * (*CTRL).i->varOmega * MOTOR.npp;
        }else{
            decoupled_d_axis_voltage = pi_id.Out;
            decoupled_q_axis_voltage = pi_iq.Out;
        }
    #else
        /* D-Axis Current Loop */
        PID_iD->Fbk = (*CTRL).i->iDQ[0];
        PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
        PID_iD->calc(PID_iD);

        /* Q-Axis Current Loop */
        #if FALSE
            PID_iQ->Fbk = (*CTRL).i->iDQ[1];
            PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
            PID_iQ->OutLimit = BezierCL.points[BezierCL.order].y;
            control_output(PID_iQ, &BezierCL);
            // control_output_adaptVersion(PID_iQ, &BezierCL);
        #else
            PID_iQ->Fbk = (*CTRL).i->iDQ[1];
            PID_iQ->Ref = (*CTRL).i->cmd_iDQ[1];
            PID_iQ->calc(PID_iQ);
        #endif

        // 电流环前馈DQ轴解耦
        REAL decoupled_d_axis_voltage;
        REAL decoupled_q_axis_voltage;
        if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
            decoupled_d_axis_voltage = PID_iD->Out - PID_iQ->Fbk * MOTOR.Lq * varOmega * MOTOR.npp;
            decoupled_q_axis_voltage = PID_iQ->Out + (MOTOR.KActive + PID_iD->Fbk * MOTOR.Ld) * varOmega * MOTOR.npp;
        }else{
            decoupled_d_axis_voltage = PID_iD->Out;
            decoupled_q_axis_voltage = PID_iQ->Out + MOTOR.KActive * varOmega * MOTOR.npp;
            // decoupled_d_axis_voltage = PID_iD->Out;
            // decoupled_q_axis_voltage = PID_iQ->Out;
        }
    #endif

    /* 对补偿后的dq轴电压进行限幅度 */
    if (decoupled_d_axis_voltage > PID_iD->OutLimit) decoupled_d_axis_voltage = PID_iD->OutLimit;
    else if (decoupled_d_axis_voltage < -PID_iD->OutLimit) decoupled_d_axis_voltage = -PID_iD->OutLimit;
    if (decoupled_q_axis_voltage > PID_iQ->OutLimit) decoupled_q_axis_voltage = PID_iQ->OutLimit;
    else if (decoupled_q_axis_voltage < -PID_iQ->OutLimit) decoupled_q_axis_voltage = -PID_iQ->OutLimit;
    (*CTRL).o->cmd_uDQ[0] = decoupled_d_axis_voltage;
    (*CTRL).o->cmd_uDQ[1] = decoupled_q_axis_voltage;

    /* 7. 反帕克变换 */
    // See D:\Users\horyc\Downloads\Documents\2003 TIA Bae SK Sul A compensation method for time delay of.pdf
    // (*CTRL).s->cosT_compensated_1p5omegaTs = cosf(used_theta_d_elec + 1.5omg_elec*CL_TS);
    // (*CTRL).s->sinT_compensated_1p5omegaTs = sinf(used_theta_d_elec + 1.5omg_elec*CL_TS);
    (*CTRL).s->cosT_compensated_1p5omegaTs = (*CTRL).s->cosT;
    (*CTRL).s->sinT_compensated_1p5omegaTs = (*CTRL).s->sinT;
    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
    (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];
    (*CTRL).o->cmd_iAB[0] = MT2A((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);
    (*CTRL).o->cmd_iAB[1] = MT2B((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT_compensated_1p5omegaTs, (*CTRL).s->sinT_compensated_1p5omegaTs);

    #if WHO_IS_USER == USER_YZZ
        yzz_inverter_Compensation_Online_PAA();
    #endif

    (*CTRL).o->dc_bus_utilization_ratio = DC_BUS_VOLTAGE_INVERSE * sqrtf( (*CTRL).o->cmd_uAB_to_inverter[0]
                                                                        * (*CTRL).o->cmd_uAB_to_inverter[0]
                                                                        + (*CTRL).o->cmd_uAB_to_inverter[1]
                                                                        * (*CTRL).o->cmd_uAB_to_inverter[1] );

    /// 8. 补偿逆变器非线性
    #if WHO_IS_USER == USER_WB
        /* wubo:  */
        wubo_inverter_Compensation( (*CTRL).i->iAB );
    #endif
    #if WHO_IS_USER == USER_CJH
        /* For scope only */
        #if PC_SIMULATION
            REAL ualbe_dist[2];
            get_distorted_voltage_via_CurveFitting((*CTRL).o->cmd_uAB[0], (*CTRL).o->cmd_uAB[1], Ia, Ib, ualbe_dist);
            INV.ual_comp = ualbe_dist[0];
            INV.ube_comp = ualbe_dist[1];
        #endif

        /* not used */
        (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
        (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    #endif
}
void _user_commands(){
    /* RPM GIVEN */
    (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;

    if (CTRL->motor->Rreq > 0){
        // 感应电机需要励磁
        (*CTRL).i->cmd_iDQ[0] = 2.0;

    }else{
        // 表贴永磁采用 iD=0 控制
        (*CTRL).i->cmd_iDQ[0] = 0.0;
        // (*CTRL).i->cmd_iDQ[0] = -20.0;

        // 凸极永磁采用 iD<0 获得更大的 有功磁链（aka 转矩系数）
        // (*CTRL).i->cmd_iDQ[0] = -1.0;
    }
    

    #if PC_SIMULATION == TRUE
        #if WHO_IS_USER == USER_WB
            ACM.TLoad = 0;
            if ( (*CTRL).timebase > 0.04 ){
                ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.5);
            }
            if( (*CTRL).timebase >  0.10 ){
                ACM.TLoad = 0;
                (*CTRL).i->cmd_varOmega = 400 * RPM_2_MECH_RAD_PER_SEC;
            }
        #elif WHO_IS_USER == USER_BEZIER
            if ((*CTRL).timebase > 0){
                (*CTRL).i->cmd_varOmega =  400 * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 0.02){
                (*CTRL).i->cmd_varOmega = -400 * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 0.04){
                ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * 3.0 *0.5);
                // ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * 3.0 * 0.95) * sin(50*2*M_PI*CTRL->timebase);
            }
            if ((*CTRL).timebase > 0.07){
                ACM.TLoad = 0.0;
            }
        #elif WHO_IS_USER == USER_CJH || WHO_IS_USER == USER_XM
            (*CTRL).i->cmd_varOmega = 0.0;

            if ((*CTRL).timebase > CL_TS){
                (*CTRL).i->cmd_varOmega =  400 * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 0.04){
                (*CTRL).i->cmd_varOmega = -400 * RPM_2_MECH_RAD_PER_SEC;
            }
            if ((*CTRL).timebase > 0.07){
                ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * 3.0 * 0.95);
                // ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * 3.0 * 0.95) * sin(50*2*M_PI*CTRL->timebase);
            }
            if ((*CTRL).timebase > 0.10){
                ACM.TLoad = 0.0;
            }
            // (*CTRL).i->cmd_varOmega = 0.0;
            // if ((*CTRL).timebase > CL_TS){
            //     (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
            // if ((*CTRL).timebase > 0.2){
            //     (*CTRL).i->cmd_varOmega =  (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
            // if ((*CTRL).timebase > 0.4){
            //     #if PC_SIMULATION
            //         ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.8);
            //     #endif
            // }
            // if ((*CTRL).timebase > 10){
            //     (*CTRL).i->cmd_varOmega = 0.0;
            // }
            // if ((*CTRL).timebase > 15){
            //     (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }

        #elif WHO_IS_USER == USER_YZZ
            (*CTRL).i->cmd_varOmega = 0.0;
            if ((*CTRL).timebase > CL_TS){
                (*CTRL).i->cmd_varOmega = (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
                #if PC_SIMULATION
                    ACM.TLoad = 0.02 * (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                #endif
            }
            if ((*CTRL).timebase > 0.5){
                // #if PC_SIMULATION
                //     ACM.TLoad = 0.3 * (0.5 * 1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.95);
                // #endif
            }

            // if ((*CTRL).timebase > 5){
            //     (*CTRL).i->cmd_varOmega = 0.5 * (*debug).set_rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
            // }
        #endif
    #endif

    /* 扫频覆盖 */
    overwrite_sweeping_frequency();
}


void overwrite_sweeping_frequency(){
    #if WHO_IS_USE == USER_WB
        //这句话应该放在最前面！
            d_sim.user.timebase_for_Sweeping += CL_TS; // Separate the timebase with the DSP timebase !!!

        #if PC_SIMULATION
            ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * 3.0 * 0.5); // 强制将负载设置为0    
        #endif

        if(d_sim.user.bool_apply_sweeping_frequency_excitation){

            if (d_sim.user.bool_speed_sweeping_with_Load == TRUE){
                //前XXX秒开启恒速模式，以使得系统达到稳态
                if ( (d_sim.user.timebase_for_Sweeping < d_sim.user.Stable_Time_for_Sweeping) && (d_sim.user.flag_clear_timebase_once == FALSE)  ){
                    // (*CTRL).i->cmd_varOmega = 0.5 * d_sim.user.CMD_SPEED_SINE_RPM * RPM_2_MECH_RAD_PER_SEC;
                    (*CTRL).i->cmd_varOmega = 0.0;
                    return;
                }
                if( d_sim.user.flag_clear_timebase_once == FALSE ){
                    d_sim.user.timebase_for_Sweeping = 0.0000; // Clear the time, suitable for generating the Sine Signal
                    d_sim.user.flag_clear_timebase_once = TRUE;
                }
            }

            // 生成扫频信号
            if ( d_sim.user.timebase_for_Sweeping  > d_sim.user.CMD_SPEED_SINE_END_TIME ){
                d_sim.user.CMD_SPEED_SINE_HZ += d_sim.user.CMD_SPEED_SINE_STEP_SIZE;
                d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = d_sim.user.CMD_SPEED_SINE_END_TIME;
                d_sim.user.CMD_SPEED_SINE_END_TIME += 1.0 / d_sim.user.CMD_SPEED_SINE_HZ;
            }
            if (d_sim.user.CMD_SPEED_SINE_HZ > d_sim.user.CMD_SPEED_SINE_HZ_CEILING){
                (*CTRL).i->cmd_varOmega = 0.0; // 到达扫频的频率上限，速度归零
                (*CTRL).i->cmd_iDQ[0] = 0.0;
                (*CTRL).i->cmd_iDQ[1] = 0.0;
            }else{
                if (d_sim.user.bool_sweeping_frequency_for_speed_loop == TRUE){
                    (*CTRL).i->cmd_varOmega = RPM_2_MECH_RAD_PER_SEC * d_sim.user.CMD_SPEED_SINE_RPM \
                        *sin(2*M_PI*d_sim.user.CMD_SPEED_SINE_HZ*(d_sim.user.timebase_for_Sweeping  - d_sim.user.CMD_SPEED_SINE_LAST_END_TIME));
                }else{
                    if (d_sim.user.bool_sweeping_frequency_for_current_loop_iD == TRUE){
                        (*CTRL).i->cmd_iDQ[0] = d_sim.user.CMD_CURRENT_SINE_AMPERE * sin(2* M_PI *d_sim.user.CMD_SPEED_SINE_HZ*(d_sim.user.timebase_for_Sweeping  - d_sim.user.CMD_SPEED_SINE_LAST_END_TIME));
                        (*CTRL).i->cmd_iDQ[1] = 0.0;
                    } else {
                        (*CTRL).i->cmd_iDQ[0] = 0.0;
                        (*CTRL).i->cmd_iDQ[1] = d_sim.user.CMD_CURRENT_SINE_AMPERE * sin(2* M_PI *d_sim.user.CMD_SPEED_SINE_HZ*(d_sim.user.timebase_for_Sweeping  - d_sim.user.CMD_SPEED_SINE_LAST_END_TIME));
                    }
                }
            }
        }
    #endif
}

void _user_Check_ThreeDB_Point( REAL Fbk, REAL Ref){
    #if WHO_IS_USE == USER_WB
        if( Fbk < 0.707 * Ref ){
            d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point = 1;
        }
        if( Fbk >= 0.707 * Ref && d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point == 1 ){
            d_sim.user.Mark_Sweeping_Freq_ThreeDB_Point = 0;
            d_sim.user.Mark_Counter += 1;
        }
        #if PC_SIMULATION
            static int flag_print_only_once = FALSE;
            // if (d_sim.user.bool_apply_sweeping_frequency_excitation && ( (*CTRL).timebase > 4.500 ) && ( flag_print_only_once == FALSE ) ){
            if (d_sim.user.bool_apply_sweeping_frequency_excitation && ( d_sim.user.CMD_SPEED_SINE_HZ >= 200 ) && ( flag_print_only_once == FALSE ) ){
                printf("VLBW is %fHz\n", d_sim.user.Mark_Counter);
                flag_print_only_once = TRUE;
            }
        #endif
    #endif
}


void _user_inverter_voltage_command(int bool_use_cmd_iAB){
    (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
    (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];
    /* We use cmd_iAB instead of iAB to look-up */
    REAL Ia, Ib;
    if (bool_use_cmd_iAB){
        Ia = (*CTRL).o->cmd_iAB[0];
        Ib = (*CTRL).o->cmd_iAB[1];
    }else{
        Ia = (*CTRL).i->iAB[0];
        Ib = (*CTRL).i->iAB[1];
    }
}
/* MAIN SWITCH as per MODE_SELECT */
int  main_switch(long mode_select){
    static long mode_select_last = 0;
    static int mode_initialized = FALSE;
    if(mode_select != mode_select_last) mode_initialized = FALSE;
    switch (mode_select){
    case MODE_SELECT_PWM_DIRECT: // 1
        if(mode_initialized == FALSE){
            mode_initialized = TRUE;
            (*CTRL).svgen1.Ta = 0.5;
            (*CTRL).svgen1.Tb = 0.5;
            (*CTRL).svgen1.Tc = 0.5;
        }
        mode_select_last = mode_select; // return 5 makes line 809 can not work properly hence we need to add this code here
        return 5; // set Axis->Select_exp_operation to 5 in experiment
        break;
    case MODE_SELECT_VOLTAGE_OPEN_LOOP: // 11
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*debug).vvvf_voltage * cos((*debug).vvvf_frequency*2*M_PI* CTRL->timebase);
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*debug).vvvf_voltage * sin((*debug).vvvf_frequency*2*M_PI* CTRL->timebase);
        break;
    case MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE: // 2
        if(mode_initialized == FALSE){
            mode_initialized = TRUE;
            // TODO: add your default setup
        }
        _pseudoEncoder();
        break;
    case MODE_SELECT_FOC: // 3
        (*CTRL).i->cmd_iDQ[0] = (*debug).set_id_command;
        (*CTRL).i->cmd_iDQ[1] = (*debug).set_iq_command;
        #if PC_SIMULATION
            // ACM.TLoad = 1.0 * sin((*CTRL).i->cmd_varOmega * d_sim.init.npp * CTRL->timebase);
        #endif

        #if WHO_IS_USER == USER_WB
            if ( d_sim.user.bool_enable_Harnefors_back_calculation == TRUE ){
                _user_wubo_FOC( (*CTRL).i->theta_d_elec, (*CTRL).i->iAB );
            }
            else{
                _onlyFOC((*CTRL).i->theta_d_elec, (*CTRL).i->iAB);
            }
        #else
            _onlyFOC((*CTRL).i->theta_d_elec, (*CTRL).i->iAB, (*CTRL).i->varOmega);
        #endif

        #if WHO_IS_USER == USER_BEZIER
            if (d_sim.user.bezier_Give_Sweeping_Ref_in_Interrupt){
                // #if PC_SIMULATION
                //     printf("Bezier Sweeping Ref is given at the Interruput!\n");
                // #endif
                if (d_sim.user.bool_apply_sweeping_frequency_excitation == TRUE){
                    overwrite_sweeping_frequency();
                }else{
                    _user_commands();
                } //TODO: add spd ref here?????
                /* Mark -3db points */
                _user_Check_ThreeDB_Point( (*CTRL).i->varOmega*MECH_RAD_PER_SEC_2_RPM, d_sim.user.CMD_SPEED_SINE_RPM );
            }
            if (d_sim.user.bool_ESO_SPEED_ON == TRUE){
                Main_esoaf_chen2021();
            }
        #endif

        break;
    case MODE_SELECT_FOC_SENSORLESS: //31
        #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
            US_P(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_P(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            US_C(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_C(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            IS_C(0)           = (*CTRL).i->iAB[0];
            IS_C(1)           = (*CTRL).i->iAB[1];
        #endif
            US_SR_P(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_P(1) = (*CTRL).o->cmd_uAB[1];
            US_SR_C(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_C(1) = (*CTRL).o->cmd_uAB[1];
            IS_SR_C(0) = (*CTRL).i->iAB[0];
            IS_SR_C(1) = (*CTRL).i->iAB[1];
        (*CTRL).i->cmd_iDQ[0] = (*debug).set_id_command; // SWEEP_FREQ_C2C
        (*CTRL).i->cmd_iDQ[1] = (*debug).set_iq_command;
        
        #if WHO_IS_USER == USER_YZZ
        // _user_commands();
        pmsm_observers();
        // observer_PMSMife();
        // controller_PMSMife_with_commands();
            OBSV.theta_d = (*CTRL).i->theta_d_elec;
            while(OBSV.theta_d > M_PI) OBSV.theta_d  -= 2*M_PI;
            while(OBSV.theta_d < -M_PI) OBSV.theta_d += 2*M_PI;
        if(d_sim.user.sensorless_only_theta_on  == 0){
            _onlyFOC((*CTRL).i->theta_d_elec, (*CTRL).i->iAB, (*CTRL).i->varOmega);
        }else if(d_sim.user.sensorless_only_theta_on  == 5){
            #if AFE_25_VM_CM_FUSION
            _onlyFOC(FE.AFEOE.theta_d, (*CTRL).i->iAB);
            #endif
        }
        #endif

        break;
    case MODE_SELECT_INDIRECT_FOC:   // 32
        _user_commands();         // 用户指令
        #if (WHO_IS_USER == USER_CJH)
            controller_IFOC();
        #endif
        break;
    case MODE_SELECT_ID_SWEEPING_FREQ: // 33
        #if WHO_IS_USER == USER_WB
            INNER_LOOP_SENSITIVITY_ANALYSIS(debug);
            (*debug).set_id_command = wubo_Signal_Generator(GENERATE_D_CURRENT_SINE);
        #endif
        (*CTRL).i->cmd_iDQ[0] = (*debug).set_id_command; // SWEEP_FREQ_C2C
        (*CTRL).i->cmd_iDQ[1] = 0.0;
        #if WHO_IS_USER == USER_WB
            _user_wubo_FOC( (*CTRL).i->theta_d_elec, (*CTRL).i->iAB );
        #else
            _onlyFOC((*CTRL).i->theta_d_elec, (*CTRL).i->iAB, (*CTRL).i->varOmega);
        #endif
        break;
    case MODE_SELECT_IQ_SWEEPING_FREQ: // 34
        #if WHO_IS_USER == USER_WB
            INNER_LOOP_SENSITIVITY_ANALYSIS(debug);
            (*debug).set_iq_command = wubo_Signal_Generator(GENERATE_Q_CURRENT_SINE);
        #endif
        (*CTRL).i->cmd_iDQ[0] = 0.0;
        (*CTRL).i->cmd_iDQ[1] = (*debug).set_iq_command; // SWEEP_FREQ_C2V
        #if WHO_IS_USER == USER_WB
            _user_wubo_FOC( (*CTRL).i->theta_d_elec, (*CTRL).i->iAB );
        #else
            _onlyFOC((*CTRL).i->theta_d_elec, (*CTRL).i->iAB, (*CTRL).i->varOmega);
        #endif
        break;
    case MODE_SELECT_FOC_HARNEFORS_1998: // 36
        #if WHO_IS_USER == USER_WB
            #if PC_SIMULATION
                ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN*0.5);
            #endif
            (*CTRL).i->cmd_iDQ[0] = (*debug).set_id_command;
            (*CTRL).i->cmd_iDQ[1] = (*debug).set_iq_command;
            _user_wubo_FOC((*CTRL).i->theta_d_elec, (*CTRL).i->iAB);
        #endif
        break;
    case MODE_SELECT_VELOCITY_LOOP: // 4
        _user_commands();         // User commands
        FOC_with_vecocity_control((*CTRL).i->theta_d_elec,
            (*CTRL).i->varOmega,
            (*CTRL).i->cmd_varOmega,
            (*CTRL).i->cmd_iDQ,
            (*CTRL).i->iAB);

        break;
    case MODE_SELECT_VELOCITY_LOOP_SENSORLESS : //41
        #if (WHO_IS_USER == USER_YZZ)
            US_P(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_P(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            US_C(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_C(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            IS_C(0)           = (*CTRL).i->iAB[0];
            IS_C(1)           = (*CTRL).i->iAB[1];
        #endif
            US_SR_P(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_P(1) = (*CTRL).o->cmd_uAB[1];
            US_SR_C(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_C(1) = (*CTRL).o->cmd_uAB[1];
            IS_SR_C(0) = (*CTRL).i->iAB[0];
            IS_SR_C(1) = (*CTRL).i->iAB[1];
        #if WHO_IS_USER == USER_YZZ
        _user_commands();
        pmsm_observers();

        // OBSV.theta_d = (*CTRL).i->theta_d_elec;
        // while(OBSV.theta_d > M_PI) OBSV.theta_d  -= 2*M_PI;
        // while(OBSV.theta_d < -M_PI) OBSV.theta_d += 2*M_PI;  // 反转！
        // if (d_sim.user.bool_ESO_SPEED_ON == TRUE){
        //     Main_esoaf_chen2021();
        // }
        // if (d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK == TRUE){
        //     (*CTRL).i->varOmega = OFSR.esoaf.xOmg * MOTOR.npp_inv;
        // }

        FOC_with_vecocity_control(FE.CMDC.theta_d, 
            FE.CMDC.omega_elec*MOTOR.npp_inv, 
            (*CTRL).i->cmd_varOmega, 
            (*CTRL).i->cmd_iDQ, 
            (*CTRL).i->iAB
        );

        // // observer();
        // if (d_sim.user.sensorless_speed_observer == 0){
        //     OBSV.varOmega = (*CTRL).i->varOmega;
        // }else if(d_sim.user.sensorless_speed_observer == 1){
        //     OBSV.varOmega = OBSV.nsoaf.xOmg * MOTOR.npp_inv;
        // }
        
        // if (d_sim.user.sensorless_only_theta_on == 1){
        //     FOC_with_vecocity_control(AFE_USED.theta_d, 
        //         OBSV.varOmega, 
        //         (*CTRL).i->cmd_varOmega, 
        //         (*CTRL).i->cmd_iDQ, 
        //         (*CTRL).i->iAB);
        // }else if (d_sim.user.sensorless_only_theta_on == 0){
        //     FOC_with_vecocity_control((*CTRL).i->theta_d_elec, 
        //         OBSV.varOmega,
        //         (*CTRL).i->cmd_varOmega,
        //         (*CTRL).i->cmd_iDQ,
        //         (*CTRL).i->iAB);
        // }
        #endif
        break;
    case MODE_SELECT_TESTING_SENSORLESS : //42
        break;
    case MODE_SELECT_VELOCITY_LOOP_WC_TUNER: // 43
        #if WHO_IS_USER == USER_WB && PC_SIMULATION == TRUE
            INNER_LOOP_SENSITIVITY_ANALYSIS(debug);
            if ( d_sim.user.bool_apply_HitWall_analysis == TRUE){
                (*debug).set_rpm_speed_command = d_sim.user.HitWall_high_RPM_command;
                static REAL last_time = 0.0;
                static int i = 0.0;
                REAL interval_time = d_sim.user.HitWall_time_interval;
                if( ((*CTRL).timebase - last_time > interval_time) && (i < NUMBER_OF_HIT_WALL_VAR_RATIO) ){
                    PID_iD->OutLimit = d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc * wubo_HW.Vdc_limit_ratio[i];
                    PID_iQ->OutLimit = d_sim.CL.LIMIT_DC_BUS_UTILIZATION * d_sim.init.Vdc * wubo_HW.Vdc_limit_ratio[i];
                    #if PC_SIMULATION
                        printf("Vdc limit is %f\n", PID_iD->OutLimit);
                    #endif
                    last_time = (*CTRL).timebase; // here right????????
                    i = i + 1;
                }
            }
        #endif
        // Runing Speed ESO
        #if WHO_IS_USER == USER_WB
        if (d_sim.user.bool_ESO_SPEED_ON = TRUE){
            Main_esoaf_chen2021();
        }
        if (d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK == TRUE){
            (*CTRL).i->varOmega = OFSR.esoaf.xOmg * MOTOR.npp_inv;
        }
        _user_commands();         // User commands
        FOC_with_vecocity_control((*CTRL).i->theta_d_elec,
            (*CTRL).i->varOmega,
            (*CTRL).i->cmd_varOmega,
            (*CTRL).i->cmd_iDQ,
            (*CTRL).i->iAB);
        #endif
        break;
    case MODE_SELECT_Marino2005: //44
    #if (WHO_IS_USER == USER_CJH)
        controller_marino2005_with_commands();
    #endif
        break;
    case MODE_SELECT_VELOCITY_LOOP_HARNEFORS_1998: //45
        break;
    case MODE_SELECT_SWEEPING_FREQ_FOR_VELOCITY_AND_CURRENT: // 46
            overwrite_sweeping_frequency();
            #if WHO_IS_USER == USER_WB
                if ( d_sim.user.bool_sweeping_frequency_for_speed_loop == TRUE ){
                    
                    // Runing Speed ESO
                    if (d_sim.user.bool_ESO_SPEED_ON = TRUE){
                        Main_esoaf_chen2021();
                    }
                    if (d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK == TRUE){
                        (*CTRL).i->varOmega = OFSR.esoaf.xOmg * MOTOR.npp_inv;
                    }
                    
                    // Get -3DB counter!
                    REAL motor_speed_RPM = (*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM;
                    _user_Check_ThreeDB_Point( motor_speed_RPM, d_sim.user.CMD_SPEED_SINE_RPM );

                    // Run Speed Closed Loop
                    FOC_with_vecocity_control((*CTRL).i->theta_d_elec,
                                (*CTRL).i->varOmega,
                                (*CTRL).i->cmd_varOmega,
                                (*CTRL).i->cmd_iDQ,
                                (*CTRL).i->iAB);
                }else {//* sweeping for current loop especially for iD currents
                    if (d_sim.user.bool_sweeping_frequency_for_current_loop_iD == TRUE){
                        _user_Check_ThreeDB_Point( (*CTRL).i->cmd_iDQ[0], d_sim.user.CMD_CURRENT_SINE_AMPERE );
                    }else{
                        _user_Check_ThreeDB_Point( (*CTRL).i->cmd_iDQ[1], d_sim.user.CMD_CURRENT_SINE_AMPERE );
                    }
                    #if WHO_IS_USER == USER_WB
                        if (d_sim.user.bool_enable_Harnefors_back_calculation){
                            _user_wubo_FOC( (*CTRL).i->theta_d_elec, (*CTRL).i->iAB );
                        }else{
                            d_sim.user.Check_Harnerfors_1998_On = -1;
                            _onlyFOC( (*CTRL).i->theta_d_elec, (*CTRL).i->iAB );
                        }
                    #else
                        _onlyFOC((*CTRL).i->theta_d_elec, (*CTRL).i->iAB);
                    #endif
                }
            #endif
        break;
    case MODE_SELECT_VELOCITY_LOOP_USING_ESO_FOR_SPEED: // 47
        _user_commands();         // User commands
        // Runing Speed ESO
        #if WHO_IS_USER == USER_WB
            if (d_sim.user.bool_ESO_SPEED_ON = TRUE){
                Main_esoaf_chen2021();
            }
            if (d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK == TRUE){
                (*CTRL).i->varOmega = OFSR.esoaf.xOmg * MOTOR.npp_inv;
            }
        #endif
        // Run Speed Closed Loop
        FOC_with_vecocity_control((*CTRL).i->theta_d_elec,
                    (*CTRL).i->varOmega,
                    (*CTRL).i->cmd_varOmega,
                    (*CTRL).i->cmd_iDQ,
                    (*CTRL).i->iAB);
        break;
    case MODE_SELECT_VARIABLE_PARAMETERS_VELOCITY_LOOP_SENSORLESS: // 48
        _user_commands();  
        #if WHO_IS_USER == USER_YZZ
        //for OBSV
            US_P(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_P(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            US_C(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_C(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            IS_C(0)           = (*CTRL).i->iAB[0];
            IS_C(1)           = (*CTRL).i->iAB[1];
        //for OFSR
            US_SR_P(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_P(1) = (*CTRL).o->cmd_uAB[1];
            US_SR_C(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_C(1) = (*CTRL).o->cmd_uAB[1];
            IS_SR_C(0) = (*CTRL).i->iAB[0];
            IS_SR_C(1) = (*CTRL).i->iAB[1];
        variabel_parameters_sensorless();
        _user_commands();
        pmsm_observers();
        OBSV.theta_d = (*CTRL).i->theta_d_elec;
        while(OBSV.theta_d > M_PI) OBSV.theta_d  -= 2*M_PI;
        while(OBSV.theta_d < -M_PI) OBSV.theta_d += 2*M_PI;  // 反转！

        if (d_sim.user.bool_ESO_SPEED_ON == TRUE){
            Main_esoaf_chen2021();
        }
        if (d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK == TRUE){
            (*CTRL).i->varOmega = OFSR.esoaf.xOmg * MOTOR.npp_inv;
        }
        // FOC_with_vecocity_control(AFE_USED.theta_d, 
        //     OBSV.nsoaf.xOmg * MOTOR.npp_inv,
        //     (*CTRL).i->cmd_varOmega,
        //     (*CTRL).i->cmd_iDQ,
        //     (*CTRL).i->iAB);
        FOC_with_vecocity_control((*CTRL).i->theta_d_elec, 
            (*CTRL).i->varOmega, 
            (*CTRL).i->cmd_varOmega, 
            (*CTRL).i->cmd_iDQ, 
            (*CTRL).i->iAB);
        #endif
        break;
    case MODE_SELECT_INVERTER_NONLINEARITY_SENSORLESS: // 49
        #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
            US_P(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_P(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            US_C(0) = (*CTRL).o->cmd_uAB[0]; // 后缀_P表示上一步的电压，P = Previous
            US_C(1) = (*CTRL).o->cmd_uAB[1]; // 后缀_C表示当前步的电压，C = Current
            IS_C(0)           = (*CTRL).i->iAB[0];
            IS_C(1)           = (*CTRL).i->iAB[1];
        #endif
            US_SR_P(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_P(1) = (*CTRL).o->cmd_uAB[1];
            US_SR_C(0) = (*CTRL).o->cmd_uAB[0];
            US_SR_C(1) = (*CTRL).o->cmd_uAB[1];
            IS_SR_C(0) = (*CTRL).i->iAB[0];
            IS_SR_C(1) = (*CTRL).i->iAB[1];
        #if WHO_IS_USER == USER_YZZ
        #if PC_SIMULATION
            _user_commands();
        #endif
        pmsm_observers();
        // Online_PAA_Based_Compensation();
        // observer_PMSMife();
        // controller_PMSMife_with_commands();
        OBSV.theta_d = (*CTRL).i->theta_d_elec;
        while(OBSV.theta_d > M_PI) OBSV.theta_d  -= 2*M_PI;
        while(OBSV.theta_d < -M_PI) OBSV.theta_d += 2*M_PI;  // 反转！
        if (d_sim.user.bool_ESO_SPEED_ON == TRUE){
            Main_esoaf_chen2021();
        }
        if (d_sim.user.bool_apply_ESO_SPEED_for_SPEED_FBK == TRUE){
            (*CTRL).i->varOmega = OFSR.esoaf.xOmg * MOTOR.npp_inv;
        }
        // observer();
        if (d_sim.user.sensorless_speed_observer == 0){
            OBSV.varOmega = (*CTRL).i->varOmega;
        }else if(d_sim.user.sensorless_speed_observer == 1){
            OBSV.varOmega = OBSV.nsoaf.xOmg * MOTOR.npp_inv;
        }
        
        if (d_sim.user.sensorless_only_theta_on == 1){
            FOC_with_vecocity_control(AFE_USED.theta_d, 
                OBSV.varOmega, 
                (*CTRL).i->cmd_varOmega, 
                (*CTRL).i->cmd_iDQ, 
                (*CTRL).i->iAB);
        }else if (d_sim.user.sensorless_only_theta_on == 0){
            FOC_with_vecocity_control((*CTRL).i->theta_d_elec, 
                OBSV.varOmega,
                (*CTRL).i->cmd_varOmega,
                (*CTRL).i->cmd_iDQ,
                (*CTRL).i->iAB);
        }
        #endif
        break;
    case MODE_SELECT_POSITION_LOOP: // 5
        #if WHO_IS_USER == USER_WB
            //TODO: Here need a command function for position loop !
            // (*debug).set_deg_position_command = d_sim.user.set_deg_position_command * sin( 2 * M_PI * d_sim.user.Position_cmd_sine_frequency * (*CTRL).timebase );
            (*debug).set_deg_position_command = d_sim.user.set_deg_position_command;
            (*CTRL).i->cmd_varTheta = (*debug).set_deg_position_command * M_PI_OVER_180;
            _user_wubo_PositionLoop_controller( (*CTRL).i->varTheta,
                                                (*CTRL).i->cmd_varTheta
            );
        #endif
        break;
    case MODE_SELECT_COMMISSIONING: // 9
        // #if ENABLE_COMMISSIONING == TRUE
        #if ENABLE_COMMISSIONING
            commissioning();
        #endif
        // #endif
        break;
    case MODE_SELECT_GENERATOR://8
        #if PC_SIMULATION == TRUE
            Generator();
        #endif
        // ACM.R = 0.4; 
        // ACM.Ld = 0.017;
        // ACM.Lq = 0.015;
        break;
    case MODE_SELECT_NYQUIST_PLOTTING: //91
        #if WHO_IS_USER == USER_WB && PC_SIMULATION == TRUE
            d_sim.user.flag_Nyquist_one_cycle_DONE = FALSE;
            if ((*CTRL).timebase > d_sim.user.CMD_SPEED_SINE_END_TIME){
                d_sim.user.flag_Nyquist_one_cycle_DONE = TRUE; // 用来清空Nyquist_sum_sin和sum_cos，以进行下一次计算
                d_sim.user.CMD_SPEED_SINE_HZ += d_sim.user.CMD_SPEED_SINE_STEP_SIZE;
                d_sim.user.CMD_SPEED_SINE_LAST_END_TIME = d_sim.user.CMD_SPEED_SINE_END_TIME;
                d_sim.user.CMD_SPEED_SINE_END_TIME += d_sim.user.Nyquist_plot_num_cycles / d_sim.user.CMD_SPEED_SINE_HZ;
            }
            if (d_sim.user.CMD_SPEED_SINE_HZ > d_sim.user.Nyquist_Freq_Ceiling){
                (*CTRL).i->cmd_iDQ[0] = 0.0; // 到达扫频的频率上限，速度归零
            }else{
                // 这里的信号其实可以是任意形式的，不一定是正弦波
                (*CTRL).i->cmd_iDQ[0] = d_sim.user.Nyquist_Input_Current_Amp * sinf ( 2 * M_PI * d_sim.user.CMD_SPEED_SINE_HZ * ( (*CTRL).timebase - d_sim.user.CMD_SPEED_SINE_LAST_END_TIME ) );
                _onlyFOC( (*CTRL).i->theta_d_elec, (*CTRL).i->iAB );

                if ( d_sim.user.flag_Nyquist_one_cycle_DONE == TRUE){
                    d_sim.user.Nyquist_Amp   = 2 / d_sim.user.Nyquist_one_cycle_count * sqrtf( d_sim.user.Nyquist_sum_sin * d_sim.user.Nyquist_sum_sin + d_sim.user.Nyquist_sum_cos * d_sim.user.Nyquist_sum_cos );
                    d_sim.user.Nyquist_Phase = atan2f( d_sim.user.Nyquist_sum_sin, d_sim.user.Nyquist_sum_cos );
                    printf("Nyquist_Amp: %f, Nyquist_Phase: %f\n", d_sim.user.Nyquist_Amp, d_sim.user.Nyquist_Phase);
                    d_sim.user.Nyquist_sum_sin = 0.0;
                    d_sim.user.Nyquist_sum_cos = 0.0;
                }
                // 利用同频cos()和sin()提取谐波信号的实部和虚部，假设系统为线性定常系统
                // A=sum( va*cos ) B=sum( va*sin )
                // a1=2A/N a2=2B/N mag=sqrt(a1^2+a2^2) deg=tan(a2/a1)
                d_sim.user.Nyquist_sum_sin += PID_iD->Ref * sinf( 2 * M_PI * d_sim.user.CMD_SPEED_SINE_HZ * ( (*CTRL).timebase - d_sim.user.CMD_SPEED_SINE_LAST_END_TIME ) );
                d_sim.user.Nyquist_sum_cos += PID_iD->Ref * cosf( 2 * M_PI * d_sim.user.CMD_SPEED_SINE_HZ * ( (*CTRL).timebase - d_sim.user.CMD_SPEED_SINE_LAST_END_TIME ) );
                d_sim.user.Nyquist_one_cycle_count++;
            }
        #endif
        break;
    case MODE_SELECT_UDQ_GIVEN_TEST: // 98
        #if WHO_IS_USER == USER_WB
            UDQ_GIVEN_TEST();
        #endif
        break;
    case MODE_SELECT_NB_MODE: // 99
        #if WHO_IS_USER == USER_WB
            NB_MODE_codes();
        #endif
        break;
    default:
        // 电压指令(*CTRL).o->cmd_uAB[0/1]通过逆变器，产生实际电压ACM.ual, ACM.ube（变换到dq系下得到ACM.ud，ACM.uq）
        // voltage_commands_to_pwm(); // this function only exists in DSP codes
        // inverter_model(); // in Simulation
        (*debug).error = 999;
        break;
    }
    mode_select_last = mode_select;
    return 0;
}
/* Other only simulation codes */
#if PC_SIMULATION
    void _user_time_varying_parameters(){
        
        // ACM.R  = d_sim.init.R  * 2.5;
        // ACM.Ld = d_sim.init.Ld * 0.25;
        // ACM.Lq = d_sim.init.Lq * 0.25;
        
        // 0. 参数时变
        // if (fabsf((*CTRL).timebase-0.025)<CL_TS){
        //     printf("[Runtime] Rotor inertia of the simulated machine has changed! Js=%g\n", ACM.Js);
            // ACM.Js     = 0.1 * d_sim.init.Js; // kg.m^2 0.41500000000000004
            // ACM.Js_inv = 1.0 / ACM.Js;
        // }
        // if (fabsf((*CTRL).timebase-0.035)<CL_TS){
        //     printf("[Runtime] Rotor inertia of the simulated machine has changed! Js=%g\n", ACM.Js);
        //     ACM.Js     = 0.1 * d_sim.init.Js; // kg.m^2
        //     ACM.Js_inv = 1.0 / ACM.Js;
        // }
        ///
        // Changing KE should go to the init_Machine to change the initial value of KE if u are running at a PMSM Ld = Lq
        ///
    }
    REAL _user_load_model(){
        static REAL Tload = 0.0;
        // static int load_state = 0;
        // static REAL dc_part = LOAD_TORQUE;
        // static REAL viscous_part = 0.0; // 这个变量去到Config文件里面了
        // viscous_part = VISCOUS_COEFF*ACM.rpm*RPM_2_ELEC_RAD_PER_SEC;
        // Tload = dc_part + viscous_part;
        if(CTRL_1.timebase > 0.5){
            Tload = 0.5;
        }
        return Tload;
    }
    void Generator(){
        REAL speed_cmd = 12000 * RPM_2_MECH_RAD_PER_SEC;
        ACM.TLoad = - 1.5 * (speed_cmd - ACM.varOmega);

        ACM.uDQ[0] =  150 * cos( 0.2*M_PI*0.5);
        ACM.uDQ[1] =  150 * sin( 0.2*M_PI*0.5);
        printf("ACM.omega_syn * ACM.KA * 1.732: %g\n", ACM.omega_syn * ACM.KA * 1.732);
        printf("P: %g\n", ACM.Tem * ACM.varOmega * MECH_RAD_PER_SEC_2_RPM);
        ACM.uAB[0] = MT2A(ACM.uDQ[0], ACM.uDQ[1], ACM.cosT, ACM.sinT);
        ACM.uAB[1] = MT2B(ACM.uDQ[0], ACM.uDQ[1], ACM.cosT, ACM.sinT);
        ACM.current_theta = atan2(ACM.iAB[1], ACM.iAB[0]) - M_PI;
        ACM.voltage_theta = atan2(ACM.uAB[1], ACM.uAB[0]);
        ACM.powerfactor = (angle_diff(ACM.voltage_theta, ACM.current_theta) ) * ONE_OVER_2PI * 360;
        CTRL->s->cosT = cos(angle_diff(ACM.voltage_theta, ACM.current_theta) );
        // if(sqrtf(ACM.iAB[1] * ACM.uAB[1] + ACM.iAB[0] * ACM.uAB[0])>0)
            // CTRL->s->cosT = ACM.TLoad * ACM.varOmega / sqrtf(ACM.iAB[1] * ACM.uAB[1] + ACM.iAB[0] * ACM.uAB[0]);
            //CTRL->s->cosT = sqrtf(ACM.iAB[1] * ACM.iAB[1] + ACM.iAB[0] * ACM.iAB[0]) * ACM.KA * ACM.omega_syn
            // CTRL->s->cosT = ACM.Tem * ACM.varOmega / (
            //         sqrtf((ACM.iAB[1] * ACM.iAB[1]) +  (ACM.iAB[0] *ACM.iAB[0])) * sqrtf((ACM.uAB[1] * ACM.uAB[1]) +  (ACM.uAB[0] *ACM.uAB[0]))
            //     );
        printf("power factor: %g\n", CTRL->s->cosT);
    }
#endif

/* Motor Speed ESO */
//Observer for speed reconstruction
#define CJH_STYLE_RK4_OBSERVER_RAW_CODE_FOR_ESO                                  \
        US_SR(0) = US_SR_P(0);                                                     \
        US_SR(1) = US_SR_P(1);                                                     \
        IS_SR(0) = IS_SR_P(0);                                                     \
        IS_SR(1) = IS_SR_P(1);                                                     \
        (*fp)(t, x, fx);                                                     \
        for(i=0;i<NS;++i){                                                   \
            k1[i] = fx[i] * hs;                                              \
            xk[i] = x[i] + k1[i]*0.5;                                        \
        }                                                                    \
                                                                            \
        IS_SR(0) = 0.5*(IS_SR_P(0)+IS_SR_C(0));                                       \
        IS_SR(1) = 0.5*(IS_SR_P(1)+IS_SR_C(1));                                       \
        US_SR(0) = 0.5*(US_SR_P(0)+US_SR_C(0));                                       \
        US_SR(1) = 0.5*(US_SR_P(1)+US_SR_C(1));                                       \
        (*fp)(t, xk, fx);                                                    \
        for(i=0;i<NS;++i){                                                   \
            k2[i] = fx[i] * hs;                                              \
            xk[i] = x[i] + k2[i]*0.5;                                        \
        }                                                                    \
                                                                            \
        (*fp)(t, xk, fx);                                                    \
        for(i=0;i<NS;++i){                                                   \
            k3[i] = fx[i] * hs;                                              \
            xk[i] = x[i] + k3[i];                                            \
        }                                                                    \
                                                                            \
        IS_SR(0) = IS_SR_C(0);                                                     \
        IS_SR(1) = IS_SR_C(1);                                                     \
        US_SR(0) = US_SR_C(0);                                                     \
        US_SR(1) = US_SR_C(1);                                                     \
        (*fp)(t, xk, fx);                                                    \
        for(i=0;i<NS;++i){                                                   \
            k4[i] = fx[i] * hs;                                              \
            x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;  \
        }

void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 4
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE_FOR_ESO 
        #undef NS
    }

void init_rk4(){
    int i;
    for(i=0; i<2; ++i){
        OFSR.rk4.us[i] = 0;
        OFSR.rk4.is[i] = 0;
        // OFSR.rk4.us_curr[i] = 0;
        OFSR.rk4.is_curr[i] = 0;
        OFSR.rk4.us_prev[i] = 0;
        OFSR.rk4.is_prev[i] = 0;
        OFSR.rk4.is_lpf[i]  = 0;
        OFSR.rk4.is_hpf[i]  = 0;
        OFSR.rk4.is_bpf[i]  = 0;

        OFSR.rk4.current_lpf_register[i] = 0;
        OFSR.rk4.current_hpf_register[i] = 0;
        OFSR.rk4.current_bpf_register1[i] = 0;
        OFSR.rk4.current_bpf_register2[i] = 0;
    }
}
//ESO
void rhf_dynamics_ESO(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    REAL xPos = x[0];
    REAL xOmg = x[1];
    REAL xTL  = x[2];
    REAL xPL  = x[3];

    /* Know Signals */
    REAL iq = AB2T(IS_SR(0), IS_SR(1), (*CTRL).s->cosT, (*CTRL).s->sinT); // Option 1
    // REAL iq = AB2T(IS(0), IS(1), cos(xPos), sin(xPos)); // Option 2
    OFSR.esoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * MOTOR.KActive * iq;

    /* 未测试，如果用iq给定会不会好一点？？ 计算量还少*/
    /* 未测试，如果用iq给定会不会好一点？？ 计算量还少*/
    /* 未测试，如果用iq给定会不会好一点？？ 计算量还少*/
    // OFSR.esoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * MOTOR.KActive * CTRL->I.cmd_iDQ[1];

    /* Output Error = sine of angle error */
    // OFSR.esoaf.output_error_sine = sin(AFE_USED.theta_d - xPos);
    // OFSR.esoaf.output_error = AFE_USED.theta_d - xPos;
    OFSR.esoaf.output_error_sine = sin((*CTRL).i->theta_d_elec - xPos);
    OFSR.esoaf.output_error = (*CTRL).i->theta_d_elec - xPos;
    // you should check for sudden change in angle error.
    if(fabsf(OFSR.esoaf.output_error)>M_PI){
        OFSR.esoaf.output_error -= sign(OFSR.esoaf.output_error) * 2*M_PI;
    }

    /* Extended State Observer */
    // xPos
    fx[0] = + OFSR.esoaf.ell[0]*OFSR.esoaf.output_error_sine + xOmg;
    // xOmg
    fx[1] = + OFSR.esoaf.ell[1]*OFSR.esoaf.output_error_sine + (OFSR.esoaf.bool_ramp_load_torque>=0) * (OFSR.esoaf.xTem - xTL) * (MOTOR.Js_inv*MOTOR.npp);
    // xTL
    fx[2] = - OFSR.esoaf.ell[2]*OFSR.esoaf.output_error_sine + xPL;
    // xPL
    fx[3] = - OFSR.esoaf.ell[3]*OFSR.esoaf.output_error_sine;
}
void eso_one_parameter_tuning(REAL omega_ob){
    // Luenberger Observer Framework
    if(OFSR.esoaf.bool_ramp_load_torque == -1){
        OFSR.esoaf.ell[0] = 2*omega_ob;
        OFSR.esoaf.ell[1] = omega_ob*omega_ob;
        OFSR.esoaf.ell[2] = 0.0;
        OFSR.esoaf.ell[3] = 0.0;        
    }else if(OFSR.esoaf.bool_ramp_load_torque == FALSE){
        OFSR.esoaf.ell[0] =                            3*omega_ob;
        OFSR.esoaf.ell[1] =                            3*omega_ob*omega_ob;
        OFSR.esoaf.ell[2] = (MOTOR.Js*MOTOR.npp_inv) * 1*omega_ob*omega_ob*omega_ob;
        OFSR.esoaf.ell[3] = 0.0;
    }else{
        // TODO: REAL check?
        OFSR.esoaf.ell[0] =                            4*omega_ob;
        OFSR.esoaf.ell[1] =                            6*omega_ob*omega_ob;
        OFSR.esoaf.ell[2] = (MOTOR.Js*MOTOR.npp_inv) * 4*omega_ob*omega_ob*omega_ob;
        OFSR.esoaf.ell[3] = (MOTOR.Js*MOTOR.npp_inv) * 1*omega_ob*omega_ob*omega_ob*omega_ob;
    }

    #if PC_SIMULATION
    printf("ESO OPT: %g, %g, %g, %g\n", OFSR.esoaf.ell[0], OFSR.esoaf.ell[1], OFSR.esoaf.ell[2], OFSR.esoaf.ell[3]);
    #endif
}
void Main_esoaf_chen2021(){

    /* OBSERVATION */

    if(OFSR.esoaf.set_omega_ob != OFSR.esoaf.omega_ob){
        OFSR.esoaf.omega_ob = OFSR.esoaf.set_omega_ob;
        eso_one_parameter_tuning(OFSR.esoaf.omega_ob);
    }

    general_4states_rk4_solver(&rhf_dynamics_ESO, (*CTRL).timebase, OFSR.esoaf.x, CL_TS);
    if(OFSR.esoaf.x[0]>M_PI){
        OFSR.esoaf.x[0] -= 2*M_PI;
    }
    if(OFSR.esoaf.x[0]<-M_PI){
        OFSR.esoaf.x[0] += 2*M_PI;
    }
    OFSR.esoaf.xPos = OFSR.esoaf.x[0];
    OFSR.esoaf.xOmg = OFSR.esoaf.x[1];
    OFSR.esoaf.xTL  = OFSR.esoaf.x[2];
    OFSR.esoaf.xPL  = OFSR.esoaf.x[3]; // rotatum

    /* Post-observer calculations */
}
void init_esoaf(){

    OFSR.esoaf.ell[0] = 0.0;
    OFSR.esoaf.ell[1] = 0.0;
    OFSR.esoaf.ell[2] = 0.0;
    OFSR.esoaf.ell[3] = 0.0;
    OFSR.esoaf.set_omega_ob = d_sim.user.CAREFUL_ESOAF_OMEGA_OBSERVER;
    OFSR.esoaf.bool_ramp_load_torque = -1;

    OFSR.esoaf.omega_ob = OFSR.esoaf.set_omega_ob;
    eso_one_parameter_tuning(OFSR.esoaf.omega_ob);
}
