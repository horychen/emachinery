#ifndef MAIN_SWITCH_H
#define MAIN_SWITCH_H
// #include "ACMSim.h"

// /* This code should be at the super_config.h*/
// #define WHO_IS_USER 2023231051
// /* This code should be at the super_config.h*/

#define USER_CJH    101976
#define USER_XM     102209
#define USER_BEZIER 224
#define USER_GZT    2021531030
#define USER_WB     2023231051
#define USER_YZZ    2023231060
#define USER_WB2    970308
#define USER_GEN    240828
#define USER_QIAN   2022231110
#define USER_CURY   201314

#define MODE_SELECT_PWM_DIRECT         1
#define MODE_SELECT_VOLTAGE_OPEN_LOOP  11
#define MODE_SELECT_WITHOUT_ENCODER_CURRENT_VECTOR_ROTATE 2
#define MODE_SELECT_FOC                      3
#define MODE_SELECT_FOC_SENSORLESS           31
#define MODE_SELECT_INDIRECT_FOC             32
#define MODE_SELECT_ID_SWEEPING_FREQ         33
#define MODE_SELECT_IQ_SWEEPING_FREQ         34
#define MODE_SELECT_FOC_HARNEFORS_1998       36
#define MODE_SELECT_VELOCITY_LOOP            4
#define MODE_SELECT_VELOCITY_LOOP_SENSORLESS 41
#define MODE_SELECT_TESTING_SENSORLESS       42
#define MODE_SELECT_VELOCITY_LOOP_WC_TUNER   43 // 这个模式被弃用了，现在于USER_WB中实现WC Tuner
#define MODE_SELECT_Marino2005               44
#define MODE_SELECT_VELOCITY_LOOP_HARNEFORS_1998   45
#define MODE_SELECT_SWEEPING_FREQ_FOR_VELOCITY_AND_CURRENT   46
#define MODE_SELECT_VELOCITY_LOOP_USING_ESO_FOR_SPEED 47
#define MODE_SELECT_VARIABLE_PARAMETERS_VELOCITY_LOOP_SENSORLESS 48
#define MODE_SELECT_INVERTER_NONLINEARITY_SENSORLESS 49
#define MODE_SELECT_POSITION_LOOP            5
#define MODE_SELECT_COMMISSIONING            9
#define MODE_SELECT_NYQUIST_PLOTTING         91
#define MODE_SELECT_UDQ_GIVEN_TEST           98
#define MODE_SELECT_GENERATOR                8
#define MODE_SELECT_NB_MODE                  99

// 前向定义解决gcc的-Wincompatible-pointer-types问题
typedef struct st_pid_regulator st_pid_regulator;

struct st_pid_regulator {
    float32 Ref;
    float32 Fbk;
    float32 Err;
    float32 ErrPrev;
    float32 P_Term; 
    float32 I_Term; 
    float32 D_Term;
    float32 OutNonSat;
    float32 OutLimit;
    float32 Out;
    float32 OutPrev; // for incremental pid
    float32 Kp;
    float32 Ki_CODE;
    float32 Kd;
    float32 SatDiff;
    float32 FbkPrev;
    void (*calc)(st_pid_regulator *);
};
typedef st_pid_regulator *st_pid_regulator_handle;

// void ACMSIMC_PIDTuner();
void PID_calc(st_pid_regulator_handle); // pid_regulator中定义了默认的离散PID，因为吴波在速度环加入了innerLoop，他的离散PID和默认的有些许区别
void incremental_PI(st_pid_regulator *r);
void tustin_PI(st_pid_regulator *r);
#define st_pid_regulator_DEFAULTS { \
    /*Reference*/ 0.0, \
    /*Feedback*/ 0.0, \
    /*Control Error*/ 0.0, \
    /*Control Error from Previous Step*/ 0.0, \
    /*Proportional Term*/  0.0, \
    /*Integral Term*/  0.0, \
    /*Derivative Term*/  0.0, \
    /*Non-Saturated Output*/  0.0, \
    /*Output Limit*/  0.95, \
    /*Output*/  0.0, \
    /*Output from Previous Step*/  0.0, \
    /*Kp*/  1.0, \
    /*Ki_CODE(Discretization)*/  0.001, \
    /*Kd*/  0.0, \
    /*Difference between Non-Saturated Output and Saturated Output*/  0.0, \
    /*Previous Feedback*/  0.0, \
    (void (*)(st_pid_regulator *)) incremental_PI \
}
    // (void (*)(Uint32)) tustin_PI \

extern st_pid_regulator PID_iD;
extern st_pid_regulator PID_iQ;
extern st_pid_regulator PID_Speed;
extern st_pid_regulator PID_Position;
// extern st_pid_regulator pid1_ia;
// extern st_pid_regulator pid1_ib;
// extern st_pid_regulator pid1_ic;
// extern st_pid_regulator pid2_ix;
// extern st_pid_regulator pid2_iy;

typedef struct{
    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float outLimit;

    /* Integrator limits */
    float intLimit;

    /* Sample time (in seconds) */
    float Ts;

    /* Controller "memory" */
    float integrator;
    float prevError;      /* Required for integrator */
    float differentiator;
    float prevMeasurement;    /* Required for differentiator */

    /* Controller output */
    float out;

    /* Controller input */
    float setpoint;
    float measurement;
} st_PIDController;
extern st_PIDController pid1_dispX;
extern st_PIDController pid1_dispY;


/* wubo:yi！？这两个函数怎么会在我那里 */
// void  PIDController_Init(st_PIDController *pid);
// float PIDController_Update(st_PIDController *pid);



struct DebugExperiment{
    long error;
    long who_is_user;
    long mode_select;
    REAL Overwrite_Current_Frequency;
    REAL Overwrite_theta_d;
    REAL set_deg_position_command;
    REAL set_rpm_speed_command;
    REAL set_iq_command;
    REAL set_id_command;
    int INVERTER_NONLINEARITY_COMPENSATION_INIT;
    int INVERTER_NONLINEARITY;
    int SENSORLESS_CONTROL;
    int SENSORLESS_CONTROL_HFSI;
    REAL vvvf_voltage;
    REAL vvvf_frequency;
    int positionLoopType;
    REAL LIMIT_DC_BUS_UTILIZATION;
    REAL LIMIT_OVERLOAD_FACTOR;
    REAL delta;
    REAL CLBW_HZ;
    REAL VL_EXE_PER_CL_EXE;
    int  Select_exp_operation;
    // BOOL bool_initilized;
    BOOL bool_apply_decoupling_voltages_to_current_regulation;
    #if WHO_IS_USER == USER_WB
        REAL zeta;
        REAL omega_n;
        REAL max_CLBW_PER_min_CLBW;
        BOOL bool_apply_WC_tunner_for_speed_loop;
        BOOL bool_sweeping_frequency_for_speed_loop;
        BOOL bool_Null_D_Control;
        BOOL bool_apply_sweeping_frequency_excitation;
        BOOL bool_Parameter_Mismatch_test;
        REAL CMD_CURRENT_SINE_AMPERE;
        REAL CMD_SPEED_SINE_RPM;
        REAL CMD_SPEED_SINE_HZ;
        REAL CMD_SPEED_SINE_STEP_SIZE;
        REAL CMD_SPEED_SINE_LAST_END_TIME;
        REAL CMD_SPEED_SINE_END_TIME;
        REAL CMD_SPEED_SINE_HZ_CEILING;
    #endif
};
/* To separate two debugs */
extern int use_first_set_three_phase;
extern struct DebugExperiment debug_1;
extern struct DebugExperiment debug_2;
extern struct DebugExperiment *debug;
/* USER: Macros */

/* User Specified Functions */
void init_debug();
void init_experiment();
void init_CTRL();
void _user_time_varying_parameters();// 时变参数
void _user_observer();
void _onlyFOC(REAL theta_d_elec, REAL iAB[2], REAL varOmega); // only current loop
void _pseudoEncoder(); // 使用内部给定角度代替码盘位置反馈
void _user_inverter_voltage_command(int bool_use_iab_cmd);
void Generator(); // 电机发电模式
REAL angle_diff(REAL a, REAL b);
// 指令和负载
extern struct SweepFreq{
    REAL time;
    REAL freq_step_size;
    REAL current_freq;
    REAL      current_freq_end_time;
    REAL last_current_freq_end_time;
} sf;
// void _user_pmsm_commands();
    // REAL *p_set_rpm_speed_command, REAL *p_set_iq_cmd, REAL *p_set_id_cmd, int *p_set_current_loop, int *p_flag_overwrite_theta_d, REAL *p_Overwrite_Current_Frequency
void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
void cmd_slow_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd);
int main_switch(long mode_select);
void FOC_with_vecocity_control(REAL theta_d_elec, REAL varOmega, REAL cmd_varOmega, REAL cmd_iDQ[2], REAL iAB[2]);
void RK4_FOC_with_vecocity_control(REAL theta_d_elec, REAL varOmega, REAL cmd_varOmega, REAL cmd_iDQ[2], REAL iAB[2]);
void rhf_PI_DynamicsforSpeed(REAL t, REAL *x, REAL *fx);
void rhf_PI_DynamicsforQcurrent(REAL t, REAL *x, REAL *fx);
void rhf_PI_DynamicsforDcurrent(REAL t, REAL *x, REAL *fx);
void General_PI_Dynamics(st_pid_regulator *r, void (*dynamic_func)(REAL, REAL *, REAL *));
void _RK4_PI_Controller_FOC(REAL theta_d_elec, REAL iAB[2]);




// #if MACHINE_TYPE == 2
typedef struct {
    REAL  Ualpha; // Input: reference alpha-axis phase voltage
    REAL  Ubeta;  // Input: reference beta-axis phase voltage
    REAL  Ta;     // Output: reference phase-a switching function
    REAL  Tb;     // Output: reference phase-b switching function
    REAL  Tc;     // Output: reference phase-c switching function
    REAL CMPA[3]; // PWM compare register value original
    REAL CMPA_DBC[3]; // PWM compare register value dead-band compensated
    REAL utilization_ratio; //* The squart of the sum of the square of Ualpha and Ubeta
} SVGENDQ;

typedef struct {
    // commands
    REAL cmd_varTheta;        // mechanical
    REAL cmd_varOmega;        // mechanical
    REAL cmd_deriv_varOmega;  // mechanical
    REAL cmd_dderiv_varOmega; // mechanical
    REAL cmd_rotor_flux_Wb;
    REAL cmd_Tem;
    REAL cmd_iDQ[2];
    REAL cmd_uDQ[2];
    // feedback
    REAL Vdc;
    REAL theta_d_elec; // this is from encoder by default
    REAL varTheta; // mechanical
    REAL varOmega; // mechanical
    REAL iAB[2];
    REAL iDQ[2];
    REAL Tem;
    REAL TLoad;
    REAL psi_active[2];
    // flux commands
    REAL cmd_psi_raw;
    REAL cmd_psi;
    REAL cmd_psi_inv;
    REAL cmd_deriv_psi;
    REAL cmd_dderiv_psi;
    REAL cmd_psi_active[2];
    REAL m0;
    REAL m1;
    REAL omega1;
    REAL theta_d_elec_previous;
} st_controller_inputs;
typedef struct {
    // field oriented control
    REAL xRho; // state for indirect field orientation (IFO)
    REAL cosT;
    REAL sinT;
    REAL cosT_compensated_1p5omegaTs;
    REAL sinT_compensated_1p5omegaTs;
    REAL cosT2;
    REAL sinT2;
    REAL omega_syn;
    REAL omega_sl;
    // states
    st_pid_regulator *iD;
    st_pid_regulator *iQ;
    st_pid_regulator *Speed;
    st_pid_regulator *Position;
    // st_pid_regulator *ix;
    // st_pid_regulator *iy;
    // st_PIDController *dispX;
    // st_PIDController *dispY;
    int the_vc_count;
    // Status of Detection
    int PSD_Done;
    int IPD_Done;
    // State of Operation
    REAL Motor_or_Generator; // 1 for motoring, -1 for regenerating
} st_controller_states;
typedef struct {
    // voltage commands
    REAL cmd_uAB[4];
    REAL cmd_uDQ[4];
    REAL cmd_uAB_to_inverter[4]; // foc control output + inverter nonlinearity compensation
    REAL cmd_uDQ_to_inverter[4];
    // current commands
    REAL cmd_iAB[4];
    REAL dc_bus_utilization_ratio;
} st_controller_outputs;
typedef struct {
    // electrical 
    REAL R;
    REAL KE;
    REAL Ld;
    REAL Ld_inv;
    REAL Lq;
    REAL Lq_inv;
    REAL DeltaL; // Ld - Lq for IPMSM
    REAL KActive;
    REAL Rreq;
    // electrical for induction motor
    // REAL Lsigma;
    // REAL Lsigma_inv;
    // REAL Lmu;
    // REAL Lmu_inv;
    REAL alpha;
    REAL alpha_inv;
    // mechanical
    REAL npp;
    REAL npp_inv;
    REAL Js;
    REAL Js_inv;
} st_motor_parameters;


//
#define MA_SEQUENCE_LENGTH            2  // 10   // 40 for Yaojie large Lq motor  // Note MA_SEQUENCE_LENGTH * CL_TS = window of moving average in seconds
#define MA_SEQUENCE_LENGTH_INVERSE    0.5// 0.1  // 0.025                        // 20 MA gives speed resolution of 3 rpm for 2500 ppr encoder
// #define MA_SEQUENCE_LENGTH         20 // 20 * CL_TS = window of moving average in seconds
// #define MA_SEQUENCE_LENGTH_INVERSE 0.05 // 20 MA gives speed resolution of 3 rpm for 2500 ppr encoder
// #define MA_SEQUENCE_LENGTH         80
// #define MA_SEQUENCE_LENGTH_INVERSE 0.0125
typedef struct {
    // Moving Average for speed calculation
    REAL MA_qepPosCnt[MA_SEQUENCE_LENGTH];
    REAL sum_qepPosCnt;
    int flag_absolute_encoder_powered;
    unsigned int cursor; // cursor is between 0~79, and it decides which element in MA queue should be kicked out.
    // for IPD
    REAL theta_d__state;
    REAL theta_d_offset;
    REAL excitation_angle_deg; // in deg
    // in experiment
    int32 encoder_abs_cnt;
    int32 encoder_abs_cnt_previous;
    int32 encoder_incremental_cnt;
    REAL rpm_raw;
    int16 encoder_direction;
    int32 OffsetCountBetweenIndexAndUPhaseAxis;
    // output
    REAL rpm;
    REAL varOmega;
    REAL theta_d_elec;
} st_enc; // encoder
typedef struct {
    REAL theta_excitation; // in rad
    REAL theta_d_elec_entered;
    Uint32 countEntered;
    int direction;
} st_psd; // phase sequence detection
typedef struct {
    REAL ia;
    REAL ib;
    REAL ic;
    REAL dist_ua;
    REAL dist_ub;
    REAL dist_uc;

    /* Adaptive Vsat */
    REAL gain_Vsat;

    /* DOB for stationary voltage */
    REAL iab_lpf[2];
    REAL iab_hpf[2];
    REAL filter_pole;
    REAL uab_DOB[2];

    /* Park.Sul-2012 */
    REAL gamma_theta_trapezoidal;
    REAL Vsat; // = 4*td*Udc/(pi*Ts)

    REAL thetaA;
    REAL cos_thetaA;
    REAL sin_thetaA;

    REAL u_comp[3]; // phase ABC quantities of compensation
    REAL ual_comp; // alpha-beta frame quantities of compensation
    REAL ube_comp;
    REAL uDcomp_atA; // Compensation Voltage's D-component at phase A frame
    REAL uQcomp_atA;

    REAL iD_atA; // D-component at phase A frame
    REAL iQ_atA;
    REAL I5_plus_I7; // Magnitude of 6th harmonic in syn. speed frame
    REAL I5_plus_I7_LPF; // lpf'd version of I5_plus_I7
    REAL I11_plus_I13;
    REAL I11_plus_I13_LPF;
    REAL I17_plus_I19;
    REAL I17_plus_I19_LPF;
    REAL theta_trapezoidal; // theta_t defined by Park.Sul-2012

    /* Chen 2021 */
    REAL I_plateau_Max;
    REAL I_plateau_Min;
    REAL I_plateau;
    REAL gamma_I_plateau;
    REAL V_plateau;
    REAL gamma_V_plateau;
    /**/
    REAL w6;
    REAL w12;
    REAL w18;
    REAL sig_a2;
    REAL sig_a3;
    REAL gamma_a2;
    REAL gamma_a3;
    REAL error_a3;
    REAL error_a2;
} st_InverterNonlinearity; // inverter
typedef struct {
    // ECAP support (i.e., the API)
    REAL terminal_DutyOnRatio[3];
    REAL terminal_voltage[3];
    REAL line_to_line_voltage[3]; // uv, vw, wu
    REAL pwm_time;
    REAL uab0[3];
    REAL dq[2];
    REAL dq_lpf[2];
    REAL dq_mismatch[2];
    // [Internal variables]
        // Run TI's example
        struct TestECapture {
            // REAL TSt1, TSt2, TSt3, TSt4;
            REAL Period1, Period2, Period3;
            REAL DutyOnTime1, DutyOffTime1, DutyOnTime2, DutyOffTime2;
        }ecapU, ecapV, ecapW;
        // On-demand ECAP decode with nonlinear filtering out disturbance. This mode gives raw ECAP values
        int flag_nonlinear_filtering;
        int flag_bad_U_capture;
        int flag_bad_V_capture;
        int flag_bad_W_capture;
        // On-interrupt ECAP decode. This mode gives ECAP values that are already free of disturbance.
        Uint32 good_capture_U[4];
        Uint32 good_capture_V[4];
        Uint32 good_capture_W[4];
        struct bad_values {
            Uint32 on1, off1, on2, off2;
        } u_ecap_bad1, v_ecap_bad1, w_ecap_bad1, u_ecap_bad2, v_ecap_bad2, w_ecap_bad2;
        // ISR counting
        Uint64 ECapIntCount[3];
        Uint64 ECapPassCount[3];
        // ISR nesting
        int password_isr_nesting; // set to 178 in EPWM1 ISR
        int decipher_password[3];
} st_capture; // eCapture
typedef struct {
    // To show that REAL type is not very accurate 64 missing by counting 1e-4 sec to 10 sec.
        Uint32  test_integer;
        REAL    test_float;
    // Mode Changing During Experiment
        // int FLAG_ENABLE_PWM_OUTPUT; // 电机模式标志位
        // int DAC_MAX5307_FLAG; // for single core case
        // int AD_offset_flag2;
        // Uint16 Rotor_angle_selection; // delete?
        // REAL Set_manual_current_iq,Set_manual_current_id,Set_manual_rpm;
    // Mode Changing During Experiment Debug
        // REAL OverwriteSpeedOutLimitDuringInit; // = 2;
        REAL overwrite_vdc; // = 180;
        int flag_overwite_vdc; // = FALSE;
        int flag_use_ecap_voltage; // = 0;
        int flag_experimental_initialized;
        int FLAG_TUNING_CURRENT_SCALE_FACTOR; // for comm (special mode for calibrate current sensor channel gain)
        int flag_do_inverter_characteristics; // for comm
        // int Seletc_exp_operation; // for exp
        int flag_auto_id_cmd; // for slow reversal
        int FLAG_INVERTER_NONLINEARITY_COMPENSATION;
    // Select Algorithm
        // int Select_algorithm;
        REAL omg_elec; // updated in observers
        REAL theta_d; // updated in observers
    //
    int16 sendCurrentCommandFlag;
    int16 sendSpeedCommandFlag;
    // 
    REAL ia;
    REAL dist_ua;
} st_global_variables; // globals

struct ControllerForExperiment{

    /* Basic quantities */
    REAL timebase;
    Uint64 timebase_counter;

    /* Machine parameters */
    st_motor_parameters *motor;

    /* Peripheral configurations */
    st_enc *enc;
    // st_enc *enc_hall;
    st_psd *psd; // phase sequence detection

    /* Inverter */
    st_InverterNonlinearity *inv;
    SVGENDQ svgen1;
    SVGENDQ svgen2;

    /* Capture (line-to-line voltage) */
    st_capture *cap;

    /* Console */
    st_global_variables *g;

    /* Black Box Model | Controller quantities */
    st_controller_inputs  *i;
    st_controller_states  *s;
    st_controller_outputs *o;
    // REAL inputs[8];
    // REAL states[5];
    // REAL outputs[4];
};
//Observer for speed reconstruction, OFSR
struct ObserverForSpeedReconstruction{
        /* Common */
        struct RK4_SR_DATA{
            REAL us[2];
            REAL is[2];
            REAL us_curr[2];
            REAL is_curr[2];
            REAL us_prev[2];
            REAL is_prev[2];

            REAL is_lpf[2];
            REAL is_hpf[2];
            REAL is_bpf[2];

            REAL current_lpf_register[2];
            REAL current_hpf_register[2];
            REAL current_bpf_register1[2];
            REAL current_bpf_register2[2];

            // REAL omg_elec; // omg_elec = npp * omg_mech
            // REAL theta_d;
        } rk4;
//Observer for speed reconstruction
        struct Chen21_ESO_AF{
                #define NS_CHEN_2021 4
                REAL xPos;
                REAL xOmg;
                REAL xTL;
                REAL xPL; // rotatum
                REAL x[NS_CHEN_2021];
                REAL ell[NS_CHEN_2021];

                int bool_ramp_load_torque; // TRUE for 4th order ESO
                REAL omega_ob; // one parameter tuning
                REAL set_omega_ob;

                REAL output_error_sine; // sin(\tilde\vartheta_d)
                REAL output_error; // \tilde\vartheta_d, need to detect bound jumping 

                REAL xTem;
            } esoaf;
};
//Observer for speed reconstruction
extern struct ObserverForSpeedReconstruction OFSR;
#define US_SR(X)   OFSR.rk4.us[X]
#define IS_SR(X)   OFSR.rk4.is[X]
#define US_SR_C(X) OFSR.rk4.us_curr[X] // 当前步电压是伪概念，测量的时候，没有电压传感器，所以也测量不到当前电压；就算有电压传感器，由于PWM比较寄存器没有更新，输出电压也是没有变化的。
#define IS_SR_C(X) OFSR.rk4.is_curr[X]
#define US_SR_P(X) OFSR.rk4.us_prev[X]
#define IS_SR_P(X) OFSR.rk4.is_prev[X]
void init_rk4();
typedef void (*pointer_flux_estimator_dynamics)(REAL t, REAL *x, REAL *fx);
    void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);

//ESO
#define ESOAF_OMEGA_OBSERVER 2000 //ESO的增益，为了方便写到d_sim来在线debug
void eso_one_parameter_tuning(REAL omega_ob);
void rhf_dynamics_ESO(REAL t, REAL *x, REAL *fx);
void init_esoaf();
void Main_esoaf_chen2021();

extern int axisCnt;
extern struct ControllerForExperiment CTRL_1;
extern struct ControllerForExperiment CTRL_2;
extern struct ControllerForExperiment *CTRL;
#define MOTOR   (*(*CTRL).motor)
#define ENC     (*(*CTRL).enc)
#define PSD     (*(*CTRL).psd)
// #define IN      (*(*CTRL).i)
// #define STATE   (*(*CTRL).s)
// #define OUT     (*(*CTRL).o)
#define INV     (*(*CTRL).inv)
#define CAP     (*(*CTRL).cap)
#define G       (*(*CTRL).g)
    // extern st_InverterNonlinearity t_inv;
    // #define INV     t_inv

#define PID_iD  (CTRL->s->iD)
#define PID_iQ  (CTRL->s->iQ)
#define PID_Speed  (CTRL->s->Speed)
#define PID_Position  (CTRL->s->Position)
// #define PID_iX  (CTRL->s->iX)
// #define PID_iY  (CTRL->s->iY)

//ESO

void commissioning();
void allocate_CTRL(struct ControllerForExperiment *p);

void overwrite_sweeping_frequency();
void _user_Check_ThreeDB_Point( REAL Fbk, REAL Ref);

// 速度观测器
REAL PostionSpeedMeasurement_MovingAvergage(int32 QPOSCNT, st_enc *p_enc);

/* Commission */
#if PC_SIMULATION
    #define ENABLE_COMMISSIONING TRUE /*Simulation*/
    #define SELF_COMM_INVERTER FALSE
    #define TUNING_CURRENT_SCALE_FACTOR_INIT FALSE2
#else
    #define ENABLE_COMMISSIONING FALSE /*Experiment*/
    #define SELF_COMM_INVERTER FALSE
    #define TUNING_CURRENT_SCALE_FACTOR_INIT FALSE
    /*As we use (*CTRL).o->iab_cmd for look up, now dead-time compensation during ENABLE_COMMISSIONING is not active*/
#endif
#define EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B FALSE

#endif
