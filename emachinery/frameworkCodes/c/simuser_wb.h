#ifndef SIMUSER_WB_H
#define SIMUSER_WB_H

#if WHO_IS_USER == USER_WB
    #include "ACMSim.h"

    /* Hit Wall Anaylsis*/
    #define NUMBER_OF_HIT_WALL_VAR_RATIO 10
    #define NUMBER_OF_HIT_WALL_VAR_RATIO_MINUS_ONE_INV 1 / (NUMBER_OF_HIT_WALL_VAR_RATIO-1)
    typedef struct {
        REAL Vdc_limit_ratio[NUMBER_OF_HIT_WALL_VAR_RATIO]; // From 200% to 10%
        REAL Motor_Current_limit_ratio[NUMBER_OF_HIT_WALL_VAR_RATIO]; // From 200% to 10%
    }wubo_Hit_Wall;
    extern wubo_Hit_Wall wubo_HW;
    void _init_wubo_Hit_Wall();



    /* Inner Loop Controller */
    typedef struct {
        REAL KFB;
        REAL KFB_Term;
        REAL KFB_Term_OutLimit;
        REAL FOC_CLBW;
        REAL FOC_VLBW;
        REAL FOC_CLBW_Hz;
        REAL FOC_VLBW_Hz;
    } SpeedInnerLoop;
    extern SpeedInnerLoop SIL_Controller;
    void _user_wubo_WC_Tuner_Online();
    void _user_wubo_TI_Tuner_Online();
    void _user_wubo_SpeedInnerLoop_controller(st_pid_regulator *r, SpeedInnerLoop *r_IL);
    void _init_WC_Tuner();
    REAL _init_WC_Tuner_Part2(REAL zeta, REAL omega_n, REAL max_CLBW_PER_min_CLBW);

    /* Dead Predict Current Controller */
    void DPCC();

    /* Harnefors 1998 back calculations*/
    typedef struct {
        REAL Err_bar;
        REAL I_Term_prev;
        REAL I_Term_prev_iD;
        REAL I_Term_prev_iQ;
        REAL K_INVERSE_iD;
        REAL K_INVERSE_iQ;
    } Harnefors_1998_BackCals;
    extern Harnefors_1998_BackCals Harnefors_1998_BackCals_Variable;
    #define HARNEFORS_1998_VAR Harnefors_1998_BackCals_Variable
    #if PC_SIMULATION
        #define HARNEFORS_UMAX d_sim.init.Vdc * 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION
    #else
        #define HARNEFORS_UMAX Axis->vdc * 0.5773 * d_sim.CL.LIMIT_DC_BUS_UTILIZATION
    #endif
    void _init_Harnerfors_1998_BackCalc();
    void _user_Harnefors_back_calc_PI_antiWindup(st_pid_regulator *r, Harnefors_1998_BackCals *H, REAL K_inverse, REAL coupling_term);
    void _user_wubo_FOC(REAL theta_d_elec, REAL iAB[2]);

    /* Position Loop Controller */
    void _user_wubo_get_SpeedFeedForward_for_PositionLoop(REAL Theta);
    void _user_wubo_PositionLoop_controller(REAL Theta, REAL Speed_FeedForward);

    /* For Sweeping & Signal Generator */
    void _user_wubo_Sweeping_Command();
    void _init_wubo_SignalGE();
    REAL wubo_Signal_Generator(int signal_mode);
    void _user_wubo_Check_ThreeDB_Point( REAL varOmega, REAL cmd_varOmega);
    #define GENERATE_D_CURRENT_SINE 1
    #define GENERATE_Q_CURRENT_SINE 2
    #define GENERATE_SPEED_SINE     3
    #define GENERATE_SPEED_SAUARE_WAVE_WITH_INV 4
    #define GENERATE_NYQUIST_SIGNAL 91
    typedef struct {
        REAL idq_amp[2];  // Unit : A
        REAL idq_freq[2]; // Unit : Hz
        REAL speed_amp;   // Unit : RPM
        REAL speed_freq;  // Unit : Hz
        REAL squareWave_amp; // Unit : XX
        REAL squareWave_quarter_cycle; // Unit : Second 1/4 cycle
        REAL squareWave_total_time; // Unit : Second
        REAL signal_out;
    } wubo_SignalGenerator;
    extern wubo_SignalGenerator wubo_SG;

    /* Inverter Compensation Method */
    void wubo_inverter_Compensation(REAL iAB[2]);
    REAL wubo_inverter_Compensation_get_dist_voltage(REAL current);
    REAL CJH_LUT_index_inverter_compensation_get_dist_voltage(REAL current_value);
    void sul_inverter_Compensation();

    /* 声明 inverter_Compensation 专用变量 */
    #define NUMBER_OF_COMPENSATION_POINTS 7
    extern REAL inverter_current_point[NUMBER_OF_COMPENSATION_POINTS];
    extern REAL inverter_voltage_point[NUMBER_OF_COMPENSATION_POINTS];


    /* Parameter Mismatch Test */
    void _wubo_ParaMis_asTime();
    void _init_wubo_ParaMis();
    #define TOTAL_PARAMETER_MISMATCH_PERIOD         7
    #define TOTAL_PARAMETER_MISMATCH_PERIOD_INV  1 / (TOTAL_PARAMETER_MISMATCH_PERIOD-1)
    #define TOTAL_PARAMETER_MISMATCH_PERIOD_MINUS_ONE_INV  1 / (TOTAL_PARAMETER_MISMATCH_PERIOD-1)
    #define LD_PARAMETER_MISMATCH_MODE 1
    #define LQ_PARAMETER_MISMATCH_MODE 2
    #define Rs_PARAMETER_MISMATCH_MODE 3
    #define Js_PARAMETER_MISMATCH_MODE 4
    #define KE_PARAMETER_MISMATCH_MODE 5
    typedef struct {
        // Five mismatch parameters
        REAL percent_Ld;
        REAL percent_Lq;
        REAL percent_Rs;
        REAL percent_Js;
        REAL percent_KE;
        // The maximum and minimum percentage of the mismatch
        REAL percent_max;
        REAL percent_min;
        // percent_Para is initialized by the max and min in user.yaml file 
        // I want to assign THIS value to above percent_Ld, percent_Lq, percent_Rs, percent_Js, percent_KE
        // But this is not simple enough cuz it use so much FLASH to store percent_Para[]
        REAL percent_Para[(int)TOTAL_PARAMETER_MISMATCH_PERIOD];
        
        REAL total_exp_time; // Record the total time of the experiment
    } wubo_Parameter_mismatch;
    extern wubo_Parameter_mismatch wubo_ParaMis;
    
    /* Items */
    void NB_MODE_codes();
    void UDQ_GIVEN_TEST();

    /* Making Noise Mode */
    #define NUMBER_OF_FREQUENCY_LEVEL 3



    /* Function Marco */
    #define INNER_LOOP_SENSITIVITY_ANALYSIS(debug) \
    do { \
        if ((debug)->bool_Parameter_Mismatch_test == TRUE) { \
            _wubo_ParaMis_asTime(); \
            _user_wubo_WC_Tuner_Online(); \
        } \
    } while (0) \

    #define HANDLE_PARAMETER_MISMATCH(mode, index) \
    do { \
        switch (mode) { \
            case LD_PARAMETER_MISMATCH_MODE: \
                wubo_ParaMis.percent_Ld = wubo_ParaMis.percent_Para[index]; \
                break; \
            case LQ_PARAMETER_MISMATCH_MODE: \
                wubo_ParaMis.percent_Lq = wubo_ParaMis.percent_Para[index]; \
                break; \
            case Rs_PARAMETER_MISMATCH_MODE: \
                wubo_ParaMis.percent_Rs = wubo_ParaMis.percent_Para[index]; \
                break; \
            case Js_PARAMETER_MISMATCH_MODE: \
                wubo_ParaMis.percent_Js = wubo_ParaMis.percent_Para[index]; \
                break; \
            case KE_PARAMETER_MISMATCH_MODE: \
                wubo_ParaMis.percent_KE = wubo_ParaMis.percent_Para[index]; \
                break; \
            default: \
                break; \
        } \
    } while (0) \

#endif
#endif
