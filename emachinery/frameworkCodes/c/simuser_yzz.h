#ifndef SIMUSER_YZZ_H
#define SIMUSER_YZZ_H

#if WHO_IS_USER == USER_YZZ

	#define CORRECTION_4_SHARED_FLUX_EST d_sim.init.KE
    #define U_MOTOR_KE                   d_sim.init.KE
    #define IM_FLUX_COMMAND_DC_PART     d_sim.init.KE // 1.3593784874408608
    #define IM_FLUX_COMMAND_SINE_PART   0.0
    void rk4_init();
/* User */
#include "super_config.h"
#include "main_switch.h"


/* Algorithms */

    /* Select [Shared Flux Estimator] */
    // #define AFE_USED FE.clfe4PMSM
    
    #define AFE_USED FE.htz
    // #define AFE_USED FE.huwu
    // #define AFE_USED FE.htz // this is for ESO speed estimation
    // #define AFE_USED FE.picorr // this is for ESO speed estimation

    /* Tuning [Shared Flux Estimator] */
        /* AFEOE or CM-VM Fusion */
        #define AFEOE_OMEGA_ESTIMATOR 5 // [rad/s] //0.5 // 5 for slow reversal
            #define AFEOE_KP (200) // (200) // ONLY KP
            #define AFEOE_KI (0) // ONLY KP
        #define AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED FALSE // no need

        /* Hu Wu 1998 recommend tau_1_inv=20 rad/s */
        // #define AFE_21_HUWU_TAU_1_INVERSE (20)
        #define AFE_21_HUWU_TAU_1_INVERSE (200.0) // [rad/s] Alg 2: 0.1 rad/s gives better performance near zero speed, but the converging rate is slower comapred to 0.9 rad/s.
        // #define AFE_21_HUWU_TAU_1_INVERSE (7.5) // [rad/s] Alg 3: increase this will reduce the transient converging time
        #define AFE_21_HUWU_KP (0.2)  //[rad/s]
        #define AFE_21_HUWU_KI (0.5) //[rad/s]

        /* Holtz 2002 */
        // #define HOLTZ_2002_GAIN_OFFSET 20

    // Marino05 调参 /// default: (17143), (2700.0), (1000), (1), (0)
    #define GAMMA_INV_xTL 17142.85714285714
    #define LAMBDA_INV_xOmg 10000 // 2700.0 is too large, leading to unstable flux amplitude contorl
    #define DELTA_INV_alpha (0*500) // 1000
    #define xAlpha_LAW_TERM_D 1 // regressor is commanded d-axis rotor current, and error is d-axis flux control error.
    #define xAlpha_LAW_TERM_Q 0 // regressor is commanded q-axis stator current, and error is q-axis flux control error.
    // 磁链反馈用谁 /// "htz",,ohtani",picorr",lascu",clest",harnefors
    #define IFE FE.picorr
    // #define IFE FE.htz
    #define FLUX_FEEDBACK_ALPHA         IFE.psi_2[0]
    #define FLUX_FEEDBACK_BETA          IFE.psi_2[1]
    #define OFFSET_COMPENSATION_ALPHA   IFE.u_offset[0]
    #define OFFSET_COMPENSATION_BETA    IFE.u_offset[1]

    // Ohtani 磁链观测系数配置/// default: 5
    // Ohtani 建议取值和转子时间常数相等
    #define GAIN_OHTANI (5)
    #define VM_OHTANI_CORRECTION_GAIN_P (5)
    /* B *//// default: P=5, I=2.5
    #define VM_PROPOSED_PI_CORRECTION_GAIN_P 30// 
    #define VM_PROPOSED_PI_CORRECTION_GAIN_I 20//80000//2.5 //2  // (2.5)
    /* No Saturation */
    #define VM_NOSAT_PI_CORRECTION_GAIN_P 20// 难调
    #define VM_NOSAT_PI_CORRECTION_GAIN_I 0//50000//10000//1000//80000//2.5 //2  // (2.5)
    /* Saturation_time_Without_Limiting */
    #define STWL_GAIN_KP 
    #define STWL_GAIN_KI 
    /* C *//// default: P=0.125*5, I=0.125*2.5, KCM=0
    #define OUTPUT_ERROR_CLEST_GAIN_KP (0.04)
    #define OUTPUT_ERROR_CLEST_GAIN_KI (0.5)
    #define OUTPUT_ERROR_CLEST_GAIN_KCM (0.0002*0.8)
    /* Closed Loop flux estimator for PMSM*/
    #define OUTPUT_ERROR_CLEF4PMSM_GAIN_KP (0.04)
    #define OUTPUT_ERROR_CLEF4PMSM_GAIN_KI (0.5)
    /* Holtz 2002 */// default: 20
    #define HOLTZ_2002_GAIN_OFFSET 10 //1 // 20 is too large, causing unstable control during reversal
    #define HOLTZ_2002_GAIN_OFFSET_SWTL 1
    /* Harnefors SCVM 2003 */// default: 2
    #define GAIN_HARNEFORS_LAMBDA 2

	// #if 1
        #define IM_ELECTRICAL_SPEED_FEEDBACK    marino.xOmg // (*CTRL).i->omg_elec
        #define IM_ELECTRICAL_POSITION_FEEDBACK marino.xRho // (*CTRL).i->theta_d_elec
	// #endif


/* from share flux estimator */ 

#ifndef SHARED_FLUX_ESTIMATOR_H
#define SHARED_FLUX_ESTIMATOR_H


/* Macro for External Access Interface */
#define US(X)   OBSV.rk4.us[X]
#define IS(X)   OBSV.rk4.is[X]
#define US_C(X) OBSV.rk4.us_curr[X] // 当前步电压是伪概念，测量的时候，没有电压传感器，所以也测量不到当前电压；就算有电压传感器，由于PWM比较寄存器没有更新，输出电压也是没有变化的。
#define IS_C(X) OBSV.rk4.is_curr[X]
#define US_P(X) OBSV.rk4.us_prev[X]
#define IS_P(X) OBSV.rk4.is_prev[X]

    #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
    // void init_rk4();

    #define AFE_11_OHTANI_1992 0
    #define AFE_12_HOLTZ_QUAN_2002 0
    #define AFE_13_LASCU_ANDREESCUS_2006 0
    #define AFE_14_MY_PROPOSED 0
    #define AFE_21_HU_WU_1998 0
    #define AFE_22_STOJIC_2015 0
    #define AFE_23_SCVM_HARNEFORS_2003 0
    #define AFE_24_OE 0
    #define AFE_25_VM_CM_FUSION 0 // See ParkSul2014Follower2020
    #define AFE_31_HOLT_QUAN_2003_LPF_ORIGINIAL 0
    #define AFE_32_HOLT_QUAN_2003_INTEGRATOR 0
    #define AFE_33_EXACT_OFFSET_COMPENSATION 0
    #define AFE_34_ADAPTIVE_LIMIT 0
    #define AFE_35_SATURATION_TIME_DIFFERENCE  1
    #define AFE_36_TOP_BUTT_EXACT_COMPENSATION 0
    #define AFE_37_NO_SATURATION_BASED  0
    #define AFE_38_OUTPUT_ERROR_CLOSED_LOOP 0
    #define AFE_39_SATURATION_TIME_WITHOUT_LIMITING 0
    #define AFE_40_JO_CHOI_METHOD 0
    #define AFE_41_LascuAndreescus2006 0
    #define AFE_42_BandP 0
    #define AFE_43_SuperTwistingA 0
    #define AFE_44_ORTEGA_2011 0
    #define AFE_45_CMwithDynamicCurrent 1
    #define ALG_PLL_norm 1//DSP-based control of sensorless IPMSM drives for wide-speedrange operation
    #define ALG_AKT_SPEED_EST_AND_RS_ID 0
    #define ALG_Awaya_InertiaId 0
    #define ALG_qaxis_inductance_identification 0
    #define RS_IDENTIFICATION 0
    #define LQ_IDENTIFICATION 0
    typedef void (*pointer_flux_estimator_dynamics)(REAL t, REAL *x, REAL *fx);
    void general_1states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
    void general_2states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
    void general_3states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
    void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
    void general_5states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
    void general_6states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
    void general_8states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);
    void general_10states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs);

    struct SharedFluxEstimatorForExperiment{
        #if AFE_11_OHTANI_1992 
        #endif
        #if AFE_12_HOLTZ_QUAN_2002 
        #endif
        #if AFE_13_LASCU_ANDREESCUS_2006 
        #endif
        #if AFE_14_MY_PROPOSED 
        #endif
        #if AFE_21_HU_WU_1998
            struct Declare_HuWu1998{
                #define NS_HuWu1998 3
                /* State */
                REAL x[NS_HuWu1998];
                REAL psi_1[2]; // Stator Flux Estimate
                REAL psi_comp; // flux amplitude for compensation
                REAL cosT;
                REAL sinT;
                REAL psi_2_limited[2]; // Active flux limited 
                /* Coefficients */
                REAL KP; // alg 3
                REAL KI; // alg 3
                REAL tau_1_inv;
                REAL limiter_KE; // alg 2
                /* Output */
                REAL theta_d;
                REAL inner_product_normalized;
                REAL output_error[2]; // \tilde i_q
                REAL psi_2[2]; // Active Flux Estimate
                REAL stator_flux_ampl;
                REAL stator_flux_ampl_limited;
                REAL active_flux_ampl;
                REAL active_flux_ampl_limited;
            } huwu;
        #endif
        #if AFE_22_STOJIC_2015 
        #endif
        #if AFE_23_SCVM_HARNEFORS_2003 
        #endif
        #if AFE_24_OE || AFE_25_VM_CM_FUSION 
            struct ActiveFluxEstimator{
                /* Coefficients */
                REAL ActiveFlux_KP;
                REAL ActiveFlux_KI;
                REAL omega_est;
                REAL set_omega_est;
                /* Output */
                REAL active_flux_ampl;
                REAL active_flux_ampl_lpf;
                REAL theta_d;
                REAL output_error[2];    // CM mismatch 
                REAL output_error_dq[2]; // CM mismatch to dq frame
                REAL psi_1[2]; // Stator Flux Estimate
                REAL psi_2[2]; // Active Flux Estimate
                REAL u_offset[2];
                /* State */
                REAL x[4];
                REAL cosT;
                REAL sinT;
                REAL dot_product; // = emf . psi_1
                REAL cross_product;
                /* Ampl Limiter */
                int  limiter_Flag;
                REAL limiter_KE;
                REAL k_af;
                REAL psi_2_limited[2];
                REAL theta_e;
            } AFEOE;
        #endif
        #if AFE_31_HOLT_QUAN_2003_LPF_ORIGINIAL 
        #endif
        #if AFE_32_HOLT_QUAN_2003_INTEGRATOR 
        #endif
        #if AFE_33_EXACT_OFFSET_COMPENSATION 
        #endif
        #if AFE_34_ADAPTIVE_LIMIT 
        #endif
        #if AFE_35_SATURATION_TIME_DIFFERENCE
            struct Variables_Holtz2003{
                REAL emf_stator[2];

                REAL psi_1[2];
                REAL psi_2[2];
                REAL psi_2_ampl;
                REAL psi_2_ampl_lpf;
                REAL u_offset[2];
                // REAL u_offset_intermediate[2];
                // REAL u_offset_ultra_low_frequency_component[2];
                REAL sat_time_offset[2];
                REAL psi_2_prev[2];
                REAL psi_1_prev[2];

                REAL psi_1_nonSat[2];
                REAL psi_2_nonSat[2];

                REAL psi_1_min[2];
                REAL psi_1_max[2];
                REAL psi_2_min[2];
                REAL psi_2_max[2];

                REAL theta_d;
                REAL cosT;
                REAL sinT;

                REAL rs_est;
                REAL rreq_est;

                REAL Delta_t;
                REAL Delta_t_last;

                REAL u_off_direct_calculated[2];          //
                REAL u_off_original_lpf_input[2];         // holtz03 original (but I uses integrator instead of LPF)
                REAL u_off_saturation_time_correction[2]; // saturation time based correction
                REAL u_off_calculated_increment[2];       // exact offset calculation for compensation
                REAL gain_off;

                REAL accumulated__u_off_saturation_time_correction[2];
                REAL sign__u_off_saturation_time_correction[2];

                long int count_negative_cycle;
                long int count_positive_cycle;

                long int count_negative_in_one_cycle[2];
                long int count_positive_in_one_cycle[2];
                long int negative_cycle_in_count[2];
                long int positive_cycle_in_count[2];

                int    flag_pos2negLevelA[2];
                int    flag_pos2negLevelB[2];
                REAL time_pos2neg[2];
                REAL time_pos2neg_prev[2];

                int    flag_neg2posLevelA[2];
                int    flag_neg2posLevelB[2];
                REAL time_neg2pos[2];
                REAL time_neg2pos_prev[2];

                REAL psi_aster_max;

                REAL maximum_of_sat_min_time[2];
                REAL maximum_of_sat_max_time[2];
                REAL sat_min_time[2];
                REAL sat_max_time[2];
                REAL sat_min_time_reg[2];
                REAL sat_max_time_reg[2];
                REAL extra_limit;
                int flag_limit_too_low;

                // REAL ireq[2];
                REAL field_speed_est;
                REAL omg_est;
                REAL slip_est;
                REAL theta_e;
            } htz;
        #endif
        #if AFE_36_TOP_BUTT_EXACT_COMPENSATION 
        #endif

        #if AFE_38_OUTPUT_ERROR_CLOSED_LOOP
        struct ClosedLoopFluxEstimator4PMSM
        {
            REAL x[4];
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
            REAL ampl_psi_2;
            REAL correction_integral_term[2];
            REAL current_error[2];
            REAL current_estimate[2];
            REAL hat_psi_d2; 
            REAL theta_d;
            REAL theta_e;
        }clfe4PMSM;
        
        #endif


        #if AFE_43_SuperTwistingA 
        struct SuperTwistingA{
            REAL current_estimate[2];
            REAL k1;
            REAL k2;

            REAL current_error[2];
            // REAL current_sqrt[2];
            // REAL current_integral[2];
            REAL psi_A[2];
            REAL emf_A[2];
            REAL theta_d;
            REAL theta_e;
            REAL theta_d_emf;
            REAL theta_e_emf;
            REAL k1_term[2];
            REAL k2_term[2];
            REAL x[6];
        } STA;
        #endif

        #if AFE_44_ORTEGA_2011
        struct Ortega2011{
            REAL x[2];
            REAL psi_1[2];
            REAL psi_2[2];
            REAL kp;
            REAL u_offset[2];
            REAL psi_sqr_error;
            REAL theta_d;
            REAL theta_e;
        } Ortega;
        #endif

        #if AFE_45_CMwithDynamicCurrent
        struct CMwithDynamicCurrent{
            REAL x[6];
            REAL cosT;
            REAL sinT;
            REAL udq[2];
            REAL idq[2];
            REAL emf_ss_dq[2];
            REAL tilde_idq[2];
            REAL hat_idq[2];
            REAL integral_tilde_id;
            REAL integral_tilde_iq;
            REAL hat_emf_A_ss;
            REAL theta_d;
            REAL gamma_id; // current observer gain
            REAL gamma_iq; // current observer gain
            REAL KP; // speed obserer gain
            REAL KI; // speed obserer gain
            REAL kp; // position observer gain
            REAL ki; // position observer gain
            REAL omega_elec;
            REAL tilde_theta_d;
            // REAL KA_inv;
        } CMDC;
        #endif
        struct Variables_Ohtani1992{
            // in AB frame
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
            // in DQ frame
            REAL psi_DQ1[2];
            REAL psi_DQ2[2];
            REAL psi_DQ2_prev[2];
            REAL deriv_psi_DQ1[2];
        } ohtani;
        // struct Variables_HuWu1998{
        //     REAL psi_1[2];
        //     REAL psi_2[2];
        //     REAL psi_2_ampl;
        //     REAL u_offset[2];
        // } huwu;
        struct Variables_HoltzQuan2002{
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
        } holtz02;
        // struct Variables_Holtz2003{ moved to shared flux estimator.h
        struct Variables_Harnefors2003_SCVM{
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
            REAL lambda;
        } harnefors;
        struct Variables_LascuAndreescus2006{
            REAL x[4];
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
            REAL correction_integral_term[2];
            REAL theta_d;
            REAL theta_e;
        } lascu;
        struct Variables_Stojic2015{
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
        } stojic;
        struct Variables_fluxModulusEstimator{
            REAL psi_DQ2[2];
            REAL psi_DQ1[2];
            REAL temp;
        } fme;
        struct Variables_ExactCompensationMethod{
            // D-Q
            REAL psi_DQ1[2];
            REAL psi_DQ2[2];
            REAL psi_DQ2_prev[2];

            // alpha-beta
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];

            REAL psi_2_prev[2];
            REAL psi_2_pprev[2];

            // sum_up_method
                // TOP
                int flag_top2top[2];
                REAL last_time_reaching_top[2];
                REAL Delta_time_top2top[2];
                REAL change_in_psi_2_top[2];
                REAL psi_2_last_top[2];
                REAL u_offset_estimated_by_psi_2_top_change[2];

                // BUTT
                int flag_butt2butt[2];
                REAL last_time_reaching_butt[2];
                REAL Delta_time_butt2butt[2];
                REAL change_in_psi_2_butt[2];
                REAL psi_2_last_butt[2];
                REAL u_offset_estimated_by_psi_2_butt_change[2];

                // REAL u_offset[2];
                REAL offset_voltage_compensation[2];

                REAL first_time_enter_top[2];
                REAL first_time_enter_butt[2];

                // 这个没用！
                // REAL top2top_sum[2];
                REAL top2top_count[2];
                // REAL psi_2_offset[2];
                // REAL u_offset_estimated_by_psi_2_sumup[2];

                REAL butt2butt_count[2];

                REAL filtered_compensation[2]; // psi_offset
                REAL filtered_compensation2[2];
                REAL u_offset_estimated_by_top_minus_butt[2];
                REAL u_offset_estimated_by_integration_correction[2];
                REAL u_offset_error[2];

                REAL bool_compensate_psi_offset[2];

                REAL psi_2_real_output[2];

                REAL psi_2_output[2];
                REAL psi_2_output2[2];
                REAL psi_2_output_last_top[2];
                REAL psi_2_output2_last_top[2];
                REAL psi_2_output_last_butt[2];
                REAL psi_2_output2_last_butt[2];

                REAL current_Lissa_move[2];
                REAL last_Lissa_move[2];

            REAL correction_integral_term[2];

            /* zero_crossing_method (below) */
            REAL psi_2_max[2]; // always not equal to zero
            REAL psi_2_min[2]; // always not equal to zero
            REAL psi_2_max_temp[2]; // can be reset to zero
            REAL psi_2_min_temp[2]; // can be reset to zero
            REAL psi_2_maxmin_difference[2];
            REAL psi_2_last_zero_level[2];
            REAL psi_2_last_zero_level_inLoopTheSame[2];
            REAL psi_2_moved_distance_in_LissajousPlot[2];
            // REAL offset_voltage[2];

            REAL flag_Pos_stageA[2];
            REAL flag_Pos_stageB[2];

            REAL flag_Neg_stageA[2];
            REAL flag_Neg_stageB[2];

            REAL Delta_t_PosMinusNeg[2];
            REAL Delta_t_NegMinusPos[2];
            REAL time_Pos[2];
            REAL time_Neg[2];

            REAL offset_voltage_by_zero_crossing[2];
        } exact;
        struct Variables_ProposedxRhoFramePICorrectionMethod{
            REAL x[4];
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
            REAL correction_integral_term[2];
            REAL theta_d;
            REAL cosT;
            REAL sinT;
        } picorr;
        struct Variables_ClosedLoopFluxEstimator{
            REAL x[5];
            REAL psi_1[2];
            REAL psi_2[2];
            REAL psi_2_ampl;
            REAL u_offset[2];
            REAL psi_dmu;
            REAL correction_integral_term[2];
        } clest;

    };
    extern struct SharedFluxEstimatorForExperiment FE;


    // #define AFEOE (FE.AFEOE)
    // #define huwu  (FE.huwu)
    // #define htz   (FE.htz)

    void simulation_test_flux_estimators();
        #if AFE_25_VM_CM_FUSION
            void Main_the_active_flux_estimator();
        #endif
        // void MainFE_HuWu_1998();
        void VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
        #if AFE_38_OUTPUT_ERROR_CLOSED_LOOP
            void Main_ClosedLoopFluxEstimatorForPMSM();
        #endif
        #if AFE_13_LASCU_ANDREESCUS_2006
            void VM_LascuAndreescus2006();
        #endif
        #if ALG_AKT_SPEED_EST_AND_RS_ID
            void SpeedEstimationFromtheVMBasedFluxEstimation();
            void RS_Identificaiton();
        #endif
        #if AFE_43_SuperTwistingA
            void Main_SuperTwistingA();
        #endif
        #if AFE_44_ORTEGA_2011
            void Main_Ortega2011();
        #endif
        #if ALG_PLL_norm
            void Main_PLL_norm(REAL emf[2]);
        #endif
    void init_afe();
    void init_FE();
    void init_FE_htz(); // Holtz 2003
    void init_No_Saturation_Based(); // No Saturation Based
    void init_ClosedLoopFluxEstimatorForPMSM(); // Closed Loop Flux Estimator For PMSM
        // observer declaration 
    void init_LascuAndreescus2006();
    void init_ake_Speed_Est_and_RS_ID();
    #endif

    #endif
    /* share flux estimator end */ 

    /* from pmsm observer */ 

    #ifndef ADD_PMSM_OBSERVER_H
    #define ADD_PMSM_OBSERVER_H

    #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        /* Select Algorithm 2*/
            #define ALG_NSOAF 1
            #define ALG_Park_Sul 2
            #define ALG_Chi_Xu 3
            #define ALG_Qiao_Xia 4
            #define ALG_CJH_EEMF 5
            #define ALG_Farza_2009 6
            #define ALG_Harnefors_2006 7
            #define ALG_ESOAF 10
            #define ALG_SynIFO 11
        // #define SELECT_ALGORITHM ALG_ESOAF
        #define SELECT_ALGORITHM 1
        // #define SELECT_ALGORITHM ALG_Chi_Xu

            #if SELECT_ALGORITHM == ALG_Chi_Xu
                #define PMSM_ELECTRICAL_SPEED_FEEDBACK    OBSV.chixu.xOmg
                #define PMSM_ELECTRICAL_POSITION_FEEDBACK OBSV.chixu.theta_d
            #elif SELECT_ALGORITHM == ALG_NSOAF
                #define PMSM_ELECTRICAL_SPEED_FEEDBACK    OBSV.nsoaf.xOmg // OBSV.harnefors.omg_elec
                #define PMSM_ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d // OBSV.harnefors.theta_d
            #elif SELECT_ALGORITHM == ALG_ESOAF
                #define PMSM_ELECTRICAL_SPEED_FEEDBACK    (-OFSR.esoaf.xOmg) // 薄片电机实验正iq产生负转速
                #define PMSM_ELECTRICAL_POSITION_FEEDBACK AFE_USED.theta_d
            #else
                // #define PMSM_ELECTRICAL_SPEED_FEEDBACK    G.omg_elec
                // #define PMSM_ELECTRICAL_POSITION_FEEDBACK G.theta_d

                // #define PMSM_ELECTRICAL_SPEED_FEEDBACK    parksul.xOmg
                // #define PMSM_ELECTRICAL_POSITION_FEEDBACK parksul.theta_d

                // #define PMSM_ELECTRICAL_SPEED_FEEDBACK    qiaoxia.xOmg
                // #define PMSM_ELECTRICAL_POSITION_FEEDBACK qiaoxia.theta_d

                #define PMSM_ELECTRICAL_SPEED_FEEDBACK    (*CTRL).i->omg_elec
                #define PMSM_ELECTRICAL_POSITION_FEEDBACK (*CTRL).i->theta_d_elec
            #endif

        /* Tuning Algorithm 2 */
        #define LOW_SPEED_OPERATION  1
        #define HIGH_SPEED_OPERATION 2
        #define OPERATION_MODE HIGH_SPEED_OPERATION

            /* Park.Sul 2014 FADO in replace of CM */
            #define PARK_SUL_OPT_1 (2*M_PI*60)
            #define PARK_SUL_OPT_2 (2*M_PI*35)
                #define PARK_SUL_T2S_1_KP (PARK_SUL_OPT_1*2)
                #define PARK_SUL_T2S_1_KI (PARK_SUL_OPT_1*PARK_SUL_OPT_1)
                #define PARK_SUL_T2S_2_KP (PARK_SUL_OPT_2*2)
                #define PARK_SUL_T2S_2_KI (PARK_SUL_OPT_2*PARK_SUL_OPT_2)
            #define PARK_SUL_CM_OPT 5 // [rad/s] pole placement
                #define PARK_SUL_CM_KP (PARK_SUL_CM_OPT*2)
                #define PARK_SUL_CM_KI (PARK_SUL_CM_OPT*PARK_SUL_CM_OPT)

        /* Chi.Xu 2009 SMO for EMF of SPMSM (Coupled position estimation via MRAS) */
        #define CHI_XU_USE_CONSTANT_SMO_GAIN TRUE
            #define CHI_XU_SIGMOID_COEFF  500 /*比200大以后，在实验中无感速度稳态误差不会再减小了，但是会影响慢反转*/
        #if OPERATION_MODE == LOW_SPEED_OPERATION
            /* note ell4Zeq is -0.5 */
            #define CHI_XU_USE_CONSTANT_LPF_POLE TRUE
            #if PC_SIMULATION
                #define CHI_XU_SMO_GAIN_SCALE 10.0  //2
                #define CHI_XU_LPF_4_ZEQ    5.0   //10.0
            #else
                #define CHI_XU_SMO_GAIN_SCALE 10 /*取2实验无感稳态不稳，取5慢反转勉强成功，取10慢反转成功*/
                #define CHI_XU_LPF_4_ZEQ    (5.0) /*这项过大（eg=100）会导致角度稳态误差，忘记了你就试试看，取=2，=5，=10，=100分别仿！真！看看。*/
            #endif

            #define CHI_XU_SPEED_PLL_KP (500*2.0) // [rad/s]
            #define CHI_XU_SPEED_PLL_KI (500*500.0)
        #elif OPERATION_MODE == HIGH_SPEED_OPERATION
            /* note ell4Zeq will become 1 */
            #define CHI_XU_SMO_GAIN_SCALE  1.5
            #define CHI_XU_LPF_4_ZEQ       10.0
            #define CHI_XU_USE_CONSTANT_LPF_POLE FALSE
            #define CHI_XU_SPEED_PLL_KP (2*500) // [rad/s] 3000 = 阶跃转速不震荡，8000=阶跃转速很震荡
            #define CHI_XU_SPEED_PLL_KI (10e4) // 从350000减少为150000，可以减少稳态估计转速的波动
        #endif

        /* Qiao.Xia 2013 SMO for EMF of SPMSM */
            #define QIAO_XIA_SIGMOID_COEFF  5000 //200 // 20
            #define QIAO_XIA_SMO_GAIN       1.5 //1.5     // 1.5
            #define QIAO_XIA_MRAS_GAIN      500 //500       // 50
            #define QIAO_XIA_ADAPT_GAIN     500 //2000 // 250 // 100

        /* CHEN 2020 NSO with Active Flux Concept */
            // #define NSOAF_SPMSM // use AP Error
            #define NSOAF_IPMSM // use only OE
            #define TUNING_IGNORE_UQ TRUE
            #define NSOAF_OMEGA_OBSERVER 20 // >150 [rad/s] // cannot be too small (e.g., 145, KP will be negative),
                #define NSOAF_TL_P (1) // 1 for experimental starting // 4 for 1500 rpm // 2 for 800 rpm
                #define NSOAF_TL_I (30)
                #define NSOAF_TL_D (0)

        /* CHEN 2021 ESO with Active Flux Concept */
            // #define ESOAF_OMEGA_OBSERVER 10
            // #define ESOAF_OMEGA_OBSERVER 30 // 30 gives acceptable steady state speed ripple, 200
            // #define ESOAF_OMEGA_OBSERVER 150 // 150 gives acceptable disturbance rejection when sudden 3 A load is applied and keeps the system not stop when Vdc changes from 150 V to 300 V.
            // #define ESOAF_OMEGA_OBSERVER 200 // 200 gives acceptable disturbance rejection when load changes between 1.5 A and 3 A.

        /* Farza 2009 for EMMF */
            #define FARZA09_HGO_EEMF_VARTHETA 10
            #define FARZA09_HGO_EEMF_GAMMA_OMEGA_INITIAL_VALUE 10

        /* CJH EEMF AO Design */
            #define CJH_EEMF_K1 (100)
            #define CJH_EEMF_K2 (CJH_EEMF_K1*CJH_EEMF_K1*0.25) // see my TCST paper@(18)
            #define CJH_EEMF_GAMMA_OMEGA (5e6)

    #endif
        /* Harnefors 2006 */


    /* One Big Struct for all PMSM observers */
    struct ObserverForExperiment{
        /* Common */
        REAL theta_d;
        REAL varOmega;
        REAL count_for_encoder;
        REAL sum_for_encoder;
        REAL aver_for_encoder[10];
        struct RK4_DATA{
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
    /* Qiao.Xia 2013 SMO for Emf for SPMSM */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_Qiao_Xia

        struct Declare_QiaoXia2013{
            #define NS_QIAO_XIA_2013 5
            REAL x[NS_QIAO_XIA_2013];
            REAL xIab[2];
            REAL xEmf[2];
            REAL xOmg;
            REAL theta_d;
            REAL xEmf_raw[2]; // SM terms
            REAL output_error[2];
            REAL sigmoid_coeff;
            REAL smo_gain;
            REAL adapt_gain;
            REAL mras_gain;
        } qiaoxia;
        #define qiaoxia OBSV.qiaoxia
    #endif

    /* Chi.Xu 2009 SMO for Emf for SPMSM */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_Chi_Xu

        struct Declare_ChiXu2009{
            #define NS_CHI_XU_2009 6
            REAL x[NS_CHI_XU_2009];
            REAL xIab[2];
            REAL xZeq[2];
            REAL xOmg;
            REAL xTheta_d; // PLL
            REAL theta_d;
            REAL xEmf_raw[2]; // SM terms
            REAL output_error[2];

            REAL sigmoid_coeff;
            REAL smo_gain;
            REAL ell4xZeq; // VSS
            REAL omega_lpf_4_xZeq;
            REAL omega_lpf_4_xZeq_const_part;
            REAL PLL_KP;
            REAL PLL_KI;
            REAL smo_gain_scale; // 几倍反电势
        } chixu;
        // #define OBSV.chixu OBSV.OBSV.chixu
    #endif

    /* Park.Sul 2014 FADO in replace of CM */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_Park_Sul

        struct Declare_ParkSul2014{
            #define NS_PARK_SUL_2014 12
            REAL x[NS_PARK_SUL_2014];
            REAL xD[2]; // estimated disturbance
            REAL xPsi1[2]; // corrected stator flux estimate
            REAL xHatPsi1[2]; // auxilliary stator flux estimate (not used)
            REAL xPsi2[2]; // rotor flux estimate
            REAL xPsi2_Limited[2]; // limited rotor flux
            REAL xPsi2_Amplitude;
            REAL xT2S_1[2];
            REAL xT2S_2[2];
            REAL t2s_1_sin_error;
            REAL t2s_2_sin_error;
            REAL xCM_state[2];

            REAL xTheta_d;
            REAL xOmg;

            REAL theta_d; // angle of estimated active flux

            REAL output_error[2];
            REAL internal_error[2];

            REAL omega_f; // estimated stator flux angular speed

            REAL T2S_1_KP;
            REAL T2S_1_KI;
            REAL T2S_2_KP;
            REAL T2S_2_KI;
            REAL CM_KP;
            REAL CM_KI;
            REAL k_df;
            REAL k_af;
            REAL limiter_KE;
            REAL limiter_Flag;
        } parksul;
        #define parksul OBSV.parksul
    #endif

    /* Natural Speed Observer with active flux concept Chen2020 */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_NSOAF

        struct Chen20_NSO_AF{
            #define NS_CHEN_2020 6
            REAL xIq;
            REAL xOmg;
            REAL xTL;
            REAL x[NS_CHEN_2020];
            REAL xBest[3]; // the actual 3 states

            REAL KP;
            REAL KI;
            REAL KD;
            REAL omega_ob; // one parameter tuning
            REAL set_omega_ob;

            REAL output_error; // \tilde i_q

            REAL active_power_real;
            REAL active_power_est;
            REAL active_power_error;

            REAL xTem;

            /* Select Signals from Block Diagram*/
            REAL LoadTorquePI;
            REAL load_torque_pid_output;
            REAL q_axis_voltage;
        } nsoaf;
        // #define OBSV.nsoaf OBSV.OBSV.nsoaf
    #endif

    /* Extended State Observer with active flux concept Chen2021 */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_ESOAF

        // struct Chen21_ESO_AF{
        //     #define NS_CHEN_2021 4
        //     REAL xPos;
        //     REAL xOmg;
        //     REAL xTL;
        //     REAL xPL; // rotatum
        //     REAL x[NS_CHEN_2021];
        //     REAL ell[NS_CHEN_2021];

        //     int bool_ramp_load_torque; // TRUE for 4th order ESO
        //     REAL omega_ob; // one parameter tuning
        //     REAL set_omega_ob;

        //     REAL output_error_sine; // sin(\tilde\vartheta_d)
        //     REAL output_error; // \tilde\vartheta_d, need to detect bound jumping 

        //     REAL xTem;
        // } esoaf;
        // // #define OFSR.esoaf OBSV.OFSR.esoaf
    #endif

    /* Farza 2009 HGO */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_Farza_2009

        struct FARZA09_EEMF_HGO{
            REAL xPsi[2];
            REAL xEemf[2];
            REAL xOmg;
            REAL xGammaOmg;
            REAL xPhi[4];

            // output
            REAL theta_d;

            REAL output_error[2]; // \varepsilon

            REAL vartheta;
            REAL vartheta_inv;
        } hgo4eemf;
        // #define OBSV.hgo4eemf OBSV.OBSV.hgo4eemf
    #endif

    /* CJH EEMF AO Design */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_CJH_EEMF

        struct CJH_EEMF_AO_Design{
            REAL xPsi[2];
            REAL xChi[2];
            REAL xOmg;

            // output
            REAL xEemf[2];
            REAL theta_d;

            REAL xEta[2];     // \eta = G(p)\phi\hat\omega
            REAL xVarsigma[2];

            REAL xUpsilon[2]; // \Upsilon = [G(p) \phi]
            REAL xZeta[2];

            REAL output_error[2]; // \varepsilon
            REAL effective_output_error[2]; // \varepsilon_\mathrm{eff}

            REAL k1;
            REAL k2;
            REAL gamma_omg;
        } cjheemf;
        // #define OBSV.cjheemf OBSV.OBSV.cjheemf
    #endif

    /* Harnefors 2006 */
    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_Harnefors_2006

        struct Harnefors2006{
            REAL theta_d;
            REAL omg_elec;

            REAL deriv_id;
            REAL deriv_iq;

            // SVF for d/q current derivative
            REAL svf_p0;
            REAL xSVF_curr[2];
            REAL xSVF_prev[2];
            REAL is_dq[2];
            REAL is_dq_curr[2];
            REAL is_dq_prev[2];
            REAL pis_dq[2];
        } harnefors;
    // #define harnefors OBSV.harnefors
    #endif

    #if PC_SIMULATION || SELECT_ALGORITHM == ALG_SynIFO

        struct SynIFO{
            REAL theta_d;
            REAL omg_elec;

            REAL hat_Tem;
            REAL hat_TL;

            REAL e_psi_q2;
            REAL gamma_inv;
            REAL lamda_inv;
        } SynIFO;
    // #define harnefors OBSV.harnefors
    #endif
    };
    extern struct ObserverForExperiment OBSV;
    #if ALG_AKT_SPEED_EST_AND_RS_ID
        struct Akatsu_Variables{
            REAL emf_lpf[2];
            REAL is_hpf[2];
            REAL is_hpf_temp[2];
            REAL ireq_lpf[2];
            REAL ireq_cal[4];
            REAL psi_cmd[2];
            REAL psi_cmd_lpf[2];
            REAL psi_cal[4];
                REAL psi_stator[2];
                REAL emf_stator[2];
                REAL field_speed_est;
                REAL field_speed_est_lpf;
                REAL omg_est;
                REAL omg_est_lpf;
            REAL the_u;
            REAL the_y;
            REAL the_u_lpf;
            REAL the_y_hpf;
            REAL the_y_hpf_temp;
            REAL the_error;
            // REAL delta_rreq;
            // REAL the_gain_P;
            REAL rs_cal;
                REAL voltage_drop[2];
                REAL voltage_drop_mod;
                REAL current_mod;
                long int count_rs;
            REAL indicator_rs;
            REAL the_error_sumup;
            REAL squared_fluxMod;
            REAL squared_fluxMod_lpf;
            REAL pseudo_iMreq_lpf;
            REAL the_u_sumup;
            REAL the_u_offset;
            // REAL zero_freq_operation_on;

            REAL sta_state[2];
            REAL lambda1;
            REAL lambda2;
            REAL the_y_lpf;
            REAL xTem;
            REAL gamma_res_transient;
            REAL GAIN_RS;
            REAL omg_ctrl_err;
            REAL omg_fb;
            REAL gamma_res_transient_shape;
        };
        extern struct Akatsu_Variables akt;
    #endif
    #if ALG_Awaya_InertiaId
        struct Awaya_Variables{
        REAL awaya_lambda;
        REAL q0;
        REAL q1_dot;
        REAL q1;
        REAL tau_est;
        REAL tau_est_lpf;
        REAL sum_A;
        REAL sum_B;
        REAL est_Js_variation;
        REAL est_Js;
    };
    extern struct Awaya_Variables awy;
    #endif
    #if ALG_PLL_norm
    struct PhaseLockLoop_Norm{
        REAL emf_ampl;
        REAL theta_elec;
        REAL epsilon_e;
        REAL omega_elec;
        REAL emf_norm[2];
        REAL omega_integral;
        REAL x[2];
        REAL kp;
        REAL ki;
        REAL emf_recon[2];
        REAL k_p_theta;
    };
    extern struct PhaseLockLoop_Norm PLLN;
    #endif
    #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)

        void init_QiaoXia2013();
        void init_ChiXu2009();
        void init_parksul2014();
        void init_nsoaf();
        void init_hgo4eemf();
        void init_cjheemf();
        void init_harnefors();
        void init_Bernard2017();
        void init_PLL_norm();
        // controller declaration
        void controller_IFOC();

        /* Call all pmsm observers at one place here */
        void pmsm_observers();
        void init_pmsm_observers();
        // Variable parameters
        void variabel_parameters_sensorless();
        /* Macros for using SVF in Harnefors 2006 */
        // void init_harnefors();
        // void harnefors_scvm();
        #define SVF_POLE_0_VALUE (2000*2*M_PI) /* 定子电阻在高速不准确，就把SVF极点加大！加到3000反而比20000要差。*/
        #define SVF_POLE_0 OBSV.harnefors.svf_p0 
        #define SVF_C(X)   OBSV.harnefors.xSVF_curr[X]
        #define SVF_P(X)   OBSV.harnefors.xSVF_prev[X]
        #define IDQ(X)     OBSV.harnefors.is_dq[X]
        #define IDQ_C(X)   OBSV.harnefors.is_dq_curr[X]
        #define IDQ_P(X)   OBSV.harnefors.is_dq_prev[X]
        #define PIDQ(X)    OBSV.harnefors.pis_dq[X]
    #endif

    #endif

// inverter nonlinearity
void Online_PAA_Based_Compensation(void);
#define GAIN_THETA_TRAPEZOIDAL (40) // 20
void inverterNonlinearity_Initialization();
void InverterNonlinearity_ExperimentalLUT(REAL ual, REAL ube, REAL ial, REAL ibe);
void InverterNonlinearity_ExperimentalSigmoid(REAL ual, REAL ube, REAL ial, REAL ibe);
REAL u_comp_per_phase(REAL Vsat, REAL thetaA, REAL theta_trapezoidal, REAL oneOver_theta_trapezoidal);
REAL lpf1(REAL x, REAL y_tminus1);
REAL shift2pi(REAL thetaA);
void yzz_inverter_Compensation_Online_PAA();
void get_distorted_voltage_via_LUT_indexed(REAL ial, REAL ibe, REAL *ualbe_dist);
void get_distorted_voltage_via_LUT(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist, REAL *lut_voltage, REAL *lut_current, int length_of_lut);
void get_distorted_voltage_via_CurveFitting(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist);
#endif

#endif
