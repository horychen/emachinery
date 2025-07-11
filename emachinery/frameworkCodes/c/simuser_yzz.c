#include "ACMSim.h"

#if WHO_IS_USER == USER_YZZ
    #if PC_SIMULATION == FALSE
        #include "All_Definition.h"
        extern st_axis *Axis;
    #endif
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
void _user_pmsm_observer(void){

    /// 3. 调用观测器：估计的电气转子位置和电气转子转速反馈
    // observer_marino2005();
    if ((*debug).SENSORLESS_CONTROL == TRUE){ // （无感）
        (*CTRL).i->varOmega     = CTRL->motor->npp_inv * PMSM_ELECTRICAL_SPEED_FEEDBACK;    // OBSV.harnefors.omg_elec;
        (*CTRL).i->theta_d_elec = CTRL->motor->npp_inv * PMSM_ELECTRICAL_POSITION_FEEDBACK; // OBSV.harnefors.theta_d;
    }
}

void rk4_init(){
    OBSV.rk4.us[0] = 0.0;
    OBSV.rk4.us[1] = 0.0;
    OBSV.rk4.is[0] = 0.0;
    OBSV.rk4.is[1] = 0.0;
    OBSV.rk4.us_curr[0] = 0.0;
    OBSV.rk4.us_curr[1] = 0.0;
    OBSV.rk4.is_curr[0] = 0.0;
    OBSV.rk4.is_curr[1] = 0.0;
    OBSV.rk4.us_prev[0] = 0.0;
    OBSV.rk4.us_prev[1] = 0.0;
    OBSV.rk4.is_prev[0] = 0.0;
    OBSV.rk4.is_prev[1] = 0.0;
    OBSV.theta_d = 0.0;
    OBSV.varOmega = 0.0;
    OBSV.count_for_encoder = 0;
    OBSV.sum_for_encoder = 0;
    int ind;
    for (ind = 0; ind < 10; ind++)
    {
        OBSV.aver_for_encoder[ind] = 0;
    }
}
#endif


/* Structs for Algorithm */
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
    struct ObserverForExperiment OBSV;
    struct SharedFluxEstimatorForExperiment FE;
    #if ALG_AKT_SPEED_EST_AND_RS_ID
    struct Akatsu_Variables akt;
    #endif
    #if ALG_Awaya_InertiaId
    struct Awaya_Variables awy;
    #endif
    #if ALG_PLL_norm
    struct PhaseLockLoop_Norm PLLN;
    #endif
    // struct Marino2005 marino={0};
    // struct Variables_SimulatedVM                         simvm      ={0};
    // struct Variables_Ohtani1992                          ohtani     ={0};
    // struct Variables_HuWu1998                            huwu       ={0};
    // struct Variables_HoltzQuan2002                       holtz02    ={0};
    // struct Variables_Holtz2003                           htz        ={0};
    // struct Variables_Harnefors2003_SCVM                  harnefors  ={0};
    // struct Variables_LascuAndreescus2006                 lascu      ={0};
    // struct Variables_Stojic2015                          stojic     ={0};
    // struct Variables_fluxModulusEstimator                fme        ={0};
    // struct Variables_ExactCompensationMethod             exact      ={0};
    // struct Variables_ProposedxRhoFramePICorrectionMethod picorr     ={0};
    // struct Variables_ClosedLoopFluxEstimator             clest      ={0};
    // struct RK4_DATA rk4;
    // struct Harnefors2006 harnefors={0};
    // struct CJH_EEMF_AO_Design OBSV.cjheemf={0};
    // struct FARZA09_EEMF_HGO OBSV.hgo4eemf={0};
    // struct Chen20_NSO_AF OBSV.nsoaf={0};
#endif

    /* from share flux estimator */ 
    #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
    /*****************************/
    /* Shared Solver and typedef */
    #define CJH_STYLE_RK4_OBSERVER_RAW_CODE                                  \
        US(0) = US_P(0);                                                     \
        US(1) = US_P(1);                                                     \
        IS(0) = IS_P(0);                                                     \
        IS(1) = IS_P(1);                                                     \
        (*fp)(t, x, fx);                                                     \
        for(i=0;i<NS;++i){                                                   \
            k1[i] = fx[i] * hs;                                              \
            xk[i] = x[i] + k1[i]*0.5;                                        \
        }                                                                    \
                                                                            \
        IS(0) = 0.5*(IS_P(0)+IS_C(0));                                       \
        IS(1) = 0.5*(IS_P(1)+IS_C(1));                                       \
        US(0) = 0.5*(US_P(0)+US_C(0));                                       \
        US(1) = 0.5*(US_P(1)+US_C(1));                                       \
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
        IS(0) = IS_C(0);                                                     \
        IS(1) = IS_C(1);                                                     \
        US(0) = US_C(0);                                                     \
        US(1) = US_C(1);                                                     \
        (*fp)(t, xk, fx);                                                    \
        for(i=0;i<NS;++i){                                                   \
            k4[i] = fx[i] * hs;                                              \
            x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])*one_over_six;  \
        }
    void general_1states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 1
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE
        #undef NS
    }
    void general_2states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 2
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE
        #undef NS
    }
    void general_3states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 3
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE
        #undef NS
    }
    // void general_4states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
    //     #define NS 4
    //     REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    //     REAL fx[NS];
    //     int i;
    //     CJH_STYLE_RK4_OBSERVER_RAW_CODE
    //     #undef NS
    // }
    void general_5states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 5
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE
        #undef NS
    }
    void general_6states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 6
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE
        #undef NS
    }
    void general_8states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 8
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE
        #undef NS
    }
    void general_10states_rk4_solver(pointer_flux_estimator_dynamics fp, REAL t, REAL *x, REAL hs){
        #define NS 10
        REAL k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        REAL fx[NS];
        int i;
        CJH_STYLE_RK4_OBSERVER_RAW_CODE
        #undef NS
    }
    #if PC_SIMULATION == TRUE
        #define OFFSET_VOLTAGE_ALPHA 0//(1*-0.2) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
        #define OFFSET_VOLTAGE_BETA  0//(1*+0.2) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
    #else
        #define OFFSET_VOLTAGE_ALPHA 0
        #define OFFSET_VOLTAGE_BETA  0
    #endif

    /***************************/
    /* Various Flux Estimators */

    #if AFE_11_OHTANI_1992 
    #endif
    #if AFE_12_HOLTZ_QUAN_2002 
    #endif
    #if AFE_13_LASCU_ANDREESCUS_2006 
    #endif
    #if AFE_14_MY_PROPOSED 
    #endif
    #if AFE_21_HU_WU_1998
        void init_FE_huwu(){
            FE.huwu.tau_1_inv = AFE_21_HUWU_TAU_1_INVERSE;
            FE.huwu.KP = AFE_21_HUWU_KP;
            FE.huwu.KI = AFE_21_HUWU_KI;

            FE.huwu.x[0] = MOTOR.KE;
            FE.huwu.x[1] = 0.0;
            FE.huwu.x[2] = MOTOR.KE;

            FE.huwu.active_flux_ampl = MOTOR.KE;
            FE.huwu.cosT = 1.0;
            FE.huwu.sinT = 0.0;

            FE.huwu.limiter_KE = 1.0*MOTOR.KE; /* FE.huwu1998: In order to eliminate dc component at the output, the limiting level should be set at a value equal to the actual flux amplitude. */
        }
        void rhf_HUWU_1998_Dynamics(REAL t, REAL *x, REAL *fx){
            // x[0], x[1]: stator flux in ab frame
            // x[2], x[3]: compensation flux (integral part)

            #define xPsi1(X)  x[0+X]

            REAL emf[2];
            emf[0] = US(0) - MOTOR.R*IS(0);
            emf[1] = US(1) - MOTOR.R*IS(1);

            /* 求定子磁链幅值的倒数 */
            REAL temp = 0.0; 
            FE.huwu.stator_flux_ampl = sqrt(xPsi1(0)*xPsi1(0) + xPsi1(1)*xPsi1(1));
            if(FE.huwu.stator_flux_ampl!=0){
                temp = 1.0 / FE.huwu.stator_flux_ampl;
            }

        #define HU_WU_ALGORITHM_2
        #ifdef HU_WU_ALGORITHM_2
            /* 计算有功磁链分量 */
            FE.huwu.psi_2[0] = xPsi1(0) - MOTOR.Lq * IS(0);
            FE.huwu.psi_2[1] = xPsi1(1) - MOTOR.Lq * IS(1);
            /* 求有功磁链幅值的倒数 */
            FE.huwu.active_flux_ampl = sqrt(FE.huwu.psi_2[0]*FE.huwu.psi_2[0] + FE.huwu.psi_2[1]*FE.huwu.psi_2[1]);
            REAL active_flux_ampl_inv=0.0;
            if(FE.huwu.active_flux_ampl!=0){
                active_flux_ampl_inv = 1.0/FE.huwu.active_flux_ampl;
            }
            /* The amplitude limiter for active flux */
            if(FE.huwu.active_flux_ampl > FE.huwu.limiter_KE){
                FE.huwu.active_flux_ampl_limited = FE.huwu.limiter_KE;
            }else{
                FE.huwu.active_flux_ampl_limited = FE.huwu.active_flux_ampl;
            }

            if(FALSE){
                /* 根据限幅后的有功磁链去求限幅后的定子磁链 */
                REAL stator_flux_limited[2];
                stator_flux_limited[0] = FE.huwu.psi_2[0]*active_flux_ampl_inv * FE.huwu.active_flux_ampl_limited + MOTOR.Lq*IS(0);
                stator_flux_limited[1] = FE.huwu.psi_2[1]*active_flux_ampl_inv * FE.huwu.active_flux_ampl_limited + MOTOR.Lq*IS(1);            
                /* 求限幅后的定子磁链幅值 */
                FE.huwu.stator_flux_ampl_limited = sqrt(stator_flux_limited[0]*stator_flux_limited[0] + stator_flux_limited[1]*stator_flux_limited[1]);

                /* Correct by stator flux */
                fx[0] = emf[0] + FE.huwu.tau_1_inv * (( FE.huwu.stator_flux_ampl_limited ) * xPsi1(0) * temp - xPsi1(0) );
                fx[1] = emf[1] + FE.huwu.tau_1_inv * (( FE.huwu.stator_flux_ampl_limited ) * xPsi1(1) * temp - xPsi1(1) );
                fx[2] = 0.0;
            }else{
                /* Simplified (Correct by rotor flux) */
                fx[0] = emf[0] + FE.huwu.tau_1_inv * (( FE.huwu.active_flux_ampl_limited ) * FE.huwu.psi_2[0] * active_flux_ampl_inv - FE.huwu.psi_2[0] );
                fx[1] = emf[1] + FE.huwu.tau_1_inv * (( FE.huwu.active_flux_ampl_limited ) * FE.huwu.psi_2[1] * active_flux_ampl_inv - FE.huwu.psi_2[1] );
                fx[2] = 0.0;
            }

        #else
            #define xPsi_Comp x[2]
            /* Algorithm 3 */
            FE.huwu.inner_product_normalized = (emf[0] * xPsi1(0) + emf[1] * xPsi1(1)) * temp;

            /* Compensation term is a flux amplitude going through an inverse Park transformation */
            fx[0] = emf[0] + FE.huwu.tau_1_inv * ( (/*P*/+ FE.huwu.KP * FE.huwu.inner_product_normalized \
                                                /*I*/+ xPsi_Comp \
                                                ) * xPsi1(0) * temp - xPsi1(0) );
            fx[1] = emf[1] + FE.huwu.tau_1_inv * ( (/*P*/+ FE.huwu.KP * FE.huwu.inner_product_normalized \
                                                /*I*/+ xPsi_Comp \
                                                ) * xPsi1(1) * temp - xPsi1(1) );

            /* State: disturbance/offset estimated by an integrator */
            fx[2] = FE.huwu.KI * FE.huwu.inner_product_normalized;

            #undef xPsi_Comp
        #endif
        #undef xPsi1
        }
        void MainFE_HUWU_1998(){
            // stator flux and integral states update
            general_3states_rk4_solver(&rhf_HUWU_1998_Dynamics, (*CTRL).timebase, FE.huwu.x, CL_TS);
            // Unpack x
            FE.huwu.psi_1[0] = FE.huwu.x[0];
            FE.huwu.psi_1[1] = FE.huwu.x[1];
            FE.huwu.psi_comp = FE.huwu.x[2];

            // active flux and position
            FE.huwu.psi_2[0] = FE.huwu.x[0] - (*CTRL).motor->Lq * IS(0);
            FE.huwu.psi_2[1] = FE.huwu.x[1] - (*CTRL).motor->Lq * IS(1);
            FE.huwu.active_flux_ampl = sqrt(FE.huwu.psi_2[0]*FE.huwu.psi_2[0] + FE.huwu.psi_2[1]*FE.huwu.psi_2[1]);
            REAL active_flux_ampl_inv=0.0;
            if(FE.huwu.active_flux_ampl!=0){
                active_flux_ampl_inv = 1.0/FE.huwu.active_flux_ampl;
                FE.huwu.cosT = FE.huwu.psi_2[0] * active_flux_ampl_inv;
                FE.huwu.sinT = FE.huwu.psi_2[1] * active_flux_ampl_inv;
            }else{
                FE.huwu.cosT = 1.0;
                FE.huwu.sinT = 0.0;
            }

            FE.huwu.theta_d = atan2(FE.huwu.psi_2[1], FE.huwu.psi_2[0]);
        }
    #endif
    #if AFE_22_STOJIC_2015 
    #endif
    #if AFE_23_SCVM_HARNEFORS_2003 
    #endif
    #if AFE_24_OE || AFE_25_VM_CM_FUSION // See ParkSul2014Follower2020
        void afe_one_parameter_tuning(REAL omega_est){
            FE.AFEOE.ActiveFlux_KP = 2* omega_est;
            FE.AFEOE.ActiveFlux_KI = omega_est*omega_est;

            #if PC_SIMULATION
            printf("Init AFEOE:\n");
            printf("\tomega_est=%g\n", omega_est);
            printf("\tAFEOE.ActiveFlux_KP=%g\n", FE.AFEOE.ActiveFlux_KP);
            printf("\tAFEOE.ActiveFlux_KI=%g\n", FE.AFEOE.ActiveFlux_KI);
            #endif
        }
        void init_afe(){
            FE.AFEOE.active_flux_ampl     = MOTOR.KE;
            FE.AFEOE.active_flux_ampl_lpf = FE.AFEOE.active_flux_ampl;
            FE.AFEOE.cosT = 1.0;
            FE.AFEOE.sinT = 0.0;

            FE.AFEOE.x[0] = CORRECTION_4_SHARED_FLUX_EST; // TODO：这里假设了初始位置是alpha轴对着d轴的
            FE.AFEOE.x[1] = 0.0;
            FE.AFEOE.ActiveFlux_KP = AFEOE_KP;
            FE.AFEOE.ActiveFlux_KI = AFEOE_KI;
            FE.AFEOE.set_omega_est = AFEOE_OMEGA_ESTIMATOR;
            FE.AFEOE.omega_est = FE.AFEOE.set_omega_est;
            #if AFE_25_VM_CM_FUSION
                // afe_one_parameter_tuning(FE.AFEOE.omega_est);
            #endif

            FE.AFEOE.limiter_KE = 1.15 * MOTOR.KE;
            FE.AFEOE.k_af = 0.0 * 100 * 2 * M_PI;
        }
        void rhf_ActiveFluxEstimator_Dynamics(REAL t, REAL *x, REAL *fx){
            // x[0], x[1]: stator flux in ab frame
            // x[2], x[3]: integral action compensating offset voltage

            /* Calculate psi_2 from psi_1, Lq and i_ab */
            FE.AFEOE.psi_2[0] = x[0] - MOTOR.Lq * IS(0);
            FE.AFEOE.psi_2[1] = x[1] - MOTOR.Lq * IS(1);

            /* Output: Flux amplitude */
            FE.AFEOE.active_flux_ampl = sqrt(FE.AFEOE.psi_2[0]*FE.AFEOE.psi_2[0] + FE.AFEOE.psi_2[1]*FE.AFEOE.psi_2[1]);

            /* State: Cosine and Sine of the flux angle */
            REAL active_flux_ampl_inv=0.0;
            if(FE.AFEOE.active_flux_ampl!=0){
                active_flux_ampl_inv = 1.0/FE.AFEOE.active_flux_ampl;
            } 
            REAL cosT = FE.AFEOE.psi_2[0] * active_flux_ampl_inv;
            REAL sinT = FE.AFEOE.psi_2[1] * active_flux_ampl_inv;

            // #define DEBUG_USING_ACTUAL_CM
            #ifdef DEBUG_USING_ACTUAL_CM
                cosT = (*CTRL).s->cosT;
                sinT = (*CTRL).s->sinT;
            #endif

            /* Input: Current at dq frame and KActive */
            REAL iDQ_at_current_step[2];
            iDQ_at_current_step[0] = AB2M(IS(0), IS(1), cosT, sinT);
            iDQ_at_current_step[1] = AB2T(IS(0), IS(1), cosT, sinT);
            REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_at_current_step[0];

            #if AFE_24_OE
                /* Output: Current error & estimate */
                REAL current_estimate[2];
                current_estimate[0] = MOTOR.Lq_inv * (x[0] - KActive*cosT);
                current_estimate[1] = MOTOR.Lq_inv * (x[1] - KActive*sinT);
                FE.AFEOE.output_error[0] = IS(0) - current_estimate[0];
                FE.AFEOE.output_error[1] = IS(1) - current_estimate[1];
            #endif

            #if AFE_25_VM_CM_FUSION // note this will potentially overwrite results of AFE_24_OE
                /* Output: Flux mismatch to KActive (CM) */
                FE.AFEOE.output_error[0] = KActive*cosT - FE.AFEOE.psi_2[0];
                FE.AFEOE.output_error[1] = KActive*sinT - FE.AFEOE.psi_2[1];
            #endif

            #if AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED
                /* The amplitude limiter */
                if(FE.AFEOE.active_flux_ampl > FE.AFEOE.limiter_KE){
                    FE.AFEOE.psi_2_limited[0] = FE.AFEOE.limiter_KE * FE.AFEOE.psi_2[0]*active_flux_ampl_inv;
                    FE.AFEOE.psi_2_limited[1] = FE.AFEOE.limiter_KE * FE.AFEOE.psi_2[1]*active_flux_ampl_inv;
                }else{
                    FE.AFEOE.psi_2_limited[0] = FE.AFEOE.psi_2[0];
                    FE.AFEOE.psi_2_limited[1] = FE.AFEOE.psi_2[1];
                }
            #else
                FE.AFEOE.psi_2_limited[0] = FE.AFEOE.psi_2[0];
                FE.AFEOE.psi_2_limited[1] = FE.AFEOE.psi_2[1];
            #endif

            /* State: stator emf as the derivative of staotr flux, psi_1 */
            REAL emf[2];
            emf[0] = US(0) - MOTOR.R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA;
            emf[1] = US(1) - MOTOR.R*IS(1) + 1*OFFSET_VOLTAGE_BETA ;
            fx[0] = emf[0] \
                /*k_af*/- FE.AFEOE.k_af*(FE.AFEOE.psi_2[0] - FE.AFEOE.psi_2_limited[0]) \
                /*P*/+ FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[0] \
                /*I*/+ x[2];
            fx[1] = emf[1] \
                /*k_af*/- FE.AFEOE.k_af*(FE.AFEOE.psi_2[1] - FE.AFEOE.psi_2_limited[1]) \
                /*P*/+ FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[1] \
                /*I*/+ x[3];

            /* State: disturbance/offset estimated by an integrator */
            fx[2] = FE.AFEOE.ActiveFlux_KI * FE.AFEOE.output_error[0];
            fx[3] = FE.AFEOE.ActiveFlux_KI * FE.AFEOE.output_error[1];
        }
        REAL k_af_speed_Hz = 1.5;
        void Main_the_active_flux_estimator(){

            /* OPT for AFE */
            if(FE.AFEOE.omega_est != FE.AFEOE.set_omega_est){
                FE.AFEOE.omega_est = FE.AFEOE.set_omega_est;
                afe_one_parameter_tuning(FE.AFEOE.omega_est);
            }

            /* Time-varying gains */
            if(fabsf((*CTRL).i->cmd_varTheta)*MOTOR.npp_inv*ONE_OVER_2PI<k_af_speed_Hz){ // [Hz]
                FE.AFEOE.k_af = 2*M_PI*100;
                FE.AFEOE.limiter_Flag = TRUE;
            }else{
                FE.AFEOE.k_af = 0.0;
                FE.AFEOE.limiter_Flag = FALSE;
            }

            #if AFE_25_FISION__FLUX_LIMITER_AT_LOW_SPEED == FALSE
                /* NO Need */
                /* NO Need */
                /* NO Need */
                FE.AFEOE.k_af = 0.0;
            #endif

            /* Proposed closed loop estimator AB frame + ODE4 */
            // stator flux and integral states update
            general_4states_rk4_solver(&rhf_ActiveFluxEstimator_Dynamics, (*CTRL).timebase, FE.AFEOE.x, CL_TS);
            // Unpack x
            FE.AFEOE.psi_1[0]    = FE.AFEOE.x[0];
            FE.AFEOE.psi_1[1]    = FE.AFEOE.x[1];
            FE.AFEOE.u_offset[0] = FE.AFEOE.x[2];
            FE.AFEOE.u_offset[1] = FE.AFEOE.x[3];

            // active flux
            FE.AFEOE.psi_2[0] = FE.AFEOE.x[0] - (*CTRL).motor->Lq * IS(0);
            FE.AFEOE.psi_2[1] = FE.AFEOE.x[1] - (*CTRL).motor->Lq * IS(1);
            FE.AFEOE.active_flux_ampl = sqrt(FE.AFEOE.psi_2[0]*FE.AFEOE.psi_2[0] + FE.AFEOE.psi_2[1]*FE.AFEOE.psi_2[1]);
            REAL active_flux_ampl_inv=0.0;
            if(FE.AFEOE.active_flux_ampl!=0){
                active_flux_ampl_inv = 1.0/FE.AFEOE.active_flux_ampl;
                FE.AFEOE.cosT = FE.AFEOE.psi_2[0] * active_flux_ampl_inv;
                FE.AFEOE.sinT = FE.AFEOE.psi_2[1] * active_flux_ampl_inv;
            }else{
                FE.AFEOE.cosT = 1.0;
                FE.AFEOE.sinT = 0.0;
            }

            FE.AFEOE.theta_d = atan2(FE.AFEOE.psi_2[1], FE.AFEOE.psi_2[0]);
            FE.AFEOE.theta_e = angle_diff(FE.AFEOE.theta_d, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
            // Convert output error to dq frame
            // REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];
            /* TODO: 思考这个dq角度怎么选最好，要不要换成电压向量的角度而不是转子角度？ */
            FE.AFEOE.active_flux_ampl_lpf = _lpf(FE.AFEOE.active_flux_ampl, FE.AFEOE.active_flux_ampl_lpf, 5);
            FE.AFEOE.output_error_dq[0] = MOTOR.KActive - FE.AFEOE.active_flux_ampl_lpf;
            FE.AFEOE.output_error[0] = MOTOR.KActive*FE.AFEOE.cosT - FE.AFEOE.psi_2[0];
            FE.AFEOE.output_error[1] = MOTOR.KActive*FE.AFEOE.sinT - FE.AFEOE.psi_2[1];
            FE.AFEOE.output_error_dq[1] = sqrt(FE.AFEOE.output_error[0]*FE.AFEOE.output_error[0] + FE.AFEOE.output_error[1]*FE.AFEOE.output_error[1]);
            // if((*CTRL).i->cmd_speed_rpm>0){
            //     FE.AFEOE.output_error_dq[0] = AB2M(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
            //     FE.AFEOE.output_error_dq[1] = AB2T(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
            // }else{
            //     FE.AFEOE.output_error_dq[0] = AB2M(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
            //     FE.AFEOE.output_error_dq[1] = AB2T(FE.AFEOE.output_error[0], FE.AFEOE.output_error[1], FE.AFEOE.cosT, FE.AFEOE.sinT);
            // }

            // Research
            REAL emf[2];
            emf[0] = US(0) - MOTOR.R*IS(0) + FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[0];
            emf[1] = US(1) - MOTOR.R*IS(1) + FE.AFEOE.ActiveFlux_KP * FE.AFEOE.output_error[1];
            FE.AFEOE.dot_product = emf[0]*FE.AFEOE.psi_1[0] + emf[1]*FE.AFEOE.psi_1[1];
            FE.AFEOE.cross_product = emf[0]*-FE.AFEOE.psi_1[1] + emf[1]*FE.AFEOE.psi_1[0];
        }
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
        REAL imife_realtime_gain_off = 1.0;
        void init_FE_htz(){
            int ind;
            for(ind=0;ind<2;++ind){
                FE.htz.emf_stator[ind] = 0;

                FE.htz.psi_1[0] = d_sim.init.KE;
                FE.htz.psi_1[1] = 0;
                FE.htz.psi_2[ind] = 0;
                FE.htz.psi_2_prev[ind] = 0;

                FE.htz.psi_1_nonSat[ind] = 0;
                FE.htz.psi_2_nonSat[ind] = 0;

                FE.htz.psi_1_min[ind] = 0;
                FE.htz.psi_1_max[ind] = 0;
                FE.htz.psi_2_min[ind] = 0;
                FE.htz.psi_2_max[ind] = 0;

                FE.htz.rs_est = d_sim.init.R;
                FE.htz.rreq_est = d_sim.init.Rreq;

                FE.htz.Delta_t = 1;
                FE.htz.u_offset[ind] = 0;
                // FE.htz.u_offset_intermediate[ind] = 0;
                FE.htz.u_off_original_lpf_input[ind]=0.0; // holtz03 original (but I uses integrator instead of LPF)
                FE.htz.u_off_saturation_time_correction[ind]=0.0; // exact offset calculation for compensation
                FE.htz.u_off_calculated_increment[ind]=0.0;    // saturation time based correction
                FE.htz.gain_off = HOLTZ_2002_GAIN_OFFSET; // 5; -> slow but stable // 50.1 // 20 -> too large then speed will oscillate during reversal near zero

                FE.htz.flag_pos2negLevelA[ind] = 0;
                FE.htz.flag_pos2negLevelB[ind] = 0;
                FE.htz.time_pos2neg[ind] = 0;
                FE.htz.time_pos2neg_prev[ind] = 0;

                FE.htz.flag_neg2posLevelA[ind] = 0;
                FE.htz.flag_neg2posLevelB[ind] = 0;
                FE.htz.time_neg2pos[ind] = 0;
                FE.htz.time_neg2pos_prev[ind] = 0;

                FE.htz.psi_aster_max = IM_FLUX_COMMAND_DC_PART + IM_FLUX_COMMAND_SINE_PART;

                FE.htz.maximum_of_sat_min_time[ind] = 0.0;
                FE.htz.maximum_of_sat_max_time[ind] = 0.0;
                FE.htz.sat_min_time[ind] = 0.0;
                FE.htz.sat_max_time[ind] = 0.0;
                FE.htz.sat_min_time_reg[ind] = 0.0;
                FE.htz.sat_max_time_reg[ind] = 0.0;
                FE.htz.extra_limit = 0.0;
                FE.htz.flag_limit_too_low = FALSE;
            }
        }
        void rhf_Holtz2003_Dynamics(REAL t, REAL *x, REAL *fx){
            #if RS_IDENTIFICATION
                FE.htz.emf_stator[0] = US(0) - akt.rs_cal * IS(0) - FE.htz.u_offset[0] + OFFSET_VOLTAGE_ALPHA;
                FE.htz.emf_stator[1] = US(1) - akt.rs_cal * IS(1) - FE.htz.u_offset[1] + OFFSET_VOLTAGE_BETA ;
            #else
                FE.htz.emf_stator[0] = US(0) - (*CTRL).motor->R * IS(0) - FE.htz.u_offset[0] + OFFSET_VOLTAGE_ALPHA;
                FE.htz.emf_stator[1] = US(1) - (*CTRL).motor->R * IS(1) - FE.htz.u_offset[1] + OFFSET_VOLTAGE_BETA ;
            #endif
            fx[0] = (FE.htz.emf_stator[0]);
            fx[1] = (FE.htz.emf_stator[1]);
        }
        REAL marino_saturation_gain_scale_factor1 = 1.0;
        REAL marino_saturation_gain_scale_factor2 = 1.0;
        REAL marino_sat_d_axis_flux_control = 1.0;
        REAL marino_sat_q_axis_flux_control = 1.0;
        int bool_positive_extra_limit = TRUE;
        void VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit(){
            #define PSI_MU_ASTER_MAX FE.htz.psi_aster_max // Holtz缺点就是实际磁链超过给定磁链时，失效！自动检测上下界同时饱和的情况，然后增大限幅？
            #define BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT FALSE // The limit is extended when both upper and lower limit are reached in a cycle.

            #define BOOL_USE_METHOD_LPF_INPUT      FALSE
            #define BOOL_USE_METHOD_INTEGRAL_INPUT TRUE

            // Euler's method is shit at higher speeds
            #if RS_IDENTIFICATION
                FE.htz.emf_stator[0] = US_C(0) - akt.rs_cal * IS_C(0) - FE.htz.u_offset[0];
                FE.htz.emf_stator[1] = US_C(1) - akt.rs_cal * IS_C(1) - FE.htz.u_offset[1];
            #else
                FE.htz.emf_stator[0] = US_C(0) - (*CTRL).motor->R*IS_C(0) - FE.htz.u_offset[0];
                FE.htz.emf_stator[1] = US_C(1) - (*CTRL).motor->R*IS_C(1) - FE.htz.u_offset[1];
            #endif
            // FE.htz.psi_1[0] += CL_TS*(FE.htz.emf_stator[0]);
            // FE.htz.psi_1[1] += CL_TS*(FE.htz.emf_stator[1]);

            // rk4 
            general_2states_rk4_solver(&rhf_Holtz2003_Dynamics, (*CTRL).timebase, FE.htz.psi_1, CL_TS);
            FE.htz.psi_2[0] = FE.htz.psi_1[0] - (*CTRL).motor->Lq*IS_C(0);
            FE.htz.psi_2[1] = FE.htz.psi_1[1] - (*CTRL).motor->Lq*IS_C(1);
            #if LQ_IDENTIFICATION
                FE.htz.psi_2[0] = FE.htz.psi_1[0] - q_inductanceid.Lq *IS_C(0);
                FE.htz.psi_2[1] = FE.htz.psi_1[1] - q_inductanceid.Lq *IS_C(1);
            #endif
            FE.htz.psi_2_ampl = sqrt(FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);
            // 限幅前求角度还是应该限幅后？
            FE.htz.theta_d = atan2(FE.htz.psi_2[1], FE.htz.psi_2[0]);
            FE.htz.cosT = cos(FE.htz.theta_d);
            FE.htz.sinT = sin(FE.htz.theta_d);

            FE.htz.psi_1_nonSat[0] += CL_TS*(FE.htz.emf_stator[0]);
            FE.htz.psi_1_nonSat[1] += CL_TS*(FE.htz.emf_stator[1]);
            FE.htz.psi_2_nonSat[0] = FE.htz.psi_1_nonSat[0] - (*CTRL).motor->Lq*IS_C(0);
            FE.htz.psi_2_nonSat[1] = FE.htz.psi_1_nonSat[1] - (*CTRL).motor->Lq*IS_C(1);
            #if LQ_IDENTIFICATION
                FE.htz.psi_2_nonSat[0] = FE.htz.psi_1_nonSat[0] - q_inductanceid.Lq *IS_C(0);
                FE.htz.psi_2_nonSat[1] = FE.htz.psi_1_nonSat[1] - q_inductanceid.Lq *IS_C(1);
            #endif

            // FE.htz.psi_aster_max = (*CTRL).taao_flux_cmd + 0.05;
            // FE.htz.psi_aster_max = (*CTRL).i->cmd_psi + FE.htz.extra_limit;
            FE.htz.psi_aster_max = (*CTRL).motor->KE + FE.htz.extra_limit;
            // FE.htz.psi_aster_max = (*CTRL).taao_flux_cmd;

            // 限幅是针对转子磁链限幅的
            int ind;
            for(ind=0;ind<2;++ind){
                if((*CTRL).i->cmd_varOmega != 0.0){
                    if(FE.htz.psi_2[ind]    > FE.htz.psi_aster_max){ // TODO BUG呀！这里怎么可以是>应该是大于等于啊！
                        FE.htz.psi_2[ind]   = FE.htz.psi_aster_max;
                        FE.htz.sat_max_time[ind] += CL_TS;
                        // marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
                        // marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
                        // marino_sat_d_axis_flux_control = 0.0;
                        // marino_sat_q_axis_flux_control = 0.0;
                    }else if(FE.htz.psi_2[ind] < -FE.htz.psi_aster_max){
                        FE.htz.psi_2[ind]   = -FE.htz.psi_aster_max;
                        FE.htz.sat_min_time[ind] += CL_TS;
                    //     marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
                    //     marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
                    //     marino_sat_d_axis_flux_control = 0.0;
                    //     marino_sat_q_axis_flux_control = 0.0;
                    }else{
                        // 这样可以及时清零饱和时间
                        if(FE.htz.sat_max_time[ind]>0){FE.htz.sat_max_time[ind] -= CL_TS;}
                        if(FE.htz.sat_min_time[ind]>0){FE.htz.sat_min_time[ind] -= CL_TS;}
                        // marino.lambda_inv = LAMBDA_INV_xOmg;
                        // marino.gamma_inv  = GAMMA_INV_xTL;
                        // marino_sat_d_axis_flux_control = 1.0;
                        // marino_sat_q_axis_flux_control = 1.0;
                    }
                }
                // 上限饱和减去下限饱和作为误差，主要为了消除实际磁链幅值大于给定的情况，实际上这种现象在常见工况下出现次数不多。
                FE.htz.u_off_saturation_time_correction[ind] = FE.htz.sat_max_time[ind] - FE.htz.sat_min_time[ind];
                // u_offset波动会导致sat_min_time和sat_max_time的波动，这个时候最有效的办法是减少gain_off。
                // 但是同时，观察饱和时间sat_min_time等的波形，可以发现它里面也会出现一个正弦波包络线。
                if(FE.htz.sat_min_time[ind] > FE.htz.maximum_of_sat_min_time[ind]){FE.htz.maximum_of_sat_min_time[ind] = FE.htz.sat_min_time[ind];}
                if(FE.htz.sat_max_time[ind] > FE.htz.maximum_of_sat_max_time[ind]){FE.htz.maximum_of_sat_max_time[ind] = FE.htz.sat_max_time[ind];}
            }

            // 数数，算磁链周期
            if(FE.htz.psi_2[0]    > 0.0){
                FE.htz.count_positive_in_one_cycle[0] += 1;
                if(FE.htz.count_negative_in_one_cycle[0]!=0){ FE.htz.negative_cycle_in_count[0] = FE.htz.count_negative_in_one_cycle[0]; FE.htz.count_negative_in_one_cycle[0] = 0;}
            }else if(FE.htz.psi_2[0] < -0.0){
                FE.htz.count_negative_in_one_cycle[0] += 1;
                if(FE.htz.count_positive_in_one_cycle[0]!=0){ FE.htz.positive_cycle_in_count[0] = FE.htz.count_positive_in_one_cycle[0]; FE.htz.count_positive_in_one_cycle[0] = 0;}
            }
            if(FE.htz.psi_2[1]    > 0.0){
                FE.htz.count_positive_in_one_cycle[1] += 1;
                if(FE.htz.count_negative_in_one_cycle[1]!=0){ FE.htz.negative_cycle_in_count[1] = FE.htz.count_negative_in_one_cycle[1]; FE.htz.count_negative_in_one_cycle[1] = 0;}
            }else if(FE.htz.psi_2[1] < -0.0){
                FE.htz.count_negative_in_one_cycle[1] += 1;
                if(FE.htz.count_positive_in_one_cycle[1]!=0){ FE.htz.positive_cycle_in_count[1] = FE.htz.count_positive_in_one_cycle[1]; FE.htz.count_positive_in_one_cycle[1] = 0;}
            }

            // 限幅后的转子磁链，再求取限幅后的定子磁链
            FE.htz.psi_1[0] = FE.htz.psi_2[0] + (*CTRL).motor->Lq*IS_C(0);
            FE.htz.psi_1[1] = FE.htz.psi_2[1] + (*CTRL).motor->Lq*IS_C(1);
            #if LQ_IDENTIFICATION
                FE.htz.psi_1[0] = FE.htz.psi_2[0] + q_inductanceid.Lq *IS_C(0);
                FE.htz.psi_1[1] = FE.htz.psi_2[1] + q_inductanceid.Lq *IS_C(1);
            #endif

            // Speed Estimation
            if(TRUE){
                // FE.htz.ireq[0] = (*CTRL).Lmu_inv*FE.htz.psi_2[0] - IS_C(0);
                // FE.htz.ireq[1] = (*CTRL).Lmu_inv*FE.htz.psi_2[1] - IS_C(1);
                REAL temp;
                temp = (FE.htz.psi_1[0]*FE.htz.psi_1[0]+FE.htz.psi_1[1]*FE.htz.psi_1[1]);
                if(temp>0.001){
                    FE.htz.field_speed_est = - (FE.htz.psi_1[0]*-FE.htz.emf_stator[1] + FE.htz.psi_1[1]*FE.htz.emf_stator[0]) / temp;
                }
                temp = (FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);
                if(temp>0.001){
                    FE.htz.slip_est = (*CTRL).motor->Rreq*(IS_C(0)*-FE.htz.psi_2[1]+IS_C(1)*FE.htz.psi_2[0]) / temp;
                }
                FE.htz.omg_est = FE.htz.field_speed_est - FE.htz.slip_est;
            }


            // TODO My proposed saturation time based correction method NOTE VERY COOL
            #define CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS \
                FE.htz.u_off_original_lpf_input[ind]         = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) /  (FE.htz.Delta_t+FE.htz.Delta_t_last); \
                FE.htz.u_off_calculated_increment[ind]       = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) / ((FE.htz.Delta_t+FE.htz.Delta_t_last) - (FE.htz.sat_max_time[ind]+FE.htz.sat_min_time[ind])); \
                FE.htz.u_off_saturation_time_correction[ind] = FE.htz.sat_max_time[ind] - FE.htz.sat_min_time[ind]; \
                FE.htz.u_off_direct_calculated[ind] += (FE.htz.count_negative_cycle+FE.htz.count_positive_cycle>4) * FE.htz.u_off_calculated_increment[ind]; // if(BOOL_USE_METHOD_DIFFERENCE_INPUT) 
                // 引入 count：刚起动时的几个磁链正负半周里，Delta_t_last 存在巨大的计算误差，所以要放弃更新哦。

            for(ind=0;ind<2;++ind){ // Loop for alpha & beta components // destroy integer outside this loop to avoid accidentally usage 

                // if( tmin!=0 && tmax!=0 ){
                //     // The sat func's limit is too small.
                //     limit += TS * min(tmin, tmax);
                // }


                /* 必须先检查是否进入levelA */
                if(FE.htz.flag_pos2negLevelA[ind] == TRUE){ 
                    if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]<0){ // 二次检查，磁链已经是负的了  <- 可以改为施密特触发器
                        if(FE.htz.flag_pos2negLevelB[ind] == FALSE){
                            FE.htz.count_negative_cycle+=1; // FE.htz.count_positive_cycle = 0;
                            // printf("POS2NEG: %g, %d\n", (*CTRL).timebase, ind);
                            // printf("%g, %g\n", FE.htz.psi_2_prev[ind], FE.htz.psi_2[ind]);
                            // getch();
                            // 第一次进入寻找最小值的levelB，说明最大值已经检测到。
                            FE.htz.psi_1_max[ind] = FE.htz.psi_2_max[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                            FE.htz.Delta_t_last = FE.htz.Delta_t;
                            FE.htz.Delta_t = FE.htz.time_pos2neg[ind] - FE.htz.time_pos2neg_prev[ind];
                            FE.htz.time_pos2neg_prev[ind] = FE.htz.time_pos2neg[ind]; // 备份作为下次耗时参考点
                            // 初始化
                            FE.htz.flag_neg2posLevelA[ind] = FALSE;
                            FE.htz.flag_neg2posLevelB[ind] = FALSE;

                            // 注意这里是正半周到负半周切换的时候才执行一次的哦！
                            CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                            // FE.htz.accumulated__u_off_saturation_time_correction[ind] += FE.htz.u_off_saturation_time_correction[ind];
                            FE.htz.sign__u_off_saturation_time_correction[ind] = -1.0;
                            // 饱和时间的正弦包络线的正负半周的频率比磁链频率低多啦！需要再额外加一个低频u_offset校正
                            FE.htz.sat_time_offset[ind] = FE.htz.maximum_of_sat_max_time[ind] - FE.htz.maximum_of_sat_min_time[ind];
                            FE.htz.maximum_of_sat_max_time[ind] = 0.0;
                            FE.htz.maximum_of_sat_min_time[ind] = 0.0;

                            FE.htz.psi_1_min[ind] = 0.0;
                            FE.htz.psi_2_min[ind] = 0.0;
                            if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                                FE.htz.sat_min_time_reg[ind] = FE.htz.sat_min_time[ind];
                                if(FE.htz.sat_max_time_reg[ind]>CL_TS && FE.htz.sat_min_time_reg[ind]>CL_TS){
                                    FE.htz.flag_limit_too_low = TRUE;
                                    FE.htz.extra_limit += 1e-2 * (FE.htz.sat_max_time_reg[ind] + FE.htz.sat_min_time_reg[ind]) / FE.htz.Delta_t; 
                                }else{
                                    FE.htz.flag_limit_too_low = FALSE;
                                    FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
                                    if(bool_positive_extra_limit){
                                        if(FE.htz.extra_limit<0.0){
                                            FE.htz.extra_limit = 0.0;
                                        }
                                    }
                                }
                                FE.htz.sat_max_time_reg[ind] = 0.0;
                            }
                            FE.htz.sat_min_time[ind] = 0.0;
                        }
                        FE.htz.flag_pos2negLevelB[ind] = TRUE;
                        if(FE.htz.flag_pos2negLevelB[ind] == TRUE){ // 寻找磁链最小值
                            if(FE.htz.psi_2[ind] < FE.htz.psi_2_min[ind]){
                                FE.htz.psi_2_min[ind] = FE.htz.psi_2[ind];
                            }
                        }
                    }else{ // 磁链还没有变负，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]>0
                        FE.htz.flag_pos2negLevelA[ind] = FALSE; /* 震荡的话，另一方的检测就有可能被触动？ */
                    }
                }
                if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]<0){ // 发现磁链由正变负的时刻
                    FE.htz.flag_pos2negLevelA[ind] = TRUE;
                    FE.htz.time_pos2neg[ind] = (*CTRL).timebase;
                }


                if(FE.htz.flag_neg2posLevelA[ind] == TRUE){ 
                    if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]>0){ // 二次检查，磁链已经是正的了
                        if(FE.htz.flag_neg2posLevelB[ind] == FALSE){
                            FE.htz.count_positive_cycle+=1; // FE.htz.count_negative_cycle = 0;
                            // 第一次进入寻找最大值的levelB，说明最小值已经检测到。
                            FE.htz.psi_1_min[ind] = FE.htz.psi_2_min[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                            FE.htz.Delta_t_last = FE.htz.Delta_t;
                            FE.htz.Delta_t = FE.htz.time_neg2pos[ind] - FE.htz.time_neg2pos_prev[ind];
                            FE.htz.time_neg2pos_prev[ind] = FE.htz.time_neg2pos[ind]; // 备份作为下次耗时参考点
                            // 初始化
                            FE.htz.flag_pos2negLevelA[ind] = FALSE;
                            FE.htz.flag_pos2negLevelB[ind] = FALSE;

                            CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                            // FE.htz.accumulated__u_off_saturation_time_correction[ind] += FE.htz.u_off_saturation_time_correction[ind];
                            FE.htz.sign__u_off_saturation_time_correction[ind] = 1.0;

                            FE.htz.psi_1_max[ind] = 0.0;
                            FE.htz.psi_2_max[ind] = 0.0;
                            if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                                FE.htz.sat_max_time_reg[ind] = FE.htz.sat_max_time[ind];
                                if(FE.htz.sat_min_time_reg[ind]>CL_TS && FE.htz.sat_max_time_reg[ind]>CL_TS){
                                    FE.htz.flag_limit_too_low = TRUE;
                                    FE.htz.extra_limit += 1e-2 * (FE.htz.sat_min_time_reg[ind] + FE.htz.sat_max_time_reg[ind]) / FE.htz.Delta_t; 
                                }else{
                                    FE.htz.flag_limit_too_low = FALSE;
                                    FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
                                    if(FE.htz.extra_limit<0.0){
                                        FE.htz.extra_limit = 0.0;
                                    }
                                }
                                FE.htz.sat_min_time_reg[ind] = 0.0;
                            }
                            FE.htz.sat_max_time[ind] = 0.0;
                        }
                        FE.htz.flag_neg2posLevelB[ind] = TRUE; 
                        if(FE.htz.flag_neg2posLevelB[ind] == TRUE){ // 寻找磁链最大值
                            if(FE.htz.psi_2[ind] > FE.htz.psi_2_max[ind]){
                                FE.htz.psi_2_max[ind] = FE.htz.psi_2[ind];
                            }
                        }
                    }else{ // 磁链还没有变正，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]<0
                        FE.htz.flag_neg2posLevelA[ind] = FALSE;
                    }
                }
                if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]>0){ // 发现磁链由负变正的时刻
                    FE.htz.flag_neg2posLevelA[ind] = TRUE;
                    FE.htz.time_neg2pos[ind] = (*CTRL).timebase;
                }
            }

            /*这里一共有四种方案，积分两种，LPF两种：
            1. Holtz03原版是用u_off_original_lpf_input过LPF，
            2. 我发现u_off_original_lpf_input过积分器才能完全补偿偏置电压，
            3. 我还提出可以直接算出偏置电压补偿误差（可加LPF），
            4. 我还提出了用饱和时间去做校正的方法*/

            // 积分方法：（从上面的程序来看，u_off的LPF的输入是每半周更新一次的。
            #if BOOL_USE_METHOD_INTEGRAL_INPUT
                #define INTEGRAL_INPUT(X)   FE.htz.u_off_saturation_time_correction[X] // exact offset calculation for compensation
                // FE.htz.sat_time_offset[X]
                // #define INTEGRAL_INPUT(X)   FE.htz.accumulated__u_off_saturation_time_correction[X]
                // #define INTEGRAL_INPUT(X)   FE.htz.u_off_original_lpf_input[X]

                long int local_sum = FE.htz.negative_cycle_in_count[0] + FE.htz.positive_cycle_in_count[0] + FE.htz.negative_cycle_in_count[1] + FE.htz.positive_cycle_in_count[1];
                if(local_sum>0){
                    FE.htz.gain_off = imife_realtime_gain_off * HOLTZ_2002_GAIN_OFFSET / ((REAL)local_sum*CL_TS);
                }
                FE.htz.u_offset[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
                FE.htz.u_offset[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);

                // 想法是好的，但是没有用，因为咱们要的不是磁链正负半周的饱和时间最大和最小相互抵消；实际上观察到正负半周是饱和时间的正弦包络线的正负半周，频率比磁链频率低多啦！
                // FE.htz.accumulated__u_off_saturation_time_correction[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
                // FE.htz.accumulated__u_off_saturation_time_correction[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);
                // if(FE.htz.sign__u_off_saturation_time_correction[0]>0){
                //     FE.htz.u_offset[0] = FE.htz.accumulated__u_off_saturation_time_correction[0];
                //     FE.htz.sign__u_off_saturation_time_correction[0] = 0;
                // }
                // if(FE.htz.sign__u_off_saturation_time_correction[1]>0){
                //     FE.htz.u_offset[1] = FE.htz.accumulated__u_off_saturation_time_correction[1];
                //     FE.htz.sign__u_off_saturation_time_correction[1] = 0;
                // }

                // 本来想二重积分消除交流波动，但是我发现饱和时间误差波动的原因是上下饱和时间清零不及时导致的。但是哦……好像也没有有效地及时清零的方法，所以还是试试双重积分吧：
                // FE.htz.u_offset_intermediate[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
                // FE.htz.u_offset_intermediate[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);
                // FE.htz.u_offset[0] += CL_TS * FE.htz.u_offset_intermediate[0];
                // FE.htz.u_offset[1] += CL_TS * FE.htz.u_offset_intermediate[1];
            #endif

            // 低通
            #if BOOL_USE_METHOD_LPF_INPUT
                #define LPF_INPUT(X) FE.htz.u_off_direct_calculated[X]
                // #define LPF_INPUT(X) FE.htz.u_off_original_lpf_input[X] // holtz03 original (but using LPF cannot fully compensate offset voltage)
                #define TAU_OFF_INVERSE (500*2*M_PI) // 越大则越接近全通 0.05 
                FE.htz.u_offset[0] = _lpf( LPF_INPUT(0), FE.htz.u_offset[0], TAU_OFF_INVERSE);
                FE.htz.u_offset[1] = _lpf( LPF_INPUT(1), FE.htz.u_offset[1], TAU_OFF_INVERSE);
            #endif

            // 差分
            // 别傻，不是在这里更新的，此处更新频率是1/CL_TS啊…… 
            // FE.htz.u_offset[0] += DIFFERENCE_INPUT(0);
            // FE.htz.u_offset[1] += DIFFERENCE_INPUT(1);

            // 直通
            // FE.htz.u_offset[0] = FE.htz.u_off_direct_calculated[0];
            // FE.htz.u_offset[1] = FE.htz.u_off_direct_calculated[1];

            // FE.htz.psi_1_nonSat[0] = FE.htz.psi_1[0];
            // FE.htz.psi_1_nonSat[1] = FE.htz.psi_1[1];
            // FE.htz.psi_2_nonSat[0] = FE.htz.psi_2[0];
            // FE.htz.psi_2_nonSat[1] = FE.htz.psi_2[1];
            FE.htz.theta_d = atan2(FE.htz.psi_2[1], FE.htz.psi_2[0]);
            FE.htz.theta_e = angle_diff(FE.htz.theta_d, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
            FE.htz.psi_2_prev[0] = FE.htz.psi_2[0];
            FE.htz.psi_2_prev[1] = FE.htz.psi_2[1];
            }

        //     // 直通
        //     // FE.htz.u_offset[0] = FE.htz.u_off_direct_calculated[0];
        //     // FE.htz.u_offset[1] = FE.htz.u_off_direct_calculated[1];

        //     // FE.htz.psi_1_nonSat[0] = FE.htz.psi_1[0];
        //     // FE.htz.psi_1_nonSat[1] = FE.htz.psi_1[1];
        //     // FE.htz.psi_2_nonSat[0] = FE.htz.psi_2[0];
        //     // FE.htz.psi_2_nonSat[1] = FE.htz.psi_2[1];

        //     FE.htz.psi_2_prev[0] = FE.htz.psi_2[0];
        //     FE.htz.psi_2_prev[1] = FE.htz.psi_2[1];
        // }
    #endif
    #if AFE_36_TOP_BUTT_EXACT_COMPENSATION 
    #endif
    
    #if AFE_38_OUTPUT_ERROR_CLOSED_LOOP
        void init_ClosedLoopFluxEstimatorForPMSM() {
        int ind;
        // 初始化 psi_1 和 psi_2 数组的所有元素
        for (ind=0;ind<2;++ind) {
            FE.clfe4PMSM.psi_1[ind] = (ind == 0) ? d_sim.init.KE : 0;
            FE.clfe4PMSM.psi_2[ind] = (ind == 0) ? d_sim.init.KE : 0;
            FE.clfe4PMSM.correction_integral_term[ind] = 0;
            FE.clfe4PMSM.u_offset[ind] = 0;
            FE.clfe4PMSM.current_error[ind] = 0.0;
            FE.clfe4PMSM.current_estimate[ind] = 0.0;
            FE.clfe4PMSM.hat_psi_d2 = 0.0;
        }
        // 初始化 psi_2_ampl
        FE.clfe4PMSM.psi_2_ampl = 0;

        // 初始化 x 数组的所有元素
        FE.clfe4PMSM.x[0] = d_sim.init.KE;
        FE.clfe4PMSM.x[1] = 0;
        FE.clfe4PMSM.x[2] = 0;
        FE.clfe4PMSM.x[3] = 0;
        FE.clfe4PMSM.theta_e = 0;
        FE.clfe4PMSM.theta_d =0;
        }
        /* 8. C. Output Error Closed-loop Flux Estimator */
        void rhf_ClosedLoopFluxEstimatorForPMSM_Dynamics(REAL t, REAL *x, REAL *fx){
            // x[0], x[1]: stator flux in ab frame
            // x[2]: \psi_{d\mu}
            // x[3], x[4]: integral action compensating offset voltage

            FE.clfe4PMSM.psi_2[0] = x[0] - MOTOR.Lq * IS(0);
            FE.clfe4PMSM.psi_2[1] = x[1] - MOTOR.Lq * IS(1);
            FE.clfe4PMSM.ampl_psi_2 = sqrt(FE.clfe4PMSM.psi_2[0]*FE.clfe4PMSM.psi_2[0] + FE.clfe4PMSM.psi_2[1]*FE.clfe4PMSM.psi_2[1]);
            REAL ampl_psi_2_inv=0.0;
            REAL cos_rho = 1.0;
            
            REAL sin_rho = 0.0;
            if(FE.clfe4PMSM.ampl_psi_2!=0){
                ampl_psi_2_inv = 1.0/FE.clfe4PMSM.ampl_psi_2;
                cos_rho = FE.clfe4PMSM.psi_2[0] * ampl_psi_2_inv;
                sin_rho = FE.clfe4PMSM.psi_2[1] * ampl_psi_2_inv;
            }

            
            /* Variant with command rho */
            // REAL cos_rho = (*CTRL).s->cosT;
            // REAL sin_rho = (*CTRL).s->sinT;
            /* Lascu CLE */
            // REAL psi_ds = x[0]*cos_rho + x[1]*sin_rho;
            // fx[2] = (*CTRL).motor->Rreq*(*CTRL).motor->Lq_inv*psi_ds - (*CTRL).motor->alpha*((*CTRL).motor->Lmu+(*CTRL).motor->Lq)*(*CTRL).motor->Lq_inv*x[2]\
            //       + OUTPUT_ERROR_CLEST_GAIN_KCM*(cos_rho*current_error[0] + sin_rho*current_error[1]);
            /* MRAS */
            REAL ids = 0.0;
            ids = IS(0)*cos_rho + IS(1)*sin_rho;

            
            FE.clfe4PMSM.hat_psi_d2 = (MOTOR.Ld - MOTOR.Lq) * ids + (*CTRL).motor->KE;
            // hat_psi_d2 = (*CTRL).motor->KE;
            FE.clfe4PMSM.current_estimate[0] = MOTOR.Lq_inv * (x[0] - FE.clfe4PMSM.hat_psi_d2*cos_rho);
            FE.clfe4PMSM.current_estimate[1] = MOTOR.Lq_inv * (x[1] - FE.clfe4PMSM.hat_psi_d2*sin_rho);

            
            FE.clfe4PMSM.current_error[0] = IS(0) - FE.clfe4PMSM.current_estimate[0];
            FE.clfe4PMSM.current_error[1] = IS(1) - FE.clfe4PMSM.current_estimate[1];

            REAL emf[2] = {0.0, 0.0};
            emf[0] = US(0) - (*CTRL).motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA \
                /*P*/+ OUTPUT_ERROR_CLEF4PMSM_GAIN_KP * FE.clfe4PMSM.current_error[0] \
                /*I*/+ x[2];
            emf[1] = US(1) - (*CTRL).motor->R*IS(1) + OFFSET_VOLTAGE_BETA  \
                /*P*/+ OUTPUT_ERROR_CLEF4PMSM_GAIN_KP * FE.clfe4PMSM.current_error[1] \
                /*I*/+ x[3];
            
            fx[0] = emf[0];
            fx[1] = emf[1];
            fx[2] = OUTPUT_ERROR_CLEF4PMSM_GAIN_KI * FE.clfe4PMSM.current_error[0];
            fx[3] = OUTPUT_ERROR_CLEF4PMSM_GAIN_KI * FE.clfe4PMSM.current_error[1];
        }
        void Main_VM_ClosedLoopFluxEstimatorForPMSM(){
            /* Proposed closed loop estimator AB frame + ODE4 */
            // stator flux and integral states update
            general_4states_rk4_solver(&rhf_ClosedLoopFluxEstimatorForPMSM_Dynamics, (*CTRL).timebase, FE.clfe4PMSM.x, CL_TS);
            // Unpack x
            FE.clfe4PMSM.psi_1[0]                    = FE.clfe4PMSM.x[0];
            FE.clfe4PMSM.psi_1[1]                    = FE.clfe4PMSM.x[1];
            FE.clfe4PMSM.correction_integral_term[0] = FE.clfe4PMSM.x[2];
            FE.clfe4PMSM.correction_integral_term[1] = FE.clfe4PMSM.x[3];
            FE.clfe4PMSM.u_offset[0] = FE.clfe4PMSM.correction_integral_term[0];
            FE.clfe4PMSM.u_offset[1] = FE.clfe4PMSM.correction_integral_term[1];
            // rotor flux updates
            FE.clfe4PMSM.psi_2[0] = FE.clfe4PMSM.psi_1[0] - (*CTRL).motor->Lq*IS_C(0);
            FE.clfe4PMSM.psi_2[1] = FE.clfe4PMSM.psi_1[1] - (*CTRL).motor->Lq*IS_C(1);    
            FE.clfe4PMSM.psi_2_ampl = sqrt(FE.clfe4PMSM.psi_2[0]*FE.clfe4PMSM.psi_2[0]+FE.clfe4PMSM.psi_2[1]*FE.clfe4PMSM.psi_2[1]);
            FE.clfe4PMSM.theta_d = atan2(FE.clfe4PMSM.psi_2[1], FE.clfe4PMSM.psi_2[0]);
            FE.clfe4PMSM.theta_e = angle_diff(FE.clfe4PMSM.theta_d, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
        }
    #endif

    #if AFE_40_JO_CHOI_METHOD
        void init_joChoi(){
        }
    #endif
    #if AFE_41_LascuAndreescus2006
        /* 3. Lascu and Andreescus 2006 TODO 非常好奇Lascu的方法会怎样！和我们的xRho校正项对比！ */
    void init_LascuAndreescus2006(){
        int ind;
        FE.lascu.x[0] = d_sim.init.KE;
        FE.lascu.x[1] = 0;
        FE.lascu.x[2] = 0;
        FE.lascu.x[3] = 0;
        for (ind=0;ind<2;++ind) {
        FE.lascu.psi_1[ind] = (ind == 0) ? d_sim.init.KE : 0;
        FE.lascu.psi_2[ind] = (ind == 0) ? d_sim.init.KE : 0;
        FE.lascu.correction_integral_term[ind] = 0;
        FE.lascu.u_offset[ind] = 0;
    }
    }
    void rhf_LascuAndreescus2006_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL rotor_flux[2];
        rotor_flux[0] = x[0]-(*CTRL).motor->Lq*IS(0);
        rotor_flux[1] = x[1]-(*CTRL).motor->Lq*IS(1);

        REAL ampl     = sqrt(rotor_flux[0]*rotor_flux[0] + rotor_flux[1]*rotor_flux[1]);
        REAL ampl_inv = 0;
        if(ampl!=0){
           ampl_inv = 1.0/ampl;
        }
        REAL rotor_flux_error[2]={0,0};
        rotor_flux_error[0] = ( (*CTRL).i->cmd_psi - ampl ) * rotor_flux[0] * ampl_inv;
        rotor_flux_error[1] = ( (*CTRL).i->cmd_psi - ampl ) * rotor_flux[1] * ampl_inv;

        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA \
            /*P*/+ 1*VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[0] \
            /*I*/+ 1*x[2];
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + 1*OFFSET_VOLTAGE_BETA  \
            /*P*/+ 1*VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[1] \
            /*I*/+ 1*x[3];
        fx[0] = emf[0];
        fx[1] = emf[1];
        fx[2] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[0];
        fx[3] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[1];
    }
    void VM_LascuAndreescus2006(){
        /* Proposed VM based Flux Command Error Feedback Correction in Controller Frame (xRho), implemented in AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_LascuAndreescus2006_Dynamics, (*CTRL).timebase, FE.lascu.x, CL_TS);
        // Unpack x
        FE.lascu.psi_1[0]                    = FE.lascu.x[0];
        FE.lascu.psi_1[1]                    = FE.lascu.x[1];
        FE.lascu.correction_integral_term[0] = FE.lascu.x[2];
        FE.lascu.correction_integral_term[1] = FE.lascu.x[3];
        FE.lascu.u_offset[0] = FE.lascu.correction_integral_term[0];
        FE.lascu.u_offset[1] = FE.lascu.correction_integral_term[1];
        // rotor flux updates
        FE.lascu.psi_2[0] = FE.lascu.psi_1[0] - (*CTRL).motor->Lq * IS_C(0);
        FE.lascu.psi_2[1] = FE.lascu.psi_1[1] - (*CTRL).motor->Lq * IS_C(1);
        FE.lascu.psi_2_ampl = sqrt(FE.lascu.psi_2[0]*FE.lascu.psi_2[0]+FE.lascu.psi_2[1]*FE.lascu.psi_2[1]);
        FE.lascu.theta_d = atan2(FE.lascu.psi_2[1], FE.lascu.psi_2[0]);
        FE.lascu.theta_e = angle_diff(FE.lascu.theta_d, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
    }

    #endif
    #if AFE_43_SuperTwistingA
    void init_SuperTwistingA(){
        FE.STA.current_error[0] = 0;
        FE.STA.current_error[1] = 0;
        FE.STA.current_estimate[0] = 0;
        FE.STA.current_estimate[1] = 0;
        FE.STA.k1 = 5;
        FE.STA.k2 = 2000;
        FE.STA.psi_A[0] = d_sim.init.KE;
        FE.STA.psi_A[1] = 0;
        FE.STA.theta_d= 0;
        FE.STA.theta_e = 0;
        FE.STA.theta_d_emf = 0;
        FE.STA.theta_e_emf = 0;
        FE.STA.emf_A[0] = 0;
        FE.STA.emf_A[1] = 0;
        FE.STA.x[0] = 0;
        FE.STA.x[1] = 0;
        FE.STA.x[2] = d_sim.init.KE *1.2; 
        FE.STA.x[3] = -0.04;
        FE.STA.x[4] = 0;
        FE.STA.x[5] = 0;
        FE.STA.emf_A[0] = 0;
        FE.STA.emf_A[1] = 0.0;
        FE.STA.k1_term[0] = 0;
        FE.STA.k1_term[1] = 0;
        FE.STA.k2_term[0] = 0;
        FE.STA.k2_term[1] = 0;
    }
    void rhf_SuperTwistingA_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0]: alpha-axis current estimate
        // x[1]: beta-axis current estimate
        // x[2]: alpha-axis active flux
        // x[3]: beta-axis active flux
        // x[4]: k2 term[0] = integral of sign error * k2
        // x[5]: k2 term[1] = integral of sign error * k2
        REAL error[2];
        REAL sign_error[2];
        REAL sqrt_error[2];

        // Feedback terms
        error[0] = IS(0) - x[0];
        error[1] = IS(1) - x[1];
        sign_error[0] = sign(error[0]);
        sign_error[1] = sign(error[1]);
        sqrt_error[0] = sqrt(fabs(error[0]));
        sqrt_error[1] = sqrt(fabs(error[1]));

        // Dynamics
        FE.STA.emf_A[0] = -(FE.STA.k1*sqrt_error[0]*sign_error[0] + x[4]);
        FE.STA.emf_A[1] = -(FE.STA.k1*sqrt_error[1]*sign_error[1] + x[5]);
        fx[0] = MOTOR.Lq_inv * (US(0) - MOTOR.R * x[0] - FE.STA.emf_A[0]);
        fx[1] = MOTOR.Lq_inv * (US(1) - MOTOR.R * x[1] - FE.STA.emf_A[1]);
        // fx[0] = MOTOR.Lq_inv * (US(0) - MOTOR.R * IS(0) + FE.STA.emf_A[0]);
        // fx[1] = MOTOR.Lq_inv * (US(1) - MOTOR.R * IS(1) + FE.STA.emf_A[1]);
        fx[2] = FE.STA.emf_A[0]; // TODO: this is an open loop estimator!!! 
        fx[3] = FE.STA.emf_A[1]; // TODO: this is an open loop estimator!!! 
        fx[4] = FE.STA.k2 * sign_error[0];
        fx[5] = FE.STA.k2 * sign_error[1];
    }
    void Main_SuperTwistingA(){
        // if (PLLN.omega_elec != 0)
        // {
        //     FE.STA.k1 = 0.008 * PLLN.omega_elec;
        //     FE.STA.k2 = 52.64 * PLLN.omega_elec * PLLN.omega_elec;
        // }else{
        //     FE.STA.k1 = 5;
        //     FE.STA.k2 = 2000;
        // }
        
        general_6states_rk4_solver(&rhf_SuperTwistingA_Dynamics, (*CTRL).timebase, FE.STA.x, CL_TS);
        // Unpack x
        FE.STA.current_estimate[0] = FE.STA.x[0];
        FE.STA.current_estimate[1] = FE.STA.x[1];
        FE.STA.psi_A[0]            = FE.STA.x[2];
        FE.STA.psi_A[1]            = FE.STA.x[3];
        FE.STA.k2_term[0]          = FE.STA.x[4];
        FE.STA.k2_term[1]          = FE.STA.x[5];
        // Main_Saturation_time_Without_Limiting(FE.STA.psi_A);
        FE.STA.current_error[0] = IS_C(0) - FE.STA.x[0];
        FE.STA.current_error[1] = IS_C(1) - FE.STA.x[1];
            // FE.STA.k1_term[0] = FE.STA.k1*sqrt(fabs(IS_C(0) - FE.STA.x[0]))*sign(IS_C(0) - FE.STA.x[0]);
            // FE.STA.k1_term[1] = FE.STA.k1*sqrt(fabs(IS_C(1) - FE.STA.x[1]))*sign(IS_C(1) - FE.STA.x[1]);
        FE.STA.k1_term[0] = FE.STA.emf_A[0] - FE.STA.k2_term[0];
        FE.STA.k1_term[1] = FE.STA.emf_A[1] - FE.STA.k2_term[1];
        Main_PLL_norm(FE.STA.emf_A);
        FE.STA.theta_d_emf = -atan2(FE.STA.emf_A[0] * sign(PLLN.omega_elec), FE.STA.emf_A[1] * sign(PLLN.omega_elec));
        FE.STA.theta_d     =  atan2(FE.STA.psi_A[1], FE.STA.psi_A[0]);
        FE.STA.theta_e     = angle_diff(FE.STA.theta_d,     (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
        FE.STA.theta_e_emf = angle_diff(FE.STA.theta_d_emf, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
    }
    #endif
    #if AFE_44_ORTEGA_2011
    void init_ortega(){
        FE.Ortega.x[0] = d_sim.init.KE*0.7;
        FE.Ortega.x[1] = 0;
        FE.Ortega.psi_1[0] = 0;
        FE.Ortega.psi_1[1] = 0;
        FE.Ortega.psi_2[0] = 0;
        FE.Ortega.psi_2[1] = 0;
        FE.Ortega.kp = 500;
        FE.Ortega.u_offset[0] = 0;
        FE.Ortega.u_offset[1] = 0;
        FE.Ortega.psi_sqr_error = 0;
        FE.Ortega.theta_d = 0;
        FE.Ortega.theta_e = 0;
    }
    void rhf_orgeta_2011_Dynamics(REAL t, REAL *x, REAL *fx){

        REAL psi_A[2];
        psi_A[0] = x[0] - MOTOR.Lq * IS(0);
        psi_A[1] = x[1] - MOTOR.Lq * IS(1);
        FE.Ortega.psi_sqr_error = MOTOR.KE*MOTOR.KE - (psi_A[0]*psi_A[0] + psi_A[1]*psi_A[1]);
        // Dynamics
        fx[0] = US(0) - MOTOR.R * IS(0) + 0.5 * FE.Ortega.kp * psi_A[0] * FE.Ortega.psi_sqr_error + OFFSET_VOLTAGE_ALPHA;
        fx[1] = US(1) - MOTOR.R * IS(1) + 0.5 * FE.Ortega.kp * psi_A[1] * FE.Ortega.psi_sqr_error + OFFSET_VOLTAGE_BETA;
    }
    void main_ortega_2011(){

        general_2states_rk4_solver(&rhf_orgeta_2011_Dynamics, (*CTRL).timebase, FE.Ortega.x, CL_TS);
        // Unpack x
        FE.Ortega.psi_1[0] = FE.Ortega.x[0];
        FE.Ortega.psi_1[1] = FE.Ortega.x[1];

        FE.Ortega.psi_2[0] = FE.Ortega.psi_1[0] - MOTOR.Lq * IS_C(0);
        FE.Ortega.psi_2[1] = FE.Ortega.psi_1[1] - MOTOR.Lq * IS_C(1);
        FE.Ortega.theta_d = atan2(FE.Ortega.psi_2[1], FE.Ortega.psi_2[0]);
        FE.Ortega.theta_e = angle_diff(FE.Ortega.theta_d, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
    }
    
    #endif
    #if AFE_45_CMwithDynamicCurrent
    void init_CMwithDynamicCurrent(){
        FE.CMDC.x[0] = 0;
        FE.CMDC.x[1] = 0;
        FE.CMDC.x[2] = 0;
        FE.CMDC.x[3] = 0;
        FE.CMDC.x[4] = 0;
        FE.CMDC.x[5] = 0;
        FE.CMDC.cosT = 1;
        FE.CMDC.sinT = 0;
        FE.CMDC.udq[0] = 0;
        FE.CMDC.udq[1] = 0;
        FE.CMDC.idq[0] = 0;
        FE.CMDC.idq[1] = 0;
        FE.CMDC.emf_ss_dq[0] = 0;
        FE.CMDC.emf_ss_dq[1] = 0;
        FE.CMDC.tilde_idq[0] = 0;
        FE.CMDC.tilde_idq[1] = 0;
        FE.CMDC.hat_idq[0] = 0.0;
        FE.CMDC.hat_idq[1] = 0.0;
        FE.CMDC.integral_tilde_id = 0.0;
        FE.CMDC.hat_emf_A_ss = 0.0;
        FE.CMDC.theta_d = 0.0;
        FE.CMDC.omega_elec = 0.0;
        FE.CMDC.tilde_theta_d = 0.0;
        FE.CMDC.kp = 1*500; // position observer gain (5000 leads to serious speed oscillation)
        FE.CMDC.ki = 0.0; //2000000; // position observer gain
        FE.CMDC.KP = 2.5; // speed obserer gain
        FE.CMDC.KI = 150; // speed obserer gain (load torque)
        FE.CMDC.gamma_id = 0*40.0; // current observer gain
        FE.CMDC.gamma_iq = 0*40.0; // current observer gain
    }
    void rhf_CMwithDynamicCurrent_Dynamics(REAL t, REAL *x, REAL *fx){
        // see "Overview" paper (18)

        // Update DQ frame quantities
        FE.CMDC.cosT = cos(x[4]);
        FE.CMDC.sinT = sin(x[4]);
        FE.CMDC.idq[0] = AB2M(IS(0), IS(1), FE.CMDC.cosT, FE.CMDC.sinT);
        FE.CMDC.idq[1] = AB2T(IS(0), IS(1), FE.CMDC.cosT, FE.CMDC.sinT);
        FE.CMDC.udq[0] = AB2M(US(0), US(1), FE.CMDC.cosT, FE.CMDC.sinT);
        FE.CMDC.udq[1] = AB2T(US(0), US(1), FE.CMDC.cosT, FE.CMDC.sinT);

        FE.CMDC.tilde_idq[0] = FE.CMDC.idq[0] - x[0];
        FE.CMDC.tilde_idq[1] = FE.CMDC.idq[1] - x[1];

        // Position Subsystem
        // 4. dynamics of theta_d
        fx[4] = x[3] + sign(x[3]) * (FE.CMDC.kp * FE.CMDC.tilde_idq[0] + FE.CMDC.ki * x[2]);

        if(TRUE){
            FE.CMDC.omega_elec = x[3]; // LASCU2020
        }else{
            FE.CMDC.omega_elec = fx[4]; // Our proposed method
        }

        // 0. dynamics of hat_i_dq[0] (closed loop estimation version)
        FE.CMDC.emf_ss_dq[0] = FE.CMDC.udq[0] - MOTOR.R * x[0] + FE.CMDC.omega_elec * MOTOR.Lq  * x[1]; // 这样写有利于稳定性证明
            // FE.CMDC.emf_ss_dq[0] = FE.CMDC.udq[0] - MOTOR.R * x[0] + FE.CMDC.omega_elec * MOTOR.Lq  * FE.CMDC.idq[1]; // Lascu2020
        fx[0] = MOTOR.Ld_inv * (FE.CMDC.emf_ss_dq[0] + FE.CMDC.gamma_id * FE.CMDC.tilde_idq[0]);
        // 2. dynamics of integral_tilde_id driven by D-axis current error
        fx[2] = FE.CMDC.tilde_idq[0];

        // MOTOR.KActive = (MOTOR.Ld - MOTOR.Lq) * FE.CMDC.idq[0] + MOTOR.KE;
        MOTOR.KActive = (MOTOR.Ld - MOTOR.Lq) * x[0] + MOTOR.KE;

        // Speed Subsystem
        // 1. dynamics of hat_i_dq[1] (open loop est imation version)
        FE.CMDC.emf_ss_dq[1] = FE.CMDC.udq[1] - MOTOR.R * x[1] - FE.CMDC.omega_elec * MOTOR.Lq * x[0]; // 这样写有利于稳定性证明
            // FE.CMDC.emf_ss_dq[1] = FE.CMDC.udq[1] - MOTOR.R * x[1] - FE.CMDC.omega_elec * MOTOR.Lq * FE.CMDC.idq[0]; // Lascu2020
        fx[1] = MOTOR.Lq_inv * (FE.CMDC.emf_ss_dq[1] - FE.CMDC.omega_elec*MOTOR.KActive + FE.CMDC.gamma_iq * FE.CMDC.tilde_idq[1]);
        // 3. dynamics of speed
        // fx[3] = MOTOR.npp*MOTOR.Js_inv * (1.5*MOTOR.npp*MOTOR.KActive*FE.CMDC.idq[1] - FE.CMDC.KP * FE.CMDC.tilde_idq[1] - FE.CMDC.KI * x[5] ); // Lascu2020
        fx[3] = MOTOR.npp*MOTOR.Js_inv * (1.5*MOTOR.npp*MOTOR.KActive*x[1] - FE.CMDC.KP * FE.CMDC.tilde_idq[1] - FE.CMDC.KI * x[5] ); // 这样写有利于稳定性证明
        // 5. dynamics of integral of iq error
        fx[5] = FE.CMDC.tilde_idq[1];
    }
    void Main_CMwithDynamicCurrent(){

        general_6states_rk4_solver(&rhf_CMwithDynamicCurrent_Dynamics, (*CTRL).timebase, FE.CMDC.x, CL_TS);
        // Unpack x
        FE.CMDC.hat_idq[0] = FE.CMDC.x[0];
        FE.CMDC.hat_idq[1] = FE.CMDC.x[1];
        FE.CMDC.integral_tilde_id = FE.CMDC.x[2];
        // FE.CMDC.omega_elec = FE.CMDC.x[3];
        FE.CMDC.theta_d = FE.CMDC.x[4];
        FE.CMDC.integral_tilde_iq = FE.CMDC.x[5];
        while(FE.CMDC.theta_d > M_PI) FE.CMDC.theta_d  -= 2*M_PI;
        while(FE.CMDC.theta_d < -M_PI) FE.CMDC.theta_d += 2*M_PI;
        FE.CMDC.tilde_theta_d = angle_diff(FE.CMDC.theta_d, ACM.theta_d) * ONE_OVER_2PI * 360;
    }
    #endif
    void simulation_test_flux_estimators(){
        // MainFE_HUWU_1998();
        #if AFE_43_SuperTwistingA
            Main_SuperTwistingA();
        #endif
        #if AFE_44_ORTEGA_2011
            main_ortega_2011();
        #endif
        #if AFE_35_SATURATION_TIME_DIFFERENCE
        VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
        #endif
        #if AFE_25_VM_CM_FUSION
        Main_the_active_flux_estimator();
        #endif
        // VM_LascuAndreescus2006();
        #if AFE_38_OUTPUT_ERROR_CLOSED_LOOP
            Main_VM_ClosedLoopFluxEstimatorForPMSM();
        #endif
        #if AFE_45_CMwithDynamicCurrent
            Main_CMwithDynamicCurrent();
        #endif
        // Main_Saturation_time_Without_Limiting();
        // VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
    }

    void init_FE(){
        // init_FE_huwu();
        #if AFE_38_OUTPUT_ERROR_CLOSED_LOOP
        init_ClosedLoopFluxEstimatorForPMSM();
        #endif
        #if AFE_25_VM_CM_FUSION
        init_afe();
        #endif
        #if AFE_43_SuperTwistingA
            init_SuperTwistingA();
        #endif
        #if AFE_44_ORTEGA_2011
            init_ortega();
        #endif
        #if AFE_35_SATURATION_TIME_DIFFERENCE
            init_FE_htz();
        #endif
        #if AFE_13_LASCU_ANDREESCUS_2006
            init_LascuAndreescus2006();
        #endif
        #if AFE_45_CMwithDynamicCurrent
            init_CMwithDynamicCurrent();
        #endif
        // init_Saturation_time_Without_Limiting();
    }

    #endif
/*share flux estimator end*/


/* from pmsm observer */ 

#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
/********************************************/
/* 4rd-order ESO 
 ********************************************/
/* The 4rd-order dynamic system */

/********************************************/
/* Natural Speed Observer for IPMSM with Active Flux Concept (Chen 2020)
 ********************************************/
/* The 3rd-order dynamic system */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_NSOAF
void rhf_NSO_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    REAL xIq  = x[0];
    REAL xOmg = x[1];
    REAL xTL  = x[2];

    /* Know Signals */
    REAL iDQ_now[2];
    iDQ_now[0]  = AB2M(IS(0), IS(1), AFE_USED.cosT, AFE_USED.sinT);
    iDQ_now[1]  = AB2T(IS(0), IS(1), AFE_USED.cosT, AFE_USED.sinT);
    REAL uQ_now = AB2T(US(0), US(1), AFE_USED.cosT, AFE_USED.sinT);
    OBSV.nsoaf.q_axis_voltage = uQ_now;

    /* Output Error = \tilde i_q (scalar) */
    OBSV.nsoaf.output_error = iDQ_now[1] - xIq;
    #ifdef NSOAF_SPMSM

        /* Filter (TODO: 不能把欧拉法放到龙贝格法里面来啊！) */
        static REAL uQ_now_filtered = 0.0;
        uQ_now_filtered = _lpf(uQ_now, uQ_now_filtered, 30); // 越大滤越狠
        // uQ_now_filtered = uQ_now;

        OBSV.nsoaf.active_power_real  = + fabsf(uQ_now_filtered) * iDQ_now[1];
        OBSV.nsoaf.active_power_est   = + fabsf(uQ_now_filtered) * xIq;
        OBSV.nsoaf.active_power_error = + fabsf(uQ_now_filtered) * OBSV.nsoaf.output_error; 
    #endif

    #ifdef NSOAF_IPMSM
        OBSV.nsoaf.active_power_real  = + iDQ_now[1];
        OBSV.nsoaf.active_power_est   = + xIq;
        OBSV.nsoaf.active_power_error = + OBSV.nsoaf.output_error; /*妈耶……结果不要把uQ拿进来就完美了……*/
    #endif

    /* State Observer */
    // xIq
    fx[0] = MOTOR.Lq_inv * (uQ_now - MOTOR.R * xIq - xOmg*(MOTOR.KE + MOTOR.Ld*iDQ_now[0])) - OBSV.nsoaf.KD*OBSV.nsoaf.active_power_error;
    #if RS_IDENTIFICATION
        fx[0] = MOTOR.Lq_inv * (uQ_now - akt.rs_cal * xIq - xOmg*(MOTOR.KE + MOTOR.Ld*iDQ_now[0])) - OBSV.nsoaf.KD*OBSV.nsoaf.active_power_error;
    #endif

    #if LQ_IDENTIFICATION
        fx[0] = q_inductanceid.Lq_inv * (uQ_now - MOTOR.R  * xIq - xOmg*(MOTOR.KE + MOTOR.Ld*iDQ_now[0])) - OBSV.nsoaf.KD*OBSV.nsoaf.active_power_error;
    #endif
    // xOmg
    // REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_now[0];
    OBSV.nsoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * MOTOR.KActive * xIq;
    // xTL = ACM.TLoad; // DEBUG
    fx[1] = MOTOR.Js_inv * MOTOR.npp * (OBSV.nsoaf.xTem - xTL - OBSV.nsoaf.KP*OBSV.nsoaf.active_power_error);
    /* Parameter Adaptation */
    // xTL
    fx[2] = OBSV.nsoaf.KI * OBSV.nsoaf.active_power_error;
}
void nso_one_parameter_tuning(REAL omega_ob){
    if(omega_ob<200){
        OBSV.nsoaf.set_omega_ob = 200;
        return;
    }

    #define LQ_INV MOTOR.Lq_inv
    REAL one_over__npp_divided_by_Js__times__Lq_id_plus_KActive = 1.0 \
        / ( MOTOR.npp 
            * MOTOR.Js_inv * LQ_INV
            * (MOTOR.Lq*(*CTRL).i->cmd_iDQ[0] + MOTOR.KActive)
          );
    REAL uq_inv = 1.0 / (*CTRL).o->cmd_uDQ[1];
    #if TUNING_IGNORE_UQ
        uq_inv = 1.0; // this is 1.#INF when init
    #endif

    // OBSV.nsoaf.KD = 3*omega_ob                                     * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;
    // OBSV.nsoaf.KP = ( (3*omega_ob*omega_ob - MOTOR.R*LQ_INV) * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive - 1.5*MOTOR.npp*MOTOR.KActive ) * uq_inv;
    // OBSV.nsoaf.KI = omega_ob*omega_ob*omega_ob                     * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;

    OBSV.nsoaf.KD = (  3*omega_ob - MOTOR.R*LQ_INV) * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;
    #if RS_IDENTIFICATION
        OBSV.nsoaf.KD = (  3*omega_ob - akt.rs_cal*LQ_INV) * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                          * uq_inv;
    #endif
    OBSV.nsoaf.KP = ( (3*omega_ob*omega_ob)         * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive - 1.5*MOTOR.npp*MOTOR.KActive ) * uq_inv;
    OBSV.nsoaf.KI = omega_ob*omega_ob*omega_ob      * one_over__npp_divided_by_Js__times__Lq_id_plus_KActive                                 * uq_inv;

    #if PC_SIMULATION
    #if TUNING_IGNORE_UQ
    printf("Init NSOAF:\n");
    printf("\tIgnore uq.\n");
    #endif
    printf("\tomega_ob=%g\n", omega_ob);
    printf("\t1.5*MOTOR.npp*MOTOR.KActive=%g\n", 1.5*MOTOR.npp*MOTOR.KActive);
    printf("\tone_over__npp_divided_by_Js__times__Lq_id_plus_KActive=%g\n", one_over__npp_divided_by_Js__times__Lq_id_plus_KActive);
    printf("\tKD=%g\n\tKP=%g\n\tKI=%g\n", OBSV.nsoaf.KD, OBSV.nsoaf.KP, OBSV.nsoaf.KI);
    #endif
}
void Main_nsoaf_chen2020(){

    /* OBSERVATION */

    // TODO: when id changes, the gain should also change.
    if(OBSV.nsoaf.set_omega_ob != OBSV.nsoaf.omega_ob){
        OBSV.nsoaf.omega_ob = OBSV.nsoaf.set_omega_ob;
        nso_one_parameter_tuning(OBSV.nsoaf.omega_ob);
        // afe_one_parameter_tuning(OBSV.nsoaf.omega_ob);
    }

    /* Unpack States */
    OBSV.nsoaf.xIq      = OBSV.nsoaf.xBest[0];
    OBSV.nsoaf.xOmg     = OBSV.nsoaf.xBest[1];
    OBSV.nsoaf.xTL      = OBSV.nsoaf.xBest[2];

    /* Know Signals */
    #define MOTOR (*(*CTRL).motor)
    REAL iDQ_now[2];
    iDQ_now[0]  = AB2M(IS_C(0), IS_C(1), AFE_USED.cosT, AFE_USED.sinT);
    iDQ_now[1]  = AB2T(IS_C(0), IS_C(1), AFE_USED.cosT, AFE_USED.sinT);
    REAL uQ_now = AB2T(US_C(0), US_C(1), AFE_USED.cosT, AFE_USED.sinT);
    OBSV.nsoaf.q_axis_voltage = uQ_now;

    /* Output Error = \tilde i_q (scalar) */
    // OBSV.nsoaf.output_error = iDQ_now[1] - OBSV.nsoaf.xIq;
    // OBSV.nsoaf.active_power_real  = + fabsf(uQ_now) * iDQ_now[1];
    // OBSV.nsoaf.active_power_est   = + fabsf(uQ_now) * OBSV.nsoaf.xIq;
    // OBSV.nsoaf.active_power_error = + fabsf(uQ_now) * OBSV.nsoaf.output_error;

    OBSV.nsoaf.active_power_real  = + 1 * iDQ_now[1];
    OBSV.nsoaf.active_power_est   = + 1 * OBSV.nsoaf.xIq;
    OBSV.nsoaf.active_power_error = + 1 * OBSV.nsoaf.output_error;

    REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_now[0];
    // if(CTRL->timebase > 1.0){
    //     KActive = FE.no_sat.ell_1 + (MOTOR.Ld - MOTOR.Lq) * iDQ_now[0];
    // }
    OBSV.nsoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * KActive * OBSV.nsoaf.xIq;
    #if LQ_IDENTIFICATION
        OBSV.nsoaf.xTem = CLARKE_TRANS_TORQUE_GAIN * MOTOR.npp * (KActive = MOTOR.KE + (MOTOR.Ld - q_inductanceid.Lq) * iDQ_now[0]) * OBSV.nsoaf.xIq;
    #endif


    /*  xTL的方程只是提供了粒子的xTL的取值建议；
        转速的方程，会根据你的粒子的xTL的值去计算出“下一步的转速”；
        然后你才根据下一步的电流来判断上面得到的“下一步的转速”到底准不准。
    */
    /* 放弃 MPO 或者说 DBO 的原因是目标函数（输出误差varepsilon）为零并不对应最优点 */
        // REAL fx[NS];
        // REAL iQ_C = AB2T(IS_C(0), IS_C(1), OBSV.nsoaf.cosT, OBSV.nsoaf.sinT);

        // REAL best_xTL;

        // 直接解算令输出误差为零的转速值（欧拉法ode1）
        // OBSV.nsoaf.solved_xOmg = ((uQ_now - MOTOR.R * xIq       ) - (iQ_C - xIq)*MOTOR.Lq*CL_TS_INVERSE) / (MOTOR.KE + MOTOR.Ld*iDQ_now[0]);
        // OBSV.nsoaf.solved_xOmg = ((uQ_now - MOTOR.R * iDQ_now[1]) - (iQ_C - xIq)*MOTOR.Lq*CL_TS_INVERSE) / (MOTOR.KE + MOTOR.Ld*iDQ_now[0]);

        // REAL iQ_C = AB2T(IS_C(0), IS_C(1), OBSV.nsoaf.cosT, OBSV.nsoaf.sinT);
        // REAL best_output_error = 99999;
        // int  best_index = 0;
        // int j;
        // for(j=0; j<10; ++j){
        //     OBSV.nsoaf.KP = (j+1) * 0.2 * NSOAF_TL_P*0;
        //     OBSV.nsoaf.KI = (j+1) * 0.2 * NSOAF_TL_I;
        //     OBSV.nsoaf.KD = (j+1) * 0.2 * NSOAF_TL_D;
        //     general_3_states_rk4_solver(&rhf_NSO_Dynamics, (*CTRL).timebase, (OBSV.nsoaf.x+j*NS), CL_TS);
        //     OBSV.nsoaf.output_error = iQ_C - *(OBSV.nsoaf.x+j*NS);
        //     if(fabsf(OBSV.nsoaf.output_error) < fabsf(best_output_error)){
        //         best_output_error = OBSV.nsoaf.output_error;
        //         best_index = j;
        //     }
        // }
        // OBSV.nsoaf.xBest[0] = OBSV.nsoaf.x[best_index*NS+0];
        // OBSV.nsoaf.xBest[1] = OBSV.nsoaf.x[best_index*NS+1];
        // OBSV.nsoaf.xBest[2] = OBSV.nsoaf.x[best_index*NS+2];

        // DEBUG
        // OBSV.nsoaf.xBest[1] = ACM.omg_elec;

    general_3states_rk4_solver(&rhf_NSO_Dynamics, (*CTRL).timebase, OBSV.nsoaf.xBest, CL_TS);


    /* Unpack x (for the best) */
    // OBSV.nsoaf.xIq      = OBSV.nsoaf.xBest[0];
    // OBSV.nsoaf.xOmg     = OBSV.nsoaf.xBest[1];
    // OBSV.nsoaf.xTL      = OBSV.nsoaf.xBest[2];
    /* Post-observer calculations */
    /* Selecting Signals From Block Diagram */
    OBSV.nsoaf.LoadTorquePI = OBSV.nsoaf.xTL + OBSV.nsoaf.KP*OBSV.nsoaf.active_power_error;
}
void init_nsoaf(){

    OBSV.nsoaf.KP = NSOAF_TL_P;
    OBSV.nsoaf.KI = NSOAF_TL_I;
    OBSV.nsoaf.KD = NSOAF_TL_D;
    OBSV.nsoaf.set_omega_ob = NSOAF_OMEGA_OBSERVER;

    MOTOR.KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->iDQ[0];
    OBSV.nsoaf.omega_ob = OBSV.nsoaf.set_omega_ob;
    nso_one_parameter_tuning(OBSV.nsoaf.omega_ob);
}
#endif

/********************************************/
/* EEMF-Speed-Adaptive-High-Gain-Observer-Farza-2009
 ********************************************/
/* The 10th-order dynamic system */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Farza_2009
#define NS 10
void rhf_func_hgoeemf(REAL *increment_n, REAL *xPsi, REAL *xEemf, 
                      REAL xOmg, REAL xGammaOmg, 
                      REAL *xPhi, REAL hs){
    // pointer to increment_n: state increment at n-th stage of RK4, where n=1,2,3,4.
    // *x???: pointer to state variables
    // x????: state variable
    // hs: step size of numerical integral

    // f = \dot x = the time derivative
    REAL f[NS];

    // DEBUG
    // xOmg = (*CTRL).i->omg_elec;

    // dependent parameters
    // MOTOR.Ld
    // MOTOR.DeltaL
    // MOTOR.R
    #define MOTOR (*(*CTRL).motor)

    /* Output Error of xPsi */
    OBSV.hgo4eemf.output_error[0] = MOTOR.Ld*IS(0) - xPsi[0];
    OBSV.hgo4eemf.output_error[1] = MOTOR.Ld*IS(1) - xPsi[1];

    /* Parameter Adaptation */
    // xOmg (recall xPhi is the filtered regreesor for speed)
    f[4] = OBSV.hgo4eemf.vartheta*OBSV.hgo4eemf.xGammaOmg * (xPhi[0]*OBSV.hgo4eemf.output_error[0] + xPhi[1]*OBSV.hgo4eemf.output_error[1]);
    // xGammaOmg
    f[5] = - OBSV.hgo4eemf.vartheta * xGammaOmg * (xPhi[0]*xPhi[0]+xPhi[1]*xPhi[1]) * xGammaOmg + OBSV.hgo4eemf.vartheta*xGammaOmg;

    /* Auxiliary States */
    // xPhi
    f[6] = OBSV.hgo4eemf.vartheta * (-2*xPhi[0] + xPhi[2]) + MOTOR.DeltaL*-IS(1);
    f[7] = OBSV.hgo4eemf.vartheta * (-2*xPhi[1] + xPhi[3]) + MOTOR.DeltaL* IS(0);
    f[8] = OBSV.hgo4eemf.vartheta * (  -xPhi[0] ) + OBSV.hgo4eemf.vartheta_inv*-xEemf[1];
    f[9] = OBSV.hgo4eemf.vartheta * (  -xPhi[1] ) + OBSV.hgo4eemf.vartheta_inv* xEemf[0];

    /* State Observer */
    // xPsi = Ld * i
    f[0] = xEemf[0] + US(0) - MOTOR.R*IS(0) + MOTOR.DeltaL*xOmg*-IS(1) + 2*OBSV.hgo4eemf.vartheta*OBSV.hgo4eemf.output_error[0] + xPhi[0]*f[4];
    f[1] = xEemf[1] + US(1) - MOTOR.R*IS(1) + MOTOR.DeltaL*xOmg* IS(0) + 2*OBSV.hgo4eemf.vartheta*OBSV.hgo4eemf.output_error[1] + xPhi[1]*f[4];
    // xEemf
    f[2] = xOmg*-xEemf[1] + OBSV.hgo4eemf.vartheta * (OBSV.hgo4eemf.vartheta * OBSV.hgo4eemf.output_error[0]                         + xPhi[2]*f[4]);
    f[3] = xOmg* xEemf[0] + OBSV.hgo4eemf.vartheta * (OBSV.hgo4eemf.vartheta * OBSV.hgo4eemf.output_error[1]                         + xPhi[3]*f[4]);

    increment_n[ 0] = ( f[ 0] )*hs;
    increment_n[ 1] = ( f[ 1] )*hs;
    increment_n[ 2] = ( f[ 2] )*hs;
    increment_n[ 3] = ( f[ 3] )*hs;
    increment_n[ 4] = ( f[ 4] )*hs;
    increment_n[ 5] = ( f[ 5] )*hs;
    increment_n[ 6] = ( f[ 6] )*hs;
    increment_n[ 7] = ( f[ 7] )*hs;
    increment_n[ 8] = ( f[ 8] )*hs;
    increment_n[ 9] = ( f[ 9] )*hs;
}
void hgo4eemf_dedicated_rk4_solver(REAL hs){
    static REAL increment_1[NS];
    static REAL increment_2[NS];
    static REAL increment_3[NS];
    static REAL increment_4[NS];
    static REAL x_temp[NS];
    static REAL *p_x_temp=x_temp;

    /* Theoritically speaking, rhf_func should be time-varing like rhf_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhf_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhf_func_hgoeemf( increment_1, OBSV.hgo4eemf.xPsi, OBSV.hgo4eemf.xEemf, 
                      OBSV.hgo4eemf.xOmg, OBSV.hgo4eemf.xGammaOmg,
                      OBSV.hgo4eemf.xPhi, hs); 
    x_temp[0]  = OBSV.hgo4eemf.xPsi[0]        + increment_1[0]*0.5;
    x_temp[1]  = OBSV.hgo4eemf.xPsi[1]        + increment_1[1]*0.5;
    x_temp[2]  = OBSV.hgo4eemf.xEemf[0]       + increment_1[2]*0.5;
    x_temp[3]  = OBSV.hgo4eemf.xEemf[1]       + increment_1[3]*0.5;
    x_temp[4]  = OBSV.hgo4eemf.xOmg           + increment_1[4]*0.5;
    x_temp[5]  = OBSV.hgo4eemf.xGammaOmg      + increment_1[5]*0.5;
    x_temp[6]  = OBSV.hgo4eemf.xPhi[0]        + increment_1[6]*0.5;
    x_temp[7]  = OBSV.hgo4eemf.xPhi[1]        + increment_1[7]*0.5;
    x_temp[8]  = OBSV.hgo4eemf.xPhi[2]        + increment_1[8]*0.5;
    x_temp[9]  = OBSV.hgo4eemf.xPhi[3]        + increment_1[9]*0.5;

    // time instant t+hs/2
    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    rhf_func_hgoeemf( increment_2, p_x_temp+0, p_x_temp+2, 
                    *(p_x_temp+4), *(p_x_temp+5),
                      p_x_temp+6, hs );
    x_temp[0]  = OBSV.hgo4eemf.xPsi[0]        + increment_2[0]*0.5;
    x_temp[1]  = OBSV.hgo4eemf.xPsi[1]        + increment_2[1]*0.5;
    x_temp[2]  = OBSV.hgo4eemf.xEemf[0]       + increment_2[2]*0.5;
    x_temp[3]  = OBSV.hgo4eemf.xEemf[1]       + increment_2[3]*0.5;
    x_temp[4]  = OBSV.hgo4eemf.xOmg           + increment_2[4]*0.5;
    x_temp[5]  = OBSV.hgo4eemf.xGammaOmg      + increment_2[5]*0.5;
    x_temp[6]  = OBSV.hgo4eemf.xPhi[0]        + increment_2[6]*0.5;
    x_temp[7]  = OBSV.hgo4eemf.xPhi[1]        + increment_2[7]*0.5;
    x_temp[8]  = OBSV.hgo4eemf.xPhi[2]        + increment_2[8]*0.5;
    x_temp[9]  = OBSV.hgo4eemf.xPhi[3]        + increment_2[9]*0.5;

    // time instant t+hs/2
    rhf_func_hgoeemf( increment_3, p_x_temp+0, p_x_temp+2, 
                    *(p_x_temp+4), *(p_x_temp+5),
                      p_x_temp+6, hs );
    x_temp[0]  = OBSV.hgo4eemf.xPsi[0]        + increment_3[0];
    x_temp[1]  = OBSV.hgo4eemf.xPsi[1]        + increment_3[1];
    x_temp[2]  = OBSV.hgo4eemf.xEemf[0]       + increment_3[2];
    x_temp[3]  = OBSV.hgo4eemf.xEemf[1]       + increment_3[3];
    x_temp[4]  = OBSV.hgo4eemf.xOmg           + increment_3[4];
    x_temp[5]  = OBSV.hgo4eemf.xGammaOmg      + increment_3[5];
    x_temp[6]  = OBSV.hgo4eemf.xPhi[0]        + increment_3[6];
    x_temp[7]  = OBSV.hgo4eemf.xPhi[1]        + increment_3[7];
    x_temp[8]  = OBSV.hgo4eemf.xPhi[2]        + increment_3[8];
    x_temp[9]  = OBSV.hgo4eemf.xPhi[3]        + increment_3[9];

    // time instant t+hs
    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    rhf_func_hgoeemf( increment_4, p_x_temp+0, p_x_temp+2, 
                    *(p_x_temp+4), *(p_x_temp+5),
                      p_x_temp+6, hs );
    // \+=[^\n]*1\[(\d+)\][^\n]*2\[(\d+)\][^\n]*3\[(\d+)\][^\n]*4\[(\d+)\][^\n]*/ ([\d]+)
    // +=   (increment_1[$5] + 2*(increment_2[$5] + increment_3[$5]) + increment_4[$5])*0.166666666666667; // $5
    OBSV.hgo4eemf.xPsi[0]         +=   (increment_1[0] + 2*(increment_2[0] + increment_3[0]) + increment_4[0])*0.166666666666667; // 0
    OBSV.hgo4eemf.xPsi[1]         +=   (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667; // 1
    OBSV.hgo4eemf.xEemf[0]        +=   (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667; // 2
    OBSV.hgo4eemf.xEemf[1]        +=   (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667; // 3
    OBSV.hgo4eemf.xOmg            +=   (increment_1[4] + 2*(increment_2[4] + increment_3[4]) + increment_4[4])*0.166666666666667; // 4
    OBSV.hgo4eemf.xGammaOmg       +=   (increment_1[5] + 2*(increment_2[5] + increment_3[5]) + increment_4[5])*0.166666666666667; // 5
    OBSV.hgo4eemf.xPhi[0]         +=   (increment_1[6] + 2*(increment_2[6] + increment_3[6]) + increment_4[6])*0.166666666666667; // 6
    OBSV.hgo4eemf.xPhi[1]         +=   (increment_1[7] + 2*(increment_2[7] + increment_3[7]) + increment_4[7])*0.166666666666667; // 7
    OBSV.hgo4eemf.xPhi[2]         +=   (increment_1[8] + 2*(increment_2[8] + increment_3[8]) + increment_4[8])*0.166666666666667; // 8
    OBSV.hgo4eemf.xPhi[3]         +=   (increment_1[9] + 2*(increment_2[9] + increment_3[9]) + increment_4[9])*0.166666666666667; // 9

    /* Post-Observer */
    OBSV.hgo4eemf.theta_d = atan2( 
                             - OBSV.hgo4eemf.xEemf[0]*(sign(OBSV.hgo4eemf.xOmg)), 
                               OBSV.hgo4eemf.xEemf[1]*(sign(OBSV.hgo4eemf.xOmg))
                           );
}
/* Main Observer */
void Main_cjh_eemfhgo_farza09(){

    /* OBSERVATION */
    hgo4eemf_dedicated_rk4_solver(1*CL_TS);
}
void init_hgo4eemf(){
    OBSV.hgo4eemf.vartheta  = FARZA09_HGO_EEMF_VARTHETA;
    if(OBSV.hgo4eemf.vartheta==0.0){
        #if PC_SIMULATION
            printf("varthteta cannot be zero\n");
        #endif
    }else{
        OBSV.hgo4eemf.vartheta_inv = 1.0 / OBSV.hgo4eemf.vartheta;
    }
    OBSV.hgo4eemf.xGammaOmg = FARZA09_HGO_EEMF_GAMMA_OMEGA_INITIAL_VALUE;
}
#undef NS
#endif


/********************************************/
/* EEMF-Error-Dynamics-based-AO-Design
 ********************************************/
/* The 13th-order dynamic system */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_CJH_EEMF
#define NS 13
void rhf_func_eemfaod(REAL *increment_n, REAL *xPsi, REAL *xChi, REAL xOmg, 
                      REAL *xEta,
                      REAL *xVarsigma,
                      REAL *xUpsilon,
                      REAL *xZeta,    
                      REAL hs){
    // pointer to increment_n: state increment at n-th stage of RK4, where n=1,2,3,4.
    // *x???: pointer to state variables
    // x????: state variable
    // hs: step size of numerical integral

    // DEBUG
    // xOmg = (*CTRL).i->omg_elec;

    // dependent parameters
    #define MOTOR (*(*CTRL).motor)
    // MOTOR.Ld
    // MOTOR.DeltaL
    // MOTOR.R
    // OBSV.cjheemf.k1
    // OBSV.cjheemf.k2
    // OBSV.cjheemf.gamma_omg

    OBSV.cjheemf.output_error[0] = MOTOR.Ld*IS(0) - xPsi[0];
    OBSV.cjheemf.output_error[1] = MOTOR.Ld*IS(1) - xPsi[1];
    OBSV.cjheemf.effective_output_error[0] = OBSV.cjheemf.output_error[0] + xEta[0] - xUpsilon[0]*xOmg;
    OBSV.cjheemf.effective_output_error[1] = OBSV.cjheemf.output_error[1] + xEta[1] - xUpsilon[1]*xOmg;

    // f = \dot x = the time derivative
    REAL f[NS];
    REAL TwoLdMinusLq = MOTOR.DeltaL + MOTOR.Ld;
    // xPsi
    f[0] = US(0) - MOTOR.R* IS(0) + xChi[0] + TwoLdMinusLq*xOmg*-IS(1) + OBSV.cjheemf.k1*OBSV.cjheemf.output_error[0];
    f[1] = US(1) - MOTOR.R* IS(1) + xChi[1] + TwoLdMinusLq*xOmg* IS(0) + OBSV.cjheemf.k1*OBSV.cjheemf.output_error[1];
    // xChi
    f[2] = xOmg*xOmg * MOTOR.DeltaL*IS(0) - xOmg*-US(1) + xOmg*MOTOR.R*-IS(1) + OBSV.cjheemf.k2*OBSV.cjheemf.output_error[0];
    f[3] = xOmg*xOmg * MOTOR.DeltaL*IS(1) - xOmg* US(0) + xOmg*MOTOR.R* IS(0) + OBSV.cjheemf.k2*OBSV.cjheemf.output_error[1];
    // xOmg (recall xUpsilon is the filtered regreesor for speed)
    f[4] = OBSV.cjheemf.gamma_omg * (xUpsilon[0]*OBSV.cjheemf.effective_output_error[0] + xUpsilon[1]*OBSV.cjheemf.effective_output_error[1]);
    // f[4] = OBSV.cjheemf.gamma_omg * (xUpsilon[0]*-OBSV.cjheemf.effective_output_error[1] + xUpsilon[1]*OBSV.cjheemf.effective_output_error[0]);

    /* Auxiliary States*/
    // xEta 
    f[5] = - OBSV.cjheemf.k1*xEta[0] + xVarsigma[0] + xOmg*TwoLdMinusLq*-IS(1);
    f[6] = - OBSV.cjheemf.k1*xEta[1] + xVarsigma[1] + xOmg*TwoLdMinusLq* IS(0);
    // varsigma
    f[7] = - OBSV.cjheemf.k2*xVarsigma[0] + xOmg*((xOmg+xOmg)*MOTOR.DeltaL*IS(0) +US(1) + MOTOR.R*-IS(1));
    f[8] = - OBSV.cjheemf.k2*xVarsigma[1] + xOmg*((xOmg+xOmg)*MOTOR.DeltaL*IS(1) -US(0) + MOTOR.R* IS(0));
    //  xUpsilon
    f[9]  = - OBSV.cjheemf.k1*xUpsilon[0] + xZeta[0] + TwoLdMinusLq*-IS(1);
    f[10] = - OBSV.cjheemf.k1*xUpsilon[1] + xZeta[1] + TwoLdMinusLq* IS(0);
    // zeta
    f[11] = - OBSV.cjheemf.k2*xZeta[0] + ((xOmg+xOmg)*MOTOR.DeltaL*IS(0) +US(1) + MOTOR.R*-IS(1));
    f[12] = - OBSV.cjheemf.k2*xZeta[1] + ((xOmg+xOmg)*MOTOR.DeltaL*IS(1) -US(0) + MOTOR.R* IS(0));

    increment_n[ 0] = ( f[ 0] )*hs;
    increment_n[ 1] = ( f[ 1] )*hs;
    increment_n[ 2] = ( f[ 2] )*hs;
    increment_n[ 3] = ( f[ 3] )*hs;
    increment_n[ 4] = ( f[ 4] )*hs;
    increment_n[ 5] = ( f[ 5] )*hs;
    increment_n[ 6] = ( f[ 6] )*hs;
    increment_n[ 7] = ( f[ 7] )*hs;
    increment_n[ 8] = ( f[ 8] )*hs;
    increment_n[ 9] = ( f[ 9] )*hs;
    increment_n[10] = ( f[10] )*hs;
    increment_n[11] = ( f[11] )*hs;
    increment_n[12] = ( f[12] )*hs;
}
void eemf_ao_dedicated_rk4_solver(REAL hs){
    static REAL increment_1[NS];
    static REAL increment_2[NS];
    static REAL increment_3[NS];
    static REAL increment_4[NS];
    static REAL x_temp[NS];
    static REAL *p_x_temp=x_temp;

    /* Theoritically speaking, rhf_func should be time-varing like rhf_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhf_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhf_func_eemfaod( increment_1, OBSV.cjheemf.xPsi, OBSV.cjheemf.xChi, OBSV.cjheemf.xOmg,
                      OBSV.cjheemf.xEta,
                      OBSV.cjheemf.xVarsigma,
                      OBSV.cjheemf.xUpsilon,
                      OBSV.cjheemf.xZeta,
                      hs); 
    x_temp[ 0]  = OBSV.cjheemf.xPsi[0]        + increment_1[ 0]*0.5;
    x_temp[ 1]  = OBSV.cjheemf.xPsi[1]        + increment_1[ 1]*0.5;
    x_temp[ 2]  = OBSV.cjheemf.xChi[0]        + increment_1[ 2]*0.5;
    x_temp[ 3]  = OBSV.cjheemf.xChi[1]        + increment_1[ 3]*0.5;
    x_temp[ 4]  = OBSV.cjheemf.xOmg           + increment_1[ 4]*0.5;
    x_temp[ 5]  = OBSV.cjheemf.xEta[0]        + increment_1[ 5]*0.5;
    x_temp[ 6]  = OBSV.cjheemf.xEta[1]        + increment_1[ 6]*0.5;
    x_temp[ 7]  = OBSV.cjheemf.xVarsigma[0]   + increment_1[ 7]*0.5;
    x_temp[ 8]  = OBSV.cjheemf.xVarsigma[1]   + increment_1[ 8]*0.5;
    x_temp[ 9]  = OBSV.cjheemf.xUpsilon[0]    + increment_1[ 9]*0.5;
    x_temp[10]  = OBSV.cjheemf.xUpsilon[1]    + increment_1[10]*0.5;
    x_temp[11]  = OBSV.cjheemf.xZeta[0]       + increment_1[11]*0.5;
    x_temp[12]  = OBSV.cjheemf.xZeta[1]       + increment_1[12]*0.5;

    // time instant t+hs/2
    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    rhf_func_eemfaod( increment_2, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
                      p_x_temp+5, 
                      p_x_temp+7, 
                      p_x_temp+9, 
                      p_x_temp+11, 
                      hs );
    x_temp[ 0]  = OBSV.cjheemf.xPsi[0]        + increment_2[ 0]*0.5;
    x_temp[ 1]  = OBSV.cjheemf.xPsi[1]        + increment_2[ 1]*0.5;
    x_temp[ 2]  = OBSV.cjheemf.xChi[0]        + increment_2[ 2]*0.5;
    x_temp[ 3]  = OBSV.cjheemf.xChi[1]        + increment_2[ 3]*0.5;
    x_temp[ 4]  = OBSV.cjheemf.xOmg           + increment_2[ 4]*0.5;
    x_temp[ 5]  = OBSV.cjheemf.xEta[0]        + increment_2[ 5]*0.5;
    x_temp[ 6]  = OBSV.cjheemf.xEta[1]        + increment_2[ 6]*0.5;
    x_temp[ 7]  = OBSV.cjheemf.xVarsigma[0]   + increment_2[ 7]*0.5;
    x_temp[ 8]  = OBSV.cjheemf.xVarsigma[1]   + increment_2[ 8]*0.5;
    x_temp[ 9]  = OBSV.cjheemf.xUpsilon[0]    + increment_2[ 9]*0.5;
    x_temp[10]  = OBSV.cjheemf.xUpsilon[1]    + increment_2[10]*0.5;
    x_temp[11]  = OBSV.cjheemf.xZeta[0]       + increment_2[11]*0.5;
    x_temp[12]  = OBSV.cjheemf.xZeta[1]       + increment_2[12]*0.5;

    // time instant t+hs/2
    rhf_func_eemfaod( increment_3, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
                      p_x_temp+5, 
                      p_x_temp+7, 
                      p_x_temp+9, 
                      p_x_temp+11, 
                      hs );
    x_temp[ 0]  = OBSV.cjheemf.xPsi[0]        + increment_3[ 0];
    x_temp[ 1]  = OBSV.cjheemf.xPsi[1]        + increment_3[ 1];
    x_temp[ 2]  = OBSV.cjheemf.xChi[0]        + increment_3[ 2];
    x_temp[ 3]  = OBSV.cjheemf.xChi[1]        + increment_3[ 3];
    x_temp[ 4]  = OBSV.cjheemf.xOmg           + increment_3[ 4];
    x_temp[ 5]  = OBSV.cjheemf.xEta[0]        + increment_3[ 5];
    x_temp[ 6]  = OBSV.cjheemf.xEta[1]        + increment_3[ 6];
    x_temp[ 7]  = OBSV.cjheemf.xVarsigma[0]   + increment_3[ 7];
    x_temp[ 8]  = OBSV.cjheemf.xVarsigma[1]   + increment_3[ 8];
    x_temp[ 9]  = OBSV.cjheemf.xUpsilon[0]    + increment_3[ 9];
    x_temp[10]  = OBSV.cjheemf.xUpsilon[1]    + increment_3[10];
    x_temp[11]  = OBSV.cjheemf.xZeta[0]       + increment_3[11];
    x_temp[12]  = OBSV.cjheemf.xZeta[1]       + increment_3[12];

    // time instant t+hs
    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    rhf_func_eemfaod( increment_4, p_x_temp+0, p_x_temp+2, *(p_x_temp+4), 
                      p_x_temp+5, 
                      p_x_temp+7, 
                      p_x_temp+9, 
                      p_x_temp+11, 
                      hs );
    // \+=[^\n]*1\[(\d+)\][^\n]*2\[(\d+)\][^\n]*3\[(\d+)\][^\n]*4\[(\d+)\][^\n]*/ ([\d]+)
    // +=   (increment_1[$5] + 2*(increment_2[$5] + increment_3[$5]) + increment_4[$5])*0.166666666666667; // $5
    OBSV.cjheemf.xPsi[0]        +=   (increment_1[0] + 2*(increment_2[0] + increment_3[0]) + increment_4[0])*0.166666666666667; // 0
    OBSV.cjheemf.xPsi[1]        +=   (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667; // 1
    OBSV.cjheemf.xChi[0]        +=   (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667; // 2
    OBSV.cjheemf.xChi[1]        +=   (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667; // 3
    OBSV.cjheemf.xOmg           +=   (increment_1[4] + 2*(increment_2[4] + increment_3[4]) + increment_4[4])*0.166666666666667; // 4
    // if((*CTRL).timebase>1.6){
    //     printf("%g\n", OBSV.cjheemf.xOmg);;
    // }
    OBSV.cjheemf.xEta[0]        +=   (increment_1[5] + 2*(increment_2[5] + increment_3[5]) + increment_4[5])*0.166666666666667; // 5
    OBSV.cjheemf.xEta[1]        +=   (increment_1[6] + 2*(increment_2[6] + increment_3[6]) + increment_4[6])*0.166666666666667; // 6
    OBSV.cjheemf.xVarsigma[0]   +=   (increment_1[7] + 2*(increment_2[7] + increment_3[7]) + increment_4[7])*0.166666666666667; // 7
    OBSV.cjheemf.xVarsigma[1]   +=   (increment_1[8] + 2*(increment_2[8] + increment_3[8]) + increment_4[8])*0.166666666666667; // 8
    OBSV.cjheemf.xUpsilon[0]    +=   (increment_1[9] + 2*(increment_2[9] + increment_3[9]) + increment_4[9])*0.166666666666667; // 9
    OBSV.cjheemf.xUpsilon[1]    +=   (increment_1[10] + 2*(increment_2[10] + increment_3[10]) + increment_4[10])*0.166666666666667; // 10
    OBSV.cjheemf.xZeta[0]       +=   (increment_1[11] + 2*(increment_2[11] + increment_3[11]) + increment_4[11])*0.166666666666667; // 11
    OBSV.cjheemf.xZeta[1]       +=   (increment_1[12] + 2*(increment_2[12] + increment_3[12]) + increment_4[12])*0.166666666666667; // 12


    OBSV.cjheemf.xEemf[0] = - (*CTRL).motor->Ld * OBSV.cjheemf.xOmg *-IS_C(1) - OBSV.cjheemf.xChi[0]; 
    OBSV.cjheemf.xEemf[1] = - (*CTRL).motor->Ld * OBSV.cjheemf.xOmg * IS_C(0) - OBSV.cjheemf.xChi[1]; 
    OBSV.cjheemf.theta_d = atan2( 
                             - OBSV.cjheemf.xEemf[0]*(sign(OBSV.cjheemf.xOmg)), 
                               OBSV.cjheemf.xEemf[1]*(sign(OBSV.cjheemf.xOmg))
                           );
}
/* Main Observer */
void Main_cjh_eemfao(){

    /* OBSERVATION */
    eemf_ao_dedicated_rk4_solver(1*CL_TS);
}
void init_cjheemf(){
    OBSV.cjheemf.k1        = CJH_EEMF_K1;
    OBSV.cjheemf.k2        = CJH_EEMF_K2;
    OBSV.cjheemf.gamma_omg = CJH_EEMF_GAMMA_OMEGA;
}
#undef NS
#endif


/********************************************/
/* Harnefor 2006
 ********************************************/
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Harnefors_2006
void init_harnefors(){
    // OBSV.harnefors.theta_d = 0.0;
    // OBSV.harnefors.omg_elec = 0.0;

    OBSV.harnefors.svf_p0 = SVF_POLE_0_VALUE;
    OBSV.harnefors.xSVF_curr[0] = 0.0;
    OBSV.harnefors.xSVF_curr[1] = 0.0;
    OBSV.harnefors.xSVF_prev[0] = 0.0;
    OBSV.harnefors.xSVF_prev[1] = 0.0;
    OBSV.harnefors.is_dq[0] = 0.0;
    OBSV.harnefors.is_dq[1] = 0.0;
    OBSV.harnefors.is_dq_curr[0] = 0.0;
    OBSV.harnefors.is_dq_curr[1] = 0.0;
    OBSV.harnefors.is_dq_prev[0] = 0.0;
    OBSV.harnefors.is_dq_prev[1] = 0.0;
    OBSV.harnefors.pis_dq[0] = 0.0;
    OBSV.harnefors.pis_dq[1] = 0.0;    
}
void state_variable_filter(REAL hs){
    // output += SVF_POLE_0 * (input - last_output) * hs;
    // last_output = output;
    // if input == output:
    //     derivative of input = SVF_POLE_0 * (input - last_output)

    /* State Variable Filter for ob.pis[.] Computed by Standalone RK4 */
    REAL xxsvf1[2];
    REAL xxsvf2[2];
    REAL xxsvf3[2];
    REAL xxsvf4[2];
    REAL xsvf_temp[2];
    // step 1 @ t
    xxsvf1[0] = SVF_POLE_0 * (IDQ_P(0) - SVF_P(0)) * hs;
    xxsvf1[1] = SVF_POLE_0 * (IDQ_P(1) - SVF_P(1)) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（上一步的状态的值，其他时变的已知量在t时刻的值） * 数值积分的步长；
    xsvf_temp[0] = SVF_P(0) + xxsvf1[0]*0.5;
    xsvf_temp[1] = SVF_P(1) + xxsvf1[1]*0.5;
    // 临时的状态变量 = 上一步的状态变量 + step1增量*0.5；

    // step 2 @ t+hs/2
    IDQ(0) = 0.5*(IDQ_P(0)+IDQ_C(0));
    IDQ(1) = 0.5*(IDQ_P(1)+IDQ_C(1));
    xxsvf2[0] = SVF_POLE_0 * (IDQ(0) - xsvf_temp[0]) * hs;
    xxsvf2[1] = SVF_POLE_0 * (IDQ(1) - xsvf_temp[1]) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（临时的状态变量的值，其他时变的已知量在t+hs/2时刻的值） * 数值积分的步长；
    xsvf_temp[0] = SVF_P(0) + xxsvf2[0]*0.5;
    xsvf_temp[1] = SVF_P(1) + xxsvf2[1]*0.5;
    // 临时的状态变量 = 上一步的状态变量 + step2增量*0.5；

    // step 3 @ t+hs/2
    xxsvf3[0] = SVF_POLE_0 * (IDQ(0) - xsvf_temp[0]) * hs;
    xxsvf3[1] = SVF_POLE_0 * (IDQ(1) - xsvf_temp[1]) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（临时的状态变量的值，其他时变的已知量在t+hs/2时刻的值） * 数值积分的步长；
    xsvf_temp[0] = SVF_P(0) + xxsvf3[0];
    xsvf_temp[1] = SVF_P(1) + xxsvf3[1];
    // 临时的状态变量 = 上一步的状态变量 + step3增量*1.0；

    // step 4 @ t+hs
    xxsvf4[0] = SVF_POLE_0 * (IDQ_C(0) - xsvf_temp[0]) * hs;
    xxsvf4[1] = SVF_POLE_0 * (IDQ_C(1) - xsvf_temp[1]) * hs;
    // 新的状态变量的增量 = 该状态变量的导数（临时的状态变量的值，其他时变的已知量在t时刻的值） * 数值积分的步长；
    SVF_C(0) += (xxsvf1[0] + 2*(xxsvf2[0] + xxsvf3[0]) + xxsvf4[0])*0.166666666666667; // 0
    SVF_C(1) += (xxsvf1[1] + 2*(xxsvf2[1] + xxsvf3[1]) + xxsvf4[1])*0.166666666666667; // 1    
    // 数值积分的结果 = （step1增量+2*step2增量+2*step3增量+step4的增量）/6

    PIDQ(0) = SVF_POLE_0 * (IDQ_C(0) - SVF_C(0));
    PIDQ(1) = SVF_POLE_0 * (IDQ_C(1) - SVF_C(1));
}
// Harnefors 2003: mu=-1, lambda=sqrt(2)
// Harnefors 2006
#define LAMBDA 2
void Main_harnefors_scvm(){

    // 这一组参数很适合伺尔沃的400W，也可以用于100W和200W（delta=6.5，VLBW=50Hz）
        // #define CJH_TUNING_A  25 // low voltage servo motor (<48 Vdc)
        // #define CJH_TUNING_B  1  // low voltage servo motor (<48 Vdc)
        // #define CJH_TUNING_C 1
    // 为NTU的伺服调试的参数（delta=3，VLBW=40Hz）
        #define CJH_TUNING_A  1 // low voltage servo motor (<48 Vdc)
        #define CJH_TUNING_B  1  // low voltage servo motor (<48 Vdc)
        #define CJH_TUNING_C 0.2 // [0.2， 0.5]
    // 可调参数壹
    REAL lambda_s = CJH_TUNING_C * LAMBDA * sign(OBSV.harnefors.omg_elec);
    // 可调参数贰
    REAL alpha_bw_lpf = CJH_TUNING_A*0.1*(1500*RPM_2_ELEC_RAD_PER_SEC) + CJH_TUNING_B*2*LAMBDA*fabsf(OBSV.harnefors.omg_elec);


    // 一阶差分计算DQ电流的导数
    static REAL last_id = 0.0;
    static REAL last_iq = 0.0;
    // #define D_AXIS_CURRENT (*CTRL).id_cmd
    // #define Q_AXIS_CURRENT (*CTRL).iq_cmd
    // OBSV.harnefors.deriv_id = ((*CTRL).id_cmd - last_id) * CL_TS_INVERSE;
    // OBSV.harnefors.deriv_iq = ((*CTRL).iq_cmd - last_iq) * CL_TS_INVERSE;
    // last_id = (*CTRL).id_cmd;
    // last_iq = (*CTRL).iq_cmd;
    #define D_AXIS_CURRENT (*CTRL).i->iDQ[0]
    #define Q_AXIS_CURRENT (*CTRL).i->iDQ[1]
    OBSV.harnefors.deriv_id = (D_AXIS_CURRENT - last_id) * CL_TS_INVERSE;
    OBSV.harnefors.deriv_iq = (Q_AXIS_CURRENT - last_iq) * CL_TS_INVERSE;
    last_id = D_AXIS_CURRENT;
    last_iq = Q_AXIS_CURRENT;

    // 用SVF计算DQ电流的导数
    IDQ_C(0) = D_AXIS_CURRENT;
    IDQ_C(1) = Q_AXIS_CURRENT;
    state_variable_filter(CL_TS);
    IDQ_P(0) = IDQ_C(0); // used in SVF 
    IDQ_P(1) = IDQ_C(1); // used in SVF 
    SVF_P(0) = SVF_C(0);
    SVF_P(1) = SVF_C(1);
    #define DERIV_ID PIDQ(0)
    #define DERIV_IQ PIDQ(1)

    #define UD_CMD (*CTRL).o->cmd_uDQ[0]
    #define UQ_CMD (*CTRL).o->cmd_uDQ[1]
    #define MOTOR  (*(*CTRL).motor)

    // // 计算反电势（考虑dq电流导数）
    // #define BOOL_COMPENSATE_PIDQ 1
    // REAL d_axis_emf = UD_CMD - MOTOR.R*D_AXIS_CURRENT + OBSV.harnefors.omg_elec*MOTOR.Lq*Q_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Ld*DERIV_ID; // eemf
    // REAL q_axis_emf = UQ_CMD - MOTOR.R*Q_AXIS_CURRENT - OBSV.harnefors.omg_elec*MOTOR.Ld*D_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Lq*DERIV_IQ; // eemf

    // 计算反电势（忽略dq电流导数）
    #define BOOL_COMPENSATE_PIDQ 0
    REAL d_axis_emf = UD_CMD - MOTOR.R*D_AXIS_CURRENT + OBSV.harnefors.omg_elec*MOTOR.Lq*Q_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Ld*DERIV_ID; // eemf
    REAL q_axis_emf = UQ_CMD - MOTOR.R*Q_AXIS_CURRENT - OBSV.harnefors.omg_elec*MOTOR.Ld*D_AXIS_CURRENT - BOOL_COMPENSATE_PIDQ*MOTOR.Lq*DERIV_IQ; // eemf

    // 数值积分获得转速和转子位置
        // Note it is bad habit to write numerical integration explictly like this. The states on the right may be accencidentally modified on the run.
    #define KE_MISMATCH 1.0 // 0.7
    OBSV.harnefors.theta_d  += CL_TS * OBSV.harnefors.omg_elec;
    OBSV.harnefors.omg_elec += CL_TS * alpha_bw_lpf * ( (q_axis_emf - lambda_s*d_axis_emf)/(MOTOR.KE*KE_MISMATCH+(MOTOR.Ld-MOTOR.Lq)*D_AXIS_CURRENT) - OBSV.harnefors.omg_elec );

    // 转子位置周期限幅
    if(OBSV.harnefors.theta_d>M_PI) OBSV.harnefors.theta_d-=2*M_PI;
    if(OBSV.harnefors.theta_d<-M_PI) OBSV.harnefors.theta_d+=2*M_PI;
}
#endif


/********************************************/
/* Qiao.Xia 2013 emfSMO with Sigmoid (Also Hongryel Kim 2011) as Nonlinear
 ********************************************/
REAL sm_sigmoid(REAL x, REAL a){
    return 2.0 / (1.0 + exp(-a*x)) - 1.0;
}
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Qiao_Xia
void init_QiaoXia2013(){
    qiaoxia.smo_gain      = 0.0; // time varying gain
    qiaoxia.sigmoid_coeff = QIAO_XIA_SIGMOID_COEFF;   // 17
    qiaoxia.adapt_gain    = QIAO_XIA_ADAPT_GAIN; // this is fixed to be 1 in the paper
    qiaoxia.mras_gain     = QIAO_XIA_MRAS_GAIN;
}
void rhf_QiaoXia2013_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    REAL xIa   = x[0];
    REAL xIb   = x[1];
    REAL xEmfa = x[2];
    REAL xEmfb = x[3];
    REAL xOmg  = x[4];

    /* Know Signals */
    // #define MOTOR (*(*CTRL).motor)

    /* Output Error = \tilde i_q (scalar) */
    qiaoxia.output_error[0] = IS(0) - xIa;
    qiaoxia.output_error[1] = IS(1) - xIb;

    /* Sliding Mode Switching Term*/
    qiaoxia.xEmf_raw[0] = qiaoxia.smo_gain * sm_sigmoid(qiaoxia.output_error[0], qiaoxia.sigmoid_coeff);
    qiaoxia.xEmf_raw[1] = qiaoxia.smo_gain * sm_sigmoid(qiaoxia.output_error[1], qiaoxia.sigmoid_coeff);
    // qiaoxia.xEmf_raw[0] = qiaoxia.smo_gain * sign(qiaoxia.output_error[0]);
    // qiaoxia.xEmf_raw[1] = qiaoxia.smo_gain * sign(qiaoxia.output_error[1]);

    /* State Observer */
    // xIab
    fx[0] = MOTOR.Lq_inv * (US(0) - MOTOR.R * xIa + qiaoxia.xEmf_raw[0]);
    fx[1] = MOTOR.Lq_inv * (US(1) - MOTOR.R * xIb + qiaoxia.xEmf_raw[1]);
    // xEmf
    #define OMG_USED xOmg //(*CTRL).i->omg_elec // 
    fx[2] = -OMG_USED*xEmfb + qiaoxia.mras_gain * (qiaoxia.xEmf_raw[0] - xEmfa);
    fx[3] =  OMG_USED*xEmfa + qiaoxia.mras_gain * (qiaoxia.xEmf_raw[1] - xEmfb);
    // xOmg
    fx[4] = qiaoxia.adapt_gain * (qiaoxia.xEmf_raw[0] - xEmfa)*-xEmfb + (qiaoxia.xEmf_raw[1] - xEmfb)*xEmfa;
}
void Main_QiaoXia2013_emfSMO(){

    /* Time-varying gains */
    qiaoxia.smo_gain = QIAO_XIA_SMO_GAIN * fabsf((*CTRL).i->cmd_varOmega*MOTOR.npp) * MOTOR.KE;
    // qiaoxia.smo_gain = QIAO_XIA_SMO_GAIN * 70 * MOTOR.KE;

    general_5states_rk4_solver(&rhf_QiaoXia2013_Dynamics, (*CTRL).timebase, qiaoxia.x, CL_TS);

    /* Unpack x (for the best) */
    qiaoxia.xIab[0] = qiaoxia.x[0];
    qiaoxia.xIab[1] = qiaoxia.x[1];
    qiaoxia.xEmf[0] = qiaoxia.x[2];
    qiaoxia.xEmf[1] = qiaoxia.x[3];
    qiaoxia.xOmg    = qiaoxia.x[4];
    // REAL temp = -atan2( qiaoxia.xEmf[0]*-sign((*CTRL).i->cmd_varOmega*MECH_RAD_PER_SEC_2_RPM),   // qiaoxia.xOmg
    //                     qiaoxia.xEmf[1]*-sign((*CTRL).i->cmd_varOmega*MECH_RAD_PER_SEC_2_RPM) ); // qiaoxia.xOmg
    REAL temp = -atan2( qiaoxia.xEmf[0]*sign(-qiaoxia.xOmg),   // qiaoxia.xOmg
                        qiaoxia.xEmf[1]*sign(-qiaoxia.xOmg) ); // qiaoxia.xOmg
    // REAL temp = -atan2( qiaoxia.xEmf[0],   // qiaoxia.xOmg
                        // qiaoxia.xEmf[1] ); // qiaoxia.xOmg
    qiaoxia.theta_d = temp;
    // if(temp>0){
    //     qiaoxia.theta_d = temp - M_PI;
    // }else{
    //     qiaoxia.theta_d = temp + M_PI;
    // }
    // if(qiaoxia.theta_d>M_PI){
    //     qiaoxia.theta_d -= M_PI;
    // }else if(qiaoxia.theta_d<-M_PI){
    //     qiaoxia.theta_d += M_PI;
    // }

    /* Post-observer calculations */
    /* Selecting Signals From Block Diagram */    
}
#endif



/********************************************/
/* SongChi.LongyaXu 2009 emfSMO 
 */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Chi_Xu
void init_ChiXu2009(){
    /*实验中Chi.Xu的状态x在调参过程中经常变成+inf，所以这里初始化一下。*/
    OBSV.chixu.x[0] = 0.0;
    OBSV.chixu.x[1] = 0.0;
    OBSV.chixu.x[2] = 0.0;
    OBSV.chixu.x[3] = 0.0;

    OBSV.chixu.smo_gain      = 400.0; // CHI_XU_USE_CONSTANT_SMO_GAIN
    OBSV.chixu.sigmoid_coeff = CHI_XU_SIGMOID_COEFF;   // 17
    OBSV.chixu.PLL_KP        = CHI_XU_SPEED_PLL_KP;
    OBSV.chixu.PLL_KI        = CHI_XU_SPEED_PLL_KI;
    OBSV.chixu.ell4xZeq      = -0.5; // -0.5 for low speed and 1.0 for high speed
    OBSV.chixu.omega_lpf_4_xZeq_const_part = CHI_XU_LPF_4_ZEQ;
    OBSV.chixu.omega_lpf_4_xZeq = OBSV.chixu.omega_lpf_4_xZeq_const_part;

    OBSV.chixu.smo_gain_scale = CHI_XU_SMO_GAIN_SCALE;
}
REAL get_theta_d_from_xZeq(REAL xZeq_a, REAL xZeq_b){
    REAL temp = -atan2( xZeq_a, xZeq_b) + M_PI*0.5;
    if(temp>0){
        temp = temp - M_PI;
    }else{
        temp = temp + M_PI;
    }
    return temp;
}
void rhf_ChiXu2009_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    #define xIab(X) x[0+X]
    #define xZeq(X) x[2+X]
    #define xOmg     x[4]
    #define xTheta_d x[5]

    /* Know Signals */
    // #define MOTOR (*(*CTRL).motor)

    /* Output Error = \tilde i_q (scalar) */
    OBSV.chixu.output_error[0] = IS(0) - xIab(0);
    OBSV.chixu.output_error[1] = IS(1) - xIab(1);

    /* Sliding Mode Switching Term*/
    // TODO: need to change sigmoid to saturation function 
    OBSV.chixu.xEmf_raw[0] = OBSV.chixu.smo_gain * sm_sigmoid(OBSV.chixu.output_error[0], OBSV.chixu.sigmoid_coeff);
    OBSV.chixu.xEmf_raw[1] = OBSV.chixu.smo_gain * sm_sigmoid(OBSV.chixu.output_error[1], OBSV.chixu.sigmoid_coeff);

    // xZeq
    fx[2] = OBSV.chixu.omega_lpf_4_xZeq * (OBSV.chixu.xEmf_raw[0] - xZeq(0));
    fx[3] = OBSV.chixu.omega_lpf_4_xZeq * (OBSV.chixu.xEmf_raw[1] - xZeq(1));

    //
    OBSV.chixu.theta_d = get_theta_d_from_xZeq(OBSV.chixu.xZeq[0], OBSV.chixu.xZeq[1]);

    /* State Observer */
    // xIab
    fx[0] = MOTOR.Lq_inv * (US(0) - MOTOR.R * xIab(0) + OBSV.chixu.ell4xZeq * xZeq(0) + OBSV.chixu.xEmf_raw[0]);
    fx[1] = MOTOR.Lq_inv * (US(1) - MOTOR.R * xIab(1) + OBSV.chixu.ell4xZeq * xZeq(1) + OBSV.chixu.xEmf_raw[1]);

    // PLL for xOmg
    if(TRUE){
        // xOmg
        fx[4] = (OBSV.chixu.PLL_KI        * sin(OBSV.chixu.theta_d - xTheta_d));
        // xTheta_d
        fx[5] = (xOmg + OBSV.chixu.PLL_KP * sin(OBSV.chixu.theta_d - xTheta_d));
    }else{
        // xOmg
        fx[4] = (OBSV.chixu.PLL_KI        * difference_between_two_angles(OBSV.chixu.theta_d, xTheta_d));
        // xTheta_d
        fx[5] = (xOmg + OBSV.chixu.PLL_KP * difference_between_two_angles(OBSV.chixu.theta_d, xTheta_d));
    }


    #undef xIab
    #undef xZeq
    #undef xOmg
    #undef xTheta_d
}
void Main_ChiXu2009_emfSMO(){

    // #define OMEGA_SYNC_USED (*CTRL).i->cmd_varOmega*MOTOR.npp
    // #define OMEGA_SYNC_USED (*CTRL).i->omg_elec
    #define OMEGA_SYNC_USED OBSV.chixu.xOmg

    #if CHI_XU_USE_CONSTANT_SMO_GAIN == FALSE
        /* Time-varying gains */
        OBSV.chixu.smo_gain = OBSV.chixu.smo_gain_scale * fabsf(OMEGA_SYNC_USED) * MOTOR.KE; // 根据Qiao.Xia 2013的稳定性证明增益要比反电势大。
        if(fabsf(OMEGA_SYNC_USED)<5){ // [rad/s]
            // 转速太低以后，就不要再减小滑模增益了
            OBSV.chixu.smo_gain = OBSV.chixu.smo_gain_scale * 5 * MOTOR.KE;
        }
    #endif

    /* ell for xZeq */
    if(fabsf((*CTRL).i->cmd_varOmega*MECH_RAD_PER_SEC_2_RPM)>180){
        OBSV.chixu.ell4xZeq = 1.0;
    }else{
        OBSV.chixu.ell4xZeq = -0.5;
    }

    /* For the sake of simplicity, a first-order LPF is used in (3).
        It is noticed that the time constant τc of LPF should be designed
        properly according to the fundamental frequency of phase
        currents of PMSM */
    #if CHI_XU_USE_CONSTANT_LPF_POLE == FALSE
        OBSV.chixu.omega_lpf_4_xZeq = 0.05*fabsf(OMEGA_SYNC_USED) + OBSV.chixu.omega_lpf_4_xZeq_const_part;
    #endif

    general_6states_rk4_solver(&rhf_ChiXu2009_Dynamics, (*CTRL).timebase, OBSV.chixu.x, CL_TS);

    /* Unpack x (for the best) */
    OBSV.chixu.xIab[0]  = OBSV.chixu.x[0];
    OBSV.chixu.xIab[1]  = OBSV.chixu.x[1];
    OBSV.chixu.xZeq[0]  = OBSV.chixu.x[2];
    OBSV.chixu.xZeq[1]  = OBSV.chixu.x[3];
    OBSV.chixu.xOmg     = OBSV.chixu.x[4];
    if(OBSV.chixu.x[5]>M_PI){
        OBSV.chixu.x[5] -= 2*M_PI;
    }else if(OBSV.chixu.x[5]<-M_PI){
        OBSV.chixu.x[5] += 2*M_PI;
    }
    OBSV.chixu.xTheta_d = OBSV.chixu.x[5];
    if(FALSE){
        // use xZeq
        OBSV.chixu.theta_d = get_theta_d_from_xZeq(OBSV.chixu.xZeq[0], OBSV.chixu.xZeq[1]);
    }else{
        // use PLL output
        OBSV.chixu.theta_d = OBSV.chixu.xTheta_d; 
    }

    /* Post-observer calculations */
    /* Selecting Signals From Block Diagram */
}
#endif


/********************************************/
/* Park.Sul 2014 FADO in replace of CM 
 */
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_Park_Sul
void init_parksul2014(){
    parksul.T2S_1_KP = PARK_SUL_T2S_1_KP;
    parksul.T2S_1_KI = PARK_SUL_T2S_1_KI;
    parksul.T2S_2_KP = PARK_SUL_T2S_2_KP;
    parksul.T2S_2_KI = PARK_SUL_T2S_2_KI;

    parksul.CM_KP = PARK_SUL_CM_KP;
    parksul.CM_KI = PARK_SUL_CM_KI;

    parksul.limiter_KE = 1.15 * MOTOR.KE;

    parksul.x[0] = MOTOR.KE;
    parksul.x[2] = MOTOR.KE;
    parksul.xPsi2[0] = MOTOR.KE;
}
void rhf_parksul2014_Dynamics(REAL t, REAL *x, REAL *fx){

    /* Unpack States */
    #define xPsi1(X)    x[0+X]
    #define xHatPsi1(X) x[2+X]
    #define xD(X)       x[4+X]
    #define xT2S_1(X)   x[6+X]
    #define xT2S_2(X)   x[8+X]
    // #define xTheta_d    x[7]
    // #define xOmg        x[8]

    /* Know Signals */
    parksul.omega_f = x[8]; // In addition, it is evident in Fig. 3 that all the integrations in the proposed observer are stopped when ω f is zero, which coincides with the natural property of stator flux at standstill.
    // parksul.omega_f = (*CTRL).i->omg_elec;

    /* Pre-calculation */
    parksul.internal_error[0] = xPsi1(0) - xHatPsi1(0);
    parksul.internal_error[1] = xPsi1(1) - xHatPsi1(1);

    parksul.xPsi2[0] = xPsi1(0) - xD(0) - MOTOR.Lq*IS(0);
    parksul.xPsi2[1] = xPsi1(1) - xD(1) - MOTOR.Lq*IS(1);
    parksul.theta_d = atan2(parksul.xPsi2[1], parksul.xPsi2[0]);

    /* The amplitude limiter */
    parksul.xPsi2_Amplitude = sqrt(parksul.xPsi2[0]*parksul.xPsi2[0] + parksul.xPsi2[1]*parksul.xPsi2[1]);
    if(parksul.xPsi2_Amplitude > parksul.limiter_KE){
        REAL temp = 1.0/parksul.xPsi2_Amplitude;
        parksul.xPsi2_Limited[0] = parksul.limiter_KE*parksul.xPsi2[0]*temp;
        parksul.xPsi2_Limited[1] = parksul.limiter_KE*parksul.xPsi2[1]*temp;
    }else{
        parksul.xPsi2_Limited[0] = parksul.xPsi2[0];
        parksul.xPsi2_Limited[1] = parksul.xPsi2[1];
    }

    /* Modified: Add output error (CM correction) to Park.Sul2014 */
    #define TRY_CM_VM_FUSION_IN_REPLACE_OF_K_DF
    #ifdef  TRY_CM_VM_FUSION_IN_REPLACE_OF_K_DF
        /* State: Cosine and Sine of the flux angle */

        // 我们已经有角度了，直接求三角函数就可以了
        // REAL temp, cosT, sinT;
        // if(parksul.active_flux_ampl!=0){
        //     temp = 1.0/parksul.xPsi2_Amplitude;
        //     cosT = parksul.xPsi2[0] * temp;
        //     sinT = parksul.xPsi2[1] * temp;
        // }else{
        //     cosT = 1.0;
        //     sinT = 0.0;
        // }

        REAL cosT = cos(parksul.theta_d);
        REAL sinT = sin(parksul.theta_d);

        /* Input: Current at dq frame and KActive */
        REAL iDQ_at_current_step[2];
        iDQ_at_current_step[0] = AB2M(IS(0), IS(1), cosT, sinT);
        iDQ_at_current_step[1] = AB2T(IS(0), IS(1), cosT, sinT);
        REAL KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * iDQ_at_current_step[0];

        /* Output: Flux mismatch to KActive (CM) */
        parksul.output_error[0] = KActive*cosT - parksul.xPsi2[0];
        parksul.output_error[1] = KActive*sinT - parksul.xPsi2[1];
    #endif

    /* Flux Estimator */
    REAL emf[2];
    emf[0] = US(0) - MOTOR.R*IS(0);
    emf[1] = US(1) - MOTOR.R*IS(1);
    // xPsi1
    fx[0] = emf[0] \
        /*k_df*/- parksul.k_df*xD(0) \
        /*k_af*/- parksul.k_af*(parksul.xPsi2[0] - parksul.xPsi2_Limited[0]) \
        /*CM-I*/+ x[10] \
        /*CM-P*/+ parksul.CM_KP * parksul.output_error[0];
    fx[10] =      parksul.CM_KI * parksul.output_error[0];
    fx[1] = emf[1] \
        /*k_df*/- parksul.k_df*xD(1) \
        /*k_af*/- parksul.k_af*(parksul.xPsi2[1] - parksul.xPsi2_Limited[1]) \
        /*CM-I*/+ x[11] \
        /*CM-P*/+ parksul.CM_KP * parksul.output_error[1];
    fx[11] =      parksul.CM_KI * parksul.output_error[1];

    /* State: disturbance/offset estimated by an integrator */
    // xHatPsi1
    fx[2] = -parksul.omega_f * (xPsi1(1)-xD(1)) + 2*fabsf(parksul.omega_f)*parksul.internal_error[0];
    fx[3] = +parksul.omega_f * (xPsi1(0)-xD(0)) + 2*fabsf(parksul.omega_f)*parksul.internal_error[1];
    // xD
    fx[4] = -parksul.omega_f * parksul.internal_error[1];
    fx[5] = +parksul.omega_f * parksul.internal_error[0];

    /* T2S-1 for position */
    parksul.t2s_1_sin_error = sin(parksul.theta_d - x[7]);
    fx[6] =        parksul.T2S_1_KI * parksul.t2s_1_sin_error;
    fx[7] = x[6] + parksul.T2S_1_KP * parksul.t2s_1_sin_error;

    /* T2S-2 for speed */
    parksul.t2s_2_sin_error = sin(parksul.theta_d - x[9]);
    fx[8] =        parksul.T2S_2_KI * parksul.t2s_2_sin_error;
    fx[9] = x[8] + parksul.T2S_2_KP * parksul.t2s_2_sin_error;

    #undef xPsi1
    #undef xHatPsi1
    #undef xD
    #undef xT2S_1
    #undef xT2S_2
}
void Main_parksul2014_FADO(){

    /* Time-varying gains */
    if(fabsf(parksul.xOmg)        *MOTOR.npp_inv*ONE_OVER_2PI<1.5){ // [Hz]
    // if(fabsf((*CTRL).i->cmd_varOmega*MOTOR.npp)*MOTOR.npp_inv*ONE_OVER_2PI<1.5){ // [Hz]
        parksul.k_df = 0.0;
        parksul.k_af = 2*M_PI*100;
        parksul.limiter_Flag = TRUE;
    }else{
        parksul.k_df = 0.50;
        parksul.k_af = 0.0;
        parksul.limiter_Flag = FALSE;
    }

    // debug: turn off k_df/k_af
    // parksul.k_df = 0.0; // overwrite k_df as zero
    // parksul.k_af = 0.0;
    parksul.CM_KP = 0.0; //200.0;  // overwrite KP as zero
    parksul.CM_KI = 0.0; //0.0;  // overwrite KI as zero

    general_10states_rk4_solver(&rhf_parksul2014_Dynamics, (*CTRL).timebase, parksul.x, CL_TS);

    /* Unpack x (for the best) */
    if(parksul.x[7]>M_PI){
        parksul.x[7]     -= 2*M_PI;
    }else if(parksul.x[7]<-M_PI){
        parksul.x[7]     += 2*M_PI;
    }
    if(parksul.x[9]>M_PI){
        parksul.x[9]     -= 2*M_PI;
    }else if(parksul.x[9]<-M_PI){
        parksul.x[9]     += 2*M_PI;
    }
    parksul.xPsi1[0]     = parksul.x[0];
    parksul.xPsi1[1]     = parksul.x[1];
    parksul.xHatPsi1[0]  = parksul.x[2]; // aux
    parksul.xHatPsi1[1]  = parksul.x[3]; // aux
    parksul.xD[0]        = parksul.x[4];
    parksul.xD[1]        = parksul.x[5];
    parksul.xT2S_1[0]    = parksul.x[6]; // speed
    parksul.xT2S_1[1]    = parksul.x[7]; // position
    parksul.xT2S_2[0]    = parksul.x[8]; // speed
    parksul.xT2S_2[1]    = parksul.x[9]; // position
    parksul.xCM_state[0] = parksul.x[10]; // CM-correction-alpha
    parksul.xCM_state[1] = parksul.x[11]; // CM-correction-beta

    parksul.xTheta_d     = parksul.x[7]; // parksul2014 uses this instead of atan2(parksul.xPsi2[1], parksul.xPsi2[0]);
    parksul.xOmg         = parksul.x[8];

    /* Post-observer calculations @(20) */
    parksul.xPsi2[0] = parksul.xPsi1[0] - parksul.xD[0] - MOTOR.Lq*IS_C(0);
    parksul.xPsi2[1] = parksul.xPsi1[1] - parksul.xD[1] - MOTOR.Lq*IS_C(1);
    parksul.xPsi2_Amplitude = sqrt(parksul.xPsi2[0]*parksul.xPsi2[0] + parksul.xPsi2[1]*parksul.xPsi2[1]);
    if(TRUE){
        parksul.theta_d = atan2(parksul.xPsi2[1], parksul.xPsi2[0]);
    }else{
        // Park Sul 2014 uses this
        parksul.theta_d = parksul.xT2S_1[1]; // x[7]
    }
}
#endif
/********************************************/
/*SYNIFO*/
#if PC_SIMULATION || SELECT_ALGORITHM == ALG_SynIFO
    void init_SynIFO(){
        
    }
#endif
/* DOB Stationary Voltage that is valid for SPMSM only */
// void stationary_voltage_DOB(){
//     // (*CTRL).i->iab[0]
//     // (*CTRL).i->iab[1]

//     (*CTRL).inv->iab_lpf[0] = _lpf((*CTRL).i->iab[0], (*CTRL).inv->iab_lpf[0], (*CTRL).inv->filter_pole);
//     (*CTRL).inv->iab_lpf[1] = _lpf((*CTRL).i->iab[1], (*CTRL).inv->iab_lpf[1], (*CTRL).inv->filter_pole);

//     (*CTRL).inv->iab_hpf[0] = (*CTRL).i->iab[0] - (*CTRL).inv->iab_lpf[0];
//     (*CTRL).inv->iab_hpf[1] = (*CTRL).i->iab[1] - (*CTRL).inv->iab_lpf[1];

//     (*CTRL).inv->uab_DOB[0] = - (*CTRL).motor->R * (*CTRL).inv->iab_lpf[0] - (*CTRL).motor->Ld * (*CTRL).inv->iab_lpf[0];
//     (*CTRL).inv->uab_DOB[1] = - (*CTRL).motor->R * (*CTRL).inv->iab_lpf[1] - (*CTRL).motor->Ld * (*CTRL).inv->iab_lpf[1];

//     // add back emf
//     // (*CTRL).inv->uab_DOB[0] += (*CTRL).i->omg_elec * (*CTRL).motor->KE * -sin((*CTRL).i->theta_d_elec);
//     // (*CTRL).inv->uab_DOB[1] += (*CTRL).i->omg_elec * (*CTRL).motor->KE *  cos((*CTRL).i->theta_d_elec);
// }

#if ALG_AKT_SPEED_EST_AND_RS_ID
    void init_ake_Speed_Est_and_RS_ID(){
        double f_plus = 2e4;
        akt.lambda1 = 1.5 * sqrt(f_plus);
        akt.lambda2 = 1.1 * f_plus;
        // printf("STA for Akatsu00: %g, %g\n", akt.lambda1, akt.lambda2);
        akt.sta_state[0] = 0;
        akt.sta_state[1] = 0;
        akt.the_y_lpf = 0.0;

        akt.emf_lpf[0] = 0;
        akt.emf_lpf[1] = 0;
        akt.is_hpf[0] = 0;
        akt.is_hpf[1] = 0;
        akt.is_hpf_temp[0] = 0;
        akt.is_hpf_temp[1] = 0;
        akt.ireq_lpf[0] = 0;
        akt.ireq_lpf[1] = 0;
        akt.ireq_cal[0] = 0;
        akt.ireq_cal[1] = 0;
        akt.ireq_cal[2] = 0;
        akt.ireq_cal[3] = 0;
        akt.psi_cmd[0] = 0;
        akt.psi_cmd[1] = 0;
        akt.psi_cmd_lpf[0] = 0;
        akt.psi_cmd_lpf[1] = 0;
        akt.psi_cal[0] = 0;
        akt.psi_cal[1] = 0;
        akt.psi_cal[2] = 0;
        akt.psi_cal[3] = 0;
            akt.psi_stator[0] = 0;
            akt.emf_stator[1] = 0;
            akt.field_speed_est = 0;
            akt.field_speed_est_lpf = 0;
            akt.omg_est = 0;
        akt.the_u = 0.0;
        akt.the_y = 0.0;
        akt.the_u_lpf = 0.0;
        akt.the_y_hpf = 0.0;
        akt.the_y_hpf_temp = 0.0;
        akt.the_error = 0.0;
        // akt.delta_rreq = 0;
        // akt.the_gain_P = ; // 仿真就快一点
        // akt.the_gain_P = TS*P1*5;
        // akt.the_gain_P = TS*P1*1;
        akt.rs_cal = MOTOR.R;
            akt.voltage_drop[0] = 0;
            akt.voltage_drop[1] = 0;
            akt.voltage_drop_mod = 0;
            akt.current_mod = 0;
            akt.count_rs = 0;
        akt.indicator_rs = 0.0;
        akt.the_error_sumup = 0.0;
        akt.squared_fluxMod = 0.1;
        akt.squared_fluxMod_lpf = 0.1;
        akt.pseudo_iMreq_lpf = 0.0;
        akt.the_u_sumup = 0.0;
        akt.the_u_offset = 0.0;
        akt.gamma_res_transient = 0.0;
        akt.GAIN_RS = 0.8;
        akt.omg_ctrl_err = 0;
        akt.gamma_res_transient = 0;
        akt.gamma_res_transient_shape = 0.001;
        // akt.zero_freq_operation_on = false;
    }
    
    void SpeedEstimationFromtheVMBasedFluxEstimation(){
        // akt.emf_stator[0] = US_C(0)-akt.rs_cal * IS_C(0);
        // akt.emf_stator[1] = US_C(1)-akt.rs_cal * IS_C(1);
        // akt.psi_stator[0] = akt.psi_cal[0] + (*CTRL).motor->Lq * IS_C(0);
        // akt.psi_stator[1] = akt.psi_cal[1] + (*CTRL).motor->Lq * IS_C(1);
        #if AFE_35_SATURATION_TIME_DIFFERENCE  
            akt.emf_stator[0] = AFE_USED.emf_stator[0];
            akt.emf_stator[1] = AFE_USED.emf_stator[1];
            akt.psi_stator[0] = AFE_USED.psi_1[0];
            akt.psi_stator[1] = AFE_USED.psi_1[1];
        #endif
        REAL temp;
        temp = (akt.psi_stator[0]*akt.psi_stator[0]+akt.psi_stator[1]*akt.psi_stator[1]);
        if(temp>0.001){
            akt.field_speed_est = - (akt.psi_stator[0]*-akt.emf_stator[1] + akt.psi_stator[1]*akt.emf_stator[0]) / temp;
        }
        akt.field_speed_est_lpf = _lpf(akt.field_speed_est, akt.field_speed_est_lpf, 7); // TAU_OFF = 5 from experiment
        akt.omg_est_lpf = akt.field_speed_est_lpf; // 只需对同步速LPF，这样滑差的快速变化可以体现在转速中！
    }
    void RS_Identificaiton(){
        akt.omg_fb = akt.omg_est_lpf;
        akt.omg_ctrl_err = akt.omg_fb - (*CTRL).i->cmd_varOmega * MOTOR.npp;

        akt.gamma_res_transient = exp(-akt.omg_ctrl_err*akt.omg_ctrl_err*akt.gamma_res_transient_shape);
        akt.xTem = (*CTRL).motor->npp * ( IS_C(1) * AFE_USED.psi_2[0] - IS_C(0) * AFE_USED.psi_2[1]);
        akt.current_mod      = (IS_C(0)*IS_C(0) + IS_C(1)*IS_C(1));    
        akt.voltage_drop_mod = IS_C(0)*US_C(0) + IS_C(1)*US_C(1) - (*CTRL).motor->npp_inv * akt.omg_est_lpf * akt.xTem; // akt.field_speed_est or CTRL.omega_syn, 0.5 = 1/im.npp
        // akt.voltage_drop_mod = IS_C(0)*US_C(0) + IS_C(1)*US_C(1) - (*CTRL).motor->npp_inv * akt.omg_est_lpf * akt.xTem;
        if(++akt.count_rs < 0.2* 10000 *1){
            akt.the_u += 1e-4 * akt.gamma_res_transient * akt.current_mod;
            akt.the_y += 1e-4 * akt.gamma_res_transient * akt.voltage_drop_mod;
            // the_u += TS * akt.current_mod;
            // the_y += TS * akt.voltage_drop_mod;
        }else{
            // if(ob.timebase>5)
            // if(akt.zero_freq_operation_on==false)
            akt.rs_cal += (akt.GAIN_RS*akt.the_u/(1+akt.the_u*akt.the_u*akt.GAIN_RS)) * (akt.the_y - akt.rs_cal*akt.the_u);
            akt.count_rs = 0; // printf("%d\n", akt.count_rs); // 800 is correct
            akt.the_u = 0.0;
            akt.the_y = 0.0;
        }
    }
#endif
#if ALG_Awaya_InertiaId
    void init_InertiaId(){
        awy.awaya_lambda = 31.4*1;
        awy.q0 = 0.0;
        awy.q1_dot = 0.0;
        awy.q1 = 0.0;
        awy.tau_est = 0.0;
        awy.tau_est_lpf = 0.0;
        awy.sum_A = 0.0;
        awy.sum_B = 0.0;
        awy.est_Js_variation = 0.0;
        awy.est_Js = 0.0;
    }
    void Awaya_InertiaId(){

        #define NORMINAL_INERTIA ((*CTRL).motor->Js)
        #define SPEED_SIGNAL (akt.omg_est_lpf)
        // #define SPEED_SIGNAL (CTRL.omg_fb)

        // Inertia Observer 2 Awaya1992 
        static double t = 0.0;
        t += CL_TS;

        // awy.q0 += TS * awy.awaya_lambda*( -awy.q0 + im.npp*awaya_fluxMod*CTRL.iTs_cmd);
        // awy.q0 += TS * awy.awaya_lambda*( -awy.q0 + im.npp*awaya_fluxMod*CTRL.iTs); // <- awy.used this 
        // awy.q0 += TS * awy.awaya_lambda*( -awy.q0 + CTRL.torque_cmd);
        // awy.q0 += TS * awy.awaya_lambda*( -awy.q0 + IM.Tem);
        #if AFE_37_NO_SATURATION_BASED || AFE_35_SATURATION_TIME_DIFFERENCE  
            akt.psi_cal[0] = AFE_USED.psi_2[0];
            akt.psi_cal[1] = AFE_USED.psi_2[1];
        #endif
        // awy.q0 += TS * awy.awaya_lambda*( -awy.q0 + im.npp*(IS_C(0)*-htz.psi_2[1]+IS_C(1)*htz.psi_2[0]));
        awy.q0 += CL_TS * awy.awaya_lambda*( -awy.q0 + CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp*(IS_C(0)*-akt.psi_cal[1]+IS_C(1)*akt.psi_cal[0]));

        awy.q1_dot   = awy.awaya_lambda*( -awy.q1 + SPEED_SIGNAL * (*CTRL).motor->npp_inv);
        awy.q1 += CL_TS * awy.q1_dot;
        awy.tau_est = -(NORMINAL_INERTIA)*awy.q1_dot + awy.q0;
        awy.tau_est_lpf = _lpf(awy.tau_est, awy.tau_est_lpf, 5); // 5 from experiment

        awy.sum_A += CL_TS * (awy.tau_est*awy.q1_dot);
        awy.sum_B += CL_TS * (awy.q1_dot*awy.q1_dot);
        if(t >= 2){      
            if(awy.sum_B<0.0001){
                awy.sum_B = 0.0001;
            }
            awy.est_Js_variation = + awy.sum_A / awy.sum_B;
            awy.est_Js = NORMINAL_INERTIA + awy.est_Js_variation;
            // printf("%g: %g, %g, %g, %g\n", CTRL.timebase, awy.est_Js, awy.est_Js_variation, awy.sum_A, awy.sum_B);
            // printf("%g: %g, %g, %g, %g\n", ob.timebase, (NORMINAL_INERTIA+awy.est_Js_variation), awy.est_Js_variation, awy.sum_A, awy.sum_B);
            t = 0.0;
            awy.sum_A = 0.0;
            awy.sum_B = 0.0;
        }
        // awy.est_Js = NORMINAL_INERTIA+awy.est_Js_variation;
        if(awy.est_Js>0.02){
            awy.est_Js = 0.02;
        }else if(awy.est_Js<0.0001){
            awy.est_Js = 0.0001;
        }
    }
#endif

#if ALG_PLL_norm
    void init_PLL_norm(){
        PLLN.emf_ampl = 0;
        PLLN.theta_elec = 0;
        PLLN.epsilon_e = 0;
        PLLN.omega_elec = 0; 
        PLLN.omega_integral = 0;
        PLLN.emf_norm[0] = 0;
        PLLN.emf_norm[1] = 0;
        PLLN.emf_ampl = 0;
        PLLN.x[0] = 0;
        PLLN.x[1] = 0;
        PLLN.kp = 39;
        PLLN.ki = 100;
        PLLN.emf_recon[0] = 0;
        PLLN.emf_recon[1] = 0;
        PLLN.k_p_theta = 1;
    }
    void rhf_PLL_norm(REAL t, REAL *x, REAL *fx){
        REAL emf_recon[2];
        emf_recon[0] = PLLN.emf_norm[0] * cos(x[1]);
        emf_recon[1] = PLLN.emf_norm[1] * sin(x[1]);
        PLLN.epsilon_e = - emf_recon[0] - emf_recon[1];
        PLLN.omega_elec = x[0] + PLLN.kp * PLLN.epsilon_e;
        fx[0] = PLLN.ki * PLLN.epsilon_e;
        fx[1] = PLLN.omega_elec;
    }
    void Main_PLL_norm(REAL emf[2]){
        PLLN.emf_ampl = sqrt(emf[1]*emf[1] + emf[0]*emf[0]);
        if(PLLN.emf_ampl != 0){
            PLLN.emf_norm[0] = emf[0]/PLLN.emf_ampl;
            PLLN.emf_norm[1] = emf[1]/PLLN.emf_ampl;
        }else{
            PLLN.emf_norm[0] = emf[0] * 10;
            PLLN.emf_norm[1] = emf[1] * 10;
        }
        general_2states_rk4_solver(&rhf_PLL_norm, (*CTRL).timebase, PLLN.x, CL_TS);
        PLLN.omega_integral = PLLN.x[0];
        PLLN.omega_elec = PLLN.omega_integral + PLLN.kp * PLLN.epsilon_e;
        PLLN.theta_elec = PLLN.x[1];
        while(PLLN.theta_elec > M_PI) PLLN.theta_elec  -= 2*M_PI;
        while(PLLN.theta_elec < -M_PI) PLLN.theta_elec+= 2*M_PI;
    }
#endif
/********************************************/
/* COMMON *
 ********************************************/
// void init_rk4(){
//     int i;
//     for(i=0; i<2; ++i){
//         OBSV.rk4.us[i] = 0;
//         OBSV.rk4.is[i] = 0;
//         // OBSV.rk4.us_curr[i] = 0;
//         OBSV.rk4.is_curr[i] = 0;
//         OBSV.rk4.us_prev[i] = 0;
//         OBSV.rk4.is_prev[i] = 0;
//         OBSV.rk4.is_lpf[i]  = 0;
//         OBSV.rk4.is_hpf[i]  = 0;
//         OBSV.rk4.is_bpf[i]  = 0;

//         OBSV.rk4.current_lpf_register[i] = 0;
//         OBSV.rk4.current_hpf_register[i] = 0;
//         OBSV.rk4.current_bpf_register1[i] = 0;
//         OBSV.rk4.current_bpf_register2[i] = 0;
//     }
// }
void pmsm_observers(){
    // stationary_voltage_DOB();

    (*CTRL).motor->KActive = MOTOR.KE + (MOTOR.Ld - MOTOR.Lq) * (*CTRL).i->cmd_iDQ[0];

    #if PC_SIMULATION
        // /* Cascaded Flux Estimator */
        simulation_test_flux_estimators();
        // OBSV.nsoaf.theta_d = FE.AFEOE.theta_d ;

        // /* Speed and Position Estimator */
        // Main_harnefors_scvm();
        // // cjh_eemfao();
        // // cjh_eemfhgo_farza09();
        Main_nsoaf_chen2020();
        #if ALG_AKT_SPEED_EST_AND_RS_ID
            SpeedEstimationFromtheVMBasedFluxEstimation();
            RS_Identificaiton();
        #endif
        // Main_esoaf_chen2021();
        // // Main_QiaoXia2013_emfSMO();
        // Main_ChiXu2009_emfSMO();
        // Main_parksul2014_FADO();
    #else
        /* 资源有限 */
        #if SELECT_ALGORITHM == ALG_NSOAF
            // MainFE_HuWu_1998(); // use algorithm 2
            VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
            #if AFE_38_OUTPUT_ERROR_CLOSED_LOOP
                Main_VM_ClosedLoopFluxEstimatorForPMSM();
            #endif
            Main_nsoaf_chen2020();
            #if AFE_25_VM_CM_FUSION
            Main_the_active_flux_estimator();
            #endif
            #if ALG_AKT_SPEED_EST_AND_RS_ID
                SpeedEstimationFromtheVMBasedFluxEstimation();
                RS_Identificaiton();
            #endif
        #elif SELECT_ALGORITHM == ALG_ESOAF
            Main_the_active_flux_estimator();
            Main_VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();
            Main_esoaf_chen2021();
        #elif SELECT_ALGORITHM == ALG_Farza_2009
            Main_cjh_eemfhgo_farza09();
        #elif SELECT_ALGORITHM == ALG_CJH_EEMF
            Main_cjh_eemfao();
        #elif SELECT_ALGORITHM == ALG_Harnefors_2006
            Main_harnefors_scvm();
        #elif SELECT_ALGORITHM == ALG_Qiao_Xia
            Main_QiaoXia2013_emfSMO();
        #elif SELECT_ALGORITHM == ALG_Chi_Xu
            Main_ChiXu2009_emfSMO();
        #elif SELECT_ALGORITHM == ALG_Park_Sul
            Main_parksul2014_FADO();
        #endif
    #endif

    G.omg_elec = PMSM_ELECTRICAL_SPEED_FEEDBACK;
    G.theta_d  = PMSM_ELECTRICAL_POSITION_FEEDBACK;

    // if(G.Select_algorithm == 1){
    //     G.omg_elec = OBSV.nsoaf.xOmg;
    //     G.theta_d  = AFE_USED.theta_d;
    // }else if(G.Select_algorithm == 2){
    //     G.omg_elec = parksul.xOmg;
    //     G.theta_d  = parksul.theta_d;
    // }else if(G.Select_algorithm == 3){
    //     G.omg_elec = OBSV.chixu.xOmg;
    //     G.theta_d  = OBSV.chixu.theta_d;
    // }else if(G.Select_algorithm == 4){
    //     G.omg_elec = qiaoxia.xOmg;
    //     G.theta_d  = qiaoxia.theta_d;
    // }


    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    // US_P(0) = US_C(0); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    // US_P(1) = US_C(1); // 由于没有测量电压，所以当前步电压是伪概念，在这里更新是无意义的
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1); 
}

void variabel_parameters_sensorless(){
    //for Lq, R, KE
    #if WHO_IS_USER == USER_YZZ
    int i;
    i = d_sim.user.Variable_Parameters_time_num/2;
    d_sim.user.Variable_Parameters_timebase += CL_TS;
    if (d_sim.user.Variable_Parameters_status == 1){
        if(d_sim.user.VP_time_num_count  < d_sim.user.Variable_Parameters_time_num){
            if ((d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*d_sim.user.VP_time_num_count)&&(d_sim.user.Variable_Parameters_timebase < d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))){
                (*CTRL).motor->R = d_sim.init.R * (1-0.01*d_sim.user.Variable_Parameters_percent*(d_sim.user.VP_time_num_count-i));
            }
            else if(d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))
            {
                d_sim.user.VP_time_num_count +=1;
            }
        }
    }else if(d_sim.user.Variable_Parameters_status == 2){
        if(d_sim.user.VP_time_num_count  < d_sim.user.Variable_Parameters_time_num){
            if ((d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*d_sim.user.VP_time_num_count)&&(d_sim.user.Variable_Parameters_timebase < d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))){
                (*CTRL).motor->Lq = d_sim.init.Lq * (1-0.01*d_sim.user.Variable_Parameters_percent*(d_sim.user.VP_time_num_count-i));
            }
            else if(d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))
            {
                d_sim.user.VP_time_num_count +=1;
            }
        }
    }else if(d_sim.user.Variable_Parameters_status == 3){
        if(d_sim.user.VP_time_num_count  < d_sim.user.Variable_Parameters_time_num){
            if ((d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*d_sim.user.VP_time_num_count)&&(d_sim.user.Variable_Parameters_timebase < d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))){
                (*CTRL).motor->KE = d_sim.init.KE * (1-0.01*d_sim.user.Variable_Parameters_percent*(d_sim.user.VP_time_num_count-i));
            }
            else if(d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))
            {
                d_sim.user.VP_time_num_count +=1;
            }
        }
    }else if(d_sim.user.Variable_Parameters_status == 4){
        if(d_sim.user.VP_time_num_count  < d_sim.user.Variable_Parameters_time_num){
            if ((d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*d_sim.user.VP_time_num_count)&&(d_sim.user.Variable_Parameters_timebase < d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))){
                OBSV.sum_for_encoder += (*CTRL).s->iQ->Fbk;
                OBSV.count_for_encoder +=1; 
            }
            else if(d_sim.user.Variable_Parameters_timebase > d_sim.user.Variable_Parameters_time*(d_sim.user.VP_time_num_count+1))
            {
                d_sim.user.VP_time_num_count +=1;
                OBSV.aver_for_encoder[d_sim.user.VP_time_num_count-1] = OBSV.sum_for_encoder / OBSV.count_for_encoder;
                OBSV.sum_for_encoder = 0;
                OBSV.count_for_encoder = 0;
            }
        }
    }
        #if PC_SIMULATION == FALSE
            if(!Axis_1.FLAG_ENABLE_PWM_OUTPUT){   
                (*CTRL).motor->KE = d_sim.init.KE;
                (*CTRL).motor->R = d_sim.init.R;
                (*CTRL).motor->Lq = d_sim.init.Lq; 
                d_sim.user.Variable_Parameters_timebase = 0;
                d_sim.user.VP_time_num_count = 0;
            }
        #endif
    #endif
}

void init_pmsm_observers(){
    // RK4
    // init_rk4();     // 龙格库塔法结构体初始化

    // FE
    init_FE();
    #if ALG_AKT_SPEED_EST_AND_RS_ID
    init_ake_Speed_Est_and_RS_ID();
    #endif
    #if ALG_Awaya_InertiaId
    init_InertiaId();
    #endif
    #if ALG_PLL_norm
    init_PLL_norm();
    #endif

    #if PC_SIMULATION
        // OBSV
        init_nsoaf();
        init_esoaf();
        init_hgo4eemf();
        init_cjheemf(); // OBSV.cjheemf 结构体初始化
        init_harnefors(); // harnefors结构体初始化
        init_QiaoXia2013();
        init_ChiXu2009();
        init_parksul2014();
    #else
        /* 资源有限 */
        #if SELECT_ALGORITHM == ALG_NSOAF
            init_nsoaf();
        #elif SELECT_ALGORITHM == ALG_ESOAF
            init_esoaf();
        #elif SELECT_ALGORITHM == ALG_Farza_2009
            init_hgo4eemf();
        #elif SELECT_ALGORITHM == ALG_CJH_EEMF
            init_cjheemf(); // OBSV.cjheemf 结构体初始化
        #elif SELECT_ALGORITHM == ALG_Harnefors_2006
            init_harnefors(); // harnefors结构体初始化
        #elif SELECT_ALGORITHM == ALG_Qiao_Xia
            init_QiaoXia2013();
        #elif SELECT_ALGORITHM == ALG_Chi_Xu
            init_ChiXu2009();
        #elif SELECT_ALGORITHM == ALG_Park_Sul
            init_parksul2014();
        #endif
    #endif
}
#endif

//inverter nonlinearity

REAL lpf1_inverter(REAL x, REAL y_tminus1)
{
    // #define LPF1_RC 0.6 // 0.7, 0.8, 1.2, 3太大滤得过分了 /* 观察I5+I7_LPF的动态情况进行确定 */
    // #define ALPHA_RC (TS/(LPF1_RC + TS)) // TODO：优化
    // return y_tminus1 + ALPHA_RC * (x - y_tminus1); // 0.00020828993959591752
    return y_tminus1 + 0.00020828993959591752 * (x - y_tminus1);
}
REAL shift2pi(REAL thetaA)
{
    if (thetaA > M_PI)
    {
        return thetaA - 2 * M_PI;
    }
    else if (thetaA < -M_PI)
    {   
        return thetaA + 2 * M_PI;
    }
    else
    {
        return thetaA;
    }
}
REAL trapezoidal_voltage_by_phase_current(REAL current, REAL V_plateau, REAL I_plateau, REAL oneOnver_I_plateau)
{
    REAL abs_current = fabsf(current);
    if (abs_current < I_plateau)
    {
        return current * V_plateau * oneOnver_I_plateau;
    }
    else
    {
        return sign(current) * V_plateau;
    }
}

REAL sig_a2 = 6.7;
REAL sig_a3 = 5.6;
REAL sigmoid(REAL x)
{
    // beta
    // REAL a1 = 0.0; // *1.28430164
    // REAL a2 = 8.99347107;
    // REAL a3 = 5.37783655;

    // -------------------------------

#ifdef _XCUBE1
    /* Si Mini OW IPM Inverter*/

    // b-phase @180 V Mini6PhaseIPMInverter
    // REAL a1 = 0.0; //1.34971502
    // REAL a2 = 16.0575341;  // yuefei tuning gives a2' = 9.2*2
    // REAL a3 = 17.59278688; // yuefei tuning gives a3' = a3*5

    // b-phase @80 V Mini6PhaseIPMInverter
    // REAL a1 = 0.0; //2.10871359 ;
    // REAL a2 = 7.36725304;
    // REAL a3 = 62.98712805;
    REAL a1 = 0.0; //(1.705,
    REAL a2 = sig_a2;
    REAL a3 = sig_a3;

#else
    /* SiC Cree 3 Phase Inverter*/

    // b-phase @20 V
    // REAL a1 = 0.0; //1.25991178;
    // REAL a2 = 2.30505904;
    // REAL a3 = 12.93721814;

    // b-phase @80 V
    // REAL a1 = 0.0; //1.28430164;
    // REAL a2 = 7.78857441;
    // REAL a3 = 6.20979077;
    REAL a1 = 0.0; // 1.6997103;
    REAL a2 = sig_a2;
    // 6.67159129;
    REAL a3 = sig_a3;
    // 8.42010418;

    // b-phase @180 V
    /*最早的拟合结果*/
    // REAL a1 = 0.0; //1.50469916;
    // REAL a2 = 13.48467604;
    // REAL a3 = 5.22150403;
    /*错误！多乘了一次beta-axis转phase系数！*/
    // REAL a1 = 0.0; //1.83573529
    // REAL a2 = sig_a2; 13.36317172;
    // REAL a3 = sig_a3; 5.81981571;
    /*纠正！100个点，平均分配*/
    // REAL a1 = 0.0; //1.8357354;
    // REAL a2 = sig_a2; 15.43046115;
    // REAL a3 = sig_a3; 5.04010863;

#endif

    return a1 * x + a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}

REAL sigmoid_online_v2(REAL x, REAL a2, REAL a3)
{
    return a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}
REAL sigmoid_online(REAL x, REAL Vsat, REAL a3)
{

    REAL a2 = Vsat * 2;
    // REAL a3 = 5.22150403;
    return a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}

void inverterNonlinearity_Initialization()
{
    INV.gamma_theta_trapezoidal = GAIN_THETA_TRAPEZOIDAL;
#ifdef _XCUBE1
    INV.Vsat = 16.0575341 / 2; // 6.67054; // 180 V SiC
#else
    // INV.Vsat = sig_a2*0.5; //6.74233802;
    INV.Vsat = 7.84; // 150 V
#endif

    INV.gain_Vsat = 0 * 10;

    INV.thetaA = 0;
    INV.cos_thetaA = 1;
    INV.sin_thetaA = 0;

    // --
    INV.u_comp[0] = 0;
    INV.u_comp[1] = 0;
    INV.u_comp[2] = 0;
    INV.ual_comp = 0;
    INV.ube_comp = 0;
    INV.uDcomp_atA = 0;
    INV.uQcomp_atA = 0;
    
    INV.iD_atA = 0;
    INV.iQ_atA = 0;
    INV.I5_plus_I7 = 0;
    INV.I5_plus_I7_LPF = 0.0;
    INV.theta_trapezoidal = 11.0 * M_PI_OVER_180; // in rad

#ifdef _XCUBE1
    INV.I_plateau_Max = 2.0;
    INV.I_plateau_Min = 0.2;
#else
    INV.I_plateau_Max = 1.0;
    INV.I_plateau_Min = 0.1;
#endif
    INV.I_plateau = 0.7;
    INV.V_plateau = 1.5 * sig_a2 * 0.5;
    INV.gamma_I_plateau = 10.0;
    INV.gamma_V_plateau = 0.0; // this is updated by the estimated disturbance to the sinusoidal flux model.

#if PC_SIMULATION
    INV.sig_a2 = 1.0 * sig_a2;
    INV.sig_a3 = 1.5 * sig_a3; // = shape parameter
#else
    INV.sig_a2 = sig_a2; // = Plateau * 2
    INV.sig_a3 = sig_a3; // = shape parameter
#endif

    INV.w6 = 1;
    INV.w12 = 0;
    INV.w18 = 0;
    INV.gamma_a2 = 400;
    INV.gamma_a3 = 700;
}
void Online_PAA_Based_Compensation(void)
{

    // /* 角度反馈: (*CTRL).i->theta_d_elec or ELECTRICAL_POSITION_FEEDBACK? */
    // INV.thetaA = ELECTRICAL_POSITION_FEEDBACK;
    // // I (current vector amplitude)
    // INV.iD_atA = sqrt(IS_C(0)*IS_C(0) + IS_C(1)*IS_C(1));

    /* 在Park2012中，a相电流被建模成了sin函数，一个sin函数的自变量角度放在正交坐标系下看就是在交轴上的，所以要-1.5*pi */

    // Phase A current's fundamental component transformation
    /* 这里使用哪个角度的关键不在于是有感的角度还是无感的角度，而是你FOC电流控制器（Park变换）用的角度是哪个？ */
    if (d_sim.user.sensorless_only_theta_on != 0)
    {
        INV.thetaA = -M_PI * 1.5 + PMSM_ELECTRICAL_POSITION_FEEDBACK + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
        // printf("thetaA = %f\n", INV.thetaA);
    }
    else
    {
        INV.thetaA = -M_PI * 1.5 + (*CTRL).i->theta_d_elec + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
    }
    INV.thetaA = shift2pi(INV.thetaA); /* Q: how to handle it when INV.thetaA jumps between pi and -pi? */ // 这句话绝对不能省去，否则C相的梯形波会出错。

    INV.cos_thetaA = cosf(INV.thetaA);
    INV.sin_thetaA = sinf(INV.thetaA);
    INV.iD_atA = AB2M(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);
    INV.iQ_atA = AB2T(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);
    // printf("iD_atA = %f, iQ_atA = %f\n", INV.iD_atA, INV.iQ_atA);

    if (FALSE)
    {
        /* Use q-axis current in phase A angle (Tentative and Failed) */
        INV.I5_plus_I7 = INV.iQ_atA * cosf(6 * INV.thetaA);
        INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF);

        INV.I11_plus_I13 = INV.iQ_atA * cosf(12 * INV.thetaA);
        INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

        INV.I17_plus_I19 = INV.iQ_atA * cosf(18 * INV.thetaA);
        INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);
    }
    else
    {
        /* Use d-axis current in phase A angle */
        INV.I5_plus_I7 = INV.iD_atA * sinf(6 * INV.thetaA);                     /* Q: Why sinf? Why not cosf? 和上面的-1.5*pi有关系吗？ */
        INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF); /* lpf1 for inverter */

        INV.I11_plus_I13 = INV.iD_atA * sinf(12 * INV.thetaA);
        INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

        INV.I17_plus_I19 = INV.iD_atA * sinf(18 * INV.thetaA);
        INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);
        // printf("I5+I7_LPF = %f\n", INV.I5_plus_I7_LPF);
        // printf("I11+I13_LPF = %f\n", INV.I11_plus_I13_LPF);
        // printf("I17+I19_LPF = %f\n", INV.I17_plus_I19_LPF);
    }

#if PC_SIMULATION
    // INV.gamma_a2 = 0.0;
    // INV.gamma_a3 = 0.0;
#endif

    /* Online Update Sigmoid a3 */
    // if((*CTRL).timebase>35){
    //     INV.gamma_I_plateau = 0.0;
    // }
    INV.sig_a3 -= CL_TS * INV.gamma_a3 // *fabsf((*CTRL).i->cmd_speed_rpm)
                  * (INV.w6 * INV.I5_plus_I7_LPF + INV.w12 * INV.I11_plus_I13_LPF + INV.w18 * INV.I17_plus_I19_LPF);
    INV.error_a3 = INV.w6 * INV.I5_plus_I7_LPF + INV.w12 * INV.I11_plus_I13_LPF + INV.w18 * INV.I17_plus_I19_LPF;
    (*CTRL).s->Motor_or_Generator = sign((*CTRL).i->varOmega * (*CTRL).i->cmd_iDQ[1]);
    // (*CTRL).s->Motor_or_Generator = sign((*CTRL).i->cmd_omg_elec);

    /* Online Update Sigmoid a2 */
    if ((*CTRL).timebase > 2)
    {
        /* Sensorless: Adaptive a2 based on flux amplitude error */
        // use linear FE
        // INV.sig_a2 += CL_TS * -100 * AFEOE.output_error_dq[0];

        // use nonlinear (saturation) FE
        FE.htz.psi_2_ampl_lpf = lpf1_inverter(FE.htz.psi_2_ampl, FE.htz.psi_2_ampl_lpf);
        INV.sig_a2 += CL_TS * -INV.gamma_a2 * (*CTRL).s->Motor_or_Generator * (MOTOR.KActive - FE.htz.psi_2_ampl_lpf);
        INV.error_a2 = MOTOR.KActive - FE.htz.psi_2_ampl_lpf;
        /* Sensored: Adaptive a2 based on position error */
        /* To use this, you must have a large enough stator current */
        /* 这个好像只对梯形波（电流值的函数）有效 ……*/
        // INV.sig_a2 += CL_TS * INV.gain_Vsat * sinf(ENC.theta_d_elec - ELECTRICAL_POSITION_FEEDBACK) * (*CTRL).s->Motor_or_Generator;
    }
    if (INV.sig_a2 > 40)
    {
        INV.sig_a2 = 40;
    }
    else if (INV.sig_a2 < 2)
    {
        INV.sig_a2 = 2;
    }

    /* Chen2021: linear approximation of u-i curve */
    // if((*CTRL).timebase>35){
    //     INV.gamma_I_plateau = 0.0;
    // }
    // INV.I_plateau += CL_TS * 0 * INV.gamma_I_plateau \
    //                         // *fabsf((*CTRL).i->cmd_speed_rpm)
    //                         *(    1*INV.I5_plus_I7_LPF
    //                             + 0*INV.I11_plus_I13_LPF
    //                             + 0*INV.I17_plus_I19_LPF
    //                          );
    // if(INV.I_plateau > INV.I_plateau_Max){
    //     INV.I_plateau = INV.I_plateau_Max;
    // }else if(INV.I_plateau < INV.I_plateau_Min){
    //     INV.I_plateau = INV.I_plateau_Min;
    // }

    /* Chen2021SlessInv 覆盖 */
    if (INV.gamma_I_plateau != 0)
    {
        REAL ia_cmd = ((*CTRL).o->cmd_iAB[0]);
        // printf("((*CTRL).o->cmd_iAB[0] = %f\n", (*CTRL).o->cmd_iAB[0]);
        REAL ib_cmd = (-0.5 * (*CTRL).o->cmd_iAB[0] - SIN_DASH_2PI_SLASH_3 * (*CTRL).o->cmd_iAB[1]);
        REAL ic_cmd = (-0.5 * (*CTRL).o->cmd_iAB[0] - SIN_2PI_SLASH_3 * (*CTRL).o->cmd_iAB[1]);
        REAL oneOver_I_plateau = 1.0 / INV.I_plateau;
        INV.u_comp[0] = trapezoidal_voltage_by_phase_current(ia_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);
        INV.u_comp[1] = trapezoidal_voltage_by_phase_current(ib_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);
        INV.u_comp[2] = trapezoidal_voltage_by_phase_current(ic_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);

        /* Online Sigmoid a3 覆盖 覆盖 */
        INV.u_comp[0] = sigmoid_online_v2(ia_cmd, INV.sig_a2, INV.sig_a3);
        INV.u_comp[1] = sigmoid_online_v2(ib_cmd, INV.sig_a2, INV.sig_a3);
        INV.u_comp[2] = sigmoid_online_v2(ic_cmd, INV.sig_a2, INV.sig_a3);
        // printf("ia_cmd = %f, INV.sig_a2 = %f, INV.sig_a3 = %f\n", ia_cmd, INV.sig_a2, INV.sig_a3);
        // printf("u_comp[0] = %f, u_comp[1] = %f, u_comp[2] = %f\n", INV.u_comp[0], INV.u_comp[1], INV.u_comp[2]);
    }

    /* 相补偿电压Clarke为静止正交坐标系电压。 */
    // 改成恒幅值变换
    INV.ual_comp = 0.66666666667 * (INV.u_comp[0] - 0.5 * INV.u_comp[1] - 0.5 * INV.u_comp[2]);
    INV.ube_comp = 0.66666666667 * 0.86602540378 * (INV.u_comp[1] - INV.u_comp[2]);
    // INV.ual_comp = SQRT_2_SLASH_3      * (INV.u_comp[0] - 0.5*INV.u_comp[1] - 0.5*INV.u_comp[2]); // sqrt(2/3.)
    // INV.ube_comp = 0.70710678118654746 * (                    INV.u_comp[1] -     INV.u_comp[2]); // sqrt(2/3.)*sinf(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)

    // 区分补偿前的电压和补偿后的电压：
    // (*CTRL).ual, (*CTRL).ube 是补偿前的电压！
    // (*CTRL).ual + INV.ual_comp, (*CTRL).ube + INV.ube_comp 是补偿后的电压！
}
void yzz_inverter_Compensation_Online_PAA()
{
    if(d_sim.user.inverter_nonlinearity_on == 1){
        Online_PAA_Based_Compensation();
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + INV.ual_comp;
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + INV.ube_comp;
    }
}


#define LUT_N_LC  70
#define LUT_N_HC  29
REAL lut_lc_voltage[70] = {0, 1.8746, 2.144, 2.34136, 2.49419, 2.60973, 2.74337, 2.88097, 3.00094, 3.10985, 3.23057, 3.34807, 3.4332, 3.50093, 3.55079, 3.59368, 3.61982, 3.64135, 3.65953, 3.67652, 3.69216, 3.7051, 3.71887, 3.73073, 3.73901, 3.75071, 3.75723, 3.76842, 3.77575, 3.78264, 3.78858, 3.79489, 3.80023, 3.80754, 3.8125, 3.81981, 3.82407, 3.8291, 3.83278, 3.83879, 3.84318, 3.84597, 3.85316, 3.85658, 3.85856, 3.86383, 3.86556, 3.86962, 3.87269, 3.87618, 3.87955, 3.88147, 3.88496, 3.88746, 3.88879, 3.89325, 3.89452, 3.8973, 3.89922, 3.9023, 3.90339, 3.90545, 3.90747, 3.90848, 3.91075, 3.91321, 3.915, 3.917, 3.91789, 3.91824};
REAL lut_hc_voltage[29] = {3.92009, 3.96505, 4.00084, 4.02508, 4.04373, 4.05894, 4.07484, 4.08679, 4.09461, 4.10467, 4.11101, 4.11741, 4.12254, 4.12758, 4.1317, 4.13569, 4.14004, 4.14258, 4.1448, 4.14871, 4.14937, 4.15295, 4.15194, 4.15295, 4.15107, 4.15136, 4.15206, 4.1492, 4.14973};
#define LUT_STEPSIZE_BIG 0.3796966241724138
#define LUT_STEPSIZE_SMALL 0.012110000369565214
#define LUT_STEPSIZE_BIG_INVERSE 2.633681566644419
#define LUT_STEPSIZE_SMALL_INVERSE 82.5763806344048
#define LUT_I_TURNING_LC 0.847700025869565
#define LUT_I_TURNING_HC 11.858902126869566
#define V_PLATEAU 4.149734790202423

REAL lookup_compensation_voltage_indexed(REAL current_value)
{
    REAL abs_current_value = fabsf(current_value);

    if (abs_current_value < LUT_I_TURNING_LC){
        REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
        int index = (int)float_index;
        REAL slope;
        if (index + 1 >= LUT_N_LC)
            slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
        else
            slope = (lut_lc_voltage[index + 1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
        return sign(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index * LUT_STEPSIZE_SMALL));
    }
    else{
        REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
        int index = (int)float_index; // THIS IS A RELATIVE INDEX!
        REAL slope;
        if (index + 1 >= LUT_N_HC)
            return sign(current_value) * V_PLATEAU;
        else
            slope = (lut_hc_voltage[index + 1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
        return sign(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index * LUT_STEPSIZE_BIG));
    }
}
void get_distorted_voltage_via_LUT_indexed(REAL ial, REAL ibe, REAL *ualbe_dist)
{

    /* 查表法 */
    if (TRUE){
        REAL ia, ib, ic;
        ia = 1 * (ial);
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

        REAL dist_ua = lookup_compensation_voltage_indexed(ia);
        REAL dist_ub = lookup_compensation_voltage_indexed(ib);
        REAL dist_uc = lookup_compensation_voltage_indexed(ic);

        G.ia = ia;
        G.dist_ua = dist_ua;

        // Clarke transformation（三分之二，0.5倍根号三）
        ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
        ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534
    }else{
        /* AB2U_AI 这宏假设了零序分量为零，即ia+ib+ic=0，但是电压并不一定满足吧，所以还是得用上面的？ */
        REAL ia, ib;            //,ic;
        ia = AB2U_AI(ial, ibe); // ia = 1 * (       ial                              );
        ib = AB2V_AI(ial, ibe); // ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );

        REAL dist_ua = lookup_compensation_voltage_indexed(ia);
        REAL dist_ub = lookup_compensation_voltage_indexed(ib);

        ualbe_dist[0] = UV2A_AI(dist_ua, dist_ub); // 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        ualbe_dist[1] = UV2B_AI(dist_ua, dist_ub); // 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc);
    }
}

/* 伪查表法（需要循环比大小，运算量不固定） */
REAL lookup_phase_current(REAL current, REAL *lut_voltage, REAL *lut_current, int length_of_lut)
{
    /* assume lut_voltage[0] is negative and lut_voltage[-1] is positive */
    int j;
    if (current < lut_current[0]){
        return lut_voltage[0];
    }else if (current > lut_current[length_of_lut - 1]){
        return lut_voltage[length_of_lut - 1];
    }else{
        for (j = 0; j < length_of_lut - 1; ++j){
            if (current > lut_current[j] && current < lut_current[j + 1]){
                REAL slope = (lut_voltage[j + 1] - lut_voltage[j]) / (lut_current[j + 1] - lut_current[j]);
                return (current - lut_current[j]) * slope + lut_voltage[j];
                // return 0.5*(lut_voltage[j]+lut_voltage[j+1]); // this is just wrong! stupid!
            }
        }
    }
#if PC_SIMULATION
    printf("DEBUG inverter.c. %g, %d, %d\n", current, j, length_of_lut);
#endif
    return 0.0;
}
REAL sigmoid_a3_tune = 1.0;
void get_distorted_voltage_via_CurveFitting(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist)
{
    // The data are measured in stator resistance identification with amplitude invariant d-axis current an``` d-axis voltage.

    /* 拟合法 */
    REAL ia, ib, ic;
    ia = 1 * (ial);
    ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
    ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);
    // printf("ic: %f\n", ic);
    // offline sigmoid
    REAL dist_ua = sigmoid(ia);
    REAL dist_ub = sigmoid(ib);
    REAL dist_uc = sigmoid(ic);

    // online tunable sigmoid (experimental)
    // REAL dist_ua = sigmoid_online(ia, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);
    // REAL dist_ub = sigmoid_online(ib, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);
    // REAL dist_uc = sigmoid_online(ic, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);

    // Clarke transformation（三分之二，0.5倍根号三）
    ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
    ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534

    // for plot
    INV.ia = ia;
    INV.ib = ib;
    INV.ic = ic;
    // INV.ia = 0;
    // INV.ib = 0;
    // INV.ic = 0; 
    INV.dist_ua = dist_ua;
    INV.dist_ub = dist_ub;
    INV.dist_uc = dist_uc;
    // INV.dist_ua = 0;
    // INV.dist_ub = 0;
    // INV.dist_uc = 0;
}
void get_distorted_voltage_via_LUT(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist, REAL *lut_voltage, REAL *lut_current, int length_of_lut)
{
    // The data are measured in stator resistance identification with amplitude invariant d-axis current and d-axis voltage.

    /* 查表法 */
    if (TRUE){
        REAL ia, ib, ic;
        ia = 1 * (ial);
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

        REAL dist_ua = lookup_phase_current(ia, lut_voltage, lut_current, length_of_lut);
        REAL dist_ub = lookup_phase_current(ib, lut_voltage, lut_current, length_of_lut);
        REAL dist_uc = lookup_phase_current(ic, lut_voltage, lut_current, length_of_lut);

        // Clarke transformation（三分之二，0.5倍根号三）
        ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
        ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534
    }else{
        /* AB2U_AI 这宏假设了零序分量为零，即ia+ib+ic=0，但是电压并不一定满足吧，所以还是得用上面的？ */
        REAL ia, ib;            //,ic;
        ia = AB2U_AI(ial, ibe); // ia = 1 * (       ial                              );
        ib = AB2V_AI(ial, ibe); // ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );
        // ic = AB2W_AI(ial, ibe); // ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3      * ibe );

        REAL dist_ua = lookup_phase_current(ia, lut_voltage, lut_current, length_of_lut);
        REAL dist_ub = lookup_phase_current(ib, lut_voltage, lut_current, length_of_lut);
        // REAL dist_uc = lookup_phase_current(ic, lut_voltage, lut_current, length_of_lut);

        // Clarke transformation（三分之二，0.5倍根号三）
        // ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        // ualbe_dist[1] = 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc); // 0.5773502695534

        ualbe_dist[0] = UV2A_AI(dist_ua, dist_ub); // 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        ualbe_dist[1] = UV2B_AI(dist_ua, dist_ub); // 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc);
    }
}


#endif
