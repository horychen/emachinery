// https://stackoverflow.com/questions/1591361/understanding-typedefs-for-function-pointers-in-c
#include "ACMSim.h"
#if ENABLE_COMMISSIONING
/* The most accurate initial position detection method is actually proposed in my 2017 TDDA paper that make use of the fact that large d-axis current do not any create torque. */

/* Initial Position Detection needs position update rate is at 1/CL_TS. */
void doIPD(){
    // 帕克变换（使用反馈位置）
    (*CTRL).s->cosT = cos(ENC.excitation_angle_deg/180.0*M_PI); // (*CTRL).i->theta_d_elec
    (*CTRL).s->sinT = sin(ENC.excitation_angle_deg/180.0*M_PI); // (*CTRL).i->theta_d_elec
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    #define CURRENT_COMMAND_VALUE (0.5*d_sim.init.IN)

    /* 根据CL_TS周期更新的位置反馈信息去辨识一个 */
    static int number_of_excitation_times = 0;
    PID_iD->Ref += CL_TS * 3;
    /* Stage 1 */
    if(number_of_excitation_times==0){
        // if reached then reset
        if( PID_iD->Ref > CURRENT_COMMAND_VALUE ){
            PID_iD->Ref = 0.0;
            number_of_excitation_times = 1;
        }
        // identify the offset angle (fast)
        if((*CTRL).i->theta_d_elec > (*CTRL).i->theta_d_elec_previous){
            ENC.excitation_angle_deg -= CL_TS*3600;
        }else if((*CTRL).i->theta_d_elec < (*CTRL).i->theta_d_elec_previous){
            ENC.excitation_angle_deg += CL_TS*3600;
        }
    /* Stage 2 */
    }else if(number_of_excitation_times==1){
        // if reached then reset
        if( PID_iD->Ref > 2*CURRENT_COMMAND_VALUE ){
            PID_iD->Ref = 0.0;
            number_of_excitation_times = 2;
        }
        // identify the offset angle (medium fast)
        if((*CTRL).i->theta_d_elec > (*CTRL).i->theta_d_elec_previous){
            ENC.excitation_angle_deg -= CL_TS*300;
        }else if((*CTRL).i->theta_d_elec < (*CTRL).i->theta_d_elec_previous){
            ENC.excitation_angle_deg += CL_TS*300;
        }
    /* Stage 3 */
    }else if(number_of_excitation_times==2){
        PID_iD->Ref = CURRENT_COMMAND_VALUE;
        // identify the offset angle (slow)
        if((*CTRL).i->theta_d_elec > (*CTRL).i->theta_d_elec_previous){
            ENC.excitation_angle_deg -= CL_TS*100;
        }else if((*CTRL).i->theta_d_elec < (*CTRL).i->theta_d_elec_previous){
            ENC.excitation_angle_deg += CL_TS*100;
        }
        static Uint32 count=0;
        if(count++>5000){
            number_of_excitation_times = 3;
        }
    /* Stage 4 */
    }else{
        PID_iD->Ref = CURRENT_COMMAND_VALUE;
        static Uint32 count=0;
        if(count++>2000){
            ENC.theta_d_offset = -((*CTRL).i->theta_d_elec - ENC.excitation_angle_deg/180.0*M_PI);
            (*CTRL).s->IPD_Done = TRUE;
            PID_iD->I_Term = 0.0; //
            PID_iQ->I_Term = 0.0; //
            (*CTRL).o->cmd_uAB[0] = 0.0;
            (*CTRL).o->cmd_uAB[1] = 0.0;
            return;
        }
    }

    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iD->calc(PID_iD);
    (*CTRL).o->cmd_uAB[0] = MT2A(PID_iD->Out, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B(PID_iD->Out, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT);
}

/* Phase Sequence Detection */
void doPSD(){
    // 帕克变换（使用反馈位置）
    (*CTRL).s->cosT = cos(PSD.theta_excitation); // (*CTRL).i->theta_d_elec
    (*CTRL).s->sinT = sin(PSD.theta_excitation); // (*CTRL).i->theta_d_elec
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    #define CURRENT_COMMAND_VALUE (0.5*d_sim.init.IN)

    PID_iD->Ref = CURRENT_COMMAND_VALUE;
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iD->calc(PID_iD);
    (*CTRL).o->cmd_uAB[0] = MT2A(PID_iD->Out, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B(PID_iD->Out, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
    (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];

    if((*CTRL).timebase>2.0){
        (*CTRL).s->PSD_Done = TRUE;
        PSD.theta_excitation = 0.0;
        PSD.theta_d_elec_entered = 0.0;
        PSD.countEntered = 0;
    }else if((*CTRL).timebase>1.75){ // EXCITE_BETA_AXIS==TRUE: 1.75 else 1.5
        // Reverse the rotation of current vector
        PSD.theta_excitation -= CL_TS*1.0*M_PI; // EXCITE_BETA_AXIS==TRUE: -CL_TS* else CL_TS*
    }else if((*CTRL).timebase>1){
        // Rotate current vector
        PSD.theta_excitation += CL_TS*1.0*M_PI; // EXCITE_BETA_AXIS==TRUE: -CL_TS* else CL_TS*

        // 第一次进来
        if(PSD.countEntered++==0){
            // 以当前位置角为d轴位置进行初始位置补偿。
            ENC.theta_d_offset = - ENC.theta_d__state;
            PSD.theta_d_elec_entered = 0.0;
        }

        // 转到中途的时候，在开始反转之前，赶快判断一下旋转的方向
        if(fabsf((*CTRL).timebase-1.49)<CL_TS){
            if( (*CTRL).i->theta_d_elec > PSD.theta_d_elec_entered)
                PSD.direction = 1;
            else{
                PSD.direction = -1;
            }
        }
    }else{
        // Excite constant current vector and wait for alignment
    }
}


/* Main */
void commissioning(){
    /* PSD */
    (*CTRL).s->PSD_Done = TRUE;
    if((*CTRL).s->PSD_Done == FALSE){
        doPSD();
        return;
    // }else{
    //     (*CTRL).o->cmd_uAB_to_inverter[0] = 0;
    //     (*CTRL).o->cmd_uAB_to_inverter[1] = 0;
    }

    /* IPD */
    // if((*CTRL).s->IPD_Done == FALSE){
    //     doIPD();
    //     inverter_voltage_command();
    //     return;
    // }
    /* SC */
    // 帕克变换（使用反馈位置）
    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec); // (*CTRL).i->theta_d_elec
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec); // (*CTRL).i->theta_d_elec
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    // 参数自整定
    StepByStepCommissioning_NEW_WB();
    _user_inverter_voltage_command(0);
}




// 声明参数自整定结构体变量
struct CommissioningDataStruct COMM;
int16 bool_single_phase_excitation = FALSE;
int16 bool_tuning = TRUE;
int16 bool_CL_tuned = FALSE;
int16 bool_VL_tuned = FALSE;
int16 bool_PL_tuned = FALSE;


#define UD_OUTPUT (*CTRL).o->cmd_uAB[0]
#define UQ_OUTPUT (*CTRL).o->cmd_uAB[1]


// 自整定初始化
void init_COMM(){
    COMM.timebase = 0.0;

    // KE
    #if PC_SIMULATION
        printf("From name plate data: %g Nm, %g Nm/A, %g Vs, %g mV/rpm\n", MOTOR_RATED_TORQUE, MOTOR_TORQUE_CONSTANT, d_sim.init.KE, MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM);
    #endif
    COMM.KE = d_sim.init.KE;

    // R
    COMM.current_sum = 0.0;
    COMM.voltage_sum = 0.0;
    COMM.counterSS = 0;
    COMM.bool_collecting = FALSE;
    int i;
    for(i=0;i<COMM_IV_SIZE_R1;++i){
        COMM.i_phase_data_R1[i] = 0.0;
        COMM.v_phase_data_R1[i] = 0.0;
    }
    for(i=0;i<COMM_IV_SIZE_L1;++i){
        COMM.i_data_L1[i] = 0.0;
        COMM.t_data_L1[i] = 0.0;
    }
    COMM.inverter_voltage_drop = 0.0;

    // L
    COMM.L = 0;
    COMM.id_prev = 0.0;
    COMM.iq_prev = 0.0;

    // Js
    COMM.number_of_repeats_Js = 0.0;

    // global status vairbale
    #if WHO_IS_USER == USER_WB
        COMM.bool_comm_status = d_sim.user.COMM_bool_comm_status;
    #else
        COMM.bool_comm_status = 4;
    #endif

    
    COMM.counterEntered = 0;
    COMM.i = 0;
}

// SS: Steady State
#define SS_RATED_CURRENT_RATIO 1e-2 // 电流误差在额定电流的 0.01 倍以下时，认为 有可能 达到电流调节稳态了
#define SS_COUNTER_CEILING ((long int)(0.2/CL_TS)) // 达到稳态后，计数，超过这个值才认为真正进入稳态
int reachSteadyStateCurrent(REAL current_error, REAL rated_current){
    static long int counterSS = 0;

    if(fabsf(current_error) < rated_current * SS_RATED_CURRENT_RATIO){
        counterSS += 1;

        // Avoid to collect over-shoot data
        if(counterSS > SS_COUNTER_CEILING){
            counterSS = 0;
            return TRUE; // 目前是一旦判断为稳态，就永远返回TRUE。当然，也可以设计成回差的形式，达到稳态后还要判断是否脱离稳态。
        }
    }
    return FALSE;
}
#define SQUARE(x) (x*x)
int linreg(int n, const REAL x[], const REAL y[], REAL* m, REAL* b, REAL* r){
    /*  
        n = number of data points
        x,y  = arrays of data
        *b = output intercept
        *m = output slope
        *r = output correlation coefficient (can be NULL if you don't want it)
    */
    REAL   sumx = 0.0;                      /* sum of x     */
    REAL   sumx2 = 0.0;                     /* sum of x**2  */
    REAL   sumxy = 0.0;                     /* sum of x * y */
    REAL   sumy = 0.0;                      /* sum of y     */
    REAL   sumy2 = 0.0;                     /* sum of y**2  */

    int i;
    for (i=0;i<n;i++){ 
        sumx  += x[i];       
        sumx2 += SQUARE(x[i]);  
        sumxy += x[i] * y[i];
        sumy  += y[i];      
        sumy2 += SQUARE(y[i]); 
    } 

    REAL denom = (n * sumx2 - SQUARE(sumx));
    if (denom == 0) {
        // singular matrix. can't solve the problem.
        *m = 0;
        *b = 0;
        if (r) 
            *r = 0;
        return 1;
    }

    *m = (   n * sumxy  -  sumx * sumy) / denom;
    *b = (sumy * sumx2  -  sumx * sumxy) / denom;
    // if (r!=NULL) {
    //     *r = (sumxy - sumx * sumy / n) /     compute correlation coeff 
    //           SQUARE((sumx2 - SQUARE(sumx)/n) *
    //           (sumy2 - SQUARE(sumy)/n));
    // }

    // REAL slope = ((n * sumxy) - (sumx * sumy )) / denom;
    // REAL intercept = ((sumy * sumx2) - (sumx * sumxy)) / denom;
    REAL term1 = ((n * sumxy) - (sumx * sumy));
    REAL term2 = ((n * sumx2) - (sumx * sumx));
    REAL term3 = ((n * sumy2) - (sumy * sumy));
    REAL term23 = (term2 * term3);
    if (fabsf(term23) > 1e-6)
        *r = (term1 * term1) / term23; // Excel returns r*r.
    
    return 0; 
}



void COMM_PI_tuning(REAL LL, REAL RR, REAL BW_current, REAL delta, REAL JJ, REAL KE, REAL npp){

    // Current loop tuning
    PID_iD->Kp = LL * BW_current;
    PID_iQ->Kp = LL * BW_current;
    // pid1_ia.Kp = LL * BW_current;
    // pid1_ib.Kp = LL * BW_current;
    // pid1_ic.Kp = LL * BW_current;

    PID_iD->Ki_CODE = PID_iD->Kp * RR/LL * CL_TS; 
    PID_iQ->Ki_CODE = PID_iQ->Kp * RR/LL * CL_TS; // R=Rs+Rr for q axis
    // pid1_ia.Ki = pid1_ia.Kp * RR/LL * CL_TS;
    // pid1_ib.Ki = pid1_ib.Kp * RR/LL * CL_TS;
    // pid1_ic.Ki = pid1_ic.Kp * RR/LL * CL_TS;

    REAL K = npp/JJ * CLARKE_TRANS_TORQUE_GAIN*npp*KE;
    REAL BW_speed = BW_current / ( delta + 2.16*exp(-delta/2.8) - 1.86 ); // [rad/s]

    REAL lowest_pole_over_0dB = BW_current;
    // REAL lowest_pole_over_0dB = 2*3.1415926 * 150; // 5 Hz The feedback frequency of hall sensor at 10 rpm is near 5 Hz.

    // My tuning
    // PID_Speed->Ki_CODE = lowest_pole_over_0dB / (delta*delta) * VL_TS; 

    PID_Speed->Kp = lowest_pole_over_0dB / (delta) / K;

    // TI's tuning
    PID_Speed->Ki_CODE = PID_Speed->Kp * lowest_pole_over_0dB / (delta*delta) * VL_TS; 

    #if PC_SIMULATION
    printf("Current loop, BW_current=%.1f Hz, Kp=%g, KpKi=%g\n", BW_current/(2*3.1415926), PID_iD->Kp,  PID_iQ->Ki_CODE*CL_TS);
    printf("Speed loop,   BW_speed=%.1f Hz,   Kp=%g, KpKi=%g, K=%g\n",      BW_speed/(2*3.1415926),   PID_Speed->Kp, PID_Speed->Ki_CODE*VL_TS, K);
    #endif
}

// 1：电阻辨识事件
void COMM_resistanceId(REAL id_fb, REAL iq_fb){
    ++COMM.counterEntered;
    if(COMM.counterEntered<=1){
        // Current loop tuning (LL, RR, BW_c, delta, JJ, KE, npp)
        COMM_PI_tuning(d_sim.init.Ld, d_sim.init.R, 2*3.1415926*CL_TS_INVERSE * 0.025, 20, d_sim.init.Js, d_sim.init.KE, d_sim.init.npp);
            // COMM_PI_tuning(7e-3, 0.8, 2*3.1415926*CL_TS_INVERSE * 0.025, 20, MOTOR_SHAFT_INERTIA, MOTOR_BACK_EMF_CONSTANT, MOTOR_NUMBER_OF_POLE_PAIRS);
    }

    #define RS_ID_NUMBER_OF_STAIRS 20 // 50
    #define RS_ID_MAXIMUM_CURRENT (1.0*d_sim.init.IN)
    REAL current_increase_per_step = RS_ID_MAXIMUM_CURRENT / RS_ID_NUMBER_OF_STAIRS;

    COMM.current_command = RS_ID_MAXIMUM_CURRENT;
    COMM.current_command -= current_increase_per_step*COMM.i;

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        #define REGULATOR PID_iQ
        #define FEEDBACK  iq_fb
    #else
        #define REGULATOR PID_iD
        #define FEEDBACK  id_fb
    #endif

    REGULATOR->Fbk = FEEDBACK;
    REGULATOR->Ref = COMM.current_command; // 改为从正到负
    REGULATOR->calc(REGULATOR);

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        UD_OUTPUT = 0.0;
        UQ_OUTPUT = REGULATOR->Out;
    #else
        UD_OUTPUT = REGULATOR->Out;
        UQ_OUTPUT = 0.0;
    #endif

    // collect steady state data
    if(COMM.bool_collecting){
        #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
            COMM.current_sum += iq_fb;
            COMM.voltage_sum += UQ_OUTPUT;
        #else
            COMM.current_sum += id_fb;
            COMM.voltage_sum += UD_OUTPUT;
        #endif
        COMM.counterSS += 1;

        if(COMM.counterSS>800){
            COMM.i_phase_data_R1[COMM.i] = COMM.current_sum/(REAL)COMM.counterSS * -SIN_DASH_2PI_SLASH_3; // beta -> phase b
            COMM.v_phase_data_R1[COMM.i] = COMM.voltage_sum/(REAL)COMM.counterSS * -SIN_DASH_2PI_SLASH_3; // beta -> phase b
            #if PC_SIMULATION
                printf("%f, %f\n", COMM.i_phase_data_R1[COMM.i], COMM.v_phase_data_R1[COMM.i]);
            #endif
            COMM.bool_collecting = FALSE;
            ++COMM.i;
        }
    }else{
        // reset
        COMM.current_sum = 0.0;
        COMM.voltage_sum = 0.0;
        COMM.counterSS = 0;

        // check steady state and assign boolean variable
        if(reachSteadyStateCurrent(REGULATOR->Ref-REGULATOR->Fbk, d_sim.init.IN)){

            COMM.bool_collecting = TRUE;

            if(fabsf(COMM.current_command) > RS_ID_MAXIMUM_CURRENT){

                // Get resistance value
                if(FALSE){
                    COMM.R = (COMM.v_phase_data_R1[1] - COMM.v_phase_data_R1[0]) / (COMM.i_phase_data_R1[1] - COMM.i_phase_data_R1[0]);
                }else{
                    REAL m,b,r;
                    if(linreg(( int)(0.3*RS_ID_NUMBER_OF_STAIRS), COMM.i_phase_data_R1, COMM.v_phase_data_R1, &m, &b, &r)){
                        #if PC_SIMULATION
                            printf("FAILURE\n");
                        #endif
                    }
                    COMM.R = m;
                    COMM.inverter_voltage_drop = b;
                }
                #if PC_SIMULATION
                    printf("R=%g Ohm, inverter_voltage_drop=%g\n", COMM.R, COMM.inverter_voltage_drop);
                    int j;
                    printf("---CURRENT LIST:\n");
                    for(j=0;j<COMM_IV_SIZE_R1;++j){
                        printf("\t%g,\n", COMM.i_phase_data_R1[j]);
                    }
                    printf("---VOLTAGE LIST:\n");
                    for(j=0;j<COMM_IV_SIZE_R1;++j){
                        printf("\t%g,\n", COMM.v_phase_data_R1[j]);
                    }
                #endif
                #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
                    COMM.last_voltage_command = UQ_OUTPUT;
                #else
                    COMM.last_voltage_command = UD_OUTPUT;
                #endif
                COMM.bool_comm_status = 2;
                COMM.counterEntered = 0;

                // UD_OUTPUT = 0.0;
                // UQ_OUTPUT = 0.0;
            }
        }
    }

    #undef RS_ID_NUMBER_OF_STAIRS
    #undef RS_ID_MAXIMUM_CURRENT 
    #undef REGULATOR
    #undef FEEDBACK
}
// 1a: 逆变器特性辨识
void COMM_resistanceId_v2(REAL id_fb, REAL iq_fb){
    ++COMM.counterEntered;
    if(COMM.counterEntered<=1){
        // Current loop tuning (LL, RR, BW_c, delta, JJ, KE, npp)
        COMM_PI_tuning(d_sim.init.Ld, d_sim.init.R, 2*3.1415926*CL_TS_INVERSE * 0.025, 20, d_sim.init.Js, d_sim.init.KE, d_sim.init.npp);
     // COMM_PI_tuning(7e-3, 0.8, 2*3.1415926*CL_TS_INVERSE * 0.025, 20, MOTOR_SHAFT_INERTIA, MOTOR_BACK_EMF_CONSTANT, MOTOR_NUMBER_OF_POLE_PAIRS);
    }
    // number of stairs (positive or negative current)
    #define NOS_II (29)
    #define NOS    (COMM_IV_SIZE_R1/2-1-NOS_II) // 200/2-30 = 70
    #define RS_ID_MAXIMUM_CURRENT (d_sim.init.IN) /* [A] This specifies beta-axis current, so we need to make it larger so that the phase current is large enough This is motor related (to identify accurate resistance) */
    #ifdef _XCUBE1
        #define RS_ID_LOW_CURRENT (0.3)
    #else
        #define RS_ID_LOW_CURRENT (1.0) /* [A] This is inverter related */ 
    #endif
    // REAL current_increase_per_step_equalStepSize = RS_ID_MAXIMUM_CURRENT / (REAL)NOS;
    REAL Delta_I_Low  = RS_ID_LOW_CURRENT / (REAL)NOS;
    REAL Delta_I_High = (RS_ID_MAXIMUM_CURRENT-RS_ID_LOW_CURRENT) / (REAL)NOS_II;

    if(COMM.i < NOS_II){
        /* 正电流-高电流段 */
        COMM.current_command = RS_ID_MAXIMUM_CURRENT;
        COMM.current_command -= Delta_I_High*COMM.i;
    }else if(COMM.i < NOS+NOS_II){
        /* 正电流-低电流段 */
        COMM.current_command = RS_ID_MAXIMUM_CURRENT;
        COMM.current_command -= Delta_I_High* NOS_II \
                              + Delta_I_Low * (COMM.i-NOS_II);
    }else if(COMM.i < NOS+2*NOS_II){
        /* 负电流-高电流段 */
        COMM.current_command = -RS_ID_MAXIMUM_CURRENT;
        COMM.current_command += Delta_I_High*(COMM.i-NOS-NOS_II);
    }else{
        /* 负电流-低电流段 */
        COMM.current_command = -RS_ID_MAXIMUM_CURRENT;
        COMM.current_command += Delta_I_High* NOS_II \
                              + Delta_I_Low * (COMM.i-NOS-2*NOS_II);
    }

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        #define REGULATOR PID_iQ
        #define FEEDBACK  iq_fb
    #else
        #define REGULATOR PID_iD
        #define FEEDBACK  id_fb
    #endif
    REGULATOR->Fbk = FEEDBACK;
    REGULATOR->Ref = COMM.current_command;
    if(G.FLAG_TUNING_CURRENT_SCALE_FACTOR)
        REGULATOR->Ref = 3;
    REGULATOR->calc(REGULATOR);

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        UD_OUTPUT = 0.0;
        UQ_OUTPUT = REGULATOR->Out;

        // for ParkSul2012
        (*CTRL).i->cmd_iDQ[0] = 0.0;
        (*CTRL).i->cmd_iDQ[1] = REGULATOR->Ref;
    #else
        UD_OUTPUT = REGULATOR->Out;
        UQ_OUTPUT = 0.0;

        // for ParkSul2012
        (*CTRL).i->cmd_iDQ[0] = REGULATOR->Ref;
        (*CTRL).i->cmd_iDQ[1] = 0.0;
    #endif
    // collect steady state data
    if(COMM.bool_collecting){
        #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
            COMM.current_sum += iq_fb;
            COMM.voltage_sum += UQ_OUTPUT;
        #else
            COMM.current_sum += id_fb;
            COMM.voltage_sum += UD_OUTPUT;
        #endif
        COMM.counterSS += 1;
        if(COMM.counterSS>800){
            COMM.i_phase_data_R1[COMM.i] = COMM.current_sum/(REAL)COMM.counterSS * -SIN_DASH_2PI_SLASH_3;
            COMM.v_phase_data_R1[COMM.i] = COMM.voltage_sum/(REAL)COMM.counterSS * -SIN_DASH_2PI_SLASH_3;
            #if PC_SIMULATION
                printf("I=%f, U=%f\n", COMM.i_phase_data_R1[COMM.i], COMM.v_phase_data_R1[COMM.i]);
            #endif
            COMM.bool_collecting = FALSE;
            ++COMM.i;
        }
        // printf("COMM.bool_collecting=%f\n", COMM.bool_collecting);
    }else{
        // reset
        COMM.current_sum = 0.0;
        COMM.voltage_sum = 0.0;
        COMM.counterSS = 0;
        // check steady state and assign boolean variable
        if(reachSteadyStateCurrent(REGULATOR->Ref-REGULATOR->Fbk, RS_ID_MAXIMUM_CURRENT)){
            COMM.bool_collecting = TRUE;
            /* 判断是否结束 */
            // if(fabsf(COMM.current_command) > RS_ID_MAXIMUM_CURRENT){
            if(COMM.i>=NOS*2+NOS_II*2){
                // Get resistance value
                if(FALSE){
                    /* use two points */
                    COMM.R = (COMM.v_phase_data_R1[1] - COMM.v_phase_data_R1[0]) / (COMM.i_phase_data_R1[1] - COMM.i_phase_data_R1[0]);
                }else{
                    /* least square fitting */
                    REAL m,b,r;
                    if(linreg(( int)(1.0*NOS_II), COMM.i_phase_data_R1, COMM.v_phase_data_R1, &m, &b, &r)){
                        #if PC_SIMULATION
                            printf("FAILURE\n");
                        #endif
                    }
                    COMM.R = m;
                    COMM.inverter_voltage_drop = b;
                }
                #if PC_SIMULATION
                    printf("R=%g Ohm, inverter_voltage_drop=%g\n", COMM.R, COMM.inverter_voltage_drop);
                    int j;
                    printf("---PHASE B CURRENT LIST:\n");
                    for(j=0;j<COMM_IV_SIZE_R1;++j){
                        printf("\t%g,\n", COMM.i_phase_data_R1[j] );
                    }
                    printf("---PHASE B VOLTAGE LIST:\n");
                    for(j=0;j<COMM_IV_SIZE_R1;++j){
                        printf("\t%g,\n", COMM.v_phase_data_R1[j] );
                    }
                #endif
                /* 无意义 */
                #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
                    COMM.last_voltage_command = UQ_OUTPUT;
                #else
                    COMM.last_voltage_command = UD_OUTPUT;
                #endif
                // 由于此时电流是零，last_voltage_command也是零，就不能做电感辨识的开环激励辨识了，直接结束吧。
                COMM.bool_comm_status = -1; 
                COMM.counterEntered = 0;

                // UD_OUTPUT = 0.0;
                // UQ_OUTPUT = 0.0;
            }
        }
    }

    #undef NOS
    #undef NOS_II
    #undef RS_ID_MAXIMUM_CURRENT
    #undef RS_ID_LOW_CURRENT
    #undef REGULATOR
    #undef FEEDBACK
}

// 2/3：电感辨识事件
void COMM_inductanceId(REAL id_fb, REAL iq_fb){
    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        #define FEEDBACK  iq_fb
    #else
        #define FEEDBACK  id_fb
    #endif

    // Collect current data for curve-fitting
    if(COMM.counterEntered >= 0 && COMM.counterEntered < COMM_IV_SIZE_L1){
        COMM.i_data_L1[COMM.counterEntered] = FEEDBACK; // COMM.counterEntered; //
        COMM.t_data_L1[COMM.counterEntered] = COMM.counterEntered*CL_TS;
        // printf("|||||%d, %g\n", COMM.counterEntered, COMM.i_data_L1[COMM.counterEntered]);
    }

    ++COMM.counterEntered;
    REAL Delta_IS_al;
    REAL Delta_US_al;
    REAL IS_slope;

    Delta_US_al = COMM.last_voltage_command;
    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        Delta_IS_al = iq_fb - COMM.iq_prev;
    #else
        Delta_IS_al = id_fb - COMM.id_prev;
    #endif
    IS_slope = Delta_IS_al / CL_TS;
    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        UD_OUTPUT = 0.0;
        UQ_OUTPUT = COMM.last_voltage_command + Delta_US_al;
    #else
        UD_OUTPUT = COMM.last_voltage_command + Delta_US_al;
        UQ_OUTPUT = 0.0;
    #endif

    // printf("Last = %g, UD_OUTPUT = %g \n", COMM.last_voltage_command, UD_OUTPUT);

    #define NUMBER_OF_SAMPLES (20)
    #define NUMBER_OF_SAMPLES_USED_IN_FITTING (20)


    REAL L_temp;
    if(COMM.counterEntered < NUMBER_OF_SAMPLES){
        L_temp = Delta_US_al / IS_slope;
        #if PC_SIMULATION
            printf("L=%g, %g\n", L_temp, IS_slope );
        #endif
        if(L_temp < COMM.L && L_temp > 0.0){
            COMM.L = L_temp;
        }
    }else{
        UD_OUTPUT = 0.0;
        UQ_OUTPUT = 0.0;

        REAL m,b,r;
        if(linreg(NUMBER_OF_SAMPLES_USED_IN_FITTING, COMM.t_data_L1, COMM.i_data_L1, &m, &b, &r)){
            #if PC_SIMULATION
                printf("FAILURE\n");
            #endif
        }
        COMM.L = Delta_US_al / m;
        #if PC_SIMULATION
            // printf("[Ideal] m=-423.301, b=1.92252, r=0.999765\n");
            printf("[Actual]  m=%g, b=%g, r=%g\n", m,b,r);
            printf("COMM.L = %g\n", COMM.L);
        #endif
        COMM.bool_comm_status = 3;
        COMM.counterEntered = 0;
    }

    COMM.id_prev = id_fb;
    COMM.iq_prev = iq_fb;                
}
#define COMM_FAST_SWITCH_MOD 5 // > 10 does not work
#define COMM_FAST_SWITCH_VOLTAGE_CHANGE (0.5*COMM.last_voltage_command)
void COMM_inductanceId_ver2(REAL id_fb, REAL iq_fb){    

    REAL id_avg = 0.0;
    REAL Delta_current = 0.0;
    int16 number_of_repeats_L2 = 0;

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        #define REGULATOR PID_iQ
    #else
        #define REGULATOR PID_iD
    #endif

    COMM.counterEntered += 1;
    if(COMM.counterEntered == COMM_FAST_SWITCH_MOD+1 || COMM.counterEntered == 1){
        #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
            id_avg = id_fb + COMM.id_prev; /* Q：求平均不需要乘以0.5吗？ */
        #else
            id_avg = id_fb + COMM.id_prev; /* Q：求平均不需要乘以0.5吗？ */
        #endif

        REGULATOR->Fbk = id_avg;
        REGULATOR->Ref = COMM.current_command;
        REGULATOR->calc(REGULATOR);

        #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
            Delta_current = fabsf(iq_fb - COMM.iq_prev);
        #else
            Delta_current = fabsf(id_fb - COMM.id_prev);
        #endif
        COMM.L = COMM_FAST_SWITCH_MOD*CL_TS * COMM_FAST_SWITCH_VOLTAGE_CHANGE / (1*Delta_current);

        COMM.id_prev = id_fb;
        COMM.iq_prev = iq_fb;
    }

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        UD_OUTPUT = 0.0;
        UQ_OUTPUT = COMM.last_voltage_command; // REGULATOR.Out;
    #else
        UD_OUTPUT = COMM.last_voltage_command; // REGULATOR.Out;
        UQ_OUTPUT = 0.0;
    #endif

    if(COMM.counterEntered <= COMM_FAST_SWITCH_MOD){
        #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
            UQ_OUTPUT = REGULATOR->Out + COMM_FAST_SWITCH_VOLTAGE_CHANGE;
        #else
            UD_OUTPUT = REGULATOR->Out + COMM_FAST_SWITCH_VOLTAGE_CHANGE;
        #endif
    }else{
        #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
            UQ_OUTPUT = REGULATOR->Out - COMM_FAST_SWITCH_VOLTAGE_CHANGE;
        #else
            UD_OUTPUT = REGULATOR->Out - COMM_FAST_SWITCH_VOLTAGE_CHANGE;
        #endif
        if(COMM.counterEntered == COMM_FAST_SWITCH_MOD*2){
            COMM.counterEntered = 0;
            #if PC_SIMULATION
                printf("L=%g\n", COMM.L);
            #endif
            number_of_repeats_L2 += 1;
            if(number_of_repeats_L2 > 500){
                COMM.bool_comm_status = 3;
                number_of_repeats_L2 = 0;
            }
        }
    }
    #undef REGULATOR
    #undef FEEDBACK
}
#define N_SAMPLE ((long int)(12.5/CL_TS)) // Resolution 0.1 Hz = 1 / (N_SAMPLE * TS)
void COMM_inductanceId_ver3(REAL id_fb, REAL iq_fb){

    // 要求分辨率达到0.1=1/N_SAMPLE/CL_TS // 目标频率应该选择使得能够被数据的长度所分辨的值，也就是0.1的整数倍即可，比如4.6Hz。
    // Rreq结果：0.1Hz（要求N_SAMPLE为100000，分辨率为0.1Hz）偏大，4.5Hz（要求N_SAMPLE为100000，分辨率为0.1Hz）刚好，10Hz偏小
    // REAL omegaL = 2*3.1415926*2;

    REAL omegaL = 2*3.1415926*2; // 2 Hz 电机会抖起来，加高频率看看

    static REAL mz_timebase = 0;
    mz_timebase = -CL_TS;
    mz_timebase += CL_TS;   

    REAL Ireal, Iimag;
    REAL cdCos, cdSin; // Coherent Demodulation

    static REAL temp_accumCosL=0.0;
    static REAL temp_accumSinL=0.0;

    static REAL dc_part_CosL = 0.0;
    static REAL dc_part_SinL = 0.0;

    #define HORY_MZ2014_AMPL_LOW (1.0*COMM.last_voltage_command) // 0.50, 0.75
    #define HORY_MZ2014_DC_BIAS (1*HORY_MZ2014_AMPL_LOW) //(2*HORY_MZ2014_AMPL_LOW)
    #define HORY_MZ2014_SAMPLE_DUTY 0.8 // 不考虑三大现实问题时，取1和0.8和0.6没有本质差别；

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        #define FEEDBACK  iq_fb
    #else
        #define FEEDBACK  id_fb
    #endif

    #if EXCITE_BETA_AXIS_AND_MEASURE_PHASE_B
        // UQ_OUTPUT = 1.0*HORY_MZ2014_AMPL_LOW*cos(omegaL*mz_timebase);
        // UQ_OUTPUT = 1.0*HORY_MZ2014_AMPL_LOW*cos(omegaL*mz_timebase) + 1.0*HORY_MZ2014_AMPL_LOW;
        UQ_OUTPUT = HORY_MZ2014_AMPL_LOW*cos(omegaL*mz_timebase) + 0.0*HORY_MZ2014_DC_BIAS;
    #else
        // 注意电阻辨识完以后，转子位置仍处于最开始的0度，这个时候只能往正方向加大电流，否则震荡
        UD_OUTPUT = HORY_MZ2014_AMPL_LOW*cos(omegaL*mz_timebase);
        // UD_OUTPUT = -1.0*HORY_MZ2014_AMPL_LOW + 0.5*HORY_MZ2014_AMPL_LOW*cos(omegaL*mz_timebase);
        // UQ_OUTPUT = 0.0;
    #endif

    if(mz_timebase>=(1.0-HORY_MZ2014_SAMPLE_DUTY)*N_SAMPLE*CL_TS){ // 前20%认为含有暂态过程（磁链的周期还没跟上电压的）
        // 求平均值进行解调
        temp_accumCosL += FEEDBACK * cos(omegaL*mz_timebase);
        temp_accumSinL += FEEDBACK * sin(omegaL*mz_timebase);

        dc_part_CosL += FEEDBACK;
        dc_part_SinL += FEEDBACK;
    }

    // BUG: 这里转换整型数，一开始用的(int)，仿真可以，但是DSP中就出现进不去这个if语句的现象。
    if((long int)(mz_timebase/CL_TS+1.5) % N_SAMPLE==0){ // +1用于回避t=0时那一次，+0.5用于四舍五入

        cdCos = (temp_accumCosL - 4.16247*0) / (HORY_MZ2014_SAMPLE_DUTY*N_SAMPLE);
        cdSin = (temp_accumSinL - 4.16247*0) / (HORY_MZ2014_SAMPLE_DUTY*N_SAMPLE);
        temp_accumCosL = 0.0;
        temp_accumSinL = 0.0;

        #if PC_SIMULATION
            printf("DC Current in L Id: %g, %g\n", 
                    (dc_part_CosL) / (HORY_MZ2014_SAMPLE_DUTY*N_SAMPLE),
                    (dc_part_SinL) / (HORY_MZ2014_SAMPLE_DUTY*N_SAMPLE));
        #endif

        dc_part_CosL = 0.0;
        dc_part_SinL = 0.0;

        mz_timebase = -CL_TS; // 时基重置，方便进行SAMPLE_DUTY比较。

        Ireal = 2.0*cdCos;
        Iimag = -2.0*cdSin;

        // Compute R
        COMM.R3 = HORY_MZ2014_AMPL_LOW * Ireal / (Ireal*Ireal + Iimag*Iimag);

        // Compute L
        COMM.L3 = - HORY_MZ2014_AMPL_LOW * Iimag / (Ireal*Ireal + Iimag*Iimag) / (omegaL);

        #if PC_SIMULATION
            printf("COMM.L=%g\n", COMM.L);
            printf("Ireal**2+Iimag**2=%g\n", sqrt(Ireal*Ireal+Iimag*Iimag));
            printf("R3=%g, L3=%g, mztime=%g, triggertime=%g\n", COMM.R3, COMM.L3, mz_timebase, (1.0-HORY_MZ2014_SAMPLE_DUTY)*N_SAMPLE*CL_TS);
        #endif

        COMM.bool_comm_status = 4; // PM flux id
        COMM.bool_comm_status = 5; // inertia id
        COMM.counterEntered = 0;



        // REAL L_guess = 2*48 * 0.05 / (2*M_PI * MOTOR_RATED_SPEED_RPM/60.0*MOTOR_NUMBER_OF_POLE_PAIRS * MOTOR_RATED_CURRENT_RMS);
        // REAL R_guess = 2*48 * 0.01 / MOTOR_RATED_CURRENT_RMS;
        // printf("L_guess=%g, R_guess=%g\n", L_guess, R_guess);
        // getch();

        // Current loop tuning (LL, RR, BW_c, delta, JJ, KE, npp)
        COMM_PI_tuning(COMM.L3, COMM.R, 2*3.1415926*CL_TS_INVERSE * 0.025, 20, d_sim.init.Js, d_sim.init.KE, d_sim.init.npp);
        // COMM_PI_tuning(COMM.L3, COMM.R, 2*3.1415926*CL_TS_INVERSE * 0.025, 400, MOTOR_SHAFT_INERTIA, MOTOR_BACK_EMF_CONSTANT, MOTOR_NUMBER_OF_POLE_PAIRS);
        // COMM_PI_tuning(COMM.L3, COMM.R, 2*3.1415926*CL_TS_INVERSE * 0.1, 10, MOTOR_SHAFT_INERTIA, MOTOR_BACK_EMF_CONSTANT, MOTOR_NUMBER_OF_POLE_PAIRS);
        bool_CL_tuned = TRUE;
    }
}

// 4：永磁体磁链（反电势系数）辨识事件
REAL x1 = 0.0;
REAL SM_K = 1e-3;
REAL slidingModeControl(REAL error, REAL Tload_est){
    // #define SM_GAIN (MOTOR_RATED_TORQUE*0.2)
    // REAL iq_cmd = 1.0 / (CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*MOTOR_BACK_EMF_CONSTANT) * SM_GAIN * sign(S);

    // #define SM_K (500)  // 过大将导致转速抖动
    #define SM_DELTA 10 // 取值越大，趋近越快，但是其作用是非线性的？
    #define SM_VAREPSILON 0.1 // 转速误差很大的时候，k是常规滑模的1/SM_VAREPSILON倍（10倍）

    #define SM_C (0*0.01) // 过大导致转速抖动，过小存在稳态误差
    // static REAL x1 = 0.0;

    // REAL S = error;

    // 滑模面 = 转速误差 + SM_C*转速误差积分
    REAL S = error * (*CTRL).motor->npp + SM_C*x1;

    x1 += CL_TS*SPEED_LOOP_CEILING * (error) * (*CTRL).motor->npp;

    REAL P_output = SM_C * error * (*CTRL).motor->npp + ( 0.0 + SM_K / (SM_VAREPSILON + ( 1.0 + 1.0/fabsf(x1) - SM_VAREPSILON)*exp(-SM_DELTA*fabsf(S))) ) * sign(S);

    if(bool_tuning){
        if(error-1>0){
            SM_K += CL_TS*SPEED_LOOP_CEILING * 0.0005;
        }else if(error<0){
            bool_tuning = FALSE;
        }
    }

    // dynamic clamping
    if( x1 > SPEED_LOOP_LIMIT_NEWTON_METER - P_output)
        x1 = SPEED_LOOP_LIMIT_NEWTON_METER - P_output;
    else if( x1 < -SPEED_LOOP_LIMIT_NEWTON_METER - P_output)
        x1 = -SPEED_LOOP_LIMIT_NEWTON_METER - P_output;

    REAL acceleration_cmd = P_output;
    // x1这里不能额外乘系数哦，x1=I_Output
    // REAL torque_cmd = 0*Tload_est + 0*x1 + acceleration_cmd / (MOTOR_NUMBER_OF_POLE_PAIRS/MOTOR_SHAFT_INERTIA);
    REAL torque_cmd = 0*Tload_est + 0*x1 + acceleration_cmd;
    return torque_cmd;
}
REAL omg_elec_est = 0.0;
REAL Tload_est = 0.0;
REAL smo_eta = 100;
REAL smo_gamma = 2;
REAL slidingModeObserver(REAL omg_mech_fb, REAL mu_m, REAL iq_fb){
    REAL est_error = omg_mech_fb * (*CTRL).motor->npp - omg_elec_est;
    omg_elec_est += CL_TS * mu_m * ( d_sim.init.npp*d_sim.init.KE*iq_fb - Tload_est + smo_eta*sign(est_error) );
    Tload_est += - CL_TS * smo_gamma * smo_eta * sign(est_error);
    return Tload_est;
}
// #define RPM_2_ELEC_RAD_PER_SEC ( (2*3.1415926*MOTOR_NUMBER_OF_POLE_PAIRS)/60.0 )
REAL rpm_speed_command = 300;
// REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv){
//     return y_tminus1 + CL_TS * time_const_inv * (x - y_tminus1);
// }
REAL filtered_voltage = 0.0;
REAL filtered_current = 0.0;
REAL filtered_omg_mech_fb = 0.0;
void COMM_PMFluxId(REAL id_fb, REAL iq_fb, REAL omg_mech_fb){

    COMM.counterEntered += 1;
    // if(COMM.counterEntered==1){
    //     // delta =10
    //     PID_Speed->Kp = 1e-6 * (CL_TS_INVERSE*0.1) / 10.0;
    //     PID_Speed->Ki_CODE = PID_Speed->Kp * (CL_TS_INVERSE*0.1) / 100.0 * (CL_TS*SPEED_LOOP_CEILING);
    // }

    PID_iD->Ref = 0.0;

    // if((*CTRL).timebase>20){
    //     rpm_speed_command = 5;
    // }

    (*CTRL).i->cmd_varOmega = rpm_speed_command * RPM_2_MECH_RAD_PER_SEC;
    REAL error = (*CTRL).i->cmd_varOmega - omg_mech_fb;
    // (*CTRL).speed_ctrl_err = error;
    // printf("%g: %g\n", (*CTRL).timebase, error);

    if(TRUE){ // PI Speed Regulator

        static int pmsm_comm_vc_count = 0;
        if(pmsm_comm_vc_count++ == SPEED_LOOP_CEILING){
            pmsm_comm_vc_count = 0;
            bool_tuning = FALSE;
            // if(bool_tuning){
            //     if(error-0.5>0){
            //         PID_Speed->Kp += CL_TS*SPEED_LOOP_CEILING * 0.0005;
            //     }else if(error-0.5<0){
            //         bool_tuning = FALSE;
            //     }
            // }

            PID_Speed->Ref = (*CTRL).i->cmd_varOmega;
            PID_Speed->Fbk = omg_mech_fb;
            PID_Speed->calc(PID_Speed);
            PID_iQ->Ref = PID_Speed->Out;
            // PID_iQ->Ref = Veclocity_Controller((*CTRL).i->cmd_varOmega, omg_mech_fb);
            // REAL torque_cmd = PID_Speed->Out;
            // PID_iQ->Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*MOTOR_BACK_EMF_CONSTANT);
            // if(PID_iQ->Ref > MOTOR_RATED_CURRENT_RMS){
            //     PID_iQ->Ref = MOTOR_RATED_CURRENT_RMS;
            // }else if(PID_iQ->Ref < -MOTOR_RATED_CURRENT_RMS){
            //     PID_iQ->Ref = -MOTOR_RATED_CURRENT_RMS;
            // }
            // printf("KT=%g\n", CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*MOTOR_BACK_EMF_CONSTANT);
            // getch();
        }
    }else{ // PI SMC

        REAL Tload_est = slidingModeObserver(omg_mech_fb, d_sim.init.npp/d_sim.init.KE, iq_fb);

        static int sm_vc_count = 0;
        if(sm_vc_count++ == SPEED_LOOP_CEILING){
            // velocity control loop execution frequency is 40 times slower than current control loop execution frequency
            sm_vc_count = 0;

            REAL torque_cmd = slidingModeControl(error, Tload_est);

            if(torque_cmd > SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = SPEED_LOOP_LIMIT_NEWTON_METER;
            }else if(torque_cmd < -SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = -SPEED_LOOP_LIMIT_NEWTON_METER;
            }
            PID_iQ->Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*d_sim.init.npp*d_sim.init.KE);
        }
    }

    // put this before _lpf(...)
    PID_iD->Fbk = id_fb;
    PID_iQ->Fbk = iq_fb;

    // put this before PID_iQ->calc(PID_iQ);
    filtered_voltage     = _lpf(PID_iQ->Out, filtered_voltage, 1);
    filtered_current     = _lpf(PID_iQ->Fbk, filtered_current, 1);
    filtered_omg_mech_fb = _lpf(omg_mech_fb, filtered_omg_mech_fb, 1);

    // voltage output
    PID_iD->calc(PID_iD);
    PID_iQ->calc(PID_iQ);
    UD_OUTPUT = MT2A(PID_iD->Out, PID_iQ->Out, (*CTRL).s->cosT, (*CTRL).s->sinT);
    UQ_OUTPUT = MT2B(PID_iD->Out, PID_iQ->Out, (*CTRL).s->cosT, (*CTRL).s->sinT);


    (*CTRL).o->cmd_iAB[0] = MT2A(id_fb, iq_fb, (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_iAB[1] = MT2B(id_fb, iq_fb, (*CTRL).s->cosT, (*CTRL).s->sinT);
    static REAL last_KE;
    last_KE = COMM.KE;
    if(fabsf(omg_mech_fb)>1e-3){
        // 有除法的地方就要小心分母为零（1.#INF）：
            // x0(id)[A],x1(iq)[A],x2(speed)[rpm],x3(position)[rad],ud[V],uq[V],IS_C(0),(*CTRL).ual,ACM.ual,ACM.theta_d,DIST_AL,COMM.KE,COMM.Js
            // 0.712084,0.0339154,0.21666,1.51514e-006,-19.4698,0.47952,0.712084,-19.4698,-19.4698,1.51514e-006,0,1.#INF,0
        COMM.KE = (filtered_voltage - COMM.R*filtered_current) * (*CTRL).motor->npp_inv /  filtered_omg_mech_fb - COMM.L3 * PID_iD->Ref;
        // COMM.KE *= sqrt(CLARKE_TRANS_TORQUE_GAIN); // 想要在恒幅值变换下拿到和恒功率变换一样的永磁体磁链值，就要乘以这个系数。
    }
    if(COMM.KE>3*d_sim.init.KE){
        COMM.KE = 3*d_sim.init.KE;
    }else if(COMM.KE<0){
        COMM.KE = 0.0;
    }

    static REAL accumKE = 0.0;
    static long int countKE = 0;

    if(bool_tuning==FALSE){
        // KE便是指是否达到稳态？
        if(fabsf(COMM.KE - last_KE)>=0.5e-2){
            COMM.counterEntered = 0; // 复用一下这个计数器用于判断KE是否收敛，可以少声明一个计数器。
        }else{ 
            // printf("%d, KE=%g, err=%g, accumKE=%g, countKE=%d\n", COMM.counterEntered, COMM.KE, error, accumKE, countKE);
        }

        // 转速控制是否达到稳态？
        if(fabsf(error)<5e-1 && omg_mech_fb*MECH_RAD_PER_SEC_2_RPM>10){
            if(COMM.counterEntered < 60000){

            }else if(COMM.counterEntered < 120000){
                accumKE += COMM.KE;
                countKE += 1;

            }else{
                COMM.KE = accumKE / (REAL)countKE;

                #if PC_SIMULATION
                    printf("COMM.KE=%g | ACM.KE=%g | PMSM_PERMANENT_MAGNET_FLUX_LINKAGE=%g | MOTOR_BACK_EMF_CONSTANT=%g\n", COMM.KE, ACM.KE, PMSM_PERMANENT_MAGNET_FLUX_LINKAGE, d_sim.init.KE);
                #endif
                // getch();

                COMM.bool_comm_status = 6; // end
                COMM.counterEntered = 0;

                accumKE = 0;
                countKE = 0;

            }
        }
    }
}

// 5：惯量辨识事件
#define TEST_SIGNAL_PERIOD (1)
#define TEST_SIGNAL_FREQUENCY (1.0/TEST_SIGNAL_PERIOD)
#define SPEED_COMMAND_BIAS (300)
#define SPEED_COMMAND_RANGE 150
#define FILTER_CONSTANT 5
void COMM_inertiaId(REAL id_fb, REAL iq_fb, REAL cosPark, REAL sinPark, REAL omg_elec_fb){

    #define STATOR_CURRENT_ALPHA MT2A(id_fb, iq_fb, cosPark, sinPark)
    #define STATOR_CURRENT_BETA  MT2B(id_fb, iq_fb, cosPark, sinPark)
    #define ROTOR_FLUX_ALPHA     MT2A(COMM.KE*1 + 0*PMSM_PERMANENT_MAGNET_FLUX_LINKAGE, 0.0, cosPark, sinPark) // 只要是接近空载，即便KE错了很多，也不影响惯量的辨识
    #define ROTOR_FLUX_BETA      MT2B(COMM.KE*1 + 0*PMSM_PERMANENT_MAGNET_FLUX_LINKAGE, 0.0, cosPark, sinPark)
    #define NORMINAL_INERTIA (1e-6) // 随便给个数量级差不多的数，不同数值只对画图有影响
    #define SPEED_SIGNAL (omg_elec_fb)

    #define AWAYA_LAMBDA (31.4)

    // Inertia Observer 2 Awaya1992 
    static REAL t = 0.0;
    t += CL_TS;
    COMM.timebase = t;

    REAL Tem = CLARKE_TRANS_TORQUE_GAIN * d_sim.init.npp*( STATOR_CURRENT_ALPHA * -ROTOR_FLUX_BETA + STATOR_CURRENT_BETA * ROTOR_FLUX_ALPHA);

    static REAL q0 = 0.0;
    q0 += CL_TS * AWAYA_LAMBDA*( -q0 + Tem);

    static REAL filtered_speed = 0.0;
    filtered_speed = _lpf(SPEED_SIGNAL, filtered_speed, FILTER_CONSTANT);

    static REAL q1_dot = 0.0;
    static REAL q1 = 0.0;
    q1_dot = AWAYA_LAMBDA*( -q1 + filtered_speed/d_sim.init.npp );
    q1 += CL_TS * q1_dot;

    static REAL filtered_q0 = 0.0;
    filtered_q0  = _lpf(q0, filtered_q0, FILTER_CONSTANT);

    static REAL tau_est_lpf = 0.0;
    tau_est_lpf = -(NORMINAL_INERTIA)*q1_dot + filtered_q0;

    static REAL sum_A = 0.0;
    static REAL sum_B = 0.0;
    static REAL est_Js_variation = 0.0;
    static REAL est_Js = NORMINAL_INERTIA;

    sum_A += CL_TS * (tau_est_lpf*q1_dot);
    sum_B += CL_TS * (q1_dot*q1_dot);

    // 给定周期转矩指令，产生周期转速响应
    // if(t < TEST_SIGNAL_PERIOD*0.5){
    //     PID_iQ->Ref = 0.2*MOTOR_RATED_CURRENT_RMS;
    // }else{
    //     PID_iQ->Ref = -0.2*MOTOR_RATED_CURRENT_RMS;
    // }
    PID_iD->Ref = 0.0;

    REAL error = (SPEED_COMMAND_BIAS+SPEED_COMMAND_RANGE*sin(2*M_PI*TEST_SIGNAL_FREQUENCY*(*CTRL).timebase))*RPM_2_ELEC_RAD_PER_SEC - SPEED_SIGNAL;
    // (*CTRL).speed_ctrl_err = error;

    if(FALSE){ // Open loop
        PID_iQ->Ref = 0.2*cos(2*3.1415926*TEST_SIGNAL_FREQUENCY*t);

    }else if(TRUE){// PI Speed Regulator

        static int comm_vc_count = 0;
        if(comm_vc_count++ == SPEED_LOOP_CEILING){
            comm_vc_count = 0;

            (*CTRL).i->cmd_varOmega = (SPEED_COMMAND_BIAS+SPEED_COMMAND_RANGE*sin(2*M_PI*TEST_SIGNAL_FREQUENCY*(*CTRL).timebase)) * RPM_2_MECH_RAD_PER_SEC;
            PID_Speed->Ref = (*CTRL).i->cmd_varOmega * (*CTRL).motor->npp;
            PID_Speed->Fbk = SPEED_SIGNAL;
            PID_Speed->calc(PID_Speed);
            PID_iQ->Ref = PID_Speed->Out;
            // REAL torque_cmd = PID_Speed->Out;
            // PID_iQ->Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*COMM.KE);
            // if(PID_iQ->Ref > MOTOR_RATED_CURRENT_RMS){
            //     PID_iQ->Ref = MOTOR_RATED_CURRENT_RMS;
            // }else if(PID_iQ->Ref < -MOTOR_RATED_CURRENT_RMS){
            //     PID_iQ->Ref = -MOTOR_RATED_CURRENT_RMS;
            // }

            // printf("PID_iQ->Ref=%g\n", PID_iQ->Ref);
        }

    }else{ // PI SMC

        REAL Tload_est = slidingModeObserver(omg_elec_fb, d_sim.init.npp/d_sim.init.KE, iq_fb);

        static int comm_vc_count = 0;
        if(comm_vc_count++ == SPEED_LOOP_CEILING){
            // velocity control loop execution frequency is 40 times slower than current control loop execution frequency
            comm_vc_count = 0;

            REAL torque_cmd = slidingModeControl(error, Tload_est);

            if(torque_cmd > SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = SPEED_LOOP_LIMIT_NEWTON_METER;
            }else if(torque_cmd < -SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = -SPEED_LOOP_LIMIT_NEWTON_METER;
            }
            PID_iQ->Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*d_sim.init.npp*COMM.KE);
        }
    }

    // 计算惯量
    if(t >= TEST_SIGNAL_PERIOD){
        if(fabsf(sum_B)<1e-6){
            sum_B = 1e-6; // avoid zero denominator (do not use use sign(sum_B))
        }
        est_Js_variation = + sum_A / sum_B;
        est_Js = NORMINAL_INERTIA + est_Js_variation;
        #if PC_SIMULATION
            printf("est_Js=%g, sum_A=%g, sum_B=%g\n", est_Js, sum_A, sum_B);
        #endif
        t = 0.0;
        sum_A = 0.0;
        sum_B = 0.0;
        COMM.number_of_repeats_Js += 1;
        if(COMM.number_of_repeats_Js>10){
            COMM.bool_comm_status = 6;
            COMM.bool_comm_status = 4; // PM flux id

            UD_OUTPUT = 0.0;
            UQ_OUTPUT = 0.0;

            COMM.Js = est_Js;
            #if PC_SIMULATION
                printf("Js=%g\n", COMM.Js);
            #endif
            return;
        }
    }

    PID_iD->Fbk = id_fb;
    PID_iD->calc(PID_iD);

    PID_iQ->Fbk = iq_fb;
    PID_iQ->calc(PID_iQ);

    UD_OUTPUT = MT2A(PID_iD->Out, PID_iQ->Out, cosPark, sinPark);
    UQ_OUTPUT = MT2B(PID_iD->Out, PID_iQ->Out, cosPark, sinPark);


    // 限幅，只影响画图
    if(est_Js>1e-2){
        est_Js = 1e-2;
    }
    COMM.Js = est_Js;
}

void COMM_end(REAL id_fb, REAL iq_fb){
    UD_OUTPUT = 0.0;
    UQ_OUTPUT = 0.0;
}


void StepByStepCommissioning_NEW_WB(){
    /* Start *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

    // 使用测量得到的电流、转速、位置等信息，生成电压指令(*CTRL).ual，(*CTRL).ube
    // commissioning();
    if(COMM.bool_comm_status == 0){
        // 初始化
        init_COMM();
    }else if(COMM.bool_comm_status == 1){
        // 电阻辨识
        if(G.flag_do_inverter_characteristics){
            COMM_resistanceId_v2( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] ); // v2 is intended only for better inverter nonlinearity identification considering the rotor movement issue (starting from negative maximal current value to force align)
        }else{
            COMM_resistanceId( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] );
        }
    }else if(COMM.bool_comm_status == 2){
        // 电感辨识（阶跃）
        COMM_inductanceId( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] );
        // 电感辨识（方波）
        // COMM_inductanceId_ver2( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] );

    }else if(COMM.bool_comm_status == 3){
        // 电感辨识（正弦）
        COMM_inductanceId_ver3( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] );
        // 更新电流PI
    }else if(COMM.bool_comm_status == 4){
        // 永磁体磁链辨识
        (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
        (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
        COMM_PMFluxId(  AB2M( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] , (*CTRL).s->cosT, (*CTRL).s->sinT), 
                        AB2T( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] , (*CTRL).s->cosT, (*CTRL).s->sinT), 
                         (*CTRL).i->varOmega);
        // 更新转速PI
    }else if(COMM.bool_comm_status == 5){
        // 惯量辨识
        // (*CTRL).i->omg_elec     = EXP.omg_elec;
        // (*CTRL).i->theta_d_elec = EXP.theta_d;
        // (*CTRL).i->omg_elec     = local_omg_elec;
        // (*CTRL).i->theta_d_elec = local_theta_d;
        (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
        (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
        (*CTRL).i->iDQ[0] = AB2M( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] , (*CTRL).s->cosT, (*CTRL).s->sinT), 
        (*CTRL).i->iDQ[1] = AB2T( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] , (*CTRL).s->cosT, (*CTRL).s->sinT), 
        COMM_inertiaId( (*CTRL).i->iDQ[0], 
                        (*CTRL).i->iDQ[1], 
                        (*CTRL).s->cosT, 
                        (*CTRL).s->sinT, 
                        (*CTRL).i->varOmega * (*CTRL).motor->npp);
    }else{
        COMM_end( (*CTRL).i->iAB[0], (*CTRL).i->iAB[1] );
        COMM.bool_comm_status = -1;
    }

    /* End *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/      
}

#if USING_OLD_COMM
void StepByStepCommissioning_OLD(){
    /* Start *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

    // 使用测量得到的电流、转速、位置等信息，生成电压指令(*CTRL).ual，(*CTRL).ube
    // commissioning();
    if(COMM.bool_comm_status == 0){
        // 初始化
        init_COMM();

    }else if(COMM.bool_comm_status == 1){
        // 电阻辨识
        if(G.flag_do_inverter_characteristics){
            COMM_resistanceId_v2(IS_C(0), IS_C(1)); // v2 is intended only for better inverter nonlinearity identification considering the rotor movement issue (starting from negative maximal current value to force align)
        }else{
            COMM_resistanceId(IS_C(0), IS_C(1));
        }
    }else if(COMM.bool_comm_status == 2){
        // 电感辨识（阶跃）
        COMM_inductanceId(IS_C(0), IS_C(1));
        // 电感辨识（方波）
        // COMM_inductanceId_ver2(IS_C(0), IS_C(1));

    }else if(COMM.bool_comm_status == 3){
        // 电感辨识（正弦）
        COMM_inductanceId_ver3(IS_C(0), IS_C(1));

        // 更新电流PI

    }else if(COMM.bool_comm_status == 4){

        // 永磁体磁链辨识
        (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
        (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
        COMM_PMFluxId(  AB2M(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT), 
                        AB2T(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT), 
                         (*CTRL).i->varOmega);

        // 更新转速PI

    }else if(COMM.bool_comm_status == 5){

        // 惯量辨识
        // (*CTRL).i->omg_elec     = EXP.omg_elec;
        // (*CTRL).i->theta_d_elec = EXP.theta_d;
        // (*CTRL).i->omg_elec     = local_omg_elec;
        // (*CTRL).i->theta_d_elec = local_theta_d;
        (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
        (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
        (*CTRL).i->iDQ[0] = AB2M(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT), 
        (*CTRL).i->iDQ[1] = AB2T(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT), 
        COMM_inertiaId( (*CTRL).i->iDQ[0], 
                        (*CTRL).i->iDQ[1], 
                        (*CTRL).s->cosT, 
                        (*CTRL).s->sinT, 
                        (*CTRL).i->varOmega * (*CTRL).motor->npp);

    }else{
        COMM_end(IS_C(0), IS_C(1));
        COMM.bool_comm_status = -1;
    }

    /* End *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/      
}
#endif

#endif
