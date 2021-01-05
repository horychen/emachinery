#include "ACMSim.h"
#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

// 声明参数自整定结构体变量
struct CommissioningDataStruct COMM;
int16 bool_comm_status = 0;
int16 bool_tuning = TRUE;
int16 bool_CL_tuned = FALSE;
int16 bool_VL_tuned = FALSE;
int16 bool_PL_tuned = FALSE;


#define UD_OUTPUT CTRL.ual_cmd
#define UQ_OUTPUT CTRL.ube_cmd


// 自整定初始化
void COMM_init(){
    COMM.timebase = 0.0;

    // KE
    printf("From name plate data: %g Nm, %g Nm/A, %g Vs, %g mV/rpm\n", MOTOR_RATED_TORQUE, MOTOR_TORQUE_CONSTANT, MOTOR_BACK_EMF_CONSTANT, MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM);
    COMM.KE = MOTOR_BACK_EMF_CONSTANT;

    // R
    COMM.current_sum = 0.0;
    COMM.voltage_sum = 0.0;
    COMM.counterSS = 0;
    COMM.bool_collecting = FALSE;
    int i;
    for(i=0;i<COMM_IV_SIZE;++i){
        COMM.i_data[i] = 0.0;
        COMM.v_data[i] = 0.0;
    }
    COMM.inverter_voltage_drop = 0.0;

    // L
    COMM.L = 0.999;
    COMM.id_prev = 0.0;
    COMM.iq_prev = 0.0;

    // global status vairbale
    bool_comm_status = 1;

    COMM.counterEntered = 0;
    COMM.i = 0;
}

// SS: Steady State
#define SS_RATED_CURRENT_RATIO 1e-2 // 电流误差在额定电流的 0.01 倍以下时，认为 有可能 达到电流调节稳态了
#define SS_COUNTER_CEILING ((long int)(0.2/CL_TS)) // 达到稳态后，计数，超过这个值才认为真正进入稳态
int reachSteadyStateCurrent(float32 current_error, float32 rated_current){
    static long int counterSS = 0;

    if(fabs(current_error) < rated_current * SS_RATED_CURRENT_RATIO){
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
    if (fabs(term23) > 1e-6)
        *r = (term1 * term1) / term23; // Excel returns r*r.
    
    return 0; 
}



void COMM_PI_tuning(REAL LL, REAL RR, REAL BW_current, REAL delta, REAL JJ, REAL KE, REAL npp){

    // Current loop tuning
    pid1_id.Kp = LL * BW_current;
    pid1_iq.Kp = LL * BW_current;
    pid1_ia.Kp = LL * BW_current;
    pid1_ib.Kp = LL * BW_current;
    pid1_ic.Kp = LL * BW_current;

    pid1_id.Ki = pid1_id.Kp * RR/LL * CL_TS; 
    pid1_iq.Ki = pid1_iq.Kp * RR/LL * CL_TS; // R=Rs+Rr for q axis
    pid1_ia.Ki = pid1_ia.Kp * RR/LL * CL_TS;
    pid1_ib.Ki = pid1_ib.Kp * RR/LL * CL_TS;
    pid1_ic.Ki = pid1_ic.Kp * RR/LL * CL_TS;

    REAL K = npp/JJ * CLARKE_TRANS_TORQUE_GAIN*npp*KE;
    REAL BW_speed = BW_current / ( delta + 2.16*exp(-delta/2.8) - 1.86 ); // [rad/s]

    REAL lowest_pole_over_0dB = BW_current;
    // REAL lowest_pole_over_0dB = 2*3.1415926 * 150; // 5 Hz The feedback frequency of hall sensor at 10 rpm is near 5 Hz.

    // My tuning
    // pid1_spd.Ki = lowest_pole_over_0dB / (delta*delta) * VL_TS; 

    pid1_spd.Kp = lowest_pole_over_0dB / (delta) / K;

    // TI's tuning
    pid1_spd.Ki = pid1_spd.Kp * lowest_pole_over_0dB / (delta*delta) * VL_TS; 

    printf("Current loop, BW_current=%.1f Hz, Kp=%g, KpKi=%g\n", BW_current/(2*3.1415926), pid1_id.Kp,  pid1_iq.Ki*CL_TS);
    printf("Speed loop,   BW_speed=%.1f Hz,   Kp=%g, KpKi=%g, K=%g\n",      BW_speed/(2*3.1415926),   pid1_spd.Kp, pid1_spd.Ki*VL_TS, K);
}

// 1：电阻辨识事件
void COMM_resistanceId(REAL id_fb, REAL iq_fb){
    ++COMM.counterEntered;
    if(COMM.counterEntered<=1){
        // Current loop tuning (LL, RR, BW_c, delta, JJ, KE, npp)
        COMM_PI_tuning(1e-3, 0.5, 2*3.1415926*CL_TS_INVERSE * 0.025, 20, MOTOR_SHAFT_INERTIA, MOTOR_BACK_EMF_CONSTANT, MOTOR_NUMBER_OF_POLE_PAIRS);        
    }

    #define RS_ID_NUMBER_OF_STAIRS 10
    #define RS_ID_MAXIMUM_CURRENT (0.3*(float32)MOTOR_RATED_CURRENT_RMS)
    float32 current_increase_per_step = RS_ID_MAXIMUM_CURRENT / RS_ID_NUMBER_OF_STAIRS;

    COMM.current_command = - RS_ID_MAXIMUM_CURRENT;
    COMM.current_command += current_increase_per_step*COMM.i;

    pid1_id.Fbk = id_fb;
    pid1_id.Ref = COMM.current_command;
    pid1_id.calc(&pid1_id);
    UD_OUTPUT = pid1_id.Out;
    UQ_OUTPUT = 0.0;

    // collect steady state data
    if(COMM.bool_collecting){
        COMM.current_sum += id_fb;
        COMM.voltage_sum += UD_OUTPUT;
        COMM.counterSS += 1;

        if(COMM.counterSS>800){
            COMM.i_data[COMM.i] = COMM.current_sum/(float32)COMM.counterSS;
            COMM.v_data[COMM.i] = COMM.voltage_sum/(float32)COMM.counterSS;
            printf("%f, %f\n", COMM.i_data[COMM.i], COMM.v_data[COMM.i]);
            COMM.bool_collecting = FALSE;
            ++COMM.i;
        }
    }else{
        // reset
        COMM.current_sum = 0.0;
        COMM.voltage_sum = 0.0;
        COMM.counterSS = 0;

        // check steady state and assign boolean variable
        if(reachSteadyStateCurrent(pid1_id.Ref-pid1_id.Fbk, MOTOR_RATED_CURRENT_RMS)){

            COMM.bool_collecting = TRUE;

            if(COMM.current_command > RS_ID_MAXIMUM_CURRENT){

                // Get resistance value
                if(FALSE){
                    COMM.R = (COMM.v_data[1] - COMM.v_data[0]) / (COMM.i_data[1] - COMM.i_data[0]);
                }else{
                    REAL m,b,r;
                    if(linreg(( int)(0.4*RS_ID_NUMBER_OF_STAIRS), COMM.i_data, COMM.v_data, &m, &b, &r)){
                        printf("FAILURE\n");
                    }
                    COMM.R = m;
                    COMM.inverter_voltage_drop = b;
                }
                printf("R=%g Ohm, inverter_voltage_drop=%g\n", COMM.R, COMM.inverter_voltage_drop);

                COMM.last_voltage_command = UD_OUTPUT;
                bool_comm_status = 3;
                COMM.counterEntered = 0;

                // UD_OUTPUT = 0.0;
                // UQ_OUTPUT = 0.0;
            }
        }
    }
}

// 2/3：电感辨识事件
void COMM_inductanceId(REAL id_fb, REAL iq_fb){

    // Collect current data for curve-fitting
    if(COMM.counterEntered >= 0 && COMM.counterEntered < COMM_IV_SIZE){
        COMM.i_data[COMM.counterEntered] = id_fb; // COMM.counterEntered; //
        COMM.v_data[COMM.counterEntered] = COMM.counterEntered*CL_TS;
        // printf("|||||%d, %g\n", COMM.counterEntered, COMM.i_data[COMM.counterEntered]);
    }

    ++COMM.counterEntered;

    float32 Delta_IS_al;
    float32 Delta_US_al;
    float32 IS_slope;

    Delta_US_al = +3*COMM.last_voltage_command;
    Delta_IS_al = id_fb - COMM.id_prev;
    IS_slope = Delta_IS_al / CL_TS;

    UD_OUTPUT = COMM.last_voltage_command + Delta_US_al;
    UQ_OUTPUT = 0.0;

    // printf("Last = %g, UD_OUTPUT = %g \n", COMM.last_voltage_command, UD_OUTPUT);

    #define NUMBER_OF_SAMPLES (20)
    #define NUMBER_OF_SAMPLES_USED_IN_FITTING (20)


    float32 L_temp;
    if(COMM.counterEntered < NUMBER_OF_SAMPLES){
        L_temp = Delta_US_al / IS_slope;
        printf("L=%g, %g\n", L_temp, IS_slope );
        if(L_temp < COMM.L && L_temp > 0.0){
            COMM.L = L_temp;
        }
    }else{
        UD_OUTPUT = 0.0;
        UQ_OUTPUT = 0.0;

        REAL m,b,r;
        if(linreg(NUMBER_OF_SAMPLES_USED_IN_FITTING, COMM.v_data, COMM.i_data, &m, &b, &r)){
            printf("FAILURE\n");
        }
        COMM.L = Delta_US_al / m;
        // printf("[Ideal] m=-423.301, b=1.92252, r=0.999765\n");
        printf("[Real]  m=%g, b=%g, r=%g\n", m,b,r);
        printf("COMM.L = %g\n", COMM.L);
        bool_comm_status = 3;
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
    static int16 number_of_repeats = 0;

    COMM.counterEntered += 1;
    if(COMM.counterEntered == COMM_FAST_SWITCH_MOD+1 || COMM.counterEntered == 1){
        id_avg = id_fb + COMM.id_prev;

        pid1_id.Fbk = id_avg;
        pid1_id.Ref = COMM.current_command;
        pid1_id.calc(&pid1_id);

        Delta_current = fabs(id_fb - COMM.id_prev);
        COMM.L = COMM_FAST_SWITCH_MOD*CL_TS * COMM_FAST_SWITCH_VOLTAGE_CHANGE / (1*Delta_current);

        COMM.id_prev = id_fb;
    }

    UD_OUTPUT = COMM.last_voltage_command; // pid1_id.Out;
    UQ_OUTPUT = 0.0;

    if(COMM.counterEntered <= COMM_FAST_SWITCH_MOD){
        UD_OUTPUT = pid1_id.Out + COMM_FAST_SWITCH_VOLTAGE_CHANGE;
    }else{
        UD_OUTPUT = pid1_id.Out - COMM_FAST_SWITCH_VOLTAGE_CHANGE;
        if(COMM.counterEntered == COMM_FAST_SWITCH_MOD*2){
            COMM.counterEntered = 0;
            printf("L=%g\n", COMM.L);
            number_of_repeats += 1;
            if(number_of_repeats > 500){
                bool_comm_status = 3;
                number_of_repeats = 0;
            }
        }
    }
}
#define N_SAMPLE ((long int)(12.5/CL_TS)) // Resolution 0.1 Hz = 1 / (N_SAMPLE * TS)
void COMM_inductanceId_ver3(REAL id_fb, REAL iq_fb){

    // 要求分辨率达到0.1=1/N_SAMPLE/CL_TS // 目标频率应该选择使得能够被数据的长度所分辨的值，也就是0.1的整数倍即可，比如4.6Hz。
    // Rreq结果：0.1Hz（要求N_SAMPLE为100000，分辨率为0.1Hz）偏大，4.5Hz（要求N_SAMPLE为100000，分辨率为0.1Hz）刚好，10Hz偏小
    REAL omegaL = 2*3.1415926*2;

    static REAL mz_timebase = -CL_TS;
    mz_timebase += CL_TS;   

    REAL Ireal, Iimag;
    REAL cdCos, cdSin; // Coherent Demodulation

    static REAL temp_accumCosL=0.0;
    static REAL temp_accumSinL=0.0;

    #define HORY_MZ2014_AMPL_LOW (0.1*COMM.last_voltage_command) // 10 [V]
    #define HORY_MZ2014_SAMPLE_DUTY 0.8 // 不考虑三大现实问题时，取1和0.8和0.6没有本质差别；

    UD_OUTPUT = HORY_MZ2014_AMPL_LOW*cos(omegaL*mz_timebase);

    if(mz_timebase>=(1.0-HORY_MZ2014_SAMPLE_DUTY)*N_SAMPLE*CL_TS){ // 前20%认为含有暂态过程（磁链的周期还没跟上电压的）
        // 求平均值进行解调
        temp_accumCosL += id_fb * cos(omegaL*mz_timebase);
        temp_accumSinL += id_fb * sin(omegaL*mz_timebase);
    }

    // BUG: 这里转换整型数，一开始用的(int)，仿真可以，但是DSP中就出现进不去这个if语句的现象。
    if((long int)(mz_timebase/CL_TS+1.5) % N_SAMPLE==0){ // +1用于回避t=0时那一次，+0.5用于四舍五入

        cdCos = temp_accumCosL / (HORY_MZ2014_SAMPLE_DUTY*N_SAMPLE);
        cdSin = temp_accumSinL / (HORY_MZ2014_SAMPLE_DUTY*N_SAMPLE);
        temp_accumCosL = 0.0;
        temp_accumSinL = 0.0;

        mz_timebase = -CL_TS; // 时基重置，方便进行SAMPLE_DUTY比较。

        Ireal = 2.0*cdCos;
        Iimag = -2.0*cdSin;

        // Compute R
        COMM.R3 = HORY_MZ2014_AMPL_LOW * Ireal / (Ireal*Ireal + Iimag*Iimag);

        // Compute L
        COMM.L = - HORY_MZ2014_AMPL_LOW * Iimag / (Ireal*Ireal + Iimag*Iimag) / (omegaL);
        COMM.L3 = COMM.L;

        printf("R=%g, L=%g, mztime=%g, triggertime=%g\n", COMM.R3, COMM.L, mz_timebase, (1.0-HORY_MZ2014_SAMPLE_DUTY)*N_SAMPLE*CL_TS);

        bool_comm_status = 4; // PM flux id
        bool_comm_status = 5; // inertia id
        COMM.counterEntered = 0;



        // REAL L_guess = 2*48 * 0.05 / (2*M_PI * MOTOR_RATED_SPEED_RPM/60.0*MOTOR_NUMBER_OF_POLE_PAIRS * MOTOR_RATED_CURRENT_RMS);
        // REAL R_guess = 2*48 * 0.01 / MOTOR_RATED_CURRENT_RMS;
        // printf("L_guess=%g, R_guess=%g\n", L_guess, R_guess);
        // getch();

        // Current loop tuning (LL, RR, BW_c, delta, JJ, KE, npp)
        // COMM_PI_tuning(COMM.L3, COMM.R, 2*3.1415926*CL_TS_INVERSE * 0.025, 20, MOTOR_SHAFT_INERTIA, MOTOR_BACK_EMF_CONSTANT, MOTOR_NUMBER_OF_POLE_PAIRS);
        COMM_PI_tuning(COMM.L3, COMM.R, 2*3.1415926*CL_TS_INVERSE * 0.025, 400, MOTOR_SHAFT_INERTIA, MOTOR_BACK_EMF_CONSTANT, MOTOR_NUMBER_OF_POLE_PAIRS);
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
    REAL S = error + SM_C*x1;

    x1 += CL_TS*SPEED_LOOP_CEILING * (error);

    REAL P_output = SM_C * error + ( 0.0 + SM_K / (SM_VAREPSILON + ( 1.0 + 1.0/fabs(x1) - SM_VAREPSILON)*exp(-SM_DELTA*fabs(S))) ) * sign(S);

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
REAL slidingModeObserver(REAL omg_elec_fb, REAL mu_m, REAL iq_fb){
    REAL est_error = omg_elec_fb - omg_elec_est;
    omg_elec_est += CL_TS * mu_m * ( MOTOR_NUMBER_OF_POLE_PAIRS*MOTOR_BACK_EMF_CONSTANT*iq_fb - Tload_est + smo_eta*sign(est_error) );
    Tload_est += - CL_TS * smo_gamma * smo_eta * sign(est_error);
    return Tload_est;
}
// #define RPM_2_RAD_PER_SEC ( (2*3.1415926*MOTOR_NUMBER_OF_POLE_PAIRS)/60.0 )
float32 rpm_speed_command = 300;
// REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv){
//     return y_tminus1 + CL_TS * time_const_inv * (x - y_tminus1);
// }
REAL filtered_voltage = 0.0;
REAL filtered_current = 0.0;
REAL filtered_omg_elec_fb = 0.0;
void COMM_PMFluxId(REAL id_fb, REAL iq_fb, REAL omg_elec_fb){

    COMM.counterEntered += 1;
    // if(COMM.counterEntered==1){
    //     // delta =10
    //     pid1_spd.Kp = 1e-6 * (CL_TS_INVERSE*0.1) / 10.0;
    //     pid1_spd.Ki = pid1_spd.Kp * (CL_TS_INVERSE*0.1) / 100.0 * (CL_TS*SPEED_LOOP_CEILING);
    // }

    pid1_id.Ref = 0.0;

    if(CTRL.timebase>20){
        rpm_speed_command = 5;
    }

    REAL error = rpm_speed_command*RPM_2_RAD_PER_SEC - omg_elec_fb;
    CTRL.speed_ctrl_err = error;
    // printf("%g: %g\n", CTRL.timebase, error);

    if(TRUE){ // PI Speed Regulator

        static int vc_count = 0;
        if(vc_count++ == SPEED_LOOP_CEILING){
            vc_count = 0;

            bool_tuning = FALSE;

            // if(bool_tuning){
            //     if(error-0.5>0){
            //         pid1_spd.Kp += CL_TS*SPEED_LOOP_CEILING * 0.0005;
            //     }else if(error-0.5<0){
            //         bool_tuning = FALSE;
            //     }
            // }

            pid1_spd.Ref = rpm_speed_command*RPM_2_RAD_PER_SEC;
            pid1_spd.Fbk = omg_elec_fb;
            pid1_spd.calc(&pid1_spd);
            pid1_iq.Ref = pid1_spd.Out;
            // REAL torque_cmd = pid1_spd.Out;
            // pid1_iq.Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*MOTOR_BACK_EMF_CONSTANT);
            // if(pid1_iq.Ref > MOTOR_RATED_CURRENT_RMS){
            //     pid1_iq.Ref = MOTOR_RATED_CURRENT_RMS;
            // }else if(pid1_iq.Ref < -MOTOR_RATED_CURRENT_RMS){
            //     pid1_iq.Ref = -MOTOR_RATED_CURRENT_RMS;
            // }
            // printf("KT=%g\n", CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*MOTOR_BACK_EMF_CONSTANT);
            // getch();
        }
    }else{ // PI SMC

        REAL Tload_est = slidingModeObserver(omg_elec_fb, MOTOR_NUMBER_OF_POLE_PAIRS/MOTOR_BACK_EMF_CONSTANT, iq_fb);

        static int vc_count = 0;
        if(vc_count++ == SPEED_LOOP_CEILING){
            // velocity control loop execution frequency is 40 times slower than current control loop execution frequency
            vc_count = 0;

            REAL torque_cmd = slidingModeControl(error, Tload_est);

            if(torque_cmd > SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = SPEED_LOOP_LIMIT_NEWTON_METER;
            }else if(torque_cmd < -SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = -SPEED_LOOP_LIMIT_NEWTON_METER;
            }
            pid1_iq.Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*MOTOR_BACK_EMF_CONSTANT);
        }
    }

    // put this before _lpf(...)
    pid1_id.Fbk = id_fb;
    pid1_iq.Fbk = iq_fb;

    // put this before pid1_iq.calc(&pid1_iq);
    filtered_voltage     = _lpf(pid1_iq.Out, filtered_voltage, 5);
    filtered_current     = _lpf(pid1_iq.Fbk, filtered_current, 5);
    filtered_omg_elec_fb = _lpf(omg_elec_fb, filtered_omg_elec_fb, 5);

    // voltage output
    pid1_id.calc(&pid1_id);
    pid1_iq.calc(&pid1_iq);
    UD_OUTPUT = MT2A(pid1_id.Out, pid1_iq.Out, CTRL.cosT, CTRL.sinT);
    UQ_OUTPUT = MT2B(pid1_id.Out, pid1_iq.Out, CTRL.cosT, CTRL.sinT);

    static REAL last_KE;
    last_KE = COMM.KE;
    if(fabs(omg_elec_fb)>1e-3){
        // 有除法的地方就要小心分母为零（1.#INF）：
            // x0(id)[A],x1(iq)[A],x2(speed)[rpm],x3(position)[rad],ud[V],uq[V],IS_C(0),CTRL.ual,ACM.ual,ACM.theta_d,DIST_AL,COMM.KE,COMM.Js
            // 0.712084,0.0339154,0.21666,1.51514e-006,-19.4698,0.47952,0.712084,-19.4698,-19.4698,1.51514e-006,0,1.#INF,0
        COMM.KE = (filtered_voltage - COMM.R*filtered_current) / filtered_omg_elec_fb - COMM.L3 * pid1_id.Ref;
        // COMM.KE *= sqrt(CLARKE_TRANS_TORQUE_GAIN); // 想要在恒幅值变换下拿到和恒功率变换一样的永磁体磁链值，就要乘以这个系数。
    }
    if(COMM.KE>3*MOTOR_BACK_EMF_CONSTANT){
        COMM.KE = 3*MOTOR_BACK_EMF_CONSTANT;
    }else if(COMM.KE<0){
        COMM.KE = 0.0;
    }

    static REAL accumKE = 0.0;
    static long int countKE = 0;

    if(bool_tuning==FALSE){
        // KE便是指是否达到稳态？
        if(fabs(COMM.KE - last_KE)>=0.5e-2){
            COMM.counterEntered = 0; // 复用一下这个计数器用于判断KE是否收敛，可以少声明一个计数器。
        }else{ 
            // printf("%d, KE=%g, err=%g, accumKE=%g, countKE=%d\n", COMM.counterEntered, COMM.KE, error, accumKE, countKE);
        }

        // 转速控制是否达到稳态？
        if(fabs(error)<5e-1 && omg_elec_fb*RAD_PER_SEC_2_RPM>10){
            if(COMM.counterEntered < 600000){

            }else if(COMM.counterEntered < 900000){
                accumKE += COMM.KE;
                countKE += 1;

            }else{
                COMM.KE = accumKE / (REAL)countKE;

                printf("KE=%g | ACM.KE=%g | PMSM_PERMANENT_MAGNET_FLUX_LINKAGE=%g | MOTOR_BACK_EMF_CONSTANT=%g\n", COMM.KE, ACM.KE, PMSM_PERMANENT_MAGNET_FLUX_LINKAGE, MOTOR_BACK_EMF_CONSTANT);
                // getch();

                bool_comm_status = 5;
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
int number_of_repeats = 0;
#define FILTER_CONSTANT 5
void COMM_intertiaId(REAL id_fb, REAL iq_fb, REAL cosPark, REAL sinPark, REAL omg_elec_fb){

    #define STATOR_CURRENT_ALPHA MT2A(id_fb, iq_fb, cosPark, sinPark)
    #define STATOR_CURRENT_BETA  MT2B(id_fb, iq_fb, cosPark, sinPark)
    #define ROTOR_FLUX_ALPHA     MT2A(COMM.KE*1 + 0*PMSM_PERMANENT_MAGNET_FLUX_LINKAGE, 0.0, cosPark, sinPark) // 只要是接近空载，即便KE错了很多，也不影响惯量的辨识
    #define ROTOR_FLUX_BETA      MT2B(COMM.KE*1 + 0*PMSM_PERMANENT_MAGNET_FLUX_LINKAGE, 0.0, cosPark, sinPark)
    #define NORMINAL_INERTIA (1e-6) // 随便给个数量级差不多的数，不同数值只对画图有影响
    #define SPEED_SIGNAL (omg_elec_fb)

    #define AWAYA_LAMBDA (31.4)

    // Inertia Observer 2 Awaya1992 
    static float32 t = 0.0;
    t += CL_TS;
    COMM.timebase = t;

    float32 Tem = CLARKE_TRANS_TORQUE_GAIN * MOTOR_NUMBER_OF_POLE_PAIRS*( STATOR_CURRENT_ALPHA * -ROTOR_FLUX_BETA + STATOR_CURRENT_BETA * ROTOR_FLUX_ALPHA);

    static float32 q0 = 0.0;
    q0 += CL_TS * AWAYA_LAMBDA*( -q0 + Tem);

    static REAL filtered_speed = 0.0;
    filtered_speed = _lpf(SPEED_SIGNAL, filtered_speed, FILTER_CONSTANT);

    static float32 q1_dot = 0.0;
    static float32 q1 = 0.0;
    q1_dot = AWAYA_LAMBDA*( -q1 + filtered_speed/MOTOR_NUMBER_OF_POLE_PAIRS );
    q1 += CL_TS * q1_dot;

    static double filtered_q0 = 0.0;
    filtered_q0  = _lpf(q0, filtered_q0, FILTER_CONSTANT);

    static float32 tau_est_lpf = 0.0;
    tau_est_lpf = -(NORMINAL_INERTIA)*q1_dot + filtered_q0;

    static float32 sum_A = 0.0;
    static float32 sum_B = 0.0;
    static float32 est_Js_variation = 0.0;
    static float32 est_Js = NORMINAL_INERTIA;

    sum_A += CL_TS * (tau_est_lpf*q1_dot);
    sum_B += CL_TS * (q1_dot*q1_dot);

    // 给定周期转矩指令，产生周期转速响应
    // if(t < TEST_SIGNAL_PERIOD*0.5){
    //     pid1_iq.Ref = 0.2*MOTOR_RATED_CURRENT_RMS;
    // }else{
    //     pid1_iq.Ref = -0.2*MOTOR_RATED_CURRENT_RMS;
    // }
    pid1_id.Ref = 0.0;

    REAL error = (SPEED_COMMAND_BIAS+SPEED_COMMAND_RANGE*cos(M_PI+2*M_PI*TEST_SIGNAL_FREQUENCY*CTRL.timebase))*RPM_2_RAD_PER_SEC - SPEED_SIGNAL;
    CTRL.speed_ctrl_err = error;

    if(FALSE){ // Open loop
        pid1_iq.Ref = 0.2*cos(2*3.1415926*TEST_SIGNAL_FREQUENCY*t);

    }else if(TRUE){// PI Speed Regulator

        static int vc_count = 0;
        if(vc_count++ == SPEED_LOOP_CEILING){
            vc_count = 0;

            pid1_spd.Ref = (SPEED_COMMAND_BIAS+SPEED_COMMAND_RANGE*cos(M_PI+2*M_PI*TEST_SIGNAL_FREQUENCY*CTRL.timebase))*RPM_2_RAD_PER_SEC;
            pid1_spd.Fbk = SPEED_SIGNAL;
            pid1_spd.calc(&pid1_spd);
            pid1_iq.Ref = pid1_spd.Out;
            // REAL torque_cmd = pid1_spd.Out;
            // pid1_iq.Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*COMM.KE);
            // if(pid1_iq.Ref > MOTOR_RATED_CURRENT_RMS){
            //     pid1_iq.Ref = MOTOR_RATED_CURRENT_RMS;
            // }else if(pid1_iq.Ref < -MOTOR_RATED_CURRENT_RMS){
            //     pid1_iq.Ref = -MOTOR_RATED_CURRENT_RMS;
            // }

            // printf("pid1_iq.Ref=%g\n", pid1_iq.Ref);
        }

    }else{ // PI SMC

        REAL Tload_est = slidingModeObserver(omg_elec_fb, MOTOR_NUMBER_OF_POLE_PAIRS/MOTOR_BACK_EMF_CONSTANT, iq_fb);

        static int vc_count = 0;
        if(vc_count++ == SPEED_LOOP_CEILING){
            // velocity control loop execution frequency is 40 times slower than current control loop execution frequency
            vc_count = 0;

            REAL torque_cmd = slidingModeControl(error, Tload_est);

            if(torque_cmd > SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = SPEED_LOOP_LIMIT_NEWTON_METER;
            }else if(torque_cmd < -SPEED_LOOP_LIMIT_NEWTON_METER){
                torque_cmd = -SPEED_LOOP_LIMIT_NEWTON_METER;
            }
            pid1_iq.Ref = torque_cmd / (CLARKE_TRANS_TORQUE_GAIN*MOTOR_NUMBER_OF_POLE_PAIRS*COMM.KE);
        }
    }

    // 计算惯量
    if(t >= TEST_SIGNAL_PERIOD){
        if(fabs(sum_B)<1e-6){
            sum_B = 1e-6; // avoid zero denominator (do not use use sign(sum_B))
        }
        est_Js_variation = + sum_A / sum_B;
        est_Js = NORMINAL_INERTIA + est_Js_variation;
        printf("est_Js=%g, sum_A=%g, sum_B=%g\n", est_Js, sum_A, sum_B);
        t = 0.0;
        sum_A = 0.0;
        sum_B = 0.0;
        number_of_repeats += 1;
        if(number_of_repeats>10){
            bool_comm_status = 6;
            UD_OUTPUT = 0.0;
            UQ_OUTPUT = 0.0;

            COMM.Js = est_Js;
            printf("Js=%g\n", COMM.Js);
            return;
        }
    }

    pid1_id.Fbk = id_fb;
    pid1_id.calc(&pid1_id);

    pid1_iq.Fbk = iq_fb;
    pid1_iq.calc(&pid1_iq);

    UD_OUTPUT = MT2A(pid1_id.Out, pid1_iq.Out, cosPark, sinPark);
    UQ_OUTPUT = MT2B(pid1_id.Out, pid1_iq.Out, cosPark, sinPark);


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




void StepByStepCommissioning(){
    // bool_comm_status = 4;

    /* Start *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

    // 使用测量得到的电流、转速、位置等信息，生成电压指令CTRL.ual，CTRL.ube
    // commissioning();
    if(bool_comm_status == 0){
        // 初始化
        COMM_init();

    }else if(bool_comm_status == 1){
        // 电阻辨识
        COMM_resistanceId(IS_C(0), IS_C(1));

    }else if(bool_comm_status == 2){
        // 电感辨识（阶跃）
        COMM_inductanceId(IS_C(0), IS_C(1));
        // 电感辨识（方波）
        // COMM_inductanceId_ver2(IS_C(0), IS_C(1));

    }else if(bool_comm_status == 3){
        // 电感辨识（正弦）
        COMM_inductanceId_ver3(IS_C(0), IS_C(1));

        // 更新电流PI

    }else if(bool_comm_status == 4){

        // bool_comm_status = 5;

        // // 计算转速（lidozzi2007）
        // if(sm.bool_position_correction_pending == TRUE){
        //     sm.bool_position_correction_pending = FALSE;
        //     local_theta_d = sm.theta_d_hall;
        // }
        // CTRL.cosT = cos(local_theta_d);
        // CTRL.sinT = sin(local_theta_d);
        // REAL Vq = AB2T(US_C(0), US_C(1), CTRL.cosT, CTRL.sinT);
        // REAL iq = AB2T(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT);
        // static REAL last_iq = 0.0;
        // static REAL last_local_omg_elec = 0.0;
        // local_omg_elec = ( Vq - COMM.R*iq - COMM.L3*(iq - last_iq) / CL_TS ) / (COMM.KE + COMM.L3*0);
        // last_iq = iq;
        // local_theta_d += CL_TS*0.5*(local_omg_elec + last_local_omg_elec);
        // last_local_omg_elec = local_omg_elec;

        // 永磁体磁链辨识
        CTRL.omg__fb     = sm.omg_elec; 
        CTRL.theta_d__fb = sm.theta_d;
        // CTRL.omg__fb     = local_omg_elec;
        // CTRL.theta_d__fb = local_theta_d;
        CTRL.cosT = cos(CTRL.theta_d__fb);
        CTRL.sinT = sin(CTRL.theta_d__fb);
        COMM_PMFluxId(AB2M(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT), AB2T(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT), CTRL.omg__fb);

        // 更新转速PI

    }else if(bool_comm_status == 5){

        // 惯量辨识
        CTRL.omg__fb     = sm.omg_elec;
        CTRL.theta_d__fb = sm.theta_d;
        // CTRL.omg__fb     = local_omg_elec;
        // CTRL.theta_d__fb = local_theta_d;
        CTRL.cosT = cos(CTRL.theta_d__fb);
        CTRL.sinT = sin(CTRL.theta_d__fb);
        COMM_intertiaId(AB2M(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT), AB2T(IS_C(0), IS_C(1), CTRL.cosT, CTRL.sinT), CTRL.cosT, CTRL.sinT, CTRL.omg__fb);

    }else{
        COMM_end(IS_C(0), IS_C(1));
    }

    /* End *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/      
}
#endif