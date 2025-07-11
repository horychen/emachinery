#include "ACMSim.h"
#include "math.h"
#include "stdio.h"

extern REAL sig_a2;
extern REAL sig_a3;

/* 功能函数 */
// 符号函数
// #if PC_SIMULATION == TRUE //* utility的函数定义要小心，由于仿真和emy-c的ACMSim.h的内容不同，可能会导致编译错误
REAL sign(REAL x){
    return (x > 0) - (x < 0);
}
// #endif

int32 sign_integer(int32 x){
    return (x > 0) - (x < 0);
}

// 判断是否为有效浮点数
int isNumber(REAL x){
    // This looks like it should always be TRUE,
    // but it's FALSE if x is an NaN (1.#QNAN0).
    return (x == x);
    // see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}

// 低通滤波器：测量值，上一步的滤波器输出，时间常数的倒数
REAL _lpf(REAL x, REAL y, REAL tau_inv){
    return y + CL_TS * tau_inv * (x - y);
}
// 高通滤波器：测量值，上一步的低通滤波器输出的地址，时间常数的倒数
REAL _hpf(REAL x, REAL *lpf_y, REAL tau_inv){
    *lpf_y = _lpf(x, *lpf_y, tau_inv);
    return x - *lpf_y;
}

#if ENCODER_TYPE == INCREMENTAL_ENCODER_QEP
// #define MA_SEQUENCE_LENGTH         20 // 20 * CL_TS = window of moving average in seconds
// #define MA_SEQUENCE_LENGTH_INVERSE 0.05
// REAL ENC.MA_qepPosCnt[MA_SEQUENCE_LENGTH];
// REAL sum_qepPosCnt = 0.0;
// unsigned int ENC.cursor=0; // ENC.cursor is between 0~79, and it decides which element in MA queue should be kicked out.
REAL PostionSpeedMeasurement_MovingAvergage(int32 QPOSCNT, st_enc *p_enc){
    // EQep1Regs
    #define enc (*p_enc)

        /* Part One: Read QEP Registers to get Position */

        /* 获取绝对位置 [cnt] 用于转子位置解算 */
        enc.encoder_abs_cnt_previous = enc.encoder_abs_cnt;
        enc.encoder_abs_cnt = positive_current_QPOSCNT_counting_down * ( (int32)QPOSCNT - enc.OffsetCountBetweenIndexAndUPhaseAxis);
        if (enc.encoder_abs_cnt > SYSTEM_QEP_QPOSMAX_PLUS_1){
            enc.encoder_abs_cnt -= SYSTEM_QEP_QPOSMAX_PLUS_1;
        }
        if (enc.encoder_abs_cnt < 0){
            enc.encoder_abs_cnt += SYSTEM_QEP_QPOSMAX_PLUS_1;
        }
        enc.theta_d_elec = enc.encoder_abs_cnt * CNT_2_ELEC_RAD;
        //    if (enc.theta_d__state > M_PI)
        //        enc.theta_d__state -= 2 * M_PI;
        //    if (enc.theta_d__state < -M_PI)
        //        enc.theta_d__state += 2 * M_PI;

        /* Part Two: Moving Average with a update period of CL_TS */

        /* 获取位置增量 [cnt] 用于滑动平均转速解算 */
        enc.encoder_incremental_cnt = enc.encoder_abs_cnt - enc.encoder_abs_cnt_previous;
        // 增量超过ppr的一半则认为是发生了Cnt被Z信号清零事件
        if (enc.encoder_incremental_cnt < -0.5 * SYSTEM_QEP_QPOSMAX_PLUS_1)
            enc.encoder_incremental_cnt += (int32)SYSTEM_QEP_QPOSMAX_PLUS_1;
        else if (enc.encoder_incremental_cnt > 0.5 * SYSTEM_QEP_QPOSMAX_PLUS_1)
            enc.encoder_incremental_cnt -= (int32)SYSTEM_QEP_QPOSMAX_PLUS_1;

        enc.sum_qepPosCnt -= enc.MA_qepPosCnt[enc.cursor];
        enc.sum_qepPosCnt += enc.encoder_incremental_cnt;
        enc.MA_qepPosCnt[enc.cursor] = enc.encoder_incremental_cnt;
        enc.cursor += 1; // 完事以后再加一
        if (enc.cursor >= MA_SEQUENCE_LENGTH){
            enc.cursor = 0; // Reset enc.cursor
        }
        enc.rpm = enc.sum_qepPosCnt * SYSTEM_QEP_REV_PER_PULSE * 60 * MA_SEQUENCE_LENGTH_INVERSE * CL_TS_INVERSE;
        enc.varOmega = enc.rpm * RPM_2_MECH_RAD_PER_SEC; // 机械转速（单位：RPM）-> 电气角速度（单位：elec.rad/s)

        return 0.0;
    #undef enc
}

#endif

REAL difference_between_two_angles(REAL first, REAL second)
{
    while (first > 2 * M_PI){
        first -= 2 * M_PI;
    }while (second > 2 * M_PI){
        second -= 2 * M_PI;
    }

    while (first < 0.0){
        first += 2 * M_PI;
    }while (second < 0.0){
        second += 2 * M_PI;
    }

    if (fabsf(first - second) < M_PI){
        return first - second;
    }else{
        if (first > second){
            return first - 2 * M_PI - second;
        }else{
            return first + 2 * M_PI - second;
        }
    }
}



#if PC_SIMULATION == TRUE
    #define DOWN_SAMPLE 1
    // 写变量meta-data到文件
    void write_header_to_file(FILE *fw)
    {
        if (d_sim.user.verbose)
            printf("\tData @ %s\n", DATA_FILE_NAME);
            fprintf(fw, DATA_LABELS);{
            // 将除了变量数据和变量名以外的信息写入文件“info.dat”，包括采样时间，降采样倍数，数据文件名。
            // FILE *fw2;
            // fw2 = fopen("../dat/info.dat", "w");
            // fprintf(fw2, "CL_TS,DOWN_SAMPLE,DATA_FILE_NAME\n");
            // fprintf(fw2, "%g, %d, %s\n", CL_TS, DOWN_SAMPLE, DATA_FILE_NAME);
            // fclose(fw2);
        }
    }
    // 写变量值到文件
    void write_data_to_file(FILE *fw)
    {
        static int bool_animate_on = FALSE;
        static int j = 0, jj = 0; // j,jj for down sampling
        // if((*CTRL).timebase>20)
        {
            if (++j == DOWN_SAMPLE){
                j = 0;
                fprintf(fw, DATA_FORMAT, DATA_DETAILS);
            }
        }
    }
    void print_info(){
        #if WHO_IS_USER == USER_WB
            if ( (*debug).bool_apply_WC_tunner_for_speed_loop == TRUE ){
                printf(">>> Wc_Tuner is Applied to the Speed Loop Control <<<\n");
            }else{
                printf("!!! TI tunner is Applied to the Speed Loop Control !!!\n");
            }
            if (d_sim.user.bool_enable_Harnefors_back_calculation == TRUE){
                printf(">>> Harnefors Back Calculation is Applied <<<\n");
            }else{
                printf("!!! Harnefors Back Calculation is NOT Applied !!!\n");
            }
            if CURRENT_LOOP_KI_TIMES_TEN
                printf(">>> Current Loop Ki is 10 times larger <<<\n");
            else
                printf("!!! Current Loop Ki is NOT 10 times larger !!!\n");
            printf(">>> Speed Loop is 1/%d of the Current Loop <<<\n", d_sim.FOC.VL_EXE_PER_CL_EXE);
            printf(">>> InnerLoop Max CLBW is %fHz and Min CLBW is %fHz\n", 2*d_sim.user.zeta*d_sim.user.omega_n/2/M_PI, 4*d_sim.user.zeta*d_sim.user.omega_n/2/M_PI);
        #endif

        if(d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
            printf(">>> Voltages to Cuurent Regulation is Applied <<<\n");
        }else{
            printf("!!! Voltages to Cuurent Regulation is NOT Applied !!!\n");
        }
        if (!__INVERTER_NONLINEARITY){
            printf("!!! Inverter Nonlinearity is NOT Considered !!!\n");
        }else{
            printf(">>> Inverter Nonlinearity is Considered <<<\n");
        }
        if (d_sim.user.INVERTER_NONLINEARITY_COMPENSATION_METHOD != 0){
            printf(">>> Inverter Nonlinearity Compensation is Applied <<<\n");
        }else{
            printf("!!! Inverter Nonlinearity Compensation is NOT Applied !!!\n");
        }

        #if WHO_IS_USER == USER_BEZIER
            if (d_sim.user.BOOL_BEZIER_ADAPTIVE_GAIN == TRUE){
                printf(">>> Bezier AdaptVersion is Applied <<<\n");
            }else{
                printf("!!! Bezier AdaptVersion is NOT Applied !!!\n");
            }
        #endif

        printf("\t[utility.c] Rreq = %f Ohm\n", (ACM.Rreq));
        printf("\t[utility.c] NUMBER_OF_STEPS = %d\t", (int)d_sim.sim.NUMBER_OF_STEPS);
        printf("MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD = %ld\n", d_sim.sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD);
        // if((*debug).SENSORLESS_CONTROL==TRUE){
        //     printf("\t[utility.c] Sensorless using observer.\n");
        // }else{
        //     printf("\t[utility.c] Sensored control.\n");
        // }
        #if(WHO_IS_USER == USER_CJH)
            if (d_sim.init.Rreq > 0.0) printf("\t[im_controller.c] Marino's alpha: %g in [%g, %g]?\n", marino.xAlpha, marino.xAlpha_min, marino.xAlpha_Max);
        #endif

        printf("\t[utility.c] Speed series PI:   Kp=%.3f, Ki=%.6f, limit=%.1f A\n", PID_Speed->Kp, d_sim.VL.SERIES_KI, PID_Speed->OutLimit);
        printf("\t[utility.c] Current series PI: Kp=%.3f, Ki=%.6f, limit=%.1f V\n", PID_iQ->Kp, d_sim.CL.SERIES_KI_Q_AXIS, PID_iQ->OutLimit);
        printf("\tPID_Speed.Kp = %f\n", PID_Speed->Kp);
        printf("\tPID_Speed.Ki_CODE = %f\n", PID_Speed->Ki_CODE);
        printf("\tPID_iQ.Kp = %f\n", PID_iQ->Kp);
        printf("\tPID_iQ.Ki_CODE = %f\n", PID_iQ->Ki_CODE);
        printf("\tPID_iD.Kp = %f\n", PID_iD->Kp);
        printf("\tPID_iD.Ki_CODE = %f\n", PID_iD->Ki_CODE);
        printf("\td_sim.FOC.CLBW_HZ = %g\n", (REAL)d_sim.FOC.CLBW_HZ);
        printf("\td_sim.FOC.delta = %g\n", (REAL)d_sim.FOC.delta);
        printf("\t(*debug).error = %d\n\t(*debug).mode_select = %d\n\t(*debug).who_is_user = %d\n", (int)(*debug).error, (int)(*debug).mode_select, (int)(*debug).who_is_user);
        #if (WHO_IS_USER == USER_WB)
            printf("\tSIL_Controller.KFB = %f\n",SIL_Controller.KFB);
            printf("\td_sim.user.zeta = %.3f\n", (REAL)d_sim.user.zeta);
            printf("\td_sim.user.omega_n = %.3f\n", (REAL)d_sim.user.omega_n);
            printf("\td_sim.user.max_CLBW_PER_min_CLBW = %.3f\n", (REAL)d_sim.user.max_CLBW_PER_min_CLBW);
        #endif
}
#endif

REAL angle_diff(REAL a, REAL b) {
    // a 和 b 必须在 [0, 2 * M_PI] 范围内
    a = fmod(a, 2 * M_PI);
    b = fmod(b, 2 * M_PI);
    REAL d1 = a - b;
    REAL d2;
    if (d1 > 0) {
        d2 = a - (b + 2 * M_PI); // d2 是负的
    } else {
        d2 = (2 * M_PI + a) - b; // d2 是正的
    }
    if (fabsf(d1) < fabsf(d2)) {
        return d1;
    } else {
        return d2;
    }
}

