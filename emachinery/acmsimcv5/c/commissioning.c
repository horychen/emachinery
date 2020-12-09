#include "ACMSim.h"

#ifdef COMMISSIONING_STATE_MACHINE

// 声明参数自整定结构体变量
struct CommissioningDataStruct COMM;

// 自整定初始化
void COMM_init(){
    COMM.timebase = 0.0;

    // R
    COMM.current_sum = 0.0;
    COMM.voltage_sum = 0.0;
    COMM.counterSS = 0;
    COMM.bool_collecting = FALSE;
    int i;
    for(i=0;i<100;++i){
        COMM.iv_data[i][0] = 0.0;
        COMM.iv_data[i][1] = 0.0;
    }

    // L
    COMM.L = 0.999;
}

// SS: Steady State
#define SS_RATED_CURRENT_RATIO 1e-2 // 电流误差在额定电流的 0.01 倍以下时，认为 有可能 达到电流调节稳态了
#define SS_COUNTER_CEILING ((long int)(0.2/CL_TS)) // 达到稳态后，计数，超过这个值才认为真正进入稳态
int reachSteadyStateCurrent(double current_error, double rated_current){
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

// 定义枚举变量，状态码和返回码，有下划线_前缀的，都是状态码。
enum state_codes { _nameplateData=0, _currentSensor=1, _initialPosId=2, _resistanceId=3, _inductanceId=4, _PMFluxLinkageId=5, _inertiaId=6, _unexpected=7, _end=8 };
enum ret_codes   { ok=0, repeat=1, quit=2 };

// 0：名牌数据输入事件
int event_nameplateData(void){
    printf("Start self-commissioning!\n");
    COMM.npp = 2;
    COMM.IN = 8;
    printf("Name plate data: npp=%d, IN=%g.\n", COMM.npp, COMM.IN);

    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    return ok;
}
// 1：电流传感器校正事件
int event_currentSensor(void){
    static long int counterEntered = 0;
    if(counterEntered++==0)
        printf("Current Sensor Calibration (pass).\n");

    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    return ok;
}
// 2：磁极位置辨识事件
int event_initialPosId(void){
    printf("Initial Position Identification (pass).\n");
    return ok;
}
// 3：电阻辨识事件
int event_resistanceId(void){
    static long int counterEntered = 0;
    if(counterEntered++==0)
        printf("Resistance Identification.\n Current [A] | Voltage [V]\n");

    #define RS_ID_NUMBER_OF_STAIRS 10
    #define RS_ID_MAXIMUM_CURRENT COMM.IN
    double current_increase_per_step = RS_ID_MAXIMUM_CURRENT / RS_ID_NUMBER_OF_STAIRS;
    static int i = 0;
    COMM.current_command = - RS_ID_MAXIMUM_CURRENT;
    COMM.current_command += current_increase_per_step*i;

    pid1_id.Fdb = IS_C(0);
    pid1_id.Ref = COMM.current_command;
    // printf("%g, %g\n", pid1_id.Fdb, pid1_id.Ref);
    pid1_id.calc(&pid1_id);
    CTRL.ual = pid1_id.Out;
    CTRL.ube = 0.0;

    // collect steady state data
    if(COMM.bool_collecting){
        COMM.current_sum += IS_C(0);
        COMM.voltage_sum += CTRL.ual;
        COMM.counterSS += 1;

        if(COMM.counterSS>800){
            COMM.iv_data[i][0] = COMM.current_sum/(double)COMM.counterSS;
            COMM.iv_data[i][1] = COMM.voltage_sum/(double)COMM.counterSS;
            printf("%f, %f\n", COMM.iv_data[i][0], COMM.iv_data[i][1]);
            COMM.bool_collecting = FALSE;
            ++i;
        }
    }else{
        // reset
        COMM.current_sum = 0.0;
        COMM.voltage_sum = 0.0;
        COMM.counterSS = 0;

        // check steady state and assign boolean variable
        if(reachSteadyStateCurrent(pid1_id.Ref-pid1_id.Fdb, RS_ID_MAXIMUM_CURRENT)){
            if(COMM.current_command > RS_ID_MAXIMUM_CURRENT){

                // Get resistance value
                COMM.R = (COMM.iv_data[0][1] - COMM.iv_data[1][1]) / (COMM.iv_data[0][0] - COMM.iv_data[1][0]);
                // COMM.R = (COMM.iv_data[18+0][1] - COMM.iv_data[18+1][1]) / (COMM.iv_data[18+0][0] - COMM.iv_data[18+1][0]);
                printf("R=%g Ohm (including inverter's on-resistance)\n", COMM.R);
                return ok;
            }

            COMM.bool_collecting = TRUE;
        }
    }

    return repeat;
}
// 4：电感辨识事件
int event_inductanceId(void){
    static double last_voltage_command;
    static long int counterEntered = 0;
    if(counterEntered++==0){
        printf("Inductance Identification.\n");
        last_voltage_command = CTRL.ual;
    }

    double Delta_IS_al;
    double Delta_US_al;
    double IS_slope;

    Delta_US_al = 0.5*last_voltage_command;
    Delta_IS_al = IS_C(0) - IS_P(0);
    IS_slope = Delta_IS_al / CL_TS;

    if(counterEntered<20){
        printf("L=%g\n", Delta_US_al/IS_slope );
        if(Delta_US_al/IS_slope < COMM.L){
            COMM.L = Delta_US_al/IS_slope;
        }
    }else{
        if(counterEntered==20){
            printf("COMM.L = %g\n", COMM.L);
        }
    }

    CTRL.ual = last_voltage_command + Delta_US_al;
    CTRL.ube = 0.0;

    if(counterEntered>800){
        return ok;
    }else{
        return repeat;
    }
}
// 5：永磁体磁链（反电势系数）辨识事件
int event_PMFluxLinkageId(void){
    static long int counterEntered = 0;
    if(counterEntered++==0)
        printf("PM Flux Linkage Identification.\n");

    double rpm_speed_command = 100; 
    double ud_cmd;
    double uq_cmd;
    ud_cmd = CTRL.ud_cmd;
    uq_cmd = CTRL.uq_cmd;
    control(rpm_speed_command, 0);

    static double last_KE;
    last_KE = COMM.KE;

    COMM.KE = (uq_cmd - COMM.R*CTRL.iq__fb) / (rpm_speed_command*RPM_2_RAD_PER_SEC) - COMM.L*CTRL.id__fb;

    // KE便是指是否达到稳态？
    if(fabs(COMM.KE - last_KE)>=1e-2){
        counterEntered = 1; // 复用一下这个计数器用于判断KE是否收敛，可以少声明一个计数器。
    }

    // 转速控制是否达到稳态？
    if(fabs(pid1_spd.Ref - pid1_spd.Fdb)<1e-1){
        if(counterEntered > 16000){        
            printf("KE = %g\n", COMM.KE);
            return ok;
        }
    }
    return repeat;
}
// 6：惯量辨识事件
#define TEST_SIGNAL_FREQUENCY 0.5
#define TEST_SIGNAL_PERIOD (2)
#define SPEED_COMMAND_BIAS (200*0)
#define SPEED_COMMAND_RANGE 10
int event_inertiaId(void){

    static long int counterEntered = 0;
    if(counterEntered++==0)
        printf("Inertia Identification.\n");

    #define STATOR_CURRENT_ALPHA IS_C(0)
    #define STATOR_CURRENT_BETA IS_C(1)
    #define ROTOR_FLUX_ALPHA MT2A(COMM.KE, 0.0, CTRL.cosT, CTRL.sinT)
    #define ROTOR_FLUX_BETA  MT2B(COMM.KE, 0.0, CTRL.cosT, CTRL.sinT)
    #define NORMINAL_INERTIA (0.10) // 随便给个数量级差不多的数，不同数值只对画图有影响
    #define SPEED_SIGNAL (CTRL.omg__fb)

    #define AWAYA_LAMBDA (31.4)

    // Inertia Observer 2 Awaya1992 
    static double t = 0.0;
    t += CL_TS;

    static double q0 = 0.0;
    static double q1 = 0.0;
    static double q1_dot = 0.0;
    static double q2 = 0.0;

    static double tau_est = 0.0;

    // double Tem = CTRL.Tem;
    double Tem = COMM.npp*( STATOR_CURRENT_ALPHA * -ROTOR_FLUX_BETA + STATOR_CURRENT_BETA * ROTOR_FLUX_ALPHA);
    q0 += CL_TS * AWAYA_LAMBDA*( -q0 + Tem);

    q1_dot = AWAYA_LAMBDA*( -q1 + SPEED_SIGNAL/COMM.npp );
    q1 += CL_TS * q1_dot;

    tau_est = -(NORMINAL_INERTIA)*q1_dot + q0;

    static double sum_A = 0.0;
    static double sum_B = 0.0;
    static double est_Js_variation = 0.0;
    static double est_Js = NORMINAL_INERTIA;

    sum_A += CL_TS * (tau_est*q1_dot);
    sum_B += CL_TS * (q1_dot*q1_dot);
    if(t >= TEST_SIGNAL_PERIOD){      
        if(sum_B<0.001){
            sum_B = 0.001; // avoid zero denominator
        }
        est_Js_variation = + sum_A / sum_B;
        est_Js = NORMINAL_INERTIA + est_Js_variation;
        t = 0.0;
        sum_A = 0.0;
        sum_B = 0.0;
    }

    // 限幅，只影响画图
    if(est_Js>0.1){
        est_Js = 0.1;
    }else if(est_Js<0.001){
        est_Js = 0.001;
    }
    COMM.Js = est_Js;

    control(SPEED_COMMAND_BIAS+SPEED_COMMAND_RANGE*cos(2*M_PI*TEST_SIGNAL_FREQUENCY*CTRL.timebase), 0.0);

    return repeat;
}
// 7：意外事件
int event_unexpected(void){
    printf("Debug!");
    return quit;
}
// 8：状态机结束
int event_end(void){
    printf("Exit!");
    printf("END.");
    return quit;
}

// 定义指针，初始化为上述事件函数的地址
// Reference: https://www.geeksforgeeks.org/enumeration-enum-c/
int (* event[])(void) = { event_nameplateData, event_currentSensor, event_initialPosId, event_resistanceId, event_inductanceId, 
                          event_PMFluxLinkageId, event_inertiaId, event_unexpected, event_end };
// 状态转移表格，本质上是一个二维数列，该二维数列由8个一维数列组成，每个一维数列有3个元素组成，分别对应三种返回码下的状态转移操作
int lookup_transitions[][3] = {
                       // return codes:
                       //      ok            repeat           quit
    [_nameplateData]   = {_currentSensor  , _nameplateData  , _end},
    [_currentSensor]   = {_initialPosId   , _currentSensor  , _end},
    [_initialPosId]    = {_resistanceId   , _initialPosId   , _end},
    [_resistanceId]    = {_inductanceId   , _resistanceId   , _end},
    [_inductanceId]    = {_PMFluxLinkageId, _inductanceId   , _end},
    [_PMFluxLinkageId] = {_inertiaId      , _PMFluxLinkageId, _end},
    [_inertiaId]       = {_end            , _inertiaId      , _end},
    [_unexpected]      = {_end            , _unexpected     , _end},
    /* transitions from end state aren't needed */
};

// 用宏去定义开始状态和结束状态所对应的状态码
#define ENTRY_STATE _nameplateData
#define END_STATE   _end

// 参数自整定状态机
void commissioning(){

    static enum state_codes cur_state = ENTRY_STATE;
    static enum ret_codes rc;
    static int (* state_func)(void);

    // printf("cur_state=%d\n", cur_state);

    if(cur_state!=END_STATE){
        // 根据当前状态码cur_state获取对应的事件函数的地址
        state_func = event[cur_state];
        // 调用该事件函数，获取返回码rc（Return Code）
        rc = state_func();
        // 根据状态转矩表格，按照当前状态码和对应的返回码，查询下一状态码
        cur_state = lookup_transitions[cur_state][rc];
    }

    // 将当前步电流存为上一步电流，供下一步使用。
    COMM.id_prev = IS_C(0);
    COMM.iq_prev = IS_C(1);
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}
#endif