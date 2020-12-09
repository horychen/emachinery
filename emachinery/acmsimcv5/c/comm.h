
#ifndef COMMISSIONING_H
#define COMMISSIONING_H

#define COMM_IV_SIZE 100
struct CommissioningDataStruct{

    float32 timebase;
    int32 counterEntered;
    int16 i;

    float32 R;
    float32 L;
    float32 R3;
    float32 L3;
    float32 KE;
    float32 Js; // shaft inertia

    // R
    float32 current_command;
    float32 current_sum;
    float32 voltage_sum;
    int32 counterSS;
    int16 bool_collecting;
    float32 i_data[COMM_IV_SIZE];
    float32 v_data[COMM_IV_SIZE];
    float32 inverter_voltage_drop;

    // L
    float32 last_voltage_command;
    float32 id_prev;
    float32 iq_prev;
};
extern struct CommissioningDataStruct COMM;
extern int16 bool_comm_status;

void COMM_PI_tuning(REAL LL, REAL RR, REAL BW_current, REAL delta, REAL JJ, REAL KE, REAL npp);

void COMM_init();
void COMM_resistanceId(REAL id_fb, REAL iq_fb);

void COMM_inductanceId(REAL id_fb, REAL iq_fb);
void COMM_inductanceId_ver2(REAL id_fb, REAL iq_fb);
void COMM_inductanceId_ver3(REAL id_fb, REAL iq_fb);

void COMM_PMFluxId(REAL id_fb, REAL iq_fb, REAL omg_elec_fb);

void COMM_intertiaId(REAL id_fb, REAL iq_fb, REAL cosPark, REAL sinPark, REAL omg_elec_fb);
void COMM_end(REAL id_fb, REAL iq_fb);

#endif

// #define PMSM_NUMBER_OF_POLE_PAIRS 4
// #define PMSM_RESISTANCE 0.95
// #define PMSM_D_AXIS_INDUCTANCE 0.0041
// #define PMSM_Q_AXIS_INDUCTANCE 0.0041
// #define PMSM_SHAFT_INERTIA (0.035*1e-4)
// #define PMSM_RATED_CURRENT_RMS 3.5
// #define PMSM_RATED_POWER_WATT  100
// #define PMSM_RATED_SPEED_RPM   3000

//             /* Start *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/

//             // 使用测量得到的电流、转速、位置等信息，生成电压指令CTRL.ual，CTRL.ube
//             // commissioning();
//             if(bool_comm_status == 0){
//                 // 初始化
//                 COMM_init();
//             }
//             else if(bool_comm_status == 1){
//                 // 电阻辨识
//                 COMM_resistanceId();
//             }else if(bool_comm_status == 2){
//                 // 电阻辨识
//                 COMM_inductanceId();
//             }else{
                
//             }

//             /* End *~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*/
