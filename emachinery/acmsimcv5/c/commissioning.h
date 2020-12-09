
#ifndef COMMISSIONING_H
#define COMMISSIONING_H

struct CommissioningDataStruct{

    float32 timebase;

    int16 npp; // number of pole pairs
    float32 IN; // rated line current in Ampere RMS

    float32 R;
    float32 L;
    float32 KE;
    float32 Js; // shaft inertia

    float32 current_command;

    float32 current_sum;
    float32 voltage_sum;
    int32 counterSS;
    int16 bool_collecting;
    float32 iv_data[100][2];


    float32 id_prev;
    float32 iq_prev;
};
extern struct CommissioningDataStruct COMM;

void COMM_init();
void commissioning();

#endif
