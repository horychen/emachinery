#ifndef PMSM_COMMISSIONING_H
#define PMSM_COMMISSIONING_H

#if ENABLE_COMMISSIONING || WHO_IS_USER == USER_WB

#define COMM_IV_SIZE_R1 200 // 300
#define COMM_IV_SIZE_L1 30
struct CommissioningDataStruct{

    int bool_comm_status;

    REAL timebase;
    int32 counterEntered;
    int16 i;

    REAL R;
    REAL L;
    REAL R3;
    REAL L3;
    REAL KE;
    REAL Js; // shaft inertia

    // R
    REAL current_command;
    REAL current_sum;
    REAL voltage_sum;
    int32 counterSS;
    int16 bool_collecting;
    REAL i_phase_data_R1[COMM_IV_SIZE_R1];
    REAL v_phase_data_R1[COMM_IV_SIZE_R1];
    REAL i_data_L1[COMM_IV_SIZE_L1];
    REAL t_data_L1[COMM_IV_SIZE_L1];
    REAL inverter_voltage_drop;

    // L
    REAL last_voltage_command;
    REAL id_prev;
    REAL iq_prev;

    // Js
    int16 number_of_repeats_Js;
};
extern struct CommissioningDataStruct COMM;
extern int16 bool_single_phase_excitation;

void COMM_PI_tuning(REAL LL, REAL RR, REAL BW_current, REAL delta, REAL JJ, REAL KE, REAL npp);

void init_COMM();
void COMM_resistanceId(REAL id_fb, REAL iq_fb);

void COMM_inductanceId(REAL id_fb, REAL iq_fb);
void COMM_inductanceId_ver2(REAL id_fb, REAL iq_fb);
void COMM_inductanceId_ver3(REAL id_fb, REAL iq_fb);

void COMM_PMFluxId(REAL id_fb, REAL iq_fb, REAL omg_elec_fb);

void COMM_inertiaId(REAL id_fb, REAL iq_fb, REAL cosPark, REAL sinPark, REAL omg_elec_fb);
void COMM_end(REAL id_fb, REAL iq_fb);

// Main Procedure
#define USING_OLD_COMM FALSE
void StepByStepCommissioning();
void StepByStepCommissioning_NEW_WB();
void StepbyStepCommissioning_OLD();

#endif
#endif
