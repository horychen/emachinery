#ifndef SIMUSER_CURY_H
#define SIMUSER_CURY_H

#if WHO_IS_USER == USER_CURY

// this offset is moved to ACMconfig.h
// #define OFFSET_COUNT_BETWEEN_ENCODER_INDEX_AND_U_PHASE_AXIS 2333 // cjh tuned with id_cmd = 3A 2024-01-19

#define NO_POSITION_CONTROL 0
#define TWOMOTOR_POSITION_CONTROL 1
#define SINGLE_POSITION_CONTROL 2
#define SHANK_LOOP_RUN 3
#define HIP_LOOP_RUN 4
#define BOTH_LOOP_RUN 5
#define IMPEDANCE_CONTROL 6


void control_two_motor_position();
REAL call_position_loop_controller(int positionLoopType);


#endif
#endif