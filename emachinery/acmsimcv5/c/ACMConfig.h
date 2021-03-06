#ifndef ACMCONFIG_H
#define ACMCONFIG_H
/* Consistent with DSP Codes */
// 电机类型
    #define INDUCTION_MACHINE_CLASSIC_MODEL 1
    #define INDUCTION_MACHINE_FLUX_ONLY_MODEL 11
    #define PM_SYNCHRONOUS_MACHINE 2
#define MACHINE_TYPE 2
	#define PMSM_RESISTANCE                    0.152
	#define PMSM_D_AXIS_INDUCTANCE             0.00046600000000000005
	#define PMSM_Q_AXIS_INDUCTANCE             0.00046600000000000005
	#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE 0.011724639454177425
	#define MOTOR_NUMBER_OF_POLE_PAIRS         4
	#define MOTOR_RATED_CURRENT_RMS            12.8
	#define MOTOR_RATED_POWER_WATT             400
	#define MOTOR_RATED_SPEED_RPM              3000
#define MOTOR_SHAFT_INERTIA 2.4000000000000004e-05

// 指令类型
    #define EXCITATION_POSITION 0
    #define EXCITATION_VELOCITY 1
    #define EXCITATION_SWEEP_FREQUENCY 2
#define EXCITATION_TYPE (1)
    #define IM_FLUX_COMMAND_SINE_HERZ   (1)

// 控制策略
	#define NULL_D_AXIS_CURRENT_CONTROL -1
	#define MTPA -2 // not supported
#define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL
#define NUMBER_OF_STEPS 80000
#define DOWN_SAMPLE 1
#define SENSORLESS_CONTROL 1
#define SENSORLESS_CONTROL_HFSI FALSE
#define VOLTAGE_CURRENT_DECOUPLING_CIRCUIT TRUE
#define SATURATED_MAGNETIC_CIRCUIT FALSE
#define INVERTER_NONLINEARITY FALSE
#define CL_TS          (5e-05)
#define CL_TS_INVERSE  (20000)
#define TS_UPSAMPLING_FREQ_EXE 0.5
#define TS_UPSAMPLING_FREQ_EXE_INVERSE 2

// 调参

#define VL_TS          (0.0002)
#define PL_TS VL_TS
#define SPEED_LOOP_CEILING (4)

#define LOAD_INERTIA    0.16
#define LOAD_TORQUE     0
#define VISCOUS_COEFF   0.0

#define CURRENT_KP (0.601612)
#define CURRENT_KI (326.18)
#define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)
#define SPEED_KP (0.0131004)
#define SPEED_KI (30.5565)
#define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)

#define SWEEP_FREQ_MAX_FREQ 200
#define SWEEP_FREQ_INIT_FREQ 2
#define SWEEP_FREQ_VELOCITY_AMPL 500
#define SWEEP_FREQ_CURRENT_AMPL 1
#define SWEEP_FREQ_C2V FALSE
#define SWEEP_FREQ_C2C FALSE

#define PC_SIMULATION TRUE
#define DATA_FILE_NAME "../dat/PMSM_Transient-205-1000-112-173.dat-004"
#endif
