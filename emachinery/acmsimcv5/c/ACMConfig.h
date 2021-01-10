#ifndef ACMCONFIG_H
#define ACMCONFIG_H
/* Consistent with DSP Codes */
// 电机类型
    #define INDUCTION_MACHINE_CLASSIC_MODEL 1
    #define INDUCTION_MACHINE_FLUX_ONLY_MODEL 11
    #define PM_SYNCHRONOUS_MACHINE 2
#define MACHINE_TYPE 1
	#define IM_STAOTR_RESISTANCE        3.04
	#define IM_ROTOR_RESISTANCE         1.6
	#define IM_TOTAL_LEAKAGE_INDUCTANCE 0.0249
	#define IM_MAGNETIZING_INDUCTANCE   0.448
	#define IM_FLUX_COMMAND_DC_PART     0.7105842093440861
	#define IM_FLUX_COMMAND_SINE_PART   0.0
	#define MOTOR_NUMBER_OF_POLE_PAIRS  2
	#define MOTOR_RATED_CURRENT_RMS     8.8
	#define MOTOR_RATED_POWER_WATT      4000
	#define MOTOR_RATED_SPEED_RPM       1440
	#define MOTOR_SHAFT_INERTIA         0.063

// 指令类型
    #define EXCITATION_POSITION 0
    #define EXCITATION_VELOCITY 1
    #define EXCITATION_SWEEP_FREQUENCY 2
#define EXCITATION_TYPE (1)
    #define IM_FLUX_COMMAND_SINE_HERZ   (1)

// 控制策略
	#define INDIRECT_FOC 1
	#define MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL 2
#define CONTROL_STRATEGY MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL
#define NUMBER_OF_STEPS 100000
#define DOWN_SAMPLE 1
#define SENSORLESS_CONTROL FALSE
#define SENSORLESS_CONTROL_HFSI FALSE
#define VOLTAGE_CURRENT_DECOUPLING_CIRCUIT TRUE
#define SATURATED_MAGNETIC_CIRCUIT FALSE
#define INVERTER_NONLINEARITY FALSE
#define CL_TS          (5e-05)
#define CL_TS_INVERSE  (20000)
#define TS_UPSAMPLING_FREQ_EXE 0.5
#define TS_UPSAMPLING_FREQ_EXE_INVERSE 2

// 调参
#define GAMMA_INV_xTL 25714.285714285714
#define LAMBDA_INV_xOmg 9000.0

#define VL_TS          (0.0002)
#define PL_TS VL_TS
#define SPEED_LOOP_CEILING (4)

#define LOAD_INERTIA    0.16
#define LOAD_TORQUE     2
#define VISCOUS_COEFF   0.007

#define CURRENT_KP (610.52)
#define CURRENT_KI (6.42842)
#define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)
#define SPEED_KP (3.40446)
#define SPEED_KI (30.5565)
#define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)

#define SWEEP_FREQ_MAX_FREQ 200
#define SWEEP_FREQ_INIT_FREQ 2
#define SWEEP_FREQ_VELOCITY_AMPL 500
#define SWEEP_FREQ_CURRENT_AMPL 1
#define SWEEP_FREQ_C2V FALSE
#define SWEEP_FREQ_C2C FALSE

#define PC_SIMULATION TRUE
#define DATA_FILE_NAME "../dat/IM_Transient-205-1000-7-2624.dat"
#endif
