#ifndef ACMCONFIG_H
#define ACMCONFIG_H

// 电机类型
    #define INDUCTION_MACHINE_CLASSIC_MODEL 1
    #define INDUCTION_MACHINE_FLUX_ONLY_MODEL 11
    #define PM_SYNCHRONOUS_MACHINE 2
#define MACHINE_TYPE 11
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

// 控制策略
	#define INDIRECT_FOC 1
#define CONTROL_STRATEGY INDIRECT_FOC
#define NUMBER_OF_STEPS 160000
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

#define VL_TS          (0.0002)
#define PL_TS VL_TS
#define SPEED_LOOP_CEILING (4)

#define CLARKE_TRANS_TORQUE_GAIN (1.5) // consistent with experiment
#define POW2AMPL (0.816496581) // = 1/sqrt(1.5) power-invariant to aplitude-invariant (the dqn vector becomes shorter to have the same length as the abc vector)
#define AMPL2POW (1.22474487)

#define LOAD_INERTIA    0.16
#define LOAD_TORQUE     0.0
#define VISCOUS_COEFF   7e-05

#define MOTOR_RATED_TORQUE ( MOTOR_RATED_POWER_WATT / (MOTOR_RATED_SPEED_RPM/60.0*2*3.1415926) )
#define MOTOR_TORQUE_CONSTANT ( MOTOR_RATED_TORQUE / (MOTOR_RATED_CURRENT_RMS*1.414) )
#define MOTOR_BACK_EMF_CONSTANT ( MOTOR_TORQUE_CONSTANT / 1.5 / MOTOR_NUMBER_OF_POLE_PAIRS )
#define MOTOR_BACK_EMF_CONSTANT_mV_PER_RPM ( MOTOR_BACK_EMF_CONSTANT * 1e3 / (1.0/MOTOR_NUMBER_OF_POLE_PAIRS/2/3.1415926*60) )

#define CURRENT_KP (1499.72)
#define CURRENT_KI (6.42842)
#define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)
#define SPEED_KP (8.36295)
#define SPEED_KI (75.061)
#define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)

#define SPEED_LOOP_PID_PROPORTIONAL_GAIN 0.00121797
#define SPEED_LOOP_PID_INTEGRAL_TIME_CONSTANT (1/312.3)
#define SPEED_LOOP_PID_DIREVATIVE_TIME_CONSTANT 0
#define SPEED_LOOP_LIMIT_NEWTON_METER (1*MOTOR_RATED_TORQUE)
#define SPEED_LOOP_LIMIT_AMPERE (1*1.414*MOTOR_RATED_CURRENT_RMS)

#define CURRENT_LOOP_PID_PROPORTIONAL_GAIN 9.2363 // (CL_TS_INVERSE*0.1*PMSM_D_AXIS_INDUCTANCE) // 9.2363
#define CURRENT_LOOP_PID_INTEGRAL_TIME_CONSTANT (1/352.143)
#define CURRENT_LOOP_PID_DIREVATIVE_TIME_CONSTANT 0
#define CURRENT_LOOP_LIMIT_VOLTS (600)
#define DATA_FILE_NAME "../dat/IM_Velocity-505-1000-18-2624.dat"
#define PC_SIMULATION TRUE

#define MACHINE_TS         (CL_TS*TS_UPSAMPLING_FREQ_EXE) //1.25e-4 
#define MACHINE_TS_INVERSE (CL_TS_INVERSE*TS_UPSAMPLING_FREQ_EXE_INVERSE) // 8000

#define SWEEP_FREQ_MAX_FREQ 200
#define SWEEP_FREQ_INIT_FREQ 2
#define SWEEP_FREQ_VELOCITY_AMPL 500
#define SWEEP_FREQ_CURRENT_AMPL 1
#define SWEEP_FREQ_C2V FALSE
#define SWEEP_FREQ_C2C FALSE

#endif
