#ifndef ACMCONFIG_H
#define ACMCONFIG_H

// 	// 参数误差
// 		#define MISMATCH_R    100.0
// 		#define MISMATCH_RREQ 100.0
// 		#define MISMATCH_LD   100.0
// 		#define MISMATCH_LQ   100.0
// 		#define MISMATCH_KE   100.0
//         #define MISMATCH_LMU  100.0


// 	// 磁链给定
// 	#define IM_MAGNETIZING_INDUCTANCE   (d_sim.init.Ld - d_sim.init.Lq)
// 	#define IM_FLUX_COMMAND_DC_PART     d_sim.init.KE // 1.3593784874408608
// 	#define IM_FLUX_COMMAND_SINE_PART   0.0
// 	#define IM_FLUX_COMMAND_SINE_HERZ   10

// #if MACHINE_TYPE % 10 == 2
// 	#define CORRECTION_4_SHARED_FLUX_EST INIT_KE
//     #define U_MOTOR_KE                   INIT_KE
// #else
// 	#define CORRECTION_4_SHARED_FLUX_EST    IM_FLUX_COMMAND_DC_PART
//     #define U_MOTOR_R                       IM_STAOTR_RESISTANCE    // typo!
//     #define U_MOTOR_RREQ                    IM_ROTOR_RESISTANCE
// #endif

/* 经常要修改的 */
// #define INVERTER_NONLINEARITY_COMPENSATION_INIT 0 // 5（9月1日及以前峣杰实验一直用的5） // 4 // 1:ParkSul12, 2:Sigmoid, 3:LUT(Obsolete), 4:LUT(by index), 5 Slessinv-a2a3Model
// #define INVERTER_NONLINEARITY                   0 // 4 // 1:ModelSul96, 2:ModelExpSigmoid, 3: ModelExpLUT, 4:LUT(by index)
// #define SENSORLESS_CONTROL FALSE
// #define SENSORLESS_CONTROL_HFSI FALSE

/* ParkSul2012 梯形波 */
// #define GAIN_THETA_TRAPEZOIDAL (40) //(500) // 20

// /* 电机类型 */ //（TODO：饱和模型里面用的还是 IM.rr 而不是 IM.rreq）
//     #define INDUCTION_MACHINE_CLASSIC_MODEL 1
//     #define INDUCTION_MACHINE_FLUX_ONLY_MODEL 11
//     #define PM_SYNCHRONOUS_MACHINE 2
// #define MACHINE_TYPE 1


// #if APPLY_WCTUNER
//     #define CURRENT_KP (0.3063)   // (1.19)    // (0.6)  // (0.84)   // (0.84)   // (0.84)
//     #define CURRENT_KI (231.578947368421) // (194.74) // (194.74)  // (194.74) // (194.74) // (194.74)
//     #define SPEED_KP   (1.3099)   // (0.19)   // (2.59)  // (0.81)   // (0.41)   // (0.36)
//     #define SPEED_KI   (763.5600)   // (3.93)    // (13.96)  // (11.00)  // (2.75)   // (19.55)
//     #define SPEED_KFB  (2.36)
// #else
//     #define CURRENT_KP (0.9550441666912972)   // (1.19)    // (0.6)  // (0.84)   // (0.84)   // (0.84)
//     #define CURRENT_KI (231.578947368421) // (194.74) // (194.74)  // (194.74) // (194.74) // (194.74)
//     #define SPEED_KP   (0.5896379954298365)   // (0.19)   // (2.59)  // (0.81)   // (0.41)   // (0.36)
//     #define SPEED_KI   (2.0106192982974678)   // (3.93)    // (13.96)  // (11.00)  // (2.75)   // (19.55)
//     #define SPEED_KFB  (0)
// #endif



// /* 指令类型 */
//     #define EXCITATION_POSITION 0
//     #define EXCITATION_SWEEP_FREQUENCY 2
//     #define EXCITATION_VELOCITY 1
// #define EXCITATION_TYPE (1)

// /* Sweep Frequency */
// #define SWEEP_FREQ_MAX_FREQ 200
// #define SWEEP_FREQ_INIT_FREQ 2
// #define SWEEP_FREQ_VELOCITY_AMPL 500
// #define SWEEP_FREQ_CURRENT_AMPL 1
// #define SWEEP_FREQ_C2V FALSE
// #define SWEEP_FREQ_C2C FALSE


#endif
