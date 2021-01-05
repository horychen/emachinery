#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

typedef struct {
   float32 Ref;
   float32 Fbk;
   float32 Err;
   float32 ErrPrev;
   float32 P_Term; 
   float32 I_Term; 
   float32 D_Term; 
   float32 OutNonSat;
   float32 OutLimit;
   float32 Out;
   float32 OutPrev; // for incremental pid
   float32 Kp;
   float32 Ki;
   float32 Kd;
   float32 SatDiff;
   void (*calc)();
} PID_REG; 
typedef PID_REG *PID_REG_handle;
void PID_calc(PID_REG_handle);
#define PID_REG_DEFAULTS { \
  /*Reference*/ 0.0, \
  /*Feedback*/ 0.0, \
  /*Control Error*/ 0.0, \
  /*Control Error from Previous Step*/ 0.0, \
  /*Proportional Term*/  0.0, \
  /*Integral Term*/  0.0, \
  /*Derivative Term*/  0.0, \
  /*Non-Saturated Output*/  0.0, \
  /*Output Limit*/  0.95, \
  /*Output*/  0.0, \
  /*Output from Previous Step*/  0.0, \
  /*Kp*/  1.0, \
  /*Ki*/  0.001, \
  /*Kd*/  0.0, \
  /*Difference between Non-Saturated Output and Saturated Output*/  0.0, \
  (void (*)(Uint32)) PID_calc \
}
extern PID_REG pid1_iM;
extern PID_REG pid1_iT;
extern PID_REG pid1_pos;
extern PID_REG pid1_spd;
extern PID_REG pid1_ia;
extern PID_REG pid1_ib;
extern PID_REG pid1_ic;

#define pid1_id pid1_iM
#define pid1_iq pid1_iT


void ACMSIMC_PIDTuner();

void commands(REAL *p_rpm_speed_command, REAL *p_amp_current_command);

#endif