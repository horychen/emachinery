
#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct {
   float32 Ref;
   float32 Fdb;
   float32 Err;
   float32 Kp;
   float32 Up; // P output
   float32 Ui; // I output
   float32 Ud; // D output
   float32 OutPreSat;
   float32 OutMax;
   float32 OutMin;
   float32 Out;
   float32 SatErr;
   float32 Ki;
   float32 Kc; // Integral correction gain
   float32 Kd;
   float32 Up1; // Previous proportional output
   int16 SatStatus;
   void (*calc)();
} PIDREG3;
typedef PIDREG3 *PIDREG3_handle;
void pid_reg3_calc(PIDREG3_handle);

extern PIDREG3 pid1_ia;
extern PIDREG3 pid1_ib;
extern PIDREG3 pid1_ic;

extern PIDREG3 pid1_id;
extern PIDREG3 pid1_iq;
extern PIDREG3 pid1_pos;
extern PIDREG3 pid1_spd;
#define PIDREG3_DEFAULTS { \
  /*Ref;*/ 0.0, \
  /*Fdb;*/ 0.0, \
  /*Err;*/ 0.0, \
  /*Kp;*/  1.0, \
  /*Up; // P output*/  0.0, \
  /*Ui; // I output*/  0.0, \
  /*Ud; // D output*/  0.0, \
  /*OutPreSat;*/  0.0, \
  /*OutMax;*/  0.95, \
  /*OutMin;*/  0.95, \
  /*Out;*/  0.0, \
  /*SatErr;*/  0.0, \
  /*Ki;*/  0.001, \
  /*Kc; // Integral correction gain*/  0.0, \
  /*Kd;*/  0.0, \
  /*Up1; // Previous proportional output*/    0, \
  /*tStatus;*/  0.0, \
    (void (*)(Uint32)) pid_reg3_calc \
}






struct ControllerForExperiment{

    double timebase;

    double ual;
    double ube;

    double R;
    double KE;
    double Ld;
    double Lq;

    // double Tload;
    // double rpm_cmd;

    double npp;
    double Js;
    double Js_inv;

    double omg__fb;
    double ial__fb;
    double ibe__fb;
    double psi_mu_al__fb;
    double psi_mu_be__fb;

    double rotor_flux_cmd;

    double omg_ctrl_err;
    double speed_ctrl_err;

    double cosT;
    double sinT;

    double omega_syn;

    double theta_d__fb;
    double id__fb;
    double iq__fb;
    double ud_cmd;
    double uq_cmd;
    double id_cmd;
    double iq_cmd;

    double Tem;
    double Tem_cmd;

    // double theta_M;
    // double iMs;
    // double iTs;
    // double uMs_cmd;
    // double uTs_cmd;
    // double iMs_cmd;
    // double iTs_cmd;

    // struct PID_Reg PID_speed;
    // struct PID_Reg PID_id;
    // struct PID_Reg PID_iq;
};
extern struct ControllerForExperiment CTRL;

void CTRL_init();
void control(double speed_cmd, double speed_cmd_dot);


void cmd_fast_speed_reversal(double timebase, double instant, double interval, double rpm_cmd);
void cmd_slow_speed_reversal(double timebase, double instant, double interval, double rpm_cmd);


void controller();


#endif
