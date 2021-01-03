#ifndef SYNCHRONOUS_MOTOR_H
#define SYNCHRONOUS_MOTOR_H
#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

// The function name for the dynamics
#define MACHINE_DYNAMICS SM_Dynamics
// id, iq, omega, theta_d, theta_d_accum
#define NUMBER_OF_STATES 5

struct SynchronousMachineSimulated{

    double timebase;
    double Ts;

    // State
    double x[NUMBER_OF_STATES];
    double x_dot[NUMBER_OF_STATES];

    // Auxliary
    double omg_elec;
    double rpm;
    double rpm_cmd;
    double rpm_deriv_cmd;
    double TLoad;
    double Tem;

    // Electrical parameter
    double R;
    double Ld;
    double Lq;
    double KE; // psi_PM

    // Mechanical parameter
    double npp;
    double npp_inv;
    double Js;
    double mu_m;

    //
    double ual;
    double ube;
    double ial;
    double ibe;

    double ual_c_dist;
    double ube_c_dist;
    double dist_al;
    double dist_be;

    double theta_d;
    double theta_d_accum;
    double ud;
    double uq;
    double id;
    double iq;

    // phase quantities
    double ua;
    double ub;
    double uc;
    double ia;
    double ib;
    double ic;
    double ea;
    double eb;
    double ec;
};
extern struct SynchronousMachineSimulated ACM;

extern int machine_simulation();
extern void Machine_init();

#endif
#endif