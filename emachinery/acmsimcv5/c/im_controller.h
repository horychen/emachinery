#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11
#ifndef IM_CONTROLLER_H
#define IM_CONTROLLER_H

struct ControllerForExperiment {

    double timebase;

    double iMs;
    double iTs;
    double iMs_cmd;
    double iTs_cmd;
    double torque_cmd;
    double uMs_cmd;
    double uTs_cmd;
    double ual;
    double ube;

    double speed_ctrl_err;

    double theta_M;
    double cosT;
    double sinT;

    double e_M;
    double e_T;

    // struct PI_Reg pi_speed;
    // struct PI_Reg pi_iMs;
    // struct PI_Reg pi_iTs;

    double omega_sl;
    double omega_syn;

    double tajima_omg;
    double K_PEM;

    double omg__fb;
    double ial__fb;
    double ibe__fb;

    double speed_fb;
    double psial_fb;
    double psibe_fb;
    double psimod_fb;
    double psimod_fb_inv;

    double rotor_flux_cmd;
    double deriv_fluxModSim;

    double rs;
    double rreq;
    double Lsigma;
    double alpha;
    double Lmu;
    double Lmu_inv;

    double Tload;

    double Js;
    double Js_inv;

    
    double omg_ctrl_err;
    double gamma_omg_transient;
    double gamma_omg_transient_shape;
    double gamma_res_transient;
    double gamma_res_transient_shape;
    double gamma_res2_transient;
    double gamma_res2_transient_shape;
    int resistance_id_on;


    int sensorless;
    int CtrMode;


    Uint32  Tctrl;          // 控制用时间,x个同步中断周期
    Uint32  NT;             //一个同步速周期（2pi电角度内）进入电流环中断的次数
    Uint16  NAdc;           //在一个同步中断周期内ADC的次数

    Uint16 fInc;            // 加速标志
    Uint16 fDec;            // 减速标志
    Uint16 fRched;          // 频率到达标志
    // ----------------------报警指示----------------------
    int SetOverMaxFreq;
    int SetBelowMinFreq;
    int UdcBelowZero;
    int OverModulation;
    // ----------------------VVVF----------------------
    double Freqmax;          // MCtrlPrm[4]=100 x10
    double Freqmin;          // MCtrlPrm[6]=5   x10
    double   DDS;            // Freq Decrease Step
    double   DIS;            // Freq Increase Step

    double   FSET;           // For Set, Hz or rpm
    double   FCUR;           // Freq Current, Hz
    double   Goal;           // Sp Goal ,rpm  //for Ramp Set //~~~
    double   Goal_dot;           // Sp Goal ,rpm  //for Ramp Set //~~~
    int     SpStart;
    double  SpCount;

    double   Speed;          // Speed
    double   Wt;             // w1*t, Synchronous angle
    double   Wht;     //liuhe
    double   W;
    double   myIq;
    double   Ws1;

    double   VCUR;           // Current Voltag Vector Amplify
    double   VCurPerUnit;    // (2*Current Voltag/Udc), Voltag Vector Amplify
    double   Cvf0;           // VVVF curve, Cvf0=P07/P05=311/50;  P07-最大输出电压;P05-转折频率
    double   Udc;

    double   lUdc;
    double   Valfa;          //ab0
    double   Vbeta;

    double   VaPU;               // 通向PWM的三相电压，以Udc/2标幺化
    double   VbPU;
    double   VcPU;

    int     CurDrct[PHASE_NUMBER];  //死区补偿
    double   IAmp;           //输出电流幅值

    double   ITMP;           //计算电流有效值用中间变量
    // ----------------------死区补偿----------------------
    Uint16 CmparBeforeCmpnst[PHASE_NUMBER];     //死区补偿前的比较值
    Uint16 CmparAfterCmpnst[PHASE_NUMBER];      //死区补偿后的比较值

    // SVPWM
    double RealSection;
};
extern struct ControllerForExperiment CTRL;

void CTRL_init();
void control(double speed_cmd, double speed_cmd_dot);


void cmd_fast_speed_reversal(double timebase, double instant, double interval, double rpm_cmd);
void cmd_slow_speed_reversal(double timebase, double instant, double interval, double rpm_cmd);


void controller();


#endif
#endif