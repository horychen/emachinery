#ifndef ADD_OBSERVER_H
#define ADD_OBSERVER_H

/* Macro for External Access Interface */
#define US(X) sm.us[X]
#define IS(X) sm.is[X]
#define US_C(X) sm.us_curr[X]
#define IS_C(X) sm.is_curr[X]
#define US_P(X) sm.us_prev[X]
#define IS_P(X) sm.is_prev[X]

struct SynchronousMachine{
    REAL us[2];
    REAL is[2];
    REAL us_curr[2];
    REAL is_curr[2];
    REAL us_prev[2];
    REAL is_prev[2];
    REAL is_lpf[2];
    REAL is_hpf[2];
    REAL is_bpf[2];

    REAL current_lpf_register[2];
    REAL current_hpf_register[2];
    REAL current_bpf_register1[2];
    REAL current_bpf_register2[2];

    REAL npp;
    REAL npp_inv;

    REAL R;
    REAL Ld;
    REAL Ld_inv;
    REAL Lq;
    REAL KE;

    REAL Js;
    REAL Js_inv;

    REAL omg_elec; // omg_elec = npp * omg_mech
    REAL omg_mech;
    REAL theta_d;

    REAL omg_elec_hall;
    REAL omg_mech_hall;
    REAL theta_d_hall;

    REAL last_omg_elec_hall;
    REAL acceleration_avg;

    int16 hallA;
    int16 hallB;
    int16 hallC;
    int16 hallABC;
    int16 last_hallABC;
    int16 speed_direction;
    int16 bool_position_correction_pending;

    REAL timebase;
    REAL timeStamp;
    REAL timeDifference;
    REAL timeDifferenceStamp;



    /* In hall_resolver() */

    REAL hall_speedRad; // 电气转速输出
    REAL hall_speedRad_pre; // 用于低通滤波（无效）
    REAL hall_speedRad_tmp; // 这个变量其实是不必要的，等于60°/时间

    Uint32 hall_speed_cnt;
    Uint32 hall_speed_cnt_pre; // 作为扇区内计数的一个指标，标志着电机转速是否慢于上一个扇区。只能判断电机是不是变慢了。

    REAL hall_change_cnt; // 电机走过了几个扇区？一般情况下是1。
    Uint16 hall_change_angle; // 两个扇区角度的平均值，在切换扇区时，等于N极的位置

    Uint16 hall_angle_est; // 电气角度输出
    REAL hall_angle_est_tmp; // 基于霍尔反馈的角度估计的增量，限幅在[-60, 60] elec. deg

    Uint16 hall_previous_previous_angle; // 上上个扇区角度
    Uint16 hall_previous_angle; // 上个扇区角度
    Uint16 hall_current_angle; // 当前扇区角度
};
extern struct SynchronousMachine sm;


void sm_init();

#endif

