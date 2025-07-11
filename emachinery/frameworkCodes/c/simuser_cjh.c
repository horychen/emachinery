#include "ACMSim.h"
#if WHO_IS_USER == USER_CJH
	#define CORRECTION_4_SHARED_FLUX_EST d_sim.init.KE
    #define U_MOTOR_KE                   d_sim.init.KE
    #define IM_FLUX_COMMAND_DC_PART     d_sim.init.KE // 1.3593784874408608
    #define IM_FLUX_COMMAND_SINE_PART   0.0
/* 逆变器非线性 */

/* 拟合法 */
#ifdef _XCUBE1
// b-phase @80 V Mini6PhaseIPMInverter
REAL sig_a2 = 7.705938744542915;
REAL sig_a3 = 67.0107635483658;

// 180V (old)
// REAL sig_a2 = 16.0575341;  // yuefei tuning gives a2' = 9.2*2
// REAL sig_a3 = 17.59278688; // yuefei tuning gives a3' = a3*5
#else
// 80 V
// REAL sig_a2 = 6.67159129;
// REAL sig_a3 = 8.42010418;

// 100 V from SlessInv paper
REAL sig_a2 = 6.7;
REAL sig_a3 = 5.6;

// 150 V
// REAL sig_a2 = 13.25723639;
// REAL sig_a3 = 5.6420585;

// 180 V
// REAL sig_a2 = 15.43046115;
// REAL sig_a3 = 5.04010863;
#endif
REAL sigmoid(REAL x)
{
    // beta
    // REAL a1 = 0.0; // *1.28430164
    // REAL a2 = 8.99347107;
    // REAL a3 = 5.37783655;

    // -------------------------------

#ifdef _XCUBE1
    /* Si Mini OW IPM Inverter*/

    // b-phase @180 V Mini6PhaseIPMInverter
    // REAL a1 = 0.0; //1.34971502
    // REAL a2 = 16.0575341;  // yuefei tuning gives a2' = 9.2*2
    // REAL a3 = 17.59278688; // yuefei tuning gives a3' = a3*5

    // b-phase @80 V Mini6PhaseIPMInverter
    // REAL a1 = 0.0; //2.10871359 ;
    // REAL a2 = 7.36725304;
    // REAL a3 = 62.98712805;
    REAL a1 = 0.0; //(1.705,
    REAL a2 = sig_a2;
    REAL a3 = sig_a3;

#else
    /* SiC Cree 3 Phase Inverter*/

    // b-phase @20 V
    // REAL a1 = 0.0; //1.25991178;
    // REAL a2 = 2.30505904;
    // REAL a3 = 12.93721814;

    // b-phase @80 V
    // REAL a1 = 0.0; //1.28430164;
    // REAL a2 = 7.78857441;
    // REAL a3 = 6.20979077;
    REAL a1 = 0.0; // 1.6997103;
    REAL a2 = sig_a2;
    // 6.67159129;
    REAL a3 = sig_a3;
    // 8.42010418;

    // b-phase @180 V
    /*最早的拟合结果*/
    // REAL a1 = 0.0; //1.50469916;
    // REAL a2 = 13.48467604;
    // REAL a3 = 5.22150403;
    /*错误！多乘了一次beta-axis转phase系数！*/
    // REAL a1 = 0.0; //1.83573529
    // REAL a2 = sig_a2; 13.36317172;
    // REAL a3 = sig_a3; 5.81981571;
    /*纠正！100个点，平均分配*/
    // REAL a1 = 0.0; //1.8357354;
    // REAL a2 = sig_a2; 15.43046115;
    // REAL a3 = sig_a3; 5.04010863;

#endif

    return a1 * x + a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}
REAL sigmoid_online_v2(REAL x, REAL a2, REAL a3)
{
    return a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}
REAL sigmoid_online(REAL x, REAL Vsat, REAL a3)
{

    REAL a2 = Vsat * 2;
    // REAL a3 = 5.22150403;
    return a2 / (1.0 + exp(-a3 * x)) - a2 * 0.5;
}

/* 真查表法 // C代码参考ui_curve_v4.py */
// #define LUT_N_LC 70
// #define LUT_N_HC 29
// REAL lut_lc_voltage[70] = {0, -0.0105529, 0.31933, 0.364001, 0.415814, 0.489953, 0.602715, 0.769718, 0.971424, 1.21079, 1.50055, 1.83306, 2.16318, 2.54303, 2.92186, 3.24129, 3.51575, 3.75058, 3.97849, 4.16454, 4.33493, 4.49719, 4.64278, 4.76509, 4.88146, 4.99055, 5.06347, 5.16252, 5.24808, 5.30369, 5.36092, 5.44246, 5.50212, 5.5786, 5.63384, 5.69022, 5.74442, 5.79613, 5.8491, 5.89762, 5.93325, 5.98141, 6.01726, 6.06201, 6.09346, 6.13419, 6.16634, 6.19528, 6.2233, 6.25819, 6.29004, 6.31378, 6.34112, 6.3669, 6.38991, 6.4147, 6.4381, 6.46156, 6.48171, 6.49962, 6.51565, 6.53689, 6.5566, 6.57761, 6.59515, 6.60624, 6.62549, 6.64589, 6.65606, 6.67132};
// REAL lut_hc_voltage[29] = {6.69023, 6.80461, 6.89879, 6.96976, 7.02613, 7.08644, 7.12535, 7.17312, 7.20858, 7.2444, 7.27558, 7.30321, 7.32961, 7.35726, 7.38272, 7.39944, 7.42055, 7.43142, 7.4416, 7.43598, 7.44959, 7.45352, 7.45434, 7.45356, 7.45172, 7.45522, 7.45602, 7.44348, 7.43926};
// #define LUT_STEPSIZE_BIG 0.11641244037931034
// #define LUT_STEPSIZE_SMALL 0.01237159786376811
// #define LUT_STEPSIZE_BIG_INVERSE 8.59014721057018
// #define LUT_STEPSIZE_SMALL_INVERSE 80.83030268294078
// #define LUT_I_TURNING_LC 0.8660118504637677
// #define LUT_I_TURNING_HC 4.241972621463768
// #define V_PLATEAU 7.43925517763064


#define LUT_N_LC  70
#define LUT_N_HC  29
REAL lut_lc_voltage[70] = {0, 1.8746, 2.144, 2.34136, 2.49419, 2.60973, 2.74337, 2.88097, 3.00094, 3.10985, 3.23057, 3.34807, 3.4332, 3.50093, 3.55079, 3.59368, 3.61982, 3.64135, 3.65953, 3.67652, 3.69216, 3.7051, 3.71887, 3.73073, 3.73901, 3.75071, 3.75723, 3.76842, 3.77575, 3.78264, 3.78858, 3.79489, 3.80023, 3.80754, 3.8125, 3.81981, 3.82407, 3.8291, 3.83278, 3.83879, 3.84318, 3.84597, 3.85316, 3.85658, 3.85856, 3.86383, 3.86556, 3.86962, 3.87269, 3.87618, 3.87955, 3.88147, 3.88496, 3.88746, 3.88879, 3.89325, 3.89452, 3.8973, 3.89922, 3.9023, 3.90339, 3.90545, 3.90747, 3.90848, 3.91075, 3.91321, 3.915, 3.917, 3.91789, 3.91824};
REAL lut_hc_voltage[29] = {3.92009, 3.96505, 4.00084, 4.02508, 4.04373, 4.05894, 4.07484, 4.08679, 4.09461, 4.10467, 4.11101, 4.11741, 4.12254, 4.12758, 4.1317, 4.13569, 4.14004, 4.14258, 4.1448, 4.14871, 4.14937, 4.15295, 4.15194, 4.15295, 4.15107, 4.15136, 4.15206, 4.1492, 4.14973};
#define LUT_STEPSIZE_BIG 0.3796966241724138
#define LUT_STEPSIZE_SMALL 0.012110000369565214
#define LUT_STEPSIZE_BIG_INVERSE 2.633681566644419
#define LUT_STEPSIZE_SMALL_INVERSE 82.5763806344048
#define LUT_I_TURNING_LC 0.847700025869565
#define LUT_I_TURNING_HC 11.858902126869566
#define V_PLATEAU 4.149734790202423

REAL lookup_compensation_voltage_indexed(REAL current_value)
{
    REAL abs_current_value = fabsf(current_value);

    if (abs_current_value < LUT_I_TURNING_LC){
        REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
        int index = (int)float_index;
        REAL slope;
        if (index + 1 >= LUT_N_LC)
            slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
        else
            slope = (lut_lc_voltage[index + 1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
        return sign(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index * LUT_STEPSIZE_SMALL));
    }
    else{
        REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
        int index = (int)float_index; // THIS IS A RELATIVE INDEX!
        REAL slope;
        if (index + 1 >= LUT_N_HC)
            return sign(current_value) * V_PLATEAU;
        else
            slope = (lut_hc_voltage[index + 1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
        return sign(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index * LUT_STEPSIZE_BIG));
    }
}
void get_distorted_voltage_via_LUT_indexed(REAL ial, REAL ibe, REAL *ualbe_dist)
{

    /* 查表法 */
    if (TRUE){
        REAL ia, ib, ic;
        ia = 1 * (ial);
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

        REAL dist_ua = lookup_compensation_voltage_indexed(ia);
        REAL dist_ub = lookup_compensation_voltage_indexed(ib);
        REAL dist_uc = lookup_compensation_voltage_indexed(ic);

        G.ia = ia;
        G.dist_ua = dist_ua;

        // Clarke transformation（三分之二，0.5倍根号三）
        ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
        ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534
    }else{
        /* AB2U_AI 这宏假设了零序分量为零，即ia+ib+ic=0，但是电压并不一定满足吧，所以还是得用上面的？ */
        REAL ia, ib;            //,ic;
        ia = AB2U_AI(ial, ibe); // ia = 1 * (       ial                              );
        ib = AB2V_AI(ial, ibe); // ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );

        REAL dist_ua = lookup_compensation_voltage_indexed(ia);
        REAL dist_ub = lookup_compensation_voltage_indexed(ib);

        ualbe_dist[0] = UV2A_AI(dist_ua, dist_ub); // 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        ualbe_dist[1] = UV2B_AI(dist_ua, dist_ub); // 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc);
    }
}

/* 伪查表法（需要循环比大小，运算量不固定） */
REAL lookup_phase_current(REAL current, REAL *lut_voltage, REAL *lut_current, int length_of_lut)
{
    /* assume lut_voltage[0] is negative and lut_voltage[-1] is positive */
    int j;
    if (current < lut_current[0]){
        return lut_voltage[0];
    }else if (current > lut_current[length_of_lut - 1]){
        return lut_voltage[length_of_lut - 1];
    }else{
        for (j = 0; j < length_of_lut - 1; ++j){
            if (current > lut_current[j] && current < lut_current[j + 1]){
                REAL slope = (lut_voltage[j + 1] - lut_voltage[j]) / (lut_current[j + 1] - lut_current[j]);
                return (current - lut_current[j]) * slope + lut_voltage[j];
                // return 0.5*(lut_voltage[j]+lut_voltage[j+1]); // this is just wrong! stupid!
            }
        }
    }
#if PC_SIMULATION
    printf("DEBUG inverter.c. %g, %d, %d\n", current, j, length_of_lut);
#endif
    return 0.0;
}
REAL sigmoid_a3_tune = 1.0;
void get_distorted_voltage_via_CurveFitting(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist)
{
    // The data are measured in stator resistance identification with amplitude invariant d-axis current and d-axis voltage.

    /* 拟合法 */
    REAL ia, ib, ic;
    ia = 1 * (ial);
    ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
    ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

    // offline sigmoid
    REAL dist_ua = sigmoid(ia);
    REAL dist_ub = sigmoid(ib);
    REAL dist_uc = sigmoid(ic);

    // online tunable sigmoid (experimental)
    // REAL dist_ua = sigmoid_online(ia, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);
    // REAL dist_ub = sigmoid_online(ib, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);
    // REAL dist_uc = sigmoid_online(ic, INV.Vsat, 17.59278688 * sigmoid_a3_tune); //INV.theta_trapezoidal);

    // Clarke transformation（三分之二，0.5倍根号三）
    ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
    ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534

    // for plot
    INV.ia = ia;
    INV.ib = ib;
    INV.ic = ic;
    INV.dist_ua = dist_ua;
    INV.dist_ub = dist_ub;
    INV.dist_uc = dist_uc;
}
void get_distorted_voltage_via_LUT(REAL ual, REAL ube, REAL ial, REAL ibe, REAL *ualbe_dist, REAL *lut_voltage, REAL *lut_current, int length_of_lut)
{
    // The data are measured in stator resistance identification with amplitude invariant d-axis current and d-axis voltage.

    /* 查表法 */
    if (TRUE){
        REAL ia, ib, ic;
        ia = 1 * (ial);
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe);
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3 * ibe);

        REAL dist_ua = lookup_phase_current(ia, lut_voltage, lut_current, length_of_lut);
        REAL dist_ub = lookup_phase_current(ib, lut_voltage, lut_current, length_of_lut);
        REAL dist_uc = lookup_phase_current(ic, lut_voltage, lut_current, length_of_lut);

        // Clarke transformation（三分之二，0.5倍根号三）
        ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5 * dist_ub - 0.5 * dist_uc);
        ualbe_dist[1] = 0.66666667 * 0.8660254 * (dist_ub - dist_uc); // 0.5773502695534
    }else{
        /* AB2U_AI 这宏假设了零序分量为零，即ia+ib+ic=0，但是电压并不一定满足吧，所以还是得用上面的？ */
        REAL ia, ib;            //,ic;
        ia = AB2U_AI(ial, ibe); // ia = 1 * (       ial                              );
        ib = AB2V_AI(ial, ibe); // ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );
        // ic = AB2W_AI(ial, ibe); // ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3      * ibe );

        REAL dist_ua = lookup_phase_current(ia, lut_voltage, lut_current, length_of_lut);
        REAL dist_ub = lookup_phase_current(ib, lut_voltage, lut_current, length_of_lut);
        // REAL dist_uc = lookup_phase_current(ic, lut_voltage, lut_current, length_of_lut);

        // Clarke transformation（三分之二，0.5倍根号三）
        // ualbe_dist[0] = 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        // ualbe_dist[1] = 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc); // 0.5773502695534

        ualbe_dist[0] = UV2A_AI(dist_ua, dist_ub); // 0.66666667 * (dist_ua - 0.5*dist_ub - 0.5*dist_uc);
        ualbe_dist[1] = UV2B_AI(dist_ua, dist_ub); // 0.66666667 * 0.8660254 * ( dist_ub -     dist_uc);
    }
}

/* Chen 2021 Linear Approximation of U-I Curve */
REAL trapezoidal_voltage_by_phase_current(REAL current, REAL V_plateau, REAL I_plateau, REAL oneOnver_I_plateau)
{
    REAL abs_current = fabsf(current);
    if (abs_current < I_plateau)
    {
        return current * V_plateau * oneOnver_I_plateau;
    }
    else
    {
        return sign(current) * V_plateau;
    }
}

/* ParkSul2012 梯形波 */
void inverterNonlinearity_Initialization()
{
    INV.gamma_theta_trapezoidal = GAIN_THETA_TRAPEZOIDAL;
#ifdef _XCUBE1
    INV.Vsat = 16.0575341 / 2; // 6.67054; // 180 V SiC
#else
    // INV.Vsat = sig_a2*0.5; //6.74233802;
    INV.Vsat = 7.84; // 150 V
#endif

    INV.gain_Vsat = 0 * 10;

    INV.thetaA = 0;
    INV.cos_thetaA = 1;
    INV.sin_thetaA = 0;

    // --
    INV.u_comp[0] = 0;
    INV.u_comp[1] = 0;
    INV.u_comp[2] = 0;
    INV.ual_comp = 0;
    INV.ube_comp = 0;
    INV.uDcomp_atA = 0;
    INV.uQcomp_atA = 0;

    INV.iD_atA = 0;
    INV.iQ_atA = 0;
    INV.I5_plus_I7 = 0;
    INV.I5_plus_I7_LPF = 0.0;
    INV.theta_trapezoidal = 11.0 * M_PI_OVER_180; // in rad

#ifdef _XCUBE1
    INV.I_plateau_Max = 2.0;
    INV.I_plateau_Min = 0.2;
#else
    INV.I_plateau_Max = 1.0;
    INV.I_plateau_Min = 0.1;
#endif
    INV.I_plateau = 0.7;
    INV.V_plateau = 1.5 * sig_a2 * 0.5;
    INV.gamma_I_plateau = 10.0;
    INV.gamma_V_plateau = 0.0; // this is updated by the estimated disturbance to the sinusoidal flux model.

#if PC_SIMULATION
    INV.sig_a2 = 1.0 * sig_a2;
    INV.sig_a3 = 1.5 * sig_a3; // = shape parameter
#else
    INV.sig_a2 = sig_a2; // = Plateau * 2
    INV.sig_a3 = sig_a3; // = shape parameter
#endif

    INV.w6 = 1;
    INV.w12 = 0;
    INV.w18 = 0;
    INV.gamma_a2 = 400;
    INV.gamma_a3 = 125;
}
#define trapezoidal_voltage_by_current_vector_angle u_comp_per_phase
REAL u_comp_per_phase(REAL Vsat, REAL thetaA, REAL theta_trapezoidal, REAL oneOver_theta_trapezoidal)
{
    REAL compensation;

    // if(thetaA>0){
    //         compensation = Vsat;
    // }else{ // thetaA < 0
    //         compensation = -Vsat;
    // }

    if (thetaA > 0)
    {
        if (thetaA < theta_trapezoidal)
        {
            compensation = thetaA * Vsat * oneOver_theta_trapezoidal;
        }
        else if (thetaA > M_PI - theta_trapezoidal)
        {
            compensation = (M_PI - thetaA) * Vsat * oneOver_theta_trapezoidal;
        }
        else
        {
            compensation = Vsat;
        }
    }
    else
    { // thetaA < 0
        if (-thetaA < theta_trapezoidal)
        {
            compensation = -thetaA * -Vsat * oneOver_theta_trapezoidal;
        }
        else if (-thetaA > M_PI - theta_trapezoidal)
        {
            compensation = (M_PI + thetaA) * -Vsat * oneOver_theta_trapezoidal;
        }
        else
        {
            compensation = -Vsat;
        }
    }

    return compensation;
}
REAL lpf1_inverter(REAL x, REAL y_tminus1)
{
    // #define LPF1_RC 0.6 // 0.7, 0.8, 1.2, 3太大滤得过分了 /* 观察I5+I7_LPF的动态情况进行确定 */
    // #define ALPHA_RC (TS/(LPF1_RC + TS)) // TODO：优化
    // return y_tminus1 + ALPHA_RC * (x - y_tminus1); // 0.00020828993959591752
    return y_tminus1 + 0.00020828993959591752 * (x - y_tminus1);
}
REAL shift2pi(REAL thetaA)
{
    if (thetaA > M_PI)
    {
        return thetaA - 2 * M_PI;
    }
    else if (thetaA < -M_PI)
    {
        return thetaA + 2 * M_PI;
    }
    else
    {
        return thetaA;
    }
}
// REAL watch_theta_trapezoidal = 0.0;
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)

void Modified_ParkSul_Compensation(void)
{

    /* Park12/14
     * */
    // Phase A current's fundamental component transformation
    INV.thetaA = -M_PI * 1.5 + (*CTRL).i->theta_d_elec + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
    INV.thetaA = shift2pi(INV.thetaA); /* Q: how to handle it when INV.thetaA jumps between pi and -pi? */                            // 这句话绝对不能省去，否则C相的梯形波会出错。

    INV.cos_thetaA = cosf(INV.thetaA);
    INV.sin_thetaA = sinf(INV.thetaA);
    INV.iD_atA = AB2M(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);
    INV.iQ_atA = AB2T(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);

    INV.I5_plus_I7 = INV.iD_atA * sinf(6 * INV.thetaA);                     /* Q: Why sinf? Why not cosf? 和上面的-1.5*pi有关系吗？ */
    INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF); /* lpf1 for inverter */

    INV.I11_plus_I13 = INV.iD_atA * sinf(12 * INV.thetaA);
    INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

    INV.I17_plus_I19 = INV.iD_atA * sinf(18 * INV.thetaA);
    INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);

    // The adjusting of theta_t via 6th harmonic magnitude
    INV.theta_trapezoidal += CL_TS * INV.gamma_theta_trapezoidal // *fabsf((*CTRL).i->cmd_speed_rpm)
                             * (INV.I5_plus_I7_LPF + 0 * INV.I11_plus_I13_LPF + 0 * INV.I17_plus_I19_LPF);
    // 两种方法选其一：第一种可用于sensorless系统。
    if (FALSE)
    {
        /* 利用theta_t的饱和特性辨识Vsat（次优解） */
    }
    else
    {
        // 由于实际逆变器有一个调制比低引入5、7次谐波的问题，所以最佳theta_t不在11度附近了。
        if (INV.theta_trapezoidal >= 25 * M_PI_OVER_180)
        {                                               // 17
            INV.theta_trapezoidal = 25 * M_PI_OVER_180; // 17
        }
        if (INV.theta_trapezoidal <= 0.223 * M_PI_OVER_180)
        {
            INV.theta_trapezoidal = 0.223 * M_PI_OVER_180;
        }
    }

#if PC_SIMULATION == TRUE
    // 用偏小的Vsat，观察theta_t的收敛是否直达下界？
    // if((*CTRL).timebase>23){
    //     INV.Vsat = 9.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>20){
    //     INV.Vsat = 7.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>15){
    //     INV.Vsat = 6.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>10){
    //     INV.Vsat = 5.0/6.0 * 16.0575341/2;
    // }else if((*CTRL).timebase>5){
    //     INV.Vsat = 4.0/6.0 * 16.0575341/2;
    // }

    // /* Adaptive Vsat based on position error */
    // INV.Vsat += CL_TS * INV.gain_Vsat * sinf(ENC.theta_d_elec - ELECTRICAL_POSITION_FEEDBACK) * sign(ENC.omg_elec);
    // if (INV.Vsat>15){
    //     INV.Vsat = 15;
    // }else if(INV.Vsat<0){
    //     INV.Vsat = 0;
    // }
#else
    /* Adaptive Vsat based on position error */
    /* TO use this, you muast have a large enough stator current */
    /* TO use this, you muast have a large enough stator current */
    /* TO use this, you muast have a large enough stator current */
    INV.Vsat += CL_TS * INV.gain_Vsat * sinf(ENC.theta_d_elec - PMSM_ELECTRICAL_POSITION_FEEDBACK) * sign(ENC.varOmega);
    if (INV.Vsat > 15)
    {
        INV.Vsat = 15;
    }
    else if (INV.Vsat < 0)
    {
        INV.Vsat = 0;
    }
#endif

#if __INVERTER_NONLINEARITY == 3 // [ModelLUT]
    // INV.Vsat = 6.67054;
#elif __INVERTER_NONLINEARITY == 2 // [ModelSigmoid]
    // INV.Vsat = sigmoid(100)*1.0;
    // INV.Vsat = sigmoid(100)*0.95;
    // INV.Vsat = sigmoid(100)*1.05;
#elif __INVERTER_NONLINEARITY == 1 // [ModelSul96]
    REAL TM = _Toff - _Ton - _Tdead + _Tcomp; // Sul1996
    REAL Udist = (_Udc * TM * CL_TS_INVERSE - _Vce0 - _Vd0) / 6.0;
    INV.Vsat = 3 * fabsf(Udist); // 4 = 2*sign(ia) - sign(ib) - sign(ic) when ia is positive and ib/ic is negative
                                // but Vsat is phase voltage distortion maximum, so we need to add a zero sequence voltage of Udist*(sign(ia) + sign(ib) + sign(ic)),
                                // so, it is 3 * |Udist| as Vsat.
    static int bool_printed = FALSE;
    if (bool_printed == FALSE)
    {
        printf("\tVsat = %g V; Udist = %g.\n", INV.Vsat, Udist);
        bool_printed = TRUE;
    }
#endif

    REAL oneOver_theta_trapezoidal = 1.0 / INV.theta_trapezoidal;
    // watch_theta_trapezoidal = INV.theta_trapezoidal / M_PI_OVER_180;
    INV.u_comp[0] = u_comp_per_phase(INV.Vsat, INV.thetaA, INV.theta_trapezoidal, oneOver_theta_trapezoidal);
    INV.u_comp[1] = u_comp_per_phase(INV.Vsat, shift2pi(INV.thetaA - TWO_PI_OVER_3), INV.theta_trapezoidal, oneOver_theta_trapezoidal);
    INV.u_comp[2] = u_comp_per_phase(INV.Vsat, shift2pi(INV.thetaA - 2 * TWO_PI_OVER_3), INV.theta_trapezoidal, oneOver_theta_trapezoidal);

    // REAL ia,ib,ic;
    // ia = 1 * (       IS_C(0)                                  );
    // ib = 1 * (-0.5 * IS_C(0) - SIN_DASH_2PI_SLASH_3 * IS_C(1) );
    // ic = 1 * (-0.5 * IS_C(0) - SIN_2PI_SLASH_3      * IS_C(1) );
    // #define MISMATCH_A3 1.1
    // INV.u_comp[0] = sigmoid_online(ia, INV.Vsat, 5.22150403 * MISMATCH_A3); //INV.theta_trapezoidal);
    // INV.u_comp[1] = sigmoid_online(ib, INV.Vsat, 5.22150403 * MISMATCH_A3); //INV.theta_trapezoidal);
    // INV.u_comp[2] = sigmoid_online(ic, INV.Vsat, 5.22150403 * MISMATCH_A3); //INV.theta_trapezoidal);

    /* 相补偿电压Clarke为静止正交坐标系电压。 */
    // 改成恒幅值变换
    INV.ual_comp = 0.66666666667 * (INV.u_comp[0] - 0.5 * INV.u_comp[1] - 0.5 * INV.u_comp[2]);
    INV.ube_comp = 0.66666666667 * 0.86602540378 * (INV.u_comp[1] - INV.u_comp[2]);
    // INV.ual_comp = SQRT_2_SLASH_3      * (INV.u_comp[0] - 0.5*INV.u_comp[1] - 0.5*INV.u_comp[2]); // sqrt(2/3.)
    // INV.ube_comp = 0.70710678118654746 * (                    INV.u_comp[1] -     INV.u_comp[2]); // sqrt(2/3.)*sinf(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)

    // 区分补偿前的电压和补偿后的电压：
    // (*CTRL).ual, (*CTRL).ube 是补偿前的电压！
    // (*CTRL).ual + INV.ual_comp, (*CTRL).ube + INV.ube_comp 是补偿后的电压！

    // for plotting
    INV.uDcomp_atA = AB2M(INV.ual_comp, INV.ube_comp, INV.cos_thetaA, INV.sin_thetaA);
    INV.uQcomp_atA = AB2T(INV.ual_comp, INV.ube_comp, INV.cos_thetaA, INV.sin_thetaA);
}

void Online_PAA_Based_Compensation(void)
{

    // /* 角度反馈: (*CTRL).i->theta_d_elec or ELECTRICAL_POSITION_FEEDBACK? */
    // INV.thetaA = ELECTRICAL_POSITION_FEEDBACK;
    // // I (current vector amplitude)
    // INV.iD_atA = sqrt(IS_C(0)*IS_C(0) + IS_C(1)*IS_C(1));

    /* 在Park2012中，a相电流被建模成了sin函数，一个sin函数的自变量角度放在正交坐标系下看就是在交轴上的，所以要-1.5*pi */

    // Phase A current's fundamental component transformation
    /* 这里使用哪个角度的关键不在于是有感的角度还是无感的角度，而是你FOC电流控制器（Park变换）用的角度是哪个？ */
    if ((*debug).SENSORLESS_CONTROL)
    {
        INV.thetaA = -M_PI * 1.5 + PMSM_ELECTRICAL_POSITION_FEEDBACK + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
    }
    else
    {
        INV.thetaA = -M_PI * 1.5 + (*CTRL).i->theta_d_elec + atan2((*CTRL).i->cmd_iDQ[1], (*CTRL).i->cmd_iDQ[0]); /* Q: why -pi*(1.5)? */ /* ParkSul2014 suggests to use PLL to extract thetaA from current command */
    }
    INV.thetaA = shift2pi(INV.thetaA); /* Q: how to handle it when INV.thetaA jumps between pi and -pi? */ // 这句话绝对不能省去，否则C相的梯形波会出错。

    INV.cos_thetaA = cosf(INV.thetaA);
    INV.sin_thetaA = sinf(INV.thetaA);
    INV.iD_atA = AB2M(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);
    INV.iQ_atA = AB2T(IS_C(0), IS_C(1), INV.cos_thetaA, INV.sin_thetaA);

    if (FALSE)
    {
        /* Use q-axis current in phase A angle (Tentative and Failed) */
        INV.I5_plus_I7 = INV.iQ_atA * cosf(6 * INV.thetaA);
        INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF);

        INV.I11_plus_I13 = INV.iQ_atA * cosf(12 * INV.thetaA);
        INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

        INV.I17_plus_I19 = INV.iQ_atA * cosf(18 * INV.thetaA);
        INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);
    }
    else
    {
        /* Use d-axis current in phase A angle */
        INV.I5_plus_I7 = INV.iD_atA * sinf(6 * INV.thetaA);                     /* Q: Why sinf? Why not cosf? 和上面的-1.5*pi有关系吗？ */
        INV.I5_plus_I7_LPF = lpf1_inverter(INV.I5_plus_I7, INV.I5_plus_I7_LPF); /* lpf1 for inverter */

        INV.I11_plus_I13 = INV.iD_atA * sinf(12 * INV.thetaA);
        INV.I11_plus_I13_LPF = lpf1_inverter(INV.I11_plus_I13, INV.I11_plus_I13_LPF);

        INV.I17_plus_I19 = INV.iD_atA * sinf(18 * INV.thetaA);
        INV.I17_plus_I19_LPF = lpf1_inverter(INV.I17_plus_I19, INV.I17_plus_I19_LPF);
    }

#if PC_SIMULATION
    // INV.gamma_a2 = 0.0;
    // INV.gamma_a3 = 0.0;
#endif

    /* Online Update Sigmoid a3 */
    // if((*CTRL).timebase>35){
    //     INV.gamma_I_plateau = 0.0;
    // }
    INV.sig_a3 -= CL_TS * INV.gamma_a3 // *fabsf((*CTRL).i->cmd_speed_rpm)
                  * (INV.w6 * INV.I5_plus_I7_LPF + INV.w12 * INV.I11_plus_I13_LPF + INV.w18 * INV.I17_plus_I19_LPF);

    // (*CTRL).s->Motor_or_Generator = sign((*CTRL).i->omg_elec * (*CTRL).i->cmd_iDQ[1]);
    // (*CTRL).s->Motor_or_Generator = sign((*CTRL).i->cmd_omg_elec);

    /* Online Update Sigmoid a2 */
    if ((*CTRL).timebase > 2)
    {
        /* Sensorless: Adaptive a2 based on flux amplitude error */
        // use linear FE
        // INV.sig_a2 += CL_TS * -100 * AFEOE.output_error_dq[0];

        // use nonlinear (saturation) FE
        INV.sig_a2 += CL_TS * -INV.gamma_a2 * (*CTRL).s->Motor_or_Generator * (MOTOR.KActive - FE.htz.psi_2_ampl_lpf);

        /* Sensored: Adaptive a2 based on position error */
        /* To use this, you must have a large enough stator current */
        /* 这个好像只对梯形波（电流值的函数）有效 ……*/
        // INV.sig_a2 += CL_TS * INV.gain_Vsat * sinf(ENC.theta_d_elec - ELECTRICAL_POSITION_FEEDBACK) * (*CTRL).s->Motor_or_Generator;
    }
    if (INV.sig_a2 > 40)
    {
        INV.sig_a2 = 40;
    }
    else if (INV.sig_a2 < 2)
    {
        INV.sig_a2 = 2;
    }

    /* Chen2021: linear approximation of u-i curve */
    // if((*CTRL).timebase>35){
    //     INV.gamma_I_plateau = 0.0;
    // }
    // INV.I_plateau += CL_TS * 0 * INV.gamma_I_plateau \
    //                         // *fabsf((*CTRL).i->cmd_speed_rpm)
    //                         *(    1*INV.I5_plus_I7_LPF
    //                             + 0*INV.I11_plus_I13_LPF
    //                             + 0*INV.I17_plus_I19_LPF
    //                          );
    // if(INV.I_plateau > INV.I_plateau_Max){
    //     INV.I_plateau = INV.I_plateau_Max;
    // }else if(INV.I_plateau < INV.I_plateau_Min){
    //     INV.I_plateau = INV.I_plateau_Min;
    // }

    /* Chen2021SlessInv 覆盖 */
    if (INV.gamma_I_plateau != 0)
    {
        REAL ia_cmd = ((*CTRL).o->cmd_iAB[0]);
        REAL ib_cmd = (-0.5 * (*CTRL).o->cmd_iAB[0] - SIN_DASH_2PI_SLASH_3 * (*CTRL).o->cmd_iAB[1]);
        REAL ic_cmd = (-0.5 * (*CTRL).o->cmd_iAB[0] - SIN_2PI_SLASH_3 * (*CTRL).o->cmd_iAB[1]);
        REAL oneOver_I_plateau = 1.0 / INV.I_plateau;
        INV.u_comp[0] = trapezoidal_voltage_by_phase_current(ia_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);
        INV.u_comp[1] = trapezoidal_voltage_by_phase_current(ib_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);
        INV.u_comp[2] = trapezoidal_voltage_by_phase_current(ic_cmd, INV.V_plateau, INV.I_plateau, oneOver_I_plateau);

        /* Online Sigmoid a3 覆盖 覆盖 */
        INV.u_comp[0] = sigmoid_online_v2(ia_cmd, INV.sig_a2, INV.sig_a3);
        INV.u_comp[1] = sigmoid_online_v2(ib_cmd, INV.sig_a2, INV.sig_a3);
        INV.u_comp[2] = sigmoid_online_v2(ic_cmd, INV.sig_a2, INV.sig_a3);
    }

    /* 相补偿电压Clarke为静止正交坐标系电压。 */
    // 改成恒幅值变换
    INV.ual_comp = 0.66666666667 * (INV.u_comp[0] - 0.5 * INV.u_comp[1] - 0.5 * INV.u_comp[2]);
    INV.ube_comp = 0.66666666667 * 0.86602540378 * (INV.u_comp[1] - INV.u_comp[2]);
    // INV.ual_comp = SQRT_2_SLASH_3      * (INV.u_comp[0] - 0.5*INV.u_comp[1] - 0.5*INV.u_comp[2]); // sqrt(2/3.)
    // INV.ube_comp = 0.70710678118654746 * (                    INV.u_comp[1] -     INV.u_comp[2]); // sqrt(2/3.)*sinf(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)

    // 区分补偿前的电压和补偿后的电压：
    // (*CTRL).ual, (*CTRL).ube 是补偿前的电压！
    // (*CTRL).ual + INV.ual_comp, (*CTRL).ube + INV.ube_comp 是补偿后的电压！
}


// /* --------------------------下面的是从 pmsm_controller.c 挪过来的公用逆变器死区电压辨识&补偿代码 */
//     #define LUT_N_LC  70
//     #define LUT_N_HC  29
//     REAL lut_lc_voltage[70] = {0, -0.0105529, 0.31933, 0.364001, 0.415814, 0.489953, 0.602715, 0.769718, 0.971424, 1.21079, 1.50055, 1.83306, 2.16318, 2.54303, 2.92186, 3.24129, 3.51575, 3.75058, 3.97849, 4.16454, 4.33493, 4.49719, 4.64278, 4.76509, 4.88146, 4.99055, 5.06347, 5.16252, 5.24808, 5.30369, 5.36092, 5.44246, 5.50212, 5.5786, 5.63384, 5.69022, 5.74442, 5.79613, 5.8491, 5.89762, 5.93325, 5.98141, 6.01726, 6.06201, 6.09346, 6.13419, 6.16634, 6.19528, 6.2233, 6.25819, 6.29004, 6.31378, 6.34112, 6.3669, 6.38991, 6.4147, 6.4381, 6.46156, 6.48171, 6.49962, 6.51565, 6.53689, 6.5566, 6.57761, 6.59515, 6.60624, 6.62549, 6.64589, 6.65606, 6.67132};
//     REAL lut_hc_voltage[29] = {6.69023, 6.80461, 6.89879, 6.96976, 7.02613, 7.08644, 7.12535, 7.17312, 7.20858, 7.2444, 7.27558, 7.30321, 7.32961, 7.35726, 7.38272, 7.39944, 7.42055, 7.43142, 7.4416, 7.43598, 7.44959, 7.45352, 7.45434, 7.45356, 7.45172, 7.45522, 7.45602, 7.44348, 7.43926};
//     #define LUT_STEPSIZE_BIG 0.11641244037931034
//     #define LUT_STEPSIZE_SMALL 0.01237159786376811
//     #define LUT_STEPSIZE_BIG_INVERSE 8.59014721057018
//     #define LUT_STEPSIZE_SMALL_INVERSE 80.83030268294078
//     #define LUT_I_TURNING_LC 0.8660118504637677
//     #define LUT_I_TURNING_HC 4.241972621463768
//     #define V_PLATEAU 7.43925517763064
// REAL lookup_compensation_voltage_indexed(REAL current_value){
//     REAL abs_current_value = fabsf(current_value);

//     if(abs_current_value < LUT_I_TURNING_LC){
//         REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
//         int index = (int)float_index;
//         REAL slope;
//         if(index+1 >= LUT_N_LC)
//             slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         else
//             slope = (lut_lc_voltage[index+1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         return sign(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index*LUT_STEPSIZE_SMALL));
//     }else{
//         REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
//         int index = (int)float_index; // THIS IS A RELATIVE INDEX!
//         REAL slope;
//         if(index+1 >= LUT_N_HC)
//             return V_PLATEAU;
//         else
//             slope = (lut_hc_voltage[index+1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
//         return sign(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index*LUT_STEPSIZE_BIG));
//     }
// }
// int test_lookup_compensation_voltage_indexed(){
//     int i=0;
//     while(TRUE){
//         i+=1;
//         printf("%g, %g\n", lookup_compensation_voltage_indexed(0.02*i), 0.02*i);
//         if(i>100)
//             break;
//     }
//     return 0;

// }
#endif




// /* --------------------------下面的是从 pmsm_controller.c 挪过来的公用逆变器死区电压辨识&补偿代码 */
//     #define LUT_N_LC  70
//     #define LUT_N_HC  29
//     REAL lut_lc_voltage[70] = {0, -0.0105529, 0.31933, 0.364001, 0.415814, 0.489953, 0.602715, 0.769718, 0.971424, 1.21079, 1.50055, 1.83306, 2.16318, 2.54303, 2.92186, 3.24129, 3.51575, 3.75058, 3.97849, 4.16454, 4.33493, 4.49719, 4.64278, 4.76509, 4.88146, 4.99055, 5.06347, 5.16252, 5.24808, 5.30369, 5.36092, 5.44246, 5.50212, 5.5786, 5.63384, 5.69022, 5.74442, 5.79613, 5.8491, 5.89762, 5.93325, 5.98141, 6.01726, 6.06201, 6.09346, 6.13419, 6.16634, 6.19528, 6.2233, 6.25819, 6.29004, 6.31378, 6.34112, 6.3669, 6.38991, 6.4147, 6.4381, 6.46156, 6.48171, 6.49962, 6.51565, 6.53689, 6.5566, 6.57761, 6.59515, 6.60624, 6.62549, 6.64589, 6.65606, 6.67132};
//     REAL lut_hc_voltage[29] = {6.69023, 6.80461, 6.89879, 6.96976, 7.02613, 7.08644, 7.12535, 7.17312, 7.20858, 7.2444, 7.27558, 7.30321, 7.32961, 7.35726, 7.38272, 7.39944, 7.42055, 7.43142, 7.4416, 7.43598, 7.44959, 7.45352, 7.45434, 7.45356, 7.45172, 7.45522, 7.45602, 7.44348, 7.43926};
//     #define LUT_STEPSIZE_BIG 0.11641244037931034
//     #define LUT_STEPSIZE_SMALL 0.01237159786376811
//     #define LUT_STEPSIZE_BIG_INVERSE 8.59014721057018
//     #define LUT_STEPSIZE_SMALL_INVERSE 80.83030268294078
//     #define LUT_I_TURNING_LC 0.8660118504637677
//     #define LUT_I_TURNING_HC 4.241972621463768
//     #define V_PLATEAU 7.43925517763064
// REAL lookup_compensation_voltage_indexed(REAL current_value){
//     REAL abs_current_value = fabsf(current_value);

//     if(abs_current_value < LUT_I_TURNING_LC){
//         REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
//         int index = (int)float_index;
//         REAL slope;
//         if(index+1 >= LUT_N_LC)
//             slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         else
//             slope = (lut_lc_voltage[index+1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         return sign(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index*LUT_STEPSIZE_SMALL));
//     }else{
//         REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
//         int index = (int)float_index; // THIS IS A RELATIVE INDEX!
//         REAL slope;
//         if(index+1 >= LUT_N_HC)
//             return V_PLATEAU;
//         else
//             slope = (lut_hc_voltage[index+1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
//         return sign(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index*LUT_STEPSIZE_BIG));
//     }
// }
// int test_lookup_compensation_voltage_indexed(){
//     int i=0;
//     while(TRUE){
//         i+=1;
//         printf("%g, %g\n", lookup_compensation_voltage_indexed(0.02*i), 0.02*i);
//         if(i>100)
//             break;
//     }
//     return 0;


/* MAIN */
void _main_inverter_voltage_command(int bool_use_cmd_iAB){
    REAL Ia, Ib;

    /* We use cmd_iAB instead of iAB to look-up */
    if (bool_use_cmd_iAB)
    {
        Ia = (*CTRL).o->cmd_iAB[0];
        Ib = (*CTRL).o->cmd_iAB[1];
    }
    else
    {
        Ia = (*CTRL).i->iAB[0];
        Ib = (*CTRL).i->iAB[1];
    }

    G.FLAG_INVERTER_NONLINEARITY_COMPENSATION = 0;
    if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 0){
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1];

        /* For scope only */
        #if PC_SIMULATION
            REAL ualbe_dist[2];
            get_distorted_voltage_via_CurveFitting((*CTRL).o->cmd_uAB[0], (*CTRL).o->cmd_uAB[1], Ia, Ib, ualbe_dist);
            INV.ual_comp = ualbe_dist[0];
            INV.ube_comp = ualbe_dist[1];
        #endif
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 4)
    {
        REAL ualbe_dist[2];
        get_distorted_voltage_via_LUT_indexed(Ia, Ib, ualbe_dist);
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + ualbe_dist[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + ualbe_dist[1];

        /* For scope only */
        INV.ual_comp = ualbe_dist[0];
        INV.ube_comp = ualbe_dist[1];
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 3)
    {
    /* 查表法-补偿 */
    // // Measured in Simulation when the inverter is modelled according to the experimental measurements
    // #define LENGTH_OF_LUT  19
    // REAL lut_current_ctrl[LENGTH_OF_LUT] = {-3.78, -3.36, -2.94, -2.52, -2.1, -1.68, -1.26, -0.839998, -0.419998, -0, 0.420002, 0.840002, 1.26, 1.68, 2.1, 2.52, 2.94, 3.36, 3.78};
    // // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-6.41808, -6.41942, -6.39433, -6.36032, -6.25784, -6.12639, -5.79563, -5.35301, -3.61951, 0, 3.5038, 5.24969, 5.73176, 6.11153, 6.24738, 6.35941, 6.43225, 6.39274, 6.39482};
    // #define MANUAL_CORRECTION 1.1 // [V]
    // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {
    //     -6.41808+MANUAL_CORRECTION,
    //     -6.41942+MANUAL_CORRECTION,
    //     -6.39433+MANUAL_CORRECTION,
    //     -6.36032+MANUAL_CORRECTION,
    //     -6.25784+MANUAL_CORRECTION,
    //     -6.12639+MANUAL_CORRECTION,
    //     -5.79563+MANUAL_CORRECTION,
    //     -5.35301+MANUAL_CORRECTION,
    //     -3.61951, 0, 3.5038,
    //      5.24969-MANUAL_CORRECTION,
    //      5.73176-MANUAL_CORRECTION,
    //      6.11153-MANUAL_CORRECTION,
    //      6.24738-MANUAL_CORRECTION,
    //      6.35941-MANUAL_CORRECTION,
    //      6.43225-MANUAL_CORRECTION,
    //      6.39274-MANUAL_CORRECTION,
    //      6.39482-MANUAL_CORRECTION};

    // #define LENGTH_OF_LUT  100
    // REAL lut_current_ctrl[LENGTH_OF_LUT] = {-4.116, -4.032, -3.948, -3.864, -3.78, -3.696, -3.612, -3.528, -3.444, -3.36, -3.276, -3.192, -3.108, -3.024, -2.94, -2.856, -2.772, -2.688, -2.604, -2.52, -2.436, -2.352, -2.268, -2.184, -2.1, -2.016, -1.932, -1.848, -1.764, -1.68, -1.596, -1.512, -1.428, -1.344, -1.26, -1.176, -1.092, -1.008, -0.923998, -0.839998, -0.755998, -0.671998, -0.587998, -0.503998, -0.419998, -0.335999, -0.251999, -0.168, -0.084, -0, 0.084, 0.168, 0.252001, 0.336001, 0.420002, 0.504002, 0.588002, 0.672002, 0.756002, 0.840002, 0.924002, 1.008, 1.092, 1.176, 1.26, 1.344, 1.428, 1.512, 1.596, 1.68, 1.764, 1.848, 1.932, 2.016, 2.1, 2.184, 2.268, 2.352, 2.436, 2.52, 2.604, 2.688, 2.772, 2.856, 2.94, 3.024, 3.108, 3.192, 3.276, 3.36, 3.444, 3.528, 3.612, 3.696, 3.78, 3.864, 3.948, 4.032, 4.116, 4.2};
    // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-6.48905, -6.49021, -6.49137, -6.49253, -6.49368, -6.49227, -6.49086, -6.48945, -6.48803, -6.48662, -6.47993, -6.47323, -6.46653, -6.45983, -6.45313, -6.44465, -6.43617, -6.42769, -6.41921, -6.41072, -6.38854, -6.36637, -6.34419, -6.32202, -6.29984, -6.27187, -6.2439, -6.21593, -6.18796, -6.15999, -6.09216, -6.02433, -5.9565, -5.88867, -5.82083, -5.73067, -5.6405, -5.55033, -5.46016, -5.36981, -5.02143, -4.67305, -4.32467, -3.97629, -3.62791, -2.90251, -2.17689, -1.45126, -0.725632, -1e-06, 0.702441, 1.40488, 2.10732, 2.80976, 3.5122, 3.86321, 4.21409, 4.56497, 4.91585, 5.26649, 5.36459, 5.46268, 5.56078, 5.65887, 5.75696, 5.8346, 5.91224, 5.98987, 6.0675, 6.14513, 6.17402, 6.20286, 6.2317, 6.26054, 6.28938, 6.31347, 6.33755, 6.36164, 6.38572, 6.40981, 6.4303, 6.4508, 6.47129, 6.49178, 6.49105, 6.48483, 6.4786, 6.47238, 6.46616, 6.45994, 6.46204, 6.46413, 6.46623, 6.46832, 6.47042, 6.47202, 6.47363, 6.47524, 6.47684, 6.47843};

    // Experimental measurements
    // #define LENGTH_OF_LUT 21
    // REAL lut_current_ctrl[LENGTH_OF_LUT] = {-4.19999, -3.77999, -3.36001, -2.94002, -2.51999, -2.10004, -1.68004, -1.26002, -0.840052, -0.419948, 5.88754e-06, 0.420032, 0.839998, 1.26003, 1.67998, 2.10009, 2.51996, 2.87326, 3.36001, 3.78002, 4.2};
    // REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-5.20719, -5.2079, -5.18934, -5.15954, -5.11637, -5.04723, -4.93463, -4.76367, -4.42522, -3.46825, 0.317444, 3.75588, 4.55737, 4.87773, 5.04459, 5.15468, 5.22904, 5.33942, 5.25929, 5.28171, 5.30045};

    // Experimental measurements 03-18-2021
    #define LENGTH_OF_LUT 41
        REAL lut_current_ctrl[LENGTH_OF_LUT] = {-4.20001, -3.98998, -3.78002, -3.57, -3.36002, -3.14999, -2.93996, -2.72993, -2.51998, -2.31003, -2.1, -1.88996, -1.67999, -1.46998, -1.25999, -1.05001, -0.839962, -0.62995, -0.420046, -0.210048, 1.39409e-05, 0.209888, 0.420001, 0.629998, 0.840008, 1.05002, 1.25999, 1.47002, 1.68001, 1.89001, 2.10002, 2.31, 2.51999, 2.73004, 2.94, 3.14996, 3.35995, 3.57001, 3.77999, 3.98998, 4.2};
        REAL lut_voltage_ctrl[LENGTH_OF_LUT] = {-5.75434, -5.74721, -5.72803, -5.70736, -5.68605, -5.66224, -5.63274, -5.59982, -5.56391, -5.52287, -5.47247, -5.40911, -5.33464, -5.25019, -5.14551, -5.00196, -4.80021, -4.48369, -3.90965, -2.47845, -0.382101, 2.02274, 3.7011, 4.35633, 4.71427, 4.94376, 5.10356, 5.22256, 5.31722, 5.39868, 5.46753, 5.5286, 5.57507, 5.62385, 5.66235, 5.70198, 5.73617, 5.76636, 5.79075, 5.81737, 5.83632};

        REAL ualbe_dist[2];
        get_distorted_voltage_via_LUT((*CTRL).o->cmd_uAB[0], (*CTRL).o->cmd_uAB[1], Ia, Ib, ualbe_dist, lut_voltage_ctrl, lut_current_ctrl, LENGTH_OF_LUT);
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + ualbe_dist[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + ualbe_dist[1];

        /* For scope only */
        INV.ual_comp = ualbe_dist[0];
        INV.ube_comp = ualbe_dist[1];
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 2)
    {
        /* 拟合法-补偿 */
        REAL ualbe_dist[2] = {0.0, 0.0};
        get_distorted_voltage_via_CurveFitting((*CTRL).o->cmd_uAB[0], (*CTRL).o->cmd_uAB[1], Ia, Ib, ualbe_dist);
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + ualbe_dist[0];
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + ualbe_dist[1];

        /* For scope only */
        INV.ual_comp = ualbe_dist[0];
        INV.ube_comp = ualbe_dist[1];
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 1)
    {   
        #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        /* 梯形波自适应 */
        Modified_ParkSul_Compensation();
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + INV.ual_comp;
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + INV.ube_comp;

        #endif
    }
    else if (G.FLAG_INVERTER_NONLINEARITY_COMPENSATION == 5)
    {   
        #if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
        /* Sigmoid自适应 */
        Online_PAA_Based_Compensation();
        (*CTRL).o->cmd_uAB_to_inverter[0] = (*CTRL).o->cmd_uAB[0] + INV.ual_comp;
        (*CTRL).o->cmd_uAB_to_inverter[1] = (*CTRL).o->cmd_uAB[1] + INV.ube_comp;

        #endif
    }
}













// 定义特定的测试指令，如快速反转等
#if 0 // defined in Experiment.c???
    struct SweepFreq sf={0.0, 1, SWEEP_FREQ_INIT_FREQ-1, 0.0, 0.0};

    void cmd_fast_speed_reversal(REAL timebase, REAL instant, REAL interval, REAL rpm_cmd){
        if(timebase > instant+2*interval){
            (*debug).set_rpm_speed_command = 1*1500 + rpm_cmd;
        }else if(timebase > instant+interval){
            (*debug).set_rpm_speed_command = 1*1500 + -rpm_cmd;
        }else if(timebase > instant){
            (*debug).set_rpm_speed_command = 1*1500 + rpm_cmd;
        }else{
            (*debug).set_rpm_speed_command = 20; // default initial command
        }
    }

    REAL short_stopping_at_zero_speed(){
        static REAL set_rpm_speed_command=0.0;

        #define RPM1 100
        #define BIAS 0
        if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = 0;
        }else if((*CTRL).timebase<5){
            set_rpm_speed_command = RPM1;
        }else if((*CTRL).timebase<10){
            set_rpm_speed_command += CL_TS * -50;
            if(set_rpm_speed_command < 0){
                set_rpm_speed_command = 0.0;
            }
        }else if((*CTRL).timebase<15){
            set_rpm_speed_command += CL_TS * -50;
            if(set_rpm_speed_command<-RPM1){
                set_rpm_speed_command = -RPM1;
            }
        }else if((*CTRL).timebase<20){
            set_rpm_speed_command += CL_TS * +50;
            if(set_rpm_speed_command > 0){
                set_rpm_speed_command = 0.0;
            }
        }else if((*CTRL).timebase<25){
            set_rpm_speed_command += CL_TS * +50;
            if(set_rpm_speed_command>RPM1){
                set_rpm_speed_command = RPM1;
            }
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }

    REAL slow_speed_reversal(REAL slope){ // slope = 20 rpm/s, 50 rpm/s
        static REAL set_rpm_speed_command=0.0;

        #define RPM1 100
        #define BIAS 0
        if((*CTRL).timebase<0.5){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = 0;
        }else if((*CTRL).timebase<1){
            set_rpm_speed_command = -50;
        }else if((*CTRL).timebase<4){
            set_rpm_speed_command = RPM1+50;
        }else if((*CTRL).timebase<15){
            set_rpm_speed_command += CL_TS * -slope;
            if(set_rpm_speed_command<-RPM1){
                set_rpm_speed_command = -RPM1;
            }
        }else if((*CTRL).timebase<25){
            set_rpm_speed_command += CL_TS * +slope;
            if(set_rpm_speed_command>RPM1){
                set_rpm_speed_command = RPM1;
            }
        }

        if((*CTRL).timebase>25 && (*CTRL).timebase<35){
            set_rpm_speed_command = RPM1*2;
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }

    REAL low_speed_operation(){
        REAL set_rpm_speed_command;

        #define RPM1 200
        #define BIAS 50
        if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = RPM1;
        }else if((*CTRL).timebase<3){
            set_rpm_speed_command = RPM1 + BIAS;
        }else if((*CTRL).timebase<6+BIAS){
            set_rpm_speed_command = -RPM1;
        }else if((*CTRL).timebase<9+BIAS){
            set_rpm_speed_command = 10;
        }else if((*CTRL).timebase<12+BIAS){
            set_rpm_speed_command = RPM1*sin(20*2*M_PI*(*CTRL).timebase);
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }

    REAL high_speed_operation(){
        REAL set_rpm_speed_command;

        #define RPM1 1500
        #define BIAS 0
        if((*CTRL).timebase<1){ // note 1 sec is not enough for stator flux to reach steady state.
            set_rpm_speed_command = 0;
        }else if((*CTRL).timebase<4){
            set_rpm_speed_command = RPM1;
        }else if((*CTRL).timebase<6+BIAS){
            set_rpm_speed_command = -RPM1;
        }else if((*CTRL).timebase<9+BIAS){
            set_rpm_speed_command = 10;
        }else if((*CTRL).timebase<12+BIAS){
            set_rpm_speed_command = RPM1*sin(2*2*M_PI*(*CTRL).timebase);
        }

        return set_rpm_speed_command;
        #undef RPM1
        #undef BIAS
    }

#endif






























#include "ACMSim.h"

#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)

// #if PC_SIMULATION==FALSE
// REAL CpuTimer_Delta = 0;
// Uint32 CpuTimer_Before = 0;
// Uint32 CpuTimer_After = 0;
// #endif


#define IM ACM // 权宜之计

// 实用函数
REAL sat_kappa(REAL x){
    if(x>marino.kappa){
        return marino.kappa;
    }else if(x<-marino.kappa){
        return -marino.kappa;
    }else{
        return x;
    }
}
REAL deriv_sat_kappa(REAL x){
    if(x>marino.kappa){
        return 0;
    }else if(x<-marino.kappa){
        return -0;
    }else{
        return 1;
    }
}

// 控制器
int32 BOOL_FAST_REVERSAL = FALSE;
int32 CONSTANT_SPEED_OPERATION = 0;
int32 Jerk = 0;
// REAL Jerk_value = 1;
REAL acc = 0.0;
// REAL delta_speed = 800;
REAL imife_accerleration_fast = 1600;        //  /60.0*2*M_PI; // 6400 rpm/s
REAL jerk = 6400 / 0.002;                    //  /60.0*2*M_PI; // rpm/s / s
REAL SINE_AMPL = 100.0 ;                     //  /60.0*2*M_PI;
REAL imife_ramp_slope = 100;                 //  /60.0*2*M_PI; // 400; // 400; // rpm/s
REAL imife_accerleration_slow = 20;          //  /60.0*2*M_PI; // rpm/s
REAL Jerk_time = 0.002;
REAL accerleration_time_inv=8;
REAL imife_reversal_end_time = 5.0; // s
REAL marino_speed_freq = 4;

void controller_marino2005_with_commands(){

    /// 0. 参数时变
    // if (fabsf((*CTRL).timebase-2.0)<CL_TS){
    //     printf("[Runtime] Rotor resistance of the simulated IM has changed!\n");
    //     ACM.alpha = 0.5*INIT_RREQ / IM_MAGNETIZING_INDUCTANCE;
    //     ACM.rreq = ACM.alpha*ACM.Lmu;
    //     ACM.rr   = ACM.alpha*(ACM.Lm+ACM.Llr);
    // }


    /// 1. 生成转速指令
    static REAL rpm_speed_command=0.0, amp_current_command=0.0;
    {
        // commands(&rpm_speed_command, &amp_current_command);
        static REAL local_dc_rpm_cmd = 0.0;
        static REAL local_dc_rpm_cmd_deriv = 0.0;

        static REAL last_omg_cmd=0.0;
        REAL OMG1;
        #define DELAY 100
        #define OFF 1000
        if((*CTRL).timebase>6){/*Variable Speed: Sinusoidal Speed + Ramp Speed*/
            if      ((*CTRL).timebase>7.0){
                OMG1 = (2*M_PI*marino_speed_freq);
            }else if((*CTRL).timebase>6.875){
                OMG1 = (2*M_PI*32);
            }else if((*CTRL).timebase>6.75){
                OMG1 = (2*M_PI*16);
            }else if((*CTRL).timebase>6.5){
                OMG1 = (2*M_PI*8);
            }else{
                OMG1 = (2*M_PI*4);
            }
            //OMG1 = (2*M_PI*4);
            OMG1 = 0;

            // 反转(step speed)
            if(BOOL_FAST_REVERSAL){
                if((int)((*CTRL).timebase*accerleration_time_inv)%10==0){ //*8*400/750
                    local_dc_rpm_cmd_deriv = -imife_accerleration_fast;
                }else if((int)((*CTRL).timebase*accerleration_time_inv)%10==5){
                    local_dc_rpm_cmd_deriv = imife_accerleration_fast;
                }
                else 
                    local_dc_rpm_cmd_deriv = 0;

            }

            // 反转
            if(BOOL_FAST_REVERSAL==FALSE){
                if((*CTRL).timebase>10 && (*CTRL).timebase<15+imife_reversal_end_time){/*Reversal*/
                    // OMG1 = 0;
                    // TODO: test positive and negative high speed
                    local_dc_rpm_cmd_deriv = -imife_accerleration_slow;
                }else if((*CTRL).timebase>15+imife_reversal_end_time && (*CTRL).timebase<15.2+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv =  1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.2+imife_reversal_end_time && (*CTRL).timebase<15.4+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv = - 1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.4+imife_reversal_end_time && (*CTRL).timebase<15.6+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv =  1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.6+imife_reversal_end_time && (*CTRL).timebase<15.8+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv = - 1000; //imife_accerleration_fast;
                }else if((*CTRL).timebase>15.8+imife_reversal_end_time && (*CTRL).timebase<16.0+imife_reversal_end_time){
                    local_dc_rpm_cmd_deriv =  1000; //imife_accerleration_fast;
                }else{
                    local_dc_rpm_cmd_deriv = 0.0;
                }
            }

            // local_dc_rpm_cmd_deriv = 0.0;

            //Jerk Hugh
            // if(Jerk==1){
            //     if((int)((*CTRL).timebase*8)%10==0){
            //         if( (*CTRL).timebase-(int)((*CTRL).timebase)>=0 && (*CTRL).timebase-(int)((*CTRL).timebase)<=Jerk_time){ //0-0.125
            //             acc = ((*CTRL).timebase-(int)((*CTRL).timebase)) * jerk;
            //             acc = - acc;
            //         }
            //         else if((*CTRL).timebase-(int)((*CTRL).timebase)>=(0.125-Jerk_time) && (*CTRL).timebase-(int)((*CTRL).timebase)<=0.125){
            //             acc = imife_accerleration_fast-(((*CTRL).timebase-(int)((*CTRL).timebase))-(0.125-Jerk_time)) * jerk;
            //             acc = - acc;
            //         }
            //         else
            //         acc = -imife_accerleration_fast;
            //     }else if((int)((*CTRL).timebase*8)%10==5){
            //         if( (*CTRL).timebase-(int)((*CTRL).timebase)>=0 && (*CTRL).timebase-(int)((*CTRL).timebase)<=Jerk_time){ //0-0.125
            //             acc = ((*CTRL).timebase-(int)((*CTRL).timebase)) * jerk;
            //             acc = acc;
            //         }
            //         else if((*CTRL).timebase-(int)((*CTRL).timebase)>=(0.125-Jerk_time) && (*CTRL).timebase-(int)((*CTRL).timebase)<=0.125){
            //             acc = imife_accerleration_fast-(((*CTRL).timebase-(int)((*CTRL).timebase))-(0.125-Jerk_time)) * jerk;
            //             acc = acc;
            //         }
            //         else
            //         acc = imife_accerleration_fast;
            //     }
            //     else 
            //         acc = 0;
            // }
            // local_dc_rpm_cmd_deriv = acc; // jerk

            local_dc_rpm_cmd            += CL_TS * local_dc_rpm_cmd_deriv;

            rpm_speed_command              = (SINE_AMPL          * sin(OMG1*(*CTRL).timebase) + local_dc_rpm_cmd       );
            (*CTRL).i->cmd_varOmega        = (SINE_AMPL          * sin(OMG1*(*CTRL).timebase) + local_dc_rpm_cmd       )*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = (SINE_AMPL*OMG1     * cos(OMG1*(*CTRL).timebase) + local_dc_rpm_cmd_deriv )*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_dderiv_varOmega = (SINE_AMPL*OMG1*OMG1*-sin(OMG1*(*CTRL).timebase) + 0                      )*RPM_2_MECH_RAD_PER_SEC;
        }else if((*CTRL).timebase>5+OFF){/*Constant Speed*/
            rpm_speed_command           = local_dc_rpm_cmd;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = 0;
            (*CTRL).i->cmd_dderiv_varOmega = 0;
        }else if((*CTRL).timebase>3+OFF){/*Ramp Speed*/
            rpm_speed_command += CL_TS*150;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;

            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else if((*CTRL).timebase>2+OFF){/*Ramp Speed (Downward)*/
            rpm_speed_command -= CL_TS*150;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;

            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else if((*CTRL).timebase>1.5+OFF){/*Ramp Speed*/
            rpm_speed_command += CL_TS*150;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;

            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else if((*CTRL).timebase>1.5+0.0){ /*Constant Speed*/
            rpm_speed_command           = local_dc_rpm_cmd;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = 0;
            (*CTRL).i->cmd_dderiv_varOmega = 0;
        }else if((*CTRL).timebase>0.5+0.0){ /*Ramp Speed*/
            rpm_speed_command           += CL_TS*imife_ramp_slope;
            local_dc_rpm_cmd            = rpm_speed_command;
            (*CTRL).i->cmd_varOmega        = rpm_speed_command*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = ((*CTRL).i->cmd_varOmega - last_omg_cmd)*CL_TS_INVERSE;
            (*CTRL).i->cmd_dderiv_varOmega = 0;
            last_omg_cmd = (*CTRL).i->cmd_varOmega;
        }else{ /*Bulding Flux*/
            rpm_speed_command           = 0; // 0*20                   * sin(2*M_PI*(*CTRL).timebase);
            (*CTRL).i->cmd_varOmega        = 0; // 0*20                   * sin(2*M_PI*(*CTRL).timebase)*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_deriv_varOmega  = 0; // 0*20*(2*M_PI)          * cos(2*M_PI*(*CTRL).timebase)*RPM_2_MECH_RAD_PER_SEC;
            (*CTRL).i->cmd_dderiv_varOmega = 0; // 0*20*(2*M_PI)*(2*M_PI) *-sin(2*M_PI*(*CTRL).timebase)*RPM_2_MECH_RAD_PER_SEC;
        }
    }

    // // Overwrite speed command
    // if(CONSTANT_SPEED_OPERATION!=0){
    //     rpm_speed_command           = 0;
    //     (*CTRL).i->cmd_varOmega        = CONSTANT_SPEED_OPERATION*RPM_2_MECH_RAD_PER_SEC;
    //     (*CTRL).i->cmd_deriv_varOmega  = 0;
    //     (*CTRL).i->cmd_dderiv_varOmega = 0;
    // }

    /// 2. 生成磁链指令
    #define TIME_COST 0.1
    if((*CTRL).timebase<TIME_COST){
        (*CTRL).i->cmd_psi_raw   += CL_TS*(*CTRL).i->m0/TIME_COST;
        (*CTRL).i->cmd_psi        = (*CTRL).i->cmd_psi_raw;
        (*CTRL).i->cmd_deriv_psi  = (*CTRL).i->m0/TIME_COST;
        (*CTRL).i->cmd_dderiv_psi = 0.0;
    }else{
        // (*CTRL).i->m1 = 0.0;
        (*CTRL).i->cmd_psi_raw    = (*CTRL).i->m0 + (*CTRL).i->m1 * sin((*CTRL).i->omega1*(*CTRL).timebase);
        (*CTRL).i->cmd_psi        = (*CTRL).i->cmd_psi_raw; // _lpf((*CTRL).i->cmd_psi_raw, (*CTRL).i->cmd_psi, 5);
        (*CTRL).i->cmd_deriv_psi  = (*CTRL).i->m1 * (*CTRL).i->omega1 * cos((*CTRL).i->omega1*(*CTRL).timebase);
        (*CTRL).i->cmd_dderiv_psi = (*CTRL).i->m1 * (*CTRL).i->omega1 * (*CTRL).i->omega1 * -sin((*CTRL).i->omega1*(*CTRL).timebase);
    }
    (*CTRL).i->cmd_psi_inv = 1.0 / (*CTRL).i->cmd_psi;
    (*CTRL).i->cmd_psi_active[0] = MT2A((*CTRL).i->cmd_psi, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT); // TODO 这里的cosT和sinT还没更新到当前步，有没有关系？
    (*CTRL).i->cmd_psi_active[1] = MT2B((*CTRL).i->cmd_psi, 0.0, (*CTRL).s->cosT, (*CTRL).s->sinT);

    /// 调用具体的控制器
    // controller_IFOC();
    controller_marino2005();

    main_inverter_voltage_command(0);
}
extern REAL marino_sat_d_axis_flux_control;
extern REAL marino_sat_q_axis_flux_control;
void controller_marino2005(){

    // Cascaded from other system
    /* Flux Est. */
    flux_observer();

        // flux feedback
        if (FALSE){
            /*Simulation only flux*/
            // marino.psi_Dmu = simvm.psi_D2_ode1;
            // marino.psi_Qmu = simvm.psi_Q2_ode1;
            // // marino.psi_Dmu = simvm.psi_D2_ode1_v2;
            // // marino.psi_Qmu = simvm.psi_Q2_ode1_v2;

            /*A. Exact Compensation based on Waveform Top and Butt */
                // marino.psi_Dmu = exact.psi_DQ2[0];
                // marino.psi_Qmu = exact.psi_DQ2[1];
                // marino.psi_Dmu = AB2M(exact.psi_2[0], exact.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
                // marino.psi_Qmu = AB2T(exact.psi_2[0], exact.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
                // #define COMPENSATION 0
                // marino.psi_Dmu = AB2M(exact.psi_2[0]-COMPENSATION*exact.filtered_compensation[0], exact.psi_2[1]-COMPENSATION*exact.filtered_compensation[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
                // marino.psi_Qmu = AB2T(exact.psi_2[0]-COMPENSATION*exact.filtered_compensation[0], exact.psi_2[1]-COMPENSATION*exact.filtered_compensation[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Dmu = AB2M(FE.exact.psi_2_real_output[0], FE.exact.psi_2_real_output[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.exact.psi_2_real_output[0], FE.exact.psi_2_real_output[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*ohtani*/
            marino.psi_Dmu = AB2M(FE.ohtani.psi_2[0], FE.ohtani.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.ohtani.psi_2[0], FE.ohtani.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*holtz2002*/

            /*boldea*/
            // marino.psi_Dmu = AB2M(FE.boldea.psi_2[0], FE.boldea.psi_2[1], CTRL.S->cosT, CTRL.S->sinT);
            // marino.psi_Qmu = AB2T(FE.boldea.psi_2[0], FE.boldea.psi_2[1], CTRL.S->cosT, CTRL.S->sinT);

            /*picorr*/
            marino.psi_Dmu = AB2M(FE.picorr.psi_2[0], FE.picorr.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.picorr.psi_2[0], FE.picorr.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*harnefors*/
            marino.psi_Dmu = AB2M(FE.harnefors.psi_2[0], FE.harnefors.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.harnefors.psi_2[0], FE.harnefors.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*lascu*/
            marino.psi_Dmu = AB2M(FE.lascu.psi_2[0], FE.lascu.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.lascu.psi_2[0], FE.lascu.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*clest*/
            marino.psi_Dmu = AB2M(FE.clest.psi_2[0], FE.clest.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.clest.psi_2[0], FE.clest.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

            /*holtz2003*/
            marino.psi_Dmu = AB2M(FE.htz.psi_2[0], FE.htz.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            marino.psi_Qmu = AB2T(FE.htz.psi_2[0], FE.htz.psi_2[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
        }

    // #if PC_SIMULATION==FALSE
    // EALLOW;
    // CpuTimer1.RegsAddr->TCR.bit.TRB = 1; // reset cpu timer to period value
    // CpuTimer1.RegsAddr->TCR.bit.TSS = 0; // start/restart
    // CpuTimer_Before = CpuTimer1.RegsAddr->TIM.all; // get count
    // EDIS;
    // #endif

    // TODO: 反馈磁链用谁？
    marino.psi_Dmu = AB2M(FLUX_FEEDBACK_ALPHA, FLUX_FEEDBACK_BETA, (*CTRL).s->cosT, (*CTRL).s->sinT);
    marino.psi_Qmu = AB2T(FLUX_FEEDBACK_ALPHA, FLUX_FEEDBACK_BETA, (*CTRL).s->cosT, (*CTRL).s->sinT);
    #if PC_SIMULATION
        // marino.psi_Dmu = AB2M(ACM.psi_Amu, ACM.psi_Bmu, (*CTRL).s->cosT, (*CTRL).s->sinT);
        // marino.psi_Qmu = AB2T(ACM.psi_Amu, ACM.psi_Bmu, (*CTRL).s->cosT, (*CTRL).s->sinT);
    #endif

    // /*Simulation only flux*/
    // marino.psi_Dmu = simvm.psi_D2_ode1;
    // marino.psi_Qmu = simvm.psi_Q2_ode1;
    // marino.psi_Dmu = ACM.psi_Dmu; // nonono, the dq flux should be obtained using (*CTRL).s->cosT/sinT.
    // marino.psi_Qmu = ACM.psi_Qmu; // nonono, the dq flux should be obtained using (*CTRL).s->cosT/sinT.

    // flux error quantities should be updated when feedback is updated | verified: this has nothing to do with the biased xTL at high speeds
    marino.e_psi_Dmu = marino.psi_Dmu - (*CTRL).i->cmd_psi;
    marino.e_psi_Qmu = marino.psi_Qmu - 0.0;

    // marino.e_psi_Dmu *= marino_sat_d_axis_flux_control; // no luck
    // marino.e_psi_Qmu *= marino_sat_q_axis_flux_control; // no luck


    // API to the fourth-order system of observer and identifiers
    observer_marino2005();
    (*CTRL).i->theta_d_elec = marino.xRho;
    REAL xOMG = marino.xOmg; // * MOTOR.npp_inv;
    (*CTRL).motor->alpha    = marino.xAlpha;
    (*CTRL).i->TLoad        = marino.xTL;

    // #define xOMG (CTRL->i->varOmega*MOTOR.npp)
    // REAL xOMG = (*CTRL).i->varOmega * MOTOR.npp;

    // 磁场可测 debug
    // marino.psi_Dmu = simvm.psi_D2;
    // marino.psi_Qmu = simvm.psi_Q2;
    // (*CTRL).i->theta_d_elec = ACM.theta_M;
    // (*CTRL).i->varOmega = (*CTRL).i->varOmega;
    // (*CTRL).motor->alpha = ACM.alpha;
    // (*CTRL).i->TLoad = ACM.TLoad;

    // flux error quantities (moved up)
    // marino.e_psi_Dmu = marino.psi_Dmu - (*CTRL).i->cmd_psi;
    // marino.e_psi_Qmu = marino.psi_Qmu - 0.0;

    (*CTRL).motor->alpha_inv = 1.0/(*CTRL).motor->alpha;
    // (*CTRL).motor->Lmu = (*CTRL).motor->Rreq * (*CTRL).motor->alpha_inv;
    // (*CTRL).motor->Lmu_inv = 1.0 / (*CTRL).motor->Lmu;

    // αβ to DQ
    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    (*CTRL).i->iDQ[0] = AB2M(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T(IS_C(0), IS_C(1), (*CTRL).s->cosT, (*CTRL).s->sinT);

    if(TRUE){
        // 当磁链幅值给定平稳时，这项就是零。
        marino.deriv_iD_cmd = 1.0*(*CTRL).motor->Lmu_inv*(  (*CTRL).i->cmd_deriv_psi \
                                                + (*CTRL).i->cmd_dderiv_psi*(*CTRL).motor->alpha_inv \
                                                - (*CTRL).i->cmd_deriv_psi*(*CTRL).motor->alpha_inv*(*CTRL).motor->alpha_inv*marino.deriv_xAlpha);
        // 重新写！
        // REAL mu_temp     = (*CTRL).motor->npp_inv*(*CTRL).motor->Js * CLARKE_TRANS_TORQUE_GAIN_INVERSE*(*CTRL).motor->npp_inv;
        // REAL mu_temp_inv = (*CTRL).motor->npp*(*CTRL).motor->Js_inv * CLARKE_TRANS_TORQUE_GAIN*(*CTRL).motor->npp;
        // 第一项很有用，第二项无用。
        marino.deriv_iQ_cmd =   (*CTRL).motor->npp_inv*(*CTRL).motor->Js * CLARKE_TRANS_TORQUE_GAIN_INVERSE*(*CTRL).motor->npp_inv * (\
            1.0*(-marino.k_omega*deriv_sat_kappa(xOMG -(*CTRL).i->cmd_varOmega ) * (marino.deriv_xOmg - (*CTRL).i->cmd_deriv_varOmega ) + (*CTRL).motor->Js_inv*(*CTRL).motor->npp*marino.deriv_xTL + (*CTRL).i->cmd_dderiv_varOmega  ) * (*CTRL).i->cmd_psi_inv\
          - 1.0*(-marino.k_omega*      sat_kappa(xOMG -(*CTRL).i->cmd_varOmega ) + (*CTRL).motor->Js_inv*(*CTRL).motor->npp*(*CTRL).i->TLoad + (*CTRL).i->cmd_deriv_varOmega ) * ((*CTRL).i->cmd_deriv_psi * (*CTRL).i->cmd_psi_inv*(*CTRL).i->cmd_psi_inv)
            );

        // current error quantities
        (*CTRL).i->cmd_iDQ[0] = ( (*CTRL).i->cmd_psi + (*CTRL).i->cmd_deriv_psi*(*CTRL).motor->alpha_inv ) * (*CTRL).motor->Lmu_inv;
        (*CTRL).i->cmd_iDQ[1] = ( (*CTRL).motor->npp_inv*(*CTRL).motor->Js *( 1*(*CTRL).i->cmd_deriv_varOmega  - marino.k_omega*sat_kappa(xOMG -(*CTRL).i->cmd_varOmega ) ) + (*CTRL).i->TLoad ) * (CLARKE_TRANS_TORQUE_GAIN_INVERSE*(*CTRL).motor->npp_inv*(*CTRL).i->cmd_psi_inv);
        marino.e_iDs = (*CTRL).i->iDQ[0] - (*CTRL).i->cmd_iDQ[0];
        marino.e_iQs = (*CTRL).i->iDQ[1] - (*CTRL).i->cmd_iDQ[1];

        marino.torque_cmd = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * ((*CTRL).i->cmd_iDQ[1] * (*CTRL).i->cmd_psi   - (*CTRL).i->cmd_iDQ[0]*(0));
        marino.torque__fb = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * ((*CTRL).i->iDQ[1]     * marino.psi_Dmu - (*CTRL).i->iDQ[0] * marino.psi_Qmu);
        // marino.torque__fb = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * ((*CTRL).i->iDQ[1]     * marino.psi_Dmu);


        // linear combination of error
        marino.zD = marino.e_iDs + (*CTRL).motor->Lsigma_inv*marino.e_psi_Dmu;
        marino.zQ = marino.e_iQs + (*CTRL).motor->Lsigma_inv*marino.e_psi_Qmu;

        // known signals to feedforward (to cancel)
        marino.Gamma_D = (*CTRL).motor->Lsigma_inv * (-(*CTRL).motor->R*(*CTRL).i->iDQ[0] -(*CTRL).motor->alpha*(*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[0] +(*CTRL).motor->alpha  *(*CTRL).i->cmd_psi +(*CTRL).s->omega_syn*marino.e_psi_Qmu) +(*CTRL).s->omega_syn*(*CTRL).i->iDQ[1] - marino.deriv_iD_cmd;
        marino.Gamma_Q = (*CTRL).motor->Lsigma_inv * (-(*CTRL).motor->R*(*CTRL).i->iDQ[1] -(*CTRL).motor->alpha*(*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[1] -xOMG *(*CTRL).i->cmd_psi -(*CTRL).s->omega_syn*marino.e_psi_Dmu) -(*CTRL).s->omega_syn*(*CTRL).i->iDQ[0] - marino.deriv_iQ_cmd;

        // voltage commands
        (*CTRL).o->cmd_uDQ[0] = (*CTRL).motor->Lsigma * (-(marino.kz+0.25*(*CTRL).motor->Lsigma*(*CTRL).motor->Lmu*marino.xAlpha_Max)*marino.zD - marino.Gamma_D);
        (*CTRL).o->cmd_uDQ[1] = (*CTRL).motor->Lsigma * (-(marino.kz+0.25*(*CTRL).motor->Lsigma*(*CTRL).motor->Lmu*marino.xAlpha_Max)*marino.zQ - marino.Gamma_Q);

    }else{
        // PI control with marino observer works if 
        // damping factor = 6.5
        // VLBW = 5 Hz
        PID_iD->Fbk = (*CTRL).i->iDQ[0];
        PID_iQ->Fbk = (*CTRL).i->iDQ[1];

        /// 5. 转速环
        static int im_vc_count = 1;
        if(im_vc_count++ == SPEED_LOOP_CEILING){
            im_vc_count = 1;

            PID_Speed->Ref = (*CTRL).i->cmd_varOmega ; //rpm_speed_command*RPM_2_ELEC_RAD_PER_SEC;
            PID_Speed->Fbk = xOMG ;
            PID_Speed->calc(PID_Speed);
            PID_iQ->Ref = PID_Speed->Out;
            (*CTRL).i->cmd_iDQ[1] = PID_iQ->Ref;
        }
        (*CTRL).i->cmd_psi = IM_FLUX_COMMAND_DC_PART;
        (*CTRL).i->cmd_iDQ[0] = (*CTRL).i->cmd_psi * (*CTRL).motor->Lmu_inv;
        PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];

        /// 6. 电流环
        REAL decoupled_M_axis_voltage=0.0, decoupled_T_axis_voltage=0.0;
        PID_iD->calc(PID_iD);
        PID_iQ->calc(PID_iQ);

        decoupled_M_axis_voltage = PID_iD->Out;
        decoupled_T_axis_voltage = PID_iQ->Out;

        (*CTRL).o->cmd_uDQ[0] = decoupled_M_axis_voltage;
        (*CTRL).o->cmd_uDQ[1] = decoupled_T_axis_voltage;

        /// 7. 反帕克变换
        // (*CTRL).o->cmd_uAB[0] = MT2A(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);
        // (*CTRL).o->cmd_uAB[1] = MT2B(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);
    }

    (*CTRL).o->cmd_uAB[0] = MT2A((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B((*CTRL).o->cmd_uDQ[0], (*CTRL).o->cmd_uDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

    // #if PC_SIMULATION==FALSE
    // CpuTimer_After = CpuTimer1.RegsAddr->TIM.all; // get count
    // CpuTimer_Delta = (REAL)CpuTimer_Before - (REAL)CpuTimer_After;
    // #endif

    // use the second 3 phase inverter
    // (*CTRL).o->cmd_uAB[0+2] = (*CTRL).o->cmd_uAB[0];
    // (*CTRL).o->cmd_uAB[1+2] = (*CTRL).o->cmd_uAB[1];

    // for view in scope
    // (*CTRL).o->cmd_iAB[0] = MT2A((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    // (*CTRL).o->cmd_iAB[1] = MT2B((*CTRL).i->cmd_iDQ[0], (*CTRL).i->cmd_iDQ[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
}



void controller_IFOC(){

    /// 3. 电气转子位置和电气转子转速反馈

    flux_observer(); // FLUX_FEEDBACK_ALPHA, FLUX_FEEDBACK_BETA
    // (*CTRL).i->varOmega = FE.htz.omg_est;=

                    // (*CTRL).i->varOmega = OFSR.esoaf.xOmg - (OFSR.esoaf.bool_ramp_load_torque<0) * (*CTRL).s->omega_sl;
                    // (*CTRL).i->varOmega = OFSR.esoaf.xOmg;

                        //（编码器反馈）
                        // (*CTRL).i->varOmega     = qep.varOmega;
                        // (*CTRL).i->theta_d_elec__fb = qep.theta_d;

                        //（实际反馈，实验中不可能）
                        // (*CTRL).i->varOmega     = ENC.varOmega ;
                        // (*CTRL).i->varOmega     = ACM.varOmega ;
                        // (*CTRL).i->varOmega     = ACM.x[4] ;

                        //（无感）
                        // harnefors_scvm();
                        // (*CTRL).i->varOmega     = omg_harnefors;
                        // (*CTRL).i->theta_d_elec__fb = theta_d_harnefors;

    // 间接磁场定向第一部分
    (*CTRL).s->xRho += CL_TS * (*CTRL).s->omega_syn;
    // (*CTRL).s->xRho = ACM.theta_d; // debug
    (*CTRL).i->theta_d_elec = (*CTRL).s->xRho;

    (*CTRL).s->cosT = cos((*CTRL).i->theta_d_elec);
    (*CTRL).s->sinT = sin((*CTRL).i->theta_d_elec);
    if((*CTRL).s->xRho > M_PI){
        (*CTRL).s->xRho -= 2*M_PI;
    }else if((*CTRL).s->xRho < -M_PI){
        (*CTRL).s->xRho += 2*M_PI; // 反转！
    }

    /// 4. 帕克变换
    (*CTRL).i->iDQ[0] = AB2M((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T((*CTRL).i->iAB[0], (*CTRL).i->iAB[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
    PID_iD->Fbk = (*CTRL).i->iDQ[0];
    PID_iQ->Fbk = (*CTRL).i->iDQ[1];

    /// 5. 转速环
    static int im_vc_count = 1;
    if(im_vc_count++ == SPEED_LOOP_CEILING){
        im_vc_count = 1;

        PID_Speed->Ref = (*CTRL).i->cmd_varOmega; //rpm_speed_command*RPM_2_ELEC_RAD_PER_SEC;
        PID_Speed->Fbk = (*CTRL).i->varOmega;
        PID_Speed->calc(PID_Speed);
        PID_iQ->Ref = PID_Speed->Out;
        (*CTRL).i->cmd_iDQ[1] = PID_iQ->Ref;
    }
    // 磁链环
    // if(ob.taao_flux_cmd_on){
    //     (*CTRL).i->cmd_iDQ[0] = IM_FLUX_COMMAND_DC_PART * (*CTRL).motor->Lmu_inv   \
    //                    + (   M1*OMG1*cos(OMG1*ob.timebase)  )/ (*CTRL).motor->Rreq; ///////////////////////////////// 
    // }else{
        // (*CTRL).i->cmd_iDQ[0] =  * (*CTRL).motor->Lmu_inv + (deriv_fluxModCmd)/ (*CTRL).motor->Rreq; 
        (*CTRL).i->cmd_psi = IM_FLUX_COMMAND_DC_PART;
        (*CTRL).i->cmd_iDQ[0] = (*CTRL).i->cmd_psi * (*CTRL).motor->Lmu_inv;
    // }
    PID_iD->Ref = (*CTRL).i->cmd_iDQ[0];
    // 计算转矩
    (*CTRL).i->cmd_Tem = CLARKE_TRANS_TORQUE_GAIN * (*CTRL).motor->npp * (*CTRL).i->cmd_iDQ[1] * ((*CTRL).i->cmd_psi);
    // 间接磁场定向第二部分
    (*CTRL).s->omega_sl  = (*CTRL).motor->Rreq*(*CTRL).i->cmd_iDQ[1]/((*CTRL).i->cmd_psi);
    (*CTRL).s->omega_syn = (*CTRL).i->varOmega * CTRL->motor->npp + (*CTRL).s->omega_sl;

    /// 5.Extra 扫频将覆盖上面产生的励磁、转矩电流指令
    #if EXCITATION_TYPE == EXCITATION_SWEEP_FREQUENCY
        #if SWEEP_FREQ_C2V == TRUE
            PID_iQ->Ref = amp_current_command; 
        #endif
        #if SWEEP_FREQ_C2C == TRUE
            PID_iQ->Ref = 0.0;
            PID_iD->Ref = amp_current_command; // 故意反的
        #else
            PID_iD->Ref = 0.0;
        #endif
    #endif



    /// 6. 电流环
    REAL decoupled_M_axis_voltage=0.0, decoupled_T_axis_voltage=0.0;
    PID_iD->calc(PID_iD);
    PID_iQ->calc(PID_iQ);
    {   // Steady state dynamics based decoupling circuits for current regulation
        if (d_sim.FOC.bool_apply_decoupling_voltages_to_current_regulation == TRUE){
            // decoupled_M_axis_voltage = vM + ((*CTRL).motor->R+(*CTRL).motor->Rreq)*(*CTRL).iMs + (*CTRL).motor->Lsigma*(-(*CTRL).s->omega_syn*(*CTRL).iTs) - (*CTRL).motor->alpha*(*CTRL).psimod_fb; // Jadot09
            // decoupled_T_axis_voltage = vT + ((*CTRL).motor->R+(*CTRL).motor->Rreq)*(*CTRL).iTs + (*CTRL).motor->Lsigma*( (*CTRL).s->omega_syn*(*CTRL).iMs) + (*CTRL).omg_fb*(*CTRL).psimod_fb;
            // decoupled_T_axis_voltage = vT + (*CTRL).s->omega_syn*((*CTRL).motor->Lsigma+(*CTRL).motor->Lmu)*(*CTRL).iMs; // 这个就不行，说明：(*CTRL).motor->Lmu*iMs != ob.taao_flux_cmd，而是会因iMs的波动在T轴控制上引入波动和不稳定

            decoupled_M_axis_voltage = PID_iD->Out + ((*CTRL).motor->Lsigma) * (-(*CTRL).s->omega_syn*(*CTRL).i->iDQ[1]); // Telford03/04
            decoupled_T_axis_voltage = PID_iQ->Out + (*CTRL).s->omega_syn*((*CTRL).i->cmd_psi + (*CTRL).motor->Lsigma*(*CTRL).i->iDQ[0]); // 这个行，但是无速度运行时，会导致M轴电流在转速暂态高频震荡。
            // decoupled_T_axis_voltage = PID_iQ->Out; // 无感用这个
        }else{
            decoupled_M_axis_voltage = PID_iD->Out;
            decoupled_T_axis_voltage = PID_iQ->Out;
        }
    }

    /// 7. 反帕克变换
    (*CTRL).o->cmd_uAB[0] = MT2A(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).o->cmd_uAB[1] = MT2B(decoupled_M_axis_voltage, decoupled_T_axis_voltage, (*CTRL).s->cosT, (*CTRL).s->sinT);


    /// 8. 补偿逆变器非线性
    main_inverter_voltage_command(TRUE);

}

void init_im_controller(){

    #define LOCAL_SCALE 1.0
    marino.kz         = LOCAL_SCALE * 2*700.0; // zd, zq
    marino.k_omega    = LOCAL_SCALE * 0.5*88*60.0; // 6000  // e_omega // 增大这个可以消除稳态转速波形中的正弦扰动（源自q轴电流给定波形中的正弦扰动，注意实际的q轴电流里面是没有正弦扰动的）


    marino.kappa      = 1e4*24; // \in [0.1, 1e4*24] no difference // e_omega // 增大这个意义不大，转速控制误差基本上已经是零了，所以kappa取0.05和24没有啥区别。

    // lammbda_inv和gamma_inv是竞争的关系
    // marino.lambda_inv = 5* 0.1 * 1.5 * 6000.0;          // omega 磁链反馈为实际值时，这两个增益取再大都没有意义。
    // marino.gamma_inv  = 10 * 3e0 * 180/INIT_JS; // TL    磁链反馈为实际值时，这两个增益取再大都没有意义。
    // marino.delta_inv  = 0*75.0; // alpha 要求磁链幅值时变

    marino.lambda_inv = LAMBDA_INV_xOmg;  // 2022-11-22 实验中发现这个过大（2700）导致系统整个一个正弦波动，母线功率持续400W，减小为1000以后母线功率37W。
    marino.gamma_inv  = GAMMA_INV_xTL;
    marino.delta_inv  = DELTA_INV_alpha;

    marino.xTL_Max = 8.0;
    marino.xAlpha_Max = 8.0;
    marino.xAlpha_min = 1.0;

    marino.xRho = 0.0;
    marino.xTL = 0.0;
    marino.xAlpha = d_sim.init.Rreq / IM_MAGNETIZING_INDUCTANCE;
    marino.xOmg = 0.0;

    marino.deriv_xTL = 0.0;
    marino.deriv_xAlpha = 0.0;
    marino.deriv_xOmg = 0.0;

    marino.psi_Dmu = 0.0;
    marino.psi_Qmu = 0.0;

    marino.zD = 0.0;
    marino.zQ = 0.0;
    marino.e_iDs = 0.0;
    marino.e_iQs = 0.0;
    marino.e_psi_Dmu = 0.0;
    marino.e_psi_Qmu = 0.0;

    marino.deriv_iD_cmd = 0.0;
    marino.deriv_iQ_cmd = 0.0;

    marino.Gamma_D = 0.0;
    marino.Gamma_Q = 0.0;

    marino.torque_cmd = 0.0;
    marino.torque__fb = 0.0;

    // struct Holtz2003
    simvm.psi_D2 = 0.0;
    simvm.psi_Q2 = 0.0;
    simvm.psi_D1_ode1 = 0.0;
    simvm.psi_Q1_ode1 = 0.0;
    simvm.psi_D2_ode1 = 0.0;
    simvm.psi_Q2_ode1 = 0.0;
    simvm.psi_D1_ode4 = 0.0;
    simvm.psi_Q1_ode4 = 0.0;
    simvm.psi_D2_ode4 = 0.0;
    simvm.psi_Q2_ode4 = 0.0;





    int i=0,j=0;

    (*CTRL).timebase = 0.0;

    /* Parameter (including speed) Adaptation */ 
        (*CTRL).motor->R      = d_sim.init.R;
        (*CTRL).motor->Rreq   = d_sim.init.Rreq;

        (*CTRL).motor->npp    = d_sim.init.npp;
        (*CTRL).motor->Lsigma = d_sim.init.Lq;
        (*CTRL).motor->Lmu    = d_sim.init.Ld - d_sim.init.Lq;
        (*CTRL).motor->Js     = d_sim.init.Js;

        (*CTRL).motor->alpha  = (*CTRL).motor->Rreq/(*CTRL).motor->Lmu;
        (*CTRL).motor->alpha_inv = 1.0/(*CTRL).motor->alpha;

        (*CTRL).motor->npp_inv     = 1.0/(*CTRL).motor->npp;
        (*CTRL).motor->Lsigma_inv  = 1.0/(*CTRL).motor->Lsigma;
        (*CTRL).motor->Lmu_inv     = 1.0/(*CTRL).motor->Lmu;
        (*CTRL).motor->Js_inv      = 1.0/(*CTRL).motor->Js;

        // (*CTRL).i->TLoad  = 0.0;

    (*CTRL).s->cosT = 1.0;
    (*CTRL).s->sinT = 0.0;
    (*CTRL).s->cosT2 = 1.0;
    (*CTRL).s->sinT2 = 0.0;

    (*CTRL).i->m0 = IM_FLUX_COMMAND_DC_PART;
    (*CTRL).i->m1 = IM_FLUX_COMMAND_SINE_PART;
    (*CTRL).i->omega1 = 2*M_PI*IM_FLUX_COMMAND_SINE_HERZ;

    // (*debug).SENSORLESS_CONTROL = SENSORLESS_CONTROL;
    // (*CTRL).s->ctrl_strategy = CONTROL_STRATEGY;

    #define AKATSU00 FALSE
    #if AKATSU00 == TRUE
    int ind;
    for(ind=0;ind<2;++ind){
    // for(i=0;i<2;++i){
        hav.emf_stator[ind] = 0;

        hav.psi_1[ind] = 0;
        hav.psi_2[ind] = 0;
        hav.psi_2_prev[ind] = 0;

        hav.psi_1_nonSat[ind] = 0;
        hav.psi_2_nonSat[ind] = 0;

        hav.psi_1_min[ind] = 0;
        hav.psi_1_max[ind] = 0;
        hav.psi_2_min[ind] = 0;
        hav.psi_2_max[ind] = 0;

        hav.rs_est = 3.04;
        hav.rreq_est = 1.6;

        hav.Delta_t = 1;
        hav.u_off[ind] = 0;
        hav.u_off_integral_input[ind] = 0;
        hav.gain_off = 0.025;

        hav.flag_pos2negLevelA[ind] = 0;
        hav.flag_pos2negLevelB[ind] = 0;
        hav.time_pos2neg[ind] = 0;
        hav.time_pos2neg_prev[ind] = 0;

        hav.flag_neg2posLevelA[ind] = 0;
        hav.flag_neg2posLevelB[ind] = 0;
        hav.time_neg2pos[ind] = 0;
        hav.time_neg2pos_prev[ind] = 0;    

        hav.sat_min_time[ind] = 0.0;
        hav.sat_max_time[ind] = 0.0;
    }

    a92v.awaya_lambda = 31.4*1;
    a92v.q0 = 0.0;
    a92v.q1_dot = 0.0;
    a92v.q1 = 0.0;
    a92v.tau_est = 0.0;
    a92v.sum_A = 0.0;
    a92v.sum_B = 0.0;
    a92v.est_Js_variation = 0.0;
    a92v.est_Js = 0.0;
    #endif

    // /*Jadot2009*/
    // (*CTRL).is_ref[0] = 0.0;
    // (*CTRL).is_ref[1] = 0.0;
    // (*CTRL).psi_ref[0] = 0.0;
    // (*CTRL).psi_ref[1] = 0.0;

    // (*CTRL).pi_vsJadot_0.Kp = 7; 
    // (*CTRL).pi_vsJadot_0.Ti = 1.0/790.0; 
    // (*CTRL).pi_vsJadot_0.Kp = 15; 
    // (*CTRL).pi_vsJadot_0.Ti = 0.075; 
    // (*CTRL).pi_vsJadot_0.Ki = (*CTRL).pi_vsJadot_0.Kp / (*CTRL).pi_vsJadot_0.Ti * TS;
    // (*CTRL).pi_vsJadot_0.i_state = 0.0;
    // (*CTRL).pi_vsJadot_0.i_limit = 300.0; // unit: Volt

    // (*CTRL).pi_vsJadot_1.Kp = 7; 
    // (*CTRL).pi_vsJadot_1.Ti = 1.0/790.0; 
    // (*CTRL).pi_vsJadot_1.Kp = 15; 
    // (*CTRL).pi_vsJadot_1.Ti = 0.075; 
    // (*CTRL).pi_vsJadot_1.Ki = (*CTRL).pi_vsJadot_1.Kp / (*CTRL).pi_vsJadot_1.Ti * TS;
    // (*CTRL).pi_vsJadot_1.i_state = 0.0;
    // (*CTRL).pi_vsJadot_1.i_limit = 300.0; // unit: Volt

    // PID调谐
    // ACMSIMC_PIDTuner();
}









































#include "ACMSim.h"

// #if PC_SIMULATION==FALSE
// REAL CpuTimer_Delta = 0;
// Uint32 CpuTimer_Before = 0;
// Uint32 CpuTimer_After = 0;
// #endif

#if MACHINE_TYPE == 1 || MACHINE_TYPE == 11
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)

#if PC_SIMULATION
    #define OFFSET_VOLTAGE_ALPHA 0//(1*-0.01 *((*CTRL).timebase>0.3))//0//(0*0.1 *((*CTRL).timebase>20)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
    #define OFFSET_VOLTAGE_BETA  0//(1*+0.01 *((*CTRL).timebase>0.3))//0//(0*0.1 *((*CTRL).timebase>4)) // (0.02*29*1.0) // this is only valid for estimator in AB frame. Use current_offset instead for DQ frame estimator
#else
    #define OFFSET_VOLTAGE_ALPHA (0.0)
    #define OFFSET_VOLTAGE_BETA  (0.0)
#endif

/********************************************
 * Marino 2005 Observer Part
 ********************************************/

/* The fourth-order dynamic system by Marino2005@Wiley */
void rhs_func_marino2005(REAL *increment_n, REAL xRho, REAL xTL, REAL xAlpha, REAL xOmg, REAL hs){
    // pointer to increment_n: state increment at n-th stage of RK4, where n=1,2,3,4.
    // *x???: pointer to state variABles
    // x????: state variABle
    // hs: step size of numerical integral

    /* 把磁链观测嫁到这里来？？*/
    /* 把磁链观测嫁到这里来？？*/
    /* 把磁链观测嫁到这里来？？*/

    // time-varying quantities
    (*CTRL).s->cosT = cos(xRho); // 这里体现了该观测器的非线性——让 q 轴电流给定和观测转速之间通过 iQs 的值产生了联系
    (*CTRL).s->sinT = sin(xRho); // 这里体现了该观测器的非线性——让 q 轴电流给定和观测转速之间通过 iQs 的值产生了联系
    (*CTRL).i->iDQ[0] = AB2M(IS(0), IS(1), (*CTRL).s->cosT, (*CTRL).s->sinT);
    (*CTRL).i->iDQ[1] = AB2T(IS(0), IS(1), (*CTRL).s->cosT, (*CTRL).s->sinT);

    // f = \dot x = the time derivative
    REAL f[4];
    // xRho
    f[0] = xOmg + xAlpha*(*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[1]*(*CTRL).i->cmd_psi_inv;
    // xTL
    f[1] = - marino.gamma_inv * (*CTRL).motor->Js * (*CTRL).i->cmd_psi * marino.e_psi_Qmu;
    // xAlpha
    f[2] = marino.delta_inv * (   xAlpha_LAW_TERM_D * marino.e_psi_Dmu*((*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[0] - (*CTRL).i->cmd_psi) \
                                + xAlpha_LAW_TERM_Q * marino.e_psi_Qmu* (*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[1]);
    // xOmg
    REAL xTem = CLARKE_TRANS_TORQUE_GAIN*(*CTRL).motor->npp*( marino.psi_Dmu*(*CTRL).i->iDQ[1] - marino.psi_Qmu*(*CTRL).i->iDQ[0] );
    f[3] = (*CTRL).motor->npp*(*CTRL).motor->Js_inv*(xTem - xTL) + 2*marino.lambda_inv*(*CTRL).i->cmd_psi*marino.e_psi_Qmu;

    // bad
    // f[0] = xOmg + xAlpha*(*CTRL).motor->Lmu*(*CTRL).i->iDQ[1]*(*CTRL).i->cmd_psi_inv;

    // REAL xTem = CLARKE_TRANS_TORQUE_GAIN*(*CTRL).motor->npp*( marino.psi_Dmu*ACM.iTs - marino.psi_Qmu*ACM.iMs );
    // f[3] = (*CTRL).motor->npp*(*CTRL).motor->Js_inv*(ACM.Tem - ACM.TLoad);
    // f[3] = (*CTRL).motor->npp*(*CTRL).motor->Js_inv*(CLARKE_TRANS_TORQUE_GAIN*ACM.npp*(ACM.x[1]*ACM.x[2]-ACM.x[0]*ACM.x[3] - ACM.TLoad));

    // f[3] = ACM.x_dot[4]; 

    increment_n[0] = ( f[0] )*hs;
    increment_n[1] = ( f[1] )*hs;
    increment_n[2] = ( f[2] )*hs;
    increment_n[3] = ( f[3] )*hs;
}
void marino05_dedicated_rk4_solver(REAL hs){
    static REAL increment_1[4];
    static REAL increment_2[4];
    static REAL increment_3[4];
    static REAL increment_4[4];
    static REAL x_temp[4];
    static REAL *p_x_temp=x_temp;

    /* Theoritically speaking, rhs_func should be time-varing like rhs_func(.,t).
       To apply codes in DSP, we do time-varing updating of IS(0) and IS(1) outside rhs_func(.) to save time. */

    /* 
     * Begin RK4 
     * */
    // time instant t
    US(0) = US_P(0);
    US(1) = US_P(1);
    IS(0) = IS_P(0);
    IS(1) = IS_P(1);
    rhs_func_marino2005( increment_1, marino.xRho, marino.xTL, marino.xAlpha, marino.xOmg, hs); 
    x_temp[0]  = marino.xRho   + increment_1[0]*0.5;
    x_temp[1]  = marino.xTL    + increment_1[1]*0.5;
    x_temp[2]  = marino.xAlpha + increment_1[2]*0.5;
    x_temp[3]  = marino.xOmg   + increment_1[3]*0.5;

    // time instant t+hs/2
    IS(0) = 0.5*(IS_P(0)+IS_C(0));
    IS(1) = 0.5*(IS_P(1)+IS_C(1));
    rhs_func_marino2005( increment_2, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    x_temp[0]  = marino.xRho   + increment_2[0]*0.5;
    x_temp[1]  = marino.xTL    + increment_2[1]*0.5;
    x_temp[2]  = marino.xAlpha + increment_2[2]*0.5;
    x_temp[3]  = marino.xOmg   + increment_2[3]*0.5;

    // time instant t+hs/2
    rhs_func_marino2005( increment_3, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    x_temp[0]  = marino.xRho   + increment_3[0];
    x_temp[1]  = marino.xTL    + increment_3[1];
    x_temp[2]  = marino.xAlpha + increment_3[2];
    x_temp[3]  = marino.xOmg   + increment_3[3];

    // time instant t+hs
    IS(0) = IS_C(0);
    IS(1) = IS_C(1);
    rhs_func_marino2005( increment_4, *(p_x_temp+0), *(p_x_temp+1), *(p_x_temp+2), *(p_x_temp+3), hs );
    // \+=[^\n]*1\[(\d+)\][^\n]*2\[(\d+)\][^\n]*3\[(\d+)\][^\n]*4\[(\d+)\][^\n]*/ ([\d]+)
    // +=   (increment_1[$5] + 2*(increment_2[$5] + increment_3[$5]) + increment_4[$5])*0.166666666666667; // $5
    marino.xRho        += (increment_1[0] + 2*(increment_2[0] + increment_3[0]) + increment_4[0])*0.166666666666667; // 0
    marino.xTL         += (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667; // 1
    marino.xAlpha      += (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667; // 2
    marino.xOmg        += (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667; // 3

    // Also get derivatives:
    (*CTRL).s->omega_sl    = marino.xAlpha*(*CTRL).motor->Lmu*(*CTRL).i->cmd_iDQ[1]*(*CTRL).i->cmd_psi_inv;
    (*CTRL).s->omega_syn   = marino.xOmg + (*CTRL).s->omega_sl;
    marino.deriv_xTL    = (increment_1[1] + 2*(increment_2[1] + increment_3[1]) + increment_4[1])*0.166666666666667 * CL_TS_INVERSE;
    marino.deriv_xAlpha = (increment_1[2] + 2*(increment_2[2] + increment_3[2]) + increment_4[2])*0.166666666666667 * CL_TS_INVERSE;
    marino.deriv_xOmg   = (increment_1[3] + 2*(increment_2[3] + increment_3[3]) + increment_4[3])*0.166666666666667 * CL_TS_INVERSE;



    // Projection Algorithm
    // xRho   \in [-M_PI, M_PI]
    if(marino.xRho > M_PI){
        marino.xRho -= 2*M_PI;
    }else if(marino.xRho < -M_PI){
        marino.xRho += 2*M_PI; // 反转！
    }
    // xTL    \in [-xTL_Max, xTL_Max]
    ;
    // xAlpha \in [-xAlpha_min, xAlpha_Max]
    if(marino.xAlpha > marino.xAlpha_Max){
        marino.xAlpha = marino.xAlpha_Max;
    }else if(marino.xAlpha < marino.xAlpha_min){
        marino.xAlpha = marino.xAlpha_min;
    }
}

#define TAU_OFFSET_INVERSE 5 // 0.1滤得更狠，但是高速后不能有效减小误差了 // 0.5
#define K_VM (0.5*0)
#if PC_SIMULATION
    /* Flux estimator that is only valid in simulation */
    void simulation_only_flux_estimator(){
        // Flux Measurements
        // simvm.psi_D2 = ACM.psi_Dmu;
        // simvm.psi_Q2 = ACM.psi_Qmu;

        // CM
        FE.fme.psi_DQ2[0] += CL_TS * (-(*CTRL).motor->alpha*FE.fme.psi_DQ2[0] + (*CTRL).motor->Rreq*(*CTRL).i->iDQ[0]);
        FE.fme.psi_DQ1[0] = FE.fme.psi_DQ2[0] + (*CTRL).motor->Lq*(*CTRL).i->iDQ[0];


        // Flux Estimator (Open Loop Voltage Model in αβ frame + ODE1)
        if(TRUE){
            // 
            simvm.emf[0] = (*CTRL).o->cmd_uAB[0] - (*CTRL).motor->R*(*CTRL).i->iAB[0] + 1*OFFSET_VOLTAGE_ALPHA;
            simvm.emf[1] = (*CTRL).o->cmd_uAB[1] - (*CTRL).motor->R*(*CTRL).i->iAB[1] + 1*OFFSET_VOLTAGE_BETA;
            simvm.psi_1[0] += CL_TS * ( simvm.emf[0] - 1*K_VM*(simvm.psi_1[0] - FE.fme.psi_DQ1[0]* (*CTRL).s->cosT) );
            simvm.psi_1[1] += CL_TS * ( simvm.emf[1] - 1*K_VM*(simvm.psi_1[1] - FE.fme.psi_DQ1[0]* (*CTRL).s->sinT) );
            simvm.psi_2[0] = simvm.psi_1[0] - (*CTRL).motor->Lq*(*CTRL).i->iAB[0];
            simvm.psi_2[1] = simvm.psi_1[1] - (*CTRL).motor->Lq*(*CTRL).i->iAB[1];
            simvm.psi_2_ampl = sqrt(simvm.psi_2[0]*simvm.psi_2[0]+simvm.psi_2[1]*simvm.psi_2[1]);

            REAL al = simvm.psi_2[0] - 0*(simvm.psi_1[0] - FE.fme.psi_DQ1[0]* (*CTRL).s->cosT);
            REAL be = simvm.psi_2[1]  - 0*(simvm.psi_1[1]  - FE.fme.psi_DQ1[0]* (*CTRL).s->sinT);
            simvm.psi_D2_ode1_v2 = AB2M(al, be, (*CTRL).s->cosT, (*CTRL).s->sinT);
            simvm.psi_Q2_ode1_v2 = AB2T(al, be, (*CTRL).s->cosT, (*CTRL).s->sinT);

            simvm.emf_DQ[0] = AB2M(simvm.emf[0], simvm.emf[1], (*CTRL).s->cosT, (*CTRL).s->sinT);
            simvm.emf_DQ[1] = AB2T(simvm.emf[0], simvm.emf[1], (*CTRL).s->cosT, (*CTRL).s->sinT);

        }else{
            static REAL temp_Alpha1 = 0.0;
            static REAL temp_Beta1 = 0.0;
            static REAL temp_Alpha2 = 0.0;
            static REAL temp_Beta2 = 0.0;

            // static REAL FE.exact.offset_voltage_compensation[2] = {0,0};
            FE.exact.offset_voltage_compensation[0] = _lpf(FE.exact.u_offset[0], FE.exact.offset_voltage_compensation[0], TAU_OFFSET_INVERSE);
            FE.exact.offset_voltage_compensation[1] = _lpf(FE.exact.u_offset[1], FE.exact.offset_voltage_compensation[1], TAU_OFFSET_INVERSE);

            temp_Alpha1 += CL_TS * ( (*CTRL).o->cmd_uAB[0] - (*CTRL).motor->R*(*CTRL).i->iAB[0] + 1*OFFSET_VOLTAGE_ALPHA - 0*FE.exact.u_offset[0] - 0.5*FE.exact.u_offset[0] - 1*FE.exact.offset_voltage_compensation[0])     + 0*GAIN_OHTANI * ( (*CTRL).i->cmd_psi_active[0] - (temp_Alpha1-(*CTRL).motor->Lq*(*CTRL).i->iAB[0]) );
            temp_Beta1  += CL_TS * ( (*CTRL).o->cmd_uAB[1] - (*CTRL).motor->R*(*CTRL).i->iAB[1] + 1*OFFSET_VOLTAGE_BETA  - 0*FE.exact.u_offset[1] - 0.5*FE.exact.u_offset[1] - 1*FE.exact.offset_voltage_compensation[1])     + 0*GAIN_OHTANI * ( (*CTRL).i->cmd_psi_active[1] - (temp_Beta1 -(*CTRL).motor->Lq*(*CTRL).i->iAB[1]) );
            temp_Alpha2 = temp_Alpha1 - (*CTRL).motor->Lq*(*CTRL).i->iAB[0];
            temp_Beta2  = temp_Beta1  - (*CTRL).motor->Lq*(*CTRL).i->iAB[1];

            REAL al = temp_Alpha2 - 0*0.5*(FE.exact.psi_2_last_top[0]+FE.exact.psi_2_last_butt[0]);
            REAL be = temp_Beta2  - 0*0.5*(FE.exact.psi_2_last_top[1]+FE.exact.psi_2_last_butt[1]);
            simvm.psi_D2_ode1_v2 = AB2M(al, be, (*CTRL).s->cosT, (*CTRL).s->sinT);
            simvm.psi_Q2_ode1_v2 = AB2T(al, be, (*CTRL).s->cosT, (*CTRL).s->sinT);
                // simvm.psi_D2_ode1_v2 = AB2M(temp_Alpha2, temp_Beta2, (*CTRL).s->cosT, (*CTRL).s->sinT);
                // simvm.psi_Q2_ode1_v2 = AB2T(temp_Alpha2, temp_Beta2, (*CTRL).s->cosT, (*CTRL).s->sinT);
        }

        // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
        // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
        // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
        // REAL ampl, cosT, sinT;
        // ampl = sqrtf(temp_Alpha2*temp_Alpha2+temp_Beta2*temp_Beta2);
        // if(ampl != 0){
        //     cosT = temp_Alpha2 / ampl;
        //     sinT = temp_Beta2  / ampl;
        //     simvm.psi_D2_ode1_v2 = AB2M(temp_Alpha2, temp_Beta2, cosT, sinT);
        //     simvm.psi_Q2_ode1_v2 = AB2T(temp_Alpha2, temp_Beta2, cosT, sinT);
        // }

        // simvm.psi_D2_ode1 = simvm.psi_D2_ode1_v2;
        // simvm.psi_Q2_ode1 = simvm.psi_Q2_ode1_v2;
        // simvm.psi_D2_ode1 = ACM.psi_Dmu;
        // simvm.psi_Q2_ode1 = ACM.psi_Qmu;




        REAL deriv_psi_D1;
        REAL deriv_psi_Q1;
        if(FALSE){
            // With CM Correction
            deriv_psi_D1 = (*CTRL).o->cmd_uDQ[0] - (*CTRL).motor->R*(*CTRL).i->iDQ[0] + (*CTRL).s->omega_syn*simvm.psi_Q1_ode1 - 1*K_VM*(simvm.psi_D1_ode1 - FE.fme.psi_DQ1[0]);
            deriv_psi_Q1 = (*CTRL).o->cmd_uDQ[1] - (*CTRL).motor->R*(*CTRL).i->iDQ[1] - (*CTRL).s->omega_syn*simvm.psi_D1_ode1 - 0*K_VM*(simvm.psi_Q1_ode1 - 0);
        }else{
            // Flux Estimator (Open Loop Voltage Model in indirect oriented DQ frame + ODE1)
            deriv_psi_D1 = (*CTRL).o->cmd_uDQ[0] - (*CTRL).motor->R*(*CTRL).i->iDQ[0] + (*CTRL).s->omega_syn*simvm.psi_Q1_ode1;
            deriv_psi_Q1 = (*CTRL).o->cmd_uDQ[1] - (*CTRL).motor->R*(*CTRL).i->iDQ[1] - (*CTRL).s->omega_syn*simvm.psi_D1_ode1;
        }

        simvm.psi_D1_ode1 += CL_TS * (deriv_psi_D1);
        simvm.psi_Q1_ode1 += CL_TS * (deriv_psi_Q1);
        simvm.psi_D2_ode1 = simvm.psi_D1_ode1 - (*CTRL).motor->Lq*(*CTRL).i->iDQ[0];
        simvm.psi_Q2_ode1 = simvm.psi_Q1_ode1 - (*CTRL).motor->Lq*(*CTRL).i->iDQ[1];

        // FE.fme.psi_DQ2_output[0] = simvm.psi_D2_ode1 - K_VM*(simvm.psi_Q1_ode1 - FE.fme.psi_DQ1[0]);
        // FE.fme.psi_DQ2_output[1] = simvm.psi_Q2_ode1 - K_VM*(simvm.psi_D1_ode1 - 0);


        FE.fme.temp += (marino.psi_Dmu - FE.fme.psi_DQ2[0]) * (*CTRL).s->cosT;
    }
#endif

/* Main Observer for Marino05 */
void observer_marino2005(){

    /* OBSERVATION */
    marino05_dedicated_rk4_solver(1*CL_TS);

    /* 备份这个采样点的数据供下次使用。所以，观测的和实际的相比，是延迟一个采样周期的。 */
    //  2017年1月20日，将控制器放到了观测器的后面。
    // * 所以，上一步电压US_P的更新也要延后了。
    // US_P(0) = US_C(0); 
    // US_P(1) = US_C(1);
    IS_P(0) = IS_C(0);
    IS_P(1) = IS_C(1);
}



/********************************************/
/* 4rd-order ESO (Copied from PMSM_observer.c)
 ********************************************/
// deleted


/********************************************
 * Collections of Flux Estimators 
 ********************************************/
void flux_observer(){

    watch.offset[0] = OFFSET_VOLTAGE_ALPHA;
    watch.offset[1] = OFFSET_VOLTAGE_BETA;

    // 如果观测到的磁链作帕克变换的时候不是用的marino.xRho的话，那么交轴磁链永远为零，转速观测校正项和xTL自适应律增益都将无效。
    // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    // Marino05需要的磁链观测是需要变换到xRho所定义的同步系下才行的！
    // Akatsu_RrId();

    #if PC_SIMULATION
        simulation_only_flux_estimator();
    #endif

    // VM_Ohtani1992();
    // VM_HuWu1998();
    // VM_HoltzQuan2002();

    // #if PC_SIMULATION==FALSE
    // EALLOW;
    // CpuTimer1.RegsAddr->TCR.bit.TRB = 1; // reset cpu timer to period value
    // CpuTimer1.RegsAddr->TCR.bit.TSS = 0; // start/restart
    // CpuTimer_Before = CpuTimer1.RegsAddr->TIM.all; // get count
    // EDIS;
    // #endif

    VM_HoltzQuan2003(); // VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();

    // #if PC_SIMULATION==FALSE
    // CpuTimer_After = CpuTimer1.RegsAddr->TIM.all; // get count
    // CpuTimer_Delta = (REAL)CpuTimer_Before - (REAL)CpuTimer_After;
    // #endif

    // VM_Harnefors2003_SCVM();
    // VM_LascuAndreescus2006();
    // VM_Stojic2015();
    // VM_ExactCompensation();
    VM_ProposedCmdErrFdkCorInFrameRho();
    // VM_ClosedLoopFluxEstimator();
    // stableFME();

    watch.psi_2[0] = FLUX_FEEDBACK_ALPHA;
    watch.psi_2[1] = FLUX_FEEDBACK_BETA;
    watch.offset_compensation[0] = OFFSET_COMPENSATION_ALPHA;
    watch.offset_compensation[1] = OFFSET_COMPENSATION_BETA;
}

/********************************************
 * Category 1 Estimators: Flux Command Correction
 ********************************************/
    /* 1. Ohtani 1992 */
    void rhf_Ohtani1992_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA + VM_OHTANI_CORRECTION_GAIN_P * ( (*CTRL).i->cmd_psi_active[0] - (x[0]-(*CTRL).motor->Lq*IS(0)) );
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + OFFSET_VOLTAGE_BETA  + VM_OHTANI_CORRECTION_GAIN_P * ( (*CTRL).i->cmd_psi_active[1] - (x[1]-(*CTRL).motor->Lq*IS(1)) );
        fx[0] = emf[0];
        fx[1] = emf[1];
    }
    void VM_Ohtani1992(){
        if(FALSE){
            #define RS (*CTRL).motor->R
            #define LSIGMA (*CTRL).motor->Lq
            #define OMEGA_SYN (*CTRL).s->omega_syn
            /* Ohtani Voltage Model in indirect oriented DQ frame + ODE1 */
            // 
            FE.ohtani.deriv_psi_DQ1[0] = (*CTRL).o->cmd_uDQ[0] - RS*(*CTRL).i->iDQ[0] + OMEGA_SYN*FE.ohtani.psi_DQ1[1] + 1*GAIN_OHTANI * ( (*CTRL).i->cmd_psi - FE.ohtani.psi_DQ2[0] );
            FE.ohtani.deriv_psi_DQ1[1] = (*CTRL).o->cmd_uDQ[1] - RS*(*CTRL).i->iDQ[1] - OMEGA_SYN*FE.ohtani.psi_DQ1[0] + 0*GAIN_OHTANI * (             0.0 - FE.ohtani.psi_DQ2[1] );
            // stator flux updates
            FE.ohtani.psi_DQ1[0] += CL_TS * (FE.ohtani.deriv_psi_DQ1[0]);
            FE.ohtani.psi_DQ1[1] += CL_TS * (FE.ohtani.deriv_psi_DQ1[1]);
            // rotor flux updates
            FE.ohtani.psi_DQ2_prev[0] = FE.ohtani.psi_DQ2[0];
            FE.ohtani.psi_DQ2_prev[1] = FE.ohtani.psi_DQ2[1];
            FE.ohtani.psi_DQ2[0] = FE.ohtani.psi_DQ1[0] - LSIGMA*(*CTRL).i->iDQ[0];
            FE.ohtani.psi_DQ2[1] = FE.ohtani.psi_DQ1[1] - LSIGMA*(*CTRL).i->iDQ[1];        
        }else{
            /* Ohtani Voltage Model in AB frame + ODE4 */
            // stator flux updates
            general_2states_rk4_solver(&rhf_Ohtani1992_Dynamics, (*CTRL).timebase, FE.ohtani.psi_1, CL_TS);
            // rotor flux updates
            FE.ohtani.psi_2[0]         = FE.ohtani.psi_1[0] - LSIGMA*IS_C(0);
            FE.ohtani.psi_2[1]         = FE.ohtani.psi_1[1] - LSIGMA*IS_C(1);
            FE.ohtani.psi_2_ampl = sqrt(FE.ohtani.psi_2[0]*FE.ohtani.psi_2[0]+FE.ohtani.psi_2[1]*FE.ohtani.psi_2[1]);
        }
    }

    /* 2. Holtz and Quan 2002 */
    void rhf_Holtz2002_Dynamics(REAL t, REAL *x, REAL *fx){

        REAL psi_2[2];
        psi_2[0] = x[0] - (*CTRL).motor->Lq*IS(0);
        psi_2[1] = x[1] - (*CTRL).motor->Lq*IS(1);
        REAL ampl_psi_2 = sqrt(psi_2[0]*psi_2[0] + psi_2[1]*psi_2[1]);
        REAL ampl_psi_2_inv=0.0;
        if(ampl_psi_2!=0){
            ampl_psi_2_inv = 1.0/ampl_psi_2;
        }
        REAL cos_rho = psi_2[0] * ampl_psi_2_inv;
        REAL sin_rho = psi_2[1] * ampl_psi_2_inv;

        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA + VM_OHTANI_CORRECTION_GAIN_P * ( (*CTRL).i->cmd_psi*cos_rho - psi_2[0] );
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + OFFSET_VOLTAGE_BETA  + VM_OHTANI_CORRECTION_GAIN_P * ( (*CTRL).i->cmd_psi*sin_rho - psi_2[1] );
        fx[0] = emf[0];
        fx[1] = emf[1];
    }
    void VM_HoltzQuan2002(){
        /* Ohtani Voltage Model in AB frame + ODE4 */
        // stator flux updates
        general_2states_rk4_solver(&rhf_Holtz2002_Dynamics, (*CTRL).timebase, FE.holtz02.psi_1, CL_TS);
        // rotor flux updates
        FE.holtz02.psi_2[0]         = FE.holtz02.psi_1[0] - LSIGMA*IS_C(0);
        FE.holtz02.psi_2[1]         = FE.holtz02.psi_1[1] - LSIGMA*IS_C(1);
        FE.holtz02.psi_2_ampl = sqrt(FE.holtz02.psi_2[0]*FE.holtz02.psi_2[0]+FE.holtz02.psi_2[1]*FE.holtz02.psi_2[1]);
    }

    /* 3. Lascu and Andreescus 2006 TODO 非常好奇Lascu的方法会怎样！和我们的xRho校正项对比！ */
    void init_LascuAndreescus2006(){
        int ind;
        FE.lascu.x[0] = d_sim.init.KE;
        FE.lascu.x[1] = 0;
        FE.lascu.x[2] = 0;
        FE.lascu.x[3] = 0;
        for (ind=0;ind<2;++ind) {
        FE.lascu.psi_1[ind] = (ind == 0) ? d_sim.init.KE : 0;
        FE.lascu.psi_2[ind] = (ind == 0) ? d_sim.init.KE : 0;
        FE.lascu.correction_integral_term[ind] = 0;
        FE.lascu.u_offset[ind] = 0;
    }
    }
    void rhf_LascuAndreescus2006_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL rotor_flux[2];
        rotor_flux[0] = x[0]-(*CTRL).motor->Lq*IS(0);
        rotor_flux[1] = x[1]-(*CTRL).motor->Lq*IS(1);

        REAL ampl     = sqrt(rotor_flux[0]*rotor_flux[0] + rotor_flux[1]*rotor_flux[1]);
        REAL ampl_inv = 0;
        if(ampl!=0){
           ampl_inv = 1.0/ampl;
        }
        REAL rotor_flux_error[2]={0,0};
        rotor_flux_error[0] = ( (*CTRL).i->cmd_psi - ampl ) * rotor_flux[0] * ampl_inv;
        rotor_flux_error[1] = ( (*CTRL).i->cmd_psi - ampl ) * rotor_flux[1] * ampl_inv;

        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + 1*OFFSET_VOLTAGE_ALPHA \
            /*P*/+ 1*VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[0] \
            /*I*/+ 1*x[2];
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + 1*OFFSET_VOLTAGE_BETA  \
            /*P*/+ 1*VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[1] \
            /*I*/+ 1*x[3];
        fx[0] = emf[0];
        fx[1] = emf[1];
        fx[2] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[0];
        fx[3] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[1];
    }
    void VM_LascuAndreescus2006(){
        /* Proposed VM based Flux Command Error Feedback Correction in Controller Frame (xRho), implemented in AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_LascuAndreescus2006_Dynamics, (*CTRL).timebase, FE.lascu.x, CL_TS);
        // Unpack x
        FE.lascu.psi_1[0]                    = FE.lascu.x[0];
        FE.lascu.psi_1[1]                    = FE.lascu.x[1];
        FE.lascu.correction_integral_term[0] = FE.lascu.x[2];
        FE.lascu.correction_integral_term[1] = FE.lascu.x[3];
        FE.lascu.u_offset[0] = FE.lascu.correction_integral_term[0];
        FE.lascu.u_offset[1] = FE.lascu.correction_integral_term[1];
        // rotor flux updates
        FE.lascu.psi_2[0] = FE.lascu.psi_1[0] - LSIGMA*IS_C(0);
        FE.lascu.psi_2[1] = FE.lascu.psi_1[1] - LSIGMA*IS_C(1);
        FE.lascu.psi_2_ampl = sqrt(FE.lascu.psi_2[0]*FE.lascu.psi_2[0]+FE.lascu.psi_2[1]*FE.lascu.psi_2[1]);
        FE.lascu.theta_d = atan2(FE.lascu.psi_2[1], FE.lascu.psi_2[0]);
        FE.lascu.theta_e = angle_diff(FE.lascu.theta_d, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
    }

    /* 4. B. Proposed Flux Command Error feedback PI Correction in Controller Frame (xRho) */
    void rhf_CmdErrFdkCorInFrameRho_Dynamics(REAL t, REAL *x, REAL *fx){

        REAL rotor_flux_error[2];
        rotor_flux_error[0] = ( (*CTRL).i->cmd_psi_active[0] - (x[0]-(*CTRL).motor->Lq*IS(0)) );
        rotor_flux_error[1] = ( (*CTRL).i->cmd_psi_active[1] - (x[1]-(*CTRL).motor->Lq*IS(1)) );

        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA \
            /*P*/+ VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[0] \
            /*I*/+ x[2];
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + OFFSET_VOLTAGE_BETA  \
            /*P*/+ VM_PROPOSED_PI_CORRECTION_GAIN_P * rotor_flux_error[1] \
            /*I*/+ x[3];
        fx[0] = emf[0];
        fx[1] = emf[1];
        fx[2] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[0];
        fx[3] = VM_PROPOSED_PI_CORRECTION_GAIN_I * rotor_flux_error[1];
    }
    void VM_ProposedCmdErrFdkCorInFrameRho(){
        /* Proposed VM based Flux Command Error Feedback Correction in Controller Frame (xRho), implemented in AB frame + ODE4 */
        // stator flux and integral states update
        general_4states_rk4_solver(&rhf_CmdErrFdkCorInFrameRho_Dynamics, (*CTRL).timebase, FE.picorr.x, CL_TS);
        // Unpack x
        FE.picorr.psi_1[0]                    = FE.picorr.x[0];
        FE.picorr.psi_1[1]                    = FE.picorr.x[1];
        FE.picorr.correction_integral_term[0] = FE.picorr.x[2];
        FE.picorr.correction_integral_term[1] = FE.picorr.x[3];
        FE.picorr.u_offset[0] = FE.picorr.correction_integral_term[0];
        FE.picorr.u_offset[1] = FE.picorr.correction_integral_term[1];
        // rotor flux updates
        FE.picorr.psi_2[0] = FE.picorr.psi_1[0] - LSIGMA*IS_C(0);
        FE.picorr.psi_2[1] = FE.picorr.psi_1[1] - LSIGMA*IS_C(1);
        FE.picorr.psi_2_ampl = sqrt(FE.picorr.psi_2[0]*FE.picorr.psi_2[0]+FE.picorr.psi_2[1]*FE.picorr.psi_2[1]);

        FE.picorr.theta_d = atan2(FE.picorr.psi_2[1], FE.picorr.psi_2[0]);
        FE.picorr.cosT = cos(FE.picorr.theta_d);
        FE.picorr.sinT = sin(FE.picorr.theta_d);
    }

/********************************************
 * Category 2 Estimators: Flux Command Free
 ********************************************/

    /* 5. Hu and Wu 1998 */
    void VM_HuWu1998(){
    }

    /* 6. Stojic 2015 */
    void VM_Stojic2015(){
    }

    /* 7. SCVM - Harnefors 2003 (TODO: Try mu=-1 ????????) */
    void rhf_Harnefors2003_Dynamics(REAL t, REAL *x, REAL *fx){
        #define OMEGA_SYN (*CTRL).s->omega_syn
        #define LAMBDA FE.harnefors.lambda

        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + 0*OFFSET_VOLTAGE_ALPHA;
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + 0*OFFSET_VOLTAGE_BETA ;
        fx[0] = - LAMBDA * fabsf(OMEGA_SYN) * x[0] + emf[0] + LAMBDA * sign(OMEGA_SYN) * emf[1];
        fx[1] = - LAMBDA * fabsf(OMEGA_SYN) * x[1] + emf[1] + LAMBDA * sign(OMEGA_SYN) *-emf[0];
    }
    void VM_Harnefors2003_SCVM(){
        #define LSIGMA (*CTRL).motor->Lq

        // stator flux updates
        general_2states_rk4_solver(&rhf_Harnefors2003_Dynamics, (*CTRL).timebase, FE.harnefors.psi_1, CL_TS);
        // rotor flux updates
        FE.harnefors.psi_2[0]         = FE.harnefors.psi_1[0] - LSIGMA*IS_C(0);
        FE.harnefors.psi_2[1]         = FE.harnefors.psi_1[1] - LSIGMA*IS_C(1);
        FE.harnefors.psi_2_ampl = sqrt(FE.harnefors.psi_2[0]*FE.harnefors.psi_2[0]+FE.harnefors.psi_2[1]*FE.harnefors.psi_2[1]);
    }

    /* 8. C. Output Error Closed-loop Flux Estimator */
    void rhf_ClosedLoopFluxEstimator_Dynamics(REAL t, REAL *x, REAL *fx){
        // x[0], x[1]: stator flux in ab frame
        // x[2]: \psi_{d\mu}
        // x[3], x[4]: integral action compensating offset voltage

        REAL psi_2[2];
        psi_2[0] = x[0] - (*CTRL).motor->Lq*IS(0);
        psi_2[1] = x[1] - (*CTRL).motor->Lq*IS(1);
        REAL ampl_psi_2 = sqrt(psi_2[0]*psi_2[0] + psi_2[1]*psi_2[1]);
        REAL ampl_psi_2_inv=0.0;
        if(ampl_psi_2!=0){
            ampl_psi_2_inv = 1.0/ampl_psi_2;
        }

        REAL cos_rho = psi_2[0] * ampl_psi_2_inv;
        REAL sin_rho = psi_2[1] * ampl_psi_2_inv;

        /* Variant with command rho */
        // REAL cos_rho = (*CTRL).s->cosT;
        // REAL sin_rho = (*CTRL).s->sinT;

        REAL current_estimate[2];
        current_estimate[0] = (*CTRL).motor->Lq_inv * (x[0] - x[2]*cos_rho);
        current_estimate[1] = (*CTRL).motor->Lq_inv * (x[1] - x[2]*sin_rho);

        REAL current_error[2];
        current_error[0] = IS(0) - current_estimate[0];
        current_error[1] = IS(1) - current_estimate[1];

        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA \
            /*P*/+ OUTPUT_ERROR_CLEST_GAIN_KP * current_error[0] \
            /*I*/+ x[3];
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + OFFSET_VOLTAGE_BETA  \
            /*P*/+ OUTPUT_ERROR_CLEST_GAIN_KP * current_error[1] \
            /*I*/+ x[4];
        fx[0] = emf[0];
        fx[1] = emf[1];
        /* Lascu CLE */
        // REAL psi_ds = x[0]*cos_rho + x[1]*sin_rho;
        // fx[2] = (*CTRL).motor->Rreq*(*CTRL).motor->Lq_inv*psi_ds - (*CTRL).motor->alpha*((*CTRL).motor->Lmu+(*CTRL).motor->Lq)*(*CTRL).motor->Lq_inv*x[2]\
        //       + OUTPUT_ERROR_CLEST_GAIN_KCM*(cos_rho*current_error[0] + sin_rho*current_error[1]);
        /* MRAS */
        REAL ids = IS(0)*cos_rho + IS(1)*sin_rho;
        fx[2] = (*CTRL).motor->Rreq*ids - (*CTRL).motor->alpha*x[2]\
              + OUTPUT_ERROR_CLEST_GAIN_KCM*(cos_rho*current_error[0] + sin_rho*current_error[1]);
        fx[3] = OUTPUT_ERROR_CLEST_GAIN_KI * current_error[0];
        fx[4] = OUTPUT_ERROR_CLEST_GAIN_KI * current_error[1];
    }
    void VM_ClosedLoopFluxEstimator(){
        /* Proposed closed loop estimator AB frame + ODE4 */
        // stator flux and integral states update
        general_5states_rk4_solver(&rhf_ClosedLoopFluxEstimator_Dynamics, (*CTRL).timebase, FE.clest.x, CL_TS);
        // Unpack x
        FE.clest.psi_1[0]                    = FE.clest.x[0];
        FE.clest.psi_1[1]                    = FE.clest.x[1];
        FE.clest.psi_dmu                     = FE.clest.x[2];
        FE.clest.correction_integral_term[0] = FE.clest.x[3];
        FE.clest.correction_integral_term[1] = FE.clest.x[4];
        FE.clest.u_offset[0] = FE.clest.correction_integral_term[0];
        FE.clest.u_offset[1] = FE.clest.correction_integral_term[1];
        // rotor flux updates
        FE.clest.psi_2[0] = FE.clest.psi_1[0] - (*CTRL).motor->Lq*IS_C(0);
        FE.clest.psi_2[1] = FE.clest.psi_1[1] - (*CTRL).motor->Lq*IS_C(1);    
        FE.clest.psi_2_ampl = sqrt(FE.clest.psi_2[0]*FE.clest.psi_2[0]+FE.clest.psi_2[1]*FE.clest.psi_2[1]);
    }

    // CM in synchronous dq frame
        // void rhf_stableFME_Dynamics(REAL t, REAL *x, REAL *fx){
        //     #define OMEGA_SYN (*CTRL).s->omega_syn
        //     #define LAMBDA FE.harnefors.lambda

        //     REAL emf[2];
        //     emf[0] = US(0) - (*CTRL).motor->R*IS(0) + 0*OFFSET_VOLTAGE_ALPHA;
        //     emf[1] = US(1) - (*CTRL).motor->R*IS(1) + 0*OFFSET_VOLTAGE_BETA ;
        //     fx[0] = - LAMBDA * fabsf(OMEGA_SYN) * x[0] + emf[0] + LAMBDA * sign(OMEGA_SYN) * emf[1];
        //     fx[1] = - LAMBDA * fabsf(OMEGA_SYN) * x[1] + emf[1] + LAMBDA * sign(OMEGA_SYN) *-emf[0];
        // }
        // void stableFME(){
            //FE.fme.psi_DQ2[0] += CL_TS * (-(*CTRL).motor->alpha*FE.fme.psi_DQ2[0] + (*CTRL).motor->Rreq*(*CTRL).i->iDQ[0]);
        // }

/********************************************
 * Category 3 Estimators: Saturation
 ********************************************/
    /* 9. Holtz and Quan 2003 (LPF) */
    void VM_HoltzQuan2003(){
        VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit();    
    }

    /* 10. Holtz and Quan 2003 (Integrator) */


    /* 11. Exact Offset Compensation */

    /* 12. Adaptive Limit */

    /* 13. Saturation Time Difference */


    /* 14. A. [Top and Butt] Exact Compensation Method (Proposed) */
    // #define GAIN_OHTANI (0.5) // must be zero for estimating u_offset by top change
    void rhf_ExactCompensation_Dynamics(REAL t, REAL *x, REAL *fx){
        REAL emf[2];
        emf[0] = US(0) - (*CTRL).motor->R*IS(0) + OFFSET_VOLTAGE_ALPHA - 0*K_VM*(x[0]-FE.fme.psi_DQ1[0]*(*CTRL).s->cosT) - 0*FE.exact.u_offset_estimated_by_integration_correction[0] - 0*FE.exact.u_offset_estimated_by_top_minus_butt[0] - 0*FE.exact.offset_voltage_compensation[0] - 0*FE.exact.u_offset_estimated_by_psi_2_top_change[0] - 0*FE.exact.u_offset[0] + 0*VM_OHTANI_CORRECTION_GAIN_P * ( (*CTRL).i->cmd_psi_active[0] - (x[0]-(*CTRL).motor->Lq*IS(0)) ) + 0*FE.exact.correction_integral_term[0];
        emf[1] = US(1) - (*CTRL).motor->R*IS(1) + OFFSET_VOLTAGE_BETA  - 0*K_VM*(x[1]-FE.fme.psi_DQ1[0]*(*CTRL).s->sinT) - 0*FE.exact.u_offset_estimated_by_integration_correction[1] - 0*FE.exact.u_offset_estimated_by_top_minus_butt[1] - 0*FE.exact.offset_voltage_compensation[1] - 0*FE.exact.u_offset_estimated_by_psi_2_top_change[1] - 0*FE.exact.u_offset[1] + 0*VM_OHTANI_CORRECTION_GAIN_P * ( (*CTRL).i->cmd_psi_active[1] - (x[1]-(*CTRL).motor->Lq*IS(1)) ) + 0*FE.exact.correction_integral_term[1];
        fx[0] = emf[0];
        fx[1] = emf[1];
    }
    void zero_crossing_method(int i){
        // ZERO cannot be changed during the loop
        #define ZERO FE.exact.psi_2_last_zero_level_inLoopTheSame[i]
        ZERO = FE.exact.psi_2_last_zero_level[i];

        #if PC_SIMULATION
        if((int)((*CTRL).timebase/CL_TS)%10000==0){
            printf("%g: %g, %g\n", (*CTRL).timebase, FE.exact.psi_2_prev[i], FE.exact.psi_2[i]);
        }
        #endif

        /* 负负负负负负负负负负负负负负负负负负负，寻找最小值 */
        // 进入磁链负半周？
        if(FE.exact.flag_Neg_stageA[i] == TRUE){
            /* 进入【磁链负半周】！ Stage A */
            // 二次检查，磁链已经是负的了？
            if(FE.exact.psi_2_prev[i]<ZERO && FE.exact.psi_2[i]<ZERO){
                /* 进入【磁链负半周】！ Stage B */
                if(FE.exact.flag_Neg_stageB[i] == FALSE){
                    //MUTE printf("[%d] Neg Stage A Enter %g\n", i, (*CTRL).timebase);
                    // 这边只做一次
                    FE.exact.flag_Neg_stageB[i] = TRUE; 
                    // 计算相邻两次过零间隔时间：Delta_t
                    FE.exact.Delta_t_NegMinusPos[i] = FE.exact.time_Neg[i] - FE.exact.time_Pos[i];
                    // 初始化对立面的flag
                    FE.exact.flag_Pos_stageA[i] = FALSE;
                    FE.exact.flag_Pos_stageB[i] = FALSE;
                    // 初始化最小值
                    FE.exact.psi_2_min[i] = 0.0;
                }
                // 寻找磁链最小值
                if(FE.exact.psi_2[i] < FE.exact.psi_2_min[i]) {FE.exact.psi_2_min[i] = FE.exact.psi_2[i];}
            }else{
                /* 结算开始 */
                //MUTE printf("[%d] Neg Quit %g\n", i, (*CTRL).timebase);
                // 退出【磁链负半周】！
                // 初始化flag
                FE.exact.flag_Neg_stageA[i] = FALSE;
                FE.exact.flag_Neg_stageB[i] = FALSE;
                // 退出的时候，顺便计算一次李萨如图圆心横/纵坐标
                FE.exact.psi_2_maxmin_difference[i] = (FE.exact.psi_2_max[i]) + (FE.exact.psi_2_min[i]);
                FE.exact.psi_2_moved_distance_in_LissajousPlot[i] = 0.5*FE.exact.psi_2_maxmin_difference[i] - ZERO;
                if( FE.exact.Delta_t_PosMinusNeg[i] !=0 ){
                    // 这里只是近似，真正的时间应该是最大值所对应的时间和最小值对应的时间的差，而不是过零变负和过零变正之间的时间差！
                    REAL time_of_offset_voltage_contributing_to_the_difference_between_max_and_min = FE.exact.Delta_t_PosMinusNeg[i];
                    FE.exact.offset_voltage_by_zero_crossing[i] = FE.exact.psi_2_maxmin_difference[i] / time_of_offset_voltage_contributing_to_the_difference_between_max_and_min;
                }
                FE.exact.psi_2_last_zero_level[i] = 0.5*(FE.exact.psi_2_max[i] + FE.exact.psi_2_min[i]);
            }
        }
        /* 磁链正切负（必须放在后面执行！否则已进入【磁链负半周】判断后，会因为当前和上一步异号马上退出） */
        // 发现磁链由正变负的时刻
        if(FE.exact.psi_2_prev[i]>ZERO && FE.exact.psi_2[i]<ZERO){
            FE.exact.flag_Neg_stageA[i] = TRUE;
            FE.exact.time_Neg[i] = (*CTRL).timebase;
            //MUTE printf("[%d] Neg Crossing %g\n", i, (*CTRL).timebase);
        }

        /* 正正正正正正正正正正正正正正正正正正正，寻找最大值 */
        // 进入【磁链正半周】？
        if(FE.exact.flag_Pos_stageA[i] == TRUE){
            /* 进入【磁链正半周】！ Stage A */
            // 二次检查，磁链已经是正的了？
            if(FE.exact.psi_2_prev[i]>ZERO && FE.exact.psi_2[i]>ZERO){
                /* 进入【磁链正半周】！ Stage B */
                if(FE.exact.flag_Pos_stageB[i] == FALSE){
                    //MUTE printf("[%d] Pos Stage A Enter %g\n", i, (*CTRL).timebase);
                    // 这边只做一次
                    FE.exact.flag_Pos_stageB[i] = TRUE; 
                    // 计算相邻两次过零间隔时间：Delta_t
                    FE.exact.Delta_t_PosMinusNeg[i] = FE.exact.time_Pos[i] - FE.exact.time_Neg[i];
                    // 初始化对立面的flag
                    FE.exact.flag_Neg_stageA[i] = FALSE;
                    FE.exact.flag_Neg_stageB[i] = FALSE;
                    // 初始化最大值
                    FE.exact.psi_2_max[i] = 0.0;
                }
                // 寻找磁链最大值
                if(FE.exact.psi_2[i] > FE.exact.psi_2_max[i]){FE.exact.psi_2_max[i] = FE.exact.psi_2[i];}
            }else{
                /* 结算开始 */
                //MUTE printf("[%d] Pos Quit %g\n", i, (*CTRL).timebase);
                // 退出【磁链正半周】！
                // 初始化flag
                FE.exact.flag_Pos_stageA[i] = FALSE;
                FE.exact.flag_Pos_stageB[i] = FALSE;
                // 退出的时候，顺便计算一次李萨如图圆心横/纵坐标
                FE.exact.psi_2_maxmin_difference[i] = (FE.exact.psi_2_max[i]) + (FE.exact.psi_2_min[i]);
                FE.exact.psi_2_moved_distance_in_LissajousPlot[i] = 0.5*FE.exact.psi_2_maxmin_difference[i] - ZERO;
                if( FE.exact.Delta_t_NegMinusPos[i] !=0 ){
                    // 这里只是近似，真正的时间应该是最大值所对应的时间和最小值对应的时间的差，而不是过零变负和过零变正之间的时间差！
                    REAL time_of_offset_voltage_contributing_to_the_difference_between_max_and_min = FE.exact.Delta_t_NegMinusPos[i];
                    FE.exact.offset_voltage_by_zero_crossing[i] = FE.exact.psi_2_maxmin_difference[i] / time_of_offset_voltage_contributing_to_the_difference_between_max_and_min;                    
                }
                FE.exact.psi_2_last_zero_level[i] = 0.5*(FE.exact.psi_2_max[i] + FE.exact.psi_2_min[i]);                
            }
        }
        /* 磁链负切正（必须放在后面执行！否则已进入【磁链正半周】判断后，会因为当前和上一步异号马上退出） */
        // 发现磁链由负变正的时刻
        if(FE.exact.psi_2_prev[i]<ZERO && FE.exact.psi_2[i]>ZERO){
            FE.exact.flag_Pos_stageA[i] = TRUE;
            FE.exact.time_Pos[i] = (*CTRL).timebase;
            #if PC_SIMULATION
            printf("[%d] Pos Crossing %g\n", i, (*CTRL).timebase);
            #endif
        }

        #if PC_SIMULATION
        // DEBUG
        if(FE.exact.psi_2_prev[i]<ZERO && FE.exact.psi_2[i]>ZERO){
            printf("!!!!!!!!!!!!!! %g\n", (*CTRL).timebase);
        }
        #endif
    }
    void VM_ExactCompensation(){
        #define RS (*CTRL).motor->R
        #define LSIGMA (*CTRL).motor->Lq
        #define OMEGA_SYN (*CTRL).s->omega_syn

        if(FALSE){
            /* Ohtani Voltage Model in indirect oriented DQ frame + ODE1 */
            // 
            REAL deriv_psi_DQ1[2];
            deriv_psi_DQ1[0] = (*CTRL).o->cmd_uDQ[0] - RS*(*CTRL).i->iDQ[0] + OMEGA_SYN*FE.exact.psi_DQ1[1] + GAIN_OHTANI * ( (*CTRL).i->cmd_psi - FE.exact.psi_DQ2[0] );
            deriv_psi_DQ1[1] = (*CTRL).o->cmd_uDQ[1] - RS*(*CTRL).i->iDQ[1] - OMEGA_SYN*FE.exact.psi_DQ1[0] + GAIN_OHTANI * (             0.0 - FE.exact.psi_DQ2[1] );
            // stator flux updates
            FE.exact.psi_DQ1[0] += CL_TS * (deriv_psi_DQ1[0]);
            FE.exact.psi_DQ1[1] += CL_TS * (deriv_psi_DQ1[1]);
            // rotor flux updates
            FE.exact.psi_DQ2_prev[0] = FE.exact.psi_DQ2[0];
            FE.exact.psi_DQ2_prev[1] = FE.exact.psi_DQ2[1];
            FE.exact.psi_DQ2[0] = FE.exact.psi_DQ1[0] - LSIGMA*(*CTRL).i->iDQ[0];
            FE.exact.psi_DQ2[1] = FE.exact.psi_DQ1[1] - LSIGMA*(*CTRL).i->iDQ[1];        
        }else{
            /* Ohtani Voltage Model in AB frame + ODE4 */

            /* Low-Pass Filter */
            // FE.exact.filtered_compensation[0] = _lpf(0.5*(FE.exact.psi_2_last_top[0]+FE.exact.psi_2_last_butt[0]), FE.exact.filtered_compensation[0], 15);
            // FE.exact.filtered_compensation[1] = _lpf(0.5*(FE.exact.psi_2_last_top[1]+FE.exact.psi_2_last_butt[1]), FE.exact.filtered_compensation[1], 15);
            // FE.exact.filtered_compensation2[0] = _lpf(0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] ), FE.exact.filtered_compensation2[0], 15);
            // FE.exact.filtered_compensation2[1] = _lpf(0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] ), FE.exact.filtered_compensation2[1], 15);
            /* No Filter */
            FE.exact.filtered_compensation[0]  = 0.5*(FE.exact.psi_2_last_top[0]+FE.exact.psi_2_last_butt[0]);
            FE.exact.filtered_compensation[1]  = 0.5*(FE.exact.psi_2_last_top[1]+FE.exact.psi_2_last_butt[1]);
            FE.exact.filtered_compensation2[0] = 0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] );
            FE.exact.filtered_compensation2[1] = 0.5*( FE.exact.psi_2_output_last_top[0]+FE.exact.psi_2_output_last_butt[0] );

            /* 补偿 psi_offset 没用！ */
            // if(FE.exact.bool_compensate_psi_offset[0]){
            //     FE.exact.bool_compensate_psi_offset[0] = FALSE;
            //     FE.exact.psi_1[0] -= FE.exact.filtered_compensation[0];
            // }
            // if(FE.exact.bool_compensate_psi_offset[1]){
            //     FE.exact.bool_compensate_psi_offset[1] = FALSE;
            //     FE.exact.psi_1[1] -= FE.exact.filtered_compensation[1];
            // }

            // FE.exact.psi_1[0] -= FE.exact.filtered_compensation[0]; // 只需要在波峰和波谷的时候补一次就可以了
            // FE.exact.psi_1[1] -= FE.exact.filtered_compensation[1]; // 只需要在波峰和波谷的时候补一次就可以了

            // stator flux updates
            // #define VM_CORRECTION_KI (2.5)
            // FE.exact.correction_integral_term[0] += CL_TS * VM_CORRECTION_KI * ( (*CTRL).i->cmd_psi_active[0] - FE.exact.psi_2[0] );
            // FE.exact.correction_integral_term[1] += CL_TS * VM_CORRECTION_KI * ( (*CTRL).i->cmd_psi_active[1] - FE.exact.psi_2[1] );
            general_2states_rk4_solver(&rhf_ExactCompensation_Dynamics, (*CTRL).timebase, FE.exact.psi_1, CL_TS);
            // rotor flux updates
            FE.exact.psi_2_pprev[0]   = FE.exact.psi_2_prev[0];
            FE.exact.psi_2_pprev[1]   = FE.exact.psi_2_prev[1];
            FE.exact.psi_2_prev[0]    = FE.exact.psi_2[0];
            FE.exact.psi_2_prev[1]    = FE.exact.psi_2[1];
            FE.exact.psi_2[0]         = FE.exact.psi_1[0] - LSIGMA*IS_C(0);
            FE.exact.psi_2[1]         = FE.exact.psi_1[1] - LSIGMA*IS_C(1);
            FE.exact.psi_2_output[0]  = FE.exact.psi_2[0]        - 1*FE.exact.filtered_compensation[0];
            FE.exact.psi_2_output[1]  = FE.exact.psi_2[1]        - 1*FE.exact.filtered_compensation[1];
            FE.exact.psi_2_output2[0] = FE.exact.psi_2_output[0] - 1*FE.exact.filtered_compensation2[0];
            FE.exact.psi_2_output2[1] = FE.exact.psi_2_output[1] - 1*FE.exact.filtered_compensation2[1];

            FE.exact.psi_2_real_output[0] = 0.5*(FE.exact.psi_2_output[0] + FE.exact.psi_2_output2[0]);
            FE.exact.psi_2_real_output[1] = 0.5*(FE.exact.psi_2_output[1] + FE.exact.psi_2_output2[1]);
            FE.exact.psi_2_ampl = sqrt(FE.exact.psi_2_real_output[0]*FE.exact.psi_2_real_output[0]+FE.exact.psi_2_real_output[1]*FE.exact.psi_2_real_output[1]);
        }

        // #define INTEGRAL_GAIN_OHTANI 1
        // FE.exact.u_offset_estimated_by_Lascu_integral_term[0] += CL_TS * INTEGRAL_GAIN_OHTANI * ( (*CTRL).i->cmd_psi_active[0] - (FE.exact.psi_2[0]-(*CTRL).motor->Lq*IS_C(0)) );
        // FE.exact.u_offset_estimated_by_Lascu_integral_term[1] += CL_TS * INTEGRAL_GAIN_OHTANI * ( (*CTRL).i->cmd_psi_active[1] - (FE.exact.psi_2[1]-(*CTRL).motor->Lq*IS_C(1)) );

        int i;
        /* αβ */
        for(i=0; i<2; ++i){

            /* TOP */
            if (   FE.exact.psi_2_prev[i] - FE.exact.psi_2_pprev[i] >= 0 \
                && FE.exact.psi_2_prev[i] - FE.exact.psi_2      [i] >= 0 \
                && FE.exact.psi_2_pprev[i] > 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2_prev[i]  > 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2[i]       > 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) ){
                /* 发现最高点，而且不能简单根据正负来判断，一旦磁链掉下去了，这判断就无法成立了 */
                    // 根据求和结果计算 u_offset
                    // REAL denumerator = 0.5*(1+FE.exact.top2top_count[i]) * CL_TS;
                    // FE.exact.u_offset_estimated_by_psi_2_sumup[i] = ( FE.exact.top2top_sum[i] / FE.exact.top2top_count[i] - FE.exact.psi_2_offset[i] ) / denumerator;
                    // FE.exact.psi_2_offset[i] = FE.exact.top2top_sum[i] / FE.exact.top2top_count[i];
                    // FE.exact.top2top_sum[i] = 0.0;
                if( FE.exact.top2top_count[i]> 10 ){ // 防止静止时疯狂判断成立，因为pprev==prev==curr // 10 / TS = 1000 Hz
                    FE.exact.top2top_count[i] = 0;

                    // 磁链最大值变化信息
                    FE.exact.flag_top2top[i] = TRUE;
                    FE.exact.change_in_psi_2_top[i] = FE.exact.psi_2[i] - FE.exact.psi_2_last_top[i];

                    { /* 补偿 psi_offset */
                        // FE.exact.psi_1[i]      -= FE.exact.change_in_psi_2_top[i];
                        // FE.exact.psi_2_pprev[i]-= FE.exact.change_in_psi_2_top[i];
                        // FE.exact.psi_2_prev[i] -= FE.exact.change_in_psi_2_top[i];
                        // FE.exact.psi_2[i]      -= FE.exact.change_in_psi_2_top[i];
                    }
                        // printf("TOP {%d} [%g] %g - %g\n", i, (*CTRL).timebase, FE.exact.psi_2[i], FE.exact.psi_2_last_top[i]);
                    FE.exact.psi_2_last_top[i]         = FE.exact.psi_2[i];
                    FE.exact.psi_2_output_last_top[i]  = FE.exact.psi_2_output[i];
                    FE.exact.psi_2_output2_last_top[i] = FE.exact.psi_2_output2[i];
                    // FE.exact.u_offset_estimated_by_top_minus_butt[i] += 190 * CL_TS * 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]);

                    FE.exact.current_Lissa_move[i] = 0.5*( FE.exact.psi_2_output_last_top[i]+FE.exact.psi_2_output_last_butt[i] );
                    FE.exact.u_offset_error[i] = (FE.exact.current_Lissa_move[i] - FE.exact.last_Lissa_move[i]) / ((*CTRL).timebase - FE.exact.last_time_reaching_butt[i]);
                    FE.exact.last_Lissa_move[i] = FE.exact.current_Lissa_move[i];

                    FE.exact.u_offset_estimated_by_integration_correction[i] = FE.exact.u_offset_error[i];

                    FE.exact.bool_compensate_psi_offset[i] = TRUE;

                    // 时间信息
                    FE.exact.Delta_time_top2top[i] = (*CTRL).timebase - FE.exact.last_time_reaching_top[i];
                    FE.exact.last_time_reaching_top[i] = (*CTRL).timebase;
                    // 根据磁链最大值幅值变化来计算 u_offset
                    if(FE.exact.first_time_enter_top[i]){FE.exact.first_time_enter_top[i] = FALSE;}
                    else{
                        FE.exact.u_offset_estimated_by_psi_2_top_change[i] = FE.exact.change_in_psi_2_top[i] / FE.exact.Delta_time_top2top[i];
                        FE.exact.u_offset[i] = 0.5*(FE.exact.u_offset_estimated_by_psi_2_top_change[i] + FE.exact.u_offset_estimated_by_psi_2_butt_change[i]);
                    }

                }else{
                    FE.exact.top2top_count[i] = 0;
                }
            }else{
                FE.exact.flag_top2top[i] = FALSE;
            }
            // sum it up
            // FE.exact.top2top_sum[i] += FE.exact.psi_2[i];
            FE.exact.top2top_count[i] += 1;

            /* BUTT */
            if (   FE.exact.psi_2_prev[i] - FE.exact.psi_2_pprev[i] <= 0 \
                && FE.exact.psi_2_prev[i] - FE.exact.psi_2      [i] <= 0 \
                && FE.exact.psi_2_pprev[i] < 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2_prev[i]  < 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) \
                && FE.exact.psi_2[i]       < 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]) ){
                /* 发现最低点，而且不能简单根据正负来判断，一旦磁链飘上去了，这判断就无法成立了！ */

                if( FE.exact.butt2butt_count[i]> 10 ){ // 10 / TS = 1000 Hz
                    FE.exact.butt2butt_count[i] = 0;

                    // 磁链最小值变化信息
                    FE.exact.flag_butt2butt[i] = -1*TRUE;
                    FE.exact.change_in_psi_2_butt[i] = FE.exact.psi_2[i] - FE.exact.psi_2_last_butt[i];

                    { /* 补偿 psi_offset */
                        // FE.exact.psi_1[i]      -= FE.exact.change_in_psi_2_butt[i];
                        // FE.exact.psi_2_pprev[i]-= FE.exact.change_in_psi_2_butt[i];
                        // FE.exact.psi_2_prev[i] -= FE.exact.change_in_psi_2_butt[i];
                        // FE.exact.psi_2[i]      -= FE.exact.change_in_psi_2_butt[i];
                    }

                    FE.exact.psi_2_last_butt[i]        = FE.exact.psi_2[i];
                    FE.exact.psi_2_output_last_butt[i] = FE.exact.psi_2_output[i];
                    FE.exact.psi_2_output2_last_butt[i] = FE.exact.psi_2_output2[i];
                    // FE.exact.u_offset_estimated_by_top_minus_butt[i] += 190 * CL_TS * 0.5*(FE.exact.psi_2_last_top[i]+FE.exact.psi_2_last_butt[i]);

                    FE.exact.current_Lissa_move[i] = 0.5*( FE.exact.psi_2_output_last_top[i]+FE.exact.psi_2_output_last_butt[i] );
                    FE.exact.u_offset_error[i] = (FE.exact.current_Lissa_move[i] - FE.exact.last_Lissa_move[i] ) / ((*CTRL).timebase - FE.exact.last_time_reaching_top[i]);
                    FE.exact.last_Lissa_move[i] = FE.exact.current_Lissa_move[i];

                    FE.exact.u_offset_estimated_by_integration_correction[i] = FE.exact.u_offset_error[i];

                    FE.exact.bool_compensate_psi_offset[i] = TRUE;

                    // 时间信息
                    FE.exact.Delta_time_butt2butt[i] = (*CTRL).timebase - FE.exact.last_time_reaching_butt[i];
                    FE.exact.last_time_reaching_butt[i] = (*CTRL).timebase;
                    // 根据磁链最大值幅值变化来计算 u_offset
                    if(FE.exact.first_time_enter_butt[i]){FE.exact.first_time_enter_butt[i] = FALSE;}
                    else{
                        FE.exact.u_offset_estimated_by_psi_2_butt_change[i] = FE.exact.change_in_psi_2_butt[i] / FE.exact.Delta_time_butt2butt[i];
                        FE.exact.u_offset[i] = 0.5*(FE.exact.u_offset_estimated_by_psi_2_top_change[i] + FE.exact.u_offset_estimated_by_psi_2_butt_change[i]);
                    }
                }else{
                    FE.exact.butt2butt_count[i] = 0;                
                }
            }else{
                FE.exact.flag_butt2butt[i] = FALSE;
            }
            // sum it up
            // FE.exact.butt2butt_sum[i] += FE.exact.psi_2[i];
            FE.exact.butt2butt_count[i] += 1;


            // zero_crossing_method(i);
        }
        // 开始。啥也没有，咱先找最大值，要靠递增到递减来确定，
        // 攥着最大值，然后要找最小值，从递减到递增来确定。
        // 有了最大值和最小值以后，可以做一次u_offset的计算和补偿。
        // 然后，攥着最小值，去找下一个最大值，有了最小值和下一个最大值以后，又可以做一次u_offset的计算和补偿。
    }



/* Holtz 2003 implemented in ACMSIMC-V4 */
REAL imife_realtime_gain_off = 1.0;
void init_FE_htz(){
    int ind;
    for(ind=0;ind<2;++ind){
        FE.htz.emf_stator[ind] = 0;

        FE.htz.psi_1[0] = d_sim.init.KE;
        FE.htz.psi_1[1] = 0;
        FE.htz.psi_2[ind] = 0;
        FE.htz.psi_2_prev[ind] = 0;

        FE.htz.psi_1_nonSat[ind] = 0;
        FE.htz.psi_2_nonSat[ind] = 0;

        FE.htz.psi_1_min[ind] = 0;
        FE.htz.psi_1_max[ind] = 0;
        FE.htz.psi_2_min[ind] = 0;
        FE.htz.psi_2_max[ind] = 0;

        FE.htz.rs_est = d_sim.init.R;
        FE.htz.rreq_est = d_sim.init.Rreq;

        FE.htz.Delta_t = 1;
        FE.htz.u_offset[ind] = 0;
        // FE.htz.u_offset_intermediate[ind] = 0;
        FE.htz.u_off_original_lpf_input[ind]=0.0; // holtz03 original (but I uses integrator instead of LPF)
        FE.htz.u_off_saturation_time_correction[ind]=0.0; // exact offset calculation for compensation
        FE.htz.u_off_calculated_increment[ind]=0.0;    // saturation time based correction
        FE.htz.gain_off = HOLTZ_2002_GAIN_OFFSET; // 5; -> slow but stable // 50.1 // 20 -> too large then speed will oscillate during reversal near zero

        FE.htz.flag_pos2negLevelA[ind] = 0;
        FE.htz.flag_pos2negLevelB[ind] = 0;
        FE.htz.time_pos2neg[ind] = 0;
        FE.htz.time_pos2neg_prev[ind] = 0;

        FE.htz.flag_neg2posLevelA[ind] = 0;
        FE.htz.flag_neg2posLevelB[ind] = 0;
        FE.htz.time_neg2pos[ind] = 0;
        FE.htz.time_neg2pos_prev[ind] = 0;

        FE.htz.psi_aster_max = IM_FLUX_COMMAND_DC_PART + IM_FLUX_COMMAND_SINE_PART;

        FE.htz.maximum_of_sat_min_time[ind] = 0.0;
        FE.htz.maximum_of_sat_max_time[ind] = 0.0;
        FE.htz.sat_min_time[ind] = 0.0;
        FE.htz.sat_max_time[ind] = 0.0;
        FE.htz.sat_min_time_reg[ind] = 0.0;
        FE.htz.sat_max_time_reg[ind] = 0.0;
        FE.htz.extra_limit = 0.0;
        FE.htz.flag_limit_too_low = FALSE;
    }
}
void rhf_Holtz2003_Dynamics(REAL t, REAL *x, REAL *fx){
    FE.htz.emf_stator[0] = US(0) - (*CTRL).motor->R * IS(0) - FE.htz.u_offset[0] + OFFSET_VOLTAGE_ALPHA;
    FE.htz.emf_stator[1] = US(1) - (*CTRL).motor->R * IS(1) - FE.htz.u_offset[1] + OFFSET_VOLTAGE_BETA ;
    fx[0] = (FE.htz.emf_stator[0]);
    fx[1] = (FE.htz.emf_stator[1]);
}
REAL marino_saturation_gain_scale_factor1 = 1.0;
REAL marino_saturation_gain_scale_factor2 = 1.0;
REAL marino_sat_d_axis_flux_control = 1.0;
REAL marino_sat_q_axis_flux_control = 1.0;
int bool_positive_extra_limit = TRUE;
void VM_Saturated_ExactOffsetCompensation_WithAdaptiveLimit(){
    #define PSI_MU_ASTER_MAX FE.htz.psi_aster_max // Holtz缺点就是实际磁链超过给定磁链时，失效！自动检测上下界同时饱和的情况，然后增大限幅？
    #define BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT FALSE // The limit is extended when both upper and lower limit are reached in a cycle.

    #define BOOL_USE_METHOD_LPF_INPUT      FALSE
    #define BOOL_USE_METHOD_INTEGRAL_INPUT TRUE

    // Euler's method is shit at higher speeds
    FE.htz.emf_stator[0] = US_C(0) - (*CTRL).motor->R*IS_C(0) - FE.htz.u_offset[0];
    FE.htz.emf_stator[1] = US_C(1) - (*CTRL).motor->R*IS_C(1) - FE.htz.u_offset[1];
    // FE.htz.psi_1[0] += CL_TS*(FE.htz.emf_stator[0]);
    // FE.htz.psi_1[1] += CL_TS*(FE.htz.emf_stator[1]);

    // rk4 
    general_2states_rk4_solver(&rhf_Holtz2003_Dynamics, (*CTRL).timebase, FE.htz.psi_1, CL_TS);
    FE.htz.psi_2[0] = FE.htz.psi_1[0] - (*CTRL).motor->Lq*IS_C(0);
    FE.htz.psi_2[1] = FE.htz.psi_1[1] - (*CTRL).motor->Lq*IS_C(1);
    FE.htz.psi_2_ampl = sqrt(FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);

    // 限幅前求角度还是应该限幅后？
    FE.htz.theta_d = atan2(FE.htz.psi_2[1], FE.htz.psi_2[0]);
    FE.htz.cosT = cos(FE.htz.theta_d);
    FE.htz.sinT = sin(FE.htz.theta_d);

    FE.htz.psi_1_nonSat[0] += CL_TS*(FE.htz.emf_stator[0]);
    FE.htz.psi_1_nonSat[1] += CL_TS*(FE.htz.emf_stator[1]);
    FE.htz.psi_2_nonSat[0] = FE.htz.psi_1_nonSat[0] - (*CTRL).motor->Lq*IS_C(0);
    FE.htz.psi_2_nonSat[1] = FE.htz.psi_1_nonSat[1] - (*CTRL).motor->Lq*IS_C(1);

    // FE.htz.psi_aster_max = (*CTRL).taao_flux_cmd + 0.05;
    FE.htz.psi_aster_max = (*CTRL).i->cmd_psi + FE.htz.extra_limit;
    // FE.htz.psi_aster_max = (*CTRL).taao_flux_cmd;

    // 限幅是针对转子磁链限幅的
    int ind;
    for(ind=0;ind<2;++ind){
        if((*CTRL).i->cmd_varOmega != 0.0){
            if(FE.htz.psi_2[ind]    > PSI_MU_ASTER_MAX){ // TODO BUG呀！这里怎么可以是>应该是大于等于啊！
                FE.htz.psi_2[ind]   = PSI_MU_ASTER_MAX;
                FE.htz.sat_max_time[ind] += CL_TS;
                // marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
                // marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
                // marino_sat_d_axis_flux_control = 0.0;
                // marino_sat_q_axis_flux_control = 0.0;
            }else if(FE.htz.psi_2[ind] < -PSI_MU_ASTER_MAX){
                FE.htz.psi_2[ind]   = -PSI_MU_ASTER_MAX;
                FE.htz.sat_min_time[ind] += CL_TS;
            //     marino.lambda_inv = marino_saturation_gain_scale_factor1*LAMBDA_INV_xOmg;
            //     marino.gamma_inv  = marino_saturation_gain_scale_factor2*GAMMA_INV_xTL;
            //     marino_sat_d_axis_flux_control = 0.0;
            //     marino_sat_q_axis_flux_control = 0.0;
            }else{
                // 这样可以及时清零饱和时间
                if(FE.htz.sat_max_time[ind]>0){FE.htz.sat_max_time[ind] -= CL_TS;}
                if(FE.htz.sat_min_time[ind]>0){FE.htz.sat_min_time[ind] -= CL_TS;}
                // marino.lambda_inv = LAMBDA_INV_xOmg;
                // marino.gamma_inv  = GAMMA_INV_xTL;
                // marino_sat_d_axis_flux_control = 1.0;
                // marino_sat_q_axis_flux_control = 1.0;
            }
        }
        // 上限饱和减去下限饱和作为误差，主要为了消除实际磁链幅值大于给定的情况，实际上这种现象在常见工况下出现次数不多。
        FE.htz.u_off_saturation_time_correction[ind] = FE.htz.sat_max_time[ind] - FE.htz.sat_min_time[ind];
        // u_offset波动会导致sat_min_time和sat_max_time的波动，这个时候最有效的办法是减少gain_off。
        // 但是同时，观察饱和时间sat_min_time等的波形，可以发现它里面也会出现一个正弦波包络线。
        if(FE.htz.sat_min_time[ind] > FE.htz.maximum_of_sat_min_time[ind]){FE.htz.maximum_of_sat_min_time[ind] = FE.htz.sat_min_time[ind];}
        if(FE.htz.sat_max_time[ind] > FE.htz.maximum_of_sat_max_time[ind]){FE.htz.maximum_of_sat_max_time[ind] = FE.htz.sat_max_time[ind];}
    }

    // 数数，算磁链周期
    if(FE.htz.psi_2[0]    > 0.0){
        FE.htz.count_positive_in_one_cycle[0] += 1;
        if(FE.htz.count_negative_in_one_cycle[0]!=0){ FE.htz.negative_cycle_in_count[0] = FE.htz.count_negative_in_one_cycle[0]; FE.htz.count_negative_in_one_cycle[0] = 0;}
    }else if(FE.htz.psi_2[0] < -0.0){
        FE.htz.count_negative_in_one_cycle[0] += 1;
        if(FE.htz.count_positive_in_one_cycle[0]!=0){ FE.htz.positive_cycle_in_count[0] = FE.htz.count_positive_in_one_cycle[0]; FE.htz.count_positive_in_one_cycle[0] = 0;}
    }
    if(FE.htz.psi_2[1]    > 0.0){
        FE.htz.count_positive_in_one_cycle[1] += 1;
        if(FE.htz.count_negative_in_one_cycle[1]!=0){ FE.htz.negative_cycle_in_count[1] = FE.htz.count_negative_in_one_cycle[1]; FE.htz.count_negative_in_one_cycle[1] = 0;}
    }else if(FE.htz.psi_2[1] < -0.0){
        FE.htz.count_negative_in_one_cycle[1] += 1;
        if(FE.htz.count_positive_in_one_cycle[1]!=0){ FE.htz.positive_cycle_in_count[1] = FE.htz.count_positive_in_one_cycle[1]; FE.htz.count_positive_in_one_cycle[1] = 0;}
    }

    // 限幅后的转子磁链，再求取限幅后的定子磁链
    FE.htz.psi_1[0] = FE.htz.psi_2[0] + (*CTRL).motor->Lq*IS_C(0);
    FE.htz.psi_1[1] = FE.htz.psi_2[1] + (*CTRL).motor->Lq*IS_C(1);

    // Speed Estimation
    if(TRUE){
        // FE.htz.ireq[0] = (*CTRL).Lmu_inv*FE.htz.psi_2[0] - IS_C(0);
        // FE.htz.ireq[1] = (*CTRL).Lmu_inv*FE.htz.psi_2[1] - IS_C(1);
        REAL temp;
        temp = (FE.htz.psi_1[0]*FE.htz.psi_1[0]+FE.htz.psi_1[1]*FE.htz.psi_1[1]);
        if(temp>0.001){
            FE.htz.field_speed_est = - (FE.htz.psi_1[0]*-FE.htz.emf_stator[1] + FE.htz.psi_1[1]*FE.htz.emf_stator[0]) / temp;
        }
        temp = (FE.htz.psi_2[0]*FE.htz.psi_2[0]+FE.htz.psi_2[1]*FE.htz.psi_2[1]);
        if(temp>0.001){
            FE.htz.slip_est = (*CTRL).motor->Rreq*(IS_C(0)*-FE.htz.psi_2[1]+IS_C(1)*FE.htz.psi_2[0]) / temp;
        }
        FE.htz.omg_est = FE.htz.field_speed_est - FE.htz.slip_est;
    }


    // TODO My proposed saturation time based correction method NOTE VERY COOL
    #define CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS \
        FE.htz.u_off_original_lpf_input[ind]         = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) /  (FE.htz.Delta_t+FE.htz.Delta_t_last); \
        FE.htz.u_off_calculated_increment[ind]       = 0.5*(FE.htz.psi_2_min[ind] + FE.htz.psi_2_max[ind]) / ((FE.htz.Delta_t+FE.htz.Delta_t_last) - (FE.htz.sat_max_time[ind]+FE.htz.sat_min_time[ind])); \
        FE.htz.u_off_saturation_time_correction[ind] = FE.htz.sat_max_time[ind] - FE.htz.sat_min_time[ind]; \
        FE.htz.u_off_direct_calculated[ind] += (FE.htz.count_negative_cycle+FE.htz.count_positive_cycle>4) * FE.htz.u_off_calculated_increment[ind]; // if(BOOL_USE_METHOD_DIFFERENCE_INPUT) 
        // 引入 count：刚起动时的几个磁链正负半周里，Delta_t_last 存在巨大的计算误差，所以要放弃更新哦。

    for(ind=0;ind<2;++ind){ // Loop for alpha & beta components // destroy integer outside this loop to avoid accidentally usage 

        // if( tmin!=0 && tmax!=0 ){
        //     // The sat func's limit is too small.
        //     limit += TS * min(tmin, tmax);
        // }


        /* 必须先检查是否进入levelA */
        if(FE.htz.flag_pos2negLevelA[ind] == TRUE){ 
            if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]<0){ // 二次检查，磁链已经是负的了  <- 可以改为施密特触发器
                if(FE.htz.flag_pos2negLevelB[ind] == FALSE){
                    FE.htz.count_negative_cycle+=1; // FE.htz.count_positive_cycle = 0;
                    // printf("POS2NEG: %g, %d\n", (*CTRL).timebase, ind);
                    // printf("%g, %g\n", FE.htz.psi_2_prev[ind], FE.htz.psi_2[ind]);
                    // getch();
                    // 第一次进入寻找最小值的levelB，说明最大值已经检测到。
                    FE.htz.psi_1_max[ind] = FE.htz.psi_2_max[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    FE.htz.Delta_t_last = FE.htz.Delta_t;
                    FE.htz.Delta_t = FE.htz.time_pos2neg[ind] - FE.htz.time_pos2neg_prev[ind];
                    FE.htz.time_pos2neg_prev[ind] = FE.htz.time_pos2neg[ind]; // 备份作为下次耗时参考点
                    // 初始化
                    FE.htz.flag_neg2posLevelA[ind] = FALSE;
                    FE.htz.flag_neg2posLevelB[ind] = FALSE;

                    // 注意这里是正半周到负半周切换的时候才执行一次的哦！
                    CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                    // FE.htz.accumulated__u_off_saturation_time_correction[ind] += FE.htz.u_off_saturation_time_correction[ind];
                    FE.htz.sign__u_off_saturation_time_correction[ind] = -1.0;
                    // 饱和时间的正弦包络线的正负半周的频率比磁链频率低多啦！需要再额外加一个低频u_offset校正
                    FE.htz.sat_time_offset[ind] = FE.htz.maximum_of_sat_max_time[ind] - FE.htz.maximum_of_sat_min_time[ind];
                    FE.htz.maximum_of_sat_max_time[ind] = 0.0;
                    FE.htz.maximum_of_sat_min_time[ind] = 0.0;

                    FE.htz.psi_1_min[ind] = 0.0;
                    FE.htz.psi_2_min[ind] = 0.0;
                    if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                        FE.htz.sat_min_time_reg[ind] = FE.htz.sat_min_time[ind];
                        if(FE.htz.sat_max_time_reg[ind]>CL_TS && FE.htz.sat_min_time_reg[ind]>CL_TS){
                            FE.htz.flag_limit_too_low = TRUE;
                            FE.htz.extra_limit += 1e-2 * (FE.htz.sat_max_time_reg[ind] + FE.htz.sat_min_time_reg[ind]) / FE.htz.Delta_t; 
                        }else{
                            FE.htz.flag_limit_too_low = FALSE;
                            FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
                            if(bool_positive_extra_limit){
                                if(FE.htz.extra_limit<0.0){
                                    FE.htz.extra_limit = 0.0;
                                }
                            }
                        }
                        FE.htz.sat_max_time_reg[ind] = 0.0;
                    }
                    FE.htz.sat_min_time[ind] = 0.0;
                }
                FE.htz.flag_pos2negLevelB[ind] = TRUE;
                if(FE.htz.flag_pos2negLevelB[ind] == TRUE){ // 寻找磁链最小值
                    if(FE.htz.psi_2[ind] < FE.htz.psi_2_min[ind]){
                        FE.htz.psi_2_min[ind] = FE.htz.psi_2[ind];
                    }
                }
            }else{ // 磁链还没有变负，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]>0
                FE.htz.flag_pos2negLevelA[ind] = FALSE; /* 震荡的话，另一方的检测就有可能被触动？ */
            }
        }
        if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]<0){ // 发现磁链由正变负的时刻
            FE.htz.flag_pos2negLevelA[ind] = TRUE;
            FE.htz.time_pos2neg[ind] = (*CTRL).timebase;
        }


        if(FE.htz.flag_neg2posLevelA[ind] == TRUE){ 
            if(FE.htz.psi_2_prev[ind]>0 && FE.htz.psi_2[ind]>0){ // 二次检查，磁链已经是正的了
                if(FE.htz.flag_neg2posLevelB[ind] == FALSE){
                    FE.htz.count_positive_cycle+=1; // FE.htz.count_negative_cycle = 0;
                    // 第一次进入寻找最大值的levelB，说明最小值已经检测到。
                    FE.htz.psi_1_min[ind] = FE.htz.psi_2_min[ind]; // 不区别定转子磁链，区别：psi_2是连续更新的，而psi_1是离散更新的。
                    FE.htz.Delta_t_last = FE.htz.Delta_t;
                    FE.htz.Delta_t = FE.htz.time_neg2pos[ind] - FE.htz.time_neg2pos_prev[ind];
                    FE.htz.time_neg2pos_prev[ind] = FE.htz.time_neg2pos[ind]; // 备份作为下次耗时参考点
                    // 初始化
                    FE.htz.flag_pos2negLevelA[ind] = FALSE;
                    FE.htz.flag_pos2negLevelB[ind] = FALSE;

                    CALCULATE_OFFSET_VOLTAGE_COMPENSATION_TERMS
                    // FE.htz.accumulated__u_off_saturation_time_correction[ind] += FE.htz.u_off_saturation_time_correction[ind];
                    FE.htz.sign__u_off_saturation_time_correction[ind] = 1.0;

                    FE.htz.psi_1_max[ind] = 0.0;
                    FE.htz.psi_2_max[ind] = 0.0;
                    if(BOOL_TURN_ON_ADAPTIVE_EXTRA_LIMIT){
                        FE.htz.sat_max_time_reg[ind] = FE.htz.sat_max_time[ind];
                        if(FE.htz.sat_min_time_reg[ind]>CL_TS && FE.htz.sat_max_time_reg[ind]>CL_TS){
                            FE.htz.flag_limit_too_low = TRUE;
                            FE.htz.extra_limit += 1e-2 * (FE.htz.sat_min_time_reg[ind] + FE.htz.sat_max_time_reg[ind]) / FE.htz.Delta_t; 
                        }else{
                            FE.htz.flag_limit_too_low = FALSE;
                            FE.htz.extra_limit -= 2e-4 * FE.htz.Delta_t;
                            if(FE.htz.extra_limit<0.0){
                                FE.htz.extra_limit = 0.0;
                            }
                        }
                        FE.htz.sat_min_time_reg[ind] = 0.0;
                    }
                    FE.htz.sat_max_time[ind] = 0.0;
                }
                FE.htz.flag_neg2posLevelB[ind] = TRUE; 
                if(FE.htz.flag_neg2posLevelB[ind] == TRUE){ // 寻找磁链最大值
                    if(FE.htz.psi_2[ind] > FE.htz.psi_2_max[ind]){
                        FE.htz.psi_2_max[ind] = FE.htz.psi_2[ind];
                    }
                }
            }else{ // 磁链还没有变正，说明是虚假过零，比如在震荡，FE.htz.psi_2[0]<0
                FE.htz.flag_neg2posLevelA[ind] = FALSE;
            }
        }
        if(FE.htz.psi_2_prev[ind]<0 && FE.htz.psi_2[ind]>0){ // 发现磁链由负变正的时刻
            FE.htz.flag_neg2posLevelA[ind] = TRUE;
            FE.htz.time_neg2pos[ind] = (*CTRL).timebase;
        }
    }

    /*这里一共有四种方案，积分两种，LPF两种：
    1. Holtz03原版是用u_off_original_lpf_input过LPF，
    2. 我发现u_off_original_lpf_input过积分器才能完全补偿偏置电压，
    3. 我还提出可以直接算出偏置电压补偿误差（可加LPF），
    4. 我还提出了用饱和时间去做校正的方法*/

    // 积分方法：（从上面的程序来看，u_off的LPF的输入是每半周更新一次的。
    #if BOOL_USE_METHOD_INTEGRAL_INPUT
        #define INTEGRAL_INPUT(X)   FE.htz.u_off_saturation_time_correction[X] // exact offset calculation for compensation
        // FE.htz.sat_time_offset[X]
        // #define INTEGRAL_INPUT(X)   FE.htz.accumulated__u_off_saturation_time_correction[X]
        // #define INTEGRAL_INPUT(X)   FE.htz.u_off_original_lpf_input[X]

        long int local_sum = FE.htz.negative_cycle_in_count[0] + FE.htz.positive_cycle_in_count[0] + FE.htz.negative_cycle_in_count[1] + FE.htz.positive_cycle_in_count[1];
        if(local_sum>0){
            FE.htz.gain_off = imife_realtime_gain_off * HOLTZ_2002_GAIN_OFFSET / ((REAL)local_sum*CL_TS);
        }
        FE.htz.u_offset[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
        FE.htz.u_offset[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);

        // 想法是好的，但是没有用，因为咱们要的不是磁链正负半周的饱和时间最大和最小相互抵消；实际上观察到正负半周是饱和时间的正弦包络线的正负半周，频率比磁链频率低多啦！
        // FE.htz.accumulated__u_off_saturation_time_correction[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
        // FE.htz.accumulated__u_off_saturation_time_correction[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);
        // if(FE.htz.sign__u_off_saturation_time_correction[0]>0){
        //     FE.htz.u_offset[0] = FE.htz.accumulated__u_off_saturation_time_correction[0];
        //     FE.htz.sign__u_off_saturation_time_correction[0] = 0;
        // }
        // if(FE.htz.sign__u_off_saturation_time_correction[1]>0){
        //     FE.htz.u_offset[1] = FE.htz.accumulated__u_off_saturation_time_correction[1];
        //     FE.htz.sign__u_off_saturation_time_correction[1] = 0;
        // }

        // 本来想二重积分消除交流波动，但是我发现饱和时间误差波动的原因是上下饱和时间清零不及时导致的。但是哦……好像也没有有效地及时清零的方法，所以还是试试双重积分吧：
        // FE.htz.u_offset_intermediate[0] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(0);
        // FE.htz.u_offset_intermediate[1] += FE.htz.gain_off * CL_TS * INTEGRAL_INPUT(1);
        // FE.htz.u_offset[0] += CL_TS * FE.htz.u_offset_intermediate[0];
        // FE.htz.u_offset[1] += CL_TS * FE.htz.u_offset_intermediate[1];
    #endif

    // 低通
    #if BOOL_USE_METHOD_LPF_INPUT
        #define LPF_INPUT(X) FE.htz.u_off_direct_calculated[X]
        // #define LPF_INPUT(X) FE.htz.u_off_original_lpf_input[X] // holtz03 original (but using LPF cannot fully compensate offset voltage)
        #define TAU_OFF_INVERSE (500*2*M_PI) // 越大则越接近全通 0.05 
        FE.htz.u_offset[0] = _lpf( LPF_INPUT(0), FE.htz.u_offset[0], TAU_OFF_INVERSE);
        FE.htz.u_offset[1] = _lpf( LPF_INPUT(1), FE.htz.u_offset[1], TAU_OFF_INVERSE);
    #endif

    // 差分
    // 别傻，不是在这里更新的，此处更新频率是1/CL_TS啊…… 
    // FE.htz.u_offset[0] += DIFFERENCE_INPUT(0);
    // FE.htz.u_offset[1] += DIFFERENCE_INPUT(1);

    // 直通
    // FE.htz.u_offset[0] = FE.htz.u_off_direct_calculated[0];
    // FE.htz.u_offset[1] = FE.htz.u_off_direct_calculated[1];

    // FE.htz.psi_1_nonSat[0] = FE.htz.psi_1[0];
    // FE.htz.psi_1_nonSat[1] = FE.htz.psi_1[1];
    // FE.htz.psi_2_nonSat[0] = FE.htz.psi_2[0];
    // FE.htz.psi_2_nonSat[1] = FE.htz.psi_2[1];
    FE.htz.theta_d = atan2(FE.htz.psi_2[1], FE.htz.psi_2[0]);
    FE.htz.theta_e = angle_diff(FE.htz.theta_d, (*CTRL).i->theta_d_elec) * ONE_OVER_2PI * 360;
    FE.htz.psi_2_prev[0] = FE.htz.psi_2[0];
    FE.htz.psi_2_prev[1] = FE.htz.psi_2[1];
}
void VM_Saturated_ExactOffsetCompensation(){
}
void VM_Saturated_ExactOffsetCompensation_WithParallelNonSaturatedEstimator(){
    /* 重点是正常情况下，咱们用VM_Saturated_ExactOffsetCompensation的 u_offset 以及磁链估计结果 psi_2 直接去校正第二个平行的观测器 ParallelNonSaturatedEstimator，
        但是一旦出现t_alpha,sat,min>0 且 t_bet,sat,max>0的情况，这时就以第二个平行观测器在一个周期内观测得到的磁链的幅值结果作为第一个观测器的限幅值，
        同时，在那个周期内，第一个观测器也并不是没用的，而是继续作为一个 u_offset detector（？实际上不行，见下面hint）
        因为就算t_alpha,sat,min>0 且 t_bet,sat,max>0，此时仍然会因为直流偏置 u_offset 的存在而导致t_alpha,sat,min != t_bet,sat,max，它们的差值就算 u_offset 的值大小的一个 indicator。
        *当然，实际情况下可以给第二个观测器设置一个比较大的饱和值，因为磁链不可能无限变大的。

        hint：值得一提的是，如果同时上下界都饱和了，那么 psi_alpha2min + psi_alpha2max = 0，此时怎么算偏置电压 u_offset 都是零了！
        那么有意思的问题就来了，这个时候根据第二个观测器输出的磁链的 (最大值+最小值)/2 是否可以作为 u_offset？
        如果答案是 yes 的话，那么打从一开始为什么要引入 Saturation 呢？
        Saturation的作用，就是能够校正已经偏离了 t轴 的磁链观测结果。只补偿 u_offset，只是保证了磁链波形不会进一步上升（或下降），但是无法把已经偏离的磁链波形的中心移回到t轴上。

        那如果我假设磁链正负总是应该对称的，每个周期更新一次 psi_offset 和 u_offset 呢？
        psi_offset 是磁链波形正负面积的积分。 u_offset 在一定时间内是造成这 psi_offset 的磁链导数。
    */
}

#endif
/* Init functions */
void rk4_init(){
    OBSV.rk4.us[0] = 0.0;
    OBSV.rk4.us[1] = 0.0;
    OBSV.rk4.is[0] = 0.0;
    OBSV.rk4.is[1] = 0.0;
    OBSV.rk4.us_curr[0] = 0.0;
    OBSV.rk4.us_curr[1] = 0.0;
    OBSV.rk4.is_curr[0] = 0.0;
    OBSV.rk4.is_curr[1] = 0.0;
    OBSV.rk4.us_prev[0] = 0.0;
    OBSV.rk4.us_prev[1] = 0.0;
    OBSV.rk4.is_prev[0] = 0.0;
    OBSV.rk4.is_prev[1] = 0.0;
}
#if (WHO_IS_USER == USER_YZZ) || (WHO_IS_USER == USER_CJH)
void observer_init(){

    // init_esoaf();

    init_FE_htz();

    int i;
    for(i=0; i<2; ++i){
        /* Ohtani 1990 */
        // FE.exact.psi_2_max[i] = +(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        // FE.exact.psi_2_min[i] = -(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);

        FE.exact.psi_2_last_top[i]  = 1*+(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_last_butt[i] = 1*-(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output_last_top[i]  = 1*+(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output_last_butt[i] = 1*-(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output2_last_top[i]  = 1*+(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);
        FE.exact.psi_2_output2_last_butt[i] = 1*-(IM_FLUX_COMMAND_DC_PART+IM_FLUX_COMMAND_SINE_PART);

        FE.exact.first_time_enter_top[i] = TRUE;
        FE.exact.first_time_enter_butt[i] = TRUE;
    }


    FE.harnefors.lambda = GAIN_HARNEFORS_LAMBDA;
}

#endif

#endif















#include "ACMSim.h"


struct HallSensor HALL;
void sensors(){
    #define USE_EQEP FALSE
    #define USE_HALL FALSE
    #if USE_EQEP == TRUE
        #define NUMBER_OF_CURRENT_LOOP (1*SPEED_LOOP_CEILING)
        #define QEP_SPEED_RESOLUTION_RPM (60.0 / (SYSTEM_QEP_PULSES_PER_REV * (NUMBER_OF_CURRENT_LOOP*CL_TS)))

        #define BOOL_USE_ABOSOLUTE_POSITION TRUE

        // TODO 这里的建模没有考虑初始d轴的偏置角！

        // TODO 反转的时候qep的转速好像是错的！

        static int sensor_vc_count = 1;
        if(sensor_vc_count++ == NUMBER_OF_CURRENT_LOOP){
            sensor_vc_count = 1;

            // qep.theta_d_mech += qep.QPOSLAT;

            // qep.difference_in_cnt = qep.QPOSLAT/(2*M_PI) * SYSTEM_QEP_PULSES_PER_REV; // 这句建模的是纯粹的增量
            // 增量式编码器测量的其实不是纯粹的增量，更像是是一个累加器，然后round off，所以用qep.QPOSLAT控制会不稳定

            /* Position */
            if(BOOL_USE_ABOSOLUTE_POSITION){
                // 使用ActualPosInCnt的方案是错误的，因为一开始转子位置可能不为零，QEP能知道的其实只是增量。
                qep.ActualPosInCnt_Previous = qep.ActualPosInCnt;
                qep.RoundedPosInCnt_Previous = qep.RoundedPosInCnt;                    

                qep.ActualPosInCnt = (ACM.theta_d_accum / (2*M_PI) * SYSTEM_QEP_PULSES_PER_REV);
                qep.number_of_revolution = qep.ActualPosInCnt / SYSTEM_QEP_PULSES_PER_REV;
                qep.theta_d_accum  = (         qep.ActualPosInCnt*SYSTEM_QEP_REV_PER_PULSE)*2*M_PI;

                int64 the_sign = sign_integer(qep.ActualPosInCnt);
                qep.RoundedPosInCnt = the_sign * (the_sign * qep.ActualPosInCnt) % (int64)SYSTEM_QEP_PULSES_PER_REV;
                qep.theta_d_mech = ( qep.RoundedPosInCnt * SYSTEM_QEP_REV_PER_PULSE ) *2*M_PI \
                                   + 0*1*15.0 / 180.0 * M_PI; // IPD Offset?
            }else{
                qep.theta_d_mech += qep.difference_in_cnt * SYSTEM_QEP_REV_PER_PULSE *(2*M_PI);
                // qep.theta_d_mech += qep.QPOSLAT/(2*M_PI) * SYSTEM_QEP_PULSES_PER_REV * SYSTEM_QEP_REV_PER_PULSE *(2*M_PI);
            }
            qep.QPOSLAT = 0;
            // Directly Calculate
            // Directly Calculate
            // Directly Calculate
            // qep.theta_d = ACM.npp*qep.theta_d_mech + CTRLQEP.theta_d_offset;
            // while(qep.theta_d> M_PI) qep.theta_d -= 2*M_PI;
            // while(qep.theta_d<-M_PI) qep.theta_d += 2*M_PI;

            if(qep.ActualPosInCnt==-1){
                int a = 0;
                a++;
            }

            /* Speed */
            // if((*CTRL).timebase>0.0008){ //2*CL_TS
            if((*CTRL).timebase>CL_TS){
                qep.moving_average[3] = qep.moving_average[2];
                qep.moving_average[2] = qep.moving_average[1];
                qep.moving_average[1] = qep.moving_average[0];
                if(BOOL_USE_ABOSOLUTE_POSITION){
                    qep.difference_in_cnt = (int32)qep.RoundedPosInCnt - (int32)qep.RoundedPosInCnt_Previous; // 无符号数相减需要格外小心

                    // 增量超过一半则认为是Cnt被清零了
                    if( qep.difference_in_cnt < -0.5*SYSTEM_QEP_PULSES_PER_REV){
                        qep.difference_in_cnt =  (int32)SYSTEM_QEP_PULSES_PER_REV + qep.RoundedPosInCnt - qep.RoundedPosInCnt_Previous;
                    }else if ( qep.difference_in_cnt > 0.5*SYSTEM_QEP_PULSES_PER_REV){
                        qep.difference_in_cnt = -(int32)SYSTEM_QEP_PULSES_PER_REV + qep.RoundedPosInCnt - qep.RoundedPosInCnt_Previous;
                    }
                }
            }

            // Incrementally Accumulated
            // Incrementally Accumulated
            // Incrementally Accumulated
            (*CTRL).i->encoder_cnt += qep.difference_in_cnt;
            if((*CTRL).i->encoder_cnt > SYSTEM_QEP_PULSES_PER_REV){(*CTRL).i->encoder_cnt -= SYSTEM_QEP_PULSES_PER_REV;}
            if((*CTRL).i->encoder_cnt < 0)                        {(*CTRL).i->encoder_cnt += SYSTEM_QEP_PULSES_PER_REV;}

            CTRLQEP.theta_d__state += qep.difference_in_cnt*SYSTEM_QEP_REV_PER_PULSE * (2*M_PI) * ACM.npp;
            if(CTRLQEP.theta_d__state> M_PI) CTRLQEP.theta_d__state -= 2*M_PI;
            if(CTRLQEP.theta_d__state<-M_PI) CTRLQEP.theta_d__state += 2*M_PI;
            qep.theta_d = CTRLQEP.theta_d__state + CTRLQEP.theta_d_offset;
            if(qep.theta_d> M_PI) qep.theta_d -= 2*M_PI;
            if(qep.theta_d<-M_PI) qep.theta_d += 2*M_PI;

            qep.moving_average[0] = qep.difference_in_cnt * QEP_SPEED_RESOLUTION_RPM; // [rpm]

            // qep.moving_average[0] = (Uint32)(ACM.theta_d_accum_increment/ (2*M_PI) * SYSTEM_QEP_PULSES_PER_REV) * QEP_SPEED_RESOLUTION_RPM; // [rpm]

            // this gives wrong results when SYSTEM_QEP_PULSES_PER_REV is larger than 1e3, e.g., 1e4.
            // qep.moving_average[0] = (qep.ActualPosInCnt - qep.ActualPosInCnt_Previous) * QEP_SPEED_RESOLUTION_RPM; // [rpm]

            // no filter
            qep.varTheta = qep.moving_average[0] * RPM_2_ELEC_RAD_PER_SEC;
            // qep.varTheta = (ACM.theta_d_accum - qep.ActualPosInCnt_Previous) *ACM.npp_inv;

            // moving average filter
            // qep.varTheta = 0.25*(qep.moving_average[3]+qep.moving_average[2]+qep.moving_average[1]+qep.moving_average[0]) * RPM_2_ELEC_RAD_PER_SEC;

            qep.omg_mech = qep.varTheta * ACM.npp_inv;

            /* Position (Option 2) */
            // qep.theta_d += (NUMBER_OF_CURRENT_LOOP*CL_TS) * qep.varTheta;
            // while(qep.theta_d> M_PI) qep.theta_d -= 2*M_PI;
            // while(qep.theta_d<-M_PI) qep.theta_d += 2*M_PI;
        }
    #endif
    #if USE_HALL == TRUE

        EXP.theta_d  = ACM.theta_d; // + 30.0/180*M_PI;
        EXP.varTheta = ACM.varTheta;
        EXP.omg_mech = EXP.varTheta * EXP.npp_inv;

        // 霍尔传感器检测
        EXP.hallABC = hall_sensor(ACM.theta_d/M_PI*180.0);
        hall_resolver(EXP.hallABC, &EXP.theta_d_hall, &EXP.varTheta_hall);

        // 测量转子d轴位置限幅
        if(EXP.theta_d_hall > M_PI){
            EXP.theta_d_hall -= 2*M_PI;
        }else if(EXP.theta_d_hall < -M_PI){
            EXP.theta_d_hall += 2*M_PI;
        }

        // // if((*CTRL).timebase>19){
        //     EXP.theta_d  = EXP.theta_d_hall;
        //     EXP.varTheta = EXP.varTheta_hall;
        // // }
    #endif
}
// void HALL_init(){

//     HALL.acceleration_avg = 0.0;
//     HALL.last_varTheta_hall = 0.0;

//     HALL.varTheta_hall = 0.0;
//     HALL.omg_mech_hall = 0.0;
//     HALL.theta_d_hall = 0.0;

//     HALL.hallA = 0;
//     HALL.hallB = 1;
//     HALL.hallC = 1;
//     HALL.hallABC = 0x3;
//     HALL.last_hallABC = 0x1;
//     HALL.speed_direction = +1;
//     HALL.bool_position_correction_pending = FALSE;

//     HALL.timebase = 0.0;
//     HALL.timeStamp = 0.0;
//     HALL.timeDifference = 0.0;
//     HALL.timeDifferenceStamp = 100000.0;
// }



















void tustin_pid(st_pid_regulator *r){
    #define DYNAMIC_CLAPMING TRUE
    #define DYNAMIC_CLAPMING_WUBO TRUE

    // 误差
    r->Err = r->Ref - r->Fbk;
    // 比例
    r->P_Term = r->Err * r->Kp;
    // 积分
    r->I_Term += r->Err * r->Ki_CODE;
    r->OutNonSat = r->I_Term;
    // Inner Loop
    r->KFB_Term = r->Fbk * r->KFB;

    // 添加积分饱和特性
    #if DYNAMIC_CLAPMING
        // dynamic clamping
        if( r->I_Term > r->OutLimit - r->Out)
            r->I_Term = r->OutLimit - r->Out;
        else if( r->I_Term < -r->OutLimit + r->Out)
            r->I_Term =      -r->OutLimit + r->Out; // OutLimit is a positive constant
    #else
        // static clamping
        if( r->I_Term > r->OutLimit)
            r->I_Term = r->OutLimit; 
        else if( r->I_Term < -r->OutLimit)
            r->I_Term = -r->OutLimit;
    #endif

    // 微分
    // r->D_Term = r->Kd * (r->Err - r->ErrPrev);

    // 输出
    r->Out = r->I_Term + r->P_Term - r->KFB_Term; // + r->D_Term
    r->OutNonSat += r->P_Term; // + r->D_Term

    // 输出限幅
    if(r->Out > r->OutLimit)
        r->Out = r->OutLimit;
    else if(r->Out < -r->OutLimit)
        r->Out = -r->OutLimit;

    // 当前步误差赋值为上一步误差
    r->ErrPrev = r->Err;
    // 记录饱和输出和未饱和输出的差
    r->SatDiff = r->Out - r->OutNonSat;
}
#endif

#endif
