#include "ACMSim.h"

#if INVERTER_NONLINEARITY

void InverterNonlinearity_SKSul96(double ual, double ube, double ial, double ibe){
    double ua,ub,uc;
    double ia,ib,ic;
    double Udist;
    double TM;
    double Rce=0.04958, Rdiode=0.05618;

    TM = _Toff - _Ton - _Tdead + _Tcomp; // Sul1996
    Udist = (_Udc*TM*CL_TS_INVERSE - _Vce0 - _Vd0) / 6.0; // Udist = (_Udc*TM/1e-4 - _Vce0 - _Vd0) / 6.0;
    // Udist = (_Udc*TM*TS_INVERSE) / 6.0;
    // Udist = 0.0;

    ia = SQRT_2_SLASH_3 * (       ial                              );
    ib = SQRT_2_SLASH_3 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );
    ic = SQRT_2_SLASH_3 * (-0.5 * ial - SIN_2PI_SLASH_3      * ibe );

    /* compute in abc frame */
    // ua = SQRT_2_SLASH_3 * (       ual                              );
    // ub = SQRT_2_SLASH_3 * (-0.5 * ual - SIN_DASH_2PI_SLASH_3 * ube );
    // uc = SQRT_2_SLASH_3 * (-0.5 * ual - SIN_2PI_SLASH_3      * ube );
    // ua += Udist * (2*sign(ia) - sign(ib) - sign(ic)) - 0.5*(Rce+Rdiode)*ia;
    // ub += Udist * (2*sign(ib) - sign(ic) - sign(ia)) - 0.5*(Rce+Rdiode)*ib;
    // uc += Udist * (2*sign(ic) - sign(ia) - sign(ib)) - 0.5*(Rce+Rdiode)*ic;
    // UAL_C_DIST = SQRT_2_SLASH_3      * (ua - 0.5*ub - 0.5*uc); // sqrt(2/3.)
    // UBE_C_DIST = 0.70710678118654746 * (         ub -     uc); // sqrt(2/3.)*sin(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)

    /* directly compute in alpha-beta frame */
    // CHECK the sign of the distortion voltage!
    // Sul把Udist视为补偿的电压（假定上升下降时间都已经知道了而且是“补偿”上去的）
    UAL_C_DIST = ual + sqrtf(1.5)*Udist*(2*sign(ia) - sign(ib) - sign(ic)) - 0.5*(Rce+Rdiode)*ial;
    UBE_C_DIST = ube + 3/sqrtf(2)*Udist*(             sign(ib) - sign(ic)) - 0.5*(Rce+Rdiode)*ibe; 

}
#endif

// 逆变器建模
double tri_stage;
void inverter_model(){

    // amplitude-invariant to power-invariant

    // 根据给定电压CTRL.ual_cmd和实际的电机电流ACM.ial，计算畸变的逆变器输出电压ACM.ual。
    #if INVERTER_NONLINEARITY
        // 考虑控制器和电机所用Clarke变换不同导致的系数变化
        InverterNonlinearity_SKSul96(CTRL.ual_cmd*sqrt(CLARKE_TRANS_TORQUE_GAIN), \
                                     CTRL.ube_cmd*sqrt(CLARKE_TRANS_TORQUE_GAIN), \
                                     ACM.ial, \
                                     ACM.ibe);
        ACM.ual = UAL_C_DIST;
        ACM.ube = UBE_C_DIST;

        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        DIST_AL = ACM.ual - CTRL.ual_cmd;
        DIST_BE = ACM.ube - CTRL.ube_cmd;

    #else
                            // 考虑控制器和电机所用Clarke变换不同导致的系数变化
        ACM.ual = CTRL.ual_cmd; //*sqrt(CLARKE_TRANS_TORQUE_GAIN); *AMPL2POW
        ACM.ube = CTRL.ube_cmd; //*sqrt(CLARKE_TRANS_TORQUE_GAIN); *AMPL2POW
    #endif

    #if MACHINE_TYPE == 2
        // 永磁电机仿真的输入电压是在dq系下的，所以要把alpha-beta系下的电压经过Park变换变为dq系下的电压。
        ACM.ud = AB2M(ACM.ual, ACM.ube, cos(ACM.theta_d), sin(ACM.theta_d));
        ACM.uq = AB2T(ACM.ual, ACM.ube, cos(ACM.theta_d), sin(ACM.theta_d));
    #endif



   // triangular carrier wave
    REAL carrier_freq = CL_TS_INVERSE; 
    REAL ceiling = 10;
    tri_stage = (int32)(4*ceiling*ACM.timebase*carrier_freq) % (int32)(4*ceiling); // https://pubs.opengroup.org/onlinepubs/9699919799/functions/fmod.html
    // if (tri_stage<ceiling)
    //     tri_stage = tri_stage;
    // else if (tri_stage<(3*ceiling) )
    //     tri_stage = 2*ceiling-tri_stage;
    // else
    //     tri_stage = tri_stage-4*ceiling;
}


