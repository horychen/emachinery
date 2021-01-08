#include "ACMSim.h"

double load_model(){
    double Tload;

    // Tload = 0.0001 * ACM.rpm + 0.002 * cos(3*ACM.x[3]); // this load causes zero speed oscillation in controller 
    // Tload = 0.005*sign(ACM.rpm);
    // Tload = LOAD_TORQUE*sign(ACM.rpm) + 0*sin(2*M_PI*1*ACM.timebase);

    Tload = LOAD_TORQUE*sign(ACM.rpm) + VISCOUS_COEFF*ACM.rpm*RPM_2_RAD_PER_SEC;

    // Tload = 0.0;
        // EPA-2019-0568.R1
        // double friction;
        // if(fabs(ACM.rpm*RPM_2_RAD_PER_SEC)<0.08){
        //     if(ACM.rpm>0){
        //         friction = 0.0125 / 0.08 * ACM.rpm*RPM_2_RAD_PER_SEC; // <- using elec rad/s is wrong
        //     }else{
        //         friction = - 0.0125 / 0.08 * ACM.rpm*RPM_2_RAD_PER_SEC;
        //     }
        // }else{
        //     if(ACM.rpm>0){
        //         friction = 0.0025 + 0.0025 * ACM.rpm*RPM_2_RAD_PER_SEC;
        //     }else{
        //         friction = - 0.0025 - 0.0025 * ACM.rpm*RPM_2_RAD_PER_SEC;
        //     }
        // }
        // ACM.Tload += friction;

        // 齿轮箱背隙建模(在背隙形程内，负载为零，否则突然加载)
    return Tload;
}
