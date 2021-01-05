#include "ACMSim.h"
#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

// 仿真电机结构体变量声明
struct SynchronousMachineSimulated ACM;

// 仿真电机结构体的初始化
void Machine_init(){

    ACM.timebase = 0.0;
    ACM.Ts = MACHINE_TS;

    int i;
    for(i=0;i<NUMBER_OF_STATES;++i){
        ACM.x[i] = 0.0;
        ACM.x_dot[i] = 0.0;
    }

    // 电机转子初始位置不为零。
    // ACM.x[???] = 0.0 * 19.0 / 180.0 * M_PI;

    ACM.omg_elec = 0.0;
    ACM.rpm = 0.0;
    ACM.rpm_cmd = 0.0;
    ACM.rpm_deriv_cmd = 0.0;
    ACM.TLoad = 0.0;
    ACM.Tem = 0.0;

    ACM.R  = PMSM_RESISTANCE;
    ACM.Ld = PMSM_D_AXIS_INDUCTANCE;
    ACM.Lq = PMSM_Q_AXIS_INDUCTANCE;

    // 需要把恒幅值变换下的磁链值转换为恒功率变换磁链值
    // PMSM_PERMANENT_MAGNET_FLUX_LINKAGE 是指永磁体磁链的幅值，所以在恒功率坐标系下仿真时要经过恒功率变换，Clarke变换后得到的永磁体磁链向量的长度长于相坐标系下的，所以差一个系数sqrt(1.5)。
    // ACM.KE = PMSM_PERMANENT_MAGNET_FLUX_LINKAGE * AMPL2POW; // Vs/rad 
    ACM.KE = PMSM_PERMANENT_MAGNET_FLUX_LINKAGE; // Vs/rad 

    ACM.npp = MOTOR_NUMBER_OF_POLE_PAIRS;
    ACM.npp_inv = 1.0 / ACM.npp;
    ACM.Js = MOTOR_SHAFT_INERTIA * (1.0+LOAD_INERTIA);
    ACM.mu_m = ACM.npp/ACM.Js;

    ACM.ual = 0.0;
    ACM.ube = 0.0;
    ACM.ial = 0.0;
    ACM.ibe = 0.0;

    ACM.theta_d = 0.0;
    ACM.theta_d_accum = 0.0;
    ACM.ud = 0.0;
    ACM.uq = 0.0;
    ACM.id = 0.0;
    ACM.iq = 0.0;
}

// 同步电机的动态方程（恒?Clarke变换）
void SM_Dynamics(double t, double *x, double *fx){
    // 记电机的动态方程为 d/dt x = f(x)
    // 本函数内出现的 fx 均表示电机状态的时间导数。
    // 定义 dq 系为转子磁场定向坐标系，即永磁体产生的磁场向量的方向为 d轴。
    // 0：d轴电流 [A]
    // 1：q轴电流 [A]
    // 2：电气转子转速 [elec. rad/s]
    // 3：电气转子d轴位置（即磁极位置） [elec. rad]

    // 电磁子系统
    fx[0] = (ACM.ud - ACM.R * x[0] + x[2]*ACM.Lq*x[1]) / ACM.Ld; // current-d
    fx[1] = (ACM.uq - ACM.R * x[1] - x[2]*ACM.Ld*x[0] - x[2]*ACM.KE) / ACM.Lq; // current-q

    // 机械子系统
    // 电磁转矩 Tem 计算
    ACM.Tem = CLARKE_TRANS_TORQUE_GAIN * ACM.npp * (x[1]*ACM.KE + (ACM.Ld - ACM.Lq)*x[0]*x[1]); // AMPL2POW*AMPL2POW = CLARKE_TRANS_TORQUE_GAIN
    fx[2] = (ACM.Tem - ACM.TLoad)*ACM.mu_m; // elec. angular rotor speed
    fx[3] = x[2];                           // elec. angular rotor position (bounded)
    fx[4] = x[2];                           // elec. angular rotor position (accumulated)
}
// 四阶龙格库塔法
void RK4(double t, double *x, double hs){
    #define NS NUMBER_OF_STATES

    double k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    double fx[NS];
    int i;

    MACHINE_DYNAMICS(t, x, fx); // timer.t,
    for(i=0;i<NS;++i){        
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }
    
    MACHINE_DYNAMICS(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }
    
    MACHINE_DYNAMICS(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }
    
    MACHINE_DYNAMICS(t, xk, fx); // timer.t+hs, 
    for(i=0;i<NS;++i){        
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0;

        // derivatives
        ACM.x_dot[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0 / hs; 
    }
    #undef NS
}



// 调用龙格库塔法求解电机状态，并为接口变量赋值
int machine_simulation(){

    // 调用龙格库塔法求解电机状态，电机状态为 ACM.x = [d轴电流，q轴电流，电气转子转速，电气转子位置]
    // 电机输入分别为 d轴电压 ACM.ud 和 q轴电压 ACM.ud。
    RK4(ACM.timebase, ACM.x, ACM.Ts);

    // 电机转子位置接口
    ACM.theta_d = ACM.x[NUMBER_OF_STATES-2];
    ACM.theta_d_accum = ACM.x[NUMBER_OF_STATES-1];
    // 转子d轴位置限幅
    if(ACM.theta_d > M_PI) ACM.theta_d -= 2*M_PI;
    if(ACM.theta_d < -M_PI) ACM.theta_d += 2*M_PI; // 反转！
    // 将状态变量x[3]和接口变量theta_d的值进行同步
    ACM.x[NUMBER_OF_STATES-2] = ACM.theta_d;

    // 电机电流接口（需要把恒功率变换转换为恒幅值变换）
    ACM.id  = ACM.x[0]; // * POW2AMPL;
    ACM.iq  = ACM.x[1]; // * POW2AMPL;
    ACM.ial = MT2A(ACM.id, ACM.iq, cos(ACM.theta_d), sin(ACM.theta_d));
    ACM.ibe = MT2B(ACM.id, ACM.iq, cos(ACM.theta_d), sin(ACM.theta_d));

    // 电机转速接口
    ACM.omg_elec = ACM.x[NUMBER_OF_STATES-3]; // 电气转速 [elec. rad/s]
    ACM.rpm = ACM.x[NUMBER_OF_STATES-3] * 60 / (2 * M_PI * ACM.npp); // 机械转速 [mech. rev/min]

    // 简单的程序跑飞检测，比如电机转速无穷大则停止程序
    if(isNumber(ACM.rpm)){
        return FALSE;
    }else{
        printf("ACM.rpm is %g\n", ACM.rpm);
        return TRUE;        
    }
}

#endif
