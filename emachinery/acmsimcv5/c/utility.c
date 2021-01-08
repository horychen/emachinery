#include "ACMSim.h"
// 功能函数
// 写变量名到文件
#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.rpm_cmd,ACM.rpm,sm.omg_elec*RAD_PER_SEC_2_RPM,harnefors.omg_elec*RAD_PER_SEC_2_RPM,ACM.x[3]/M_PI*180,harnefors.theta_d/M_PI*180,PIDQ(0),harnefors.deriv_id,PIDQ(1),harnefors.deriv_iq,CTRL.iq_cmd,ACM.iq,CTRL.id_cmd,ACM.id,ACM.x[4]/M_PI*180,ACM.ud,ACM.uq\n"
#define DATA_DETAILS ACM.rpm_cmd,ACM.rpm,sm.omg_elec*RAD_PER_SEC_2_RPM,harnefors.omg_elec*RAD_PER_SEC_2_RPM,ACM.x[3]/M_PI*180,harnefors.theta_d/M_PI*180,PIDQ(0),harnefors.deriv_id,PIDQ(1),harnefors.deriv_iq,CTRL.iq_cmd,ACM.iq,CTRL.id_cmd,ACM.id,ACM.x[4]/M_PI*180,ACM.ud,ACM.uq

void write_header_to_file(FILE *fw){
    printf("%s\n", DATA_FILE_NAME);

    fprintf(fw, DATA_LABELS);

    {
        // 将除了变量数据和变量名以外的信息写入文件“info.dat”，包括采样时间，降采样倍数，数据文件名。
        FILE *fw2;
        fw2 = fopen("../dat/info.dat", "w");
        fprintf(fw2, "CL_TS,DOWN_SAMPLE,DATA_FILE_NAME\n");
        fprintf(fw2, "%g, %d, %s\n", CL_TS, DOWN_SAMPLE, DATA_FILE_NAME);
        fclose(fw2);
    }
}
// 写变量值到文件
void write_data_to_file(FILE *fw){
    static int bool_animate_on = FALSE;
    static int j=0,jj=0; // j,jj for down sampling

    // if(CTRL.timebase>20)
    {
        if(++j == DOWN_SAMPLE)
        {
            j=0;
            fprintf(fw, DATA_FORMAT, DATA_DETAILS);
        }
    }
}
// 符号函数
double sign(double x){
    return (x > 0) - (x < 0);    
}
// 浮点数的绝对值函数
double fabs(double x){
    return (x >= 0) ? x : -x;
}
// 判断是否为有效浮点数
int isNumber(double x){
    // This looks like it should always be TRUE, 
    // but it's FALSE if x is an NaN (1.#QNAN0).
    return (x == x); 
    // see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}


//低通滤波器：测量值，上一步的滤波器输出，时间常数的倒数
REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv){
    return y_tminus1 + CL_TS * time_const_inv * (x - y_tminus1);
}
