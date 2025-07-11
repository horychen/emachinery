import numpy as np
import os
import script_SpeedSweepingFreqData2BodeAndBandwidth   as SFDBB
import script_CurrentSweepingFreqData2BodeAndBandwidth as CFDBB


'''Performance Calculation'''
def cal_CLBW_VLBW(cplot_data, d_sim):
    
    # 通过CJH脚本计算出来的Bandwidth
    FOC_CLBW      = 'N/A'
    FOC_VLBW      = 'N/A'
    FOC_CLBW_calc = 'N/A'
    FOC_VLBW_calc = 'N/A'

    # TODO:这里最好有who_is_user反馈是最好的！
    if 'user.bool_apply_WC_tunner_for_speed_loop' in d_sim:
        if d_sim['user.bool_apply_WC_tunner_for_speed_loop'] == True:
            max_CLBW_PER_min_CLBW = d_sim['user.max_CLBW_PER_min_CLBW']
            max_CLBW = d_sim['user.zeta'] * d_sim['user.omega_n'] * 4
            min_CLBW = d_sim['user.zeta'] * d_sim['user.omega_n'] * 2
            
            if max_CLBW_PER_min_CLBW > 1:
                raise ValueError('max_CLBW_PER_min_CLBW > 1 change change it !?')
            else:
                # 通过一个小于1的比例系数来选取电流环带宽
                FOC_CLBW = max_CLBW_PER_min_CLBW * max_CLBW + (1-max_CLBW_PER_min_CLBW) * min_CLBW
        else:
            FOC_CLBW = d_sim['CL.SERIES_KP_Q_AXIS'] / d_sim['init.Lq'] # Kp/Lq is the current bandwidth

        if d_sim['user.bool_apply_WC_tunner_for_speed_loop'] == True:
            FOC_VLBW = d_sim['user.omega_n'] * np.sqrt(1 - 2 * d_sim['user.zeta'] **2 + np.sqrt(4 * d_sim['user.zeta'] ** 4 - 4 * d_sim['user.zeta'] ** 2 + 2))
        else:
            FOC_CLBW = d_sim['CL.SERIES_KP_Q_AXIS'] / d_sim['init.Lq']
            Gain = d_sim['FOC.delta'] + 2.16 * np.exp(-1 * d_sim['FOC.delta'] / 2.8) - 1.86 # 从CLBW计算VLBW的估计公式
            FOC_VLBW = FOC_CLBW / Gain
    else:
        FOC_CLBW = d_sim['CL.SERIES_KP_Q_AXIS'] / d_sim['init.Lq']
        Gain = d_sim['FOC.delta'] + 2.16 * np.exp(-1 * d_sim['FOC.delta'] / 2.8) - 1.86 # 从CLBW计算VLBW的估计公式
        FOC_VLBW = FOC_CLBW / Gain
        # Bezier暂时没有计算理论带宽的公式，这里先NA
        if 'user.bezier_order' in d_sim:
            FOC_CLBW = 'N/A'
            FOC_VLBW = 'N/A'

    #TODO: 电流环的带宽需要有判断！原脚本要求txt文件！确保d_sim的key和RPM值是对应上的，这一点不同用户可能用的值不一样！
    #TODO: 这里先是往txt写数据，又从txt读数据，这个过程是不是有点多余，yes，所以可以直接把这三个值传进来，不用txt文件，简化一些
    if (d_sim['user.bool_apply_sweeping_frequency_excitation'] == True) :
        if  (d_sim['user.bool_sweeping_frequency_for_speed_loop'] == True):
            ACM_RPM = cplot_data['ACM.varOmega * MECH_RAD_PER_SEC_2_RPM']
            cmd_RPM = cplot_data['(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM']
            simulation_time = cplot_data['(*CTRL).timebase']
            # File name for saving the result
            output_file = os.path.dirname(__file__) + '/data/SpeedSweepingData.txt'
            #TODO: DataFiles居然要手动添加，得加到git里面去？
            # Writing to the file
            with open(output_file, 'w') as f:
                # Writing the header information
                f.write("[Signal Names] MAYBE WROING\n")
                f.write("SamplePeriod: 50 μs\n")
                f.write("SampleTime: 10.237 s\n")
                f.write("signal1  ID:0x23040000  Name:Speed loop reference  Unit:rpm\n")
                f.write("signal2  ID:0x60690000  Name:Speed loop feedback  Unit:rpm\n\n")
                # Writing the data
                f.write("Time(s)  signal1  signal2\n")
                for t, cmd_rpm, acm_rpm in zip(simulation_time, cmd_RPM, ACM_RPM):
                    f.write(f"{t:.6f}  {cmd_rpm:.6f}  {acm_rpm:.6f}\n")
                print(f">>> Frequency Data has been written to {output_file} <<<")
            freq_at_minus_3dB, freq_at_minus_3dB_rad_s = SFDBB.BodeAndBandwidthCalc_BasedOn_CJHScript( d_sim['user.CMD_SPEED_SINE_HZ_CEILING'], d_sim['user.CMD_SPEED_SINE_HZ']+1 );
            FOC_VLBW_calc = freq_at_minus_3dB_rad_s;
        else:
            if d_sim['user.bool_sweeping_frequency_for_current_loop_iD'] == True:
                ACM_current = cplot_data['(*CTRL).i->iDQ[0]']
                cmd_current = cplot_data['(*CTRL).i->cmd_iDQ[0]']
            else:
                ACM_current = cplot_data['(*CTRL).i->iDQ[1]']
                cmd_current = cplot_data['(*CTRL).i->cmd_iDQ[1]']

            simulation_time = cplot_data['(*CTRL).timebase']
            # File name for saving the result
            output_file = os.path.dirname(__file__) + '/data/CurrentSweepingData.txt'
            #TODO: DataFiles居然要手动添加，得加到git里面去？
            # Writing to the file
            with open(output_file, 'w') as f:
                # Writing the header information
                f.write("[Signal Names] MAYBE WROING\n")
                f.write("SamplePeriod: 50 μs\n")
                f.write("SampleTime: 10.237 s\n")
                f.write("signal1  ID:0x23040000  Name:Current loop reference Unit:A\n")
                f.write("signal2  ID:0x60690000  Name:Current loop feedback  Unit:A\n\n")
                # Writing the data
                f.write("Time(s)  signal1  signal2\n")
                for t, cmd_current, acm_current in zip(simulation_time, cmd_current, ACM_current):
                    f.write(f"{t:.6f}  {cmd_current:.6f}  {acm_current:.6f}\n")
                print(f">>> Frequency Data has been written to {output_file} <<<")
            freq_at_minus_3dB, freq_at_minus_3dB_rad_s = CFDBB.BodeAndBandwidthCalc_BasedOn_CJHScript(d_sim['user.CMD_SPEED_SINE_HZ_CEILING'], d_sim['user.CMD_SPEED_SINE_HZ']+1);
            FOC_CLBW_calc = freq_at_minus_3dB_rad_s;
    else:
        pass

    performance_index_data = {
        'CLBW[rad/s]': [f'{FOC_CLBW_calc:.2f}' if FOC_CLBW_calc != 'N/A' else 'N/A'        , f'{FOC_CLBW:.2f}' if FOC_CLBW != 'N/A' else 'N/A'],
        'CLBW[Hz]'   : [f'{FOC_CLBW_calc/2/np.pi:.2f}' if FOC_CLBW_calc != 'N/A' else 'N/A', f'{FOC_CLBW/2/np.pi:.2f}' if FOC_CLBW != 'N/A' else 'N/A'],
        'VLBW[rad/s]': [f'{FOC_VLBW_calc:.2f}' if FOC_VLBW_calc != 'N/A' else 'N/A'        , f'{FOC_VLBW:.2f}' if FOC_VLBW != 'N/A' else 'N/A'],
        'VLBW[Hz]'   : [f'{FOC_VLBW_calc/2/np.pi:.2f}' if FOC_VLBW_calc != 'N/A' else 'N/A', f'{FOC_VLBW/2/np.pi:.2f}' if FOC_VLBW != 'N/A' else 'N/A'],
    }

    return performance_index_data
