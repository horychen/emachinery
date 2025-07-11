import pandas as pd
import numpy as np

def overshoot_Ptime_Stime_calc(cplot_data, d_sim):
    # 用的实际电机转速，不错！
    ACM_RPM = cplot_data['ACM.varOmega * MECH_RAD_PER_SEC_2_RPM']
    cmd_RPM = cplot_data['(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM']

    # build a time axis
    simulation_time = 1

    cmd_RPM_max = cmd_RPM.max()
    cmd_RPM_min = cmd_RPM.min() # negative command RPM maximum value

    ACM_RPM_max = ACM_RPM.max()
    print('RPM = ', ACM_RPM_max)
    ACM_RPM_min = ACM_RPM.min() # negative RPM maximum value

    overshoot = (ACM_RPM_max - cmd_RPM_max) / cmd_RPM_max
    if d_sim['user.zeta'] < 1:
        overshoot_theory = np.exp(-np.pi * d_sim['user.zeta'] / np.sqrt(1 - d_sim['user.zeta']**2))
    else:
        overshoot_theory = 0

    # 找到系统响应的峰值时间
    peak_index = np.argmax(ACM_RPM)
    print('peak_index = ', peak_index)
    global_machine_times = get_simulation_time(d_sim)

    # 峰值时间
    peak_time = global_machine_times[peak_index]

    # 调节时间以2%为边界
    # 找到系统响应经过峰值时间后，第一次进入到响应2%范围内的时间点
    tolerance = 0.02
    upper_limit = cmd_RPM_max * (1 + tolerance)
    lower_limit = cmd_RPM_max * (1 - tolerance)
    settling_time = None
    for t, y in zip(global_machine_times, ACM_RPM):
        if t > peak_time and y > lower_limit and y < upper_limit:
            settling_time = t
            break
    if 0 < d_sim['user.zeta'] < 1:
        settling_time_theory = 4 / d_sim['user.zeta'] / d_sim['user.omega_n']
    else:
        settling_time_theory = 0
    # 如果settling_time为None，转换为字符串'None'
    if settling_time is None:
        settling_time = 0
    
    performance_index_data = {
    'overshoot': [f'{overshoot:.2%}', f'{overshoot_theory:.2%}'],
    'peak_time': [f'{peak_time:.4f}', 'N/A'],
    'settling_time': [f'{settling_time:.4f}', f'{settling_time_theory:.4f}'],
    }
    return performance_index_data


def get_simulation_time(d_sim):
    CL_TS = d_sim['sim.CLTS']
    NUMBER_OF_STEPS = d_sim['sim.NUMBER_OF_STEPS']
    simulation_time_max = CL_TS * NUMBER_OF_STEPS
    simulation_time_array = np.arange(0, simulation_time_max, CL_TS)
    return simulation_time_array