import numpy as np

eval_dim = 3
popsize = 190
number_of_iterations = 1000
optimize_goal = ['steady state error', 'energy cost', 'rising time']

def evaluate_simResults(simulation_result, d_sim, max_p_value=None):
    data_lib = ['(*CTRL).i->cmd_varOmega * MECH_RAD_PER_SEC_2_RPM',
                '(*CTRL).i->varOmega * MECH_RAD_PER_SEC_2_RPM',
                '(*CTRL).i->iDQ[1]']
    data = [simulation_result[key].to_list() for key in data_lib]
    # speed_control_error = command - feedback in r/min
    speed_control_error = np.abs(np.array(data[0]) - np.array(data[1])) # speed error in rpm
    q_axis_current = np.array(data[2]) # q-axis current
    try:
        fitness = eval_result = [
            # 跟踪误差 tracking # 从 SECONDS_LOAD_DISTURBANCE+0.1 开始积分
            float(
                np.sum(np.abs(
                    speed_control_error[:int(d_sim['user.bezier_seconds_load_disturbance']/d_sim['sim.CLTS'])]
                    )) * d_sim['sim.CLTS']
                ),
            # 热损耗 # 从SECONDS_STEP_COMMAND开始积分
            float(
                np.sum(q_axis_current**2) * d_sim['sim.CLTS']
                ),
            # 抗扰性能 disturbance rejection # 从 SECONDS_LOAD_DISTURBANCE 开始积分
            float(
                np.sum(
                    np.abs(speed_control_error)[int(d_sim['user.bezier_seconds_load_disturbance']/d_sim['sim.CLTS']):]
                    ) * d_sim['sim.CLTS']
                ),
            # todo: annotation below to ignore the max_p_value
            # max_p_value
        ]
    except Exception as e:
        raise e
    return eval_result

