"""
你可以在这个文件随意实现自己的功能，
"""

import streamlit as st
import os
from rich import print
from pylab import np, plt, mpl
import _plugins.plugin_MOO.index as moo


"""
def func1():
    pass
def func2():
    pass
"""
import yaml
import utils.tuner as tuner
def user_pre_process(d_sim, user_config):
    # TODO： 你可以任意更改下面的内容
    """
    eg:
    func1()
    func2()
    ...
    """

    def _update(user_no, yaml_fname, d_sim, user_config):
        if d_sim['user.who_is_user'] == user_no:
            with open(os.path.dirname(__file__) + yaml_fname, encoding='utf-8') as f: 
                user_config_overwrite = yaml.load(f, Loader=yaml.FullLoader)
                # print(user_config_overwrite)
                user_config['default_var_list'].extend(user_config_overwrite['default_var_list'])
                user_config['signal_library'].extend(user_config_overwrite['signal_library'])
                d_sim.update(user_config_overwrite['simulation'])
                print(f"[user_script_main.py] overwrite d_sim using {user_no}'s configuration file {yaml_fname}")
        return d_sim
    d_sim = _update(101976, '/user_config_cjh.yaml', d_sim, user_config)
    d_sim = _update(102209, '/user_config_xm.yaml', d_sim, user_config)
    d_sim = _update(2023231060, '/user_config_yzz.yaml', d_sim, user_config)
    d_sim = _update(2021531030, '/user_config_gzt.yaml', d_sim, user_config)
    d_sim = _update(2023231051, '/user_config_wb.yaml', d_sim, user_config)
    d_sim = _update(970308, '/user_config_wb2.yaml', d_sim, user_config)
    d_sim = _update(224, '/user_config_bezier.yaml', d_sim, user_config)
    d_sim = _update(2024233243, '/user_config_zjl.yaml', d_sim, user_config)
    d_sim = _update(2022231110, '/user_config_qian.yaml', d_sim, user_config)
    d_sim = _update(201314, '/user_config_cury.yaml', d_sim, user_config)

    # 调参（一定要先 load user_config_xxx.yaml 再调参）
    VLBW_Hz, d_currentKp, d_currentKi, q_currentKp, q_currentKi, speedKp, speedKi, speedKFB, (mag, phase, omega), input, output = tuner.InstaSPIN_series_PI_tuner(
        d_sim['FOC.delta'],
        d_sim['FOC.CLBW_HZ'],
        d_sim['init.Ld'],
        d_sim['init.Lq'],
        d_sim['init.R'],
        d_sim['init.Js'],
        d_sim['init.npp'],
        d_sim['init.KE'], bool_render=False)
    st.subheader('TI tuning results:')
    st.write(f'{VLBW_Hz = :.4g}' + f'| {speedKp = :.4g}' + f'| {speedKi = :.4g}' + f'| {q_currentKp = :.4g}' + f'| {q_currentKi = :.4g}')
    d_sim['CL.SERIES_KP_D_AXIS'] = d_currentKp
    d_sim['CL.SERIES_KI_D_AXIS'] = d_currentKi
    d_sim['CL.SERIES_KP_Q_AXIS'] = q_currentKp
    d_sim['CL.SERIES_KI_Q_AXIS'] = q_currentKi
    d_sim['VL.SERIES_KP'] = speedKp
    d_sim['VL.SERIES_KI'] = speedKi
    d_sim['user.VL_FEEDBACK_KFB'] = speedKFB

    # needed for Bezier optimize.py???
    if not os.path.exists(os.path.dirname(__file__)+"/frameworkCodes/plugin_moo_args.txt"):
        with open(os.path.dirname(__file__)+"/frameworkCodes/plugin_moo_args.txt", 'w') as f:
            f.write(
                "0,0\n"
                "274.8176831125652,7.403814662963234\n"
                "352.66071190426777,4.228145678804964\n"
                "590.3660674883257,3.6477912963992822\n"
                "618.0182761302431,11.039199652252561\n")

    return d_sim

def user_bezier_super_config(motor_name, data, user_extend_settings, super_config_C_content, super_config_header_content):
    if user_extend_settings:
        if user_extend_settings.get("bezier_moo"):
            super_config_header_content += (
                f"\n\n\n#define ARGS_PATH \"../plugin_moo_args.txt\"\n"
            )
        elif user_extend_settings.get("bezier_C_save"):
            # super_config_header_content += (user_extend_settings["bezier_C_save"])
            with open(os.path.dirname(__file__) + '/frameworkCodes/c/simuser_bezier.h', 'r', encoding='utf-8') as f:
                lines = f.readlines()
                # print(lines)
                if '#endif' in lines[-1] and '#endif' in lines[-2]:
                    lines = lines[:-7] + ["#if PC_SIMULATION==FALSE\n"+user_extend_settings["bezier_C_save"]+"\n#endif\n"] + lines[-2:]
                # print(lines)
            with open(os.path.dirname(__file__) + '/frameworkCodes/c/simuser_bezier.h', 'w', encoding='utf-8') as f:
                f.writelines(lines)
    else:
        if data['user'].get('bezier_order'):
            if not os.path.exists(os.path.dirname(__file__) + f"/frameworkCodes/acmsimc_bezier_points"):
                os.makedirs(os.path.dirname(__file__) + f"/frameworkCodes/acmsimc_bezier_points")
            file_path = os.path.dirname(__file__) + f"/frameworkCodes/acmsimc_bezier_points/{motor_name}-{data['user']['bezier_order']}-{data['user']['bezier_order_current']}.txt"
            if not os.path.exists(file_path):
                # raise FileNotFoundError(f"文件不存在: {file_path}")
                super_config_header_content += (
                    f"\n\n\n#define ARGS_PATH \"../acmsimc_bezier_points/{motor_name}-{data['user']['bezier_order']}-{data['user']['bezier_order_current']}.txt\"\n"
                )
                print(f"文件不存在: {file_path}")
                print(f"继续运行，bezier控制不可用！")
            else:
                print(f"文件存在: {file_path}")
                super_config_header_content += (
                    f"\n\n\n#define ARGS_PATH \"../acmsimc_bezier_points/{motor_name}-{data['user']['bezier_order']}-{data['user']['bezier_order_current']}.txt\"\n"
                )
    return super_config_C_content, super_config_header_content


def user_py_post_process(d_sim, simulation_result):
    # TODO： 你可以任意更改下面的内容
    """
    eg:
    func1()
    func2()
    ...
    """
    pass


def user_cplot_post_process(d_sim, user_plot_config, post_run):
    # TODO： 你可以任意更改下面的内容
    if post_run:
        if not os.path.exists(os.path.join(os.path.dirname(__file__), 'session_state.yaml')):
            raise FileNotFoundError(f"文件不存在: {os.path.join(os.path.dirname(__file__), 'session_state.yaml')}")
        with open(os.path.join(os.path.dirname(__file__), 'session_state.yaml'), 'r') as f:
            session_state = yaml.load(f, Loader=yaml.FullLoader)
            bool_OverwriteUserConfigYaml = session_state['bool_OverwriteUserConfigYaml']
    else:
        bool_OverwriteUserConfigYaml = st.session_state.bool_OverwriteUserConfigYaml

    def _update(user_no, yaml_fname, d_sim, user_plot_config):
        if d_sim['user.who_is_user'] == user_no:
            with open(os.path.dirname(__file__) + yaml_fname, encoding='utf-8') as f:
                user_plot_config_overwrite = yaml.load(f, Loader=yaml.FullLoader)
                user_plot_config['signal_library'].extend(user_plot_config_overwrite['signal_library'])
                for key, value in user_plot_config_overwrite['cplot'].items():
                    if key == 'subplot':
                        user_plot_config['cplot'][key].extend(value)
                    else:
                        user_plot_config['cplot'][key] = value
            if bool_OverwriteUserConfigYaml:
                user_plot_config['cplot']['subplot'] = user_plot_config_overwrite['cplot']['subplot']
    _update(101976, '/user_config_cjh.yaml', d_sim, user_plot_config)
    _update(102209, '/user_config_xm.yaml', d_sim, user_plot_config)
    _update(2023231060, '/user_config_yzz.yaml', d_sim, user_plot_config)
    _update(2021531030, '/user_config_gzt.yaml', d_sim, user_plot_config)
    _update(2023231051, '/user_config_wb.yaml', d_sim, user_plot_config)
    _update(970308, '/user_config_wb2.yaml', d_sim, user_plot_config)
    _update(224, '/user_config_bezier.yaml', d_sim, user_plot_config)
    _update(240828, '/user_config_gen.yaml', d_sim, user_plot_config)
    _update(2022231110, '/user_config_gen.yaml', d_sim, user_plot_config)
    _update(2024233243, '/user_config_zjl.yaml', d_sim, user_plot_config)
    _update(201314, '/user_config_cury.yaml', d_sim, user_plot_config)

    return

def read_points_from_txt(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"文件不存在: {path}")
    with open(path, 'r') as f:
        lines = f.readlines()
        st.write(lines)
        points = []
        for line in lines:
            points.append(list(map(float, line.split(','))))
    return points

def plot_Bzier(points, order):
        Bzier, Gain = moo.get_Bzier_curve(points[0:order + 1])
        plt.style.use('bmh')
        mpl.rc('font', family='Times New Roman', size=10.0)
        mpl.rc('legend', fontsize=10)
        mpl.rcParams['text.color'] = 'black'
        mpl.rcParams['axes.labelcolor'] = 'black'
        mpl.rcParams['xtick.color'] = 'black'
        mpl.rcParams['ytick.color'] = 'black'	
        mpl.rcParams['lines.linewidth'] = 0.75
        mpl.rcParams['mathtext.fontset'] = 'stix'
        fig, axes = plt.subplots(nrows=2, ncols=1, dpi=150,
                                facecolor='w', sharex=True)
        axes[0].plot(Bzier[0][0], Bzier[0][1], label=f'Bzier point')
        axes[0].legend()
        axes[1].plot(Gain[0][0], Gain[0][1], label=f'Gain point')
        axes[1].legend()
        return fig

def user_cplot_post_plot_process(d_sim):
    if d_sim['user.who_is_user'] == 224:
        st.write("贝塞尔曲线：")
        # read points
        bezier_points = read_points_from_txt(os.path.dirname(__file__) + f"/frameworkCodes/acmsimc_bezier_points/{st.session_state.user_selected_motor}-{st.session_state.d_sim['user.bezier_order']}-{st.session_state.d_sim['user.bezier_order_current']}.txt")
        st.write(bezier_points)
        st.pyplot(plot_Bzier(bezier_points, st.session_state.d_sim['user.bezier_order']))


"""Controller"""
def example_controller():
    pass
