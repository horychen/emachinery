"""
example
"""

import streamlit as st
import st_main
import st_interact as interact
import os
import yaml
import pandas as pd
from pylab import np, plt, mpl
import user_script_main
from math import ceil

def load_yaml():
    with open(os.path.dirname(__file__)+'/../../user_config.yaml', encoding='utf-8') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    return config

def x_y_plot(user_selected_motor, d_sim, x, y): # args = [d_sim, '{number_channels_per_column} x {number_columns}']
    if d_sim is None:
        with open(os.path.dirname(__file__) + f'/../../frameworkCodes/dat/{user_selected_motor}_d_sim.yaml', 'r', encoding='utf-8') as f:
            d_sim = yaml.load(f, Loader=yaml.FullLoader)

    user_plot_config = load_yaml()
    user_script_main.user_cplot_post_process(d_sim, user_plot_config, False)
    base_path = os.path.dirname(__file__)+"/../../frameworkCodes"
    mpl.style.use('ggplot')

    # 字体
    for key, value in user_plot_config['config']['mpl'].items():
        mpl.rcParams[key] = value
    for key, value in user_plot_config['config']['plt'].items():
        plt.rcParams[key] = value

    # 实用函数
    def cyclic_generator(list_of_things):
        N, i = len(list_of_things), 0
        while True:
            yield list_of_things[i]
            i += 1
            if i > (N-1):
                i = 0

    # 画图风格
    cjh_linestyles = [
        '-', '--', (0, (3, 1, 1, 1)), ':', '-.',
        '-', '--', (0, (3, 1, 1, 1)), ':', '-.',
        '-', '--', (0, (3, 1, 1, 1)), ':', '-.',
    ]
    cjh_linestyle = cyclic_generator(cjh_linestyles)
    # 颜色设置叁/叁
    cjh_colors = user_plot_config['config']['cjh_colors']
    cjh_color = cyclic_generator(cjh_colors)
    # 注意，不可以做 list(self.cjh_color)，因为这是一个无止境循环的发生器，会卡住的。。。

    # 读取数据
    if not os.path.exists(base_path+'/dat'):
        os.makedirs(base_path+'/dat')
    data_file_name = base_path+'/dat/'+str(user_selected_motor)+'.dat'
    if not os.path.exists(data_file_name):
        print(f"[cplot.py] Data file {data_file_name} does not exist. Please run main first!")
        return []
    with open(data_file_name, 'r') as f:
        if not f.read():
            print(f"[cplot.py] Data file {data_file_name} is empty. Please run main first!")
            return []
    df_profiles = pd.read_csv(data_file_name, na_values=['1.#QNAN' , '-1#INF00', '-1#IND00'])
    try:
        no_samples = df_profiles.shape[0]
    except:
        print(f"[cplot.py] Data file {data_file_name} is empty. Please run main first!")
        return []

    DOWN_SAMPLE = 1
    CL_TS = 1e-4
    time = np.arange(1, no_samples+1) * DOWN_SAMPLE * CL_TS

    # 读取x,y，画图
    figs = []
    for i in range(len(x)):
        fig, ax = plt.subplots(figsize=(12, 8))
        x_signal = df_profiles[x[i]]
        y_signal = df_profiles[y[i]]
        ax.set_xlabel(x[i])
        ax.set_ylabel(y[i])
        ax.plot(x_signal, y_signal)
        figs.append(fig)
    return figs

def convert(value):
    convert_value = []
    convert_value.append(value[0])
    for i in range(1, len(value)):
        if value[i] == convert_value[-1]:
            continue
        else:
            convert_value.append(value[i])
    return convert_value

def main(d_sim, user_config):
    st.title("Plugin example")
    interact.c_save_run_module(d_sim, user_config)
    interact.c_simulation_visual_module(d_sim)
    x = st.multiselect('Select x', user_config['signal_library'])
    y = st.multiselect('Select y', user_config['signal_library'])
    figs = x_y_plot(st.session_state.user_selected_motor, d_sim, x, y)
    for fig in figs:
        st.pyplot(fig)
