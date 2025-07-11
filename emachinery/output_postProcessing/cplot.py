from ast import arg
import os, yaml, pandas as pd
from pylab import np, plt, mpl
from math import ceil
def load_yaml():
    with open(os.path.dirname(__file__)+'/../user_config.yaml', encoding='utf-8') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    return config

import user_script_main

def read_data(user_selected_motor):
    base_path = os.path.dirname(__file__)+"/../frameworkCodes"
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
    return df_profiles


def main(user_selected_motor, *args, post_run=False): # args = [d_sim, '{number_channels_per_column} x {number_columns}']
    number_channels_per_column = None
    number_columns = 1
    # print(args)
    if args is not None:
        if len(args)==1:
            d_sim = args[0]
        else:
            d_sim = args[0]
            temp_list = args[1].split('x')
            number_channels_per_column = int(temp_list[0])
            if len(temp_list)>1:
                number_columns = int(args[1].split('x')[1])

    if d_sim is None:
        with open(os.path.dirname(__file__) + f'/../frameworkCodes/dat/{user_selected_motor}_d_sim.yaml', 'r', encoding='utf-8') as f:
            d_sim = yaml.load(f, Loader=yaml.FullLoader)

    user_plot_config = load_yaml()
    user_script_main.user_cplot_post_process(d_sim, user_plot_config, post_run)
    base_path = os.path.dirname(__file__)+"/../frameworkCodes"

    # 风格
    # further customize: https://stackoverflow.com/questions/35223312/matplotlib-overriding-ggplot-default-style-properties and https://matplotlib.org/2.0.2/users/customizing.html
    mpl.style.use('ggplot')
    # plt.style.use("dark_background")
    # mpl.style.use('grayscale')
    # mpl.style.use('classic')
    
    # 字体
    for key, value in user_plot_config['config']['mpl'].items():
        mpl.rcParams[key] = value
    for key, value in user_plot_config['config']['plt'].items():
        plt.rcParams[key] = value

    # 实用函数
    import subprocess

    def subprocess_cmd(command_string, bool_run_parallel=False):
        process = subprocess.Popen(
            command_string, stdout=subprocess.PIPE, shell=True)
        if not bool_run_parallel:
            streamdata = proc_stdout = process.communicate()[0].strip()
            print(proc_stdout)
        return process

    def cyclic_generator(list_of_things):
        N, i = len(list_of_things), 0
        while True:
            yield list_of_things[i]
            i += 1
            if i > (N-1):
                i = 0

    # 画图风格
    cjh_linestyles = [
        '--', '-', (0, (3, 1, 1, 1)), ':', '-.',
        '--', '-', (0, (3, 1, 1, 1)), ':', '-.',
        '--', '-', (0, (3, 1, 1, 1)), ':', '-.',
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
    no_traces = df_profiles.shape[1]

    DOWN_SAMPLE = 1
    CL_TS = 1e-4
    time = np.arange(1, no_samples+1) * DOWN_SAMPLE * CL_TS


    number_channels = len(user_plot_config['cplot']['subplot'])
    if number_channels_per_column is None:
        number_channels_per_column = number_channels

    def get_fig_axes():
        fig, axes = plt.subplots(number_channels_per_column, number_columns, figsize=(
            user_plot_config['cplot']['width'],
            user_plot_config['cplot']['height']*number_channels),
            facecolor='none',
            sharex=False)
        try:
            return fig, list(axes.ravel())
        except: # only one axis so need to make it a list to be compact
            return fig, [axes]
    number_of_figs = ceil(number_channels / (number_channels_per_column*number_columns))
    figs, axes = [], []
    for _ in range(number_of_figs):
        new_fig, new_axes = get_fig_axes()
        figs.append(new_fig)
        axes.extend(new_axes)

    trace_counter = 0
    for index, subplot_config in enumerate(user_plot_config['cplot']['subplot']):
        ax = axes[index]
        for subplot_signal_index in range(len(subplot_config['y'])):
            trace_counter += 1
            if subplot_config['y'][subplot_signal_index]['y_data'] not in df_profiles.keys():
                # print(f"Warning: cannot find signal {subplot_config['y'][subplot_signal_index]['y_data']} in the signal library, check your DATA_LABELS.")
                print(f"Warning: there is no signal {subplot_config['y'][subplot_signal_index]['y_data']} in the simulation output .dat file.\nCheck your DATA_LABELS.")
            else:
                signal = df_profiles[ subplot_config['y'][subplot_signal_index]['y_data'] ]
                ax.plot(time, signal,
                        linestyle=cjh_linestyles[subplot_signal_index],
                        color=cjh_colors[subplot_signal_index], lw=2,
                        label=subplot_config['y'][subplot_signal_index]['y_label'],
                        alpha=0.65, zorder=100-trace_counter)  # 如果是减去trace_counter，那么就是后来的画在下面
        ax.set_ylabel(subplot_config['y_title'], fontsize=30)
        ax.set_xlabel("Time [s]")
        ax.set_title(subplot_config['title'])
        ax.legend(loc='lower right').set_zorder(202)
    # 创建 output 文件夹
    output_dir = os.path.join(os.path.dirname(__file__), "output")  # 构造相对路径
    os.makedirs(output_dir, exist_ok=True)  # 如果目录不存在，则创建

    # # 保存每个图形到 output 文件夹
    # for i, fig in enumerate(figs):
    #     output_path = os.path.join(output_dir, f"figure_{i + 1}.png")  # 构造保存路径
    #     fig.savefig(output_path, dpi=300, bbox_inches='tight')  # 保存图片
    #     print(f"Figure saved to: {output_path}")
    return figs

if __name__ == "__main__":
    main('TEST_PHIL_LAB_PID_CODES-0-0-0-0')
    plt.show()
