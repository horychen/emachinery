import streamlit as st
import os
import os, sys, ast, json, itertools
from rich import print
import matplotlib.pyplot as plt
from tqdm import tqdm
import pandas as pd
from pylab import np, plt, mpl
import streamlit as st
import _plugins.plugin_MOO.BzierValidation as bv
import _plugins.plugin_MOO.optimize as optimize
from output_postProcessing import cplot
import super_config
import yaml
import _plugins.plugin_MOO.user as user
from matplotlib.colors import Normalize
import datetime
import importlib
import streamlit as st
import _plugins
import pickle

def format(path):
    lines = ''
    with open(path, 'r') as f:
        log = f.read()
        lines = log.split('\n')
        lines = lines[:-1]
        lines = [line[35:] for line in lines]
    data = {'para': [], 'value': [], 'motor_name': ""}
    data['motor_name'] = lines[0].split('@')[0]
    lines = lines[1:]
    for i in range(len(lines)):
        if i % 3 == 0: continue
        elif i % 3 == 1: data['para'].append(eval(lines[i])) # don't use ast.literal_eval that cannot parse [np.float(...), np.float(...), np.float(...), ...]
        else:
            if lines[i] == '[inf, inf]': data['para'].pop()
            else: data['value'].append(eval(lines[i])) # ast.literal_eval
    data['len'] = len(data['para'])
    print(f'Pop individual size: {len(data["para"])}')
    return data


def pareto_points_render(data, order, order_current):
    print(f'Order: {order} | Current order: {order_current}. Rendering pareto points...')

    # 提取目标值，转置数据以便更方便访问
    obj_list = list(zip(*data['value']))  
    obj_min = [min(obj) for obj in obj_list]
    obj_max = [max(obj) for obj in obj_list]
    
    pareto_index = []
    data_len = data['len']
    obj_num = len(obj_list)
    
    sorted_indices = sorted(range(data_len), key=lambda x: obj_list[0][x])
    
    for i in tqdm(sorted_indices):
        is_pareto = True
        for j in pareto_index:
            if all(obj_list[f][i] >= obj_list[f][j] for f in range(obj_num)):
                is_pareto = False
                break
        if is_pareto:
            pareto_index.append(i)

    pareto = {
        'para': [data['para'][index] for index in pareto_index],
        'value': [data['value'][index] for index in pareto_index],
        'order': order,
        'order_current': order_current,
        'size': len(pareto_index),
        'obj_num': obj_num,
        'obj_min': obj_min,
        'obj_max': obj_max,
        'motor_name': data['motor_name'],
    }
    
    return pareto


def convert_log_to_json_click(path, order, current_order):
    if not os.path.exists(path):
        st.toast(path+'do not exist!!!', icon="🚨")
        return
    data = format(path)
    pareto = pareto_points_render(data, order, current_order)
    # save pareto points to json file
    if not os.path.exists(os.path.dirname(__file__) + '/data/pareto'):
        os.mkdir(os.path.dirname(__file__) + '/data/pareto')
    with open(os.path.dirname(__file__) + f'/data/pareto/pareto-{order}-{current_order}.json', 'w') as f:
        json.dump(pareto, f)
    st.toast('Generate Pareto Front from log to '+ os.path.dirname(__file__) + f'/data/pareto/pareto-{order}-{current_order}.json successfully', icon='✌️')
    print('Generate Pareto Front from log to '+ os.path.dirname(__file__) + f'/data/pareto/pareto-{order}-{current_order}.json successfully')
    return

def convert_log_to_json(path, order, order_current):
    st.button('Generate Pareto Front from log as json', on_click=convert_log_to_json_click, args=[path, order, order_current], use_container_width=True)
    return


def section_convert_pkl_to_json():
    st.divider()
    # find file that end with pkl
    pkl_files = []
    if not os.path.exists(os.path.dirname(__file__)+"/data"):
        os.mkdir(os.path.dirname(__file__)+"/data")
        os.mkdir(os.path.dirname(__file__)+"/data/pareto")
    for file in os.listdir(os.path.dirname(__file__)+"/data"):
        if file.endswith(".pkl"):
            pkl_files.append(file)
    if len(pkl_files) == 0:
        st.warning('No pkl file in data folder')
        return
    pkl_files = st.multiselect('Choose pkl files to convert to json', pkl_files, [])
    if st.button('Generate Pareto Front from pkl as json', use_container_width=True):
        for file in pkl_files:
            path = os.path.dirname(__file__) + f'/data/{file}'
            with open(path, 'rb') as pickle_file:
                save_data = pickle.load(pickle_file)
                data = { 
                    'para': save_data['para'],
                    'value': save_data['value'],
                    'motor_name': save_data['motor_name'], 
                    'order': save_data['order'], 
                    'order_current': save_data['order_current'], 
                    'size': len(save_data['para']), 
                    'obj_num': len(save_data['value'][0]),
                    'obj_min': None, 
                    'obj_max': None 
                }
                data['obj_min'] = [min(obj) for obj in zip(*data['value'])]
                data['obj_max'] = [max(obj) for obj in zip(*data['value'])]
            if not os.path.exists(os.path.dirname(__file__) + '/data/pareto'):
                os.mkdir(os.path.dirname(__file__) + '/data/pareto')
            output_path = os.path.join(os.path.dirname(__file__), f'data/pareto/pareto@pop@{data["motor_name"]}@{data["order"]}@{data["order_current"]}.json')
            with open(output_path, 'w') as f:
                json.dump(data, f)
            st.toast('Generate Pareto Front from pkl to ' + output_path + ' successfully', icon="✌️")
            print('Generate Pareto Front from pkl to ' + output_path + ' successfully')
    st.divider()

def checkExistBzier():
    existJson = []
    data_path = os.path.dirname(__file__) + '/data/pareto'
    for file in os.listdir(data_path):
        print(file)
        if file.endswith(".json"):
            existJson.append(file)
    return existJson
            

def load_pareto_from_json(path):
    if not os.path.exists(path):
        st.subheader('Data does not exist. Please generate json file first!')
        return None
    with open(path, 'r') as f:
        pareto = json.load(f)
    if not pareto:
        st.subheader('Data is empty. Please generate json file first!')
        return None
    return pareto

def section_limits(pareto):
    limits = pareto['obj_max']
    for i in range(pareto['obj_num']):
        st.write(f"Min: {pareto['obj_min'][i]:.2f}   |   Max: {pareto['obj_max'][i]:.2f}")
        obj_min = pareto['obj_min'][i]
        obj_max = pareto['obj_max'][i]
        if obj_min == obj_max:
            st.warning(f"Object{i+1} has the same min and max value, please check the data", icon='🚨')
            limits[i] = obj_max
        else:
            limits[i] = st.slider(f'Limitation: max f_{i+1}', min_value=obj_min, max_value=obj_max, value=obj_max)
        limits[i] = st.number_input(f'Limitation: max f_{i+1}', min_value=obj_min, max_value=obj_max, value=limits[i])
        st.write(f"Limitation: max f_{i+1} = {limits[i]:.2f}")
        st.write('---')
    return limits

def save_to_bezier_points_txt(data, order, order_current, motor):
    path = os.path.dirname(__file__) + f'/../../frameworkCodes/acmsimc_bezier_points/{motor}-{order}-{order_current}.txt'
    with open(path, 'w') as f:
        f.write(data)
    st.toast(f'Save bezier points to {motor}-{order}-{order_current}.txt successfully', icon='✌️')

def save_to_bezier_points_C_code(data, d_sim, user_config):
    acm = super_config.SuperConfig(st.session_state.user_selected_motor)
    acm.update_super_config(d_sim, user_config, user_extend_settings={"bezier_C_save": data})
    pass

def filter_data(pareto, limits, d_sim, user_config):
    filtered_data_index = []
    for data_index in range(pareto['size']):
        tmp_filter_flag = True
        for obj_index in range(pareto['obj_num']):
            if pareto['value'][data_index][obj_index] > limits[obj_index]:
                tmp_filter_flag = False
        if tmp_filter_flag:
            filtered_data_index.append(data_index)

    # side bar data visual
    sidebar_filtered_whole_data = []
    for index in filtered_data_index:
        sidebar_filtered_whole_data.append(
            [*pareto['value'][index], *list(itertools.chain(*pareto['para'][index]))]
        )
    sidebar_filtered_whole_data = pd.DataFrame(sidebar_filtered_whole_data)
    sidebar_filtered_whole_data_title = [f"Point{int(i/2)}_{i%2}" for i in range(2*(pareto['order']+1))]
    for i in range(pareto['obj_num']): 
        sidebar_filtered_whole_data_title.insert(0, f"Object{pareto['obj_num']-i}")
    sidebar_filtered_whole_data.columns = sidebar_filtered_whole_data_title
    sidebar_filtered_whole_data.index.name = 'Ind'
    st.write(sidebar_filtered_whole_data)
    
    table_index = [f'Ind{i}' for i in range(len(filtered_data_index))]
    user_select_table_index = st.selectbox(
        'Choose a data to plot',
        table_index,
    )
    user_choose_data = pareto['para'][filtered_data_index[table_index.index(user_select_table_index)]]
    
    st.write(f"fitness of the data you choose:")
    st.write(pareto['value'][filtered_data_index[table_index.index(user_select_table_index)]])
    
    st.write(f"Pareto point you choose:")
    st.code(user_choose_data)
    
    order = pareto['each_order'][filtered_data_index[table_index.index(user_select_table_index)]]
    order_current = pareto['each_order_current'][filtered_data_index[table_index.index(user_select_table_index)]]
    motor_name = pareto['motor_name'][filtered_data_index[table_index.index(user_select_table_index)]]
    if order_current == 0:
        x_coords = ', '.join(f"{user_choose_data[i][0]}" for i in range(order+1))
        y_coords = ', '.join(f"{user_choose_data[i][1]}" for i in range(order+1))
    else:
        x_coords = ', '.join(f"{user_choose_data[i][0]}" for i in range(order+order_current+2))
        y_coords = ', '.join(f"{user_choose_data[i][1]}" for i in range(order+order_current+2))
    motor_name_tmp = f"// This is for motor \"{motor_name}\""
    x_tmp = f"#define SIM_2_EXP_DEFINE_BEZIER_POINTS_X REAL x_tmp[{order+1}] = {{{x_coords}}};"
    y_tmp = f"#define SIM_2_EXP_DEFINE_BEZIER_POINTS_Y REAL y_tmp[{order+1}] = {{{y_coords}}};"
    
    st.warning("The motor name is different from the motor name in the simulation settings. Please check the motor name in the simulation settings.", icon='🚨')
    st.write("Copy the following code to your C code:")
    st.code(motor_name_tmp + '\n' + x_tmp + '\n' + y_tmp)
    if st.button("Save to C code (写入到simuser_bezier.h，仅供实验用)", use_container_width=True):
        save_to_bezier_points_C_code(motor_name_tmp + '\n' + x_tmp + '\n' + y_tmp, d_sim, user_config)
    
    st.write(f"Copy the following code to {motor_name}-{order}-{order_current}.txt:")
    lines = [f"{user_choose_data[i][0]},{user_choose_data[i][1]}" for i in range(order+1)]
    tmp = '\n'.join(lines)
    st.code(tmp)
    
    if st.button("Save bezier points to txt (保存到文件夹acmsimc_bezier_points中的txt,可以供C部分仿真使用)", use_container_width=True):
        save_to_bezier_points_txt(tmp, order, order_current, motor_name)

    return user_choose_data, filtered_data_index, order, order_current

def simulation_and_plot(points, order, order_current, d_sim, user_config):
    if st.button('Show simulation result (用当前选择的点来进行仿真)', use_container_width=True, type='primary'):
        
        path_to_dat = os.path.dirname(__file__) + f'/../../frameworkCodes/dat/{st.session_state.user_selected_motor}.dat'
        if os.path.exists(path_to_dat):
            os.remove(path_to_dat)
        
        with open(os.path.dirname(__file__)+'/../../frameworkCodes/plugin_moo_args.txt', 'w') as f:
            f.write('\n'.join([f"{points[i][0]},{points[i][1]}" for i in range(order+1)]))
        # print(d_sim)
        
        optimize.init_simulation_settings(d_sim, st.session_state.user_selected_motor, order, order_current, user_config)
        figs = cplot.main(st.session_state.user_selected_motor, d_sim)
        if len(figs) == 0: st.warning("No data to plot")
        else:
            for fig in figs:
                st.pyplot(fig)

def plot_pareto_points(data, filtered_data_index):
    axis = st.multiselect('Choose the axis order to plot the pareto points', user.optimize_goal, [])
    axis_index = [user.optimize_goal.index(_) for _ in axis]
    if st.button('Show pareto points', use_container_width=True, type='primary'):
        if len(axis) == 3:
            x, y, z = [], [], []
            for i in range(len(filtered_data_index)):
                x.append(data[filtered_data_index[i]][axis_index[0]])
                y.append(data[filtered_data_index[i]][axis_index[1]])
                z.append(data[filtered_data_index[i]][axis_index[2]])
            fig = plt.figure()
            norm = Normalize(vmin=min(z), vmax=max(z))
            scatter = plt.scatter(x, y, c=z, cmap='viridis',
                                marker='o', norm=norm, s=5)
            cbar = plt.colorbar(scatter, orientation='vertical')
            cbar.set_label(axis[2])
            plt.xlabel(axis[0])
            plt.ylabel(axis[1])
            st.pyplot(fig)
        elif len(axis) == 2:
            x, y = [], []
            for i in range(len(filtered_data_index)):
                x.append(data[filtered_data_index[i]][axis_index[0]])
                y.append(data[filtered_data_index[i]][axis_index[1]])
            fig = plt.figure()
            plt.scatter(x, y, marker='o', s=5)
            plt.xlabel(axis[0])
            plt.ylabel(axis[1])
            st.pyplot(fig)
        # return fig

def load_multiple_pareto_from_json(paths):
    combined_pareto = {
        'para': [],
        'value': [],
        'order': 0,
        'order_current': 0,
        'size': 0,
        'obj_num': 0,
        'obj_min': None,
        'obj_max': None,
        'each_order': [],
        'each_order_current': [],
        'motor_name': []
    }
    for path in paths:
        if not os.path.exists(path):
            st.subheader(f'Data does not exist at {path}. Please generate json file first!')
            continue
        with open(path, 'r') as f:
            pareto = json.load(f)
        if not pareto:
            st.subheader(f'Data is empty at {path}. Please generate json file first!')
            continue
        combined_pareto['para'].extend(pareto['para'])
        combined_pareto['value'].extend(pareto['value'])
        combined_pareto['order'] = max(combined_pareto['order'], pareto['order'])
        combined_pareto['order_current'] = max(combined_pareto['order_current'], pareto['order_current'])
        combined_pareto['size'] += pareto['size']
        combined_pareto['obj_num'] = pareto['obj_num']
        combined_pareto['each_order'].extend([pareto['order'] for _ in range(pareto['size'])])
        combined_pareto['each_order_current'].extend([pareto['order_current'] for _ in range(pareto['size'])])
        combined_pareto['motor_name'].extend([pareto['motor_name'] for _ in range(pareto['size'])])
        if combined_pareto['obj_min'] is None:
            combined_pareto['obj_min'] = pareto['obj_min']
            combined_pareto['obj_max'] = pareto['obj_max']
            continue
        combined_pareto['obj_min'] = [min(x, y) for x, y in zip(combined_pareto['obj_min'], pareto['obj_min'])]
        combined_pareto['obj_max'] = [max(x, y) for x, y in zip(combined_pareto['obj_max'], pareto['obj_max'])]
        
    for index in range(combined_pareto['size']):
        while len(combined_pareto['para'][index]) < combined_pareto['order'] + 1:
            combined_pareto['para'][index].append([0,0])
    return combined_pareto

def generate_multiple_pareto_paths(options):
    paths = []
    data_path = os.path.dirname(__file__) + '/data/pareto/'
    for file in options:
        path = data_path + file
        paths.append(path)
    return paths


def get_Bzier_curve(points):
    BzierController = bv.BezierController(points, 0)
    Bzier = []
    Gain = []
    t_values = np.linspace(0, 1, 100)
    x_values = [BzierController.bezier_x(t) for t in t_values]
    y_values = [BzierController.bezier_y(t) for t in t_values]
    Bzier.append([x_values, y_values])
    epsilon = 1e-7
    Gain.append([x_values, np.array(y_values)/(np.array(x_values) + epsilon)])
    return Bzier, Gain


def plot_Bzier(filtered_data_index, multiple_pareto, filtered_index):
    if st.button('Show Bzier curves', use_container_width=True, type='primary'):
        if filtered_data_index == []:
            return
        Bzier_Curves = []
        Gain_Curves = []
        for index in filtered_data_index:
            points = multiple_pareto['para'][index]
            point_order = multiple_pareto['each_order'][index] 
            Bzier, Gain = get_Bzier_curve(points[0:point_order + 1])
            Bzier_Curves.append(Bzier)
            Gain_Curves.append(Gain)
        plt.style.use('bmh')
        mpl.rc('font', family='Times New Roman', size=10.0)
        mpl.rc('legend', fontsize=10)
        mpl.rcParams['text.color'] = 'black'
        mpl.rcParams['axes.labelcolor'] = 'black'
        mpl.rcParams['xtick.color'] = 'black'
        mpl.rcParams['ytick.color'] = 'black'	
        mpl.rcParams['lines.linewidth'] = 0.75
        mpl.rcParams['mathtext.fontset'] = 'stix'
        fig, axes = plt.subplots(nrows=1, ncols=1, dpi=150,
                                facecolor='w', sharex=True)
        for i in range(len(Bzier_Curves)):
            plt.plot(Bzier_Curves[i][0][0], Bzier_Curves[i][0][1], label=f'Bzier point {filtered_index[i]}')
        plt.legend()
        st.pyplot(fig)
        fig, axes = plt.subplots(nrows=1, ncols=1, dpi=150,
                                facecolor='w', sharex=True)
        for i in range(len(Gain_Curves)):
            plt.plot(Gain_Curves[i][0][0], Gain_Curves[i][0][1], label=f'Gain point {filtered_index[i]}')
        plt.legend()
        st.pyplot(fig)

# def save_as_plugin_moo_dsim_yaml(order, order_current, d_sim, motor):
#     save_data = {
#         'order': order,
#         'order_current': order_current,
#         'd_sim': d_sim,
#         'motor': motor
#     }
#     with open(os.path.dirname(__file__) + '/data/moo_dsim.yaml', 'w') as f:
#         yaml.dump(save_data, f, default_flow_style=False, allow_unicode=True)
#     st.toast('Save as plugin moo dsim yaml successfully', icon='✌️')
#     st.info('Run the following command to start optimize:')
#     st.code(f'python {os.path.dirname(__file__)}../../main.py optimize')
#     return

def optimization_moudle(d_sim, user_config):
    st.header('0. Optimization', divider=True)
    order = st.number_input('Velocity loop Bezier order', value=4, step=1, min_value=4, max_value=10)
    order_current = st.number_input('Current loop Bezier order', value=0, step=1, min_value=0, max_value=4)
    if st.button('Optimization', use_container_width=True):
        save_data = {
            'order': order,
            'order_current': order_current,
            'd_sim': dict(d_sim),
            'motor': st.session_state.user_selected_motor,
            'user_config': user_config
        }
        path = os.path.dirname(__file__) + '/data'
        if not os.path.exists(path):
            os.mkdir(path)
        with open(os.path.dirname(__file__) + '/data/moo_dsim.yaml', 'w') as f:
            yaml.dump(save_data, f, default_flow_style=False, allow_unicode=True)
        st.toast(f'Save as plugin {os.path.dirname(__file__)}/data/moo_dsim.yaml successfully', icon='✌️')
        st.toast('Optimization now is ready to start.', icon='🚀')
        st.info('Run the following command to start optimize:')
        st.code(f'python {os.path.dirname(__file__)}/../../main.py moo')

def main(d_sim, user_config):
    st.title('MOO | 多目标优化Bezier controller | Visualize the optimal result')
    st.info("""
插件名叫做 plugin-MOO\n
\n
基本的配置参数如下：\n
\n
User defined parameter:\n
	order = 4 to 9\n
	current_order = 0 ( now only support 0 )\n
\n
MOO problem:\n
	min steady state error\n
	min energy cost (current squared)\n
	min rising time\n
	min disturbance rejection tuning range\n
	over bezier_points_x [order], bezier_points_y [order]\n
\n
limited by the range:\n
	0<= bezier_points_x < 700 r/min\n
	0<= bezier_points_y < 16.9 A\n
\n
FOC.delta 和 FOC.CLBW 都会影响到这个优化的结果。
""")
    optimization_moudle(d_sim, user_config)

    st.header('1. Bezier order configuration', divider=True)
    order = st.number_input("Velocity loop Bezier order", value=4, step=1, min_value=4, max_value=10, key='order')
    order_current = st.number_input("Current loop Bezier order", value=0, step=1, min_value=0, max_value=4, key='order_current')
    path = os.path.dirname(__file__) + f'/data/opt-{order}-{order_current}.log'
    convert_log_to_json(path, order, order_current)
    
    section_convert_pkl_to_json()

    exist_Bezier = checkExistBzier()
    options = st.multiselect(
    "Choose Bzier orders",
    exist_Bezier,
    [],
    )
    multiple_pareto_paths = generate_multiple_pareto_paths(options)
    multiple_pareto = load_multiple_pareto_from_json(multiple_pareto_paths)
    # st.write(multiple_pareto)

    pareto = multiple_pareto
    st.header('2. Bezier points select', divider=True)
    st.info("f1: 是稳态误差[rad]；f2是能量损耗[A^2]；f3是抗扰性能[rad]；f4是max_p_value[A]")
    limits = section_limits(pareto)

    st.header('3. Filtered data', divider=True)
    if pareto['size'] == 0:
        st.warning('No data to plot')
        return
    user_select_points, filtered_data_index, order, order_current = filter_data(pareto, limits, d_sim, user_config)
    simulation_and_plot(user_select_points, order, order_current, d_sim, user_config)
    plot_pareto_points(pareto['value'], filtered_data_index)

    st.header('4. Draw B-Zier curve', divider=True)
    filtered_data_index_trans = [f'Ind{i}' for i in range(len(filtered_data_index))]
    all_options = st.checkbox("Select all options")
    if all_options:
        filtered_index = st.multiselect("Choose Bzier orders", filtered_data_index_trans, filtered_data_index_trans, disabled=True, )
    else:
        filtered_index = st.multiselect( "Choose Bzier orders", filtered_data_index_trans, [], )
    filtered_data_index = [filtered_data_index[filtered_data_index_trans.index(_)] for _ in filtered_index]
    print(filtered_data_index)

    plot_Bzier(filtered_data_index, multiple_pareto, filtered_index)