# from copyreg import pickle
import sys
import json
import os
from platformdirs import user_config_dir
import pygmo as pg
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.colors import Normalize
from scipy.optimize import fsolve
from scipy.spatial import ConvexHull
import tqdm
import logging
import time
from rich import print
import re
import scipy
import itertools
import sympy
from rich import print
import super_config as super_config 
import platform
import subprocess
import pandas as pd
import yaml
import _plugins.plugin_MOO.user as user
import pickle

# font_path = './Times New Roman.ttf'  # 替换为 Times New Roman 字体文件的路径
# font_prop = fm.FontProperties(fname=font_path)
# plt.rcParams['font.family'] = font_prop.get_name()

def run_simulation(points):
    current_dir = os.path.dirname(__file__)
    with open(current_dir+'/../../frameworkCodes/plugin_moo_args.txt', 'w') as f:
        for point in points:
            f.write(str(point[0])+','+str(point[1])+'\n')
    target_dir = current_dir+'/../../frameworkCodes/c'
    origin_dir = os.getcwd()
    system_type = platform.system()
    os.chdir(target_dir)
    try:
        if system_type == 'Windows':
            subprocess.run('main', shell=True, check=True)
        else:
            subprocess.run('./main', shell=True, check=True)
    except Exception as e:
        print(e)
    finally:
        os.chdir(origin_dir)

def get_simulation_result(motor_name):
    current_dir = os.path.dirname(__file__)
    file_path = current_dir+f'/../../frameworkCodes/dat/{motor_name}.dat'
    data = pd.read_csv(file_path)
    return data

def optimize_main(points, d_sim, motor_name, max_p_value=None):
    run_simulation(points)
    simulation_result = get_simulation_result(motor_name)
    eval_result = user.evaluate_simResults(simulation_result, d_sim, max_p_value)
    return eval_result

FIX_THE_LAST_BEZIER_CONTROL_HANDLE = 1 # choose from 1 or 0

class Bezier(object):
    def __init__(self, d_sim, motor_name='default', eval_dim=4, order=4, order_current=0):
        self.d_sim = d_sim
        self.motor_name = motor_name
        self.eval_dim = eval_dim  # 评价维度

        self.order = order
        self.order_current = order_current

        # 定义搜索空间的大小（Bezier控制点的横纵坐标
        self.rpm_maximum_effective_speed_error = d_sim['user.bezier_rpm_maximum_effective_speed_error']  # [r/min]
        self.A_current_limit = d_sim['init.IN']                                      # [A]
        self.V_voltage_limit = d_sim['init.Vdc']                                                         # [V] 
        print(f'{self.V_voltage_limit=:.1f}, {self.A_current_limit=:.1f}')

        self.bounds_denorm = []
        if self.order:
            # 速度环Bezier控制点生成
            for _ in range(self.order - FIX_THE_LAST_BEZIER_CONTROL_HANDLE):
                self.bounds_denorm.append([0.0, self.rpm_maximum_effective_speed_error])
                self.bounds_denorm.append([0.0, self.A_current_limit])
        # 电流环Bezier控制点生成
        if self.order_current:
            for _ in range(self.order_current - FIX_THE_LAST_BEZIER_CONTROL_HANDLE):
                self.bounds_denorm.append([0.0, self.A_current_limit])
                self.bounds_denorm.append([0.0, self.V_voltage_limit])
            # 对应电流环controller中self.p的生成
            self.bounds_denorm.append([0.0, self.V_voltage_limit])

        self.counter_fitness_called = 0
        self.counter_fitness_return = 0

    def evaluate_design(self, x_denorm):
        # convert x_denorm into control/handle points of Bezier curve.
        if self.order:
            params_list = [(0, 0)] # the first contorol handle is fixed
            for i in range(self.order - FIX_THE_LAST_BEZIER_CONTROL_HANDLE):
                params_list.append((x_denorm[2*i], x_denorm[2*i+1]))
            if FIX_THE_LAST_BEZIER_CONTROL_HANDLE:
                params_list.append((self.rpm_maximum_effective_speed_error, self.A_current_limit)) # the last control handle is fixed

        if self.order_current:
            params_list.append((0, 0))
            for i in range(self.order_current - FIX_THE_LAST_BEZIER_CONTROL_HANDLE):
                params_list.append(
                    (x_denorm[self.order*2+2*i], x_denorm[self.order*2+2*i+1]))
            if FIX_THE_LAST_BEZIER_CONTROL_HANDLE:
                params_list.append((self.A_current_limit, self.V_voltage_limit)) # the last control handle is fixed
            # 对应电流环controller中self.p
            params_list.append((x_denorm[-2], x_denorm[-1])) # 放最后

        # print(f'{params_list=}')
        # print('position_loop ', self.sturm_therorem(params_list[:self.order+1]))
        # print('current_loop ', (not self.order_current or self.sturm_therorem(params_list[self.order+1:-1])))

        if self.sturm_therorem(params_list[:self.order+1]) and (not self.order_current or self.sturm_therorem(params_list[self.order+1:-1])):
            # 根据已有的 bezier control points 计算最大允许的倒数第二个
            #! 不对 max_p_value进行计算
            if False:
                max_p_value = self.findP(params_list[:self.order+1])
                print(f'{max_p_value=}')
            return optimize_main(params_list, self.d_sim, self.motor_name) 
        else:
            return [float('inf')]*self.eval_dim

    def findP(self, points):
        p = 0
        step = 1e-2
        IsFeasible = True
        x = sympy.Symbol('x')
        limits = [-0.0, 1.0]
        while IsFeasible:
            p += step
            ans = 0
            sturm_seq = sympy.sturm(self.bezier_derivative(points, x, p)[1])
            values_at_start = [polynomial.subs(
                x, limits[0]).evalf() for polynomial in sturm_seq]
            values_at_end = [polynomial.subs(
                x, limits[1]).evalf() for polynomial in sturm_seq]
            count_start = len(list(itertools.groupby(
                values_at_start, lambda values_at_start: values_at_start > 0)))
            count_end = len(list(itertools.groupby(
                values_at_end, lambda values_at_end: values_at_end > 0)))
            ans = count_start - count_end
            if ans != 0:
                IsFeasible = False
        p -= step
        p = -p
        while not IsFeasible:
            ans = 0
            sturm_seq = sympy.sturm(self.bezier_derivative(points, x, p)[1])
            values_at_start = [polynomial.subs(
                x, limits[0]).evalf() for polynomial in sturm_seq]
            values_at_end = [polynomial.subs(
                x, limits[1]).evalf() for polynomial in sturm_seq]
            count_start = len(list(itertools.groupby(
                values_at_start, lambda values_at_start: values_at_start > 0)))
            count_end = len(list(itertools.groupby(
                values_at_end, lambda values_at_end: values_at_end > 0)))
            ans = count_start - count_end
            if ans != 0:
                p += step
                IsFeasible = False
            else:
                IsFeasible = True

        return np.abs(p)

    def bezier_derivative(self, points, t, p=0):
        re = np.array([0.0, 0.0])
        for i in range(len(points)):
            if i == len(points)-1:
                p = 0
            re = re + np.array(points[i]+np.array([0, p]))*scipy.special.comb(len(points)-1, i) * \
                ((1-t)**(len(points)-1-i))*(t**(i-1))*i
            re = re + np.array(points[i]+np.array([0, p]))*scipy.special.comb(len(points)-1, i)*(
                (1-t)**(len(points)-1-i-1))*(t**i)*(i-len(points)+1)
        return re

    def sturm_therorem(self, points):
        x = sympy.Symbol('x')
        limits = [-0.0, 1.0]
        for i in range(len(points[0])):
            ans = 0
            sturm_seq = sympy.sturm(self.bezier_derivative(points, x)[i])
            values_at_start = [polynomial.subs(x, limits[0]).evalf() for polynomial in sturm_seq]
            values_at_end = [polynomial.subs(x, limits[1]).evalf() for polynomial in sturm_seq]
            count_start = len(list(itertools.groupby(values_at_start, lambda values_at_start: values_at_start > 0)))
            count_end = len(list(itertools.groupby(values_at_end, lambda values_at_end: values_at_end > 0)))
            ans = count_start - count_end
            if ans != 0:
                return False
        return True

class Problem_Bezier(object):
    
    def __init__(self, motor_name, nobj):
        self.motor_name = motor_name
        self.nobj = nobj
    
    def fitness(self, x):
        global bezier
        if bezier.counter_fitness_called == bezier.counter_fitness_return:
            bezier.counter_fitness_called += 1
        else:
            raise Exception('bezier.counter_fitness_called')
        x_denorm = x
        # print('-'*40)
        logging_info = '[(0, 0)'
        if bezier.order:
            for i in range(bezier.order - FIX_THE_LAST_BEZIER_CONTROL_HANDLE):
                logging_info += ',({}, {})'.format(x_denorm[2*i], x_denorm[2*i+1])
            if FIX_THE_LAST_BEZIER_CONTROL_HANDLE:
                logging_info += ',({}, {})'.format(bezier.rpm_maximum_effective_speed_error, bezier.A_current_limit)

        if bezier.order_current:
            logging_info += ',(0, 0)'
            for i in range(bezier.order_current - FIX_THE_LAST_BEZIER_CONTROL_HANDLE):
                logging_info += ',({}, {})'.format(
                    x_denorm[2*bezier.order+2*i], x_denorm[2*bezier.order+2*i+1])
            if FIX_THE_LAST_BEZIER_CONTROL_HANDLE:
                logging_info += ',({}, {})'.format(bezier.A_current_limit, bezier.V_voltage_limit)
            # 对应 bezier.p 积分项，放最后
            logging_info += ',({}, {})'.format(x_denorm[-2], x_denorm[-1])
        logging_info += ']'
        # print(f'Parameters: {logging_info}')
        eval_result = bezier.evaluate_design(x_denorm)
        bezier.counter_fitness_return += 1
        if any(item == float('inf') for item in eval_result):
            return [float('inf')]*bezier.eval_dim
        print(f'ind{bezier.counter_fitness_called} f: {eval_result}')
        #f1: steady state
        #f2: energy
        #f3: step response
        #f4: tunnable region
        logging.info('ind'+str(bezier.counter_fitness_called)+'-'*20+time.strftime("%Y-%m-%D-%H-%M-%S", time.localtime())+'  '+str(self.motor_name)+'-'*20)
        logging.info(logging_info)
        logging.info(eval_result)
        return eval_result

    def get_nobj(self):
        return self.nobj

    def get_bounds(self):
        global bezier
        print('get_bounds:', bezier.bounds_denorm)
        min_b, max_b = np.asarray(bezier.bounds_denorm).T
        return (min_b.tolist(), max_b.tolist())

    def get_name(self):
        return "Bezier Controller"

def update_bezier_params(d_sim, order, order_current):
    d_sim['user.bezier_order'] = order
    d_sim['user.bezier_order_current'] = order_current
    return d_sim

def init_simulation_settings(d_sim, motor_name, order, order_current, user_config):
    update_bezier_params(d_sim, order, order_current)
    acm = super_config.SuperConfig(motor_name)
    acm.update_super_config(d_sim, user_config, user_extend_settings={'bezier_moo': True})
    acm.run_simulation()
    return d_sim

def optimize(order, order_current, d_sim, motor_name='default', user_config=None):
    print(f'{order=}, {order_current=}')
    d_sim = init_simulation_settings(d_sim, motor_name, order, order_current, user_config)

    # Build data folder
    file_path = os.path.dirname(__file__)+'/'
    now = time.strftime("%Y-%m-%d_user-%H-%M-%S", time.localtime())
    now = re.sub(r'[:*]', '', now)
    if not os.path.exists(file_path+'data'):
        os.mkdir(file_path+'data')
    if os.path.exists(file_path+'data/opt-{}-{}.log'.format(order, order_current)):
        if not os.path.exists(file_path+'data/past'):
            os.mkdir(file_path+'data/past')
        os.rename(file_path+'data/opt-{}-{}.log'.format(order, order_current), file_path +
                  'data/past/past-{}-{}-{}.log'.format(order, order_current, now + '-' + str(order)))
    logging.basicConfig(filename=file_path+'data/opt-{}-{}.log'.format(order, order_current),
                        level=logging.INFO, format='[%(asctime)s - %(levelname)s] - %(message)s')
    logging.info(f"{motor_name}@{order}@{order_current}")
    # 定义优化目标
    global bezier
    bezier = Bezier(d_sim, motor_name, user.eval_dim, order, order_current)

    udp = Problem_Bezier(motor_name, user.eval_dim)
    prob = pg.problem(udp)
    print(prob)

    popsize = user.popsize
    print('-' * 40 + '\nPop size is', popsize)
    algo = pg.algorithm(pg.moead(gen=1, weight_generation="grid", decomposition="tchebycheff", neighbours=5, CR=1, F=0.5, eta_m=20,
                        # https://esa.github.io/pagmo2/docs/python/algorithms/py_algorithms.html#pygmo.moead
                                 realb=0.9, limit=2, preserve_diversity=True))
    print('-' * 40, '\n', algo)

    pop = pg.population(prob, size=popsize)

    number_of_iterations = user.number_of_iterations
    print('-' * 40 + '\nNumber of iterations is', number_of_iterations)
    previous_pop = {}


    for i in tqdm.tqdm(range(number_of_iterations)):
        print('=' * 40 + 'order: ' + str(bezier.order) + '=' * 40)
        pop = algo.evolve(pop)
        
        print('-------------------------------')
        print(pop.get_f()) 
        
        # 检查个体信息是否更新
        pop_data = {
            'Decision vector': pop.get_x(),
            'Fitness vector': pop.get_f()
        }
        
        if previous_pop:
            if np.array_equal(pop_data['Decision vector'], previous_pop['Decision vector']):
                print("No updates detected, stopping evolution.")
                print('Optimization ended at generation: ', i)
                break
            else:
                previous_pop = pop_data

        value = [array.tolist() for array in pop.get_f()]
        para = [array.tolist() for array in pop.get_x()]

        for array in para:
            array.insert(0, 0.0)
            array.insert(0, 0.0)

        if FIX_THE_LAST_BEZIER_CONTROL_HANDLE:
            for array in para:
                array.insert(2 * order, bezier.rpm_maximum_effective_speed_error)
                array.insert(2 * order + 1, bezier.A_current_limit)

        if order_current:
            for array in para:
                array.insert(2 * order + 2, 0.0)
                array.insert(2 * order + 3, 0.0)
                if FIX_THE_LAST_BEZIER_CONTROL_HANDLE:
                    array.insert(2 * order + 2 * order_current+2, bezier.A_current_limit)
                    array.insert(2 * order + 2 * order_current+3, bezier.V_voltage_limit)
        if order_current:
            points_num = order+1+order_current+1
        else:
            points_num = order+1
        para = np.array(para).reshape(-1, points_num, 2).tolist()
        
        save_data = {'para': para, 'value': value, 'motor_name': motor_name, 'order': order, 'order_current': order_current}
        file_path = os.path.dirname(__file__) + '/data/'
        with open(file_path + f'pop@{motor_name}@{order}@{order_current}.pkl', 'wb') as pickle_file:
            pickle.dump(save_data, pickle_file, protocol=pickle.HIGHEST_PROTOCOL)

    print('Final Pareto Front:\n', pop)

    # ax = pg.plot_non_dominated_fronts(pop.get_f())
    # print(pop.get_x())
    # plt.xlabel('Error')
    # plt.ylabel('iDQ')
    # plt.title('Pareto Front of order {}'.format(bezier.order))
    # plt.show()

def load_moo_dsim_yaml():
    with open(os.path.dirname(__file__) + '/data/moo_dsim.yaml', 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data['order'], data['order_current'], data['d_sim'], data['motor'], data['user_config']

def main():
    order, order_current, d_sim, motor, user_config = load_moo_dsim_yaml()
    # TODO
    d_sim['sim.CLTS'] = float(d_sim['sim.CLTS'])
    optimize(order, order_current, d_sim, motor, user_config)
