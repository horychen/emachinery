import pandas as pd
import os, platform, subprocess, yaml
import streamlit as st
import pkg_resources
from rich import print
import pandas as pd
import user_script_main

PYTHON_SHEET_NAME = 'Python 仿真'
C_SHEET_NAME = 'C语言仿真'

TEMPLATE_CONTENT_FRONT = '''// This header file is automatically generated. Any modification to this file will get lost.\n#ifndef SUPER_CONFIG_H\n#define SUPER_CONFIG_H\n#include "typedef.h"\n\n'''
TEMPLATE_CONTENT_END = '''\nextern ST_D_SIM d_sim;\nvoid init_d_sim();\n#endif // SUPER_CONFIG_H\n'''

class SuperConfig:
    '''
    Usage:
    acm = SuperConfig(motor_name)
    acm.update_super_config(d_sim, user_config)
    acm.run_simulation()
    '''
    def __init__(self, motor_name):
        self.motor_name = motor_name
        self.c_macros = [] # 宏的名字列表（参数的c名字）
        self.super_config_header_content_front = TEMPLATE_CONTENT_FRONT
        self.super_config_header_content = ""
        self.super_config_header_content_end = TEMPLATE_CONTENT_END
        self.super_config_C_content = ""

    # def load_signal_library(self):
    #     with open(os.path.dirname(__file__)+'/user_config.yaml', encoding='utf-8') as f:
    #         user_config = yaml.load(f, Loader=yaml.FullLoader)
    #     return user_config['signal_library']

    def parse_d_sim(self, d_sim):
        # print(d_sim)
        data = {}
        # d_sim = dict(d_sim)
        for key, val in d_sim.items():
            try:
                parent_node, child_node = key.split('.')
            except ValueError as e:
                print(f'yaml[simulation]里的字符串{key}要用点分离')
                print(f'yaml[simulation]里的字符串{key}要用点分离')
                print(f'yaml[simulation]里的字符串{key}要用点分离')
                raise e
            if parent_node not in data:
                data[parent_node] = {}
            data[parent_node][child_node] = val
        return data

    # def get_d_sim_value(self, key):
    #     val = None
    #     for parent_key, val in self.d_sim_as_data['user']['who_is_user'].items():
    #         print(parent_key, val)
    #         # for child_key, child_val in val.items():
    #         #     if (child_key) == 'who_is_user':
    #         #         val = child_val
    #         #         break
    #     return val 

    def super_config_generator(self, data, user_extend_settings=None):
        # print(f'[super_config.py] {data=}')
        
        for parent_key, val in data.items():
            self.super_config_header_content += f"\ntypedef struct {{\n"
            for child_key, child_val in val.items():
                if isinstance(child_val, bool):
                    self.super_config_header_content += f"    BOOL {child_key};\n"
                elif isinstance(child_val, (int)):
                    self.super_config_header_content += f"    long {child_key};\n"
                else:
                    self.super_config_header_content += f"    REAL {child_key};\n"
            self.super_config_header_content += f"}} ST_{parent_key};\n"
        self.super_config_header_content += f"\n\ntypedef struct {{\n"
        for key in data.keys():
            self.super_config_header_content += f"    ST_{key} {key};\n"
        self.super_config_header_content += f"}} ST_D_SIM;\n"
        
        
        self.super_config_C_content = "// This c file is automatically generated. Any modification to this file will get lost.\n#include \"super_config.h\"\n#include <stdio.h>\n\n"
        self.super_config_C_content += "void init_d_sim() {\n"
        for parent_key, val in data.items():
            for child_key, child_val in val.items():
                if child_val == True or child_val == False or child_val == 'True' or child_val == 'False':
                    self.super_config_C_content += f"    d_sim.{parent_key}.{child_key} = {str(child_val).upper()};\n"
                else:
                    self.super_config_C_content += f"    d_sim.{parent_key}.{child_key} = {child_val};\n"
            self.super_config_C_content += "\n"
        self.super_config_C_content += "}\n"
        
        self.super_config_C_content, self.super_config_header_content = user_script_main.user_bezier_super_config(self.motor_name, data, user_extend_settings, self.super_config_C_content, self.super_config_header_content)

    def data_file_generat_format(self, data_details):
        data_format_str = "\n\n\n#define DATA_FORMAT \""
        data_labels_str = "#define DATA_LABELS \""
        data_details_str = "#define DATA_DETAILS "
        for data_detail in data_details:
            data_format_str += "%g,"
            data_labels_str += str(data_detail)+","
            data_details_str += str(data_detail)+","
        data_format_str = data_format_str[:-1] + "\\n\"\n"
        data_labels_str = data_labels_str[:-1] + "\\n\"\n"
        data_details_str = data_details_str[:-1] + "\n\n\n"
        return data_format_str + data_labels_str + data_details_str

    def update_super_config(self, d_sim, user_config, user_extend_settings=None):
        self.super_config_header_content = ""
        self.d_sim_as_data = self.parse_d_sim(d_sim)
        self.super_config_generator(self.d_sim_as_data, user_extend_settings)

        # st.write(st.session_state.user_selected_motor)

        # 生成数据文件数据标签与路径
        data_details = user_config['signal_library'] # self.load_signal_library()
        data_unite =  self.data_file_generat_format(data_details)
        data_file_name = "#define DATA_FILE_NAME \"../dat/" + str(self.motor_name) + ".dat\"\n"
        
        config_content = self.super_config_header_content_front + \
            f'''#define WHO_IS_USER {self.d_sim_as_data['user']['who_is_user']} \n'''\
            + self.super_config_header_content + data_unite + data_file_name + self.super_config_header_content_end
        # print(config_content)
        path_to_header = os.path.dirname(__file__) + '/frameworkCodes/c/super_config.h'
        path_to_c = os.path.dirname(__file__) + '/frameworkCodes/c/super_config.c'
        with open(path_to_header, 'w') as file:
            file.write(config_content)
        with open(path_to_c, 'w') as file:
            file.write(self.super_config_C_content)

    def run_simulation(self):
        current_dir = os.path.dirname(__file__)
        target_dir = current_dir+'/frameworkCodes/c'
        origin_dir = os.getcwd()
        system_type = platform.system()
        os.chdir(target_dir)
        try:
            if system_type == 'Windows':
                subprocess.run('gmake', shell=True, check=True)
                subprocess.run('main', shell=True, check=True)

            else:
                subprocess.run('make clean', shell=True, check=True)
                subprocess.run('make', shell=True, check=True)
                subprocess.run('./main', shell=True, check=True)
        except Exception as e:
            print(e)
        finally:
            os.chdir(origin_dir)

if __name__ == "__main__":
    d_sim = {
        "sim.CL_TS": 0.0001,
        "sim.NUMBER_OF_STEPS": 50000,
        "sim.MACHINE_SIMULATIONs_PER_SAMPLING_PERIOD": 1,
        "init.npp": 2,
        "init.IN": 4.6,
        "init.R": 5.5,
        "init.Ld": 0.5800000000000001,
        "init.Lq": 0.022,
        "init.KE": 1.3593784874408605,
        "init.Rreq": 2.1,
        "init.Js": 0.063,
        "init.Vdc": 600,
        # "user_system_input_code": "if ii < 1: (*CTRL).cmd_idq[0] = 0.0; (*CTRL).cmd_rpm = 50 \nelif ii <5: ACM.TLoad = 0.2 \nelif ii <100: (*CTRL).cmd_rpm = -50",
        "CTRL.bool_apply_speed_closed_loop_control": True,
        "CTRL.bool_apply_decoupling_voltages_to_current_regulation": False,
        "CTRL.bool_apply_sweeping_frequency_excitation": False,
        "CTRL.bool_overwrite_speed_commands": True,
        "CTRL.bool_zero_id_control": True,
        "FOC.VL_EXE_PER_CL_EXE": 5,
        "FOC.delta": 15,
        "FOC.CLBW_HZ": 400,
        "FOC.CL_KI_factor_when__bool_apply_decoupling_voltages_to_current_regulation__is_False": 10,
        "CL.SERIES_KP": 55.29203070318036,
        "CL.SERIES_KI": 250.00000000000003,
        "CL.LIMIT_DC_BUS_UTILIZATION": 0.96,
        "VL.SERIES_KP": 1.2941884120310692,
        "VL.SERIES_KI": 11.170107212763709,
        "VL.LIMIT_OVERLOAD_FACTOR": 1.0,
    }

    acm = SuperConfig('test')
    # print(acm.parsing_structs(d_sim))
    acm.update_super_config(d_sim, None)
    acm.run_simulation()
    print(acm.super_config_header_content)
