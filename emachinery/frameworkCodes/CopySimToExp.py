from shutil import copyfile
import os

def main(goal_path):
    # Do not copy ACMSim.h
    # Do not copy ACMSim.h
    # Do not copy ACMSim.h
    for fname in [
        'pi_math.h',
        'shared_flux_estimator.c',
        'shared_flux_estimator.h',
        'pmsm_observer.c',
        'pmsm_observer.h',
        'pmsm_comm.c',
        'pmsm_comm.h',
        'utility.c',
        'ACMConfig.h',
        'super_config.h',
        'super_config.c',
        'main_switch.c',
        'main_switch.h',
        'typedef.h',
        'simuser_bezier.c',
        'simuser_bezier.h',
        'simuser_cjh.c',
        'simuser_cjh.h',
        'simuser_wb.h',
        'simuser_wb.c',
        'simuser_yzz.h',
        'simuser_yzz.c',
        ]:
        origin_path = os.path.dirname(__file__)
        copyfile(origin_path+f'/c/{fname}', rf'{goal_path}/{fname}')
        # 
        # 这里用的绝对路径，你得改你得改你得改
        # 这里用的绝对路径，你得改你得改你得改
        # 这里用的绝对路径，你得改你得改你得改
        # 这里用的绝对路径，你得改你得改你得改
        # 找到ProjectPanGu-C\User_acmsimcv5
        # 找到ProjectPanGu-C\User_acmsimcv5
        # 找到ProjectPanGu-C\User_acmsimcv5
        # 
        # C:\Users\Wu\Desktop\panguc\ProjectPanGu-C\User_acmsimcv5
        # python .\CopySimToExp.py C:\Users\Wu\Desktop\panguc\ProjectPanGu-C\User_acmsimcv5
        # python .\CopySimToExp.py D:\DrH\Codes\ProjectPanGu-C\User_acmsimcv5

if __name__ == '__main__':
    if len(os.sys.argv) > 1 and len(os.sys.argv) < 3:
        main(os.sys.argv[1])
        print('Sim to Exp Done!')
    elif len(os.sys.argv) > 2:
        print('Too many arguments! Please check if the path has space! If so, please use REAL quotes to wrap the path.')
    else:
        print('Please input the path of the destination folder!')

# 老的emy-c用的copy文件
        # 'pi_math.h',
        # 'pid_regulator.c',
        # 'pid_regulator.h',
        # 'shared_flux_estimator.c',
        # 'shared_flux_estimator.h',
        # 'im_controller.c',
        # 'im_controller.h',
        # 'im_observer.c',
        # 'im_observer.h',
        # 'pmsm_controller.c',
        # 'pmsm_controller.h',
        # 'pmsm_observer.c',
        # 'pmsm_observer.h',
        # 'pmsm_comm.c',
        # 'pmsm_comm.h',
        # 'global_variables_definitions.c',
        # 'utility.c',
        # 'ACMConfig.h',
        # 'super_config.h',
        # 'super_config.c',
        # # 'Bezier.c',
        # # 'Bezier.h',
        # # 'brentq.c',
        # # 'brentq.h',
        # 'regulator_speedInnerLoop.c',
        # 'regulator_speedInnerLoop.h',
        # 'main_switch.c',
        # 'main_switch.h',
        # 'inverter_Compensation.c',
        # 'inverter_Compensation.h',
        # 'typedef.h'