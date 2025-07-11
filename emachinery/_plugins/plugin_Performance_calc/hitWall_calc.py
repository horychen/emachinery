import pandas as pd
import numpy as np

def min_max_performance_calc(cplot_data):

    # 以pd.DataFrame的形式读取数据
    plot_data = pd.DataFrame(cplot_data)
    # 以字典的key的读取数据
    ACM_iD  = cplot_data['ACM.iDQ[0]']
    ACM_iQ  = cplot_data['ACM.iDQ[1]']
    ACM_uA  = cplot_data['ACM.uAB[0]']
    ACM_uB  = cplot_data['ACM.uAB[1]']


    # 以pd.DataFrame的形式求max和min
    process_data = pd.DataFrame({
    'ACM_iD (A)': ACM_iD,
    'ACM_iQ (A)': ACM_iQ,
    'ACM_u_α (V)': ACM_uA,
    'ACM_u_β (V)': ACM_uB
    })
    df = pd.DataFrame(process_data)
    min_values = df.min()
    max_values = df.max()
    # 合并最小值和最大值到一个数据框
    min_max_df = pd.DataFrame({
        'Min': min_values,
        'Max': max_values
    })
    
    return min_max_df
