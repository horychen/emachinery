#coding:u8
from pylab import plt, mpl, np
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
# from pprint import pprint
from collections import OrderedDict as O
import pandas as pd
# plot style
#style = np.random.choice(plt.style.available); print(style); 
# plt.style.use('grayscale') # ['grayscale', u'dark_background', u'bmh', u'grayscale', u'ggplot', u'fivethirtyeight']
plt.style.use('ggplot')
# plot setting
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.family'] = 'STIXGeneral'
mpl.rcParams['legend.fontsize'] = 12.5
# mpl.rcParams['legend.family'] = 'Times New Roman'
mpl.rcParams['font.family'] = ['Times New Roman']
mpl.rcParams['font.size'] = 14.0
# mpl.style.use('classic')
font = {'family' : 'Times New Roman', #'serif',
        'color' : 'darkblue',
        'weight' : 'normal',
        'size' : 14,}
textfont = {'family' : 'Times New Roman', #'serif',
            'color' : 'darkblue',
            'weight' : 'normal',
            'size' : 11.5,}

import os
from time import sleep

######################
# Plotting
def get_axis(cNr):
    # fig, axes = plt.subplots(ncols=cNr[0], nrows=cNr[1], dpi=150, sharex=True);
    fig, axes = plt.subplots(ncols=cNr[0], nrows=cNr[1], sharex=True, figsize=(16*0.8, 9*0.8), dpi=80, facecolor='w', edgecolor='k');
    fig.subplots_adjust(right=0.95, bottom=0.1, top=0.95, hspace=0.2, wspace=0.02)    
    # fig.subplots_adjust(right=0.85, bottom=0.1, top=0.95, hspace=0.25)
    if sum(cNr)<=2:
        return axes
    else:
        return axes.ravel()

def plot_key(ax, key, df):
    ax.plot(time, df[key].values, '-', lw=1)
    ax.set_ylabel(key, fontdict=font)

def plot_it(ax, ylabel, d, time=None):
    count = 0
    for k, v in d.items():
        if count == 0:
            count += 1
            # ax.plot(time, v, '--', lw=2, label=k)
            ax.plot(time, v, '-.', lw=1)
        else:
            # ax.plot(time, v, '-', lw=2, label=k)
            ax.plot(time, v, '-.', lw=1)

    # ax.legend(loc='lower right', shadow=True)
    # ax.legend(bbox_to_anchor=(1.08,0.5), borderaxespad=0., loc='center', shadow=True)
    ax.set_ylabel(ylabel, fontdict=font)
    # ax.set_xlim(0,35) # shared x
    # ax.set_ylim(0.85,1.45)

def get_data_frame(path='.'):

    # info.dat
    df_info = pd.read_csv(path+"/dat/info.dat", na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
    data_file_name = df_info['DATA_FILE_NAME'].values[0].strip()

    # 这个大的数据文件产生有滞后，可能文件还没生成，程序就已经跑到这里开始调用read_csv了！
    # 所以要判断一下，数据文件应该是在info.dat之后生成的。
    # while True:
    #     print('st_mtime 1:', os.stat(path+'/dat/'+data_file_name).st_mtime )
    #     print('st_mtime 2:', os.stat(path+"/dat/info.dat").st_mtime)
    #     if os.stat(path+'/dat/'+data_file_name).st_mtime < os.stat(path+"/dat/info.dat").st_mtime:
    #         print('Sleep in ACMPlot.py')
    #         sleep(0.1)
    #         break    

    # ???.dat
    df_profiles = pd.read_csv(path+'/dat/'+data_file_name, na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
    no_samples = df_profiles.shape[0]
    no_traces  = df_profiles.shape[1]
    # print(data_file_name)

    print('read in', path+'/dat/'+data_file_name)
    print(df_profiles.shape)
    # print(df_profiles)

    return df_info, df_profiles, no_samples, no_traces 

if __name__ == '__main__':

    df_info, df_profiles, no_samples, no_traces = get_data_frame()
    print(df_info, 'Simulated time: %g s.'%(no_samples * df_info['CL_TS'].values[0] * df_info['DOWN_SAMPLE'].values[0]), 'Key list:', sep='\n')
    for key in df_profiles.keys():
        print('\t', key)

    time = np.arange(1, no_samples+1) * df_info['DOWN_SAMPLE'].values[0] * df_info['CL_TS'].values[0]

    ax_list = []
    for i in range(0, no_traces, 6):
        ax_list += list(get_axis((1,6)))

    for idx, key in enumerate(df_profiles.keys()):
        plot_it(ax_list[idx], key, O([
                                        (str(idx), df_profiles[key]),  
                                        # (str(idx), df_profiles[key]),  
                                        ]), time)
        if (idx+1)%6 == 0:
            ax_list[idx].set_xlabel('time [s]', fontdict=font)


    plt.show()
    quit()


