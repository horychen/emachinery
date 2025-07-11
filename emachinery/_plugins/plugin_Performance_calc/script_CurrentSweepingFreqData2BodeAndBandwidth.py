# %reset -f
import pandas as pd
from pylab import plt, np
from pylab import fft
import math
import os
# %matplotlib inline

#TODO: WE need move the plotting of this script to keep it clean
def BodeAndBandwidthCalc_BasedOn_CJHScript(max_freq, begin_freq):
    # Target file name
    # fname = 'data/SweepingData.txt'
    # fname = 'data/SweepingData.txt'
    # fname = 'data/SpeedSweepingData.txt'
    fname = os.path.dirname(__file__) + '/data/CurrentSweepingData.txt'
    # max_freq = 1000 # [H200
    # Read name and meta data
    signal_names = []
    with open(fname, 'r', encoding='gb2312') as f:
        for _ in range(5):
            line = f.readline()
            print(line)
            if 'Unit' in line:
                signal_name = line[line.find('Name')+5 : line.find('Unit')]
                signal_names.append(signal_name)

    # read data as Data Frame
    df = pd.read_csv(fname, skiprows=7, sep='\s+', encoding='gb2312', names=['Time(s)', signal_names[0], signal_names[1]])

    # Unpack as Series
    time  = df['Time(s)']
    x_ref = df[signal_names[0]]
    x_qep = df[signal_names[1]] #+ 100 # add dc offset for test

    # Basic DFT parameters
    # N = df.shape[0]
    Ts = df['Time(s)'][1] - df['Time(s)'][0]
    samplingFreq = 1/Ts
    EndTime = df['Time(s)'].iloc[-1]
    print('End Time:', EndTime, 's')
    print('Sampling Frequency:', samplingFreq*1e-3, 'kHz')

    # Plot signal in time domain
    for index, value in enumerate(x_ref):
        if value!=x_ref.iloc[0]:
            index_begin = index
            time_begin = index*Ts
            break
    for index, value in enumerate(x_ref[::-1]):
        if value!=x_ref.iloc[-1]:
            index_end = -index
            time_end = EndTime - index*Ts
            break
    print('index_begin', index_begin)
    print('index_end', index_end)
    print('time_begin:', time_begin, 's')
    print('time_end', time_end,   's')
    time  = time [index_begin:index_end]
    x_ref = x_ref[index_begin:index_end]
    x_qep = x_qep[index_begin:index_end]
    plt.figure(figsize=(20,4))
    plt.title('Origianl Signal')
    plt.xlabel('Time [s]')
    plt.ylabel('Current [A]')
    plt.plot(time, x_ref, label='ref')
    plt.plot(time, x_qep, label='qep')

    # plt.xlim([0, 8/target_Hz])
    print()
    print('Max reference current:', max(x_ref))
    print('Max measured  current:', max(x_qep))
    print('Min reference current:', min(x_ref))
    print('Min measured  current:', min(x_qep))


    print()
    TimeSpan = time.iloc[-1]
    print('Time Span:', TimeSpan, 's')

    ######################################################### Do FFT to obtain the freq

    list_qep_max_amplitude = []
    list_qep_max_frequency = []
    list_ref_max_amplitude = []
    list_ref_max_frequency = []

    index_single_tone_begin = 0
    index_single_tone_end = 0
    # max_freq = 100 # debug
    ######
    # 原脚本的循环是从2Hz开始，这是因为扫频扫1Hz，没有什么意义，1s太长了，但是这就牵扯到一个问题
    # 怎么把1s后的数据拿出来，而不是简单的数据赋值，以后要么规定从2Hz开始扫吧！
    # 这样的话1就要改成2了
    ######
    sweepingFreq_begin = begin_freq # [Hz]
    ######
    for freq in range(sweepingFreq_begin, max_freq): # datum point at 1 Hz is absent
        period = 1/freq
        index_single_tone_begin = index_single_tone_end
        index_single_tone_end = index_single_tone_begin + int(period/Ts)
            
        ST_time  =  time[index_single_tone_begin:index_single_tone_end]
        ST_x_ref = x_ref[index_single_tone_begin:index_single_tone_end]
        ST_x_qep = x_qep[index_single_tone_begin:index_single_tone_end]

        if len(ST_x_ref) > 0 and len(ST_x_qep) > 0:
            x_ref_dft = fft(ST_x_ref)
            x_qep_dft = fft(ST_x_qep)
        else:
            print(f"Skipping FFT for frequency {freq} due to insufficient data points.")
        # Do DFT (Raw)
        N = len(ST_time)
        plt.figure(1, figsize=(20,4))
        plt.plot(abs(x_ref_dft)/N, '.--', alpha=0.5, label='x_ref-dft');
        plt.plot(abs(x_qep_dft)/N, '.--', alpha=0.5, label='x_qep-dft');
        #     plt.legend(loc='center')

        # Convert raw DFT results into human-friendly forms
        resolution = samplingFreq/N # [Hz]
        Neff = math.ceil(N/2) # number of effective points
                            # 其实理论上来说，这里是比较复杂的，当N为偶数的时候，奈奎斯特频率就是采样频率的二分之一？当N为奇数的时候，还会多出一个分量，这个分量和直流分量是一对，具体我忘了……可能有错
        x_ref_hat = np.append(x_ref_dft[0]/N, 2*x_ref_dft[1:Neff+1]/N)  # 原始复数dft结果（双边变单边，除了直流分量，其他分量全部要乘以2）
        x_qep_hat = np.append(x_qep_dft[0]/N, 2*x_qep_dft[1:Neff+1]/N)

        # Plot DFT for human to read
        plt.figure(2, figsize=(20,4))
        plt.plot(np.array(list(range(0, Neff+1)))*resolution, abs(x_ref_hat), '--s', alpha=0.5, label='ref');
        plt.plot(np.array(list(range(0, Neff+1)))*resolution, abs(x_qep_hat), '--o', alpha=0.5, label='qep');

        # qep related data collection
        max_amplitude = max(abs(x_qep_hat));      list_qep_max_amplitude.append(max_amplitude)
        max_index     = np.argmax(abs(x_qep_hat))
        max_frequency = (max_index+0)*resolution; list_qep_max_frequency.append(max_frequency)

        # ref related data collection
        max_amplitude = max(abs(x_ref_hat));      list_ref_max_amplitude.append(max_amplitude)
        max_index     = np.argmax(abs(x_ref_hat))
        max_frequency = (max_index+0)*resolution; list_ref_max_frequency.append(max_frequency)
        
    ######################################################### To get the bode
    plt.figure(3, figsize=(20,4))
    if max_freq>100:
        #TODO：这里的100其实可以改成输入值
        list_ref_max_frequency = list(range(2, max_freq))
        list_qep_max_frequency = list(range(2, max_freq))
    # TODO：修复bug，这里freq缺1Hz    
    # Warning：这里我手插如一个1Hz的信号，这么干是不对的！！！可是为了画图，先这么干吧
    list_ref_max_frequency.insert(0, 1)
    list_qep_max_frequency.insert(0, 1)
    ######## 2Hz Version ########
    # plt.figure(3, figsize=(20,4))
    # if max_freq>500:
    #     list_ref_max_frequency = list(range(2, max_freq))
    #     list_qep_max_frequency = list(range(2, max_freq))

    # plt.plot(list_ref_max_frequency, list_ref_max_amplitude, '--')
    # plt.plot(list_qep_max_frequency, list_qep_max_amplitude, '--')
    # plt.xlabel('Freq [Hz]')
    # plt.ylabel('Speed [rpm]')

    ######################################################### plot Bode

    closed_loop_transfer_function = [qep/ref for ref, qep in zip(list_ref_max_amplitude, list_qep_max_amplitude)]

    ######################################################### calculation for bandwidth

    plt.figure(4, figsize=(20,4))
    # closed_loop_transfer_function = [qep/ref for ref, qep in zip(list_ref_max_amplitude, list_qep_max_amplitude)]
    dB_values = [20*np.log10(el) for el in closed_loop_transfer_function]
    plt.plot(list_qep_max_frequency, dB_values, '--')

    # Add -3dB line
    plt.axhline(y=-3, color='r', linestyle='-')

    # Find -3dB point
    for i, dB in enumerate(dB_values):
        if dB < -3:
            break
        
    freq_at_minus_3dB = list_qep_max_frequency[i]
    # Convert to rad/s
    freq_at_minus_3dB_rad_s = freq_at_minus_3dB * 2 * np.pi

    return freq_at_minus_3dB, freq_at_minus_3dB_rad_s




