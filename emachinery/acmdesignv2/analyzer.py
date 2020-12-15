import pandas as pd
from pylab import plt, np
from pylab import fft
import math
import os
# dot_dat_file_dir = os.path.dirname(os.path.realpath(__file__)) + '/'


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx, array[idx]

def analyze(dot_dat_file_dir, motor_dict, sweepFreq_dict, bool_use_commanded_freq=True):

    if(not os.path.exists(dot_dat_file_dir)):
        print('Cannot locate:', dot_dat_file_dir)
        return None

    # read data as Data Frame and process
    # df_info     = pd.read_csv(dot_dat_file_dir+'dat/info.dat', na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
    df_profiles = pd.read_csv(dot_dat_file_dir, na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])

    # print(dot_dat_file_dir)
    # print(df_profiles)

    no_samples = df_profiles.shape[0]
    no_traces  = df_profiles.shape[1]

    Ts = motor_dict['CL_TS']
    # print('Ts =', Ts)

    # print('Simulated time: %g s.'%(no_samples * Ts * motor_dict['DOWN_SAMPLE']), 'Key list:', sep='\n')
    # for key in df_profiles.keys():
    #     print('\t', key)

    # Unpack as Series
    t = np.arange(1, no_samples+1) * motor_dict['DOWN_SAMPLE'] * Ts
    time  = t
    if sweepFreq_dict["SWEEP_FREQ_C2V"]:
        # C2V
        key_ref = 'CTRL.iq_cmd'; # note this is q-axis current command instead of measured q-axis current
        key_qep = 'sm.omg_elec*RAD_PER_SEC_2_RPM';
        x_ref = df_profiles[key_ref] # [Apk]
        x_qep = df_profiles[key_qep]/60*2*np.pi*motor_dict["n_pp"] # [rpm] -> [elec.rad/s]
    elif sweepFreq_dict["SWEEP_FREQ_C2C"]:
        # C2C
        key_ref = 'CTRL.id_cmd';
        key_qep = 'ACM.id';
        x_ref = df_profiles[key_ref] # [Apk] 
        x_qep = df_profiles[key_qep] # [Apk]
    else:
        # V2V
        key_ref = 'ACM.rpm_cmd';
        key_qep = 'sm.omg_elec*RAD_PER_SEC_2_RPM';
        x_ref = df_profiles[key_ref] # [rpm] # 闭环系统传递函数分析单时候，输入输出的单位是一样的，求传递函数的时候一除，就算用的是rpm也都无所谓了（虽然理论上是按elec.rad/s设计的）
        x_qep = df_profiles[key_qep] # [rpm]

    # Basic DFT parameters
    # N = df.shape[0]
    samplingFreq = 1/Ts
    EndTime = t[-1]
    #     print('End Time:', EndTime, 's')
    # print('Sampling Frequency:', samplingFreq*1e-3, 'kHz')
    #     # print('Number of Points:', N)
    #     print()

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
    # print('index_begin', index_begin)
    # print('index_end', index_end)
    # print('time_begin:', time_begin, 's')
    # print('time_end', time_end,   's')
    # time  = time [index_begin:index_end]
    # x_ref = x_ref[index_begin:index_end]
    # x_qep = x_qep[index_begin:index_end]
    # plt.figure(100, figsize=(20,4))
    # plt.title('Origianl Signal')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Speed [elec.rad/s]')
    # plt.plot(time, x_ref, label='ref')
    # # plt.figure(101)
    # plt.plot(time, x_qep, label='qep')
    # plt.show()
    # print('----------------')
    # print('xref')
    # for el in x_ref.values:
    #     print(el)
    # print('xqep')
    # for el in x_qep.values:
    #     print(el)
    # # # plt.savefig(r'C:\Users\horyc\Desktop\test.png')
    # quit()

    #     # plt.xlim([0, 8/target_Hz])
    #     print()
    #     print('Max reference speed:', max(x_ref))
    #     print('Max measured speed:', max(x_qep))
    #     print('Min reference speed:', min(x_ref))
    #     print('Min measured speed:', min(x_qep))


    #     print()
    #     TimeSpan = time.iloc[-1]
    #     print('Time Span:', TimeSpan, 's')

    list_qep_max_amplitude = []
    list_qep_max_frequency = []
    list_qep_max_phase = []
    list_ref_max_amplitude = []
    list_ref_max_frequency = []
    list_ref_max_phase = []
    list_phase_difference = []


    list_commanded_frequency = []

    index_single_tone_begin = 0
    index_single_tone_end = 0
    # max_freq = 5 # debug
    # for freq in range(2, max_freq): # datum point at 1 Hz and last point are absent in experiment
    for freq in range(sweepFreq_dict['init_freq'], sweepFreq_dict['max_freq']):
        period = 1.0/freq # 1.0 Duration for each frequency
        index_single_tone_begin = index_single_tone_end
        index_single_tone_end = index_single_tone_begin + int(period/Ts)

        if False:
            # use only steady state profile for this frequency
            ST_time  =  time[int(index_single_tone_end-0.1*(index_single_tone_end-index_single_tone_begin)):index_single_tone_end]
            ST_x_ref = x_ref[int(index_single_tone_end-0.1*(index_single_tone_end-index_single_tone_begin)):index_single_tone_end]
            ST_x_qep = x_qep[int(index_single_tone_end-0.1*(index_single_tone_end-index_single_tone_begin)):index_single_tone_end]
        else:
            # use complete profile for this frequency
            ST_time  =  time[index_single_tone_begin:index_single_tone_end]
            ST_x_ref = x_ref[index_single_tone_begin:index_single_tone_end]
            ST_x_qep = x_qep[index_single_tone_begin:index_single_tone_end]
            # plt.plot(ST_time, ST_x_ref)
            # plt.plot(ST_time, ST_x_qep)

        # print(freq, index_single_tone_begin, index_single_tone_end)

        if(len(ST_x_ref))<1:
            print('sweep frequency too high: no data')
            break

        x_ref_dft = fft(ST_x_ref)
        x_qep_dft = fft(ST_x_qep)

        # Do DFT (Raw)
        N = len(ST_time)
        # plt.figure(1, figsize=(20,4))
        # plt.plot(abs(x_ref_dft)/N, '.--', alpha=0.5, label='x_ref-dft');
        # plt.plot(abs(x_qep_dft)/N, '.--', alpha=0.5, label='x_qep-dft');
        # #     plt.legend(loc='center')

        # Convert raw DFT results into human-friendly forms
        resolution = samplingFreq/N # [Hz]
        Neff = math.ceil(N/2) # number of effective points
                              # 其实理论上来说，这里是比较复杂的，当N为偶数的时候，奈奎斯特频率就是采样频率的二分之一？当N为奇数的时候，还会多出一个分量，这个分量和直流分量是一对，具体我忘了……可能有错
        x_ref_hat = np.append(x_ref_dft[0]/N, 2*x_ref_dft[1:Neff+1]/N)  # 原始复数dft结果（双边变单边，除了直流分量，其他分量全部要乘以2）
        x_qep_hat = np.append(x_qep_dft[0]/N, 2*x_qep_dft[1:Neff+1]/N)

        # # Plot DFT for human to read
        # plt.figure(2, figsize=(20,4))
        # plt.plot(np.array(list(range(0, Neff+1)))*resolution, abs(x_ref_hat), '--s', alpha=0.5, label='ref');
        # plt.plot(np.array(list(range(0, Neff+1)))*resolution, abs(x_qep_hat), '--o', alpha=0.5, label='qep');

        # qep related data collection
        max_amplitude = max(abs(x_qep_hat));      list_qep_max_amplitude.append(max_amplitude)
        max_index     = np.argmax(abs(x_qep_hat))
        max_frequency = (max_index+0)*resolution; list_qep_max_frequency.append(max_frequency)
        # qep phase
        max_complexNumber = x_qep_hat[max_index]
        max_phase = np.arctan2(max_complexNumber.imag, max_complexNumber.real); list_qep_max_phase.append(max_phase)
        qep_complexNumber = max_complexNumber

        #     print(f'Frequency Resolution: {resolution:.2f} Hz', end=' | ')
        #     print(f'Max Amplitude: {max_amplitude:.2f} rpm', end=' | ')
        #     print(f'Corresponding Frequency: {max_frequency:.2f} Hz')

        # ref related data collection
        max_amplitude = max(abs(x_ref_hat));      list_ref_max_amplitude.append(max_amplitude)
        max_index     = np.argmax(abs(x_ref_hat))
        max_frequency = (max_index+0)*resolution; list_ref_max_frequency.append(max_frequency)
        # ref phase
        max_complexNumber = x_ref_hat[max_index]
        max_phase = np.arctan2(max_complexNumber.imag, max_complexNumber.real); list_ref_max_phase.append(max_phase)
        ref_complexNumber = max_complexNumber

        # phase difference
        list_phase_difference.append( -np.arccos( (qep_complexNumber.real*ref_complexNumber.real+qep_complexNumber.imag*ref_complexNumber.imag)
                                                 /(abs(ref_complexNumber)*abs(qep_complexNumber)) )/np.pi*180 )
        # commanded frequency
        list_commanded_frequency.append(freq)

    closed_loop_transfer_function = [qep/ref for ref, qep in zip(list_ref_max_amplitude, list_qep_max_amplitude)]
    CL_VL_TF = [20*np.log10(el) for el in closed_loop_transfer_function]

    if bool_use_commanded_freq == True:
        return CL_VL_TF, list_phase_difference, list_commanded_frequency, sweepFreq_dict['max_freq']
    else:
        return CL_VL_TF, list_phase_difference, list_qep_max_frequency, sweepFreq_dict['max_freq']

# def plot()
#     plt.figure(4, figsize=(20,8))
#     closed_loop_transfer_function = [qep/ref for ref, qep in zip(list_ref_max_amplitude, list_qep_max_amplitude)]

#     CL_VL_TF_ABS = [20*np.log10(el) for el in closed_loop_transfer_function]
#     if bool_use_commanded_freq == True:
#         plt.plot(list_commanded_frequency, CL_VL_TF_ABS, '--.', label=fname)
#     else:
#         plt.plot(list_qep_max_frequency, CL_VL_TF_ABS, '--.', label=fname)


#     print(key_ref, key_qep)
#     if 'speed' in key_ref:
#         index, value = find_nearest(CL_VL_TF_ABS, -3) # find -3dB freq
#         VLBW = list_qep_max_frequency[index]
#         plt.text(VLBW, -5, f'{VLBW:.0f} Hz', color='red', fontsize=20)
#         # plt.xticks(range(0,max_freq+1, int(max_freq/20)))
#         plt.plot([0,max_freq], [-3, -3], 'k--')
#         plt.ylabel('Velocity Closed-loop transfer function amplitude [dB]')
#     else:
#         if 'id' not in key_qep:
#             plt.ylabel('C2V [dB] (elec.rad/s/A)')
#         else:
#             plt.ylabel('Current Closed-loop transfer function amplitude [dB]')

#     # plt.text(np.log10(VLBW), -5, f'{VLBW:.0f} Hz', fontsize=20)
#     # plt.plot(CL_VL_TF_FREQ, CL_VL_TF_ABS, '--.', label=fname)
#     # plt.plot([0,np.log10(max_freq)], [-3, -3], 'k--')

#     plt.xscale('log')
#     plt.xlabel('Frequency [Hz]')
#     #plt.ylabel('Closed-loop transfer function amplitude [1]')
#     # plt.xlim([0,50])
#     # plt.ylim([-10,0])
#     # plt.grid()
#     plt.legend()

def folder(dot_dat_file_dir, motor_dict, data_fname_prefix='default', **kwarg):
    active_data_fnames = []
    max_freq_list = []
    for fname in os.listdir(dot_dat_file_dir+'../_simulator/dat/'):
        if data_fname_prefix in fname:
            active_data_fnames.append(fname)
            max_freq_list.append(int(fname[fname.find('@')+1:fname.find('Hz')]))
    # print(max_freq_list)
    # quit()

    # Target file name
    for fname, max_freq in zip(active_data_fnames, max_freq_list):
        analyze(dot_dat_file_dir, fname, max_freq, ad, **kwarg)

    # print('Measured bandwidth is', VLBW, 'Hz')
