from jsons import ACMInfo
from acmdesignv2 import tuner
from acmdesignv2 import simulator 
from acmdesignv2 import analyzer

# import os 
from pylab import np, plt, mpl
import control
mpl.rcParams['figure.dpi'] = 120
plt.style.use('ggplot')

# import Experiment


if __name__ == '__main__':

    mj = ACMInfo.MotorJson().d
    motor = mj["SEW400W (SF60B04030C2004)"]["基本参数"]

    n_pp = motor["极对数 [1]"]
    R    = motor["电机线电阻 [Ohm]"]/2
    L    = motor["电机D轴线电感 [mH]"]/2*1e-3
    KE   = motor["转矩常数 [Nm/Arms]"] / 1.5 / n_pp * 1.414
    J_s  = motor["转动惯量 [kg.cm^2]"]*1e-4
    IN   = motor["额定电流 [Arms]"]
    PW   = motor["额定功率 [Watt]"]
    RPM  = motor["额定转速 [rpm]"]

    motor_dict = dict()

    motor_dict['DOWN_SAMPLE'] = 1

    motor_dict['n_pp'] = n_pp
    motor_dict['Rs'] = R
    motor_dict['Ld'] = L
    motor_dict['Lq'] = L
    motor_dict['KE'] = KE
    motor_dict['J_s'] = J_s
    motor_dict['IN'] = IN
    motor_dict['PW'] = PW
    motor_dict['RPM'] = RPM

    CL_TS = 1/20e3
    VL_TS = 4*CL_TS # if modified, need to change SPEED_LOOP_CEILING in ACMConfig.h as well
    motor_dict['CL_TS'] = CL_TS
    motor_dict['VL_TS'] = VL_TS

    motor_dict['JLoadRatio'] = JLoadRatio = 0.16 # 0.16 # 3 [%]
    motor_dict['Tload'] = TLoad = 0.0 # 0.05 # [Nm]
    motor_dict['ViscousCoeff'] = B = 0.7e-4

if __name__ == '__main__':
# for JLoadRatio in [0.16, 3, 10, 30]:
    motor_dict['JLoadRatio'] = JLoadRatio

    # Iterate for a preset speed loop bandwidth based on your delta value

    # Pick your favorite step repsonse by "shape" via damping factor \delta
    # for delta in [2, 4, 6.5, 8, 10, 15, 20]:
    # for delta in [2, 6.5,  15]:
    for delta in [6.5]:
    # for delta in [15]:

        # Specify your desired speed closed-loop bandwidth
        # desired_BW_velocity_HZ = 223
        desired_BW_velocity_HZ = 100

        currentPI, speedPI, 上位机电流PI, 上位机速度PI, MagPhaseOmega, BW_in_Hz \
            = tuner.iterate_for_desired_bandwidth(delta, desired_BW_velocity_HZ, motor_dict)
        currentKp, currentKi = currentPI
        speedKp, speedKi = speedPI
        上位机电流KP, 上位机电流KI = 上位机电流PI
        上位机速度KP, 上位机速度KI = 上位机速度PI
        print(f'\n\n\
            #define CURRENT_KP {currentKp:g}\n\
            #define CURRENT_KI {currentKi:g}\n\
            #define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)\n\
            #define SPEED_KP {speedKp:g}\n\
            #define SPEED_KI {speedKi:g}\n\
            #define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)\n')

        fig5 = plt.figure(5)
        fig5.axes[0].set_ylim([-3, 10]) # -3dB
        fig5.axes[1].set_ylim([-90, 0]) # 90 deg
        print('------------end of tuner\n\n\n')



        motor_dict['data_fname_prefix'] = 'demo-closedLoop'

        max_freq = 2*desired_BW_velocity_HZ
        init_freq = 2

        # change to false to save time
        if True:
            ad = simulator.acm_designer(r'D:\DrH\Codes\acmsimcv5') # os.path.dirname(os.path.realpath(__file__))
            ad.update_ACMConfig_evaluate_PI_coefficients(currentPI, speedPI, 上位机电流PI, 上位机速度PI, 
                                                         max_freq=max_freq, init_freq=init_freq,
                                                         motor_dict=motor_dict)
            data_fname = ad.compile_c_and_run()
            # plt.figure()
            data_file_name = ad.plot_PI_coefficients()
            print('------------end of simulator\n\n\n')
            # plt.show()

        # 1. Ploe simualted Bode plot
        dot_dat_file_dir = r'D:/DrH/Codes/acmsimcv5/dat/' + data_fname

        sweepFreq_dict = dict()
        sweepFreq_dict["max_freq"] = max_freq
        sweepFreq_dict["init_freq"] = init_freq
        sweepFreq_dict["SWEEP_FREQ_C2V"] = False
        sweepFreq_dict["SWEEP_FREQ_C2C"] = False

        print(motor_dict)
        print(sweepFreq_dict)
        dB, Deg, Freq, max_freq = analyzer.analyze(dot_dat_file_dir, motor_dict, sweepFreq_dict)
        # for el in (dB, Deg, Freq, max_freq):
        #     print(el)

        plt.figure(4, figsize=(20,8))
        plt.plot(Freq, dB, '--.', label=data_fname)

        index, value = analyzer.find_nearest(dB, -3) # find -3dB freq
        VLBW = Freq[index]
        plt.text(VLBW, -5, f'{VLBW:.0f} Hz', color='red', fontsize=20)
        plt.plot([0,max_freq], [-3, -3], 'k--')
        plt.ylabel('Velocity Closed-loop transfer function amplitude [dB]')

        plt.xscale('log')
        plt.xlabel('Frequency [Hz]')
        plt.legend()


        # 2. Plot designed Bode plot
        # plt.figure(4, figsize=(20,8))
        mag, phase, omega = MagPhaseOmega
        index_max_freq = sum(omega/(2*np.pi) < max_freq)
        plt.plot((omega/(2*np.pi))[:index_max_freq], 20*np.log10(mag[:index_max_freq]), '-.', label=f'designed:$\\delta={delta}$')

        # 3. Plot measured Bode plot
        # fname = r'D:\ISMC\SweepFreq\Jupyter\VLBW-Data/' + 'BiasSine500rpm' + data_file_name[data_file_name.find(data_fname_prefix)+len(data_fname_prefix)+len('-CLOSED-@'):-4]+'.txt'
        # try:
        #     CL_VL_TF, list_phase_difference, list_qep_max_frequency, max_freq = Experiment.analyze_experimental_measurements(fname)
        # except FileNotFoundError as e:
        #     raise e
        #     print(str(e))
        #     pass
        # except Exception as e:
        #     raise e
        # finally:
        #     index, value = Experiment.find_nearest(CL_VL_TF, -3) # find -3dB freq
        #     VLBW = list_qep_max_frequency[index]
        #     plt.text(VLBW, -5, f'{VLBW:.0f} Hz', color='purple', fontsize=20)
        #     plt.plot(list_qep_max_frequency, CL_VL_TF, '--.', label=fname)

        plt.legend()
        # break


        # 潜在误差来源：DFT提取的幅值是有损失的！

if __name__ == '!__main__':
    analyzer.main(ad, data_fname_prefix=data_fname_prefix)
    print('------------end of analyzer\n\n\n')

if __name__ == '__main__':
    plt.show()
