import subprocess
import time
from time import sleep
import os
import pandas as pd
from pylab import np, plt, mpl

class acm_designer(object):
    def __init__(self, work_dir):
        self.work_dir = work_dir # os.path.dirname(os.path.realpath(__file__))

        # optimization related 
        self.bounds_denorm = [  [100, 1000], # BW-current
                                [1.1, 5] ] # delta
        self.counter_fitness_called = 0
        self.counter_fitness_return = 0

        # # simulation-data-aquisition related
        # CL_TS = CL_TS

    def evaluate_design(self, x_denorm):

        with open('_eval/ACMConfig.h', 'r') as f:
            new_line = []
            for line in f.readlines():
                if '#define CURRENT_LOOP_BANDWIDTH' in line:
                    new_line.append(f'#define CURRENT_LOOP_BANDWIDTH (2*M_PI*{x_denorm[0]:g})\n') # 15
                elif '#define DELTA_THE_DAMPING_FACTOR' in line:
                    new_line.append(f'#define DELTA_THE_DAMPING_FACTOR ({x_denorm[1]:g})\n') # 0.08
                else:
                    new_line.append(line)
        with open('_eval/ACMConfig.h', 'w') as f:
            f.writelines(new_line)

        if os.path.exists(self.work_dir+'/_eval/main.exe'):
            os.remove(self.work_dir+'/_eval/main.exe')
        subprocess.run( ["gcc", "_eval/main.c", "_eval/controller.c", "_eval/observer.c", "-o", "_eval/main"] )
        while not os.path.exists(self.work_dir+'/_eval/main.exe'):
            time.sleep(0.1)
            print('sleep for main.exe')

        # subprocess.run( [self.work_dir+'/_eval/main.exe'] )
        os.system('cd _eval && main')
        while not os.path.exists(self.work_dir+'/_eval/pi_opti.dat'):
            time.sleep(0.1)
            print('sleep for .dat')

        speed_profile = np.loadtxt('_eval/pi_opti.dat', skiprows=1)
        os.remove(self.work_dir+'/_eval/pi_opti.dat')

        TS = 0.000025 # ACMConfig.h
        print( max(speed_profile), 'rpm' )
        print( min(speed_profile, key=lambda x:abs(x-90)), 'rpm' ) # [rpm]
        print( np.abs(speed_profile-90).argmin() * TS * 1000, 'ms' ) # [ms]

        rise_time = np.abs(speed_profile-90).argmin() * TS * 1000

        over_shoot = max(speed_profile)
        if over_shoot<=100:
            over_shoot = 100

        # modify this function to call matlab and eMach to evaluate the design with free variables as in x_denorm
        return over_shoot, rise_time

    def update_ACMConfig_evaluate_PI_coefficients(self, currentPI, speedPI, 
            上位机电流PI, 上位机速度PI, 
            max_freq=None, init_freq=2,
            motor_dict=dict(), 
            SWEEP_FREQ_C2V=False,
            SWEEP_FREQ_C2C=False):

        self.motor_dict = motor_dict
        CL_TS = self.motor_dict['CL_TS']

        if max_freq is not None:
            freq_step_size = 1
            endTime = 0
            for current_freq in range(init_freq, max_freq+1, freq_step_size):
                endTime += 1.0/current_freq # 1.0 Duration for each frequency
            NUMBER_OF_STEPS_CL_TS = endTime/CL_TS + 10000
        else:
            endTime = motor_dict['EndTime']
            NUMBER_OF_STEPS_CL_TS = endTime/CL_TS

        if SWEEP_FREQ_C2C:
            # self.data_fname = f"../dat/{data_fname_prefix}-{上位机电流PI[0]:.0f}-{上位机电流PI[1]:.0f}-{上位机速度PI[0]:.0f}-{上位机速度PI[1]:.0f}@{max_freq:.0f}Hz.dat"
            self.data_fname = f"../dat_Closed/{motor_dict['data_fname_prefix']}-@{max_freq:.0f}Hz-{上位机电流PI[0]:.0f}-{上位机电流PI[1]:.0f}-{上位机速度PI[0]:.0f}-{上位机速度PI[1]:.0f}.dat"
        elif SWEEP_FREQ_C2V:
            self.data_fname = f"../dat_Open/{motor_dict['data_fname_prefix']}-@{max_freq:.0f}Hz-{上位机电流PI[0]:.0f}-{上位机电流PI[1]:.0f}-{上位机速度PI[0]:.0f}-{上位机速度PI[1]:.0f}.dat"
        else:
            self.data_fname = f"../dat/{motor_dict['data_fname_prefix']}-{上位机电流PI[0]:.0f}-{上位机电流PI[1]:.0f}-{上位机速度PI[0]:.0f}-{上位机速度PI[1]:.0f}.dat"

        with open(self.work_dir+'/c/ACMConfig.h', 'r') as f:
            new_line = []
            for line in f.readlines():
                if   '#define NUMBER_OF_STEPS' in line: new_line.append(f'#define NUMBER_OF_STEPS {NUMBER_OF_STEPS_CL_TS:.0f}\n')
                elif '#define CURRENT_KP ' in line: new_line.append(f'#define CURRENT_KP ({currentPI[0]:g})\n')
                elif '#define CURRENT_KI ' in line: new_line.append(f'#define CURRENT_KI ({currentPI[1]:g})\n')
                elif '#define SPEED_KP '   in line: new_line.append(f'#define SPEED_KP ({speedPI[0]:g})\n')
                elif '#define SPEED_KI '   in line: new_line.append(f'#define SPEED_KI ({speedPI[1]:g})\n')
                elif '#define CL_TS '      in line: new_line.append(f'#define CL_TS ({CL_TS:g})\n')
                elif '#define CL_TS_INVERSE'        in line: new_line.append(f'#define CL_TS_INVERSE ({1.0/CL_TS:g})\n')
                elif '#define SPEED_LOOP_CEILING'   in line: new_line.append(f'#define SPEED_LOOP_CEILING (4)\n')
                elif '#define SWEEP_FREQ_MAX_FREQ'  in line: new_line.append(f'#define SWEEP_FREQ_MAX_FREQ {max_freq:.0f}\n')
                elif '#define SWEEP_FREQ_INIT_FREQ' in line: new_line.append(f'#define SWEEP_FREQ_INIT_FREQ {init_freq:.0f}\n')
                elif '#define SWEEP_FREQ_C2V'       in line: new_line.append(f'#define SWEEP_FREQ_C2V {"TRUE" if SWEEP_FREQ_C2V else "FALSE"}\n')
                elif '#define SWEEP_FREQ_C2C'       in line: new_line.append(f'#define SWEEP_FREQ_C2C {"TRUE" if SWEEP_FREQ_C2C else "FALSE"}\n')
                elif '#define PMSM_NUMBER_OF_POLE_PAIRS'          in line: new_line.append(f'#define PMSM_NUMBER_OF_POLE_PAIRS          {motor_dict["n_pp"]}\n')
                elif '#define PMSM_RESISTANCE'                    in line: new_line.append(f'#define PMSM_RESISTANCE                    {motor_dict["Rs"]}\n')
                elif '#define PMSM_D_AXIS_INDUCTANCE'             in line: new_line.append(f'#define PMSM_D_AXIS_INDUCTANCE             {motor_dict["Ld"]}\n')
                elif '#define PMSM_Q_AXIS_INDUCTANCE'             in line: new_line.append(f'#define PMSM_Q_AXIS_INDUCTANCE             {motor_dict["Lq"]}\n')
                elif '#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE' in line: new_line.append(f'#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE {motor_dict["KE"]}\n')
                elif '#define PMSM_SHAFT_INERTIA'                 in line: new_line.append(f'#define PMSM_SHAFT_INERTIA                 {motor_dict["J_s"]}\n')
                elif '#define PMSM_RATED_CURRENT_RMS'             in line: new_line.append(f'#define PMSM_RATED_CURRENT_RMS             {motor_dict["IN"]}\n')
                elif '#define PMSM_RATED_POWER_WATT'              in line: new_line.append(f'#define PMSM_RATED_POWER_WATT              {motor_dict["PW"]}\n')
                elif '#define PMSM_RATED_SPEED_RPM'               in line: new_line.append(f'#define PMSM_RATED_SPEED_RPM               {motor_dict["RPM"]}\n')
                elif '#define LOAD_INERTIA'                       in line: new_line.append(f'#define LOAD_INERTIA                       {motor_dict["JLoadRatio"]}\n')
                elif '#define LOAD_TORQUE'                        in line: new_line.append(f'#define LOAD_TORQUE                        {motor_dict["Tload"]}\n')
                elif '#define VISCOUS_COEFF'                      in line: new_line.append(f'#define VISCOUS_COEFF                      {motor_dict["ViscousCoeff"]}\n')
                elif '#define DATA_FILE_NAME'                     in line: new_line.append(f'#define DATA_FILE_NAME "{self.data_fname}"\n')
                else: new_line.append(line)
        with open(self.work_dir+'/c/ACMConfig.h', 'w') as f:
            f.writelines(new_line)        

    def plot_PI_coefficients(self, key_ref='ACM.rpm_cmd', key_qep='sm.omg_elec*RAD_PER_SEC_2_RPM'):
        # read data and process
        df_info = pd.read_csv(self.work_dir+r"/dat/info.dat", na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
        data_file_name = df_info['DATA_FILE_NAME'].values[0].strip()
        # print(data_file_name)
        df_profiles = pd.read_csv(self.work_dir+'/dat/'+data_file_name, na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])

        print(self.work_dir+'/dat/'+data_file_name)
        print(df_profiles)
        sleep(3) # wait 3 sec, or else ,analyzer will read in a incomplete .dat file for df_profiles.

        no_samples = df_profiles.shape[0]
        no_traces  = df_profiles.shape[1]
        print(df_info, 'Simulated time: %g s.'%(no_samples * df_info['CL_TS'].values[0] * df_info['DOWN_SAMPLE'].values[0]), 'Key list:', sep='\n')
        for key in df_profiles.keys():
            print('\t', key)

        t = np.arange(1, no_samples+1) * df_info['DOWN_SAMPLE'].values[0] * df_info['CL_TS'].values[0]

        begin = 0
        end = -1
        if 'rpm' not in key_ref:
            plt.figure(11)
            plt.subplot(311)
            plt.title('[Simulated] Current and Speed Profiles')
            plt.plot(t[begin:end], df_profiles[key_ref].values[begin:end], label=key_ref)
            plt.ylabel(key_ref)
            plt.xlabel('Time [s]')
            plt.legend(loc='upper left')

            plt.subplot(312)
            plt.plot(t[begin:end], df_profiles[key_qep].values[begin:end], '--', label=key_qep, lw=0.9)
            plt.ylabel(key_qep)
            plt.xlabel('Time [s]')
            plt.legend(loc='upper left')

            if 'id' not in key_qep:
                plt.subplot(313)
                plt.plot(t[begin:end], df_profiles[key_qep].values[begin:end]/60*2*np.pi*self.motor_dict["n_pp"], '--', label=key_qep, lw=0.9)
                plt.ylabel('Speed [elec.rad/s]')
                plt.xlabel('Time [s]')
                plt.legend(loc='upper left')
            
        else:
            plt.figure(11)
            plt.title('[Simulated] Speed Profiles')
            plt.plot(t[begin:end], df_profiles[key_ref].values[begin:end], label=key_ref)
            plt.plot(t[begin:end], df_profiles[key_qep].values[begin:end], '--', label=key_qep, lw=0.9)
            plt.ylabel('Speed [rpm]')
            plt.xlabel('Time [s]')
            plt.legend(loc='upper left')
        # plt.show()

        return data_file_name

    def compile_c_and_run(self):
        os.system(f"cd /d {self.work_dir}/c && gmake main && start cmd /c main")
        return self.data_fname


if __name__ == '__main__':

    motor_dict = dict()
    motor_dict['n_pp'] = 4
    motor_dict['Rs']   =  0.152
    motor_dict['Ld']   =  0.000466
    motor_dict['Lq']   =  0.000466
    motor_dict['KE']   =  0.023331
    motor_dict['J_s']  = 1.6000000000000003e-05
    motor_dict['IN']   =  12.8
    motor_dict['PW']   =  400
    motor_dict['RPM']  = 3000 



    CURRENT_KP = 3.22076
    CURRENT_KI = 326.18
    SPEED_KP = 0.0303833
    SPEED_KI = 163.586
    currentPI = [CURRENT_KP, CURRENT_KI]
    speedPI = [SPEED_KP, SPEED_KI]
    上位机电流PI = [1100, 1000]
    上位机速度PI = [302.228, 344.581]


    data_fname_prefix = 'simulator-as-main'

    max_freq = 233*2
    SWEEP_OPEN_LOOP = False

    max_freq = 2000
    SWEEP_OPEN_LOOP = True

    ad = acm_designer()
    ad.update_ACMConfig_evaluate_PI_coefficients(currentPI, speedPI, 上位机电流PI, 上位机速度PI, 
                                                 max_freq=max_freq, 
                                                 motor_dict=motor_dict,
                                                 SWEEP_OPEN_LOOP=SWEEP_OPEN_LOOP)
    data_fname = ad.compile_c_and_run()
    if SWEEP_OPEN_LOOP == True:
        ad.plot_PI_coefficients(key_ref='iq_ref', key_qep='sm.omg')
    else:
        ad.plot_PI_coefficients()
    plt.show()
