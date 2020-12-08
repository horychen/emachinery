# https://www.youtube.com/watch?v=2C5VnE9wPhk
# background-color: rgb(255, 255, 255)

import pkg_resources # to include resource file: mainWindow.ui # https://stackoverflow.com/questions/6028000/how-to-read-a-static-file-from-inside-a-python-package/20885799

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi
from PyQt5 import QtCore #, QtGui, QtWidgets

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

from pylab import np, mpl, plt
plt.style.use('ggplot')
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.family'] = 'STIXGeneral'
import matplotlib.animation as animation
import pandas as pd
import random

import os, sys
from collections import OrderedDict
import json
import re
from time import sleep

from emachinery.utils.conversion import ElectricMachinery
from emachinery.jsons import ACMInfo
from emachinery.gui.stylesheet.toggle_stylesheet import toggle_stylesheet

# Promoted Widgets: use these two absolute imports to replace the relative imports in mainWindow.py for uploading to PyPI
# from emachinery.gui.consolewidget import ConsoleWidget
# from emachinery.gui.mplwidget import MplWidget

from emachinery.gui.mainWindow_v2 import Ui_MainWindow

from emachinery.gui import latexdemo

Help = r''' 
If you want to import a new module (say tuner) in a new directory (acmdesignv2), you need to follow these steps:

1. Open __init__.py in emachinery folder, and add path to sys.
2. Create an __init__.py file inside your new directory.
3. Cd to D:\DrH\Codes\emachineryTestPYPI and do: pip install -e .
4. You will see following message:
    ---------------------------
    D:\DrH\Codes\emachineryTestPYPI>pip install -e .
    Obtaining file:///D:/DrH/Codes/emachineryTestPYPI
    Installing collected packages: emachinery
      Attempting uninstall: emachinery
        Found existing installation: emachinery 1.0.3
        Uninstalling emachinery-1.0.3:
          Successfully uninstalled emachinery-1.0.3
      Running setup.py develop for emachinery
    Successfully installed emachinery

Or else,
You are going to see the following error message:
    ---------------------------
    Traceback (most recent call last):
      File "D:\DrH\Codes\emachineryTestPYPI\emachinery\gui\main.py", line 28, in <module>
        from emachinery.acmdesignv2 import tuner
    ModuleNotFoundError: No module named 'emachinery.acmdesignv2'

If you see error message similar to:
    'EmachineryWidget' object has no attribute 'lineEdit_path2boptPython'
this is likely that you should call self.ui.lineEdit_path2boptPython instead of self.lineEdit_path2boptPython.
'''
from emachinery.acmdesignv2 import tuner
from emachinery.acmdesignv2 import simulator


class EmachineryWidget(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)

        ''' generate file path within package
        '''
        # __name__ in case you're within the package
        # - otherwise it would be 'lidtk' in this example as it is the package name
        filepath_to_mainWindow_v2 = pkg_resources.resource_filename(__name__, 'mainWindow_v2.ui')  # always use slash

        ''' load ui or import ui class
        '''
        try:
            # raise ModuleNotFoundError
            print('CJH: plan A to load .ui file.\n')
            self.ui = loadUi(filepath_to_mainWindow_v2, self) # baseinstance=self
        except ModuleNotFoundError as e:
            # raise e
            print('Error caught:', str(e)) # No module named 'consolewidget' # This module is imported in the .ui file so it will not be found.
            print('CJH: will use plan B now.\n')
            self.ui = Ui_MainWindow()
            self.ui.setupUi(self)
        except Exception as e:
            raise e

        '''tab: Name Plate Data
        '''
        self.mj = ACMInfo.MotorJson().d
        self.ui.comboBox_MachineName.addItems(self.mj.keys())
        self.ui.comboBox_MachineName.activated.connect(self.comboActivate_namePlateData)
        self.comboActivate_namePlateData()

        self.motor_dict = self.get_motor_dict(self.mj)

        '''tab_2: Plots
        '''
        # update plot
        self.ui.pushButton_getSignal.clicked.connect(self.update_graph)
        # undate model
        self.ui.pushButton_updateModel.clicked.connect(self.update_emy)

        # Matplotlib navigation bar to: self or tabWidget
        self.toolbar = NavigationToolbar(self.ui.MplWidget.canvas, self)
            # self.addToolBar(self.toolbar) # add to mainWindow
        self.ui.verticalLayout_inTab2.addWidget(self.toolbar) # add to tab 2 only

        '''tab_3: FEA-based Optimization
        '''
        try:
            path = self.ui.lineEdit_path2boptPython.text()
            with open(path+'/codes3/machine_specifications.json', 'r', encoding='utf-8') as f:
                self.bopt_fea_config_dict = json.load(f, object_pairs_hook=OrderedDict) # https://stackoverflow.com/questions/10844064/items-in-json-object-are-out-of-order-using-json-dumps/23820416
            self.ui.comboBox_MachineSpec.addItems(self.bopt_fea_config_dict.keys())
            self.ui.comboBox_MachineSpec.activated.connect(self.update_machineSpec)
            self.update_machineSpec()
            with open(path+'/codes3/machine_simulation.json', 'r', encoding='utf-8') as f:
                self.bopt_machine_spec_dict = json.load(f, object_pairs_hook=OrderedDict) # https://stackoverflow.com/questions/10844064/items-in-json-object-are-out-of-order-using-json-dumps/23820416
            self.ui.comboBox_FEAConfig.addItems(self.bopt_machine_spec_dict.keys())
            self.ui.comboBox_FEAConfig.activated.connect(self.update_FEAConfig)
            self.update_FEAConfig()
        except Exception as e:
            print(str(e))
            print('[Warn] Skip FEA-based Optimization')
            pass

        '''tab_4: C-based Simulation
        '''
        self.path2acmsimc = self.ui.lineEdit_path2acmsimc.text()
        self.ui.pushButton_runCBasedSimulation.clicked.connect(self.runCBasedSimulation)

        # Read in ACM Plot Settings
        self.filepath_to_ACMPlotLabels  = pkg_resources.resource_filename(__name__, self.ui.lineEdit_path2ACMPlotLabels.text())
        self.filepath_to_ACMPlotSignals = pkg_resources.resource_filename(__name__, self.ui.lineEdit_path2ACMPlotSignals.text()) 
        try:
            with open(self.filepath_to_ACMPlotLabels, 'r') as f:
                self.ui.plainTextEdit_ACMPlotLabels.clear()
                self.ui.plainTextEdit_ACMPlotLabels.appendPlainText(f.read())
            with open(self.filepath_to_ACMPlotSignals, 'r') as f:
                self.ui.plainTextEdit_ACMPlotDetails.clear()
                self.ui.plainTextEdit_ACMPlotDetails.appendPlainText(f.read())
        except Exception as e:
            print(e)
            print('ACMPlot settings are not found. Will use the default instead.')

        '''tab_5: ACMPlot
        '''
        self.ui.pushButton_ACMPlotHere.clicked.connect(self.update_ACMPlot)
        # Matplotlib navigation bar to: self or tabWidget
        self.toolbar = NavigationToolbar(self.ui.MplWidget_ACMPlot.canvas, self)
        self.ui.verticalLayout_CBSMplToolBar.addWidget(self.toolbar) # add to tab 2 only

        self.last_no_samples = None

        '''tab_?: Controller Tuning
        '''
        latex_repo = latexdemo.LaTeX_Repo()
        self.ui.label_qpix_CLKP.setPixmap(latex_repo.qpixmap_CLKP)
        self.ui.label_qpix_CLKI.setPixmap(latex_repo.qpixmap_CLKI)
        self.ui.label_qpix_VLKP.setPixmap(latex_repo.qpixmap_VLKP)
        self.ui.label_qpix_VLKI.setPixmap(latex_repo.qpixmap_VLKI)
        self.ui.label_qpix_Note1.setPixmap(latex_repo.qpixmap_Note1)
        self.ui.label_qpix_Note2.setPixmap(latex_repo.qpixmap_Note2)
        self.ui.pushButton_pidTuner.clicked.connect(self.series_pid_tuner)

        '''menu
        '''
        # Style sheet
        self.ui.actionDark.triggered.connect(lambda: toggle_stylesheet("QDarkStyleSheet.qss")) # need to use "./stylesheet/QDarkStyleSheet.qss" if pkg_resources is not used in toggle_stylesheet.py
        self.ui.actionLight.triggered.connect(lambda: toggle_stylesheet(""))

        '''MainWindow
        '''
        self.setWindowTitle("Figure: Electric Machinery")

        '''
        todo
        '''
        # Recover last input
        # self.lineEdit_npp.textChanged[str].connect(self.doSomething)
        # read in c header file and figure out what are the possible labels


    ''' PI Regulator Tuning '''
    def series_pid_tuner(self):
        # Specify your desired damping factor
        delta = eval(self.ui.lineEdit_dampingFactor_delta.text())
        # Specify your desired speed closed-loop bandwidth
        desired_VLBW_HZ = eval(self.ui.lineEdit_desiredVLBW.text())

        currentPI, speedPI, 上位机电流PI, 上位机速度PI, MagPhaseOmega, BW_in_Hz = tuner.iterate_for_desired_bandwidth(delta, desired_VLBW_HZ, self.motor_dict)
        currentKp, currentKi     = currentPI
        speedKp, speedKi         = speedPI
        上位机电流KP, 上位机电流KI = 上位机电流PI
        上位机速度KP, 上位机速度KI = 上位机速度PI
        CLBW_Hz, VLBW_Hz         = BW_in_Hz

        self.ui.lineEdit_CLBW        .setText(f'{CLBW_Hz:g}')
        self.ui.lineEdit_designedVLBW.setText(f'{VLBW_Hz:g}')
        self.ui.lineEdit_currentKP   .setText(f'{currentKp:g}')
        self.ui.lineEdit_currentKI   .setText(f'{currentKi:g}')
        self.ui.lineEdit_speedKP     .setText(f'{speedKp:g}')
        self.ui.lineEdit_speedKI     .setText(f'{speedKi:g}')
        self.ui.lineEdit_PC_currentKP.setText(f'{上位机电流KP:g}')
        self.ui.lineEdit_PC_currentKI.setText(f'{上位机电流KI:g}')
        self.ui.lineEdit_PC_speedKP  .setText(f'{上位机速度KP:g}')
        self.ui.lineEdit_PC_speedKI  .setText(f'{上位机速度KI:g}')

        # print(f'\n\n\
        #     #define CURRENT_KP {currentKp:g}\n\
        #     #define CURRENT_KI {currentKi:g}\n\
        #     #define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)\n\
        #     #define SPEED_KP {speedKp:g}\n\
        #     #define SPEED_KI {speedKi:g}\n\
        #     #define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)\n')

    ''' C-based Simulation '''
    # read in .dat file for plot
    def get_data_frame(self, path='.'):

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

        if self.last_no_samples == no_samples:
            self.last_no_samples = None
            self.anim.event_source.stop()
            print('Stop ACMPlot animation.\n----------\n\n')

        else:
            self.last_no_samples = no_samples

            print(df_profiles.shape, end='')
            print('read in', path+'/dat/'+data_file_name)            
            # print(df_profiles)

        return df_info, df_profiles, no_samples, no_traces 
    # plot as animation
    def update_ACMPlot(self):
        # if(not self.bool_import_ACMPlot):
        #     sys.path.append(self.path2acmsimc)
        #     import ACMPlot

        def ACMアニメ(i):
            try:
                df_info, df_profiles, no_samples, no_traces = self.get_data_frame(self.path2acmsimc)

                time = np.arange(1, no_samples+1) * df_info['DOWN_SAMPLE'].values[0] * df_info['CL_TS'].values[0]

                self.ui.MplWidget_ACMPlot.canvas.figure.clf()
                trace_counter = 0
                number_of_subplot = len(self.list__number_of_traces_per_subplot)
                # for i, key in enumerate(df_profiles.keys()):
                # for i, key in enumerate(self.list__label):
                first_ax = None
                for i, number_of_traces_per_subplot in enumerate(self.list__number_of_traces_per_subplot):
                    ax = self.ui.MplWidget_ACMPlot.canvas.figure.add_subplot(number_of_subplot*100+11+i, sharex=first_ax)
                    if first_ax is None:
                        first_ax = ax

                    for j in range(number_of_traces_per_subplot):
                        key = self.list__detail[trace_counter]
                        # print('key:', key)
                        try:
                            signal = df_profiles[key]
                        except Exception as e:
                            print('debug:', df_profiles.keys())
                            raise e
                        trace_counter += 1
                        ax.plot(time, signal, '-.', lw=1, label=key)
                    ax.set_ylabel(self.list__label[i])
                    ax.legend(loc='lower center')
                ax.set_xlabel('Time [s]')

                # adjust height per number of traces
                self.ui.MplWidget_ACMPlot.setMinimumSize(QtCore.QSize(500, 200*no_traces))



                # axes = self.ui.MplWidget_ACMPlot.canvas.figure.get_axes()

            except Exception as e:
                # could capture an error when a negative sign is just got printed to the file and the file is read by the script.
                print(str(e))
                pass

        # plot it once (need to sleep for the data to complete)
        # ACMアニメ(0)

        # animate it (it is okay for incomeplete data)
        self.anim = animation.FuncAnimation(self.ui.MplWidget_ACMPlot.canvas.figure, ACMアニメ, interval=100)
        self.ui.MplWidget_ACMPlot.canvas.draw()
    # save setting, compile .c and run .exe
    def runCBasedSimulation(self, bool_=True, bool_savePlotSetting=True, bool_updatePlotSetting=True):
        # why bool_ is always set to False???
        # print(bool_, bool_savePlotSetting, bool_updatePlotSetting)

        def savePlotSettings():
            with open(  self.filepath_to_ACMPlotLabels, 'w') as f:
                f.write(self.ui.plainTextEdit_ACMPlotLabels.toPlainText())
            with open(  self.filepath_to_ACMPlotSignals, 'w') as f:
                f.write(self.ui.plainTextEdit_ACMPlotDetails.toPlainText())

        def decode_labelsAndSignals():
            # #define DATA_LABELS
            labels = [el.strip() for el in self.ui.plainTextEdit_ACMPlotLabels.toPlainText().split('\n') if el.strip()!='']
            # avoid using ',' or ';' in label, because comma will be interpreted as new column by pandas
            # labels = [el.replace(',','|') for el in labels]
            self.list__label = labels
            # print(labels)

            details = self.ui.plainTextEdit_ACMPlotDetails.toPlainText()
            # print(details)

            # 每个通道有几条信号？
            self.list__number_of_traces_per_subplot = []
            for detail in [el.strip() for el in self.ui.plainTextEdit_ACMPlotDetails.toPlainText().split('\n') if el.strip()!='']:
                number_of_traces_per_subplot = len( [el.strip() for el in detail.split(',') if el.strip()!=''] )
                self.list__number_of_traces_per_subplot.append(number_of_traces_per_subplot)

            # #define DATA_DETAILS
            details = [el.strip() for el in re.split('\n|,', details) if el.strip()!='']
            self.list__detail = details
            # print(details)
            return details

        def updatePlotSettings(path2acmsimc, details):
            self = None
            with open(path2acmsimc+'/c/utility.c', 'r', encoding='utf-8') as f:
                new_lines = []
                for line in f.readlines():
                    if   '#define DATA_LABELS '    in line: new_lines.append(rf'#define DATA_LABELS "{  ",".join(details)}\n"'       +'\n')
                    elif '#define DATA_DETAILS '   in line: new_lines.append(rf'#define DATA_DETAILS {  ",".join(details)}'          +'\n')
                    elif '#define DATA_FORMAT '    in line: new_lines.append(rf'#define DATA_FORMAT "{("%g,"*len(details))[:-1]}\n"' +'\n')
                    else: new_lines.append(line)
            with open(path2acmsimc+'/c/utility.c', 'w', encoding='utf-8') as f:
                f.writelines(new_lines)

        def updateACMConfig(path2acmsimc, motor_dict):
            self = None
            NUMBER_OF_STEPS_CL_TS = motor_dict['EndTime']/motor_dict['CL_TS']
            print('NUMBER_OF_STEPS_CL_TS', NUMBER_OF_STEPS_CL_TS)
            with open(path2acmsimc+'/c/ACMConfig.h', 'r', encoding='utf-8') as f:
                new_lines = []
                for line in f.readlines():
                    if   '#define NUMBER_OF_STEPS' in line: new_lines.append(f'#define NUMBER_OF_STEPS {NUMBER_OF_STEPS_CL_TS:.0f}\n')
                    elif '#define CL_TS '          in line: new_lines.append(f'#define CL_TS          ({motor_dict["CL_TS"]:g})\n')
                    elif '#define CL_TS_INVERSE'   in line: new_lines.append(f'#define CL_TS_INVERSE  ({1.0/motor_dict["CL_TS"]:g})\n')
                    elif '#define VL_TS '          in line: new_lines.append(f'#define VL_TS          ({motor_dict["VL_TS"]:g})\n')
                    elif '#define LOAD_INERTIA'    in line: new_lines.append(f'#define LOAD_INERTIA    {motor_dict["JLoadRatio"]}\n')
                    elif '#define LOAD_TORQUE'     in line: new_lines.append(f'#define LOAD_TORQUE     {motor_dict["Tload"]}\n')
                    elif '#define VISCOUS_COEFF'   in line: new_lines.append(f'#define VISCOUS_COEFF   {motor_dict["ViscousCoeff"]}\n')
                    elif '#define DATA_FILE_NAME'  in line: new_lines.append(f'#define DATA_FILE_NAME "{"../dat/"+motor_dict["data_fname_prefix"]+".dat"}"\n')
                    else: new_lines.append(line)
            with open(path2acmsimc+'/c/ACMConfig.h', 'w', encoding='utf-8') as f:
                f.writelines(new_lines)

        # update panel inputs
        self.motor_dict = self.get_motor_dict(self.mj)

        # save 
        if bool_savePlotSetting: savePlotSettings()

        # decode labels and signals for plot
        details = decode_labelsAndSignals()

        # update path/to/acmsimcv5/c/utility.c
        if bool_updatePlotSetting: updatePlotSettings(self.path2acmsimc, self.list__detail)
        updateACMConfig(self.path2acmsimc, self.motor_dict)

        # compile c and run
        if os.path.exists(self.path2acmsimc+"/dat/info.dat"):
            os.remove(self.path2acmsimc+"/dat/info.dat")
        os.system(f"cd /d {self.path2acmsimc}/c && gmake main && start cmd /c main")
        while not os.path.exists(self.path2acmsimc+"/dat/info.dat"):
            # print('sleep for info.dat')
            sleep(0.1)

        # Animate ACMPlot
        # print('sleep for .dat file')
        # sleep(2) # it takes time for main.exe to write data into the disk.
        self.update_ACMPlot()
    def runCBasedSimulation_SweepFrequnecyAnalysis(self):
        for delta in [6.5]:
        # for delta in [15]:

            # Specify your desired speed closed-loop bandwidth
            # desired_BW_velocity_HZ = 223
            desired_BW_velocity_HZ = 100

            currentPI, speedPI, 上位机电流PI, 上位机速度PI, MagPhaseOmega = tuner.iterate_for_desired_bandwidth(delta, desired_BW_velocity_HZ)
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

            max_freq = 2*desired_BW_velocity_HZ
            init_freq = 2

            # change to false to save time
            if True:
                ad = simulator.acm_designer(work_dir=self.path2acmsimc)
                ad.update_ACMConfig_evaluate_PI_coefficients(currentPI, speedPI, 上位机电流PI, 上位机速度PI, 
                                                             max_freq=max_freq, init_freq=init_freq,
                                                             motor_dict=motor_dict)

                self.ui.plainTextEdit_ACMPlotLabels.appendPlainText('\nSweepSpeed [rpm]')
                self.ui.plainTextEdit_ACMPlotDetails.appendPlainText('rpm_speed_command,sm.omg')
                self.runCBasedSimulation(self, bool_savePlotSetting=False, bool_updatePlotSetting=True)

                # plt.figure()
                data_file_name = ad.plot_PI_coefficients()
                print('------------end of simulator\n\n\n')
                # plt.show()

            # # 1. Ploe simualted Bode plot
            # dB, Deg, Freq, max_freq = analyzer.analyze(ad.data_fname, max_freq, ad)
            # plt.figure(4, figsize=(20,8))
            # plt.plot(Freq, dB, '--.', label=ad.data_fname)

            # index, value = analyzer.find_nearest(dB, -3) # find -3dB freq
            # VLBW = Freq[index]
            # plt.text(VLBW, -5, f'{VLBW:.0f} Hz', color='red', fontsize=20)
            # plt.plot([0,max_freq], [-3, -3], 'k--')
            # plt.ylabel('Velocity Closed-loop transfer function amplitude [dB]')

            # plt.xscale('log')
            # plt.xlabel('Frequency [Hz]')
            # plt.legend()


            # # 2. Plot designed Bode plot
            # # plt.figure(4, figsize=(20,8))
            # mag, phase, omega = MagPhaseOmega
            # index_max_freq = sum(omega/(2*np.pi) < max_freq)
            # plt.plot((omega/(2*np.pi))[:index_max_freq], 20*np.log10(mag[:index_max_freq]), '-.', label=f'designed:$\\delta={delta}$')

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

            # plt.legend()
            # break        

    ''' Optimization Section '''
    # choose machine specification
    def update_machineSpec(self):
        pass
    def update_FEAConfig(self):
        pass

    ''' General Information '''
    def comboActivate_namePlateData(self):
        motor_dict = self.get_motor_dict(self.mj)
        self.ui.lineEdit_npp         .setText(str(motor_dict['n_pp']))
        self.ui.lineEdit_RatedCurrent.setText(str(motor_dict['IN']))
        self.ui.lineEdit_RatedPower  .setText(str(motor_dict['PW']))
        self.ui.lineEdit_RatedSpeed  .setText(str(motor_dict['RPM']))

        # 另一个tab下的LineEdit也同步更新
        self.ui.lineEdit_RO_MachineName.setText(self.ui.comboBox_MachineName.currentText())
    # parameter conversion
    def update_emy(self):
        self.emy = ElectricMachinery( NUMBER_OF_POLE_PAIRS  = int  (self.ui.lineEdit_npp.text()),
                                      RATED_CURRENT_RMS     = float(self.ui.lineEdit_RatedCurrent.text()),
                                      RATED_POWER_WATT      = float(self.ui.lineEdit_RatedPower.text()),
                                      RATED_SPEED_RPM       = float(self.ui.lineEdit_RatedSpeed.text()),
            )
        self.ui.ConsoleWidget.push_vars({
            'emy': self.emy
            })
        self.ui.label_pushedVariables.setText('emy')
    # decide which motor is used
    def get_motor_dict(self, mj):
        # read from json file
        motor = mj[self.ui.comboBox_MachineName.currentText()]["基本参数"]

        n_pp = motor["极对数 [1]"]
        R    = motor["电机线电阻 [Ohm]"]/2
        Ld   = motor["电机D轴线电感 [mH]"]/2*1e-3
        Lq   = motor["电机Q轴线电感 [mH]"]/2*1e-3
        KE   = motor["转矩常数 [Nm/Arms]"] / 1.5 / n_pp * 1.414
        J_s  = motor["转动惯量 [kg.cm^2]"]*1e-4
        IN   = motor["额定电流 [Arms]"]
        PW   = motor["额定功率 [Watt]"]
        RPM  = motor["额定转速 [rpm]"]

        motor_dict = dict()
        motor_dict['n_pp'] = n_pp
        motor_dict['Rs'] = R
        motor_dict['Ld'] = Ld
        motor_dict['Lq'] = Lq
        motor_dict['KE'] = KE
        motor_dict['J_s'] = J_s
        motor_dict['IN'] = IN
        motor_dict['PW'] = PW
        motor_dict['RPM'] = RPM

        # below is by user GUI input
        CL_TS = eval(self.ui.lineEdit_CLTS.text())
        VL_TS = eval(self.ui.lineEdit_CLTS.text())
        EndTime = eval(self.ui.lineEdit_EndTime.text())
        motor_dict['CL_TS'] = CL_TS
        motor_dict['VL_TS'] = VL_TS
        motor_dict['EndTime'] = EndTime

        motor_dict['JLoadRatio']   = eval(self.ui.lineEdit_LoadInertiaPercentage.text()) #[%]
        motor_dict['Tload']        = eval(self.ui.lineEdit_LoadTorque.text()) #[Nm]
        motor_dict['ViscousCoeff'] = eval(self.ui.lineEdit_ViscousCoefficient.text()) #[Nm/(rad/s)]

        motor_dict['data_fname_prefix'] = self.ui.lineEdit_OutputDataFileName.text()

        # this will be passed to C based simulation
        return motor_dict

    ''' plot demo '''
    def update_graph(self):
        if self.ui.MplWidget.canvas.axes is None:
           self.ui.MplWidget.canvas.axes = self.ui.MplWidget.canvas.figure.add_subplot(111)

        fs = 500
        f = random.randint(1, 100)
        ts = 1/fs
        length_of_signal = 100
        t = np.linspace(0,1,length_of_signal)
        
        cosinus_signal = np.cos(2*np.pi*f*t)
        sinus_signal = np.sin(2*np.pi*f*t)

        self.ui.MplWidget.canvas.axes.clear()
        self.ui.MplWidget.canvas.axes.plot(t, cosinus_signal)
        self.ui.MplWidget.canvas.axes.plot(t, sinus_signal)
        self.ui.MplWidget.canvas.axes.legend(('cosinus', 'sinus'),loc='upper right')
        self.ui.MplWidget.canvas.axes.set_title('Cosinus - Sinus Signal')
        self.ui.MplWidget.canvas.draw()

def main():
    app = QApplication([])
    window = EmachineryWidget()
    window.show()
    app.exec_()

if __name__ == '__main__':
    print(os.path.dirname(os.path.realpath(__file__)))
    main()
