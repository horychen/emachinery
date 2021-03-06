# https://www.youtube.com/watch?v=2C5VnE9wPhk
# background-color: rgb(255, 255, 255)

import pkg_resources # to include resource file: mainWindow.ui # https://stackoverflow.com/questions/6028000/how-to-read-a-static-file-from-inside-a-python-package/20885799

from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
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
import copy

from emachinery.utils.conversion import ElectricMachinery
from emachinery.jsons import ACMInfo
from emachinery.gui.stylesheet.toggle_stylesheet import toggle_stylesheet

# Promoted Widgets: use these two absolute imports to replace the relative imports in mainWindow.py for uploading to PyPI
# from emachinery.gui.consolewidget import ConsoleWidget
# from emachinery.gui.mplwidget import MplWidget

from emachinery.gui.mainWindow_v2 import Ui_MainWindow

from emachinery.gui import latexdemo

# from emachinery.newcode import newcode

Help = r''' 
If you want to import a new module (say tuner) in a new directory (e.g., acmdesignv2), you need to follow these steps:

1. Open __init__.py in emachinery folder, and add path to new directory to sys.
2. Create an __init__.py file inside your new directory.
3. Delete emachinery.egg-info/ and other temporary folders (build, dist) if there are.
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
You are going to see the following error message when running:
    ---------------------------
    Traceback (most recent call last):
      File "D:\DrH\Codes\emachineryTestPYPI\emachinery\gui\main.py", line 28, in <module>
        from emachinery.acmdesignv2 import tuner
    ModuleNotFoundError: No module named 'emachinery.acmdesignv2'

5. After you pip install from PyPI, if you see error message similar to:
    'EmachineryWidget' object has no attribute 'lineEdit_path2boptPython'
this is likely that you should call self.ui.lineEdit_path2boptPython instead of self.lineEdit_path2boptPython.
This is not a problem if you use load .ui, but it is a problem if you import .py obtained via pyuic5.

6. 还有一个不方便的地方就是，如果正式安装了emachinery，就没法在本地测试了，因为会优先import库emachinery中的.py文件作为模块。
换句话说，只能用“pip install -e .”进行本地测试。
'''
KnownIssues=r'''
- If a function is connected, the first kwarg is always set to True or False by PyQt5. Search for bool_always_[\D]*_bug for example.
'''
from emachinery.acmdesignv2 import tuner
# from emachinery.acmdesignv2 import simulator
from emachinery.acmdesignv2 import analyzer


class EmachineryWidget(QMainWindow):
    def __init__(self, path2acmsimc):
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

        '''tab 1: Name Plate Data
        '''
        self.mj = ACMInfo.MotorJson().d

        self.ui.comboBox_MachineName.activated.connect(self.comboActivate_namePlateData)
        self.ui.comboBox_MachineType.activated.connect(self.comboActivate_machineType)
        # self.comboActivate_machineType() # should be called after when configini is initialized

        # Move this to later, because we need to update lineEdit_OutputDataFileName from recovery first
        # self.motor_dict = self.get_motor_dict(self.mj);

        # Select Excitation
        self.list__radioButton4Excitation = [
            self.ui.radioButton_Excitation_Position,
            self.ui.radioButton_Excitation_Velocity,
            self.ui.radioButton_Excitation_SweepFrequency]
        self.list__checkedExcitation = [radioButton.isChecked() for radioButton in self.list__radioButton4Excitation]
        for radioButton in self.list__radioButton4Excitation:
            radioButton.toggled.connect(self.radioChecked_ACMExcitation)

        self.control_dict   = dict(); self.control_dict = self.get_control_dict()
        self.sweepFreq_dict = dict(); self.sweepFreq_dict = self.get_sweepFreq_dict()



        '''tab_3: C-based Simulation
        '''
        self.ui.pushButton_runCBasedSimulation.clicked.connect(self.runCBasedSimulation)

        # init comboBox_plotSettings
        self.ui.comboBox_plotSettings.activated.connect(self.update_ACMPlotLabels_and_ACMPlotSignals)
        # this will invoke: self.update_ACMPlotLabels_and_ACMPlotSignals(bool_re_init=True) for ya
        self.comboActivate_machineType() 

        # Motor Dict ( The order matters, should be after self.comboActivate_machineType() )
        print(self.ui.lineEdit_OutputDataFileName.text())
        self.motor_dict = self.get_motor_dict(self.mj);


        self.ui.lineEdit_path2acmsimc.setText(self.path2acmsimc)
        self.ui.lineEdit_path2acmsimc.textChanged[str].connect(self.save_path2acmsimc)


        # settings for sweep frequency
        self.ui.radioButton_openLoop.toggled.connect(self.radioChecked_Settings4SweepFrequency)
        self.ui.radioButton_currentLoopOnly.toggled.connect(self.radioChecked_Settings4SweepFrequency)





        '''tab_3: ACMPlot
        '''
        self.ui.pushButton_ACMPlotHere.clicked.connect(self.clicked__pushButton_ACMPlotHere)
        # Matplotlib navigation bar to: self or tabWidget
        self.toolbar = NavigationToolbar(self.ui.MplWidget_ACMPlot.canvas, self)
        self.ui.verticalLayout_CBSMplToolBar.addWidget(self.toolbar) # add to tab 2 only

        # this is used to judge when to stop animation
        self.last_no_samples = None




        '''tab_2: Controller Tuning
        '''
        latex_repo = latexdemo.LaTeX_Repo()
        self.ui.label_qpix_CLKP.setPixmap(latex_repo.qpixmap_CLKP)
        self.ui.label_qpix_CLKI.setPixmap(latex_repo.qpixmap_CLKI)
        self.ui.label_qpix_VLKP.setPixmap(latex_repo.qpixmap_VLKP)
        self.ui.label_qpix_VLKI.setPixmap(latex_repo.qpixmap_VLKI)
        self.ui.label_qpix_Note1.setPixmap(latex_repo.qpixmap_Note1)
        self.ui.label_qpix_Note2.setPixmap(latex_repo.qpixmap_Note2)
        self.ui.label_qpix_Note3.setPixmap(latex_repo.qpixmap_Note3)
        self.ui.pushButton_pidTuner.clicked.connect(self.run_series_pid_tuner)
        self.run_series_pid_tuner()
        # self.ui.pushButton_SweepFreq.clicked.connect(self.runCBasedSimulation_SweepFrequnecyAnalysis)





        '''tab_4: Bode Plot
        '''
        self.ui.pushButton_bodePlot.clicked.connect(self.update_BodePlot)


        '''tab_5: Plots
        '''
        # update plot
        self.ui.pushButton_getSignal.clicked.connect(self.update_graph)
        # undate model
        self.ui.pushButton_updateModel.clicked.connect(self.update_emy)

        # Matplotlib navigation bar to: self or tabWidget
        self.toolbar = NavigationToolbar(self.ui.MplWidget.canvas, self)
            # self.addToolBar(self.toolbar) # add to mainWindow
        self.ui.verticalLayout_inTab2.addWidget(self.toolbar) # add to tab 2 only



        '''tab_6: FEA-based Optimization
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

        '''tab_7: Parametric Analysis
        '''
        self.ui.pushButton_runCBasedSimulation4PA.clicked.connect(self.runCBasedSimulation4PA)
        self.ui.plainTextEdit_PANames.clear()
        self.ui.plainTextEdit_PANames.appendPlainText(self.configini['PANames'])
        self.ui.plainTextEdit_PAValues.clear()
        self.ui.plainTextEdit_PAValues.appendPlainText(self.configini['PAValues'])

        self.ui.pushButton_plot4PA.clicked.connect(self.plot4PA)

        data_file_name_list = self.getDataFileNameList4PA()
        self.ui.comboBox_PADataFileSelected.clear()
        self.ui.comboBox_PADataFileSelected.addItems(data_file_name_list)
        self.ui.comboBox_PADataFileSelected.activated.connect(self.update_PASelected)


        '''menu
        '''
        # Style sheet
        self.ui.actionDark.triggered.connect(lambda: toggle_stylesheet("QDarkStyleSheet.qss")) # need to use "./stylesheet/QDarkStyleSheet.qss" if pkg_resources is not used in toggle_stylesheet.py
        self.ui.actionLight.triggered.connect(lambda: toggle_stylesheet(""))

        '''MainWindow
        '''
        self.setWindowTitle("Electric Machinery")
        # self.setWindowTitle("Figure: Electric Machinery")

        '''
        todo
        '''
        # read in c header file and figure out what are the possible labels

    ''' PI Regulator Tuning '''
    def run_series_pid_tuner(self):
        # Specify your desired damping factor
        delta = eval(self.ui.lineEdit_dampingFactor_delta.text())
        # Specify your desired speed closed-loop bandwidth
        desired_VLBW_HZ = eval(self.ui.lineEdit_desiredVLBW.text())

        # Update CL_TS and CL_VS
        self.motor_dict['CL_TS'] = eval(self.ui.lineEdit_CLTS.text())
        self.motor_dict['VL_TS'] = eval(self.ui.lineEdit_VLTS.text())

        currentPI, speedPI, 上位机电流PI, 上位机速度PI, tuple_designedMagPhaseOmega, BW_in_Hz = tuner.iterate_for_desired_bandwidth(delta, desired_VLBW_HZ, self.motor_dict)

        currentKp, currentKi     = currentPI
        speedKp, speedKi         = speedPI
        上位机电流KP, 上位机电流KI = 上位机电流PI
        上位机速度KP, 上位机速度KI = 上位机速度PI
        self.C2C_designedMagPhaseOmega, self.C2V_designedMagPhaseOmega, self.V2V_designedMagPhaseOmega = tuple_designedMagPhaseOmega
        CLBW_Hz, VLBW_Hz, CutOff_Hz = BW_in_Hz

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
    def get_control_dict(self):

        self.control_dict["currentPI"] = (eval(self.ui.lineEdit_currentKP.text()), eval(self.ui.lineEdit_currentKI.text()))
        self.control_dict["speedPI"]   = (eval(self.ui.lineEdit_speedKP.text()),   eval(self.ui.lineEdit_speedKI.text()))
        # print(f'\n\n\
        #     #define CURRENT_KP {currentKp:g}\n\
        #     #define CURRENT_KI {currentKi:g}\n\
        #     #define CURRENT_KI_CODE (CURRENT_KI*CURRENT_KP*CL_TS)\n\
        #     #define SPEED_KP {speedKp:g}\n\
        #     #define SPEED_KI {speedKi:g}\n\
        #     #define SPEED_KI_CODE (SPEED_KI*SPEED_KP*VL_TS)\n')
        
        self.list__checkedExcitation = [radioButton.isChecked() for radioButton in self.list__radioButton4Excitation]
        self.control_dict['ExcitationType'] = np.argmax(self.list__checkedExcitation)

        # pass to console
        self.console_push_variable({'control_dict':self.control_dict})

        self.console_push_variable({'tuner_trick':
            ''' (1) A value of damping factor (δ) equal to 1.0 corresponds to the condition where the velocity open-loop gain intercepts 0 dB right at the frequency of the current controller bandwidth. This results in pole/zero cancellation at this frequency with a phase margin of zero. It goes without saying that zero phase margin equals bad things for your system. \n
                (2) Where "δ" we will define as the damping factor. The larger δ is, the further apart the zero corner frequency and the current loop pole will be. And the further apart they are, the phase margin is allowed to peak to a higher value in-between these frequencies. This improves stability at the expense of speed loop bandwidth. If δ = 1, then the zero corner frequency and the current loop pole are equal, which results in pole/zero cancellation and the system will be unstable. Theoretically, any value of δ > 1 is stable since phase margin > 0. However, values of δ close to 1 result in severely underdamped performance.\n
                (3) In short, δ > 1 gives positive phase margin and the larger δ gets, the larger the phase margin becomes, and at the same time, the velocity performance would become sluggish (i.e., you will need to increase your current bandwidth to really high to achieve higher velocity bandwidth).\n
            '''})

        # print(self.control_dict, end='\n'*2)
        return self.control_dict
    def get_sweepFreq_dict(self):

        self.sweepFreq_dict["max_freq"] = eval(self.ui.lineEdit_maxFreq2Sweep.text())
        self.sweepFreq_dict["init_freq"] = 2
        self.sweepFreq_dict["SWEEP_FREQ_C2V"] = self.ui.radioButton_openLoop.isChecked() # speed open loop
        self.sweepFreq_dict["SWEEP_FREQ_C2C"] = self.ui.radioButton_currentLoopOnly.isChecked()

        # pass to console
        self.console_push_variable({'sweepFreq_dict':self.sweepFreq_dict})

        # print(self.sweepFreq_dict, end='\n'*2)
        return self.sweepFreq_dict
    def update_BodePlot(self):
        try:
            self.data_file_name
        except AttributeError as e:
            print('Warning: run C based simulation first with Sweep Frequency excitation first.')
            return 
        dot_dat_file_dir = self.path2acmsimc+'/dat/'+self.data_file_name
        # print(self.motor_dict)
        # print(self.sweepFreq_dict)
        ret = analyzer.analyze(dot_dat_file_dir, self.motor_dict, self.sweepFreq_dict)
        if ret is None:
            msg = QMessageBox()
            msg.setWindowTitle("Error")
            msg.setText('Output file: '+dot_dat_file_dir+' does not exist.')
            msg.setIcon(QMessageBox.Critical)
            x = msg.exec_()
        else:
            # 1. Plot simualted Bode plot
            dB, Deg, Freq, max_freq = ret
            # for el in ret:
            #     print(el)

            self.ui.MplWidget_bodePlot.canvas.figure.clf()
            ax = self.ui.MplWidget_bodePlot.canvas.figure.add_subplot(111)
            ax.plot(Freq, dB, '--.', label=self.data_file_name)

            if self.sweepFreq_dict["SWEEP_FREQ_C2V"]:
                # Open-Loop

                # C2V
                ax.set_ylabel('Velocity / Current open-loop-tf amplitude [dB] (elec.rad/s/Apk)')
            else:
                # Closed-Loop

                # Bandwidth@-3dB
                index, value = analyzer.find_nearest(dB, -3) # find -3dB freq
                VLBW_Hz = Freq[index]
                ax.text(VLBW_Hz, -5, f'{VLBW_Hz:.0f} Hz', color='red', fontsize=20)
                ax.plot([0,max_freq], [-3, -3], 'k--')

                if self.sweepFreq_dict["SWEEP_FREQ_C2C"]:
                    # C2C
                    ax.set_ylabel('Current closed-loop-tf amplitude [dB]')
                else:
                    # V2V
                    ax.set_ylabel('Velocity closed-loop-tf amplitude [dB]')
            ax.set_xscale('log')
            ax.set_xlabel('Frequency [Hz]')

            # 2. Plot designed Bode plot
            if self.sweepFreq_dict["SWEEP_FREQ_C2V"]:
                # C2V
                mag, phase, omega = self.C2V_designedMagPhaseOmega
            elif self.sweepFreq_dict["SWEEP_FREQ_C2C"]:
                # C2C
                mag, phase, omega = self.C2C_designedMagPhaseOmega
            else:
                # V2V
                mag, phase, omega = self.V2V_designedMagPhaseOmega
            index_max_freq = sum(omega/(2*np.pi) < max_freq)
            ax.plot((omega/(2*np.pi))[:index_max_freq], 20*np.log10(mag[:index_max_freq]), '-.', label=f'designed:$\\delta={eval(self.ui.lineEdit_dampingFactor_delta.text())}$')

            # # 3. Plot measured Bode plot
            # fname = r'D:\ISMC\SweepFreq\Jupyter\VLBW_Hz-Data/' + 'BiasSine500rpm' + data_file_name[data_file_name.find(data_file_name_prefix)+len(data_file_name_prefix)+len('-CLOSED-@'):-4]+'.txt'
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
            #     VLBW_Hz = list_qep_max_frequency[index]
            #     plt.text(VLBW_Hz, -5, f'{VLBW_Hz:.0f} Hz', color='purple', fontsize=20)
            #     plt.plot(list_qep_max_frequency, CL_VL_TF, '--.', label=fname)

            ax.legend()

        # analyzer.folder(self.data_file_name, self.motor_dict, data_file_name_prefix=self.motor_dict['data_file_name_prefix'])

        # ax.plot(time, signal, '-.', lw=1, label=key)
        # ax.set_ylabel(self.list__label[i])
        # ax.legend(loc='lower center')

        # ax.set_xlabel('Time [s]')
        # ax.set_ylabel('Time [s]')

        # adjust height per number of traces
        # self.ui.MplWidget_bodePlot.setMinimumSize(QtCore.QSize(500, 200*no_traces))

        self.ui.MplWidget_bodePlot.canvas.draw()

    def get_mtype(self):

        if 'Induction Machine' in self.ui.comboBox_MachineType.currentText():
            mtype = 'im'
        elif 'Synchronous Machine' in self.ui.comboBox_MachineType.currentText():
            mtype = 'pmsm'
        else:
            raise 'Unknown machine type'
        return mtype

    ''' C-based Simulation '''
    def update_ACMPlotLabels_and_ACMPlotSignals(self, configini, bool_always_true_bug=True, bool_re_init=False):

        # selected = self.ui.comboBox_plotSettings.currentText()
        # print('selected:', selected)

        # 初始化（电机类型切换的时候也得运行）
        if bool_re_init:

            mtype = self.get_mtype()

            # 产生下拉菜单
            self.ui.comboBox_plotSettings.clear()
            self.ui.comboBox_plotSettings.addItems([f'{mtype}_default'] + [f'{mtype}_user{i:02d}' for i in range(10)])

            # 为了重新启动以后能记住上一次所选择的PlotSetting
            for i in range(self.ui.comboBox_plotSettings.count()):
                if configini[f'{mtype}-PlotSettings'] in self.ui.comboBox_plotSettings.itemText(i):
                    self.ui.comboBox_plotSettings.setCurrentIndex(i)
                    break

        # Read in ACM Plot Settings
        # 根据所选择的配置更新ACMPlot绘图信号列表
        selected = self.ui.comboBox_plotSettings.currentText()
        # print('selected:', selected)
        self.filepath_to_ACMPlotLabels  = pkg_resources.resource_filename(__name__, f'./plot_setting_files/labels_{selected}.txt')
        self.filepath_to_ACMPlotSignals = pkg_resources.resource_filename(__name__, f'./plot_setting_files/signals_{selected}.txt')
        self.ui.lineEdit_path2ACMPlotLabels.setText(f'./plot_setting_files/labels_{selected}.txt')
        self.ui.lineEdit_path2ACMPlotSignals.setText(f'./plot_setting_files/signals_{selected}.txt')
        try:
            with open(self.filepath_to_ACMPlotLabels, 'r') as f:
                self.ui.plainTextEdit_ACMPlotLabels.clear()
                self.ui.plainTextEdit_ACMPlotLabels.appendPlainText(f.read())
            with open(self.filepath_to_ACMPlotSignals, 'r') as f:
                self.ui.plainTextEdit_ACMPlotDetails.clear()
                self.ui.plainTextEdit_ACMPlotDetails.appendPlainText(f.read())
        except FileNotFoundError as e:
            with open(self.filepath_to_ACMPlotLabels, 'w') as f:
                pass
            with open(self.filepath_to_ACMPlotLabels, 'r') as f:
                self.ui.plainTextEdit_ACMPlotLabels.clear()
                self.ui.plainTextEdit_ACMPlotLabels.appendPlainText(f.read())
            with open(self.filepath_to_ACMPlotSignals, 'w') as f:
                pass
            with open(self.filepath_to_ACMPlotSignals, 'r') as f:
                self.ui.plainTextEdit_ACMPlotDetails.clear()
                self.ui.plainTextEdit_ACMPlotDetails.appendPlainText(f.read())
        except Exception as e:
            raise e
            # print('ACMPlot settings are not found. Will use the default instead.')

    def save_Config4CBasedSimulation(self):
        # 读取需要记住的用户输入
        self.configini['EndTime'] = self.ui.lineEdit_EndTime.text()
        self.configini['CLTS'] = self.ui.lineEdit_CLTS.text()
        self.configini['VLTS'] = self.ui.lineEdit_VLTS.text()
        self.configini['LoadTorque'] = self.ui.lineEdit_LoadTorque.text()
        self.configini['LoadInertiaPercentage'] = self.ui.lineEdit_LoadInertiaPercentage.text()
        self.configini['ViscousCoefficient'] = self.ui.lineEdit_ViscousCoefficient.text()
        self.configini['OutputDataFileName'] = self.ui.lineEdit_OutputDataFileName.text()
        mtype = self.get_mtype()
        self.configini[f'{mtype}-PlotSettings'] = self.ui.comboBox_plotSettings.currentText()[-2:]
        self.configini['PANames'] = self.ui.plainTextEdit_PANames.toPlainText()
        self.configini['PAValues'] = self.ui.plainTextEdit_PAValues.toPlainText()
        with open(self.filepath_to_configini, 'w') as f:
            json.dump(self.configini, f, ensure_ascii=False, indent=4)
    def save_path2acmsimc(self):
        self.path2acmsimc = self.ui.lineEdit_path2acmsimc.text()
        self.configini['path2acmsimc'] = self.path2acmsimc
        with open(self.filepath_to_configini, 'w') as f:
            json.dump(self.configini, f, ensure_ascii=False, indent=4)
    def get_data_file_name(self):

        上位机电流KP = eval(self.ui.lineEdit_PC_currentKP.text())
        上位机电流KI = eval(self.ui.lineEdit_PC_currentKI.text())
        上位机速度KP = eval(self.ui.lineEdit_PC_speedKP  .text())
        上位机速度KI = eval(self.ui.lineEdit_PC_speedKI  .text())

        if not os.path.exists(self.path2acmsimc+'/dat'):
            os.makedirs(self.path2acmsimc+'/dat')

        # get ExcitationType
        self.list__checkedExcitation = [radioButton.isChecked() for radioButton in self.list__radioButton4Excitation]
        self.control_dict['ExcitationType'] = np.argmax(self.list__checkedExcitation)

        if self.control_dict['ExcitationType'] == 2:
            if not os.path.exists(self.path2acmsimc+'/dat/Closed'):
                os.makedirs(self.path2acmsimc+'/dat/Closed')
            if not os.path.exists(self.path2acmsimc+'/dat/Open'):
                os.makedirs(self.path2acmsimc+'/dat/Open')
            if self.sweepFreq_dict["SWEEP_FREQ_C2V"]:
                self.data_file_name = f"../dat/Open/{  self.motor_dict['data_file_name_prefix']}-C2V-@{self.sweepFreq_dict['max_freq']:.0f}Hz-{上位机电流KP:.0f}-{上位机电流KI:.0f}-{上位机速度KP:.0f}-{上位机速度KI:.0f}.dat"
            else:
                if self.sweepFreq_dict["SWEEP_FREQ_C2C"]:
                    self.data_file_name = f"../dat/Closed/{self.motor_dict['data_file_name_prefix']}-C2C-@{self.sweepFreq_dict['max_freq']:.0f}Hz-{上位机电流KP:.0f}-{上位机电流KI:.0f}-{上位机速度KP:.0f}-{上位机速度KI:.0f}.dat"
                else:
                    self.data_file_name = f"../dat/Closed/{self.motor_dict['data_file_name_prefix']}-V2V-@{self.sweepFreq_dict['max_freq']:.0f}Hz-{上位机电流KP:.0f}-{上位机电流KI:.0f}-{上位机速度KP:.0f}-{上位机速度KI:.0f}.dat"

        else:
            # file name with PI coefficients
            self.data_file_name = f"../dat/{self.motor_dict['data_file_name_prefix']}-{上位机电流KP:.0f}-{上位机电流KI:.0f}-{上位机速度KP:.0f}-{上位机速度KI:.0f}.dat"
        # print('self.data_file_name:', self.data_file_name, end='\n'*2)
        return self.data_file_name
    # read in .dat file for plot
    def get_dataFrame(self, data_file_name):

        # info.dat
        # df_info = pd.read_csv(path+"/dat/info.dat", na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
        # data_file_name = df_info['DATA_FILE_NAME'].values[0].strip()

        # 这个大的数据文件产生有滞后，可能文件还没生成，程序就已经跑到这里开始调用read_csv了！
        # 所以要判断一下，数据文件应该是在info.dat之后生成的。
        # while True:
        #     print('st_mtime 1:', os.stat(path+'/dat/'+data_file_name).st_mtime )
        #     print('st_mtime 2:', os.stat(path+"/dat/info.dat").st_mtime)
        #     if os.stat(path+'/dat/'+data_file_name).st_mtime < os.stat(path+"/dat/info.dat").st_mtime:
        #         print('Sleep in ACMPlot.py')
        #         sleep(0.1)
        #         break    

        # moved outside of this function
        # data_file_name = self.get_data_file_name()

        # ???.dat
        # data_file_name = path+'/dat/'+data_file_name
        df_profiles = pd.read_csv(self.path2acmsimc+'/dat/'+data_file_name, na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
        no_samples = df_profiles.shape[0]
        no_traces  = df_profiles.shape[1]
        # print(data_file_name)

        return df_profiles, no_samples, no_traces 
    # plot as animation
    def clicked__pushButton_ACMPlotHere(self):
        self.update_ACMPlot(data_file_name=self.ui.comboBox_PADataFileSelected.currentText())
    def update_ACMPlot(self, bool_always_false_bug=None, data_file_name=None):
        # if(not self.bool_import_ACMPlot):
        #     sys.path.append(self.path2acmsimc)
        #     import ACMPlot

        if data_file_name is None:
            data_file_name = self.data_file_name

        # 需要动画的原因是画图的程序和main.exe是同时运行的，数据还没完整就开始画了，所以要动画。
        def ACMアニメ(i):
            df_profiles, no_samples, no_traces = self.get_dataFrame(data_file_name)

            if self.last_no_samples == no_samples:
                self.last_no_samples = None
                self.anim.event_source.stop()
                print('Stop ACMPlot animation.\n----------\n\n')

            else:
                self.last_no_samples = no_samples

                print(df_profiles.shape, end=' | ')
                print('read in', self.path2acmsimc+'/dat/'+data_file_name)
                # print(df_profiles)


            time = np.arange(1, no_samples+1) * self.motor_dict['DOWN_SAMPLE'] * self.motor_dict['CL_TS']

            self.ui.MplWidget_ACMPlot.canvas.figure.clf()
            trace_counter = 0

            # for i, key in enumerate(df_profiles.keys()):
            # for i, key in enumerate(self.list__label):
            first_ax = None
            for i, number_of_traces_per_subplot in enumerate(list__number_of_traces_per_subplot):
                ax = self.ui.MplWidget_ACMPlot.canvas.figure.add_subplot(number_of_subplot, 1, 1+i, sharex=first_ax)
                if first_ax is None:
                    first_ax = ax

                for j in range(number_of_traces_per_subplot):
                    key = details[trace_counter]
                    # print('key:', key)
                    try:
                        signal = df_profiles[key]
                    except Exception as e:
                        print('debug:', df_profiles.keys())
                        raise e
                    trace_counter += 1
                    try:
                        ax.plot(time, signal, '-.', lw=1.5, label=key, alpha=0.5)
                    except ValueError as e:
                        print('ValueError during ax.plot():', e, '\nThis is known issue and will ignore and continue.') # ValueError: could not convert string to float: '3.33723e-'
                        pass
                ax.set_ylabel(list__label[i])
                ax.legend(loc='lower center')
            ax.set_xlabel('Time [s]')

            # axes = self.ui.MplWidget_ACMPlot.canvas.figure.get_axes()

        # before animation, some global information is set here first
        details, list__number_of_traces_per_subplot, list__label = self.decode_labelsAndSignals()
        number_of_subplot = len(list__number_of_traces_per_subplot)

        # plot it once (need to sleep for the data to complete)
        # ACMアニメ(0)

        # animate it (it is okay for incomeplete data)
        self.anim = animation.FuncAnimation(self.ui.MplWidget_ACMPlot.canvas.figure, ACMアニメ, interval=500)

        # adjust height per number of traces
        df_profiles, no_samples, no_traces = self.get_dataFrame(data_file_name)
        self.ui.MplWidget_ACMPlot.setMinimumSize(QtCore.QSize(500, 200*no_traces))
        # draw
        self.ui.MplWidget_ACMPlot.canvas.draw()
    def radioChecked_ACMExcitation(self):
        # This is not needed as there is an option for radioButton as autoExclusive
        # temp = [radioButton.isChecked() for radioButton in self.list__radioButton4Excitation]
        # if sum(temp)>=2:
        #     index = 0
        #     for new, old in zip(temp, self.list__checkedExcitation):
        #         # 原来已经被勾上的
        #         if new == old == True:
        #             self.list__radioButton4Excitation[index].setChecked()
        #         # 刚刚被勾上的
        #         if new != old and new == True:
        #             self.control_dict['ExcitationType'] = index
        #         index += 1

        self.list__checkedExcitation = [radioButton.isChecked() for radioButton in self.list__radioButton4Excitation]
        self.control_dict['ExcitationType'] = np.argmax(self.list__checkedExcitation)

        # sweep frequency
        if self.control_dict['ExcitationType'] == 2:
            self.ui.groupBox_sweepFrequency.setEnabled(True)
        else:
            self.ui.groupBox_sweepFrequency.setEnabled(False)
            self.ui.radioButton_openLoop.setChecked(False)
            self.ui.radioButton_currentLoopOnly.setChecked(False)
    def radioChecked_Settings4SweepFrequency(self):
        # This is not needed as there is an option for radioButton as autoExclusive
        if self.ui.radioButton_openLoop.isChecked() and self.ui.radioButton_currentLoopOnly.isChecked():
            msg = QMessageBox()
            msg.setWindowTitle("Warning")
            msg.setText("Not support open-loop and current-loop-only at the same time!\nPlease select only one of them.")
            msg.setIcon(QMessageBox.Warning)
            x = msg.exec_()
            self.ui.radioButton_openLoop.setChecked(False)
            self.ui.radioButton_currentLoopOnly.setChecked(False)
    def decode_labelsAndSignals(self):
        # #define DATA_LABELS
        labels = [el.strip() for el in self.ui.plainTextEdit_ACMPlotLabels.toPlainText().split('\n') if el.strip()!='']
        # avoid using ',' or ';' in label, because comma will be interpreted as new column by pandas
        # labels = [el.replace(',','|') for el in labels]
        # self.list__label = labels
        # print(labels)

        details = self.ui.plainTextEdit_ACMPlotDetails.toPlainText()
        # print(details)

        # 每个通道有几条信号？
        list__number_of_traces_per_subplot = []
        for detail in [el.strip() for el in self.ui.plainTextEdit_ACMPlotDetails.toPlainText().split('\n') if el.strip()!='']:
            number_of_traces_per_subplot = len( [el.strip() for el in detail.split(',') if el.strip()!=''] )
            list__number_of_traces_per_subplot.append(number_of_traces_per_subplot)

        # #define DATA_DETAILS
        details = [el.strip() for el in re.split('\n|,', details) if el.strip()!='']
        # self.list__detail = details

        # print(details)
        return details, list__number_of_traces_per_subplot, labels
    # save setting, compile .c and run .exe
    def compile_and_run(self):
        # compile c and run
        if os.path.exists(self.path2acmsimc+"/dat/info.dat"):
            os.remove(self.path2acmsimc+"/dat/info.dat")
        os.system(f"cd /d {self.path2acmsimc}/c && gmake main && start cmd /c main")
        while not os.path.exists(self.path2acmsimc+"/dat/info.dat"):
            # print('sleep for info.dat')
            sleep(0.1)

    def runCBasedSimulation(self, bool_always_true_bug=True, bool_savePlotSetting=True, bool_updatePlotSetting=True):
        # why bool_ is always set to False???
        # print(bool_, bool_savePlotSetting, bool_updatePlotSetting)

        self.save_Config4CBasedSimulation()

        def savePlotSettings():
            with open(  self.filepath_to_ACMPlotLabels, 'w') as f:
                f.write(self.ui.plainTextEdit_ACMPlotLabels.toPlainText())
            with open(  self.filepath_to_ACMPlotSignals, 'w') as f:
                f.write(self.ui.plainTextEdit_ACMPlotDetails.toPlainText())

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

        def updateACMConfig(path2acmsimc, motor_dict, control_dict, sweepFreq_dict):
            # print(motor_dict)
            def conditions_to_continue(line):
                # 原来如果有定义这些宏，那就不要了
                if     '#define IM_STAOTR_RESISTANCE' in line \
                    or '#define IM_ROTOR_RESISTANCE' in line \
                    or '#define IM_TOTAL_LEAKAGE_INDUCTANCE' in line \
                    or '#define IM_MAGNETIZING_INDUCTANCE' in line \
                    or '#define IM_FLUX_COMMAND_DC_PART' in line \
                    or '#define IM_FLUX_COMMAND_SINE_PART' in line \
                    or '#define PMSM_RESISTANCE' in line \
                    or '#define PMSM_D_AXIS_INDUCTANCE' in line \
                    or '#define PMSM_Q_AXIS_INDUCTANCE' in line \
                    or '#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE' in line \
                    or '#define MOTOR_NUMBER_OF_POLE_PAIRS' in line \
                    or '#define MOTOR_RATED_CURRENT_RMS' in line \
                    or '#define MOTOR_RATED_POWER_WATT' in line \
                    or '#define MOTOR_RATED_SPEED_RPM' in line \
                    or '#define MOTOR_SHAFT_INERTIA' in line \
                    or '#define INDIRECT_FOC 1' in line \
                    or '#define MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL 2' in line \
                    or '#define NULL_D_AXIS_CURRENT_CONTROL -1' in line \
                    or '#define MTPA -2' in line:
                    return True
                else:
                    return False

            # self = None
            NUMBER_OF_STEPS_CL_TS = motor_dict['EndTime']/motor_dict['CL_TS']
            # print('NUMBER_OF_STEPS_CL_TS', NUMBER_OF_STEPS_CL_TS)
            with open(path2acmsimc+'/c/ACMConfig.h', 'r', encoding='utf-8') as f:
                new_lines = []
                for line in f.readlines():
                    # Basic Quantities
                    if   '#define NUMBER_OF_STEPS' in line: new_lines.append(f'#define NUMBER_OF_STEPS {NUMBER_OF_STEPS_CL_TS:.0f}\n'); continue
                    elif '#define CL_TS '          in line: new_lines.append(f'#define CL_TS          ({motor_dict["CL_TS"]:g})\n'); continue
                    elif '#define CL_TS_INVERSE'   in line: new_lines.append(f'#define CL_TS_INVERSE  ({1.0/motor_dict["CL_TS"]:g})\n'); continue
                    elif '#define VL_TS '          in line: new_lines.append(f'#define VL_TS          ({motor_dict["VL_TS"]:g})\n'); continue
                    elif '#define DATA_FILE_NAME'  in line: new_lines.append(f'#define DATA_FILE_NAME "{self.data_file_name}"\n'); continue

                    # Load Related Quantities;
                    elif '#define LOAD_INERTIA'    in line: new_lines.append(f'#define LOAD_INERTIA    {motor_dict["JLoadRatio"]}\n'); continue
                    elif '#define LOAD_TORQUE'     in line: new_lines.append(f'#define LOAD_TORQUE     {motor_dict["Tload"]}\n'); continue
                    elif '#define VISCOUS_COEFF'   in line: new_lines.append(f'#define VISCOUS_COEFF   {motor_dict["ViscousCoeff"]}\n'); continue

                    # Machine Type
                    if "Induction Machine" in self.ui.comboBox_MachineType.currentText():
                        if '#define MACHINE_TYPE' in line: 
                            new_lines.append(f'#define MACHINE_TYPE {1}\n') # 11 or 1
                            # Machine Parameters
                            new_lines.append(f'\t#define IM_STAOTR_RESISTANCE        {motor_dict["Rs"]}\n')
                            new_lines.append(f'\t#define IM_ROTOR_RESISTANCE         {motor_dict["Rreq"]}\n')
                            new_lines.append(f'\t#define IM_TOTAL_LEAKAGE_INDUCTANCE {motor_dict["Lsigma"]}\n')
                            new_lines.append(f'\t#define IM_MAGNETIZING_INDUCTANCE   {motor_dict["Lmu"]}\n')
                            new_lines.append(f'\t#define IM_FLUX_COMMAND_DC_PART     {motor_dict["flux_cmd_dc_part"]}\n')
                            new_lines.append(f'\t#define IM_FLUX_COMMAND_SINE_PART   {motor_dict["flux_cmd_sine_part"]}\n')
                            new_lines.append(f'\t#define MOTOR_NUMBER_OF_POLE_PAIRS  {motor_dict["n_pp"]}\n')
                            new_lines.append(f'\t#define MOTOR_RATED_CURRENT_RMS     {motor_dict["IN"]}\n')
                            new_lines.append(f'\t#define MOTOR_RATED_POWER_WATT      {motor_dict["PW"]}\n')
                            new_lines.append(f'\t#define MOTOR_RATED_SPEED_RPM       {motor_dict["RPM"]}\n')
                            new_lines.append(f'\t#define MOTOR_SHAFT_INERTIA         {motor_dict["J_s"]}\n')
                            continue
                        if 'define CONTROL_STRATEGY' in line:
                            # Control Methods
                            new_lines.append(f'\t#define INDIRECT_FOC 1\n')
                            new_lines.append(f'\t#define MARINO_2005_ADAPTIVE_SENSORLESS_CONTROL 2\n')
                            new_lines.append(line)
                            continue
                    elif "Synchronous Machine" in self.ui.comboBox_MachineType.currentText():
                        if '#define MACHINE_TYPE' in line: 
                            new_lines.append(f'#define MACHINE_TYPE {2}\n')
                            # Machine Parameters
                            new_lines.append(f'\t#define PMSM_RESISTANCE                    {motor_dict["Rs"]}\n')
                            new_lines.append(f'\t#define PMSM_D_AXIS_INDUCTANCE             {motor_dict["Ld"]}\n')
                            new_lines.append(f'\t#define PMSM_Q_AXIS_INDUCTANCE             {motor_dict["Lq"]}\n')
                            new_lines.append(f'\t#define PMSM_PERMANENT_MAGNET_FLUX_LINKAGE {motor_dict["KE"]}\n')
                            new_lines.append(f'\t#define MOTOR_NUMBER_OF_POLE_PAIRS         {motor_dict["n_pp"]}\n')
                            new_lines.append(f'\t#define MOTOR_RATED_CURRENT_RMS            {motor_dict["IN"]}\n')
                            new_lines.append(f'\t#define MOTOR_RATED_POWER_WATT             {motor_dict["PW"]}\n')
                            new_lines.append(f'\t#define MOTOR_RATED_SPEED_RPM              {motor_dict["RPM"]}\n')
                            new_lines.append(f'\t#define MOTOR_SHAFT_INERTIA                {motor_dict["J_s"]}\n')
                            continue
                        if 'define CONTROL_STRATEGY' in line:
                            # Control Methods
                            new_lines.append(f'\t#define NULL_D_AXIS_CURRENT_CONTROL -1\n')
                            new_lines.append(f'\t#define MTPA -2 // not supported\n')
                            new_lines.append(line)
                            # new_lines.append(f'#define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL')
                            continue
                    # Ignore old Machiner Parameters macros
                    if conditions_to_continue(line): continue

                    # Control Related Quantities
                    if len(control_dict.keys()) > 2: # there could be the case in which only ExcitationType assigned to control_dict
                        # PID Coefficients
                        if   '#define CURRENT_KP '     in line: new_lines.append(f'#define CURRENT_KP ({control_dict["currentPI"][0]:g})\n'); continue
                        elif '#define CURRENT_KI '     in line: new_lines.append(f'#define CURRENT_KI ({control_dict["currentPI"][1]:g})\n'); continue
                        elif '#define SPEED_KP '       in line: new_lines.append(f'#define SPEED_KP ({  control_dict["speedPI"]  [0]:g})\n'); continue
                        elif '#define SPEED_KI '       in line: new_lines.append(f'#define SPEED_KI ({  control_dict["speedPI"]  [1]:g})\n'); continue
                        elif '#define EXCITATION_TYPE' in line: new_lines.append(f'#define EXCITATION_TYPE ({control_dict["ExcitationType"]})\n'); continue
                        elif '#define CURRENT_LOOP_LIMIT_VOLTS' in line: new_lines.append(f'#define CURRENT_LOOP_LIMIT_VOLTS ({motor_dict["Udc"]})\n'); continue

                    # Sweep Frequency Related Quantities
                    if len(sweepFreq_dict.keys()) > 0:
                        if   '#define SWEEP_FREQ_MAX_FREQ'  in line: new_lines.append(f'#define SWEEP_FREQ_MAX_FREQ {     sweepFreq_dict["max_freq"]:.0f}\n'); continue
                        elif '#define SWEEP_FREQ_INIT_FREQ' in line: new_lines.append(f'#define SWEEP_FREQ_INIT_FREQ {    sweepFreq_dict["init_freq"]:.0f}\n'); continue
                        elif '#define SWEEP_FREQ_C2V'       in line: new_lines.append(f'#define SWEEP_FREQ_C2V {"TRUE" if sweepFreq_dict["SWEEP_FREQ_C2V"] else "FALSE"}\n'); continue
                        elif '#define SWEEP_FREQ_C2C'       in line: new_lines.append(f'#define SWEEP_FREQ_C2C {"TRUE" if sweepFreq_dict["SWEEP_FREQ_C2C"] else "FALSE"}\n'); continue

                    # No if-clause is activated, so simply append the line.
                    new_lines.append(line)
            with open(path2acmsimc+'/c/ACMConfig.h', 'w', encoding='utf-8') as f:
                f.writelines(new_lines)

        # update panel inputs
        self.motor_dict = self.get_motor_dict(self.mj)
        self.control_dict = self.get_control_dict()
        self.sweepFreq_dict = self.get_sweepFreq_dict()

        # save 
        if bool_savePlotSetting: savePlotSettings()

        # decode labels and signals for plot
        details, _, _ = self.decode_labelsAndSignals()

        # update path/to/acmsimcv5/c/utility.c
        if bool_updatePlotSetting: updatePlotSettings(self.path2acmsimc, details)
        self.data_file_name = self.get_data_file_name() # 这里是正式产生 self.data_file_name 的地方哦
        updateACMConfig(self.path2acmsimc, self.motor_dict, self.control_dict, self.sweepFreq_dict)
        print('ACMConfig.h is updated.')

        if self.ui.checkBox_compileAndRun.isChecked():
            self.compile_and_run()

            # Animate ACMPlot
            # print('sleep for .dat file')
            # sleep(2) # it takes time for main.exe to write data into the disk.
            # but we don't need to use sleep() to wait for that anymore, because we have animated ACMPlot now which works well with incomplete data
            self.update_ACMPlot()
    # def runCBasedSimulation_SweepFrequnecyAnalysis(self):

    def runCBasedSimulation4PA(self, bool_always_false_bug=99999999, bool_=88888888):
        # see the bug clearly
        # print(self, bool_always_false_bug, bool_)

        # Basic Run to make sure everything is up-to-date and saved.
        self.runCBasedSimulation()

        # 先根据多个参数的名字，找到它们在文件ACMConfig.h中的行数，即PAIndex。
        PANameTuple = eval(self.ui.plainTextEdit_PANames.toPlainText())
        original_buf = None
        PAIndexList = []
        with open(self.path2acmsimc+'/c/ACMConfig.h', 'r', encoding='utf-8') as f:
            original_buf = list(f.readlines())
            for index, line in enumerate(original_buf):
                for PAName in PANameTuple:
                    if ' ' + PAName + ' ' in line:
                        PAIndexList.append(index)
                        break

        # 输出数据文件所在行数
        DataFileNameIndex = None
        for i in range(-1,-100, -1):
            if 'DATA_FILE_NAME' in original_buf[i]:
                DataFileNameIndex = i
                break

        # 根据每一个PAValueTuple，更新ACMConfig.h，并编译运行，保存数据。
        count = 0
        for _, PAValueTuple in enumerate(eval(self.ui.plainTextEdit_PAValues.toPlainText())):
            if PAValueTuple is ():
                return 'PAValueTuple is empty.'
            count += 1

            write_buf = copy.deepcopy(original_buf)

            # 打开文件
            with open(self.path2acmsimc+'/c/ACMConfig.h', 'w', encoding='utf-8') as f:

                # 每个参数都要修改ACMConfig.h文件的一行
                for PAName, PAIndex, PAValue in zip(PANameTuple, PAIndexList, PAValueTuple):
                    write_buf[PAIndex] = f'#define {PAName} {PAValue}\n'

                # 修改输出数据文件名
                write_buf[DataFileNameIndex] = f'#define DATA_FILE_NAME "{self.data_file_name}-{_+1:03d}"\n'

                # 写入ACMConfig.h
                f.writelines(write_buf)

            # 编译+运行C
            self.compile_and_run()

        # stop the animation as we are going to plot the canvas for PA
        self.anim.event_source.stop()


        # 更新可选数据文件下拉菜单
        data_file_name_list = self.getDataFileNameList4PA()
        self.ui.comboBox_PADataFileSelected.clear()
        self.ui.comboBox_PADataFileSelected.addItems(data_file_name_list)
        self.ui.comboBox_PADataFileSelected.activated.connect(self.update_PASelected)


        # 画图
        self.plot4PA(data_file_name_list = [f"{self.data_file_name}-{_+1:03d}" for _ in range(count)])
    def getDataFileNameList4PA(self):
        data_file_name = self.get_data_file_name()
        target = data_file_name[len("../dat/"):]
        data_file_name_list = ["../dat/"+file for file in os.listdir(self.path2acmsimc+'/dat/') if target in file] #  and 'dat-' in file
        return data_file_name_list
    def plot4PA(self, bool_always_false_bug=None, data_file_name_list=None):
        # print(bool_always_false_bug, data_file_name_list)

        # 不运行C，直接画图的话
        if data_file_name_list is None:
            data_file_name_list = self.getDataFileNameList4PA()

        def 类似ACMアニメ_但只画第一个信号_j等于0(df_profiles, no_samples, no_traces, PAIndex=None):
            time = np.arange(1, no_samples+1) * self.motor_dict['DOWN_SAMPLE'] * self.motor_dict['CL_TS']
            trace_counter = 0
            first_ax = None
            for i, number_of_traces_per_subplot in enumerate(list__number_of_traces_per_subplot):
                ax = axes[i]
                # ax = self.ui.MplWidget_ACMPlot.canvas.figure.add_subplot(number_of_subplot, 1, 1+i, sharex=first_ax)
                # if first_ax is None:
                #     first_ax = ax

                for j in range(number_of_traces_per_subplot):
                    key = details[trace_counter]
                    signal = df_profiles[key]
                    trace_counter += 1
                    if j==number_of_traces_per_subplot-1: # 我改变主意了，还是画最后一个吧
                        ax.plot(time, signal, '-.', lw=1.0, label=key+f'{PAIndex}', alpha=0.75)
                ax.set_ylabel(list__label[i])
                ax.legend(loc='lower center')
            ax.set_xlabel('Time [s]')
            return no_traces

        details, list__number_of_traces_per_subplot, list__label = self.decode_labelsAndSignals()
        number_of_subplot = len(list__number_of_traces_per_subplot)

        self.ui.MplWidget_ACMPlot.canvas.figure.clf()
        axes = self.ui.MplWidget_ACMPlot.canvas.figure.subplots(nrows=number_of_subplot, ncols=1, sharex=True)
        for index, data_file_name in enumerate(data_file_name_list[::-1]): # 我要倒着画
            PAIndex = data_file_name[-4:] # len(data_file_name_list) - index
            print('Parametric Analysis Read In:', data_file_name)
            no_traces = 类似ACMアニメ_但只画第一个信号_j等于0(*self.get_dataFrame(data_file_name), PAIndex=PAIndex) # df_profiles, no_samples, no_traces


        # adjust height per number of traces
        self.ui.MplWidget_ACMPlot.setMinimumSize(QtCore.QSize(500, 200*no_traces))
        # draw
        self.ui.MplWidget_ACMPlot.canvas.draw()
    def update_PASelected(self):
        if self.ui.comboBox_PADataFileSelected.currentText()[-3:] == 'dat':
            self.ui.lineEdit_PAValueSelected.setText('')
            self.ui.label_PANameSelected.setText('')
        else:
            PAIndex = int(self.ui.comboBox_PADataFileSelected.currentText()[-3:]) - 1
            self.ui.lineEdit_PAValueSelected.setText(', '.join(f'{el:g}' for el in eval(self.ui.plainTextEdit_PAValues.toPlainText())[PAIndex]))
            self.ui.label_PANameSelected.setText(self.ui.plainTextEdit_PANames.toPlainText()) # maybe replace \n with ?

    ''' Optimization Section '''
    # choose machine specification
    def update_machineSpec(self):
        pass
    def update_FEAConfig(self):
        pass



    ''' General Information '''
    def comboActivate_machineType(self):
        pmsm_list, im_list = [], []
        for key, machine in self.mj.items():
            # Permanent Magnet Motor
            if '永磁' in machine['基本参数']['电机类型']:
                pmsm_list.append(key)
            elif '感应' in machine['基本参数']['电机类型']:
                im_list.append(key)
        self.ui.comboBox_MachineName.clear()
        if 'Synchronous Machine' in self.ui.comboBox_MachineType.currentText():
            self.ui.comboBox_MachineName.addItems(pmsm_list) #(self.mj.keys())
        elif 'Induction Machine' in self.ui.comboBox_MachineType.currentText():
            self.ui.comboBox_MachineName.addItems(im_list) #(self.mj.keys())


        # Recover last user input
        self.filepath_to_configini = pkg_resources.resource_filename(__name__, f'config_{self.get_mtype()}.json') # updated in self.comboActivate_machineType()
        try:
            with open(self.filepath_to_configini, 'r') as f:
                configini = self.configini = json.load(f)
            if os.path.exists(configini['path2acmsimc']):
                self.path2acmsimc = configini['path2acmsimc']
            else:
                print(f'''Path "{configini['path2acmsimc']}" does not exist. Will use default C codes instead.''')
                self.path2acmsimc = path2acmsimc # the default code comes along with the emachinery package
            self.ui.lineEdit_EndTime.setText(configini['EndTime'])
            self.ui.lineEdit_CLTS.setText(configini['CLTS'])
            self.ui.lineEdit_VLTS.setText(configini['VLTS'])
            self.ui.lineEdit_LoadTorque.setText(configini['LoadTorque'])
            self.ui.lineEdit_LoadInertiaPercentage.setText(configini['LoadInertiaPercentage'])
            self.ui.lineEdit_ViscousCoefficient.setText(configini['ViscousCoefficient'])
            self.ui.lineEdit_OutputDataFileName.setText(configini['OutputDataFileName'])
        except Exception as e:
            raise e

        # 更新绘图信号对
        # update file path to plot settings
        self.update_ACMPlotLabels_and_ACMPlotSignals(configini, bool_re_init=True)
            # self.ui.lineEdit_path2ACMPlotLabels.setText('./plot_setting_files/labels_pmsm.txt')
            # self.ui.lineEdit_path2ACMPlotSignals.setText('./plot_setting_files/signals_pmsm.txt')
            # self.ui.lineEdit_path2ACMPlotLabels.setText('./plot_setting_files/labels_im.txt')
            # self.ui.lineEdit_path2ACMPlotSignals.setText('./plot_setting_files/signals_im.txt')
            # self.update_ACMPlotLabels_and_ACMPlotSignals()

        # 更新铭牌值
        self.comboActivate_namePlateData()

    def comboActivate_namePlateData(self):
        self.motor_dict = self.get_motor_dict(self.mj)
        # print('debug main.py', self.motor_dict)
        self.ui.lineEdit_npp         .setText(str(self.motor_dict['n_pp']))
        self.ui.lineEdit_RatedCurrent.setText(str(self.motor_dict['IN']))
        self.ui.lineEdit_RatedPower  .setText(str(self.motor_dict['PW']))
        self.ui.lineEdit_RatedSpeed  .setText(str(self.motor_dict['RPM']))

        # tab "C-based Simulation" 另一个tab下的LineEdit也同步更新
        self.ui.lineEdit_RO_MachineName.setText(self.ui.comboBox_MachineName.currentText())
        self.ui.lineEdit_DCBusVoltage.setText(str(self.motor_dict['Udc']))
    # parameter conversion
    def console_push_variable(self, d):
        self.ui.ConsoleWidget.push_vars(d)
        if self.ui.label_pushedVariables.text() == 'None':
            self.ui.label_pushedVariables.setText(', '.join(d.keys()))
        else:
            # Duplicated variable names would exist
            # self.ui.label_pushedVariables.setText(self.ui.label_pushedVariables.text()+', '+', '.join(d.keys()))

            # Avoid duplicated variable names
            for key in d.keys():
                if key not in self.ui.label_pushedVariables.text():
                    self.ui.label_pushedVariables.setText(self.ui.label_pushedVariables.text()+f', {key}')
    def update_emy(self):
        self.emy = ElectricMachinery( NUMBER_OF_POLE_PAIRS  = int  (self.ui.lineEdit_npp.text()),
                                      RATED_CURRENT_RMS     = float(self.ui.lineEdit_RatedCurrent.text()),
                                      RATED_POWER_WATT      = float(self.ui.lineEdit_RatedPower.text()),
                                      RATED_SPEED_RPM       = float(self.ui.lineEdit_RatedSpeed.text()),
            )
        self.console_push_variable({'emy':self.emy})
    # decide which motor is used
    def get_motor_dict(self, mj):
        # read from json file
        motor = mj[self.ui.comboBox_MachineName.currentText()]["基本参数"]
        motor_dict = dict()
        motor_dict['DOWN_SAMPLE'] = 1 # set here but not implemented in c-simulation

        motor_dict['n_pp'] = n_pp = motor["极对数 [1]"]
        motor_dict['IN']   = IN   = motor["额定电流 [Arms]"]
        motor_dict['PW']   = PW   = motor["额定功率 [Watt]"]
        motor_dict['RPM']  = RPM  = motor["额定转速 [rpm]"]

        motor_dict['J_s']  = J_s  = motor["转动惯量 [kg.cm^2]"]*1e-4
        motor_dict['Udc']  = Udc  = motor["母线电压 [Vdc]"]

        if '感应' in motor['电机类型']:
            motor_dict['Rs']     = Rs     = motor["定子电阻 [Ohm]"]
            motor_dict['Rreq']   = Rreq   = motor["反伽马转子电阻 [Ohm]"]
            motor_dict['Lsigma'] = Lsigma = motor["反伽马漏电感 [mH]"]*1e-3
            motor_dict['Lmu']    = Lmu    = motor["定子D轴电感 [mH]"]*1e-3 - Lsigma
            motor_dict['KE']     = KE     = motor["额定反电势系数 [Wb]"]
            motor_dict['Ls']     = Lsigma + Lmu # this will be used in tuner.py for iteration for current BW

            motor_dict['flux_cmd_dc_part']   = KE * eval(self.ui.lineEdit_FluxCmdPercentage.text())*0.01
            motor_dict['flux_cmd_sine_part'] = motor_dict['flux_cmd_dc_part'] * eval(self.ui.lineEdit_FluxCmdSinePartPercentage.text())*0.01

        elif '永磁' in motor['电机类型']:
            motor_dict['Rs'] = R  = motor["定子电阻 [Ohm]"]
            motor_dict['Ld'] = Ld = motor["定子D轴电感 [mH]"]*1e-3
            motor_dict['Lq'] = Lq = motor["定子Q轴电感 [mH]"]*1e-3
            motor_dict['KE'] = KE = motor["额定反电势系数 [Wb]"]
            motor_dict['Ls'] = Lq  # this will be used in tuner.py for iteration for current BW

        # below is by user GUI input
        motor_dict['CL_TS'] = CL_TS = eval(self.ui.lineEdit_CLTS.text())
        motor_dict['VL_TS'] = VL_TS = eval(self.ui.lineEdit_VLTS.text())
        motor_dict['EndTime'] = EndTime = eval(self.ui.lineEdit_EndTime.text())

        motor_dict['JLoadRatio']   = eval(self.ui.lineEdit_LoadInertiaPercentage.text())*0.01 #[%]
        motor_dict['Tload']        = eval(self.ui.lineEdit_LoadTorque.text()) #[Nm]
        motor_dict['ViscousCoeff'] = eval(self.ui.lineEdit_ViscousCoefficient.text()) #[Nm/(rad/s)]

        motor_dict['data_file_name_prefix'] = self.ui.lineEdit_OutputDataFileName.text()

        # pass to console
        self.console_push_variable({'motor_dict':motor_dict})

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
    window = EmachineryWidget(path2acmsimc=__file__[:-11]+'acmsimcv5')
    window.show()
    app.exec_()

if __name__ == '__main__':
    # print(os.path.dirname(os.path.realpath(__file__)))
    main()
