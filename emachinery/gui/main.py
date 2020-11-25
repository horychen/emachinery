# https://www.youtube.com/watch?v=2C5VnE9wPhk
# ------------------------------------------------------
# ---------------------- main.py -----------------------
# ------------------------------------------------------
# background-color: rgb(255, 255, 255)

from PyQt5.QtWidgets import QApplication, QMainWindow

from PyQt5.uic import loadUi

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)

import numpy as np
import random

from emachinery.utils.conversion import ElectricMachinery
from emachinery.jsons import ACMInfo

from stylesheet.toggle_stylesheet import toggle_stylesheet

class EmachineryWidget(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        loadUi("mainWindow.ui", self)

        # title
        self.setWindowTitle("Figure: Electric Machinery")

        # update plot
        self.pushButton_getSignal.clicked.connect(self.update_graph)
        # undate model
        self.pushButton_updateModel.clicked.connect(self.update_emy)

        # Matplotlib navigation bar to: self or tabWidget
        self.toolbar = NavigationToolbar(self.MplWidget.canvas, self)
            # self.addToolBar(self.toolbar) # add to mainWindow
        self.verticalLayout_inTab2.addWidget(self.toolbar) # add to tab 2 only

        # Name Plate Data
        self.mj = ACMInfo.MotorJson().d
        self.comboBox_MachineName.addItems(self.mj.keys())
        self.comboBox_MachineName.activated.connect(self.update_namePlateData)
        self.update_namePlateData()

        # Style sheet
        self.actionDark.triggered.connect(lambda: toggle_stylesheet("./stylesheet/QDarkStyleSheet.qss"))
        self.actionLight.triggered.connect(lambda: toggle_stylesheet(""))

        # Recover last input

        # self.lineEdit_npp.textChanged[str].connect(self.doSomething)

    def update_namePlateData(self):
        motor_dict = self.get_motor(self.comboBox_MachineName.currentText())
        self.lineEdit_npp         .setText(str(motor_dict['n_pp']))
        self.lineEdit_RatedCurrent.setText(str(motor_dict['IN']))
        self.lineEdit_RatedPower  .setText(str(motor_dict['PW']))
        self.lineEdit_RatedSpeed  .setText(str(motor_dict['RPM']))

    def update_emy(self):
        self.emy = ElectricMachinery( NUMBER_OF_POLE_PAIRS  = int  (self.lineEdit_npp.text()),
                                      RATED_CURRENT_RMS     = float(self.lineEdit_RatedCurrent.text()),
                                      RATED_POWER_WATT      = float(self.lineEdit_RatedPower.text()),
                                      RATED_SPEED_RPM       = float(self.lineEdit_RatedSpeed.text()),
            )
        self.ConsoleWidget.push_vars({
            'emy': self.emy
            })
        self.label_pushedVariables.setText('emy')

    def get_motor(self, motor_name="SEW200W (SF60B04030C2004)"):
        motor = self.mj[motor_name]["基本参数"]

        n_pp = motor["极对数 [1]"]
        R    = motor["电机线电阻 [Ohm]"]/2
        L    = motor["电机D轴线电感 [mH]"]/2*1e-3
        KT   = motor["转矩常数 [Nm/Arms]"] * 1.414
        KE   = motor["转矩常数 [Nm/Arms]"] / 1.5 / n_pp * 1.414
        J_s  = motor["转动惯量 [kg.cm^2]"]*1e-4
        IN   = motor["额定电流 [Arms]"]
        PW   = motor["额定功率 [Watt]"]
        RPM  = motor["额定转速 [rpm]"]

        motor_dict = dict()
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

        motor_dict['JLoadRatio']   = JLoadRatio = 0.4 # 0.16 # 3 [%]
        motor_dict['Tload']        = TLoad      = 0.0*0.07 # 0.05 # [Nm]
        motor_dict['ViscousCoeff'] = B = 0.7e-4
        return motor_dict

    def update_graph(self):
        fs = 500
        f = random.randint(1, 100)
        ts = 1/fs
        length_of_signal = 100
        t = np.linspace(0,1,length_of_signal)
        
        cosinus_signal = np.cos(2*np.pi*f*t)
        sinus_signal = np.sin(2*np.pi*f*t)

        self.MplWidget.canvas.axes.clear()
        self.MplWidget.canvas.axes.plot(t, cosinus_signal)
        self.MplWidget.canvas.axes.plot(t, sinus_signal)
        self.MplWidget.canvas.axes.legend(('cosinus', 'sinus'),loc='upper right')
        self.MplWidget.canvas.axes.set_title('Cosinus - Sinus Signal')
        self.MplWidget.canvas.draw()

def main():
    app = QApplication([])
    window = EmachineryWidget()
    window.show()
    app.exec_()

if __name__ == '__main__':
    main()
