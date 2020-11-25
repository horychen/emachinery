# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 945)
        MainWindow.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 801, 921))
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.tab)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 288, 199))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.comboBox_MachineType = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.comboBox_MachineType.setObjectName("comboBox_MachineType")
        self.comboBox_MachineType.addItem("")
        self.comboBox_MachineType.addItem("")
        self.verticalLayout.addWidget(self.comboBox_MachineType)
        self.comboBox_MachineName = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.comboBox_MachineName.setObjectName("comboBox_MachineName")
        self.verticalLayout.addWidget(self.comboBox_MachineName)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_npd = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_npd.setFrameShape(QtWidgets.QFrame.Panel)
        self.label_npd.setFrameShadow(QtWidgets.QFrame.Plain)
        self.label_npd.setObjectName("label_npd")
        self.verticalLayout_2.addWidget(self.label_npd)
        self.label_npd_2 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_npd_2.setFrameShape(QtWidgets.QFrame.Panel)
        self.label_npd_2.setObjectName("label_npd_2")
        self.verticalLayout_2.addWidget(self.label_npd_2)
        self.label_npd_3 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_npd_3.setFrameShape(QtWidgets.QFrame.Panel)
        self.label_npd_3.setObjectName("label_npd_3")
        self.verticalLayout_2.addWidget(self.label_npd_3)
        self.label_npd_4 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_npd_4.setFrameShape(QtWidgets.QFrame.Panel)
        self.label_npd_4.setObjectName("label_npd_4")
        self.verticalLayout_2.addWidget(self.label_npd_4)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.lineEdit_npp = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.lineEdit_npp.setObjectName("lineEdit_npp")
        self.verticalLayout_3.addWidget(self.lineEdit_npp)
        self.lineEdit_RatedCurrent = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.lineEdit_RatedCurrent.setObjectName("lineEdit_RatedCurrent")
        self.verticalLayout_3.addWidget(self.lineEdit_RatedCurrent)
        self.lineEdit_RatedPower = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.lineEdit_RatedPower.setObjectName("lineEdit_RatedPower")
        self.verticalLayout_3.addWidget(self.lineEdit_RatedPower)
        self.lineEdit_RatedSpeed = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.lineEdit_RatedSpeed.setObjectName("lineEdit_RatedSpeed")
        self.verticalLayout_3.addWidget(self.lineEdit_RatedSpeed)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.pushButton_updateModel = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton_updateModel.setObjectName("pushButton_updateModel")
        self.horizontalLayout_4.addWidget(self.pushButton_updateModel)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem1)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.ConsoleWidget = ConsoleWidget(self.tab)
        self.ConsoleWidget.setEnabled(True)
        self.ConsoleWidget.setGeometry(QtCore.QRect(10, 220, 781, 661))
        self.ConsoleWidget.setObjectName("ConsoleWidget")
        self.label_pushedVariables = QtWidgets.QLabel(self.tab)
        self.label_pushedVariables.setGeometry(QtCore.QRect(330, 30, 461, 101))
        self.label_pushedVariables.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop)
        self.label_pushedVariables.setObjectName("label_pushedVariables")
        self.label_pushedVariables0 = QtWidgets.QLabel(self.tab)
        self.label_pushedVariables0.setGeometry(QtCore.QRect(330, 10, 141, 16))
        self.label_pushedVariables0.setObjectName("label_pushedVariables0")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.tab_2)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(50, 20, 721, 831))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_inTab2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_inTab2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_inTab2.setObjectName("verticalLayout_inTab2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem2)
        self.pushButton_getSignal = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_getSignal.setFont(font)
        self.pushButton_getSignal.setObjectName("pushButton_getSignal")
        self.horizontalLayout_2.addWidget(self.pushButton_getSignal)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem3)
        self.verticalLayout_inTab2.addLayout(self.horizontalLayout_2)
        self.MplWidget = MplWidget(self.verticalLayoutWidget_2)
        self.MplWidget.setObjectName("MplWidget")
        self.verticalLayout_inTab2.addWidget(self.MplWidget)
        self.tabWidget.addTab(self.tab_2, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 21))
        self.menubar.setObjectName("menubar")
        self.menuLoad = QtWidgets.QMenu(self.menubar)
        self.menuLoad.setObjectName("menuLoad")
        self.menuTheme = QtWidgets.QMenu(self.menubar)
        self.menuTheme.setObjectName("menuTheme")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionLocate = QtWidgets.QAction(MainWindow)
        self.actionLocate.setObjectName("actionLocate")
        self.actionDark = QtWidgets.QAction(MainWindow)
        self.actionDark.setObjectName("actionDark")
        self.actionLight = QtWidgets.QAction(MainWindow)
        self.actionLight.setObjectName("actionLight")
        self.menuLoad.addAction(self.actionLocate)
        self.menuTheme.addAction(self.actionDark)
        self.menuTheme.addAction(self.actionLight)
        self.menubar.addAction(self.menuLoad.menuAction())
        self.menubar.addAction(self.menuTheme.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.comboBox_MachineType.setItemText(0, _translate("MainWindow", "Synchronous Machine"))
        self.comboBox_MachineType.setItemText(1, _translate("MainWindow", "Induction Machine"))
        self.label_npd.setText(_translate("MainWindow", "Number of Poles"))
        self.label_npd_2.setText(_translate("MainWindow", "Rated Current [Arms]"))
        self.label_npd_3.setText(_translate("MainWindow", "Rated Power [W]"))
        self.label_npd_4.setText(_translate("MainWindow", "Rated Speed [rpm]"))
        self.lineEdit_npp.setText(_translate("MainWindow", "0"))
        self.lineEdit_RatedCurrent.setText(_translate("MainWindow", "0"))
        self.lineEdit_RatedPower.setText(_translate("MainWindow", "0"))
        self.lineEdit_RatedSpeed.setText(_translate("MainWindow", "0"))
        self.pushButton_updateModel.setText(_translate("MainWindow", "Update Model"))
        self.label_pushedVariables.setText(_translate("MainWindow", "None"))
        self.label_pushedVariables0.setText(_translate("MainWindow", "Variables Pushed qtConsole: "))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Name Plate Data"))
        self.pushButton_getSignal.setText(_translate("MainWindow", "Generate\n"
"Signals"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Plots"))
        self.menuLoad.setTitle(_translate("MainWindow", "Load"))
        self.menuTheme.setTitle(_translate("MainWindow", "Theme"))
        self.actionLocate.setText(_translate("MainWindow", "Json File"))
        self.actionDark.setText(_translate("MainWindow", "Dark"))
        self.actionLight.setText(_translate("MainWindow", "Light"))
# from consolewidget import ConsoleWidget
# from mplwidget import MplWidget
from emachinery.gui.consolewidget import ConsoleWidget
from emachinery.gui.mplwidget import MplWidget


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
