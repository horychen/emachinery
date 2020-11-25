from PyQt5.QtWidgets import QApplication,  QMainWindow
import sys
from PyQt5 import QtGui



class Window(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setGeometry(300, 300, 500, 400)
        self.setWindowTitle("Figure PyQt5 EMachinerY")

        # self.textbox = QLinez

        self.show()

def run():
    App = QApplication(sys.argv)
    window = Window()
    sys.exit(App.exec())

if __name__ == '!__main__':
    run()







import sys
import time

import numpy as np

# https://matplotlib.org/3.3.2/gallery/user_interfaces/embedding_in_qt_sgskip.html
from matplotlib.backends.qt_compat import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import (
    FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI_matplotlib()
        self.initUI()

    def initUI_matplotlib(self):
        self._main = QtWidgets.QWidget()
        self.setCentralWidget(self._main)
        layout = QtWidgets.QVBoxLayout(self._main)

        static_canvas = FigureCanvas(Figure(figsize=(5, 3)))
        layout.addWidget(static_canvas)
        self.addToolBar(NavigationToolbar(static_canvas, self))

        dynamic_canvas = FigureCanvas(Figure(figsize=(5, 3)))
        layout.addWidget(dynamic_canvas)
        self.addToolBar(QtCore.Qt.BottomToolBarArea,
                        NavigationToolbar(dynamic_canvas, self))

        self._static_ax = static_canvas.figure.subplots()
        t = np.linspace(0, 10, 501)
        self._static_ax.plot(t, np.tan(t), ".")

        self._dynamic_ax = dynamic_canvas.figure.subplots()
        t = np.linspace(0, 10, 101)
        # Set up a Line2D.
        self._line, = self._dynamic_ax.plot(t, np.sin(t + time.time()))
        self._timer = dynamic_canvas.new_timer(50)
        self._timer.add_callback(self._update_canvas)
        self._timer.start()

    def initUI(self):

        layout = QtWidgets.QVBoxLayout(self._main)

        # my code
        self.setWindowTitle('Figure emachinery')
        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText('TT')
        self.label1.move(200,200)

        b1 = QtWidgets.QPushButton(self)
        b1.setText('Scie me')
        def b1_clicked():
            self.label1.setText('you clicked')
            # self._update_label()
        b1.clicked.connect(b1_clicked)
        layout.addWidget(b1)

    # def _update_label(self):
    #     self.label1.adjustSize()

    def _update_canvas(self):
        t = np.linspace(0, 10, 101)
        # Shift the sinusoid as a function of time.
        self._line.set_data(t, np.sin(t + time.time()))
        self._line.figure.canvas.draw()


if __name__ == "!__main__":
    # Check whether there is already a running QApplication (e.g., if running
    # from an IDE).
    qapp = QtWidgets.QApplication.instance()
    if not qapp:
        qapp = QtWidgets.QApplication(sys.argv)

    app = ApplicationWindow()
    app.show()
    app.activateWindow()
    app.raise_()
    qapp.exec_()














from threading import Thread
from pyqtconsole.console import PythonConsole
if __name__ == '!__main__':

    app = QApplication([])
    console = PythonConsole()
    console.show()
    console.eval_in_thread()

    sys.exit(app.exec_())






# https://stackoverflow.com/questions/11513132/embedding-ipython-qt-console-in-a-pyqt-application
from qtconsole.qt import QtGui
from qtconsole.rich_jupyter_widget import RichJupyterWidget
from qtconsole.inprocess import QtInProcessKernelManager

import logging

class ConsoleWidget(RichJupyterWidget):

    def __init__(self, customBanner=None, *args, **kwargs):
        super(ConsoleWidget, self).__init__(*args, **kwargs)

        if customBanner is not None:
            self.banner = customBanner

        self.font_size = 6
        self.kernel_manager = kernel_manager = QtInProcessKernelManager()

        kernel_manager.start_kernel(show_banner=False)
        # kernel_manager.kernel.log.setLevel(logging.CRITICAL) # 
        kernel_manager.kernel.gui = 'qt'
        self.kernel_client = kernel_client = self._kernel_manager.client()
        kernel_client.start_channels()

        def stop():
            kernel_client.stop_channels()
            kernel_manager.shutdown_kernel()
            guisupport.get_app_qt().exit()

        self.exit_requested.connect(stop)

    def push_vars(self, variableDict):
        """
        Given a dictionary containing name / value pairs, push those variables
        to the Jupyter console widget
        """
        self.kernel_manager.kernel.shell.push(variableDict)

    def clear(self):
        """
        Clears the terminal
        """
        self._control.clear()

        # self.kernel_manager

    def print_text(self, text):
        """
        Prints some plain text to the console
        """
        self._append_plain_text(text)

    def execute_command(self, command):
        """
        Execute a command in the frame of the console widget
        """
        self._execute(command, False)


if __name__ == '__main__':
    app = QtGui.QApplication([])
    widget = ConsoleWidget()
    widget.show()
    app.exec_()
