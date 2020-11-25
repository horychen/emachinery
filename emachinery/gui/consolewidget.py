
# https://stackoverflow.com/questions/11513132/embedding-ipython-qt-console-in-a-pyqt-application
from qtconsole.qt import QtGui
from qtconsole.rich_jupyter_widget import RichJupyterWidget
from qtconsole.inprocess import QtInProcessKernelManager

# import logging

from PyQt5.QtWidgets import QWidget

class ConsoleWidget(RichJupyterWidget):

    # def __init__(self, customBanner=None, *args, **kwargs):
    def __init__(self, parent=None, *args, **kwargs):
        QWidget.__init__(self, parent)

        super(ConsoleWidget, self).__init__(*args, **kwargs)

        # if customBanner is not None:
        #     self.banner = customBanner
        self.banner = "This is an embedded qtconsole.\nSuggested commands:\n\tvars(emy)\n\n"
                    # [attr for attr in dir(emy) if not callable(getattr(emy, attr)) and not attr.startswith('__')]
                    # https://stackoverflow.com/questions/1398022/looping-over-all-member-variables-of-a-class-in-python

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
