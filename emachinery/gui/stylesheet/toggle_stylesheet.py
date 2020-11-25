from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QFile, QTextStream

import pkg_resources # to include resource file: mainWindow.ui # https://stackoverflow.com/questions/6028000/how-to-read-a-static-file-from-inside-a-python-package/20885799

def toggle_stylesheet(path):
    '''
    Toggle the stylesheet to use the desired path in the Qt resource
    system (prefixed by `:/`) or generically (a path to a file on
    system).

    :path:      A full path to a resource or file on system
    '''

    path = pkg_resources.resource_filename(__name__, path)

    # get the QApplication instance,  or crash if not set
    app = QApplication.instance()
    if app is None:
        raise RuntimeError("No Qt Application found.")

    if path == "":
        app.setStyleSheet("")
        return

    file = QFile(path)
    file.open(QFile.ReadOnly | QFile.Text)
    stream = QTextStream(file)
    app.setStyleSheet(stream.readAll())
