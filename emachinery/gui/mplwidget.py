# ------------------------------------------------------
# -------------------- mplwidget.py --------------------
# ------------------------------------------------------
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from matplotlib.backends.backend_qt5agg import FigureCanvas

from matplotlib.figure import Figure

from pylab import plt

class MplWidget(QWidget):
    
    def __init__(self, parent = None):

        QWidget.__init__(self, parent)
        
        self.canvas = FigureCanvas(Figure(tight_layout=True))
        
        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.canvas)
        
        self.setLayout(vertical_layout)

        self.canvas.axes = None #self.canvas.figure.add_subplot(111)


        # add new animation feature
        def on_press(event):
            if event.key.isspace():
                if anim.running:
                    anim.event_source.stop()
                else:
                    anim.event_source.start()
                anim.running ^= True
        self.canvas.mpl_connect('key_press_event', on_press)

