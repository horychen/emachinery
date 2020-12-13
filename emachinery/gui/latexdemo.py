# https://stackoverflow.com/questions/32035251/displaying-latex-in-pyqt-pyside-qtablewidget

import sys
from pylab import plt, mpl
from matplotlib.backends.backend_agg import FigureCanvasAgg
from PyQt5 import QtGui, QtCore, QtWidgets

mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.family'] = 'STIXGeneral'

def mathTex_to_QPixmap(mathTex, fs, fontcolor='black'):

    #---- set up a mpl figure instance ----

    fig = plt.figure()
    fig.patch.set_facecolor('none')
    fig.set_canvas(FigureCanvasAgg(fig))
    renderer = fig.canvas.get_renderer()

    #---- plot the mathTex expression ----

    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis('off')
    ax.patch.set_facecolor('none')
    t = ax.text(0, 0, mathTex, ha='left', va='bottom', fontsize=fs, color=fontcolor)

    #---- fit figure size to text artist ----

    fwidth, fheight = fig.get_size_inches()
    fig_bbox = fig.get_window_extent(renderer)

    text_bbox = t.get_window_extent(renderer)

    tight_fwidth = text_bbox.width * fwidth / fig_bbox.width
    tight_fheight = text_bbox.height * fheight / fig_bbox.height

    fig.set_size_inches(tight_fwidth, tight_fheight)

    #---- convert mpl figure to QPixmap ----

    buf, size = fig.canvas.print_to_buffer()
    qimage = QtGui.QImage.rgbSwapped(QtGui.QImage(buf, size[0], size[1],
                                                  QtGui.QImage.Format_ARGB32))
    qpixmap = QtGui.QPixmap(qimage)

    return qpixmap

class LaTeX_Repo(object):
    def __init__(self, fs=18, fontcolor='white'):
        self.qpixmap_CLKP = mathTex_to_QPixmap(r'${\rm curK}_P = L \times \mathrm{curBW}$', fs=fs, fontcolor=fontcolor)
        self.qpixmap_CLKI = mathTex_to_QPixmap(r'${\rm curK}_I = {R}/{L}$', fs=fs, fontcolor=fontcolor)
        self.qpixmap_VLKP = mathTex_to_QPixmap(r'$\mathrm{velK}_P=\frac{\mathrm{curBW}}{\delta}\frac{J_{\mathrm{total}}}{n_{pp}}\frac{1}{\left( \frac{3}{2}n_{pp}K_E \right)}$', fs=fs, fontcolor=fontcolor)
        self.qpixmap_VLKI = mathTex_to_QPixmap(r'$\mathrm{velK}_I=\frac{\mathrm{curBW}}{\delta ^2}$', fs=fs, fontcolor=fontcolor)
        self.qpixmap_Note1 = mathTex_to_QPixmap(r'$\mathrm{Velocity}~\mathrm{PI}~\mathrm{Regulator}~\mathrm{Input}:~\mathrm{Velocity~Control~Error}~\left[ \mathrm{elec}.\mathrm{rad}/\sec \right] $', fs=fs, fontcolor=fontcolor)
        self.qpixmap_Note2 = mathTex_to_QPixmap(r'$\mathrm{Velocity}~\mathrm{PI}~\mathrm{Regulator}~\mathrm{Output}:~\mathrm{Current~Command}~\left[ \mathrm{Apeak} \right] $', fs=fs, fontcolor=fontcolor)
        self.qpixmap_Note3 = mathTex_to_QPixmap(r'${\rm Series~Type~PI~Regulator}:~i_{q}^{*}=\left( \mathrm{velK}_P+\frac{\mathrm{velK}_P\mathrm{velK}_I}{p} \right) \left( \omega ^*-\omega \right) $', fs=fs, fontcolor=fontcolor)

if __name__ == '__main__':
    
    mathTex_to_QPixmap('$C_{soil}=(1 - n) C_m + \\theta_w C_w$', fs=16)
