from dataclasses import dataclass
from pylab import plt, mpl, np
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes, mark_inset
from collections import OrderedDict as OD
import pandas as pd
import os
mpl.rcParams['legend.fontsize'] = 10
mpl.rcParams['font.family'] = ['serif'] # default is sans-serif
mpl.rcParams['font.serif'] = ['Times New Roman']
label_font = {'family':'Times New Roman', 'weight':'normal', 'size':12} # label font
tick_font = {'family':'Times New Roman', 'weight':'normal', 'size':10} # tick font

# this changes font for ticks for my TEC-ISMB paper figures but has no effect here, why?
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.family'] = ['STIXGeneral', 'Times New Roman']
mpl.rcParams['font.size'] = 14.0
mpl.rcParams['legend.fontsize'] = 12.5

# plt.style.use('ggplot') 
# plt.style.use('grayscale') # print plt.style.available # get [u'dark_background', u'bmh', u'grayscale', u'ggplot', u'fivethirtyeight']
mpl.style.use('classic')


######################
# Data
def load_dataframe(path='./', index=0):

    df_list = []
    fname_list = []
    for fname in os.listdir(path):
        if '.dat' in fname and 'info' not in fname and 'pdf' not in fname:
            print('--------------', fname)
            fname_list.append(fname)
            df = pd.read_csv(path+fname, na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
            df_list.append(df)

    # index = 0
    df    = df_list[index]
    fname = fname_list[index]
    no_samples = df.shape[0]
    no_traces  = df.shape[1]
    df_info = pd.read_csv(path+"info.dat", na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
    time = np.arange(1,no_samples+1,1) * float(df_info['DOWN_SAMPLE']) * float(df_info['CL_TS'])  # sampling frequency is 10e3
    print('---- List of Keys:')
    for key in df.keys():
        print('\t', key)

    return df_list, time, fname

######################
# Plotting
def locallyZoomIn(ax, data_zoomed, ylim=None, loc1=None, loc2=None, y_axis_shrink=None, **kwarg):
    # extent = [-3, 4, -4, 3]

    if(y_axis_shrink!=None):
        temp_tuple = (ylim[0]*y_axis_shrink, ylim[1]*y_axis_shrink)
        ylim = temp_tuple
        for ind, el in enumerate(data_zoomed):
            temp_tuple = (el[0], el[1]*y_axis_shrink)
            data_zoomed[ind] = temp_tuple

    axins = zoomed_inset_axes(ax, **kwarg) # zoom = 
    # axins.plot(Z2, extent=extent, interpolation="nearest", origin="lower")
    for ind, el in enumerate(data_zoomed):
        if ind==1:
            if len(data_zoomed)>1:
                axins.plot(el[0],el[1], 'r-', alpha=0.7, linewidth=0.8) # '--'
            else:
                axins.plot(el[0],el[1], 'k-', alpha=0.7, lw=0.8)
        else:
            # axins.plot(*el, lw=2)
            axins.plot(el[0],el[1], lw=1)

    # sub region of the original image
    # x1, x2, y1, y2 = 1.2*pi, 1.25*pi, -0.78, -0.54
    # axins.set_xlim(x1, x2)
    # axins.set_ylim(y1, y2)

    if ylim!=None:
        axins.set_ylim(ylim)

    plt.xticks(np.arange(  axins.viewLim.intervalx[0],
                        axins.viewLim.intervalx[1],
                        axins.viewLim.width/2), visible=False)
    plt.yticks(np.arange(  axins.viewLim.intervaly[0],
                        axins.viewLim.intervaly[1],
                        axins.viewLim.height/4), visible=False)

    if(y_axis_shrink==None):
        # draw a bbox of the region of the inset axes in the parent axes and
        # connecting lines between the bbox and the inset axes area
        if loc1!=None and loc2!=None:
            mark_inset(ax, axins, loc1=loc1, loc2=loc2, fc="none", ec="0.5")
        else:
            mark_inset(ax, axins, loc1=2, loc2=3, fc="none", ec="0.5")
        plt.draw()
    return axins
def plot_it(ax, time, ylabel, OrDi, bool_legend=False):
    cnt_temp = 0
    for k, v in OrDi.items():
        cnt_temp += 1
        if cnt_temp == 2:
            ax.plot(time, v, 'r-', alpha=0.7, lw=0.8)
        elif cnt_temp == 3:
            print('A thrid trace is plotted in blue')
            ax.plot(time, v, 'b-', alpha=0.7, lw=0.8)
        else:
            ax.plot(time, v, 'k-', alpha=0.7, lw=0.8) # label=k
    if bool_legend:
        # ax.legend(loc='lower right', shadow=True)
        ax.legend(bbox_to_anchor=(1.08,0.5), borderaxespad=0., loc='center', shadow=True)
    ax.set_ylabel(ylabel, fontdict=label_font)
    # ax.set_xlim(0,35) # shared x
    # ax.set_ylim(0.85,1.45)

######################
# CSP
@dataclass
class CJHStylePlot():
    path2info_dot_dat: str = None # this is path to a file
    nrows: int          = 1
    ncols: int          = 1
    sharex: bool        = True
    sharey: bool        = False
    figsize: tuple      = (8, 9)
    facecolor: str      = 'w'
    edgecolor: str      = 'k'

    def __post_init__(self):

        if self.ncols != 1:
            self.sharex = False
            self.sharey = True

        if self.path2info_dot_dat is not None:
            """ This is an ACMSimC Plot"""
            # this is a directory 
            self.dir_of_info_dot_dat = os.path.dirname(os.path.abspath(self.path2info_dot_dat)) + '/'
            # print('path2info_dot_dat:', self.path2info_dot_dat)
            print('dir_of_info_dot_dat:', self.dir_of_info_dot_dat)

        else:
            """ This is general dataframe based plot """
            self.fig, self.axes_v = plt.subplots(  
                                         nrows=     self.nrows,
                                         ncols=     self.ncols,
                                         sharex=    self.sharex,
                                         sharey=    self.sharey,
                                         figsize=   self.figsize,
                                         facecolor= self.facecolor,
                                         edgecolor= self.edgecolor,
                                       ); # modifying dpi would change the size of fonts, so don't do that
            # fig.subplots_adjust(wspace=0.01)
            if self.nrows == 1:
                self.axes_v = [self.axes_v]
            if self.ncols != 1:
                self.axes_v = np.ravel(self.axes_v)

    """ ACMSimC Plot """
    def load(self):
        self.df_list, self.time, self.fname = load_dataframe(self.dir_of_info_dot_dat)
        return self.df_list, self.time, self.fname
    def plot(self, list_of_ylabel, list_of_signalNames, 
             customize_plot=lambda axes_v:None,
             customize_plot_after=lambda axes_v:None,
             index=0, tb=0, te=None, bool_sumatrapdf=True):
        if self.df_list is None:
            self.load()

        df, time, fname = self.df_list[index], self.time, self.fname

        b, e = 0, -1
        if tb != 0:
            CL_TS = time[1] - time[0]
            b = int(tb / CL_TS)
            if te is not None:
                e = int(te / CL_TS)

        ''' 图片对象在这里当场生成，使得外部可以轻松在调用csp.plot之前，通过修改csp.figsize来修改图片的大小。
        '''
        fig, axes_v = self.fig, self.axes_v = plt.subplots(  
                                 ncols=     self.ncols,
                                 nrows=     self.nrows,
                                 sharex=    self.sharex,
                                 figsize=   self.figsize,
                                 facecolor= self.facecolor,
                                 edgecolor= self.edgecolor,
                               ); # modifying dpi would change the size of fonts
        # fig.subplots_adjust(wspace=0.01)

        customize_plot(axes_v)

        for i, ylabel, signalNames in zip(range(self.nrows), list_of_ylabel, list_of_signalNames):
            ax = axes_v[i]
            OrDi = OD()
            for ind, signalName in enumerate(signalNames):
                OrDi[str(ind)] = df[signalName][b:e]
            plot_it(ax, time[b:e]-tb, ylabel, OrDi)

        for ax in axes_v:
            ax.grid(True)
            # ax.axvspan(14, 29, facecolor='r', alpha=0.1)
            ax.set_yticklabels([f'{el:g}' for el in ax.get_yticks()], fontdict=label_font) # 为了修改ytick的字体  # https://matplotlib.org/3.3.3/api/_as_gen/matplotlib.axes.Axes.set_yticklabels.html

        # axes_v[-1].set_xlim((0,130))
        # axes_v[-1].set_xticks(np.arange(0, 201, 20))
        axes_v[-1].set_xlabel('Time, $t$ [s]', fontdict=label_font)
        axes_v[-1].set_xticklabels(axes_v[-1].get_xticks(), fontdict=tick_font)

        customize_plot_after(axes_v)

        # print(self.path2info_dot_dat)
        # print(fname[:-4])
        # print(fname)
        # fig.tight_layout() # tight layout is bad if you have too many subplots. Use bbox_inches instead
        path2outputFile = self.dir_of_info_dot_dat+fname[:-4] + f'-{index+1:03d}'
        fig.savefig(              f'{path2outputFile}.pdf', dpi=300, bbox_inches='tight', pad_inches=0)
        if bool_sumatrapdf:
            os.system('sumatrapdf ' + f'{path2outputFile}.pdf')
        os.system(f'pdfcrop         {path2outputFile}.pdf \
                                    {path2outputFile}-crop.pdf')
        # plt.show()

    """ General Plot for Academic Papers """
    def aca_plot(self, dict_label_dataframe, # a dict of ylabel: df
                    x='Time', # df[x] is usually time
                    y='[V]',  # df[y] is usually signal
                    b=0, e=-1, # truncate data (plot only the part we are interested in)
                    begin=0, end=None, # shift in time (x-axis)
                    adjust_figure=lambda axes_v:None, # additioanl ax properties manipulation before plot
                    adjust_ylabel=lambda axes_v:None, # additioanl ax properties manipulation after plot
                ):

        """ Must adjust this before set_yticklabels or else the ticks could be wrong """
        adjust_figure(self.axes_v)
        for ylabel, ax in zip(dict_label_dataframe.keys(), self.axes_v):
            for df, style in zip(dict_label_dataframe[ylabel], ['k-', 'r--', 'b--', 'g--']):
                alpha = 1.0
                if len(dict_label_dataframe[ylabel] ) == 2:
                    # If there are two traces, the black one is dashed and red is solid.
                    if style[0] == 'k':
                        style = 'k--'
                    if style[0] == 'r':
                        style = 'r-'
                        alpha = 0.7
                ax.plot(df[x][b:e]-df[x].values[0]-begin, df[y][b:e], style, alpha=alpha, lw=0.8)
            ax.set_ylabel(ylabel, fontdict=label_font, multialignment='center')
            # print('\t', ylabel)
            ax.set_yticklabels([f'{el:g}' for el in ax.get_yticks()], fontdict=tick_font)

        for ax in self.axes_v:
            ax.grid(True)
            # ax.axvspan(14, 29, facecolor='r', alpha=0.1)
            # ax.set_yticklabels([f'{el:g}' for el in ax.get_yticks()], font) # 为了修改ytick的字体  # https://matplotlib.org/3.3.3/api/_as_gen/matplotlib.axes.Axes.set_yticklabels.html

            if self.ncols != 1:
                ax.set_xlabel('Time, $t$ [s]', fontdict=label_font)
                ax.set_xticklabels([f'{el:g}' for el in self.axes_v[-1].get_xticks()], fontdict=tick_font)

        if end is not None:
            print('\tTime range:', [0, end-begin])
            self.axes_v[-1].set_xlim([0, end-begin])

        if self.ncols == 1:
            self.axes_v[-1].set_xlabel('Time, $t$ [s]', fontdict=label_font)
            self.axes_v[-1].set_xticklabels([f'{el:g}' for el in self.axes_v[-1].get_xticks()], fontdict=tick_font)

        ''' Move this to the end '''
        adjust_ylabel(self.axes_v)

        return self.axes_v
    def aca_save(self, path2outputFile, bool_sumatrapdf=True, fig=None):
        if fig is None:
            fig = self.fig
        fig.savefig(              f'{path2outputFile}.pdf', dpi=400, bbox_inches='tight', pad_inches=0)
        os.system(f'pdfcrop         "{path2outputFile}.pdf" \
                                    "{path2outputFile}-crop.pdf"')
        if bool_sumatrapdf:
            os.system('sumatrapdf ' + f'"{path2outputFile}-crop.pdf"')

if __name__ == '__main__':
    # import emachinery
    # CJHStylePlot = emachinery.acmsimcv5.CJHAcademicPlot.__cjhAcademicPlotSettings.CJHStylePlot
    csp = CJHStylePlot( path2info_dot_dat=r'D:\DrH\Codes\emachineryTestPYPI\emachinery\acmsimcv5\dat\IM_Marino05_PI-Ohtani/info.dat', 
                        nrows=6)

    list_of_ylabel = [
      'Speed\n[rpm]',
      '$\\alpha$-axis Flux\n[Wb]',
      'Torque Current\n[A]',
      'Offset Voltage\n[V]',
      'Magn. Current\n[A]',
      'Load Torque\n[Nm]',
    ]

    list_of_signalNames = [
    ######################################
    ("ACM.rpm_cmd",
    "ACM.rpm",
    "marino.xOmg*ELEC_RAD_PER_SEC_2_RPM",
    ),
    ######################################
    ("ACM.psi_Amu",
    "ohtani.psi_2_real_output[0]",
    ),
    ######################################
    ("(*CTRL).I->iDQ_cmd[1]",
    # "ACM.iTs",
    "(*CTRL).I->iDQ[1]",
    ) ,
    ######################################
    ("ohtani.correction_integral_term[0]",
    "ohtani.correction_integral_term[1]",
    ) ,
    ######################################
    ("(*CTRL).I->iDQ_cmd[0]",
    # "ACM.iMs",
    "(*CTRL).I->iDQ[0]",
    ) ,
    ######################################
    ("ACM.TLoad",
    "marino.xTL",
    ),
    ]

    def customize_plot(axes_v):
        axes_v[0].set_ylim([-300, 300])
    csp.plot(list_of_ylabel, list_of_signalNames, customize_plot=customize_plot)

if __name__ == '__main__':
    # %%
    # import sys
    # sys.path.insert(1, r'D:\DrH\Codes\emachineryTestPYPI\emachinery\acmsimcv5\CJHAcademicPlot')
    from __cjhAcademicPlotSettings import CJHStylePlot, pd, np, plt, os

    ''' Current Function
    '''
    I_AMPL = 1.0
    PERIOD = 0.02 # s
    CL_TS = 1e-4
    time = np.arange(0, PERIOD, CL_TS)
    phase_angle = np.arange(-np.pi, np.pi, 2*np.pi / len(time))

    """ Compensation Voltage Function By Current Value """
    def sigmoid(x, a1=0, a2=2, a3=10):
        return a1 * x + a2 / (1.0 + np.exp(-a3 * x)) - a2*0.5

    def main():
        dict_label_dataframe = dict()

        for key, I_AMPL in zip(['1', '2'], [1, 0.2]):
            phase_current = np.sin(phase_angle) * I_AMPL
            ui_curve = [sigmoid(current_value) for current_value in phase_current]
            dict_label_dataframe.update(
                {
                    key: [
                        pd.DataFrame({
                            'Time': time,
                            'Signal': phase_current
                        }), 
                        pd.DataFrame({
                            'Time': time,
                            'Signal': ui_curve
                        }), 
                    ] # 如果不是一个list，则CSP会报错：TypeError: string indices must be integers
                }
            )

        def adjust_ylabel(axes_v):
            for ax in axes_v:
                ax.set_ylabel('[A] or [V]')

        csp = CJHStylePlot( nrows=2, figsize=(4, 3) )
        csp.aca_plot(dict_label_dataframe, x='Time', y='Signal', 
                        adjust_ylabel=adjust_ylabel)
        csp.axes_v[0].set_ylim([-1.1,1.1])
        csp.aca_save(f'{os.path.dirname(__file__)}/compensation_votlage_shape', bool_sumatrapdf=True)

    main()