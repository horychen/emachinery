# This script is obsolete

from pylab import plt, mpl, np
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from collections import OrderedDict as O
mpl.rcParams['legend.fontsize'] = 15
font = {'family' : 'Times New Roman', #'serif',
        'weight' : 'normal',
        'size'   : 15}
mpl.rcParams['font.family'] = ['serif'] # default is sans-serif
mpl.rcParams['font.serif'] = ['Times New Roman']

# plt.style.use('ggplot') 
# plt.style.use('grayscale') # print plt.style.available # get [u'dark_background', u'bmh', u'grayscale', u'ggplot', u'fivethirtyeight']
mpl.style.use('classic')

######################
# Read in Data
import csv
f_name = './noo0025.mat.dat' 
bias = 125
b = bias * 1000
e = (bias+70) * 1000 
ll = [  [],[],[],[],[],[],[],[],[],[],
        [],[],[],[],[],[],[],[],[],[],
        [],[],[],[],[],[],[],[],[],[] ]
print('Read in data...')
with open(f_name, mode='r') as f:
    buf = f.readlines()
    reader = csv.reader(buf)
    for row in reader:
        for ind, el in enumerate(row):
            ll[ind].append(float(el))
for ind, l in enumerate(ll):
    if l==[]:
        ll_len = ind
        break
    else:
        ll[ind] = ll[ind][b:e]
print('Quantities amount:', ll_len)
print('Data Length:', ll[0].__len__())

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
                axins.plot(el[0],el[1], '--', linewidth=1) # '--'
            else:
                axins.plot(el[0],el[1], lw=1)
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

def get_axis(cNr):
    # fig, axes = plt.subplots(ncols=cNr[0], nrows=cNr[1], dpi=150, sharex=True);
    fig, axes = plt.subplots(ncols=cNr[0], nrows=cNr[1], sharex=True, figsize=(8,8), facecolor='w', edgecolor='k');
    fig.subplots_adjust(right=0.95, bottom=0.1, top=0.95, hspace=0.2, wspace=0.02)    
    # fig.subplots_adjust(right=0.85, bottom=0.1, top=0.95, hspace=0.25)
    if sum(cNr)<=2:
        return axes
    else:
        return axes.ravel()

def plot_it(ax, ylabel, d, bool_legend=False):
    cnt_temp = 0
    for k, v in d.items():
        cnt_temp += 1
        if cnt_temp == 2:
            ax.plot(time, v, 'r-', alpha=0.7, lw=0.8)
        else:
            ax.plot(time, v, 'k-', alpha=0.7, lw=0.8) # label=k
    if bool_legend:
        # ax.legend(loc='lower right', shadow=True)
        ax.legend(bbox_to_anchor=(1.08,0.5), borderaxespad=0., loc='center', shadow=True)
    ax.set_ylabel(ylabel, fontdict=font)
    # ax.set_xlim(0,35) # shared x
    # ax.set_ylim(0.85,1.45)

time = np.arange(1,ll[0].__len__()+1,1) * 1e-3  # sampling frequency is 10e3

fig, axes_v = plt.subplots(ncols=1, nrows=6, sharex=True, figsize=(8,9), facecolor='w', edgecolor='k'); # modifying dpi would change the size of fonts

ax = axes_v[0]
# ax.set_ylim((0,520))
plot_it(ax, 'Speed\n[rpm]', 
                            O([         
                                        (r'$\omega$',                        ll[0]),
                                        (r'$\hat\omega^{\rm NO}$',           ll[1]),
                                        (r'$\omega^*$',                      ll[6]) ]))
ax.annotate(r'$\omega^*$', xy=(12, 1000), xycoords='data',
                    xytext=(5, 900), size=16, # xytext=(0.15, 0.15), textcoords='figure fraction',
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.2"))
ax.annotate(r'$\omega$', xy=(15, 1000), xycoords='data',
                    xytext=(15, 700), size=16, # xytext=(0.15, 0.15), textcoords='figure fraction',
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.2"))
ax.annotate('$\\hat\\omega$', xy=(20, 1025), xycoords='data',
                    xytext=(25, 950), size=16,
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=+.2"))
ax.annotate(r'$\hat J_s=\frac{1}{2}J_s$', xy=(27, 500), xycoords='data',
                    xytext=(23, 800), size=16, # xytext=(0.15, 0.15), textcoords='figure fraction',
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.05"))
ax.annotate(r'$\hat J_s=\frac{3}{2}J_s$', xy=(47, 500), xycoords='data',
                    xytext=(43, 800), size=16, # xytext=(0.15, 0.15), textcoords='figure fraction',
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.05"))
if True:
    # ax = plt.axes([0.17, 0.92, .25, .05], axisbg='w') #前两个是位置，后两个是大小
    ax = plt.axes([0.17, 0.92, .25, .05]) #前两个是位置，后两个是大小
    bz = int(12*1000)
    ez = int(22.5*1000)
    ax.plot(time[bz:ez],ll[0][bz:ez], lw=0.8)
    ax.plot(time[bz:ez],ll[1][bz:ez], 'r-', lw=0.8)
    ax.plot(time[bz:ez],ll[6][bz:ez], 'k--', lw=0.8)
    ax.grid()
    # title('Zoomed-in Plot')
    ax.set_ylim([960,1060])
    plt.setp(ax, yticks=np.arange(960,1065,50))
    ax.get_yaxis().get_major_formatter().set_useOffset(False) # https://stackoverflow.com/questions/14711655/how-to-prevent-numbers-being-changed-to-exponential-form-in-python-matplotlib-fi
    ax.set_ylabel('Speed\n[rpm]')
    ax.set_xlim([(bz/1000.0),(ez/1000.0)])
    # ax.axvspan(int(bz/1000.), int(ez/1000.), facecolor='r', alpha=0.05)
if True:
    ax = plt.axes([0.60, 0.92, .25, .05]) #前两个是位置，后两个是大小
    # ax = plt.axes([0.60, 0.92, .25, .05], axisbg='w') #前两个是位置，后两个是大小
    bz = int(52*1000)
    ez = int(62.5*1000)
    ax.plot(time[bz:ez],ll[0][bz:ez], lw=0.8)
    ax.plot(time[bz:ez],ll[1][bz:ez], 'r-', lw=0.8)
    ax.plot(time[bz:ez],ll[6][bz:ez], 'k--', lw=0.8)
    ax.grid()
    # title('Zoomed-in Plot')
    ax.set_ylim([960,1060])
    plt.setp(ax, yticks=np.arange(960,1065,50))
    ax.get_yaxis().get_major_formatter().set_useOffset(False) # https://stackoverflow.com/questions/14711655/how-to-prevent-numbers-being-changed-to-exponential-form-in-python-matplotlib-fi
    ax.set_ylabel('Speed\n[rpm]')
    ax.set_xlim([(bz/1000.0),(ez/1000.0)])
    # ax.axvspan(int(bz/1000.), int(ez/1000.), facecolor='r', alpha=0.05)



ax = axes_v[1]
ax.set_ylim((-10,20))
# ax.set_yticks(np.arange(0, 15.5, 3))
plot_it(ax, '$T_L,\\hat T_L$\n[Nm]', 
                            O([     ('1',         ll[4]),
                                    ('2',         ll[5]) ]))
                                    # ('3',    np.array(ll[8])*np.array(ll[9])/0.52*1.4744) ])) # The load torque constant is 1.4744 Nm/A ]))
ax.annotate(r'$T_L$', xy=(12, -5), xycoords='data',
                    xytext=(5, -5), size=16, # xytext=(0.15, 0.15), textcoords='figure fraction',
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))
ax.annotate(r'$\hat T_L$', xy=(18.5, 5), xycoords='data',
                    xytext=(20, -8), size=16,
                    arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.2"))


ax = axes_v[2]
ax.set_ylim((0.8,2.0))
ax.set_yticks(np.arange(0.8, 2.05, 0.2))
plot_it(ax, '$\\hat r_{req}$\n[$\\Omega$]', 
                            O([         
                                        ('1',         np.array(ll[7]) )]))

ax = axes_v[3]
ax.set_ylim((-0.5,2))
# ax.set_yticks(np.arange(-2, 13, 3))
plot_it(ax, '$U_{\\rm offset}$\n[AWb]', 
                            O([         
                                        (r'$off$',           ll[2]) ]))


ax = axes_v[4]
ax.set_ylim((-100,110))
# ax.set_yticks(np.arange(0, 3200, 600))    
plot_it(ax, '$\\tilde\\omega$\n[rpm]', 
                            O([         (r'$\omega$',                        np.array(ll[0]) - np.array(ll[1]) ) ]))
# ax.annotate(r'$\omega$', xy=(2, 1500), xycoords='data',
#                     xytext=(1.5, 1800), size=16, # xytext=(0.15, 0.15), textcoords='figure fraction',
#                     arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.2"))
# ax.annotate(r'$\hat\omega$', xy=(2, 1400), xycoords='data',
#                     xytext=(2.5, 900), size=16,
#                     arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.2"))

ax = axes_v[5]
ax.set_ylim((-500,500))
ax.set_yticks(np.arange(-500, 505, 250))    
plot_it(ax, '$e_\\omega$\n[rpm]', 
                            O([         #(r'$\omega$',                            np.array(ll[6]) - np.array(ll[0]) ),
                                        (r'$\hat\omega$',                        np.array(ll[6]) - np.array(ll[1]) ) ]))
# ax.annotate(r'$\omega$', xy=(2, 1500), xycoords='data',
#                     xytext=(1.5, 1800), size=16, # xytext=(0.15, 0.15), textcoords='figure fraction',
#                     arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.2"))
# ax.annotate(r'$\hat\omega$', xy=(2, 1400), xycoords='data',
#                     xytext=(2.5, 900), size=16,
#                     arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=-.2"))


for ax in axes_v:
    ax.grid(True)
    # ax.axvspan(14, 29, facecolor='r', alpha=0.1)

# axes_v[-1].set_xlim((0,130))
# axes_v[-1].set_xticks(np.arange(0, 201, 20))
axes_v[-1].set_xlabel(r'Time [s]')

# savefig(r'C:\Dr.H\(0) GET WORKING\18 JESTPE01_Dib12Akatsu00\JESTPE01_TEX\picAuto/'+'fig05.png', dpi=300, bbox_inches = 'tight', pad_inches = 0)

plt.show()
