from pylab import plt, mpl
import pickle, os, yaml

def matplot_example_plot():
    with open(os.path.dirname(__file__)+"/../data/python/global_machine_times.pkl", 'rb') as f:
        global_machine_times = pickle.load(f)
    with open(os.path.dirname(__file__)+"/../data/python/gdd.pkl", 'rb') as f:
        gdd = pickle.load(f)
    fig = example_plot(gdd, global_machine_times, False)
    plt.show()

def example_plot(gdd, global_machine_times, save=True):
    # https://matplotlib.org/stable/gallery/style_sheets/style_sheets_reference.html
    plt.style.use('bmh')
    mpl.rc('font', family='Times New Roman', size=10.0)
    # mpl.rc('legend', fontsize=10)
    # mpl.rc('lines', linewidth=4, linestyle='-.')
    mpl.rcParams['lines.linewidth'] = 0.75
    mpl.rcParams['mathtext.fontset'] = 'stix'

    with open(os.path.dirname(__file__)+'/../user_plot_config.yaml', encoding='utf-8') as f:
        user_plot_config = yaml.load(f, Loader=yaml.FullLoader)

    fig, axes = plt.subplots(nrows=len(user_plot_config['pyplot']['subplot']),
                            ncols=1, dpi=300, facecolor='w',
                            figsize=(
                                user_plot_config['pyplot']['width'],
                                user_plot_config['pyplot']['height'] *
                                len(user_plot_config['pyplot']['subplot'])),
                            sharex=True
                            )

    for subplot_config in user_plot_config['pyplot']['subplot']:
        ax = axes[user_plot_config['pyplot']['subplot'].index(subplot_config)]
        for subplot_signal_index in range(len(subplot_config['y'])):
            signal = gdd[subplot_config['y'][subplot_signal_index]['y_data']]
            label = subplot_config['y'][subplot_signal_index]['y_label']
            ax.plot(global_machine_times, signal, label=label)
        ax.set_ylabel(subplot_config['y_title'])
        ax.legend(loc=1)
        ax.grid(True)
    ax.set_xlabel(user_plot_config['pyplot']['x_label'])
    return fig