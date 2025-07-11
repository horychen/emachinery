import os, sys
from output_postProcessing import cplot, pyplot
import _plugins.plugin_MOO.optimize as moo

if __name__ == '__main__':
    print(f'{sys.argv=}')
    if len(sys.argv) > 1:
        if sys.argv[1] == "pyplot":
            pyplot.matplot_example_plot()

        elif sys.argv[1] == "cplot":
            from pylab import plt
            # import numpy as np
            # import matplotlib.pyplot as plt 
            if len(sys.argv) == 3:
                cplot.main(sys.argv[2], None, post_run=True)
            else:
                cplot.main(sys.argv[2], None, sys.argv[3], post_run=True)
            plt.show()

        elif sys.argv[1] == "moo":
            moo.main()
    else:
        os.chdir(os.path.dirname(__file__))
        os.system('streamlit run st_main.py')

# Path: emachinery/visualize.py
# shell command: streamlit run visualize.py
