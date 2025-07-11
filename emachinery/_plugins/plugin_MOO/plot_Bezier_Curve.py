import matplotlib.pyplot as plt
from pylab import np, plt, mpl
import streamlit as st
import streamlit as st
import BzierValidation as bv
import scienceplots

plt.style.use(['science','ieee'])

def get_Bzier_curve(points):
    BzierController = bv.BezierController(points, 0)
    Bzier = []
    Gain = []
    t_values = np.linspace(0, 1, 100)
    x_values = [BzierController.bezier_x(t) for t in t_values]
    y_values = [BzierController.bezier_y(t) for t in t_values]
    Bzier.append([x_values, y_values])
    epsilon = 1e-7
    Gain.append([x_values, np.array(y_values)/(np.array(x_values) + epsilon)])
    return Bzier, Gain

# define your bezier points
bezier_points = [
    [0.0, 0.0],
    [50, 5.4],
    [100, 5.5],
    [110, 5.9],
    [500.0, 6.0]
]

Bzier, Gain = get_Bzier_curve(bezier_points)

fig, ax = plt.subplots(2, 1, figsize=(12, 6))
ax[0].plot(Bzier[0][0], Bzier[0][1])
ax[0].scatter([p[0] for p in bezier_points], [p[1] for p in bezier_points], color='red')
ax[1].plot(Gain[0][0], Gain[0][1])

plt.savefig('bezier_curve.png')
plt.close(fig)