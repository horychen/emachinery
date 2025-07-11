
from numba.experimental import jitclass
from numba import njit, int32, float64
from pylab import np, plt, mpl
import os
# import CubicEquationSolver
# import warnings
import scipy
import itertools
import sympy
SHOW_BEZIER = False
class BezierController:
    def __init__(self, points, terminal):
        self.points = points
        self.terminal = terminal
        if SHOW_BEZIER:
            self.bezier_show()
        if not self.sturm_therorem():
            raise Exception('The Bezier curve is not a convex curve, please check the points you input.')

    def sturm_therorem(self):
        x=sympy.Symbol('x')
        limits = [-0.0, 1.0]
        for i in range(len(self.points[0])):
            ans=0
            sturm_seq=sympy.sturm(self.bezier_derivative(x)[i])
            values_at_start = [polynomial.subs(x,limits[0]).evalf() for polynomial in sturm_seq]
            values_at_end = [polynomial.subs(x,limits[1]).evalf() for polynomial in sturm_seq]
            count_start = len(list(itertools.groupby(values_at_start, lambda values_at_start: values_at_start > 0)))
            count_end = len(list(itertools.groupby(values_at_end, lambda values_at_end: values_at_end > 0)))
            ans = count_start - count_end
            if ans != 0:
                return False
        return True
    
    def bezier(self, t):
        re = np.array([0.0, 0.0])
        for i in range(len(self.points)):
            re += np.array(self.points[i])*scipy.special.comb(len(self.points)-1, i) * \
                ((1-t)**(len(self.points)-1-i))*(t**i)
        return re
        # return (1 - t) ** 3 * points[0] + 3 * (1 - t) ** 2 * t * points[1] + 3 * (1 - t) * t ** 2 * points[
        #     2] + t ** 3 * points[3]

    def bezier_derivative(self, t):
        re = np.array([0.0, 0.0])
        for i in range(len(self.points)):
            re=re+np.array(self.points[i])*scipy.special.comb(len(self.points)-1, i) * \
                ((1-t)**(len(self.points)-1-i))*(t**(i-1))*i
            re=re+ np.array(self.points[i])*scipy.special.comb(len(self.points)-1, i)*(
                (1-t)**(len(self.points)-1-i-1))*(t**i)*(i-len(self.points)+1)
        return re

    def bezier_x(self, t):
        return self.bezier(t)[0]

    def bezier_y(self, t):
        return self.bezier(t)[1]

    def find_t_for_given_x(self, x):
        root = scipy.optimize.brentq(lambda t: self.bezier_x(t) - x, -0.000001, 1.0000001)
        # root = scipy.optimize.brenth(lambda t: self.bezier_x(t) - x, -0.000001, 1.0000001)
        # root=scipy.optimize.toms748(lambda t: self.bezier_x(t) - x, -0.000001, 1.0000001)
        return root.real

    def find_y_for_given_x(self, x):
        t = self.find_t_for_given_x(x)
        y = self.bezier_y(t)
        return y

    def set_reg(self, reg):
        self.reg = reg

    def set_points(self, points):
        self.points = points

    def bezier_show(self):
        plt.style.use('bmh')
        mpl.rc('font', family='Times New Roman', size=10.0)
        mpl.rc('legend', fontsize=10)
        mpl.rcParams['lines.linewidth'] = 0.75
        mpl.rcParams['mathtext.fontset'] = 'stix'
        fig, axes = plt.subplots(nrows=1, ncols=1, dpi=150,
                                 facecolor='w', sharex=True)
        t_values = np.linspace(0, 1, 100)
        x_values = [self.bezier_x(t) for t in t_values]
        y_values = [self.bezier_y(t) for t in t_values]
        plt.plot(x_values, y_values, label='output')
        plt.show()
        epsilon = 1e-7
        plt.plot(x_values, np.array(y_values)/(np.array(x_values) + epsilon), '--.', color='k', label='gain')
        plt.legend()
        plt.show()

    def control_output(self, reg):
        error = reg.setpoint - reg.measurement
        if np.abs(error) > self.points[-1][0]:
            error = np.sign(error) * self.points[-1][0]
        out=self.find_y_for_given_x(np.abs(error))
        # reg.Out = np.sign(np.sign(error)*out+reg.differentiator) * out
        # reg.Out=(error)/(np.abs(error)+1e-16)*out # sigmoid function

        GuanZhongTao = True
        if GuanZhongTao:
            reg.Out = (error/(error+1e-7)) * out
        else:
            reg.Out = np.sign(error) * out
        reg.prevError = error
        reg.prevMeasurement = reg.measurement
        if abs(reg.Out) > abs(reg.OutLimit):
            raise("Out of Limit")
        return reg.Out

BEZIER_POINTS = [(0, 0),(167.19914134825365, 44.6915324901932),(284.6302061302766, 79.7836670190831),(379.7836670190831, 143.94129962856175)]
if __name__ == '__main__':
    bezier_controller = BezierController(BEZIER_POINTS, BEZIER_POINTS[-1][1])

    for i in range(10):
        ttt = (i+1)* 0.1
        print("x :",bezier_controller.bezier_x(ttt), "y:",bezier_controller.bezier_y(ttt), "\n")
