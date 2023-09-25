import numpy as np
import scipy.interpolate as si
from scipy.spatial.distance import pdist
import matplotlib.pyplot as plt

import yaml


# for d in range(1,5):
#     print (p2)
#     x,y = p2.T
#     plt.plot(x,y,'k-')

# plt.minorticks_on()
# plt.legend()
# plt.xlabel('x')
# plt.ylabel('y')
# plt.gca().set_aspect('equal', adjustable='box')
# plt.show()



class PrecomputedTrajectory:
    def __init__(self, config_yaml):
        with open(config_yaml, 'r') as f:
            data = yaml.safe_load(f)
            self.control_points = np.array(data['control_points'])
            speed = data['speed']

    def get_trajectory_points(self, periodic=True):
        points = self.scipy_bspline(self.control_points, n=50, degree=3, periodic=periodic)
        return points

    def get_trajectory_length(trajectory_points):
        distance = 0.0
        for prev, curr in zip(trajectory_points[:-1], trajectory_points[1:]):
            distance += pdist(prev, curr)
        return distance
    
    # https://stackoverflow.com/questions/34803197/fast-b-spline-algorithm-with-numpy-scipy
    def scipy_bspline(self, cv, n=100, degree=3, periodic=False):
        """ Calculate n samples on a bspline

            cv :      Array ov control vertices
            n  :      Number of samples to return
            degree:   Curve degree
            periodic: True - Curve is closed
        """
        cv = np.asarray(cv)
        count = cv.shape[0]

        # Closed curve
        if periodic:
            kv = np.arange(-degree,count+degree+1)
            factor, fraction = divmod(count+degree+1, count)
            cv = np.roll(np.concatenate((cv,) * factor + (cv[:fraction],)),-1,axis=0)
            degree = np.clip(degree,1,degree)

        # Opened curve
        else:
            degree = np.clip(degree,1,count-1)
            kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

        # Return samples
        max_param = count - (degree * (1-periodic))
        spl = si.BSpline(kv, cv, degree)
        return spl(np.linspace(0,max_param,n))
