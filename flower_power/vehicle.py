import numpy as np
import math
from .trajectory import PrecomputedTrajectory
from queue import Queue
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
from scipy.spatial.transform import Rotation as R

from collections import deque

class RealTimeOutlierFilter:
    def __init__(self, window_size=10, threshold=2.0):
        self.window_size = window_size
        self.threshold = threshold
        self.window = []
    
    def add_data_point(self, data_point):
        if len(self.window) >= self.window_size:
            self.window.pop(0)
        self.window.append(data_point)
        
    def is_outlier(self, data_point):
        if len(self.window) < self.window_size:
            return False  # Not enough data to determine
        
        avg = sum(self.window) / len(self.window)
        std_dev = (sum([(dp - avg) ** 2 for dp in self.window]) / len(self.window)) ** 0.5
        
        z_score = (data_point - avg) / std_dev if std_dev != 0 else 0
        return abs(z_score) > self.threshold


# filter = RealTimeOutlierFilter(window_size=10, threshold=2.0)

# # Example Usage
# data_points = [10, 12, 14, 16, 10, 13, 10, 200, 12, 14, 12, 14, 13, 10, 12]

# for data_point in data_points:
#     filter.add_data_point(data_point)
#     if filter.is_outlier(data_point):
#         print(f"{data_point} is an outlier")



class Vehicle:
    def __init__(self):
        self.X = Pose()

    def step(self):
        # do physics here
        pass

    def get_state(self):
        return self.X
    
class OnlineVehicle(Vehicle):
    def __init__(self):
        super().__init__()
        self.xd = Pose()
        self.measurements_filtered = {'left': Queue(maxsize=100), 'right': Queue(maxsize=100)}
        self.filter = {'left': RealTimeOutlierFilter(),
                       'right': RealTimeOutlierFilter()}

        self.F = {'left': 0.0, 'right': 0.0}

        # Physical parameters
        self.m = 1.0
        self.I = 1.0
        self.Cd = 1.0
        
    def update(self, left, right):
        # if either measurement is spurious, ignore it
        self.filter['left'].add_data_point(left.range)
        self.filter['right'].add_data_point(right.range)
        if not self.filter['left'].is_outlier(left.range):
            print('putting left', left.range)
            self.measurements_filtered['left'].put(left)
        if not self.filter['right'].is_outlier(right.range):
            print('putting left', right.range)
            self.measurements_filtered['right'].put(right)


    def step(self):
        pass

class PrecomputedVehicle(Vehicle):
    def __init__(self, trajectory_params_yaml='/home/ed/Projects/flower_power/flower_power/config.yaml'):
        super().__init__()
        self.trajectory = PrecomputedTrajectory(trajectory_params_yaml).get_trajectory_points(periodic=True)[:-1]
        self.index = 0
        self.first = True

    def step(self):
        if self.first:
            yaw = 0.0
            self.first = False
        else:
            prev = self.trajectory[self.index - 1]
            curr = self.trajectory[self.index]
            yaw = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
        self.X.position.x = self.trajectory[self.index][0]
        self.X.position.y = self.trajectory[self.index][1]
        q = R.from_rotvec(yaw * np.array([0, 0, 1])).as_quat()
        self.X.orientation.x = q[0]
        self.X.orientation.y = q[1]
        self.X.orientation.z = q[2]
        self.X.orientation.w = q[3]

        self.index = (self.index + 1) % len(self.trajectory)

#   var dt = t_curr - t_prev;

#   if (dists_prev[0] < dist_max && dists_curr[0] < dist_max &&
#       Math.abs(dists_curr[0] - dists_prev[0]) < d_dist_dt_max) {
#     F[0] = (dists_curr[0] - dists_prev[0]) / dt;
#   } else {
#     F[0] = 0;
#   }

#   console.log("");
#   console.log("### NEW ###");
#   console.log(dists_curr[0], dists_prev[0]);
#   dists_prev[0] = dists_curr[0];

#   // Pre-calc heading vector
#   var heading = theta.z;
#   var sn = Math.sin(heading);
#   var cs = Math.cos(heading);

#   // linear
#   var paddle_force = F[0];
#   var drag = vel.scaled(Cd).scaled(-1).scaled(vel.norm());
#   var acc = new Vec3(paddle_force * cs, paddle_force * sn, 0).add(drag).scaled(1/mass);
#   vel = acc.scaled(dt).plus(vel);
#   pos = vel.scaled(dt).plus(pos);