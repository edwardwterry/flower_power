import numpy as np
import math
from .trajectory import PrecomputedTrajectory
from queue import Queue
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

from collections import deque

class MovingWindowFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
        self.total = 0.0
    
    def add(self, value):
        if len(self.buffer) == self.window_size:
            self.total -= self.buffer[0]
        self.buffer.append(value)
        self.total += value
    
    def get_average(self):
        if not self.buffer:
            return 0.0  # or raise an exception if preferred
        return self.total / len(self.buffer)


class Vehicle:
    def __init__(self, dt):
        self.X = Pose()

        self.dt = dt

    def step(self):
        # do physics here
        pass

    def get_state(self):
        return self.X
    
class OnlineVehicle(Vehicle):
    def __init__(self):
        self.xd = Pose()
        self.distances = {'left': Queue(), 'right': Queue()}

        self.F = {'left': 0.0, 'right': 0.0}

        # Physical parameters
        self.m = 1.0
        self.I = 1.0
        self.Cd = 1.0
        
    def update(self, left, right):
        # if either measurement is spurious, ignore it
        self.distances['left'].put(left)
        self.distances['right'].put(right)

    def filter_outliers(self):
        pass

    def step(self):
        pass

class PrecomputedVehicle(Vehicle):
    def __init__(self, dt, trajectory_params_yaml='/home/ed/Projects/flower_power/flower_power/config.yaml'):
        super().__init__(dt)
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