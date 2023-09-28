import numpy as np
import math
from .trajectory import PrecomputedTrajectory
from queue import Queue
from geometry_msgs.msg import Pose, Accel
from sensor_msgs.msg import Range
from scipy.spatial.transform import Rotation as R

from collections import deque

def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

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

class RingBuffer:
    def __init__(self, size=4):
        self.size = size
        self.buffer = [None] * size
        self.idx = 0
        self.full = False

    def add(self, element):
        self.buffer[self.idx] = element
        if self.idx + 1 >= self.size:
            self.full = True
        self.idx = (self.idx + 1) % self.size

    def get_all_elements(self):
        if not self.full:
            return self.buffer[:self.idx]
        else:
            return self.buffer[self.idx:] + self.buffer[:self.idx]
    
    def average_pair_gradient(self):
        gradient = 0.0
        if not self.full:
            return gradient
        elms = self.get_all_elements()
        for i in range(1, len(elms)):
            t_prev = stamp_to_sec(elms[i-1].header.stamp)
            t_curr = stamp_to_sec(elms[i].header.stamp)
            dt = t_curr - t_prev

            d_prev = elms[i-1].range
            d_curr = elms[i].range
            gradient += (d_curr - d_prev) / dt
        return gradient

class Vehicle:
    def __init__(self):
        self.X = Pose()
        self.Xd = Accel()

    def step(self):
        # do physics here
        pass

    def get_state(self):
        return self.X
    
    def linear_velocity_abs(self):
        return np.linalg.norm(self.Xd.linear.x ** 2 + self.Xd.linear.y ** 2)

class OnlineVehicle(Vehicle):
    def __init__(self):
        super().__init__()
        self.xd = Pose()
        self.measurements_filtered = {'left': RingBuffer(), 'right': RingBuffer()}
        self.filter = {'left': RealTimeOutlierFilter(),
                       'right': RealTimeOutlierFilter()}

        self.F = {'left': 0.0, 'right': 0.0}

        # Physical parameters
        self.m = 1.0
        self.I = 1.0
        self.Cd_linear = 10.0
        self.Cd_angular = 1.0
        self.dy = 0.5
        
    def update(self, left, right):
        # if either measurement is spurious, ignore it
        self.filter['left'].add_data_point(left.range)
        self.filter['right'].add_data_point(right.range)
        if not self.filter['left'].is_outlier(left.range):
            # print('putting left', left)
            self.measurements_filtered['left'].add(left)
        if not self.filter['right'].is_outlier(right.range):
            # print('putting right', right)
            self.measurements_filtered['right'].add(right)

    def step(self):
        F_left = self.measurements_filtered['left'].average_pair_gradient()
        F_right = -F_left#self.measurements_filtered['right'].average_pair_gradient()

        measurements_left = self.measurements_filtered['left'].get_all_elements()
        if len(measurements_left) < 2: 
            # not enough to calculate dt
            return
        t_prev = stamp_to_sec(measurements_left[-2].header.stamp)
        t_curr = stamp_to_sec(measurements_left[-1].header.stamp)
        dt = t_curr - t_prev

        D_linear = self.linear_velocity_abs() * self.Cd_linear * -np.sign(self.Xd.linear.x)
        a = (F_left + F_right + D_linear) / self.m
        self.Xd.linear.x += a * dt
        self.X.position.x += self.Xd.linear.x * dt
        # print(f'{a:.2f} {self.Xd.linear.x:.2f} {self.X.position.x:.2f}')

        D_angular = abs(self.Xd.angular.z) * self.Cd_angular * -np.sign(self.Xd.angular.z)
        alpha = (-F_left * self.dy + F_right * self.dy + D_angular) / self.I
        self.Xd.angular.z += alpha * dt
        prev = R.from_quat([self.X.orientation.x,
                          self.X.orientation.y,
                          self.X.orientation.z,
                          self.X.orientation.w])
        delta = R.from_euler('z', self.Xd.angular.z * dt, degrees=False)
        curr = prev * delta
        curr_quat = curr.as_quat()
        self.X.orientation.x = curr_quat[0]
        self.X.orientation.y = curr_quat[1]
        self.X.orientation.z = curr_quat[2]
        self.X.orientation.w = curr_quat[3]
        # print(f'{curr.as_euler("zxy")[0]}')
        # print(f'{D_angular:.2f} {F_left:.2f} {alpha:.2f} {self.Xd.angular.z:.2f} {self.X.position.x:.2f}')

        # TODO heading integration


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

