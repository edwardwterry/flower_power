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

class PaddleInsertedDetector:
    def __init__(self, num_timesteps_thr=3, window_size=10, max_thr=0.2):
        self.window_size = window_size
        self.num_timesteps_thr = num_timesteps_thr
        self.queue = deque(maxlen=window_size)
        self.max_thr = max_thr
    
    def add_measurement(self, measurement):
        if len(self.queue) > self.window_size:
            self.queue.popleft()
        self.queue.append(measurement)
    
    def is_inserted(self):
        if len(self.queue) < self.num_timesteps_thr:
            return False
        last = list(self.queue)[-self.num_timesteps_thr:]
        return all(elm < self.max_thr for elm in last)


class RingBuffer:
    def __init__(self, size=20):
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

    def clear(self):
        self.__init__()


class Vehicle:
    def __init__(self):
        self.X = Pose()
        self.Xd = Accel()

    def get_state(self):
        return self.X

    def linear_velocity_abs(self):
        return np.linalg.norm(self.Xd.linear.x ** 2 + self.Xd.linear.y ** 2)


class OnlineVehicle(Vehicle):
    def __init__(self):
        super().__init__()
        self.measurements_filtered = {
            'left': RingBuffer(), 'right': RingBuffer()}

        self.inserted = {'left': False, 'right': False}
        self.inserted_filters = {'left': PaddleInsertedDetector(),
                                 'right': PaddleInsertedDetector()}

        # Physical parameters
        self.m = 0.5
        self.I = 2.0
        self.Cd_linear = 0.6
        self.Cd_angular = 1.0

        self.latest_dt = 0.0
        self.t_prev = None
        self.t_curr = None
        self.dt = None

    def update(self, left, right):
        self.inserted_filters['left'].add_measurement(left.range)
        self.inserted_filters['right'].add_measurement(right.range)

        self.inserted['left'] = self.inserted_filters['left'].is_inserted()
        self.inserted['right'] = self.inserted_filters['right'].is_inserted()

        self.t_prev = self.t_curr
        self.t_curr = stamp_to_sec(left.header.stamp)
        # Timestamps are synchronized by design
        if self.t_prev is None:
            return
        self.dt = self.t_curr - self.t_prev

        if not self.inserted['left']:
            self.measurements_filtered['left'].clear()
        else:
            self.measurements_filtered['left'].add(left)

        if not self.inserted['right']:
            self.measurements_filtered['right'].clear()
        else:
            self.measurements_filtered['right'].add(right)

    def step(self):
        if self.dt is None:
            # Haven't yet got 2 measurements
            return

        F_left = self.measurements_filtered['left'].average_pair_gradient()
        F_right = -F_left#self.measurements_filtered['right'].average_pair_gradient()

        prev = R.from_quat([self.X.orientation.x,
                            self.X.orientation.y,
                            self.X.orientation.z,
                            self.X.orientation.w])

        heading = prev.as_euler('zyx', degrees=False)[0]

        D_linear = self.linear_velocity_abs() * self.Cd_linear * - \
            np.sign(self.Xd.linear.x)
        a = (F_left + F_right + D_linear) / self.m
        self.Xd.linear.x += a * self.dt * np.cos(heading)
        self.Xd.linear.y += a * self.dt * np.sin(heading)
        self.X.position.x += self.Xd.linear.x * self.dt
        self.X.position.y += self.Xd.linear.y * self.dt
        print(f'{a:.2f} {self.Xd.linear.x:.2f} {self.X.position.x:.2f}')

        D_angular = abs(self.Xd.angular.z) * self.Cd_angular * -np.sign(self.Xd.angular.z)
        alpha = (-F_left + F_right + D_angular) / self.I
        self.Xd.angular.z += alpha * self.dt

        delta = R.from_euler('z', self.Xd.angular.z * self.dt, degrees=False)
        curr = prev * delta
        curr_quat = curr.as_quat()
        self.X.orientation.x = curr_quat[0]
        self.X.orientation.y = curr_quat[1]
        self.X.orientation.z = curr_quat[2]
        self.X.orientation.w = curr_quat[3]
        print(f'{curr.as_euler("zxy")[0]}')
        print(f'{D_angular:.2f} {F_left:.2f} {alpha:.2f} {self.Xd.angular.z:.2f} {self.X.position.x:.2f}')


class PrecomputedVehicle(Vehicle):
    def __init__(self, trajectory_params_yaml='/home/ed/Projects/flower_power/flower_power/config.yaml'):
        super().__init__()
        self.trajectory = PrecomputedTrajectory(
            trajectory_params_yaml).get_trajectory_points(periodic=True)[:-1]
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
