import numpy as np
import math
from .trajectory import PrecomputedTrajectory
from queue import Queue
from geometry_msgs.msg import Pose, Accel, Twist
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
        self.Xd = Twist()

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
        self.m = 5.0
        self.I = 2.0
        self.Cd_linear_long = 1.0
        self.Cd_linear_lat = 2.0
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
        print()
        F_left = self.measurements_filtered['left'].average_pair_gradient()
        F_right = self.measurements_filtered['right'].average_pair_gradient()
        if abs(F_left) < 0.1:
            F_left = 0.0
        if abs(F_right) < 0.1:
            F_right = 0.0
        print(f'F left: {F_left:.2f}')
        print(f'F right: {F_right:.2f}')

        prev = R.from_quat([self.X.orientation.x,
                            self.X.orientation.y,
                            self.X.orientation.z,
                            self.X.orientation.w])

        heading = prev.as_euler('zyx', degrees=False)[0]

        rot_matrix = np.array([[np.cos(heading), np.sin(heading)],
                               [-np.sin(heading), np.cos(heading)]])

        vel_body = np.array([self.Xd.linear.x, self.Xd.linear.y]) @ np.linalg.inv(rot_matrix)
        print('vel body', vel_body)
        # D_linear_body = np.array([self.linear_velocity_abs() * self.Cd_linear * -np.sign(vel_body[0]), 0.0])
        D_linear_body = -vel_body * np.array([self.Cd_linear_long, self.Cd_linear_lat])
        # D_linear_body = np.array([self.linear_velocity_abs() * self.Cd_linear * -np.sign(vel_body[0]), 0.0])
        F_linear_body = np.array([F_left + F_right, 0.0])
        D_linear_world = D_linear_body @ rot_matrix
        F_linear_world = F_linear_body @ rot_matrix
        a_linear_world = (F_linear_world + D_linear_world) / self.m
        print('D linear body', D_linear_body)
        print('F linear body', F_linear_body)
        print('D linear world', D_linear_world)
        print('F linear world', F_linear_world)
        print('a linear world', a_linear_world)
        self.Xd.linear.x += a_linear_world[0] * self.dt
        self.Xd.linear.y += a_linear_world[1] * self.dt
        self.X.position.x += self.Xd.linear.x * self.dt
        self.X.position.y += self.Xd.linear.y * self.dt
        # print(f'D linear x: {D_linear.linear.x:.2f}')
        # print(f'D linear y: {D_linear.linear.y:.2f}')
        # print(f'accel x: {a.linear.x:.2f}')
        # print(f'accel y: {a.linear.y:.2f}')
        # print(f'a * self.dt * np.cos(heading): {a.linear.x * self.dt:.2f}')
        # print(f'a * self.dt * np.sin(heading): {a.linear.y * self.dt:.2f}')
        print(f'vel lin x: {self.Xd.linear.x:.2f}')
        print(f'vel lin y: {self.Xd.linear.y:.2f}')
        print(f'pos x: {self.X.position.x:.2f}')
        print(f'pos y: {self.X.position.y:.2f}')
        print(f'heading: {heading:.2f}')

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
        print(f'D angular: {D_angular:.2f}')
        print(f'vel rot z: {self.Xd.angular.z:.2f}')
        print(f'alpha: {alpha:.2f}')


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
