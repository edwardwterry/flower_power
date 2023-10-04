import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Range
from rclpy.qos import qos_profile_sensor_data

from .vehicle import OnlineVehicle, PrecomputedVehicle
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class Simulator(Node):
    def __init__(self, dt=0.02):
        super().__init__('simulator')
        self.vehicles = {'ego': OnlineVehicle(flip_left_right=False,
                                              flip_forward_aft=True)}
        self.timer = self.create_timer(dt, self.tick_callback)
        tss = TimeSynchronizer([Subscriber(self, Range, 'range_left', qos_profile=qos_profile_sensor_data),
                                Subscriber(self, Range, 'range_right', qos_profile=qos_profile_sensor_data)],
                                10)
        tss.registerCallback(self.sensor_callback)
        self.pubs = {'target': self.create_publisher(PoseStamped, 'target/pose', 10),
                     'ego': self.create_publisher(PoseStamped, 'ego/pose', 10),
                     'force_left': self.create_publisher(Float32, 'force_left', 10),
                     'force_right': self.create_publisher(Float32, 'force_right', 10)}

    def tick_callback(self):
        # self.vehicles['target'].step()
        for name, vehicle in self.vehicles.items():
            pose_msg = vehicle.get_state()
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.pose = pose_msg
            pose_stamped_msg.header.frame_id = name
            self.pubs[name].publish(pose_stamped_msg)
        
        paddle_forces = self.vehicles['ego'].get_paddle_forces()
        force_left_msg = Float32()
        force_right_msg = Float32()
        force_left_msg.data = paddle_forces['left']
        force_right_msg.data = paddle_forces['right']
        self.pubs['force_left'].publish(force_left_msg)
        self.pubs['force_right'].publish(force_right_msg)
    
    def sensor_callback(self, left_msg, right_msg):
        self.vehicles['ego'].update(left_msg, right_msg)
        self.vehicles['ego'].step()

def main():
    rclpy.init()
    sim = Simulator()
    rclpy.spin(sim)
    

if __name__ == '__main__':
    main()
